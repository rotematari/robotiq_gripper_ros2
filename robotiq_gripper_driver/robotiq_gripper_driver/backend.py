"""Robotiq register protocol and the serial (Modbus RTU) and simulated backends.

Register map: Robotiq Hand-E / 2F-85 / 2F-140 manuals, section 4 (Control).
Commands are three holding registers from 1000 (0x03E8); status is three input
registers from 2000 (0x07D0). Each register packs two bytes, high byte first.
"""

from __future__ import annotations

import inspect
import threading
import time
from dataclasses import dataclass
from typing import Protocol

COMMAND_ADDRESS = 1000
STATUS_ADDRESS = 2000

# gOBJ values (valid only while gGTO == 1)
OBJECT_MOVING = 0
OBJECT_CONTACT_OPENING = 1
OBJECT_CONTACT_CLOSING = 2
OBJECT_AT_REQUEST = 3

# gSTA values
STATE_RESET = 0
STATE_ACTIVATING = 1
STATE_ACTIVATED = 3

FAULTS = {
    0x05: "action delayed: reactivation must complete first",
    0x07: "activation bit must be set before the action",
    0x08: "maximum operating temperature exceeded (>= 85 C); wait for cool-down below 80 C",
    0x09: "no communication for at least 1 s",
    0x0A: "under minimum operating voltage",
    0x0B: "automatic release in progress",
    0x0C: "internal fault; contact Robotiq support",
    0x0D: "activation fault; check that nothing blocks the fingers",
    0x0E: "overcurrent triggered",
    0x0F: "automatic release completed",
}


def fault_message(code: int) -> str:
    """Text for gFLT `code`; empty for 0."""
    if code == 0:
        return ""
    return FAULTS.get(code, f"unknown fault 0x{code:02X}")


@dataclass(frozen=True)
class RawStatus:
    """Decoded status registers (gripper manual names)."""

    gACT: int
    gGTO: int
    gSTA: int
    gOBJ: int
    gFLT: int
    kFLT: int
    gPR: int
    gPO: int
    gCU: int

    @property
    def activated(self) -> bool:
        return self.gACT == 1 and self.gSTA == STATE_ACTIVATED


def decode_status(registers: list[int]) -> RawStatus:
    """Decode the three status registers read from address 2000."""
    if len(registers) != 3:
        raise ValueError(f"expected 3 status registers, got {len(registers)}")
    gripper_status = registers[0] >> 8
    fault_status = registers[1] >> 8
    return RawStatus(
        gACT=gripper_status & 0x01,
        gGTO=(gripper_status >> 3) & 0x01,
        gSTA=(gripper_status >> 4) & 0x03,
        gOBJ=(gripper_status >> 6) & 0x03,
        gFLT=fault_status & 0x0F,
        kFLT=(fault_status >> 4) & 0x0F,
        gPR=registers[1] & 0xFF,
        gPO=registers[2] >> 8,
        gCU=registers[2] & 0xFF,
    )


def encode_command(
    *, activate: bool, go_to: bool, position: int = 0, speed: int = 0, force: int = 0
) -> list[int]:
    """Encode the three command registers written to address 1000.

    Automatic release (rATR, rARD) is never set by this driver.
    """
    for name, value in (("position", position), ("speed", speed), ("force", force)):
        if type(value) is not int or not 0 <= value <= 255:
            raise ValueError(f"{name} must be an integer from 0 to 255, got {value!r}")
    action_request = (0x01 if activate else 0) | (0x08 if go_to else 0)
    return [action_request << 8, position, (speed << 8) | force]


class Backend(Protocol):
    """Register-level access to one gripper. Implementations need not be thread-safe."""

    def read_status(self) -> RawStatus: ...

    def write_command(self, registers: list[int]) -> None: ...

    def close(self) -> None: ...


class BackendError(RuntimeError):
    """A serial transaction failed or returned a Modbus error."""


class ModbusBackend:
    """Modbus RTU over a USB-to-RS-485 adapter, using pymodbus (3.6 or newer)."""

    def __init__(self, port: str, device_id: int = 9, timeout: float = 0.2) -> None:
        if not port or port.strip().lower() == "auto":
            raise ValueError("port must be an explicit serial device path")
        if type(device_id) is not int or not 1 <= device_id <= 247:
            raise ValueError("device_id must be an integer from 1 to 247")
        from pymodbus.client import ModbusSerialClient

        # The Robotiq serial settings are fixed: 115200 bit/s, 8N1.
        self._client = ModbusSerialClient(
            port=port, baudrate=115200, bytesize=8, parity="N", stopbits=1, timeout=timeout
        )
        # pymodbus renamed the unit-id keyword from `slave` to `device_id` in 3.10.
        params = inspect.signature(self._client.read_input_registers).parameters
        key = "device_id" if "device_id" in params else "slave"
        self._unit = {key: device_id}
        if not self._client.connect():
            raise BackendError(f"could not open serial port {port}")

    def read_status(self) -> RawStatus:
        result = self._client.read_input_registers(STATUS_ADDRESS, count=3, **self._unit)
        if result.isError():
            raise BackendError(f"status read failed: {result}")
        return decode_status(list(result.registers))

    def write_command(self, registers: list[int]) -> None:
        result = self._client.write_registers(COMMAND_ADDRESS, registers, **self._unit)
        if result.isError():
            raise BackendError(f"command write failed: {result}")

    def close(self) -> None:
        self._client.close()


class FakeBackend:
    """Simulated gripper for trying the driver and panel without hardware.

    Activation takes `activation_time` seconds. Fingers move linearly at a rate
    set by the speed register. When closing past `object_register`, the fingers
    stop there with a contact (gOBJ = 2); None means no object. Set `fault` to a
    gFLT code to simulate a gripper fault.
    """

    def __init__(
        self,
        activation_time: float = 1.5,
        object_register: int | None = None,
        clock=time.monotonic,
    ) -> None:
        self._activation_time = activation_time
        self._object_register = object_register
        self._clock = clock
        self._lock = threading.Lock()
        self._command = encode_command(activate=False, go_to=False)
        self._activation_start: float | None = None
        self._position = 0.0
        self._last_update = clock()
        self.fault = 0

    def write_command(self, registers: list[int]) -> None:
        with self._lock:
            self._advance()
            was_active = self._command[0] & 0x0100
            self._command = list(registers)
            if registers[0] & 0x0100 and not was_active:
                self._activation_start = self._clock()
            if not registers[0] & 0x0100:
                self._activation_start = None

    def read_status(self) -> RawStatus:
        with self._lock:
            self._advance()
            act = (self._command[0] >> 8) & 0x01
            gto = (self._command[0] >> 11) & 0x01
            activated = self._activated()
            if not act:
                state = STATE_RESET
            else:
                state = STATE_ACTIVATED if activated else STATE_ACTIVATING
            obj = OBJECT_MOVING
            target = self._command[1]
            if gto and activated:
                stop = self._stop_register(target)
                if abs(self._position - stop) < 0.5:
                    obj = OBJECT_AT_REQUEST if stop == target else OBJECT_CONTACT_CLOSING
            moving = gto and activated and obj == OBJECT_MOVING
            return RawStatus(
                gACT=act,
                gGTO=gto,
                gSTA=state,
                gOBJ=obj,
                gFLT=self.fault,
                kFLT=0,
                gPR=target if act else 0,
                gPO=round(self._position),
                gCU=30 if moving else 0,
            )

    def close(self) -> None:
        pass

    def _activated(self) -> bool:
        start = self._activation_start
        return start is not None and self._clock() - start >= self._activation_time

    def _stop_register(self, target: int) -> float:
        obj = self._object_register
        if obj is not None and target > obj and self._position <= obj:
            return float(obj)
        return float(target)

    def _advance(self) -> None:
        now = self._clock()
        dt = now - self._last_update
        self._last_update = now
        gto = (self._command[0] >> 11) & 0x01
        if not (gto and self._activated()):
            return
        stop = self._stop_register(self._command[1])
        speed = self._command[2] >> 8
        rate = 50.0 + 1.5 * speed  # registers per second
        step = rate * dt
        if self._position < stop:
            self._position = min(self._position + step, stop)
        else:
            self._position = max(self._position - step, stop)
