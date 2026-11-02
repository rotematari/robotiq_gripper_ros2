"""Thread-safe gripper state and commands, independent of ROS.

One poller thread calls `poll()` at a steady rate. That keeps the gripper's
communication watchdog fed (fault 0x09 after 1 s of silence). Commands and
waits run in other threads and see each new status through `wait_for_status()`.

Commands that write rACT = 1 (go-to, stop) are sent only while the latest poll
succeeded and showed an activated gripper. Writing rACT = 1 to a gripper that
is not activated starts an activation sweep, so a stale or failed status never
counts as activated.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass

from robotiq_gripper_driver.backend import (
    OBJECT_AT_REQUEST,
    OBJECT_CONTACT_CLOSING,
    OBJECT_CONTACT_OPENING,
    Backend,
    RawStatus,
    encode_command,
)


class GripperNotReadyError(RuntimeError):
    """A command needs a communicating, activated gripper."""


class ActivationBusyError(RuntimeError):
    """Another activation request is still waiting."""


@dataclass(frozen=True)
class Snapshot:
    """The latest poll result; `status` is the last successful read."""

    seq: int
    status: RawStatus | None
    connected: bool
    error: str

    @property
    def ready(self) -> bool:
        """True only if the latest read succeeded and showed an activated gripper."""
        return self.connected and self.status is not None and self.status.activated


def move_outcome(status: RawStatus, requested: int) -> str | None:
    """Return 'reached', 'stalled' or None (still moving) for a go-to to `requested`.

    The gripper must echo the new position request first (gPR), so a status
    left over from the previous motion is never taken as the result.
    """
    if status.gGTO != 1 or status.gPR != requested:
        return None
    if status.gOBJ == OBJECT_AT_REQUEST:
        return "reached"
    if status.gOBJ in (OBJECT_CONTACT_OPENING, OBJECT_CONTACT_CLOSING):
        return "stalled"
    return None


class GripperController:
    def __init__(self, backend: Backend) -> None:
        self._backend = backend
        # Lock order: _io, then _changed. A read and its publication happen under
        # _io, so a sequence number taken after a write (also under _io) is never
        # older than a status read before that write.
        self._io = threading.Lock()
        self._changed = threading.Condition()
        self._activating = threading.Lock()
        self._snapshot = Snapshot(0, None, False, "no status read yet")
        self._command = encode_command(activate=False, go_to=False)

    def snapshot(self) -> Snapshot:
        with self._changed:
            return self._snapshot

    def poll(self) -> Snapshot:
        """Read status once and publish it to waiters; never raises on I/O errors."""
        with self._io:
            try:
                status = self._backend.read_status()
                connected, error = True, ""
            except Exception as exc:  # serial and Modbus errors surface as different types
                status, connected, error = None, False, f"{type(exc).__name__}: {exc}"
            with self._changed:
                previous = self._snapshot
                self._snapshot = Snapshot(
                    previous.seq + 1,
                    status if status is not None else previous.status,
                    connected,
                    error,
                )
                self._changed.notify_all()
                return self._snapshot

    def wait_for_status(self, after_seq: int, timeout: float) -> Snapshot | None:
        """Wait for a snapshot newer than `after_seq`; None on timeout."""
        deadline = time.monotonic() + timeout
        with self._changed:
            while self._snapshot.seq <= after_seq:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._changed.wait(remaining)
            return self._snapshot

    def activate(self, timeout: float) -> bool:
        """Start activation and wait for it; False if it did not finish in time.

        Activation moves the fingers through their full stroke. Already-activated
        grippers are left alone. Raises GripperNotReadyError without communication
        and ActivationBusyError if another activation is waiting.
        """
        if not self._activating.acquire(blocking=False):
            raise ActivationBusyError("activation already in progress")
        try:
            start = self.fresh_snapshot()
            if not start.connected:
                raise GripperNotReadyError(f"no communication with the gripper: {start.error}")
            if start.ready:
                return True
            deadline = time.monotonic() + timeout
            # Activation starts on a 0 -> 1 edge of rACT. A gripper left with
            # rACT = 1 (an earlier attempt, or a fault) ignores another 1, so
            # clear it first and wait until the gripper reports gACT = 0.
            seq = self._write(encode_command(activate=False, go_to=False))
            while (remaining := deadline - time.monotonic()) > 0:
                snap = self.wait_for_status(seq, remaining)
                if snap is None:
                    return False
                seq = snap.seq
                if snap.connected and snap.status.gACT == 0:
                    break
            seq = self._write(encode_command(activate=True, go_to=False))
            while (remaining := deadline - time.monotonic()) > 0:
                snap = self.wait_for_status(seq, remaining)
                if snap is None:
                    break
                seq = snap.seq
                if snap.ready:
                    return True
            return False
        finally:
            self._activating.release()

    def fresh_snapshot(self, timeout: float = 0.5) -> Snapshot:
        """Return the latest snapshot, waiting up to `timeout` for a successful read.

        Rides out single failed reads (the RS-485 link drops an occasional reply)
        without ever treating a stale status as current.
        """
        snap = self.snapshot()
        deadline = time.monotonic() + timeout
        while not snap.connected and (remaining := deadline - time.monotonic()) > 0:
            newer = self.wait_for_status(snap.seq, remaining)
            if newer is None:
                break
            snap = newer
        return snap

    def reset(self) -> None:
        """Deactivate and clear faults (rACT = 0). The fingers stop where they are."""
        self._write(encode_command(activate=False, go_to=False))

    def stop(self) -> bool:
        """Stop motion and hold position (rGTO = 0); return False if nothing was sent.

        Sent only while the gripper is communicating and activated (see module doc).
        """
        if not self.fresh_snapshot().ready:
            return False
        _, position, speed_force = self._command
        self._write(
            encode_command(
                activate=True,
                go_to=False,
                position=position,
                speed=speed_force >> 8,
                force=speed_force & 0xFF,
            )
        )
        return True

    def go_to(self, position: int, speed: int, force: int) -> int:
        """Send a go-to; return the snapshot sequence number at the time of writing."""
        snap = self.fresh_snapshot()
        if not snap.connected:
            raise GripperNotReadyError(f"no communication with the gripper: {snap.error}")
        if not snap.ready:
            raise GripperNotReadyError("gripper is not activated; call activate first")
        return self._write(
            encode_command(activate=True, go_to=True, position=position, speed=speed, force=force)
        )

    def close(self) -> None:
        with self._io:
            self._backend.close()

    def _write(self, registers: list[int]) -> int:
        """Write command registers; return the latest snapshot sequence number."""
        with self._io:
            self._backend.write_command(registers)
            self._command = list(registers)
            with self._changed:
                return self._snapshot.seq
