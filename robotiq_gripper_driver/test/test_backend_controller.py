import threading
import time

import pytest

from robotiq_gripper_driver.backend import (
    OBJECT_AT_REQUEST,
    OBJECT_CONTACT_CLOSING,
    STATE_ACTIVATED,
    FakeBackend,
    RawStatus,
    decode_status,
    encode_command,
    fault_message,
)
from robotiq_gripper_driver.controller import (
    ActivationBusyError,
    GripperController,
    GripperNotReadyError,
    move_outcome,
)


def test_decode_status_matches_manual_layout():
    # gOBJ=2, gSTA=3, gGTO=1, gACT=1 -> 0b10111001; fault 0x0E, gPR 0xC8; gPO 0xC0, gCU 0x12
    status = decode_status([0b10111001 << 8, (0x0E << 8) | 0xC8, (0xC0 << 8) | 0x12])
    assert status == RawStatus(
        gACT=1, gGTO=1, gSTA=STATE_ACTIVATED, gOBJ=2, gFLT=0x0E, kFLT=0, gPR=0xC8, gPO=0xC0, gCU=0x12
    )
    assert status.activated
    assert fault_message(0x0E) == "overcurrent triggered"
    assert fault_message(0) == ""
    with pytest.raises(ValueError):
        decode_status([0, 0])


def test_encode_command_packs_action_and_setpoints():
    assert encode_command(activate=True, go_to=True, position=200, speed=10, force=20) == [
        0x0900,
        200,
        (10 << 8) | 20,
    ]
    assert encode_command(activate=False, go_to=False) == [0, 0, 0]
    with pytest.raises(ValueError):
        encode_command(activate=True, go_to=True, position=256)


def _status(gGTO=1, gOBJ=0, gPR=100):
    return RawStatus(gACT=1, gGTO=gGTO, gSTA=3, gOBJ=gOBJ, gFLT=0, kFLT=0, gPR=gPR, gPO=0, gCU=0)


def test_move_outcome_needs_the_new_request_echo():
    # Leftover "at request" from the previous target 50 is not the result for 100.
    assert move_outcome(_status(gOBJ=OBJECT_AT_REQUEST, gPR=50), 100) is None
    assert move_outcome(_status(gOBJ=0, gPR=100), 100) is None
    assert move_outcome(_status(gGTO=0, gOBJ=OBJECT_AT_REQUEST), 100) is None
    assert move_outcome(_status(gOBJ=OBJECT_AT_REQUEST), 100) == "reached"
    assert move_outcome(_status(gOBJ=OBJECT_CONTACT_CLOSING), 100) == "stalled"


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


def _poll_until(ctrl, clock, predicate, steps=400, dt=0.01):
    for _ in range(steps):
        clock.now += dt
        snap = ctrl.poll()
        if predicate(snap.status):
            return snap.status
    raise AssertionError("condition not reached")


def test_fake_gripper_activates_moves_and_detects_an_object():
    clock = FakeClock()
    ctrl = GripperController(FakeBackend(activation_time=0.5, object_register=180, clock=clock))
    ctrl.poll()
    with pytest.raises(GripperNotReadyError):
        ctrl.go_to(100, 255, 0)
    assert not ctrl.stop()  # must not start activation
    assert not ctrl.poll().status.gACT

    ctrl._write(encode_command(activate=True, go_to=False))
    _poll_until(ctrl, clock, lambda s: s.activated)

    ctrl.go_to(100, 255, 0)
    status = _poll_until(ctrl, clock, lambda s: move_outcome(s, 100) is not None)
    assert move_outcome(status, 100) == "reached" and status.gPO == 100

    ctrl.go_to(255, 255, 0)
    status = _poll_until(ctrl, clock, lambda s: move_outcome(s, 255) is not None)
    assert move_outcome(status, 255) == "stalled" and status.gPO == 180

    assert ctrl.stop()
    assert ctrl.poll().status.gGTO == 0
    ctrl.reset()
    assert not ctrl.poll().status.gACT


def test_activate_waits_for_the_poller_and_times_out():
    ctrl = GripperController(FakeBackend(activation_time=0.05))
    stop = threading.Event()

    def poller():
        while not stop.is_set():
            ctrl.poll()
            stop.wait(0.005)

    thread = threading.Thread(target=poller)
    thread.start()
    try:
        assert ctrl.activate(timeout=2.0)
        assert ctrl.activate(timeout=0.0)  # already activated
    finally:
        stop.set()
        thread.join()

    slow = GripperController(FakeBackend(activation_time=10.0))
    slow.poll()
    assert not slow.activate(timeout=0.05)  # nobody polls: no new status


class FailingBackend:
    def read_status(self):
        raise OSError("device unplugged")

    def write_command(self, registers):
        raise OSError("device unplugged")

    def close(self):
        pass


def test_poll_reports_io_errors_without_raising():
    ctrl = GripperController(FailingBackend())
    snap = ctrl.poll()
    assert not snap.connected and "device unplugged" in snap.error and snap.seq == 1
    assert ctrl.wait_for_status(snap.seq, timeout=0.01) is None


class FlakyBackend(FakeBackend):
    """A fake gripper whose link can be cut; records every command write."""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.link_up = True
        self.writes = []

    def read_status(self):
        if not self.link_up:
            raise OSError("no response")
        return super().read_status()

    def write_command(self, registers):
        self.writes.append(list(registers))
        super().write_command(registers)


def test_no_command_is_written_on_a_stale_status():
    clock = FakeClock()
    backend = FlakyBackend(activation_time=0.0, clock=clock)
    ctrl = GripperController(backend)
    ctrl.poll()
    ctrl._write(encode_command(activate=True, go_to=False))
    _poll_until(ctrl, clock, lambda s: s.activated)
    backend.writes.clear()

    backend.link_up = False
    snap = ctrl.poll()
    assert not snap.connected and snap.status.activated  # the stale status still says activated
    assert not ctrl.stop()
    with pytest.raises(GripperNotReadyError, match="no communication"):
        ctrl.go_to(100, 255, 0)
    with pytest.raises(GripperNotReadyError, match="no communication"):
        ctrl.activate(timeout=0.1)
    assert backend.writes == []


def test_concurrent_activation_is_refused():
    ctrl = GripperController(FakeBackend(activation_time=10.0))
    ctrl.poll()
    ctrl._activating.acquire()
    try:
        with pytest.raises(ActivationBusyError):
            ctrl.activate(timeout=0.1)
    finally:
        ctrl._activating.release()


def test_stop_rides_out_a_single_failed_read():
    backend = FlakyBackend(activation_time=0.0)
    ctrl = GripperController(backend)
    ctrl.poll()
    ctrl._write(encode_command(activate=True, go_to=False))
    while not ctrl.poll().status.activated:
        pass
    backend.link_up = False
    ctrl.poll()  # one failed read
    backend.writes.clear()
    backend.link_up = True
    poller = threading.Timer(0.05, ctrl.poll)  # the next read succeeds
    poller.start()
    try:
        assert ctrl.stop()
    finally:
        poller.join()
    assert len(backend.writes) == 1 and backend.writes[0][0] == 0x0100  # rACT=1, rGTO=0


def test_activate_resets_a_gripper_left_with_ract_set():
    # Seen on the lab Hand-E: rACT = 1 latched but the gripper never activated
    # (gACT = 1, gSTA = 0). Activation needs a 0 -> 1 edge, so activate() must
    # clear rACT before setting it.
    backend = FlakyBackend(activation_time=0.05)
    backend.write_command(encode_command(activate=True, go_to=False))
    backend._activation_start = None  # latched, not activating
    ctrl = GripperController(backend)
    ctrl.poll()
    backend.writes.clear()
    stop = threading.Event()

    def poller():
        while not stop.is_set():
            ctrl.poll()
            time.sleep(0.01)

    thread = threading.Thread(target=poller)
    thread.start()
    try:
        assert ctrl.activate(timeout=2.0)
    finally:
        stop.set()
        thread.join()
    assert [w[0] for w in backend.writes] == [0x0000, 0x0100]  # reset, then activate
