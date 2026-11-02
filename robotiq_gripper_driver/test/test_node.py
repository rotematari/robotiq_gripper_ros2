"""End-to-end ROS tests of the node against the simulated gripper (a few seconds)."""

import contextlib
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")

from control_msgs.action import ParallelGripperCommand  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from robotiq_gripper_msgs.msg import GripperStatus  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

from robotiq_gripper_driver.backend import FakeBackend  # noqa: E402
from robotiq_gripper_driver.node import GripperNode  # noqa: E402

TIMEOUT = 5.0


@contextlib.contextmanager
def _running(backend):
    rclpy.init()
    node = GripperNode(backend=backend)
    client = rclpy.create_node("test_client")
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.add_node(client)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        yield node, client
    finally:
        executor.shutdown()
        node.destroy_node()
        client.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=2.0)


@pytest.fixture
def backend():
    # Hand-E object at register 180: closing past it stops with a contact.
    return FakeBackend(activation_time=0.2, object_register=180)


@pytest.fixture
def system(backend):
    with _running(backend) as running:
        yield running


def _wait(future, timeout=TIMEOUT):
    deadline = time.monotonic() + timeout
    while not future.done():
        if time.monotonic() > deadline:
            raise AssertionError("timed out")
        time.sleep(0.01)
    return future.result()


def _trigger(client, name):
    srv = client.create_client(Trigger, f"/robotiq_gripper/{name}")
    assert srv.wait_for_service(TIMEOUT)
    return _wait(srv.call_async(Trigger.Request()))


def _goal(width, speed=None, force=None):
    goal = ParallelGripperCommand.Goal()
    goal.command.position = [width]
    goal.command.velocity = [] if speed is None else [speed]
    goal.command.effort = [] if force is None else [force]
    return goal


def _send(action, goal):
    handle = _wait(action.send_goal_async(goal))
    if not handle.accepted:
        return None, None
    result = _wait(handle.get_result_async(), timeout=10.0)
    return result.status, result.result


def test_status_activation_motion_contact_and_rejection(system):
    _, client = system
    statuses = []
    client.create_subscription(GripperStatus, "/robotiq_gripper/status", statuses.append, 10)
    action = ActionClient(client, ParallelGripperCommand, "/robotiq_gripper/gripper_cmd")
    assert action.wait_for_server(TIMEOUT)

    # Not activated yet: accepted (valid) but aborted.
    status, _ = _send(action, _goal(0.025))
    assert status == 6  # GoalStatus.STATUS_ABORTED

    response = _trigger(client, "activate")
    assert response.success, response.message

    status, result = _send(action, _goal(0.025, speed=0.15, force=50.0))
    assert status == 4  # STATUS_SUCCEEDED
    assert result.reached_goal and not result.stalled
    assert result.state.position[0] == pytest.approx(0.025, abs=0.0005)

    # Closing stops on the simulated object at register 180 (about 14.7 mm open).
    status, result = _send(action, _goal(0.0, speed=0.15))
    assert status == 4 and result.stalled and not result.reached_goal
    assert result.state.position[0] == pytest.approx(0.050 * 75 / 255, abs=0.0005)

    # Out-of-range width and force are rejected before execution.
    assert _send(action, _goal(0.06)) == (None, None)
    assert _send(action, _goal(0.01, force=500.0)) == (None, None)

    assert statuses, "no status published"
    last = statuses[-1]
    assert last.model == "hande" and last.activated and last.connected
    assert last.stroke == pytest.approx(0.050) and last.max_force == pytest.approx(130.0)


def _status_now(client):
    got = []
    sub = client.create_subscription(GripperStatus, "/robotiq_gripper/status", got.append, 10)
    deadline = time.monotonic() + TIMEOUT
    while not got and time.monotonic() < deadline:
        time.sleep(0.01)
    client.destroy_subscription(sub)
    assert got, "no status published"
    return got[-1]


def test_new_goal_preempts_and_cancel_stops(system):
    _, client = system
    action = ActionClient(client, ParallelGripperCommand, "/robotiq_gripper/gripper_cmd")
    assert action.wait_for_server(TIMEOUT)
    assert _trigger(client, "activate").success

    # Slowest speed: the first goal is still moving when the second arrives. The
    # first must end through preemption, well before any timeout (>= 7.5 s here).
    first = _wait(action.send_goal_async(_goal(0.0, speed=0.02)))
    time.sleep(0.2)
    second = _wait(action.send_goal_async(_goal(0.05, speed=0.15)))
    sent = time.monotonic()
    assert _wait(first.get_result_async(), 1.0).status == 6  # aborted (preempted)
    assert time.monotonic() - sent < 1.0
    done = _wait(second.get_result_async(), 10.0)
    assert done.status == 4 and done.result.reached_goal
    assert done.result.state.position[0] == pytest.approx(0.05, abs=0.0005)

    moving = _wait(action.send_goal_async(_goal(0.0, speed=0.02)))
    time.sleep(0.2)
    _wait(moving.cancel_goal_async())
    assert _wait(moving.get_result_async(), 1.0).status == 5  # STATUS_CANCELED
    time.sleep(0.15)
    assert not _status_now(client).go_to_active  # the cancel sent a stop

    assert _trigger(client, "reset").success
    time.sleep(0.15)
    assert not _status_now(client).activated
    stop = _trigger(client, "stop")
    assert stop.success and stop.message.startswith("nothing sent")


def test_stop_service_ends_a_running_goal(system):
    _, client = system
    action = ActionClient(client, ParallelGripperCommand, "/robotiq_gripper/gripper_cmd")
    assert action.wait_for_server(TIMEOUT)
    assert _trigger(client, "activate").success
    moving = _wait(action.send_goal_async(_goal(0.0, speed=0.02)))
    time.sleep(0.2)
    stop = _trigger(client, "stop")
    assert stop.success and stop.message == "stopped"
    assert _wait(moving.get_result_async(), 1.0).status == 6
    time.sleep(0.15)
    assert not _status_now(client).go_to_active


def test_a_gripper_fault_aborts_the_goal(system, backend):
    _, client = system
    action = ActionClient(client, ParallelGripperCommand, "/robotiq_gripper/gripper_cmd")
    assert action.wait_for_server(TIMEOUT)
    assert _trigger(client, "activate").success
    moving = _wait(action.send_goal_async(_goal(0.0, speed=0.02)))
    time.sleep(0.2)
    backend.fault = 0x0E
    assert _wait(moving.get_result_async(), 1.0).status == 6
    status = _status_now(client)
    assert status.fault_code == 0x0E and status.fault_message == "overcurrent triggered"


class DeadBackend:
    def read_status(self):
        raise OSError("no response from the gripper")

    def write_command(self, registers):
        raise OSError("no response from the gripper")

    def close(self):
        pass


def test_disconnected_gripper_reports_and_survives():
    with _running(DeadBackend()) as (_, client):
        status = _status_now(client)
        assert not status.connected and "no response" in status.fault_message
        response = _trigger(client, "activate")
        assert not response.success and "no communication" in response.message
        assert _trigger(client, "stop").message.startswith("nothing sent")
        reset = _trigger(client, "reset")
        assert not reset.success and "OSError" in reset.message
        action = ActionClient(client, ParallelGripperCommand, "/robotiq_gripper/gripper_cmd")
        assert action.wait_for_server(TIMEOUT)
        assert _send(action, _goal(0.02))[0] == 6
        assert not _status_now(client).connected  # still polling


class _StubHandle:
    """Just enough of a ServerGoalHandle to drive `_execute` directly."""

    def __init__(self, uuid, request):
        self.goal_id = type("GoalId", (), {"uuid": uuid})()
        self.request = request
        self.is_cancel_requested = False
        self.ended = None

    def publish_feedback(self, feedback):
        pass

    def succeed(self):
        self.ended = "succeed"

    def canceled(self):
        self.ended = "cancel"

    def abort(self):
        self.ended = "abort"


def test_cancelling_a_preempted_goal_does_not_stop_the_new_goal(system):
    # Common client pattern: send the new goal, then cancel the old one. The old
    # goal no longer owns the gripper, so its cancel must not stop the new motion.
    # Driven directly so the cancel always lands after the preemption.
    node, client = system
    assert _trigger(client, "activate").success
    old, new = bytes(range(16)), bytes(range(1, 17))
    handle = _StubHandle(old, _goal(0.0, speed=0.02))
    node._active_goal = old
    go_to, stops = node._ctrl.go_to, []

    def go_to_then_preempt_and_cancel(*args):
        seq = go_to(*args)
        node._active_goal = new  # the new goal was accepted...
        handle.is_cancel_requested = True  # ...and then the old one cancelled
        return seq

    node._ctrl.go_to = go_to_then_preempt_and_cancel
    node._ctrl.stop = lambda: stops.append(True) or True
    node._execute(handle)
    assert handle.ended == "cancel"
    assert not stops, "the cancelled old goal stopped the new goal's motion"
    assert node._active_goal == new
