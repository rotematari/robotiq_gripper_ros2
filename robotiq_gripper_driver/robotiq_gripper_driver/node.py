"""ROS 2 node: Robotiq gripper action, services, status and joint state."""

from __future__ import annotations

import sys
import threading
import time

import rclpy
from control_msgs.action import ParallelGripperCommand
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from robotiq_gripper_msgs.msg import GripperStatus
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from robotiq_gripper_driver.backend import (
    OBJECT_CONTACT_CLOSING,
    OBJECT_CONTACT_OPENING,
    OBJECT_MOVING,
    BackendError,
    FakeBackend,
    ModbusBackend,
    fault_message,
)
from robotiq_gripper_driver.controller import (
    ActivationBusyError,
    GripperController,
    GripperNotReadyError,
    move_outcome,
)
from robotiq_gripper_driver.models import Converter, get_model

ACTIVATION_TIMEOUT = 15.0  # s; activation sweeps the full stroke


class GripperNode(Node):
    def __init__(self, backend=None) -> None:
        super().__init__("robotiq_gripper")
        p = self._declare_parameters()
        self._model = get_model(p["model"])
        self._conv = Converter(self._model, p["open_register"], p["closed_register"])
        self._joint = p["joint_name"]
        self._timeout = p["move_timeout"]
        if p["status_rate"] < 2.0:
            # The gripper faults (0x09) after 1 s without communication.
            raise ValueError("status_rate must be at least 2 Hz")
        if self._timeout <= 0:
            raise ValueError("move_timeout must be positive")
        self._poll_period = 1.0 / p["status_rate"]
        speed = p["default_speed"] if p["default_speed"] > 0 else self._model.min_speed
        force = p["default_force"] if p["default_force"] > 0 else self._model.min_force
        self._default_speed = self._conv.speed_to_register(speed)
        self._default_force = self._conv.force_to_register(force)

        if backend is not None:
            mode = f"injected {type(backend).__name__}"
        else:
            mode = p["port"] if p["backend"] == "modbus" else "simulated gripper"
            if p["backend"] == "fake":
                backend = FakeBackend()
            elif p["backend"] == "modbus":
                if not p["port"]:
                    raise ValueError("parameter 'port' is required (for example /dev/ttyUSB0)")
                backend = ModbusBackend(p["port"], p["device_id"])
            else:
                raise ValueError(f"backend must be 'modbus' or 'fake', got {p['backend']!r}")
        self._ctrl = GripperController(backend)

        self._goal_lock = threading.Lock()
        self._active_goal = None  # goal id of the goal allowed to run
        group = ReentrantCallbackGroup()
        self._status_pub = self.create_publisher(GripperStatus, "~/status", 10)
        self._joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self.create_service(Trigger, "~/activate", self._on_activate, callback_group=group)
        self.create_service(Trigger, "~/stop", self._on_stop, callback_group=group)
        self.create_service(Trigger, "~/reset", self._on_reset, callback_group=group)
        self._action = ActionServer(
            self,
            ParallelGripperCommand,
            "~/gripper_cmd",
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            handle_accepted_callback=self._on_accepted,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=group,
        )
        # The poller runs in its own thread so the gripper's 1 s communication
        # watchdog never depends on free executor threads.
        self._stop_polling = threading.Event()
        self._poller = threading.Thread(target=self._poll_loop, name="gripper_poll", daemon=True)
        self._poller.start()

        self.get_logger().info(f"Robotiq {self._model.name} on {mode}")
        if p["auto_activate"]:
            threading.Thread(target=self._auto_activate, daemon=True).start()

    def _declare_parameters(self) -> dict:
        defaults = {
            "port": "",
            "model": "hande",
            "device_id": 9,
            "backend": "modbus",
            "joint_name": "gripper_width",
            "status_rate": 20.0,
            "default_speed": 0.0,  # m/s; 0 means the model minimum
            "default_force": 0.0,  # N; 0 means the model minimum
            "open_register": 0,
            "closed_register": 255,
            "move_timeout": 5.0,  # s beyond a full stroke at the commanded speed
            "auto_activate": False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        return {name: self.get_parameter(name).value for name in defaults}

    # --- status ---------------------------------------------------------

    def _poll_loop(self) -> None:
        next_time = time.monotonic()
        while not self._stop_polling.is_set():
            try:
                self._publish(self._ctrl.poll())
            except Exception as exc:  # keep polling: the gripper faults without communication
                self.get_logger().error(f"status publish failed: {exc}", throttle_duration_sec=5.0)
            next_time = max(next_time + self._poll_period, time.monotonic())
            self._stop_polling.wait(next_time - time.monotonic())

    def _publish(self, snap) -> None:
        if not snap.connected:
            self.get_logger().error(f"gripper read failed: {snap.error}", throttle_duration_sec=5.0)
        msg = self._status_msg(snap)
        self._status_pub.publish(msg)
        if snap.status is not None:
            joint = JointState()
            joint.header.stamp = msg.header.stamp
            joint.name = [self._joint]
            joint.position = [msg.width]
            self._joint_pub.publish(joint)

    def _status_msg(self, snap) -> GripperStatus:
        m = self._model
        msg = GripperStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.model = m.name
        msg.connected = snap.connected
        msg.stroke, msg.min_force, msg.max_force = m.stroke, m.min_force, m.max_force
        msg.min_speed, msg.max_speed = m.min_speed, m.max_speed
        s = snap.status
        if s is None:
            msg.fault_message = snap.error
            return msg
        msg.activated = s.activated
        msg.activation_state = s.gSTA
        msg.go_to_active = bool(s.gGTO)
        msg.object_status = s.gOBJ
        msg.moving = bool(s.gGTO) and s.gOBJ == OBJECT_MOVING
        msg.object_detected = bool(s.gGTO) and s.gOBJ in (
            OBJECT_CONTACT_OPENING,
            OBJECT_CONTACT_CLOSING,
        )
        msg.fault_code = s.gFLT
        msg.fault_message = fault_message(s.gFLT) if snap.connected else snap.error
        msg.width = self._conv.register_to_width(s.gPO)
        msg.requested_width = self._conv.register_to_width(s.gPR)
        msg.current = s.gCU * 0.01  # manual: current ~ 10 x value, in mA
        msg.position_register, msg.requested_register, msg.current_register = s.gPO, s.gPR, s.gCU
        return msg

    # --- services -------------------------------------------------------

    def _on_activate(self, _request, response):
        try:
            response.success = self._ctrl.activate(ACTIVATION_TIMEOUT)
            response.message = "activated" if response.success else self._activation_failure()
        except (ActivationBusyError, GripperNotReadyError) as exc:
            response.success, response.message = False, str(exc)
        except Exception as exc:
            response.success, response.message = False, f"{type(exc).__name__}: {exc}"
        return response

    def _activation_failure(self) -> str:
        snap = self._ctrl.snapshot()
        if not snap.connected:
            return f"no communication: {snap.error}"
        if snap.status is not None and snap.status.gFLT:
            return f"fault 0x{snap.status.gFLT:02X}: {fault_message(snap.status.gFLT)}"
        return f"activation did not finish within {ACTIVATION_TIMEOUT:g} s"

    def _auto_activate(self) -> None:
        # Wait for the first status so activation sees the real gripper state.
        self._ctrl.wait_for_status(0, 2.0)
        try:
            if not self._ctrl.activate(ACTIVATION_TIMEOUT):
                self.get_logger().error(f"auto activation failed: {self._activation_failure()}")
        except Exception as exc:
            self.get_logger().error(f"auto activation failed: {exc}")

    def _on_stop(self, _request, response):
        # Under the goal lock, so no goal can write a go-to after this stop.
        try:
            with self._goal_lock:
                self._active_goal = None  # ends any running goal
                sent = self._ctrl.stop()
            response.success = True
            response.message = "stopped" if sent else "nothing sent: not activated or no communication"
        except Exception as exc:
            response.success, response.message = False, f"{type(exc).__name__}: {exc}"
        return response

    def _on_reset(self, _request, response):
        try:
            with self._goal_lock:
                self._active_goal = None
                self._ctrl.reset()
            response.success, response.message = True, "reset: deactivated, faults cleared"
        except Exception as exc:
            response.success, response.message = False, f"{type(exc).__name__}: {exc}"
        return response

    # --- action ---------------------------------------------------------

    def _registers_for(self, goal) -> tuple[int, int, int]:
        """Convert a goal to (position, speed, force) registers; raise ValueError if invalid."""
        cmd = goal.command
        if len(cmd.position) != 1:
            raise ValueError("command.position must hold exactly one value (opening width, m)")
        if cmd.name and list(cmd.name) != [self._joint]:
            raise ValueError(f"command.name must be empty or [{self._joint!r}]")
        if len(cmd.velocity) > 1 or len(cmd.effort) > 1:
            raise ValueError("command.velocity and command.effort hold at most one value")
        position = self._conv.width_to_register(cmd.position[0])
        speed = self._conv.speed_to_register(cmd.velocity[0]) if cmd.velocity else self._default_speed
        force = self._conv.force_to_register(cmd.effort[0]) if cmd.effort else self._default_force
        return position, speed, force

    def _on_goal(self, goal) -> GoalResponse:
        try:
            self._registers_for(goal)
        except ValueError as exc:
            self.get_logger().warning(f"rejected gripper goal: {exc}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_accepted(self, handle) -> None:
        # A new goal preempts the running one; the old execute loop sees this and aborts.
        with self._goal_lock:
            self._active_goal = bytes(handle.goal_id.uuid)
        handle.execute()

    def _execute(self, handle):
        goal_id = bytes(handle.goal_id.uuid)
        position, speed, force = self._registers_for(handle.request)
        result = ParallelGripperCommand.Result()
        error = None
        # The check and the write happen under the goal lock: only the active goal
        # writes, and a stop or a newer goal always writes after it.
        with self._goal_lock:
            if self._active_goal != goal_id:
                error = "preempted before it started"
            else:
                try:
                    seq = self._ctrl.go_to(position, speed, force)
                except GripperNotReadyError as exc:
                    error = str(exc)
                except Exception as exc:
                    error = f"{type(exc).__name__}: {exc}"
        if error is not None:
            self.get_logger().error(f"gripper goal aborted: {error}")
            return self._finish(handle, result, "abort")

        # Allow a full stroke at the commanded speed plus the configured margin.
        travel = self._model.stroke / self._conv.register_to_speed(speed)
        limit = travel + self._timeout
        deadline = time.monotonic() + limit
        while True:
            # Only the goal that owns the gripper stops it: cancelling a goal that
            # was already preempted must not stop the newer goal's motion.
            with self._goal_lock:
                owner = self._active_goal == goal_id
                cancelled = handle.is_cancel_requested
                if cancelled and owner:
                    self._active_goal = None
                    self._ctrl.stop()
            if cancelled:
                return self._finish(handle, result, "cancel")
            if not owner:
                return self._finish(handle, result, "abort")
            snap = self._ctrl.wait_for_status(seq, 0.1)
            if snap is None:
                if time.monotonic() > deadline:
                    self._stop_if_owner(goal_id)
                    self.get_logger().error("gripper goal aborted: no status before the timeout")
                    return self._finish(handle, result, "abort")
                continue
            seq = snap.seq
            status = snap.status
            self._fill_state(result.state, status)
            feedback = ParallelGripperCommand.Feedback()
            self._fill_state(feedback.state, status)
            handle.publish_feedback(feedback)
            if snap.connected and status.gFLT >= 0x08:
                # Minor and major faults; 0x05 and 0x07 resolve once a command is taken.
                self.get_logger().error(f"gripper goal aborted: {fault_message(status.gFLT)}")
                return self._finish(handle, result, "abort")
            outcome = move_outcome(status, position) if snap.connected else None
            if outcome is not None:
                result.reached_goal = outcome == "reached"
                result.stalled = outcome == "stalled"
                return self._finish(handle, result, "succeed")
            if time.monotonic() > deadline:
                self._stop_if_owner(goal_id)
                self.get_logger().error(f"gripper goal aborted: not done within {limit:.1f} s")
                return self._finish(handle, result, "abort")

    def _stop_if_owner(self, goal_id: bytes) -> None:
        with self._goal_lock:
            if self._active_goal == goal_id:
                self._ctrl.stop()

    def _fill_state(self, state, status) -> None:
        state.header.stamp = self.get_clock().now().to_msg()
        state.name = [self._joint]
        state.position = [self._conv.register_to_width(status.gPO)]

    def _finish(self, handle, result, how):
        with self._goal_lock:
            if self._active_goal == bytes(handle.goal_id.uuid):
                self._active_goal = None
        if how == "succeed":
            handle.succeed()
        elif how == "cancel":
            handle.canceled()
        else:
            handle.abort()
        return result

    def destroy_node(self) -> None:
        self._stop_polling.set()
        self._poller.join(timeout=2.0)
        self._ctrl.close()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        try:
            node = GripperNode()
        except (ValueError, BackendError) as exc:
            rclpy.logging.get_logger("robotiq_gripper").fatal(str(exc))
            sys.exit(1)
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

