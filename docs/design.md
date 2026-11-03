# Design: ROS 2 Jazzy driver and RViz panel for Robotiq grippers over USB/RS-485

Agreed with Rotem on 2 November 2026.

## Intent

Drive a Robotiq Hand-E, 2F-85 or 2F-140 connected to a Linux PC through a
USB-to-RS-485 adapter: from an RViz panel next to MoveIt, from the command line,
and from any ROS 2 client. The main use is "go to width X with force Y". The repo
is meant to be public and is not tied to one lab or one robot arm.

## Goals

- One driver node owns the serial port. It offers the standard Jazzy gripper
  action (`control_msgs/action/ParallelGripperCommand`), activate/stop/reset
  services, a detailed status topic, and a `sensor_msgs/JointState` with the
  finger opening.
- Commands and status use SI units: opening width in m, speed in m/s, force in N.
  The driver converts these to the gripper's 0-255 registers for each model.
- An RViz panel with width, speed and force sliders; Activate, Open, Close, Go and
  Stop buttons; and a status line.
- It works without a robot arm and without MoveIt.

## Non-goals

- Closed-loop force regulation from external sensors (tactile pads, wrist F/T).
  "Force" is the gripper's own force limit: the fingers move to the target and,
  on contact, stop and hold with about that force.
- A ros2_control hardware interface, and anything running inside an arm's
  real-time controller manager.
- Robot description (URDF) for the grippers.
- Modbus TCP, the UR tool-flange route, and other Robotiq products (3F, E-Pick).

## Anti-goals

- Moving the fingers without an explicit request. Activation sweeps the fingers
  fully open and closed, so it only runs when the activate service is called or
  `auto_activate` is set to true; it defaults to false.
- Automatic serial-port probing. The port is always explicit.

## Decisions

- **Serial layer: [pymodbus](https://github.com/pymodbus-dev/pymodbus) directly**,
  behind a small backend interface (`backend.py`), with this repo's own register
  encode/decode (about 60 lines, unit-tested). It works with pymodbus 3.6 (Ubuntu
  24.04 apt, rosdep key `python3-pymodbus`) and newer pip releases, so no pip-only
  dependency blocks a build-farm release. The plan was to reuse
  [pyRobotiqGripper](https://github.com/castetsb/pyRobotiqGripper) 3.2.5, which is
  built on pymodbus. Reading its code showed behaviour unsafe for a driver: `stop()`
  writes rACT = 1 even when the gripper is not activated, which starts an
  activation sweep. Its status read raises on major faults before storing the fault
  registers; `move()` refuses to run after a stop (gGTO = 0); and its baudrate
  argument is ignored. Rotem approved the switch on 2 November 2026.
- **Commands only on fresh status:** go-to and stop (both write rACT = 1) are
  sent only while the latest status read succeeded and showed an activated
  gripper. A stale or failed read never counts as activated. After a failed
  read, a command waits up to 0.5 s for the next successful one, so a single
  dropped reply does not refuse a stop.
- **Status polling has its own thread**, not an executor timer, so the gripper's
  1 s communication watchdog never depends on free executor threads.
- **Standard action on Jazzy:** `ParallelGripperCommand`. `position[0]` is the
  opening width in m (required). `velocity[0]` (m/s) and `effort[0]` (N) are
  optional; when absent, `default_speed` and `default_force` are used.
- **New goals preempt the active goal.** This matches ros2_controllers'
  gripper controller and suits a slider-driven panel.
- **Out-of-range goals are rejected, not clamped.** The status topic publishes
  the model's ranges so clients can check before sending.
- **Done detection:** the goal finishes only once the gripper echoes the new
  position request (gPR) and reports gOBJ 1, 2 or 3. That avoids treating the
  previous motion's "at position" status as the new result.

## Unit conversions (approximate; model table in `models.py`)

| Model | Stroke | Force | Closing speed |
|---|---|---|---|
| Hand-E | 50 mm | 20-130 N | 20-150 mm/s |
| 2F-85 | 85 mm | 20-235 N | 20-150 mm/s |
| 2F-140 | 140 mm | 10-125 N | 30-250 mm/s |

Sources: Hand-E manual, Specifications section (measured with coupling
GRP-CPL-062 and the basic aluminium fingertips); Robotiq Adaptive Grippers
product sheet for the 2F models.

- **Width:** linear between `open_register` (default 0) and `closed_register`
  (default 255). The manual calls the mapping quasi-linear, at about 0.2 mm per
  count on the Hand-E, and also mentions 245 as the closed limit after
  activation. Calibrate both registers against calipers for accurate widths.
- **Force and speed:** linear from the model's minimum (register 0) to its
  maximum (register 255). Robotiq publishes no register-to-newton curve, and the
  force register actually limits motor current. Treat newtons as an approximate
  scale, not a measurement. Register 0 is the gentlest setting and also turns off
  the gripper's automatic re-grasp.

## Interfaces (node `robotiq_gripper`, all names relative to its namespace)

| Name | Type | Notes |
|---|---|---|
| `~/gripper_cmd` | `control_msgs/action/ParallelGripperCommand` | go to width with speed and force |
| `~/activate`, `~/stop`, `~/reset` | `std_srvs/srv/Trigger` | reset clears faults and deactivates |
| `~/status` | `robotiq_gripper_msgs/msg/GripperStatus` | registers, SI values, model ranges |
| `joint_states` | `sensor_msgs/msg/JointState` | one joint, `joint_name`, opening width in m |

Parameters, with all defaults in `config/<model>.yaml` and validation in
`node.py` at startup: `port` (required), `model`, `device_id`, `baudrate`,
`joint_name`, `status_rate`, `default_speed`, `default_force`, `open_register`,
`closed_register`, `move_timeout` (time allowed beyond a full stroke at the
commanded speed), `auto_activate`, `backend` (`modbus`, or `fake` for a simulated
gripper that needs no hardware).

## Where it lives

Public repository [rotematari/robotiq_gripper_ros2](https://github.com/rotematari/robotiq_gripper_ros2),
published on 3 November 2026; the PhD workspace includes it as a submodule at
`packages/robotiq-gripper-ros2`. Build products (`build/`,
`install/`, `log/`, `.venv/`) are git-ignored next to the source.

## Verification

- Unit tests run without hardware (budget under 10 s): unit conversions, register
  encoding, done detection, preemption, cancellation, stop, fault abort, a
  disconnected gripper, and no command writes on a stale status.
- Offline end-to-end check: run the node with the fake backend, then drive it
  from the CLI and from the RViz panel.
- Bench check on the lab Hand-E, with Rotem present: activate, open, close, go to
  a width, close on an object at low force, stop, and recover from a fault.
