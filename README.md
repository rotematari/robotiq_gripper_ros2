# robotiq_gripper_ros2

ROS 2 Jazzy driver and RViz panel for **Robotiq Hand-E, 2F-85 and 2F-140** grippers
connected to a Linux PC through a USB-to-RS-485 adapter (Modbus RTU).

- `robotiq_gripper_driver`: Python node with the standard Jazzy gripper action
  (`control_msgs/action/ParallelGripperCommand`), activate/stop/reset services,
  a status topic and a joint state. It works in SI units: opening width in m,
  speed in m/s, force in N.
- `robotiq_gripper_rviz`: RViz panel to go to a width with a chosen speed and force,
  open, close, stop, activate and reset, with live status.
- `robotiq_gripper_msgs`: the status message.

A simulated gripper (`backend:=fake`) lets you try everything without hardware.

## Install

Ubuntu 24.04 with ROS 2 Jazzy. In a colcon workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/rotematari/robotiq_gripper_ros2.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y   # installs python3-pymodbus
colcon build
source install/setup.bash
```

Serial access needs the `dialout` group (`sudo usermod -aG dialout $USER`, then log in again).

## Run

```bash
ros2 launch robotiq_gripper_driver gripper.launch.py \
  port:=/dev/serial/by-id/usb-FTDI_USB_TO_RS-485_XXXXXXXX-if00-port0 model:=hande
```

`model` is `hande`, `2f85` or `2f140` and selects `config/<model>.yaml`; pass
`config:=<file>` to use your own. Without hardware: `backend:=fake`.

In RViz: **Panels > Add New Panel > robotiq_gripper_rviz > GripperPanel**. Press
**Activate** once after power-up. Activation moves the fingers through their full
stroke, so keep them clear.

From the command line:

```bash
ros2 service call /robotiq_gripper/activate std_srvs/srv/Trigger
# go to 20 mm opening at 50 mm/s with a 60 N force limit
ros2 action send_goal /robotiq_gripper/gripper_cmd control_msgs/action/ParallelGripperCommand \
  "{command: {position: [0.020], velocity: [0.05], effort: [60.0]}}"
```

## Interfaces

| Name | Type | |
|---|---|---|
| `~/gripper_cmd` | `control_msgs/action/ParallelGripperCommand` | `position[0]`: opening width (m), required. `velocity[0]` (m/s) and `effort[0]` (N) are optional and default to `default_speed` / `default_force`. Out-of-range goals are rejected. A new goal preempts the running one. A goal is aborted after a full stroke's travel time at its speed plus `move_timeout`, or on a gripper fault. The result reports `reached_goal`, or `stalled` when the fingers stopped on an object. |
| `~/activate` | `std_srvs/srv/Trigger` | Activates if needed and waits for it to finish (up to 15 s). Refused while another activation is waiting or without communication. |
| `~/stop` | `std_srvs/srv/Trigger` | Stops and holds the current position; ends any running goal. Sends nothing unless the gripper is communicating and activated. |
| `~/reset` | `std_srvs/srv/Trigger` | Deactivates and clears faults. Activate again before moving. |
| `~/status` | `robotiq_gripper_msgs/msg/GripperStatus` | Activation, motion, object detection, fault, width, current, raw registers and the model's ranges. |
| `joint_states` | `sensor_msgs/msg/JointState` | One joint (`joint_name`): the opening width in m. |

Parameters and their defaults are documented in [`config/hande.yaml`](robotiq_gripper_driver/config/hande.yaml).

## What "force" means

Robotiq grippers have no force controller. The force setting limits the motor
current: the fingers move toward the target and, if they touch an object first,
stop and squeeze with about that force. The driver maps newtons linearly onto the
gripper's 0-255 force register, from the model's minimum to its maximum (Hand-E
20-130 N, 2F-85 20-235 N, 2F-140 10-125 N). Robotiq publishes the range but no
calibration curve, so treat the newtons as approximate. The lowest setting also
turns off the gripper's automatic re-grasp. Widths are linear between
`open_register` and `closed_register`; calibrate those with calipers if you need
accurate widths. Details: [docs/design.md](docs/design.md).

## Tests

```bash
colcon test --packages-select robotiq_gripper_driver && colcon test-result --verbose
```

The tests run against the simulated gripper and take a few seconds.

## License

Apache-2.0.
