"""Start the Robotiq gripper driver.

    ros2 launch robotiq_gripper_driver gripper.launch.py port:=/dev/serial/by-id/<adapter> model:=hande
    ros2 launch robotiq_gripper_driver gripper.launch.py backend:=fake    # no hardware
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument("port", default_value="", description="serial device of the RS-485 adapter"),
        DeclareLaunchArgument("model", default_value="hande", description="hande, 2f85 or 2f140"),
        DeclareLaunchArgument("backend", default_value="modbus", description="modbus or fake"),
        DeclareLaunchArgument("namespace", default_value="", description="ROS namespace for the node"),
        DeclareLaunchArgument(
            "config",
            default_value=PathJoinSubstitution(
                [FindPackageShare("robotiq_gripper_driver"), "config", [LaunchConfiguration("model"), ".yaml"]]
            ),
            description="parameter file (defaults to config/<model>.yaml)",
        ),
    ]
    node = Node(
        package="robotiq_gripper_driver",
        executable="robotiq_gripper_node",
        name="robotiq_gripper",
        namespace=LaunchConfiguration("namespace"),
        parameters=[
            LaunchConfiguration("config"),
            {"port": LaunchConfiguration("port"), "backend": LaunchConfiguration("backend")},
        ],
        output="screen",
    )
    return LaunchDescription([*args, node])
