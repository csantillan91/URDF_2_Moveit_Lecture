"""Show the URDF in RViz with a slider per joint.

    ros2 launch buzz_arm_description display.launch.py
    ros2 launch buzz_arm_description display.launch.py gui:=false   # no RViz, no sliders

Three nodes:
  robot_state_publisher      reads the URDF, publishes /robot_description and
                             the /tf transforms for every link from /joint_states
  joint_state_publisher_gui  one slider per non-fixed joint, publishes /joint_states
                             (gui:=false swaps in joint_state_publisher, which
                             publishes every joint at zero)
  rviz2                      RobotModel + TF displays, fixed frame base_link
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("buzz_arm_description"))
    urdf_text = (share / "urdf" / "buzz_arm.urdf").read_text()

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true",
                              description="Launch RViz and the joint slider window."),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": urdf_text}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            condition=IfCondition(LaunchConfiguration("gui")),
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            condition=UnlessCondition(LaunchConfiguration("gui")),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(share / "rviz" / "display.rviz")],
            condition=IfCondition(LaunchConfiguration("gui")),
        ),
    ])
