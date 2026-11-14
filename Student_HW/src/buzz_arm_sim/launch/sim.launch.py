"""Gazebo physics server + the arm + its controllers. No MoveIt, no RViz.

    ros2 launch buzz_arm_sim sim.launch.py
    ros2 launch buzz_arm_sim sim.launch.py gui:=true
    ros2 launch buzz_arm_sim sim.launch.py wall:=true     # homework section 4

What comes up:
  - gz sim (server; add gui:=true for the 3-D window)
  - with wall:=true, models/wall.sdf spawned into the world
  - robot_state_publisher with the sim xacro
  - ros_gz_sim create: spawns the arm from /robot_description
  - ros_gz_bridge: /clock, so every ROS node can run on simulation time
  - controller spawners for joint_state_broadcaster, buzz_arm_controller,
    gripper_controller. controller_manager itself lives inside Gazebo,
    loaded by the plugin in the xacro.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_share = Path(get_package_share_directory("buzz_arm_sim"))
    gui = LaunchConfiguration("gui")
    world = LaunchConfiguration("world")
    wall = LaunchConfiguration("wall")

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            str(sim_share / "urdf" / "buzz_arm_sim.urdf.xacro"),
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false", description="Start the Gazebo 3-D window."),
        DeclareLaunchArgument("world", default_value=str(sim_share / "worlds" / "table.sdf")),
        DeclareLaunchArgument("wall", default_value="false", description="Also spawn the homework wall (models/wall.sdf)."),

        # Gazebo resolves package://buzz_arm_description/meshes/... by searching
        # these directories for a folder named buzz_arm_description.
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", str(Path(get_package_share_directory("buzz_arm_description")).parent)),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])),
            launch_arguments={
                # -r: start unpaused. -s: server only; the GUI is a second process below.
                "gz_args": ["-r -s ", world],
                "on_exit_shutdown": "true",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])),
            launch_arguments={"gz_args": "-g"}.items(),
            condition=IfCondition(gui),
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=["-topic", "robot_description", "-name", "buzz_arm", "-z", "0.0"],
            output="screen",
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            # `create` ignores the <pose> inside the file; the pose is given here.
            # Same numbers as the `wall` scene in buzz_arm_hw/config/scenes.yaml.
            arguments=["-file", str(sim_share / "models" / "wall.sdf"), "-name", "wall",
                       "-x", "0.284", "-y", "0.260", "-z", "0.111", "-Y", "1.0472"],
            output="screen",
            condition=IfCondition(wall),
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                # grasp commands (ROS -> Gazebo) and the plugin's state (Gazebo -> ROS)
                "/buzz_arm/attach@std_msgs/msg/Empty]gz.msgs.Empty",
                "/buzz_arm/detach@std_msgs/msg/Empty]gz.msgs.Empty",
                "/buzz_arm/attached@std_msgs/msg/String[gz.msgs.StringMsg",
            ],
        ),
        # The DetachableJoint plugin starts attached; release the cube once
        # everything is up so it rests on the table until the sequencer grasps it.
        TimerAction(
            period=8.0,
            actions=[ExecuteProcess(
                cmd=["gz", "topic", "-t", "/buzz_arm/detach", "-m", "gz.msgs.Empty", "-p", ""],
                output="screen",
            )],
        ),

        Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"]),
        Node(package="controller_manager", executable="spawner", arguments=["buzz_arm_controller"]),
        Node(package="controller_manager", executable="spawner", arguments=["gripper_controller"]),
    ])
