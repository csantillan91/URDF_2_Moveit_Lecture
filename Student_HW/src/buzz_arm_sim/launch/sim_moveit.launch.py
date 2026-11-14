"""Gazebo + MoveIt: the Part 1 demo with physics behind the controllers.

    ros2 launch buzz_arm_sim sim_moveit.launch.py                    # Gazebo window + RViz
    ros2 launch buzz_arm_sim sim_moveit.launch.py gui:=false         # RViz only, physics still runs
    ros2 launch buzz_arm_sim sim_moveit.launch.py gui:=false use_rviz:=false   # headless
    ros2 launch buzz_arm_sim sim_moveit.launch.py wall:=true         # homework section 4: wall in Gazebo AND in the planning scene

Same as buzz_arm_moveit_config/demo.launch.py except:
  - sim.launch.py replaces ros2_control_node + mock hardware (Gazebo owns
    controller_manager, see the plugin block in urdf/buzz_arm_sim.urdf.xacro)
  - move_group and RViz load the sim xacro as robot_description and run on
    simulation time (/clock from Gazebo)
  - no static world->base_link transform node: the sim xacro has a real
    `world` link and fixed joint, so robot_state_publisher publishes it
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    sim_share = Path(get_package_share_directory("buzz_arm_sim"))
    moveit_config = (
        MoveItConfigsBuilder("buzz_arm", package_name="buzz_arm_moveit_config")
        .robot_description(file_path=str(sim_share / "urdf" / "buzz_arm_sim.urdf.xacro"))
        .to_moveit_configs()
    )
    sim_time = {"use_sim_time": True}

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", description="Gazebo 3-D window."),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("world", default_value=str(sim_share / "worlds" / "table.sdf")),
        DeclareLaunchArgument("wall", default_value="false",
                              description="Homework wall: spawn it in Gazebo and add it to the planning scene."),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(sim_share / "launch" / "sim.launch.py")),
            launch_arguments={
                "gui": LaunchConfiguration("gui"),
                "world": LaunchConfiguration("world"),
                "wall": LaunchConfiguration("wall"),
            }.items(),
        ),

        # The wall has to exist twice: in Gazebo (sim.launch.py spawns
        # models/wall.sdf) and in MoveIt's planning scene, or the planner does
        # not know to avoid it and the arm drives into it. Same box, same pose,
        # described in config/wall_scene.yaml. Loaded once move_group is up.
        TimerAction(
            period=12.0,
            actions=[ExecuteProcess(
                cmd=["ros2", "run", "buzz_arm_planning_lab", "scene_objects", "load", "wall",
                     "--file", str(sim_share / "config" / "wall_scene.yaml")],
                output="screen",
            )],
            condition=IfCondition(LaunchConfiguration("wall")),
        ),

        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {
                    "publish_robot_description_semantic": True,
                    "allow_trajectory_execution": True,
                    "publish_planning_scene": True,
                    "publish_geometry_updates": True,
                    "publish_state_updates": True,
                    "publish_transforms_updates": True,
                    "monitor_dynamics": False,
                },
                sim_time,
            ],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            output="log",
            arguments=["-d", str(moveit_config.package_path / "config" / "moveit.rviz")],
            parameters=[
                moveit_config.planning_pipelines,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
                sim_time,
            ],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ])
