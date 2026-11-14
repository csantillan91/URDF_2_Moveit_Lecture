"""Everything in one process: robot_state_publisher, ros2_control, move_group, RViz.

    ros2 launch buzz_arm_moveit_config demo.launch.py
    ros2 launch buzz_arm_moveit_config demo.launch.py use_rviz:=false

What generate_demo_launch brings up, in order:
  - static_virtual_joint_tfs   world -> base_link static TF (from the SRDF virtual joint)
  - rsp.launch.py              resolves the xacro, publishes /robot_description + /tf
  - move_group.launch.py       MoveIt planner
  - moveit_rviz.launch.py      RViz with the MotionPlanning panel (use_rviz:=false to skip)
  - ros2_control_node          controller_manager; runs the mock hardware
  - spawn_controllers          joint_state_broadcaster, buzz_arm_controller, gripper_controller
"""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("buzz_arm", package_name="buzz_arm_moveit_config").to_moveit_configs()
    return generate_demo_launch(moveit_config)
