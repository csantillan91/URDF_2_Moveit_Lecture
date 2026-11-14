"""Pick up the block, carry it, set it down: the homework sequencer.

    ros2 run buzz_arm_hw pick_place
    ros2 run buzz_arm_hw pick_place --list            # print the steps and stop
    ros2 run buzz_arm_hw pick_place --start-at 7      # skip ahead (steps count from 1)

The sequence is the STEPS list below. Two kinds of step:

    "arm: NAME" / "gripper: NAME"
        Plan and execute a move to a named pose, a <group_state> in your
        SRDF, through the same /move_action server that RViz's Plan &
        Execute and plan_benchmark use. home, open and closed exist from
        Part 1; the rest are yours to add (HW.md section 2).

    "attach" / "detach"
        The grasp. Gazebo does not simulate friction well enough to hold a
        block between two fingers, so grasping is a command: one message
        on /buzz_arm/attach welds the block to the palm link wherever the
        block is at that instant; /buzz_arm/detach lets go.

MoveIt does not see the weld. planner_attach() and planner_detach() below
tell the planner that the arm is carrying a 4 x 4 x 15 cm box, so that it
plans around obstacles with the block and not just with the arm. They are
written for you; section 4 of HW.md asks you to call them.

Every move prints what plan_benchmark prints: path length in radians,
duration, and on failure the MoveItErrorCodes name. The reason for a
failure is in the move_group terminal.
"""

import argparse
import sys
import time

import rclpy
from buzz_arm_planning_lab.plan_benchmark import ERROR_NAMES, Benchmark, path_length, trajectory_duration
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, MoveItErrorCodes, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Empty

STEPS = [
    "arm: home",
    "gripper: open",
    "arm: pick_hover",
    "arm: pick_grasp",
    "gripper: grab",
    "attach",
    "arm: pick_hover",
    "arm: place_hover",
    "arm: place_grasp",
    "detach",
    "gripper: open",
    "arm: retreat",
    "arm: home",
]

GROUPS = {"arm": "buzz_arm", "gripper": "gripper"}

# The block as Gazebo spawns it (buzz_arm_sim/worlds/table.sdf): centre in
# base_link and full size, metres. planner_attach() assumes the block is
# still there when you grasp it.
BLOCK_ID = "block"
BLOCK_XYZ = (0.39, 0.15, 0.075)
BLOCK_SIZE = (0.04, 0.04, 0.15)

# The planner attaches the box to this link. Gazebo welds it to Link_5;
# tool0 is a fixed frame on Link_5, so for collision checking they are the
# same body. The fingers are allowed to touch the block without it counting
# as a collision.
GRASP_LINK = "tool0"
TOUCH_LINKS = ["Link_5", "Joint_RFinger", "Link_RFinger", "Joint_LFinger", "Link_LFinger"]


class PickPlace(Benchmark):
    """named_state(), wait_until_still() and plan_once() come from plan_benchmark."""

    def __init__(self):
        super().__init__(name="pick_place")
        self.attach_pub = self.create_publisher(Empty, "/buzz_arm/attach", 1)
        self.detach_pub = self.create_publisher(Empty, "/buzz_arm/detach", 1)
        self.scene = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        if not self.scene.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("/apply_planning_scene not available. Is move_group running?")
            sys.exit(1)

    # ---- moves -----------------------------------------------------------

    def move(self, group: str, pose_name: str, planner: str = "RRTConnect", planning_time: float = 5.0) -> int:
        """Plan to a named pose and execute it. Returns the MoveItErrorCodes value."""
        goal = self.named_state(group, pose_name)
        result = self.plan_once(group, planner, "ompl", goal, planning_time, 1, execute=True)
        if result is None:
            print("        rejected by move_group (no result)")
            return MoveItErrorCodes.FAILURE
        code = result.error_code.val
        if code == MoveItErrorCodes.SUCCESS:
            traj = result.planned_trajectory
            print(f"        ok  length={path_length(traj):.4f} rad  duration={trajectory_duration(traj):.2f} s  "
                  f"points={len(traj.joint_trajectory.points)}  plan={result.planning_time:.2f} s")
        else:
            print(f"        failed: {ERROR_NAMES.get(code, code)}  (the reason is in the move_group terminal)")
        return code

    # ---- the grasp, Gazebo side -------------------------------------------

    def _publish(self, pub):
        # The bridge to Gazebo subscribes a moment after we start; do not let
        # the first message fall on the floor.
        deadline = time.time() + 2.0
        while pub.get_subscription_count() == 0 and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        pub.publish(Empty())
        rclpy.spin_once(self, timeout_sec=0.2)

    def gazebo_attach(self):
        """Weld the block to Link_5, wherever the block is right now."""
        self._publish(self.attach_pub)
        print("        Gazebo: block welded to the palm")

    def gazebo_detach(self):
        """Release the weld; the block is back under gravity."""
        self._publish(self.detach_pub)
        print("        Gazebo: block released")

    # ---- the grasp, planner side --------------------------------------------

    def _apply(self, scene: PlanningScene) -> bool:
        scene.is_diff = True
        future = self.scene.call_async(ApplyPlanningScene.Request(scene=scene))
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        return bool(future.result() and future.result().success)

    def planner_attach(self):
        """Tell MoveIt the arm is carrying the block.

        An AttachedCollisionObject is a CollisionObject plus the link it
        rides on. From now on the planner moves the box with that link and
        checks it against the world like any other part of the robot. The
        box pose is given in base_link, where we know the block stands;
        MoveIt converts it into the tool0 frame at this instant, so attach
        *after* the fingers are around the block, not before.
        """
        attached = AttachedCollisionObject()
        attached.link_name = GRASP_LINK
        attached.touch_links = TOUCH_LINKS
        attached.object.id = BLOCK_ID
        attached.object.header.frame_id = "base_link"
        attached.object.operation = CollisionObject.ADD
        # The planner's copy is 1 cm shorter at the bottom. The real block
        # stands on the table, and a box that touches the robot's `floor`
        # link already counts as a collision, so the very first move after
        # attaching would be refused with START_STATE_IN_COLLISION.
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [BLOCK_SIZE[0], BLOCK_SIZE[1], BLOCK_SIZE[2] - 0.01]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = BLOCK_XYZ[0], BLOCK_XYZ[1], BLOCK_XYZ[2] + 0.005
        attached.object.primitives = [box]
        attached.object.primitive_poses = [pose]

        scene = PlanningScene()
        scene.robot_state.is_diff = True          # change only the attached objects, not the joint values
        scene.robot_state.attached_collision_objects = [attached]
        ok = self._apply(scene)
        print(f"        planner: block attached to {GRASP_LINK}" if ok else "        planner: attach FAILED")

    def planner_detach(self):
        """Tell MoveIt the block is no longer part of the arm.

        Detaching puts the box back into the world as an obstacle, exactly
        where the arm left it. We then remove it from the world too: the
        fingers are still touching it, and MoveIt refuses to plan from a
        start state that is in collision, so the "gripper: open" step right
        after this would fail with START_STATE_IN_COLLISION.
        """
        detach = AttachedCollisionObject()
        detach.object.id = BLOCK_ID
        detach.object.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [detach]
        ok = self._apply(scene)

        remove = CollisionObject()
        remove.id = BLOCK_ID
        remove.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.world.collision_objects = [remove]
        ok = self._apply(scene) and ok
        print("        planner: block detached and forgotten" if ok else "        planner: detach FAILED")


def run_step(node: PickPlace, step: str) -> int:
    if step == "attach":
        node.gazebo_attach()
        return MoveItErrorCodes.SUCCESS
    if step == "detach":
        node.gazebo_detach()
        return MoveItErrorCodes.SUCCESS
    kind, pose_name = (s.strip() for s in step.split(":", 1))
    return node.move(GROUPS[kind], pose_name)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--list", action="store_true", help="Print the steps and exit.")
    parser.add_argument("--start-at", type=int, default=1, metavar="N", help="Begin at step N (1-based).")
    args = parser.parse_args(argv)

    if args.list:
        for i, step in enumerate(STEPS, 1):
            print(f"{i:2d}  {step}")
        return

    rclpy.init()
    node = PickPlace()
    try:
        for i, step in enumerate(STEPS, 1):
            if i < args.start_at:
                continue
            print(f"[{i:2d}/{len(STEPS)}] {step}")
            run_step(node, step)
        print("done")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
