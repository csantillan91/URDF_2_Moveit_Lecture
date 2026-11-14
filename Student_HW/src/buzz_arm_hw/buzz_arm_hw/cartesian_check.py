"""How much of a straight line can the gripper actually follow?

    ros2 run buzz_arm_hw cartesian_check pick_hover pick_grasp
    ros2 run buzz_arm_hw cartesian_check pick_hover place_hover

Takes two named poses A and B from your SRDF. Nothing moves. It asks
move_group two questions and prints both answers:

  1. Cartesian: "starting at A, move tool0 in a straight line to where it
     is at B" (service /compute_cartesian_path). MoveIt walks along the
     line in small steps and solves IK at each one. It stops at the first
     step where IK fails or the arm would be in collision, and reports the
     FRACTION of the line it managed.

  2. Joint space: "starting at A, plan to the joint values of B" (the
     ordinary planner, same as plan_benchmark). The joints move in a
     straight line in joint space; tool0 does not. We run tool0 through
     forward kinematics at every waypoint and print how far it strayed from
     the straight line.

The orientation of tool0 at B is used as the target orientation along the
whole line, so a Cartesian path is only possible if the arm can hold that
orientation at every point in between.
"""

import argparse
import math
import sys

import rclpy
from buzz_arm_planning_lab.plan_benchmark import ERROR_NAMES, Benchmark, make_goal_constraints, path_length
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetCartesianPath, GetPositionFK
from sensor_msgs.msg import JointState

TIP = "tool0"


def pos(p: Pose):
    return (p.position.x, p.position.y, p.position.z)


def dist_to_segment(p, a, b) -> float:
    ab = [b[i] - a[i] for i in range(3)]
    ap = [p[i] - a[i] for i in range(3)]
    l2 = sum(v * v for v in ab)
    t = 0.0 if l2 == 0 else max(0.0, min(1.0, sum(ap[i] * ab[i] for i in range(3)) / l2))
    closest = [a[i] + t * ab[i] for i in range(3)]
    return math.dist(p, closest)


class CartesianCheck(Benchmark):
    def __init__(self):
        super().__init__(name="cartesian_check")
        self.fk = self.create_client(GetPositionFK, "/compute_fk")
        self.cart = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        for client in (self.fk, self.cart):
            if not client.wait_for_service(timeout_sec=10.0):
                self.get_logger().error(f"{client.srv_name} not available. Is move_group running?")
                sys.exit(1)

    def state(self, joints: dict[str, float]) -> RobotState:
        rs = RobotState()
        rs.is_diff = True                     # joints we do not name keep their current values
        rs.joint_state = JointState(name=list(joints), position=[float(v) for v in joints.values()])
        return rs

    def fk_pose(self, joints: dict[str, float]) -> Pose:
        req = GetPositionFK.Request()
        req.header.frame_id = "base_link"
        req.fk_link_names = [TIP]
        req.robot_state = self.state(joints)
        future = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        res = future.result()
        if res is None or res.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error("FK failed")
            sys.exit(1)
        return res.pose_stamped[0].pose

    def cartesian(self, group, start: dict, target: Pose, step=0.005):
        req = GetCartesianPath.Request()
        req.header.frame_id = "base_link"
        req.group_name = group
        req.link_name = TIP
        req.start_state = self.state(start)
        req.waypoints = [target]
        req.max_step = step
        req.jump_threshold = 0.0              # do not reject large joint jumps; we want to see them
        req.avoid_collisions = True
        future = self.cart.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        return future.result()

    def joint_space(self, group, start: dict, goal: dict, planning_time=5.0):
        goal_msg = MoveGroup.Goal()
        req = goal_msg.request
        req.group_name = group
        req.pipeline_id = "ompl"
        req.planner_id = "RRTConnect"
        req.allowed_planning_time = planning_time
        req.num_planning_attempts = 1
        req.max_velocity_scaling_factor = 1.0
        req.max_acceleration_scaling_factor = 1.0
        req.start_state = self.state(start)
        req.goal_constraints = [make_goal_constraints(goal)]
        goal_msg.planning_options.plan_only = True
        send = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send, timeout_sec=10.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            return None
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=planning_time + 30.0)
        return result_future.result().result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("start", help="Named pose A (SRDF group_state).")
    parser.add_argument("goal", help="Named pose B.")
    parser.add_argument("--group", default="buzz_arm")
    parser.add_argument("--step", type=float, default=0.005, help="Cartesian step along the line, metres.")
    args = parser.parse_args(argv)

    rclpy.init()
    node = CartesianCheck()
    try:
        a = node.named_state(args.group, args.start)
        b = node.named_state(args.group, args.goal)
        pa, pb = node.fk_pose(a), node.fk_pose(b)
        line = math.dist(pos(pa), pos(pb))
        print(f"{TIP} at {args.start}: ({pa.position.x:.3f}, {pa.position.y:.3f}, {pa.position.z:.3f})")
        print(f"{TIP} at {args.goal}:  ({pb.position.x:.3f}, {pb.position.y:.3f}, {pb.position.z:.3f})")
        print(f"straight line: {line * 100:.1f} cm\n")

        print(f"1. Cartesian path {args.start} -> {args.goal} (step {args.step * 100:.1f} cm)")
        res = node.cartesian(args.group, a, pb, args.step)
        if res is None or res.error_code.val != MoveItErrorCodes.SUCCESS:
            code = res.error_code.val if res else None
            print(f"   service failed: {ERROR_NAMES.get(code, code)}")
        else:
            pts = res.solution.joint_trajectory.points
            print(f"   fraction achieved: {res.fraction:.2f}  ({res.fraction * line * 100:.1f} of {line * 100:.1f} cm, "
                  f"{len(pts)} waypoints, {path_length(res.solution):.3f} rad of joint motion)")
            if res.fraction < 0.999 and pts:
                names = res.solution.joint_trajectory.joint_names
                last = node.fk_pose(dict(zip(names, pts[-1].positions)))
                print(f"   stopped with {TIP} at ({last.position.x:.3f}, {last.position.y:.3f}, {last.position.z:.3f}): "
                      "no IK solution for the next step, or the arm would be in collision there "
                      "(move_group's terminal says which)")

        print(f"\n2. Joint-space plan {args.start} -> {args.goal} (RRTConnect)")
        plan = node.joint_space(args.group, a, b)
        if plan is None or plan.error_code.val != MoveItErrorCodes.SUCCESS:
            code = plan.error_code.val if plan else None
            print(f"   failed: {ERROR_NAMES.get(code, code)}")
        else:
            traj = plan.planned_trajectory
            names = traj.joint_trajectory.joint_names
            worst = 0.0
            for pt in traj.joint_trajectory.points:
                p = node.fk_pose(dict(zip(names, pt.positions)))
                worst = max(worst, dist_to_segment(pos(p), pos(pa), pos(pb)))
            print(f"   found: {len(traj.joint_trajectory.points)} waypoints, {path_length(traj):.3f} rad of joint motion")
            print(f"   {TIP} strays up to {worst * 100:.1f} cm from the straight line")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
