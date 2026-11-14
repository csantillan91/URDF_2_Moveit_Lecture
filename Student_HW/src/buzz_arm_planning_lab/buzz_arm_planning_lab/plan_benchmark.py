"""Benchmark OMPL planners through move_group.

    ros2 run buzz_arm_planning_lab plan_benchmark --goal reach_low
    ros2 run buzz_arm_planning_lab plan_benchmark --goal reach_low \
        --planners RRTConnect RRT RRTstar PRM --trials 5 --time 2 --label bar --out results.csv
    ros2 run buzz_arm_planning_lab plan_benchmark --goal home --execute

The goal is a named pose from your SRDF: `home` and `reach_low` are the two
the lab uses, and the arm should be at `home` when you start. For each
planner and each trial this sends one MotionPlanRequest to the
/move_action action server (the same thing RViz's Plan button does), asks
for a plan only, and records:

    success        did the planner return a trajectory
    planning_time  seconds move_group reports it spent planning
    n_points       waypoints in the returned trajectory
    duration       seconds the trajectory would take to execute, after
                   time parameterization (last point's time_from_start)
    path_length    joint-space length of the path in radians

A named pose is a <group_state> in your SRDF, looked up by reading
move_group's robot_description_semantic parameter. --joints takes explicit
joint values instead. Start state is wherever the robot is now.

With --execute the script waits for the arm to stop moving before each
plan: move_group rejects a trajectory whose start is more than 0.01 rad from
the robot's current state, and a real or simulated arm takes a moment to
settle after every move. If the arm never settles you get a warning naming
the joint that is still moving.
"""

import argparse
import csv
import math
import statistics
import sys
import xml.etree.ElementTree as ET

import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, RobotTrajectory
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

# -2 -> "INVALID_MOTION_PLAN", -4 -> "CONTROL_FAILED", ... for readable failure lines.
ERROR_NAMES = {v: k for k, v in vars(MoveItErrorCodes).items() if k.isupper() and isinstance(v, int)}

def make_goal_constraints(joint_targets: dict[str, float], tolerance: float = 0.01) -> Constraints:
    """A joint-space goal: one JointConstraint per joint.

    A MotionPlanRequest describes the goal as a list of Constraints. Each
    JointConstraint says: this joint must end within
    [position - tolerance_below, position + tolerance_above].
    This is what RViz sends when you pick a named pose as the goal state.
    """
    constraints = Constraints()
    for name, value in joint_targets.items():
        constraints.joint_constraints.append(JointConstraint(
            joint_name=name,
            position=float(value),
            tolerance_above=tolerance,
            tolerance_below=tolerance,
            weight=1.0,
        ))
    return constraints


def path_length(trajectory: RobotTrajectory) -> float:
    """Joint-space length of the path in radians: sum of the distances between consecutive waypoints.

    A straight line between two configurations has the same length as the
    distance between them; a path that detours around an obstacle is longer.
    (This was the Part 2 exercise.)
    """
    pts = trajectory.joint_trajectory.points
    return sum(math.dist(a.positions, b.positions) for a, b in zip(pts, pts[1:]))



def trajectory_duration(trajectory: RobotTrajectory) -> float:
    points = trajectory.joint_trajectory.points
    if not points:
        return 0.0
    t = points[-1].time_from_start
    return t.sec + t.nanosec * 1e-9


class Benchmark(Node):
    def __init__(self, name: str = "plan_benchmark"):
        super().__init__(name)
        self.client = ActionClient(self, MoveGroup, "/move_action")
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("/move_action not available. Is move_group running?")
            sys.exit(1)

    def named_state(self, group: str, name: str) -> dict[str, float]:
        """Look up a <group_state> in the SRDF that move_group loaded."""
        params = self.create_client(GetParameters, "/move_group/get_parameters")
        if not params.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("Cannot reach /move_group parameters.")
            sys.exit(1)
        future = params.call_async(GetParameters.Request(names=["robot_description_semantic"]))
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        srdf = future.result().values[0].string_value
        root = ET.fromstring(srdf)
        for state in root.iter("group_state"):
            if state.get("group") == group and state.get("name") == name:
                return {j.get("name"): float(j.get("value")) for j in state.iter("joint")}
        available = [s.get("name") for s in root.iter("group_state") if s.get("group") == group]
        self.get_logger().error(f"No group_state '{name}' for group '{group}'. Available: {available}")
        sys.exit(1)

    def wait_until_still(self, vel_tol=0.01, hold=0.3, timeout=3.0, ignore=("Mimic_LFinger",)):
        """Block until every joint has reported |velocity| < vel_tol for `hold` seconds.

        move_group refuses to execute a trajectory whose first point is more
        than 0.01 rad from where the robot is right now. On real or simulated
        hardware the arm is still settling for a moment after each move, so
        planning again immediately fails with "start point deviates from
        current robot state". Returns (settled, seconds_waited, worst_joint, worst_vel).

        Joints in `ignore` are not watched: in Gazebo the mimic finger is
        moved by the physics engine, not by a controller, and twitches.
        """
        if not hasattr(self, "_joint_state"):
            self._joint_state = None
            self.create_subscription(JointState, "/joint_states", lambda m: setattr(self, "_joint_state", m), 10)
        start = self.get_clock().now()
        still_since = None
        worst = ("", 0.0)
        while (self.get_clock().now() - start).nanoseconds * 1e-9 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            js = self._joint_state
            if js is None or not js.velocity:
                continue
            watched = [k for k in range(len(js.velocity)) if js.name[k] not in ignore]
            i = max(watched, key=lambda k: abs(js.velocity[k]))
            worst = (js.name[i], abs(js.velocity[i]))
            now = self.get_clock().now()
            if worst[1] < vel_tol:
                still_since = still_since or now
                if (now - still_since).nanoseconds * 1e-9 >= hold:
                    return True, (now - start).nanoseconds * 1e-9 - hold, *worst
            else:
                still_since = None
        return False, timeout, *worst

    def plan_once(self, group, planner, pipeline, goal, planning_time, attempts, execute):
        goal_msg = MoveGroup.Goal()
        req = goal_msg.request
        req.group_name = group
        req.pipeline_id = pipeline
        req.planner_id = planner
        req.allowed_planning_time = planning_time
        req.num_planning_attempts = attempts
        req.max_velocity_scaling_factor = 1.0
        req.max_acceleration_scaling_factor = 1.0
        # Plan from where the robot is now. (When executing, move_group always
        # starts from the current state and warns if you tell it anything else.)
        req.start_state.is_diff = not execute
        req.goal_constraints = [make_goal_constraints(goal)]
        goal_msg.planning_options.plan_only = not execute

        send = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send, timeout_sec=10.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            return None
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=planning_time + 60.0)
        return result_future.result().result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--group", default="buzz_arm", help="Planning group name from your SRDF.")
    parser.add_argument("--goal", help="Named pose (SRDF group_state) to plan to, e.g. reach_low or home.")
    parser.add_argument("--joints", nargs="+", metavar="JOINT=VALUE", help="Explicit joint goal instead of --goal.")
    parser.add_argument("--planners", nargs="+", default=["RRTConnect"], help="Planner ids from ompl_planning.yaml.")
    parser.add_argument("--pipeline", default="ompl")
    parser.add_argument("--trials", type=int, default=5)
    parser.add_argument("--time", type=float, default=5.0, help="allowed_planning_time in seconds.")
    parser.add_argument("--attempts", type=int, default=1, help="num_planning_attempts.")
    parser.add_argument("--label", default="", help="Free text written to every CSV row, e.g. the scene name.")
    parser.add_argument("--out", help="Append rows to this CSV file.")
    parser.add_argument("--execute", action="store_true", help="Execute each plan instead of plan-only.")
    args = parser.parse_args(argv)

    if not args.goal and not args.joints:
        parser.error("give --goal NAME or --joints JOINT=VALUE ...")

    rclpy.init()
    node = Benchmark()
    try:
        if args.joints:
            goal = {kv.split("=")[0]: float(kv.split("=")[1]) for kv in args.joints}
        else:
            goal = node.named_state(args.group, args.goal)
        node.get_logger().info(f"Goal: {goal}")

        rows = []
        for planner in args.planners:
            for trial in range(1, args.trials + 1):
                if args.execute:
                    settled, waited, joint, vel = node.wait_until_still()
                    if not settled:
                        node.get_logger().warn(f"arm still moving after {waited:.1f} s: {joint} at {vel:.3f} rad/s. "
                                               "Planning anyway; expect 'start point deviates'.")
                    elif waited > 0.1:
                        node.get_logger().info(f"waited {waited:.2f} s for the arm to settle ({joint} was last to stop)")
                result = node.plan_once(args.group, planner, args.pipeline, goal,
                                        args.time, args.attempts, args.execute)
                ok = result is not None and result.error_code.val == MoveItErrorCodes.SUCCESS
                traj = result.planned_trajectory if ok else RobotTrajectory()
                row = {
                    "label": args.label,
                    "planner": planner,
                    "trial": trial,
                    "success": int(ok),
                    "error_code": result.error_code.val if result else "no_result",
                    "planning_time": round(result.planning_time, 4) if result else "",
                    "n_points": len(traj.joint_trajectory.points),
                    "duration": round(trajectory_duration(traj), 3) if ok else "",
                    "path_length": round(path_length(traj), 4) if ok else "",
                }
                rows.append(row)
                if ok:
                    print(f"{planner:12s} trial {trial:2d}  success=1  "
                          f"plan={row['planning_time']}s  points={row['n_points']}  "
                          f"duration={row['duration']}s  length={row['path_length']} rad")
                else:
                    code = result.error_code.val if result else None
                    print(f"{planner:12s} trial {trial:2d}  success=0  "
                          f"plan={row['planning_time']}s  failed: {ERROR_NAMES.get(code, code)}  "
                          "(the reason is in the move_group terminal)")

        print("\nSummary (successful trials only):")
        for planner in args.planners:
            good = [r for r in rows if r["planner"] == planner and r["success"]]
            n = sum(1 for r in rows if r["planner"] == planner)
            if not good:
                print(f"  {planner:12s} 0/{n} succeeded")
                continue
            times = [r["planning_time"] for r in good]
            lengths = [r["path_length"] for r in good]
            sd = statistics.stdev(times) if len(times) > 1 else 0.0
            print(f"  {planner:12s} {len(good)}/{n} succeeded  "
                  f"planning {statistics.mean(times):.3f} +/- {sd:.3f} s  "
                  f"path length {statistics.mean(lengths):.3f} rad  "
                  f"duration {statistics.mean(r['duration'] for r in good):.2f} s")

        if args.out:
            new_file = True
            try:
                new_file = open(args.out).read(1) == ""
            except FileNotFoundError:
                pass
            with open(args.out, "a", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
                if new_file:
                    writer.writeheader()
                writer.writerows(rows)
            print(f"\nAppended {len(rows)} rows to {args.out}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
