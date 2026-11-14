"""Which way can the gripper point from a given spot? Ask the IK solver.

    ros2 run buzz_arm_hw ik_check pick_grasp
    ros2 run buzz_arm_hw ik_check pick_grasp --no-collisions
    ros2 run buzz_arm_hw ik_check --xyz 0.30 0.10 0.20

Takes a position for tool0 (from a named SRDF pose through forward
kinematics, or given directly) and asks /compute_ik for the same position
with the gripper pointing in several directions:

    radial out        away from the arm's base, horizontally
    radial, N down/up the same, tilted N degrees towards the table or the ceiling
    down / up         straight down at the table / straight up
    radial in         towards the base
    tangent           sideways, 90 degrees from radial, horizontally
    45 off radial     halfway between radial and tangent

For each direction it tries eight different rolls of the gripper about its
own pointing axis, and reports whether ANY of them has a solution. Nothing
moves. This is the same solver RViz's interactive marker uses, so a
direction that fails here is one the marker would refuse too.

A pose in space has six numbers (three position, three orientation). This
arm has five joints. Look at which directions succeed and which fail, and
ask what the missing sixth number is.
"""

import argparse
import math
import sys

import rclpy
from buzz_arm_planning_lab.plan_benchmark import ERROR_NAMES, Benchmark
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from sensor_msgs.msg import JointState

TIP = "tool0"
BASE_AXIS_XY = (0.13407, 0.0)       # where Revolute_1's axis meets the table (URDF origin of Revolute_1)


def normalize(v):
    n = math.sqrt(sum(c * c for c in v))
    return tuple(c / n for c in v)


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def quaternion_from_axes(x, y, z):
    """Rotation matrix with columns x, y, z -> quaternion (qx, qy, qz, qw)."""
    m = [[x[0], y[0], z[0]], [x[1], y[1], z[1]], [x[2], y[2], z[2]]]
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return ((m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, 0.25 * s)
    i = max(range(3), key=lambda k: m[k][k])
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + m[i][i] - m[j][j] - m[k][k]) * 2
    q = [0.0, 0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[j] = (m[j][i] + m[i][j]) / s
    q[k] = (m[k][i] + m[i][k]) / s
    q[3] = (m[k][j] - m[j][k]) / s
    return tuple(q)


def orientation_pointing(z, roll):
    """Quaternion whose Z axis is `z`, rolled by `roll` radians about it."""
    z = normalize(z)
    helper = (0.0, 0.0, 1.0) if abs(z[2]) < 0.9 else (1.0, 0.0, 0.0)
    x0 = normalize(cross(helper, z))
    y0 = cross(z, x0)
    c, s = math.cos(roll), math.sin(roll)
    x = tuple(c * x0[i] + s * y0[i] for i in range(3))
    y = cross(z, x)
    return quaternion_from_axes(x, y, z)


class IKCheck(Benchmark):
    def __init__(self):
        super().__init__(name="ik_check")
        self.fk = self.create_client(GetPositionFK, "/compute_fk")
        self.ik = self.create_client(GetPositionIK, "/compute_ik")
        for client in (self.fk, self.ik):
            if not client.wait_for_service(timeout_sec=10.0):
                self.get_logger().error(f"{client.srv_name} not available. Is move_group running?")
                sys.exit(1)

    def fk_position(self, joints: dict[str, float]):
        req = GetPositionFK.Request()
        req.header.frame_id = "base_link"
        req.fk_link_names = [TIP]
        req.robot_state.joint_state = JointState(name=list(joints), position=[float(v) for v in joints.values()])
        future = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        p = future.result().pose_stamped[0].pose.position
        return (p.x, p.y, p.z)

    def solve(self, group, xyz, quat, seed: dict[str, float], avoid_collisions: bool):
        req = GetPositionIK.Request()
        q = req.ik_request
        q.group_name = group
        q.ik_link_name = TIP
        q.avoid_collisions = avoid_collisions
        q.timeout.nanosec = 200_000_000
        q.robot_state.joint_state = JointState(name=list(seed), position=[float(v) for v in seed.values()])
        ps = PoseStamped()
        ps.header.frame_id = "base_link"
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = xyz
        ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = quat
        q.pose_stamped = ps
        future = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        res = future.result()
        if res is None:
            return MoveItErrorCodes.FAILURE, None
        if res.error_code.val == MoveItErrorCodes.SUCCESS:
            js = res.solution.joint_state
            return res.error_code.val, {n: round(p, 3) for n, p in zip(js.name, js.position) if n in seed}
        return res.error_code.val, None


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("pose", nargs="?", help="Named SRDF pose whose tool0 position to use.")
    parser.add_argument("--xyz", nargs=3, type=float, metavar=("X", "Y", "Z"), help="tool0 position in base_link instead.")
    parser.add_argument("--group", default="buzz_arm")
    parser.add_argument("--no-collisions", action="store_true", help="Ignore collisions: pure reachability.")
    parser.add_argument("--rolls", type=int, default=8, help="How many rolls about the pointing axis to try.")
    args = parser.parse_args(argv)
    if not args.pose and not args.xyz:
        parser.error("give a named pose or --xyz X Y Z")

    rclpy.init()
    node = IKCheck()
    try:
        seed = node.named_state(args.group, "home")
        if args.pose:
            joints = node.named_state(args.group, args.pose)
            xyz = node.fk_position(joints)
            seed = joints
        else:
            xyz = tuple(args.xyz)

        dx, dy = xyz[0] - BASE_AXIS_XY[0], xyz[1] - BASE_AXIS_XY[1]
        radial = normalize((dx, dy, 0.0))
        tangent = (-radial[1], radial[0], 0.0)
        r = math.hypot(dx, dy)
        print(f"{TIP} position ({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f}) in base_link: "
              f"{r * 100:.1f} cm out from the base axis, azimuth {math.degrees(math.atan2(dy, dx)):.0f} deg")
        print(f"collision checking {'OFF' if args.no_collisions else 'ON'}; {args.rolls} rolls per direction\n")

        def tilted(deg):
            """Radial-out, tilted `deg` degrees down (negative: up) from horizontal."""
            a = math.radians(deg)
            return normalize((radial[0] * math.cos(a), radial[1] * math.cos(a), -math.sin(a)))

        directions = [
            ("radial out", radial),
            ("radial, 20 down", tilted(20)),
            ("radial, 45 down", tilted(45)),
            ("radial, 20 up", tilted(-20)),
            ("down", (0.0, 0.0, -1.0)),
            ("up", (0.0, 0.0, 1.0)),
            ("radial in", tuple(-c for c in radial)),
            ("tangent", tangent),
            ("45 off radial", normalize(tuple(radial[i] + tangent[i] for i in range(3)))),
        ]
        print(f"{'direction':16s} {'result':10s} example joint values")
        for label, z in directions:
            found = None
            last_code = None
            for k in range(args.rolls):
                roll = 2 * math.pi * k / args.rolls
                code, sol = node.solve(args.group, xyz, orientation_pointing(z, roll), seed, not args.no_collisions)
                last_code = code
                if sol:
                    found = sol
                    break
            if found:
                print(f"{label:16s} {'OK':10s} {found}")
            else:
                print(f"{label:16s} {'no':10s} {ERROR_NAMES.get(last_code, last_code)} for every roll")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
