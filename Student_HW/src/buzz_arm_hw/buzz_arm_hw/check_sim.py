"""Preflight for the homework simulation: is exactly one of everything running?

    ros2 run buzz_arm_hw check_sim

Run it in a second terminal after `ros2 launch buzz_arm_sim sim_moveit.launch.py`.
It reports:

    Gazebo servers      how many processes publish the world's statistics.
                        Must be 1. A second one is a leftover from an earlier
                        launch; your controllers may be living in that one
                        while the window shows this one.
    controller_manager  how many ROS nodes have that name. Must be 1, same
                        reason.
    controllers         the three controllers and their state. All active.
    move_group          present, and /move_action answering.
    block               where Gazebo says the block is. Flags it if it has
                        fallen over or left its spot.

Exit code 0 when everything is as expected, 1 otherwise.
"""

import re
import subprocess
import sys

import rclpy
from controller_manager_msgs.srv import ListControllers
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient
from rclpy.node import Node

BLOCK_XYZ = (0.39, 0.15, 0.075)


def gz(*args, timeout=6.0) -> str:
    try:
        return subprocess.run(["gz", *args], capture_output=True, text=True, timeout=timeout).stdout
    except FileNotFoundError:
        return ""
    except subprocess.TimeoutExpired:
        return ""


def gazebo_worlds() -> dict[str, int]:
    """World name -> number of servers publishing its /stats topic."""
    topics = gz("topic", "-l")
    worlds = sorted(set(re.findall(r"^/world/([^/]+)/stats$", topics, re.M)))
    counts = {}
    for world in worlds:
        info = gz("topic", "-i", "-t", f"/world/{world}/stats")
        # Only the Publishers block: the GUI window subscribes to this topic too.
        publishers = info.split("Subscribers")[0]
        counts[world] = publishers.count("gz.msgs.WorldStatistics")
    return counts


def block_pose():
    out = gz("model", "-m", "block", "--pose")
    triples = re.findall(r"\[\s*([-\d.e]+)\s+([-\d.e]+)\s+([-\d.e]+)\s*\]", out)
    if len(triples) < 2:
        return None
    xyz = tuple(float(v) for v in triples[0])
    rpy = tuple(float(v) for v in triples[1])
    return xyz, rpy


def main(argv=None):
    problems = []

    def report(ok: bool, text: str):
        print(("  OK   " if ok else "  !!   ") + text)
        if not ok:
            problems.append(text)

    print("Gazebo")
    worlds = gazebo_worlds()
    if not worlds:
        report(False, "no Gazebo world found (is the launch running? is `gz` on your PATH?)")
    for world, n in worlds.items():
        report(n == 1, f"world '{world}': {n} server(s) publishing" + (" (two Gazebos: see Troubleshooting)" if n > 1 else ""))

    pose = block_pose()
    if pose is None:
        report(False, "no model named 'block' in the world")
    else:
        (x, y, z), (roll, pitch, _) = pose
        upright = abs(roll) < 0.1 and abs(pitch) < 0.1
        on_spot = abs(x - BLOCK_XYZ[0]) < 0.02 and abs(y - BLOCK_XYZ[1]) < 0.02
        where = f"block at ({x:.3f}, {y:.3f}, {z:.3f})"
        if not upright:
            report(False, where + ", fallen over. `ros2 run buzz_arm_hw reset_block`")
        elif not on_spot:
            report(True, where + ", not at its start spot (fine if you moved it on purpose)")
        else:
            report(True, where + ", upright on its spot")

    print("ROS")
    rclpy.init()
    node = Node("check_sim")
    try:
        # give discovery a moment, or a fresh node sees an empty graph
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.1)
        names = [n for n, _ in node.get_node_names_and_namespaces()]
        n_cm = names.count("controller_manager")
        report(n_cm == 1, f"controller_manager nodes: {n_cm}" + (" (two Gazebos: see Troubleshooting)" if n_cm > 1 else ""))
        report("move_group" in names, "move_group node " + ("present" if "move_group" in names else "missing"))

        client = node.create_client(ListControllers, "/controller_manager/list_controllers")
        if client.wait_for_service(timeout_sec=3.0):
            future = client.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
            controllers = future.result().controller if future.result() else []
            for c in sorted(controllers, key=lambda c: c.name):
                report(c.state == "active", f"{c.name}: {c.state}")
            expected = {"joint_state_broadcaster", "buzz_arm_controller", "gripper_controller"}
            missing = expected - {c.name for c in controllers}
            if missing:
                report(False, f"controllers not loaded: {sorted(missing)}")
        else:
            report(False, "/controller_manager/list_controllers not answering")

        action = ActionClient(node, MoveGroup, "/move_action")
        report(action.wait_for_server(timeout_sec=3.0), "/move_action server " + ("answering" if action.server_is_ready() else "not found"))
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print()
    if problems:
        print(f"{len(problems)} problem(s). Fix them before running anything else.")
        sys.exit(1)
    print("All good.")


if __name__ == "__main__":
    main()
