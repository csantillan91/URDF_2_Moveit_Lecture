"""Put the block back on its start spot.

    ros2 run buzz_arm_hw reset_block

Three things, in order:
  1. /buzz_arm/detach, in case the block is still welded to the arm.
  2. Remove any 'block' from the MoveIt planning scene, attached or not.
  3. Ask Gazebo to teleport the model to its spawn pose (world service
     /world/<world>/set_pose). It lands upright on the table.

The arm is left where it is. `ros2 run buzz_arm_hw pick_place --start-at 1`
begins with a move to home anyway.
"""

import argparse
import subprocess
import sys
import time

import rclpy
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.node import Node
from std_msgs.msg import Empty

BLOCK_XYZ = (0.39, 0.15, 0.075)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--world", default="buzz_world", help="Gazebo world name (see <world name=...> in the SDF).")
    parser.add_argument("--model", default="block")
    args = parser.parse_args(argv)

    rclpy.init()
    node = Node("reset_block")
    try:
        detach = node.create_publisher(Empty, "/buzz_arm/detach", 1)
        deadline = time.time() + 2.0
        while detach.get_subscription_count() == 0 and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if detach.get_subscription_count():
            detach.publish(Empty())
            rclpy.spin_once(node, timeout_sec=0.2)
            print("detach sent")
        else:
            print("no one is listening on /buzz_arm/detach (bridge not running?)")

        scene_srv = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        get_srv = node.create_client(GetPlanningScene, "/get_planning_scene")
        if scene_srv.wait_for_service(timeout_sec=3.0) and get_srv.wait_for_service(timeout_sec=3.0):
            req = GetPlanningScene.Request()
            req.components.components = req.components.WORLD_OBJECT_NAMES | req.components.ROBOT_STATE_ATTACHED_OBJECTS
            future = get_srv.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
            current = future.result().scene if future.result() else None
            attached = current and any(a.object.id == args.model for a in current.robot_state.attached_collision_objects)
            in_world = current and any(o.id == args.model for o in current.world.collision_objects)

            if attached:
                detached = AttachedCollisionObject()
                detached.object.id = args.model
                detached.object.operation = CollisionObject.REMOVE
                scene = PlanningScene(is_diff=True)
                scene.robot_state.is_diff = True
                scene.robot_state.attached_collision_objects = [detached]
                future = scene_srv.call_async(ApplyPlanningScene.Request(scene=scene))
                rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
            if attached or in_world:
                removed = CollisionObject()
                removed.id = args.model
                removed.operation = CollisionObject.REMOVE
                scene = PlanningScene(is_diff=True)
                scene.world.collision_objects = [removed]
                future = scene_srv.call_async(ApplyPlanningScene.Request(scene=scene))
                rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
            print("planning scene: block removed" if (attached or in_world) else "planning scene: had no block")
        else:
            print("move_group not running; planning scene left alone")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    x, y, z = BLOCK_XYZ
    req = f'name: "{args.model}", position: {{x: {x}, y: {y}, z: {z + 0.005}}}, orientation: {{w: 1}}'
    cmd = ["gz", "service", "-s", f"/world/{args.world}/set_pose",
           "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", req]
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=8.0)
    except FileNotFoundError:
        print("`gz` not found on PATH")
        sys.exit(1)
    except subprocess.TimeoutExpired:
        print(f"Gazebo did not answer on /world/{args.world}/set_pose. Is the simulation running?")
        sys.exit(1)
    if "data: true" in out.stdout:
        print(f"Gazebo: {args.model} back at ({x}, {y}, {z})")
    else:
        print(f"Gazebo refused: {out.stdout.strip()} {out.stderr.strip()}")
        sys.exit(1)


if __name__ == "__main__":
    main()
