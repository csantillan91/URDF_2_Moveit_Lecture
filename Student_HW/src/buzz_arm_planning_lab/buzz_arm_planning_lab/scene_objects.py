"""Load or clear obstacle scenes in the MoveIt planning scene.

    ros2 run buzz_arm_planning_lab scene_objects load wall
    ros2 run buzz_arm_planning_lab scene_objects load wall --file my_scenes.yaml
    ros2 run buzz_arm_planning_lab scene_objects clear
    ros2 run buzz_arm_planning_lab scene_objects list

How it works. move_group keeps a "planning scene": the robot model plus a
world of collision objects. Anything in the world is an obstacle for the
planner. We add boxes to that world through the /apply_planning_scene
service, sending a PlanningScene message with is_diff=True so only the
objects we mention are changed. RViz's MotionPlanning display shows the
result, and the Scene Objects tab there is doing exactly the same thing by
hand.

Each box is a CollisionObject: an id, a frame, a primitive shape, and a
pose for that shape. Sending the same id again with operation REMOVE takes
it out.
"""

import argparse
import math
import sys
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


def rpy_to_quaternion(roll: float, pitch: float, yaw: float):
    """ZYX Euler angles to (x, y, z, w). Same convention as tf2."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def box_object(spec: dict, default_frame: str) -> CollisionObject:
    """Turn one YAML entry into a CollisionObject with operation ADD."""
    obj = CollisionObject()
    obj.id = str(spec["id"])
    obj.header.frame_id = spec.get("frame", default_frame)
    obj.operation = CollisionObject.ADD

    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [float(v) for v in spec["size"]]

    pose = Pose()
    x, y, z = spec["xyz"]
    pose.position.x, pose.position.y, pose.position.z = float(x), float(y), float(z)
    qx, qy, qz, qw = rpy_to_quaternion(*[float(v) for v in spec.get("rpy", [0, 0, 0])])
    pose.orientation.x, pose.orientation.y = qx, qy
    pose.orientation.z, pose.orientation.w = qz, qw

    obj.primitives.append(box)
    obj.primitive_poses.append(pose)
    return obj


class SceneClient(Node):
    def __init__(self):
        super().__init__("scene_objects")
        self.apply = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.get = self.create_client(GetPlanningScene, "/get_planning_scene")
        for client in (self.apply, self.get):
            if not client.wait_for_service(timeout_sec=10.0):
                self.get_logger().error(f"{client.srv_name} not available. Is move_group running?")
                sys.exit(1)

    def current_object_ids(self) -> list[str]:
        req = GetPlanningScene.Request()
        req.components.components = req.components.WORLD_OBJECT_NAMES
        future = self.get.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        return [o.id for o in future.result().scene.world.collision_objects]

    def apply_objects(self, objects: list[CollisionObject]) -> bool:
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = objects
        future = self.apply.call_async(ApplyPlanningScene.Request(scene=scene))
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        return bool(future.result() and future.result().success)

    def clear(self) -> list[str]:
        ids = self.current_object_ids()
        removals = []
        for obj_id in ids:
            obj = CollisionObject()
            obj.id = obj_id
            obj.operation = CollisionObject.REMOVE
            removals.append(obj)
        if removals:
            self.apply_objects(removals)
        return ids


def load_scene_file(path: Path) -> dict:
    with open(path) as f:
        data = yaml.safe_load(f)
    return data.get("scenes", {})


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("command", choices=["load", "clear", "list"])
    parser.add_argument("scene", nargs="?", help="Scene name (for load).")
    parser.add_argument("--file", help="Scene YAML. Defaults to the package's config/scenes.yaml.")
    parser.add_argument("--frame", default="base_link", help="Frame for boxes that do not name one.")
    parser.add_argument("--keep", action="store_true", help="Do not clear existing objects before loading.")
    args = parser.parse_args(argv)

    scene_file = Path(args.file) if args.file else Path(get_package_share_directory("buzz_arm_planning_lab")) / "config" / "scenes.yaml"
    scenes = load_scene_file(scene_file)

    if args.command == "list":
        for name, boxes in scenes.items():
            print(f"{name}: {', '.join(b['id'] for b in boxes)}")
        return

    rclpy.init()
    client = SceneClient()
    try:
        if args.command == "clear":
            removed = client.clear()
            print(f"Removed {len(removed)} object(s): {removed}")
            return

        if args.scene not in scenes:
            print(f"Unknown scene '{args.scene}'. Available: {', '.join(scenes)}")
            sys.exit(1)
        if not args.keep:
            client.clear()
        objects = [box_object(spec, args.frame) for spec in scenes[args.scene]]
        ok = client.apply_objects(objects)
        print(f"{'Loaded' if ok else 'FAILED to load'} scene '{args.scene}': {[o.id for o in objects]}")
        print(f"Planning scene now contains: {client.current_object_ids()}")
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
