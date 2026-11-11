"""perception_sim: publishes only the hidden scenario obstacles within the sensor radius
of the UGV (FR-18)."""
import rclpy
from geometry_msgs.msg import Point, Pose, PoseArray
from rclpy.node import Node

from ugv_patrol_planner.nodes.world import load_world, to_cell, to_point


class PerceptionSim(Node):
    def __init__(self):
        super().__init__('perception_sim')
        cfg, _, scenario = load_world(self)
        self.radius = cfg.sensor_radius
        self.hidden = [tuple(c) for c in scenario.get('hidden_obstacles', [])]
        self.pub = self.create_publisher(PoseArray, '/perception/obstacles', 10)
        self.create_subscription(Point, '/ugv/pose', self.on_pose, 10)

    def on_pose(self, msg: Point):
        r, c = to_cell(msg)
        seen = [h for h in self.hidden if (h[0] - r) ** 2 + (h[1] - c) ** 2 <= self.radius ** 2]
        if seen:
            out = PoseArray()
            out.header.frame_id = 'grid'
            out.header.stamp = self.get_clock().now().to_msg()
            out.poses = [Pose(position=to_point(h)) for h in seen]
            self.pub.publish(out)


def main():
    rclpy.init()
    node = PerceptionSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
