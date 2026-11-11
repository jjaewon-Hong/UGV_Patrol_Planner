"""drive_sim: simple UGV drive model. Moves one cell toward the commanded neighbour per tick
and publishes the current cell at a fixed rate (FR-16)."""
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node

from ugv_patrol_planner.nodes.world import load_world, to_cell, to_point


class DriveSim(Node):
    def __init__(self):
        super().__init__('drive_sim')
        cfg, self.terrain, scenario = load_world(self)
        self.declare_parameter('rate_hz', 5.0)
        self.pos = tuple(scenario.get('start_cell', cfg.base_cell))
        self.pending = None
        self.pub = self.create_publisher(Point, '/ugv/pose', 10)
        self.create_subscription(Point, '/controller/next_cell', self.on_next_cell, 10)
        self.create_timer(1.0 / self.get_parameter('rate_hz').value, self.tick)

    def on_next_cell(self, msg: Point):
        cell = to_cell(msg)
        if abs(cell[0] - self.pos[0]) + abs(cell[1] - self.pos[1]) == 1:
            self.pending = cell
        elif cell != self.pos:
            self.get_logger().warn(f'ignoring non-adjacent move {self.pos} -> {cell}')

    def tick(self):
        if self.pending is not None:
            self.pos, self.pending = self.pending, None
        self.pub.publish(to_point(self.pos))


def main():
    rclpy.init()
    node = DriveSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
