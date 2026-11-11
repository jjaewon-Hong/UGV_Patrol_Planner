"""battery_sim: drains the battery by distance and climbing on every move (FR-19)
using the same energy model as EnergyMonitor (U4)."""
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Float32

from ugv_patrol_planner.mission.energy import energy_step
from ugv_patrol_planner.nodes.world import load_world, to_cell


class BatterySim(Node):
    def __init__(self):
        super().__init__('battery_sim')
        self.cfg, self.terrain, scenario = load_world(self)
        self.level = float(scenario.get('initial_battery_wh', self.cfg.capacity_wh))
        self.prev = None
        self.empty_reported = False
        self.pub = self.create_publisher(Float32, '/battery/level', 10)
        self.create_subscription(Point, '/ugv/pose', self.on_pose, 10)
        self.create_timer(0.5, self.publish)

    def on_pose(self, msg: Point):
        cell = to_cell(msg)
        if self.prev is not None and cell != self.prev:
            used = energy_step(self.terrain, self.prev, cell, self.cfg.wh_per_m, self.cfg.wh_per_climb_m)
            self.level = max(0.0, self.level - used)
            self.publish()
            if self.level == 0.0 and cell != tuple(self.cfg.base_cell) and not self.empty_reported:
                self.get_logger().error(f'battery empty outside base at {cell}')  # NFR-03 violation
                self.empty_reported = True
        self.prev = cell

    def publish(self):
        self.pub.publish(Float32(data=self.level))


def main():
    rclpy.init()
    node = BatterySim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
