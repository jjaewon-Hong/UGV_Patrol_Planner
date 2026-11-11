"""L4 PatrolControllerNode: ROS 2 interface around MissionManager (U5).

Subscribes operator commands, pose, obstacles and battery; converts them to unit calls;
publishes the returned outputs. All ROS 2 code of the controller lives here.
"""
import json

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped, Point
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float32, String

from ugv_patrol_planner.common import MissionState
from ugv_patrol_planner.mission.energy import EnergyMonitor
from ugv_patrol_planner.mission.manager import (Alarm, MissionManager, NextCell, PathUpdate, Status,
                                                parse_command)
from ugv_patrol_planner.mission.scheduler import PatrolScheduler
from ugv_patrol_planner.nodes.world import load_world, to_cell, to_point
from ugv_patrol_planner.planning.astar import AStarPlanner

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class PatrolControllerNode(Node):
    def __init__(self):
        super().__init__('patrol_controller')
        self.cfg, self.terrain, _ = load_world(self)
        self.planner = AStarPlanner(self.terrain)
        self.mission = MissionManager(self.terrain, self.planner, PatrolScheduler(),
                                      EnergyMonitor(self.terrain, self.cfg))
        self.pub_next = self.create_publisher(Point, '/controller/next_cell', 10)
        self.pub_path = self.create_publisher(PathMsg, '/controller/path', LATCHED)
        self.pub_status = self.create_publisher(String, '/mission/status', LATCHED)
        self.pub_alarm = self.create_publisher(String, '/mission/alarm', 10)
        self.create_subscription(String, '/operator/command', self.on_command_msg, 10)
        self.create_subscription(Point, '/ugv/pose', self.on_pose_msg, 10)
        self.create_subscription(PoseArray, '/perception/obstacles', self.on_obstacles_msg, 10)
        self.create_subscription(Float32, '/battery/level', self.on_battery_msg, 10)
        self.publish([self.mission.status()])
        self.get_logger().info(f'map {self.terrain.n}x{self.terrain.n} loaded, base {self.cfg.base_cell}')

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # -------------------------------------------------------------- inputs
    def on_command_msg(self, msg: String):
        try:
            cmd = parse_command(msg.data)
        except ValueError as e:  # malformed text: alarm, keep running (NFR-04)
            self.publish([Alarm(str(e))])
            return
        self.publish(self.mission.on_command(cmd, self.now()))

    def on_pose_msg(self, msg: Point):
        cell = to_cell(msg)
        if not self.terrain.in_bounds(cell):
            self.publish([Alarm(f'지도 밖 위치: {cell}')])
            return
        self.publish(self.mission.on_pose(cell, self.now()))

    def on_obstacles_msg(self, msg: PoseArray):
        cells = [to_cell(p.position) for p in msg.poses]
        outside = [c for c in cells if not self.terrain.in_bounds(c)]
        if outside:
            self.publish([Alarm(f'지도 밖 장애물 무시: {outside}')])
        outs = self.mission.on_obstacles(cells, self.now())
        if any(isinstance(o, Status) and o.state == MissionState.REPLANNING for o in outs):
            ms = self.planner.last_plan_ms
            limit = self.cfg.replan_time_limit_s * 1000
            log = self.get_logger().warn if ms > limit else self.get_logger().info
            log(f're-planned in {ms:.0f} ms (limit {limit:.0f} ms)')  # NFR-01 monitoring
        self.publish(outs)

    def on_battery_msg(self, msg: Float32):
        self.publish(self.mission.on_battery(float(msg.data)))

    # -------------------------------------------------------------- outputs
    def publish(self, outputs):
        for o in outputs:
            if isinstance(o, NextCell):
                self.pub_next.publish(to_point(o.cell))
            elif isinstance(o, PathUpdate):
                msg = PathMsg()
                msg.header.frame_id = 'grid'
                msg.header.stamp = self.get_clock().now().to_msg()
                for c in o.path:
                    ps = PoseStamped()
                    ps.header.frame_id = 'grid'
                    ps.pose.position = to_point(c)
                    msg.poses.append(ps)
                self.pub_path.publish(msg)
                goal = self.mission.target.pid if self.mission.target else '기지'
                self.get_logger().info(f'경로 갱신: {o.path[0]} -> {o.path[-1]} ({goal}, {len(o.path) - 1}칸)')
            elif isinstance(o, Status):
                self.pub_status.publish(String(data=json.dumps(o.info, ensure_ascii=False)))
                note = f" ({o.info['note']})" if 'note' in o.info else ''
                self.get_logger().info(f'임무 상태: {o.state.value}{note}')
            elif isinstance(o, Alarm):
                self.pub_alarm.publish(String(data=o.text))
                self.get_logger().warn(f'경보: {o.text}')


def main():
    rclpy.init()
    node = PatrolControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
