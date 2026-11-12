"""visualizer (View): 2D OpenCV map and RViz2 markers (FR-17).

Shows elevation as grey shading, obstacles, planned path, patrol points, base and UGV.
Parameters: gui (open an OpenCV window), snapshot (save the image to this file every second).
"""
import json

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseArray
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import ColorRGBA, Float32, String
from visualization_msgs.msg import Marker, MarkerArray

from ugv_patrol_planner.nodes.world import load_world, to_cell

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
SCALE = 6  # screen pixels per cell
# BGR colours
RED, BLUE, GREEN, YELLOW, CYAN, WHITE = (40, 40, 220), (220, 120, 30), (60, 200, 60), (0, 210, 255), (200, 200, 0), (255, 255, 255)


class Visualizer(Node):
    def __init__(self):
        super().__init__('visualizer')
        self.cfg, self.terrain, _ = load_world(self)
        self.declare_parameter('gui', True)
        self.declare_parameter('snapshot', '')
        self.gui = bool(self.get_parameter('gui').value)
        self.snapshot = str(self.get_parameter('snapshot').value)
        self.n = self.terrain.n
        self.elev = np.array([[self.terrain.elevation((r, c)) for c in range(self.n)] for r in range(self.n)])
        self.obstacles = {(r, c) for r in range(self.n) for c in range(self.n) if self.terrain.is_obstacle((r, c))}
        self.path: list = []
        self.pos = None
        self.status: dict = {}
        self.battery = None
        self.base_img = self._terrain_image()

        self.create_subscription(Point, '/ugv/pose', self.on_pose, 10)
        self.create_subscription(PoseArray, '/perception/obstacles', self.on_obstacles, 10)
        self.create_subscription(PathMsg, '/controller/path', self.on_path, LATCHED)
        self.create_subscription(String, '/mission/status', self.on_status, LATCHED)
        self.create_subscription(Float32, '/battery/level', self.on_battery, 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/viz/markers', LATCHED)
        self.create_timer(0.1, self.draw)
        self.create_timer(1.0, self.publish_markers)
        if self.snapshot:
            self.create_timer(1.0, self.save_snapshot)

    # -------------------------------------------------------------- inputs
    def on_pose(self, msg):
        self.pos = to_cell(msg)

    def on_obstacles(self, msg):
        self.obstacles.update(to_cell(p.position) for p in msg.poses)

    def on_path(self, msg):
        self.path = [to_cell(ps.pose.position) for ps in msg.poses]

    def on_battery(self, msg):
        self.battery = float(msg.data)

    def on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
        except ValueError:
            pass

    # -------------------------------------------------------------- OpenCV view
    def _terrain_image(self) -> np.ndarray:
        e = self.elev
        grey = ((e - e.min()) / max(np.ptp(e), 1e-9) * 180 + 50).astype(np.uint8)
        img = cv2.cvtColor(grey, cv2.COLOR_GRAY2BGR)
        return cv2.resize(img, (self.n * SCALE, self.n * SCALE), interpolation=cv2.INTER_NEAREST)

    def _px(self, cell):
        return cell[1] * SCALE + SCALE // 2, cell[0] * SCALE + SCALE // 2

    def render(self) -> np.ndarray:
        img = self.base_img.copy()
        for r, c in self.obstacles:
            cv2.rectangle(img, (c * SCALE, r * SCALE), ((c + 1) * SCALE - 1, (r + 1) * SCALE - 1), RED, -1)
        if len(self.path) > 1:
            pts = np.array([self._px(c) for c in self.path], dtype=np.int32)
            cv2.polylines(img, [pts], False, BLUE, 2)
        bx, by = self._px(tuple(self.cfg.base_cell))
        cv2.rectangle(img, (max(bx - 6, 0), max(by - 6, 0)), (bx + 6, by + 6), CYAN, 2)  # base, kept on screen
        for p in self.status.get('points', []):
            x, y = self._px(tuple(p['cell']))
            self._flag(img, x, y, p['pid'] == self.status.get('target'))
            cv2.putText(img, p['pid'], (x + 12, y - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, YELLOW, 1)
        if self.pos is not None:
            cv2.circle(img, self._px(self.pos), 5, GREEN, -1)
        # Status bar above the map (not on top of it), so row 0 stays visible.
        state = self.status.get('state', '-')
        # OpenCV fonts cannot draw Korean; show the English state name.
        names = {'대기': 'IDLE', '순찰 중': 'PATROLLING', '재계획 중': 'REPLANNING', '복귀 중': 'RETURNING', '정지': 'STOPPED'}
        battery = '-' if self.battery is None else f'{self.battery:.1f} Wh'
        bar = np.full((24, img.shape[1], 3), 30, dtype=np.uint8)
        cv2.putText(bar, f"state: {names.get(state, state)}   target: {self.status.get('target') or '-'}"
                    f"   battery: {battery}", (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, WHITE, 1)
        return np.vstack([bar, img])

    @staticmethod
    def _flag(img, x, y, active):
        """Patrol point as a small flag: pole from the cell centre, pennant at the top."""
        cv2.line(img, (x, y), (x, y - 16), WHITE, 2)
        pennant = np.array([[x + 1, y - 16], [x + 12, y - 12], [x + 1, y - 8]], dtype=np.int32)
        cv2.fillPoly(img, [pennant], YELLOW)
        cv2.polylines(img, [pennant], True, RED if active else (0, 0, 0), 2 if active else 1)  # current target

    def draw(self):
        if self.gui:
            cv2.imshow('UGV patrol planner', self.render())
            cv2.waitKey(1)

    def save_snapshot(self):
        cv2.imwrite(self.snapshot, self.render())

    # -------------------------------------------------------------- RViz2 markers
    def _xyz(self, cell, lift=0.0) -> Point:
        cs = self.cfg.cell_size_m
        r, c = cell
        return Point(x=c * cs, y=(self.n - 1 - r) * cs, z=float(self.elev[r, c]) + lift)

    def _marker(self, mid, mtype, sx, sy, sz, rgba) -> Marker:
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = 'patrol', mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = float(sx), float(sy), float(sz)
        m.color = ColorRGBA(r=float(rgba[0]), g=float(rgba[1]), b=float(rgba[2]), a=float(rgba[3]))
        return m

    def publish_markers(self):
        cs = self.cfg.cell_size_m
        e = self.elev
        out = MarkerArray()
        terrain = self._marker(0, Marker.CUBE_LIST, cs, cs, 1.0, (1, 1, 1, 1))
        lo, span = e.min(), max(np.ptp(e), 1e-9)
        for r in range(self.n):
            for c in range(self.n):
                terrain.points.append(self._xyz((r, c)))
                g = float(0.2 + 0.7 * (e[r, c] - lo) / span)
                terrain.colors.append(ColorRGBA(r=g, g=g, b=g, a=1.0))
        out.markers.append(terrain)
        obs = self._marker(1, Marker.CUBE_LIST, cs, cs, cs, (0.9, 0.15, 0.15, 1))
        obs.points = [self._xyz(c, cs / 2) for c in self.obstacles]
        out.markers.append(obs)
        path = self._marker(2, Marker.LINE_STRIP, cs / 4, 0, 0, (0.1, 0.45, 0.9, 1))
        path.points = [self._xyz(c, cs / 6) for c in self.path]
        out.markers.append(path)
        pts = self._marker(3, Marker.SPHERE_LIST, cs, cs, cs, (1, 0.8, 0, 1))
        pts.points = [self._xyz(tuple(p['cell']), cs / 2) for p in self.status.get('points', [])]
        out.markers.append(pts)
        if self.pos is not None:
            ugv = self._marker(4, Marker.SPHERE, 1.3 * cs, 1.3 * cs, 1.3 * cs, (0.2, 0.85, 0.2, 1))
            ugv.pose.position = self._xyz(self.pos, 0.7 * cs)
            out.markers.append(ugv)
        self.pub_markers.publish(out)


def main():
    rclpy.init()
    node = Visualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()
