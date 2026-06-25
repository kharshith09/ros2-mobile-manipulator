#!/usr/bin/env python3
"""
Frontier-based autonomous map explorer.
Subscribes: /map, /scan
Publishes:  /cmd_vel
Pose via:   TF  map -> base_footprint
"""

import math
import os
import subprocess
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
import numpy as np
from scipy import ndimage

MAP_SAVE_PATH = os.path.expanduser(
    '~/ros2_ws2/src/bot_navigation/maps/colorful_house_map'
)

# ── tuning ───────────────────────────────────────────────────────────
GOAL_TOL       = 0.6    # m — close enough to a frontier
OBSTACLE_DIST  = 0.35   # m — stop / replan if obstacle this close ahead
LINEAR_SPEED   = 0.22   # m/s top forward speed
ANGULAR_GAIN   = 2.0    # heading P-gain
TURN_TOL       = 0.12   # rad — finished turning
MIN_FRONTIER   = 5      # minimum frontier-region size (cells)
MIN_FRONTIER_DIST = 0.8  # m — ignore frontiers closer than this
RECOVERY_TIME  = 1.5    # seconds to back-up on obstacle
TICK_RATE      = 10     # Hz
# ─────────────────────────────────────────────────────────────────────

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        self.map_sub  = self.create_subscription(OccupancyGrid, '/map',  self.map_cb,  5)
        self.scan_sub = self.create_subscription(LaserScan,     '/scan', self.scan_cb, 5)
        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map   = None
        self.scan  = None
        self.state = 'FIND'
        self.target      = None   # (wx, wy)
        self.target_yaw  = None
        self.recovery_t    = 0.0
        self.recovery_dir  = 1.0   # alternates +/- to escape corners
        self.consec_blocks = 0     # consecutive obstacle hits on same target
        self.visited       = set()  # frontier centroids already tried

        self.timer = self.create_timer(1.0 / TICK_RATE, self.tick)
        self.get_logger().info('Frontier explorer ready — waiting for map...')

    # ── callbacks ────────────────────────────────────────────────────

    def map_cb(self, msg):
        self.map = msg

    def scan_cb(self, msg):
        self.scan = msg

    # ── helpers ──────────────────────────────────────────────────────

    def get_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
            x   = t.transform.translation.x
            y   = t.transform.translation.y
            q   = t.transform.rotation
            yaw = math.atan2(
                2*(q.w*q.z + q.x*q.y),
                1 - 2*(q.y**2 + q.z**2))
            return x, y, yaw
        except Exception:
            return None

    def find_frontiers(self):
        """Return list of (wx, wy, size) for each frontier cluster."""
        info = self.map.info
        grid = np.array(self.map.data, dtype=np.int8).reshape(
            info.height, info.width)

        free    = (grid == 0)
        unknown = (grid == -1)

        # frontier cell = free AND touches an unknown cell
        adj_unknown = ndimage.binary_dilation(unknown, structure=np.ones((3,3)))
        mask = free & adj_unknown

        labeled, n = ndimage.label(mask)
        centroids = []
        for i in range(1, n + 1):
            region = labeled == i
            if region.sum() < MIN_FRONTIER:
                continue
            rows, cols = np.where(region)
            wx = info.origin.position.x + cols.mean() * info.resolution
            wy = info.origin.position.y + rows.mean() * info.resolution
            centroids.append((wx, wy, int(region.sum())))
        return centroids

    def obstacle_ahead(self):
        if self.scan is None:
            return False
        r = np.array(self.scan.ranges, dtype=np.float32)
        r[np.isinf(r) | np.isnan(r)] = self.scan.range_max
        n   = len(r)
        arc = max(1, n // 12)          # ±15° front arc
        front = np.concatenate([r[:arc], r[-arc:]])
        return bool(np.any(front < OBSTACLE_DIST))

    def _save_map(self):
        self.get_logger().info(f'Saving map to {MAP_SAVE_PATH}.*')
        try:
            result = subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                 '-f', MAP_SAVE_PATH,
                 '--ros-args', '-p', 'use_sim_time:=true'],
                capture_output=True, text=True, timeout=30
            )
            if result.returncode == 0:
                self.get_logger().info(
                    f'Map saved:  {MAP_SAVE_PATH}.pgm  +  .yaml')
            else:
                self.get_logger().error(
                    f'map_saver failed:\n{result.stderr}')
        except Exception as e:
            self.get_logger().error(f'Map save error: {e}')

    @staticmethod
    def angle_diff(a, b):
        d = a - b
        while d >  math.pi: d -= 2 * math.pi
        while d < -math.pi: d += 2 * math.pi
        return d

    def stop(self):
        try:
            self.cmd_pub.publish(Twist())
        except Exception:
            pass

    # ── main loop ────────────────────────────────────────────────────

    def tick(self):
        if self.map is None:
            return
        pose = self.get_pose()
        if pose is None:
            return
        rx, ry, ryaw = pose
        cmd = Twist()
        now = self.get_clock().now().nanoseconds * 1e-9

        # ── FIND ─────────────────────────────────────────────────────
        if self.state == 'FIND':
            frontiers = self.find_frontiers()
            # filter already-visited targets and frontiers too close
            frontiers = [f for f in frontiers
                         if math.hypot(f[0] - rx, f[1] - ry) > MIN_FRONTIER_DIST
                         and not any(math.hypot(f[0]-vx, f[1]-vy) < 1.0
                                     for vx, vy in self.visited)]

            if not frontiers:
                self.get_logger().info(
                    'No more frontiers — exploration complete!')
                self.state = 'DONE'
                return

            # pick closest frontier
            best = min(frontiers,
                       key=lambda f: math.hypot(f[0] - rx, f[1] - ry))
            self.target     = (best[0], best[1])
            self.target_yaw = math.atan2(best[1] - ry, best[0] - rx)
            dist            = math.hypot(best[0] - rx, best[1] - ry)

            self.get_logger().info(
                f'[FIND] → frontier ({best[0]:.1f}, {best[1]:.1f})  '
                f'dist={dist:.1f}m  size={best[2]}  remaining={len(frontiers)}')
            self.state = 'TURN'

        # ── TURN ─────────────────────────────────────────────────────
        elif self.state == 'TURN':
            err = self.angle_diff(self.target_yaw, ryaw)
            if abs(err) > TURN_TOL:
                cmd.angular.z = ANGULAR_GAIN * err
                cmd.angular.z = max(-1.5, min(1.5, cmd.angular.z))
            else:
                self.get_logger().info('[TURN] aligned → DRIVE')
                self.state = 'DRIVE'

        # ── DRIVE ────────────────────────────────────────────────────
        elif self.state == 'DRIVE':
            dx   = self.target[0] - rx
            dy   = self.target[1] - ry
            dist = math.hypot(dx, dy)

            if dist < GOAL_TOL:
                self.get_logger().info('[DRIVE] reached frontier → FIND')
                self.visited.add(self.target)
                self.state = 'FIND'
                return

            if self.obstacle_ahead():
                self.consec_blocks += 1
                self.recovery_dir *= -1.0
                self.get_logger().warn(
                    f'[DRIVE] obstacle! → RECOVER (hit #{self.consec_blocks})')
                self.recovery_t = now
                self.state = 'RECOVER'
                return

            self.consec_blocks = 0

            target_yaw = math.atan2(dy, dx)
            err = self.angle_diff(target_yaw, ryaw)
            cmd.linear.x  = min(LINEAR_SPEED, dist * 0.5)
            cmd.angular.z = ANGULAR_GAIN * err
            cmd.angular.z = max(-1.2, min(1.2, cmd.angular.z))

        # ── RECOVER ──────────────────────────────────────────────────
        elif self.state == 'RECOVER':
            elapsed = now - self.recovery_t
            duration = RECOVERY_TIME + min(self.consec_blocks * 0.5, 2.0)
            if elapsed < duration:
                cmd.linear.x  = -0.12
                cmd.angular.z =  0.8 * self.recovery_dir
            else:
                self.get_logger().info('[RECOVER] done → FIND')
                if self.consec_blocks >= 3:
                    self.visited.add(self.target)
                    self.consec_blocks = 0
                self.state = 'FIND'

        # ── DONE ─────────────────────────────────────────────────────
        elif self.state == 'DONE':
            self.stop()
            self.get_logger().info('Map fully explored — saving map...')
            self.timer.cancel()
            self._save_map()
            return

        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
        print('\nExplorer stopped.')


if __name__ == '__main__':
    main()
