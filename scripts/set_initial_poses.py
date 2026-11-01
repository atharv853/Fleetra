#!/usr/bin/env python3
"""Replaces the manual "2D Pose Estimate" clicks in RViz.

The exercise page says: set each robot's initial pose "by estimating its position
from Gazebo" and check that "the lidar rays roughly match the obstacle outline".
This script does exactly that, programmatically:

  1. read the robot's CURRENT pose from Gazebo (`gz model -m <robot> -p`).
     Falls back to the spawn pose from aws_amazon_robot_multiple.py
     (robot1: 0,-1,0  robot2: 0,4,0) if Gazebo cannot be queried.
     (The JdeRobot robot model slowly creeps/turns in place after spawning
     - ~7 deg/min on Gazebo 11 - so by the time Nav2 is up the spawn pose can be
     10-20 deg off; that is why the live pose is used.)
  2. refine it with a small scan-to-map match (+-0.4 m, +-0.25 rad) so the lidar
     endpoints sit on the map's obstacles,
  3. publish geometry_msgs/PoseWithCovarianceStamped on /<robot>/initialpose
     (every 2 s) until /<robot>/amcl_pose agrees,
  4. report the final scan/map alignment (% of lidar endpoints on map obstacles).

Usage: set_initial_poses.py [robot:x,y,yaw ...]   (spawn poses = fallback)
Exit code 0 only if every robot converged with >= MIN_MATCH alignment.
"""
import math
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

SPAWN = ['robot1:0.0,-1.0,0.0', 'robot2:0.0,4.0,0.0']   # aws_amazon_robot_multiple.py
LASER_OFFSET = (-0.064, 0.0)       # base_footprint -> base_scan (from TF)
POS_TOL, YAW_TOL = 0.10, 0.06      # amcl_pose vs published pose
MIN_MATCH = 0.70                   # min fraction of scan points on obstacles
TIMEOUT = 90.0


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def angdiff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def gazebo_pose(name):
    try:
        out = subprocess.run(['gz', 'model', '-m', name, '-p'], capture_output=True,
                             text=True, timeout=10).stdout.split()
        x, y, _z, _r, _p, yaw = (float(v) for v in out[:6])
        return x, y, yaw
    except Exception:
        return None


class Robot:
    def __init__(self, node, name, x, y, yaw):
        self.node, self.name = node, name
        self.spawn = (x, y, yaw)
        self.target = None
        self.amcl = self.scan = self.map = None
        self._grid = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub = node.create_publisher(PoseWithCovarianceStamped, f'/{name}/initialpose', 10)
        node.create_subscription(PoseWithCovarianceStamped, f'/{name}/amcl_pose',
                                 lambda m: setattr(self, 'amcl', m), latched)
        node.create_subscription(LaserScan, f'/{name}/scan',
                                 lambda m: setattr(self, 'scan', m), qos_profile_sensor_data)
        node.create_subscription(OccupancyGrid, f'/{name}/map', self._on_map, latched)

    def _on_map(self, m):
        self.map = m
        grid = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width) > 50
        g = grid.copy()                       # dilate 1 cell (5 cm) for tolerance
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                g |= np.roll(np.roll(grid, dx, 1), dy, 0)
        self._grid = g

    # ---------- scan / map alignment ----------
    def _scan_pts(self):
        s = self.scan
        r = np.array(s.ranges)
        a = s.angle_min + np.arange(len(r)) * s.angle_increment
        ok = np.isfinite(r) & (r > s.range_min) & (r < min(s.range_max, 10.0))
        r, a = r[ok], a[ok]
        return LASER_OFFSET[0] + r * np.cos(a), LASER_OFFSET[1] + r * np.sin(a)

    def match(self, x, y, yaw, pts=None):
        if self.scan is None or self._grid is None:
            return None
        lx, ly = pts if pts is not None else self._scan_pts()
        if len(lx) == 0:
            return None
        info = self.map.info
        c, s = math.cos(yaw), math.sin(yaw)
        i = ((x + c * lx - s * ly - info.origin.position.x) / info.resolution).astype(int)
        j = ((y + s * lx + c * ly - info.origin.position.y) / info.resolution).astype(int)
        v = (i >= 0) & (i < info.width) & (j >= 0) & (j < info.height)
        return float(self._grid[j[v], i[v]].sum()) / len(lx)

    def refine(self, x, y, yaw):
        """Local brute-force search around (x,y,yaw); coarse then fine."""
        pts = self._scan_pts()
        best = (self.match(x, y, yaw, pts), x, y, yaw)
        for span, step, aspan, astep in ((0.4, 0.05, 0.25, 0.02), (0.06, 0.01, 0.03, 0.005)):
            cx, cy, cyaw = best[1], best[2], best[3]
            for th in np.arange(cyaw - aspan, cyaw + aspan + 1e-9, astep):
                for dx in np.arange(-span, span + 1e-9, step):
                    for dy in np.arange(-span, span + 1e-9, step):
                        sc = self.match(cx + dx, cy + dy, th, pts)
                        if sc > best[0]:
                            best = (sc, cx + dx, cy + dy, float(th))
        return best

    # ---------- AMCL ----------
    def publish(self):
        x, y, yaw = self.target
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.pose.pose.position.x = x
        m.pose.pose.position.y = y
        m.pose.pose.orientation.z = math.sin(yaw / 2)
        m.pose.pose.orientation.w = math.cos(yaw / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.05 ** 2
        cov[35] = 0.03 ** 2
        m.pose.covariance = cov
        self.pub.publish(m)

    def amcl_xyyaw(self):
        p = self.amcl.pose.pose
        return p.position.x, p.position.y, yaw_of(p.orientation)

    def converged(self):
        if self.amcl is None or self.target is None:
            return False
        x, y, yaw = self.amcl_xyyaw()
        # The robot may creep slightly between publishing and AMCL's answer, so
        # compare with the live Gazebo-based target re-evaluated by AMCL's own frame.
        return (math.hypot(x - self.target[0], y - self.target[1]) < POS_TOL
                and abs(angdiff(yaw, self.target[2])) < YAW_TOL)


def spin_for(node, secs, cond=lambda: False):
    end = time.time() + secs
    while time.time() < end and not cond():
        rclpy.spin_once(node, timeout_sec=0.05)


def main():
    specs = sys.argv[1:] or SPAWN
    rclpy.init()
    node = Node('demo_set_initial_poses')
    log = node.get_logger()
    robots = []
    for spec in specs:
        name, vals = spec.split(':')
        robots.append(Robot(node, name, *(float(v) for v in vals.split(','))))

    # wait for map + scan (max 30 s)
    spin_for(node, 30, lambda: all(r.scan is not None and r._grid is not None for r in robots))

    for r in robots:
        g = gazebo_pose(r.name)
        guess = g if g else r.spawn
        src = 'Gazebo' if g else 'spawn pose (Gazebo query failed)'
        r.scan = None
        spin_for(node, 3, lambda: r.scan is not None)           # fresh scan
        if r.scan is not None and r._grid is not None:
            sc0 = r.match(*guess)
            sc, x, y, yaw = r.refine(*guess)
            log.info(f'{r.name}: {src} ({guess[0]:.2f}, {guess[1]:.2f}, yaw {guess[2]:.2f}) '
                     f'match {100*sc0:.0f}% -> scan-refined ({x:.2f}, {y:.2f}, yaw {yaw:.2f}) '
                     f'match {100*sc:.0f}%')
            r.target = (x, y, yaw)
        else:
            log.warn(f'{r.name}: no scan/map yet, using {src} unrefined')
            r.target = guess

    deadline = time.time() + TIMEOUT
    last_pub = 0.0
    while time.time() < deadline and not all(r.converged() for r in robots):
        if time.time() - last_pub > 2.0:
            for r in robots:
                if not r.converged():
                    log.info(f'{r.name}: initialpose -> ({r.target[0]:.2f}, {r.target[1]:.2f}, '
                             f'yaw {r.target[2]:.2f})')
                    r.publish()
            last_pub = time.time()
        rclpy.spin_once(node, timeout_sec=0.05)

    # final verification against a fresh scan
    for r in robots:
        r.scan = None
    spin_for(node, 5, lambda: all(r.scan is not None for r in robots))
    ok = True
    for r in robots:
        if not r.converged():
            log.error(f'{r.name}: amcl_pose did not converge')
            ok = False
            continue
        x, y, yaw = r.amcl_xyyaw()
        m = r.match(x, y, yaw)
        msg = (f'{r.name}: amcl_pose = ({x:.2f}, {y:.2f}, yaw {yaw:.2f}); '
               f'lidar/map alignment = ' + (f'{100*m:.0f}%' if m is not None else 'n/a'))
        if m is None or m < MIN_MATCH:
            log.error(msg + '  <-- POOR ALIGNMENT')
            ok = False
        else:
            log.info(msg + '  OK')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
