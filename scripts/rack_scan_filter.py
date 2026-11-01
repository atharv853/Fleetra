#!/usr/bin/env python3
"""Box filter for the lidar: /<robot>/scan -> /<robot>/scan_filtered.

Returns that fall inside the footprint of a rack the robot is carrying
(rack 2.0 x 0.8 m, legs at +-0.98/+-0.38 m, + docking slack) are set to inf, so
the robot does not see its own rack legs as obstacles (AMCL and costmaps use
scan_filtered). Box is in base_footprint; lidar sits at x=-0.064 m (fixed joint).
Usage: rack_scan_filter.py robot1 robot2
"""
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

BOX_X, BOX_Y, LASER_X = 1.25, 0.6, -0.064


class Filter:
    def __init__(self, node, robot):
        self.pub = node.create_publisher(LaserScan, f'/{robot}/scan_filtered', 10)
        node.create_subscription(LaserScan, f'/{robot}/scan', self.cb, qos_profile_sensor_data)

    def cb(self, m):
        out = list(m.ranges)
        a = m.angle_min
        for i, r in enumerate(out):
            if math.isfinite(r):
                x, y = LASER_X + r * math.cos(a), r * math.sin(a)
                if abs(x) < BOX_X and abs(y) < BOX_Y:
                    out[i] = float('inf')
            a += m.angle_increment
        m.ranges = out
        self.pub.publish(m)


def main():
    rclpy.init()
    node = Node('rack_scan_filter', parameter_overrides=[
        rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
    filters = [Filter(node, r) for r in (sys.argv[1:] or ['robot1', 'robot2'])]  # noqa: F841
    rclpy.spin(node)


if __name__ == '__main__':
    main()
