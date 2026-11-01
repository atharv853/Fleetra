#!/usr/bin/env python3
"""count_msgs.py TOPIC [secs] - prints number of LaserScan msgs received (exit 0 if >0)."""
import sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
rclpy.init(); n = Node('count_msgs'); c = [0]
n.create_subscription(LaserScan, sys.argv[1], lambda m: c.__setitem__(0, c[0] + 1), qos_profile_sensor_data)
end = time.time() + float(sys.argv[2] if len(sys.argv) > 2 else 5)
while time.time() < end: rclpy.spin_once(n, timeout_sec=0.1)
print(sys.argv[1], c[0]); sys.exit(0 if c[0] else 1)
