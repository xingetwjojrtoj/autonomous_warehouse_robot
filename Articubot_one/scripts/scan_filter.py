#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan


class ScanFilter(Node):

    def __init__(self):
        super().__init__("scan_filter")

        self.subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10)

        self.publisher = self.create_publisher(
            LaserScan,
            "/scan_front",
            10)

        self.front_min = -math.pi    # -90°
        self.front_max =  math.pi     # +90°

    def scan_callback(self, msg):

        start_index = int(
            (self.front_min - msg.angle_min) /
            msg.angle_increment)

        end_index = int(
            (self.front_max - msg.angle_min) /
            msg.angle_increment)

        start_index = max(0, start_index)
        end_index = min(len(msg.ranges) - 1, end_index)

        filtered = LaserScan()

        filtered.header = msg.header

        filtered.angle_min = self.front_min
        filtered.angle_max = self.front_max

        filtered.angle_increment = msg.angle_increment
        filtered.time_increment = msg.time_increment
        filtered.scan_time = msg.scan_time

        filtered.range_min = msg.range_min
        filtered.range_max = msg.range_max

        filtered.ranges = msg.ranges[start_index:end_index + 1]

        if len(msg.intensities) > 0:
            filtered.intensities = msg.intensities[start_index:end_index + 1]

        self.publisher.publish(filtered)


def main(args=None):

    rclpy.init(args=args)

    node = ScanFilter()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
