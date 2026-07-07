#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanDensityCheck(Node):
    def __init__(self):
        super().__init__("scan_density_check")
        self.create_subscription(LaserScan, "/scan", self.cb, 10)

    @staticmethod
    def zone(angle):
        deg = math.degrees(angle)
        if abs(deg) <= 60.0:
            return "front"
        if 60.0 < deg <= 120.0:
            return "left"
        if -120.0 <= deg < -60.0:
            return "right"
        return "back"

    def cb(self, msg):
        buckets = {key: [] for key in ["front", "left", "right", "back"]}
        all_finite = []
        for i, distance in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if math.isfinite(distance):
                all_finite.append(distance)
                if distance < 4.0:
                    buckets[self.zone(angle)].append(distance)

        print("frame:", msg.header.frame_id)
        print("angle_min/max:", round(math.degrees(msg.angle_min), 1), round(math.degrees(msg.angle_max), 1))
        print("angle_increment_deg:", round(math.degrees(msg.angle_increment), 3))
        print("finite_total:", len(all_finite), "min:", min(all_finite) if all_finite else None)
        for key, values in buckets.items():
            print(key, "count_under_4m:", len(values), "min:", min(values) if values else None)
        rclpy.shutdown()


def main():
    rclpy.init()
    node = ScanDensityCheck()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
