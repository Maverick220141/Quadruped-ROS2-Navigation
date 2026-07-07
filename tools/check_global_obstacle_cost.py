#!/usr/bin/env python3
import math
import sys

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class GlobalObstacleCostCheck(Node):
    def __init__(self):
        super().__init__("global_obstacle_cost_check")
        self.scan = None
        self.costmap = None
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(LaserScan, "/scan", self.scan_cb, sensor_qos)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap", self.costmap_cb, 10)
        self.timer = self.create_timer(0.2, self.check)

    def scan_cb(self, msg):
        self.scan = msg

    def costmap_cb(self, msg):
        self.costmap = msg

    def check(self):
        if self.scan is None or self.costmap is None:
            return

        hit = None
        for i, r in enumerate(self.scan.ranges):
            angle = self.scan.angle_min + i * self.scan.angle_increment
            if not math.isfinite(r):
                continue
            if abs(angle) > math.radians(60):
                continue
            if r < self.scan.range_min or r > min(self.scan.range_max, 4.0):
                continue
            if hit is None or r < hit[0]:
                hit = (r, angle)

        if hit is None:
            print("No finite front scan hit within +/-60deg and 4.0m")
            rclpy.shutdown()
            return

        r, angle = hit
        x_src = r * math.cos(angle)
        y_src = r * math.sin(angle)

        try:
            tf = self.tf_buffer.lookup_transform(
                self.costmap.header.frame_id,
                self.scan.header.frame_id,
                self.scan.header.stamp,
                timeout=Duration(seconds=0.2),
            )
            tf_mode = "scan_stamp"
        except TransformException as exc:
            print(f"TF at scan stamp failed: {exc}")
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.costmap.header.frame_id,
                    self.scan.header.frame_id,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.2),
                )
                tf_mode = "latest"
            except TransformException as exc2:
                print(f"TF latest failed: {exc2}")
                rclpy.shutdown()
                return

        yaw = yaw_from_quat(tf.transform.rotation)
        c = math.cos(yaw)
        s = math.sin(yaw)
        x_map = tf.transform.translation.x + c * x_src - s * y_src
        y_map = tf.transform.translation.y + s * x_src + c * y_src

        info = self.costmap.info
        gx = int((x_map - info.origin.position.x) / info.resolution)
        gy = int((y_map - info.origin.position.y) / info.resolution)

        print("scan frame:", self.scan.header.frame_id)
        print("costmap frame:", self.costmap.header.frame_id)
        print("tf mode:", tf_mode)
        print(f"front hit: range={r:.3f} angle_deg={math.degrees(angle):.1f}")
        print(f"point in map: x={x_map:.3f} y={y_map:.3f}")
        print(
            "costmap:",
            f"origin=({info.origin.position.x:.3f},{info.origin.position.y:.3f})",
            f"size=({info.width},{info.height})",
            f"res={info.resolution}",
        )
        print(f"grid: x={gx} y={gy}")

        if gx < 0 or gy < 0 or gx >= info.width or gy >= info.height:
            print("RESULT: point is OUT_OF_BOUNDS, global costmap cannot mark it")
            rclpy.shutdown()
            return

        data = self.costmap.data
        idx = gy * info.width + gx
        cell = data[idx]

        radius_cells = max(1, int(0.4 / info.resolution))
        near_values = []
        for yy in range(max(0, gy - radius_cells), min(info.height, gy + radius_cells + 1)):
            for xx in range(max(0, gx - radius_cells), min(info.width, gx + radius_cells + 1)):
                near_values.append(data[yy * info.width + xx])

        print("cell cost:", cell)
        print("nearby max cost:", max(near_values))
        print("nearby lethal_or_near:", sum(1 for v in near_values if v >= 90))
        if max(near_values) <= 0:
            print("RESULT: scan point is inside global map, but global costmap did NOT mark it")
        else:
            print("RESULT: global costmap has cost near the scan point")
        rclpy.shutdown()


def main():
    rclpy.init()
    node = GlobalObstacleCostCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
