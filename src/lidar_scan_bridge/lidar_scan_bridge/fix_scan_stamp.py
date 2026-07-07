# lidar_scan_bridge/fix_scan_stamp.py
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan


class FixScanStamp(Node):
    """Republish LaserScan with a stable frame and a TF-friendly timestamp."""

    def __init__(self):
        super().__init__('fix_scan_stamp')
        self.declare_parameter('input_scan', '/scan_raw')
        self.declare_parameter('output_scan', '/scan')
        self.declare_parameter('output_frame', 'base_link')
        self.declare_parameter('stamp_offset_sec', -0.08)

        self.input_topic = self.get_parameter('input_scan').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_scan').get_parameter_value().string_value
        self.output_frame = self.get_parameter('output_frame').get_parameter_value().string_value
        self.stamp_offset_sec = (
            self.get_parameter('stamp_offset_sec').get_parameter_value().double_value
        )

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.sub = self.create_subscription(LaserScan, self.input_topic, self.cb, sensor_qos)

        pub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pub = self.create_publisher(LaserScan, self.output_topic, pub_qos)

        self.get_logger().info(
            f'FixScanStamp: {self.input_topic} -> {self.output_topic}, '
            f'frame={self.output_frame}, stamp_offset_sec={self.stamp_offset_sec}'
        )

    def cb(self, msg: LaserScan):
        stamp = self.get_clock().now() + Duration(seconds=self.stamp_offset_sec)
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.output_frame
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(FixScanStamp())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
