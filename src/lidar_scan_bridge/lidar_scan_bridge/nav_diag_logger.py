import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class NavDiagLogger(Node):
    """
    诊断“左转/右转交替发送”问题：
    - 观测 cmd_vel 角速度是否快速符号翻转
    - 同时打印 amcl_pose yaw 与协方差
    - 同时打印 map->base_link / map->odom 的 yaw
    """

    def __init__(self) -> None:
        super().__init__("nav_diag_logger")

        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("amcl_topic", "/amcl_pose")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("flip_threshold", 0.12)
        self.declare_parameter("flip_window_s", 0.35)
        self.declare_parameter("summary_period_s", 1.0)

        self.cmd_topic = self.get_parameter("cmd_topic").value
        self.amcl_topic = self.get_parameter("amcl_topic").value
        self.base_frame = self.get_parameter("base_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.map_frame = self.get_parameter("map_frame").value
        self.flip_threshold = float(self.get_parameter("flip_threshold").value)
        self.flip_window_s = float(self.get_parameter("flip_window_s").value)
        summary_period_s = float(self.get_parameter("summary_period_s").value)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.last_cmd_wz: Optional[float] = None
        self.last_cmd_t: Optional[rclpy.time.Time] = None
        self.last_amcl_yaw: Optional[float] = None
        self.last_amcl_cov_yaw: Optional[float] = None

        self.create_subscription(Twist, self.cmd_topic, self.cmd_cb, 20)
        self.create_subscription(PoseWithCovarianceStamped, self.amcl_topic, self.amcl_cb, 20)
        self.create_timer(summary_period_s, self.summary_tick)

        self.get_logger().info(
            f"NavDiagLogger started. cmd={self.cmd_topic}, amcl={self.amcl_topic}, "
            f"flip_th={self.flip_threshold:.2f}, window={self.flip_window_s:.2f}s"
        )

    def amcl_cb(self, msg: PoseWithCovarianceStamped) -> None:
        q = msg.pose.pose.orientation
        self.last_amcl_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        self.last_amcl_cov_yaw = msg.pose.covariance[35]

    def cmd_cb(self, msg: Twist) -> None:
        wz = float(msg.angular.z)
        now = self.get_clock().now()
        if self.last_cmd_wz is not None and self.last_cmd_t is not None:
            dt = (now - self.last_cmd_t).nanoseconds / 1e9
            sign_flip = (wz * self.last_cmd_wz) < 0.0
            large_enough = abs(wz) >= self.flip_threshold and abs(self.last_cmd_wz) >= self.flip_threshold
            if sign_flip and large_enough and dt <= self.flip_window_s:
                map_base_yaw = self.lookup_yaw(self.map_frame, self.base_frame)
                map_odom_yaw = self.lookup_yaw(self.map_frame, self.odom_frame)
                self.get_logger().warn(
                    "[FlipDiag] cmd_wz flip prev=%.3f curr=%.3f dt=%.3fs "
                    "map->base=%.3f map->odom=%.3f amcl_yaw=%.3f cov_yaw=%.4f"
                    % (
                        self.last_cmd_wz,
                        wz,
                        dt,
                        map_base_yaw if map_base_yaw is not None else float("nan"),
                        map_odom_yaw if map_odom_yaw is not None else float("nan"),
                        self.last_amcl_yaw if self.last_amcl_yaw is not None else float("nan"),
                        self.last_amcl_cov_yaw if self.last_amcl_cov_yaw is not None else float("nan"),
                    )
                )

        self.last_cmd_wz = wz
        self.last_cmd_t = now

    def lookup_yaw(self, target_frame: str, source_frame: str) -> Optional[float]:
        try:
            trans = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.03),
            )
            q = trans.transform.rotation
            return quat_to_yaw(q.x, q.y, q.z, q.w)
        except TransformException:
            return None

    def summary_tick(self) -> None:
        map_base_yaw = self.lookup_yaw(self.map_frame, self.base_frame)
        map_odom_yaw = self.lookup_yaw(self.map_frame, self.odom_frame)
        self.get_logger().info(
            "[NavDiag] cmd_wz=%.3f map->base=%.3f map->odom=%.3f amcl_yaw=%.3f cov_yaw=%.4f"
            % (
                self.last_cmd_wz if self.last_cmd_wz is not None else float("nan"),
                map_base_yaw if map_base_yaw is not None else float("nan"),
                map_odom_yaw if map_odom_yaw is not None else float("nan"),
                self.last_amcl_yaw if self.last_amcl_yaw is not None else float("nan"),
                self.last_amcl_cov_yaw if self.last_amcl_cov_yaw is not None else float("nan"),
            )
        )


def main() -> None:
    rclpy.init()
    node = NavDiagLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
