import argparse
import subprocess
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


class ReadyWatcher(Node):
    def __init__(self, odom_frame: str, base_frame: str) -> None:
        super().__init__('quadruped_navigation_ready_command_runner')
        self._odom_frame = odom_frame
        self._base_frame = base_frame
        self._last_scan_monotonic = 0.0
        self._last_odom_tf_monotonic = 0.0
        self._tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        # 同时创建 best_effort/reliable 两个订阅，避免 /scan 的 Reliability QoS 不匹配导致收不到消息。
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(LaserScan, '/scan', self._on_scan, best_effort_qos)
        self.create_subscription(LaserScan, '/scan', self._on_scan, reliable_qos)

    def _on_scan(self, _msg: LaserScan) -> None:
        self._last_scan_monotonic = time.monotonic()

    def _refresh_odom_to_base_recent(self) -> None:
        try:
            self._tf_buffer.lookup_transform(
                self._odom_frame,
                self._base_frame,
                Time(),
                timeout=Duration(seconds=0.08),
            )
            self._last_odom_tf_monotonic = time.monotonic()
        except TransformException:
            pass

    def is_ready(self, max_data_age_sec: float) -> tuple[bool, bool]:
        self._refresh_odom_to_base_recent()
        now = time.monotonic()
        scan_ready = (now - self._last_scan_monotonic) <= max_data_age_sec
        odom_ready = (now - self._last_odom_tf_monotonic) <= max_data_age_sec
        return scan_ready, odom_ready


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Run a shell command after scan and odom TF become stable.')
    parser.add_argument('--command', default='')
    parser.add_argument('--timeout-sec', type=float, default=180.0)
    parser.add_argument('--max-data-age-sec', type=float, default=1.5)
    parser.add_argument('--stable-hold-sec', type=float, default=2.0)
    parser.add_argument('--delay-after-ready-sec', type=float, default=6.0)
    parser.add_argument('--odom-frame', default='odom')
    parser.add_argument('--base-frame', default='base_link')
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.command.strip():
        print('No command configured for run_when_ready, skipping.')
        return 0

    rclpy.init(args=None)
    node = ReadyWatcher(args.odom_frame, args.base_frame)
    logger = node.get_logger()
    start_time = time.monotonic()
    ready_since = None

    logger.info('Waiting for readiness before running configured command...')

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            scan_ready, odom_ready = node.is_ready(args.max_data_age_sec)
            now = time.monotonic()

            if scan_ready and odom_ready:
                if ready_since is None:
                    ready_since = now
                    logger.info('Readiness conditions met, holding briefly for stability...')
                elif now - ready_since >= args.stable_hold_sec:
                    logger.info(f'Readiness stable. Waiting {args.delay_after_ready_sec:.1f} seconds before running command...')
                    time.sleep(args.delay_after_ready_sec)
                    break
            else:
                if ready_since is not None:
                    logger.warn('Readiness lost during hold, restarting wait window.')
                ready_since = None

            if now - start_time > args.timeout_sec:
                logger.error('Timed out waiting to run configured command.')
                return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()

    result = subprocess.run(args.command, shell=True, check=False)
    return result.returncode


if __name__ == '__main__':
    sys.exit(main())
