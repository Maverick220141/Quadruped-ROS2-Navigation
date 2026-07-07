import argparse
import math
import os
import subprocess
import sys
import time
from typing import Optional

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformException, TransformListener


class ReadyWatcher(Node):
    def __init__(self, odom_frame: str, base_frame: str, time_lag_topic: str) -> None:
        super().__init__('quadruped_navigation_ready_watcher')
        self._odom_frame = odom_frame
        self._base_frame = base_frame
        self._last_scan_monotonic = 0.0
        self._last_odom_tf_monotonic = 0.0
        self._last_lag_sec = float('nan')
        self._tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
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
        lag_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(Float64, time_lag_topic, self._on_lag, lag_qos)

    def _on_scan(self, _msg: LaserScan) -> None:
        self._last_scan_monotonic = time.monotonic()

    def _on_lag(self, msg: Float64) -> None:
        self._last_lag_sec = float(msg.data)

    def invalidate_lag_sample(self) -> None:
        """重启 fast_lio 后清掉缓存，避免 transient_local 上一条旧 lag 立刻再次触发重启。"""
        self._last_lag_sec = float('nan')

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

    def is_ready(self, max_data_age_sec: float) -> tuple[bool, bool, float]:
        self._refresh_odom_to_base_recent()
        now = time.monotonic()
        scan_ready = (now - self._last_scan_monotonic) <= max_data_age_sec
        odom_ready = (now - self._last_odom_tf_monotonic) <= max_data_age_sec
        if math.isnan(self._last_lag_sec):
            return scan_ready, odom_ready, float('inf')
        return scan_ready, odom_ready, self._last_lag_sec


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Wait for /scan and odom TF, then exec nav launch.')
    parser.add_argument('--map', required=True)
    parser.add_argument('--params-file', required=True)
    parser.add_argument('--use-sim-time', default='false')
    parser.add_argument('--use-rviz', default='false')
    parser.add_argument('--timeout-sec', type=float, default=120.0)
    parser.add_argument('--max-data-age-sec', type=float, default=1.5)
    parser.add_argument(
        '--time-lag-topic',
        default='/fast_lio/time_lag_sec',
        help='Fast-LIO 发布的 lag（与 [TimeDiag] 同源，std_msgs/Float64）。',
    )
    parser.add_argument(
        '--max-odom-lag-sec',
        type=float,
        default=0.06,
        help='允许的最大 lag（秒）；与 launch 参数 nav_ready_max_odom_lag_sec 对应。',
    )
    parser.add_argument(
        '--max-odom-lag-release-sec',
        type=float,
        default=0.12,
        help='hold 窗口内：超过该 lag 才判失败（滞回）。',
    )
    parser.add_argument('--stable-hold-sec', type=float, default=2.0)
    parser.add_argument('--odom-frame', default='odom')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument(
        '--restart-fast-lio-on-bad-lag',
        type=str,
        default='false',
        help='true/false：lag 持续超阈值时对 fastlio_mapping 发 SIGINT（依赖上层 launch 对 fast_lio 使用 respawn）。',
    )
    parser.add_argument(
        '--restart-lag-threshold-sec',
        type=float,
        default=1.0,
        help='仅当 lag（有限值）>= 该秒数才触发重启。',
    )
    parser.add_argument(
        '--restart-cooldown-sec',
        type=float,
        default=25.0,
        help='两次 SIGINT 之间的基础冷却（秒）。',
    )
    parser.add_argument(
        '--restart-grace-after-kill-sec',
        type=float,
        default=15.0,
        help='叠加在 cooldown 之后的静默（秒）：实际间隔 = cooldown + grace，给 respawn+IMU 初始化留时间。',
    )
    return parser.parse_args()


def _truthy(s: str) -> bool:
    return str(s).strip().lower() in ('1', 'true', 'yes', 'on')


def main() -> int:
    args = parse_args()
    rclpy.init(args=None)
    node = ReadyWatcher(args.odom_frame, args.base_frame, args.time_lag_topic)
    logger = node.get_logger()
    restart_on_bad_lag = _truthy(args.restart_fast_lio_on_bad_lag)

    start_time = time.monotonic()
    ready_since = None
    last_not_ready_log_time = 0.0
    logger.info(
        f'等待 /scan、TF {args.odom_frame}->{args.base_frame}、{args.time_lag_topic} lag '
        f'<={args.max_odom_lag_sec:.3f}s（hold 内放弃则 >{args.max_odom_lag_release_sec:.3f}s）后启动 my_nav …'
    )

    def lag_acceptable(sec: float, in_hold: bool) -> bool:
        limit = args.max_odom_lag_release_sec if in_hold else args.max_odom_lag_sec
        if sec < -0.05:
            return False
        return sec <= limit

    try:
        last_met_log_mono = -1e9
        last_lost_log_mono = -1e9
        last_fastlio_restart_mono: Optional[float] = None
        restart_interval = args.restart_cooldown_sec + args.restart_grace_after_kill_sec
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            scan_ready, odom_ready, lag_sec = node.is_ready(args.max_data_age_sec)
            now = time.monotonic()
            in_hold = ready_since is not None
            lag_ready = lag_acceptable(lag_sec, in_hold)

            if scan_ready and odom_ready and lag_ready:
                if ready_since is None:
                    ready_since = now
                    if now - last_met_log_mono >= 5.0:
                        logger.info('Readiness conditions met, holding briefly for stability...')
                        last_met_log_mono = now
                elif now - ready_since >= args.stable_hold_sec:
                    logger.info(
                        f'Readiness stable (lag={lag_sec:.4f}s <= {args.max_odom_lag_sec:.3f}s), launching my_nav now.'
                    )
                    break
            else:
                if ready_since is not None:
                    if now - last_lost_log_mono >= 5.0:
                        logger.warn(
                            'Readiness lost during hold, restarting wait window '
                            f'(lag={lag_sec:.3f}s；限频 5s)'
                        )
                        last_lost_log_mono = now
                ready_since = None
                if now - last_not_ready_log_time >= 2.0:
                    logger.warn(
                        f'Not ready: scan={scan_ready} tf={odom_ready} lag_ok={lag_ready} lag={lag_sec} '
                        f'(hold_release>{args.max_odom_lag_release_sec:.3f}s, topic={args.time_lag_topic})'
                    )
                    last_not_ready_log_time = now
                    # 仅在 scan/tf 已就绪但 lag 仍很差时重启，避免启动期无点云时误杀
                    if (
                        restart_on_bad_lag
                        and scan_ready
                        and odom_ready
                        and (not lag_ready)
                        and (not math.isinf(lag_sec))
                        and lag_sec >= args.restart_lag_threshold_sec
                        and (
                            last_fastlio_restart_mono is None
                            or (now - last_fastlio_restart_mono) >= restart_interval
                        )
                    ):
                        logger.warn(
                            f'[restart] lag={lag_sec:.3f}s >= {args.restart_lag_threshold_sec}s '
                            '→ SIGINT fastlio_mapping；'
                            f'下次最早 {restart_interval:.0f}s 内不会再发 SIGINT（cooldown+grace）'
                        )
                        subprocess.run(
                            ['pkill', '-INT', '-f', 'fastlio_mapping'],
                            check=False,
                        )
                        last_fastlio_restart_mono = now
                        node.invalidate_lag_sample()

            if now - start_time > args.timeout_sec:
                logger.error('Timed out waiting for /scan and odom TF.')
                return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()

    os.execvp(
        'ros2',
        [
            'ros2',
            'launch',
            'my_nav',
            'nav2_bringup_gicp.launch.py',
            f'map:={args.map}',
            f'params_file:={args.params_file}',
            f'use_sim_time:={args.use_sim_time}',
            f'use_rviz:={args.use_rviz}',
        ],
    )
    return 0


if __name__ == '__main__':
    sys.exit(main())
