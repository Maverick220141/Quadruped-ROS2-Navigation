# 四足机器人 ROS2 建图导航避障 / Quadruped ROS2 Navigation

## 中文说明

本开源仓库包含一套面向四足机器人实机运行的 ROS2 建图、定位、导航与避障工程系统。
仓库内容覆盖 Livox MID360 传感器接入、FAST-LIO 建图与里程计、GICP 全局定位、Nav2 自主导航、
点云到二维激光数据转换、虚拟障碍避障、地图资产管理，以及 AgiBot 机器狗运动控制接口。
整体目标是将开源算法、ROS2 导航组件和真实机器人硬件接口整合成可部署、可调试、可迁移的工程化系统。

### 项目功能

- 建图与里程计：基于 Livox MID360 内置 IMU 的 `FAST_LIO`。
- 可选图优化建图源码：`unused_packages/LIO-SAM`，默认不参与主链路构建。
- 传感器驱动：`livox_ros_driver2`。
- 点云转二维激光：`lidar_scan_bridge`。
- 全局定位：`gicp_localization` 发布 `map -> odom`。
- Nav2 导航：`my_nav` 中包含 Nav2 参数、行为树、虚拟障碍层以及适配 MPPI 的里程计反馈。
- 顶层启动：`quadruped_navigation_bringup`。
- 机器狗运动接口：`agibot_motion_service` 消费 `/cmd_vel` 并下发运动指令。
- 地图资产：`configs/` 和 bringup 安装配置中保留 2D 栅格地图与 3D PCD 地图。

### 我的主要工作

- 负责建图、定位、导航和避障相关功能的开发、集成、实机部署验证与参数调优。
- 搭建 MID360 + FAST-LIO + GICP + Nav2 的实机运行链路，并统一由 `quadruped_navigation_bringup` 作为顶层启动入口。
- 完成 Livox MID360、FAST-LIO、点云转 LaserScan、GICP 定位、Nav2 导航和 AgiBot 运动接口之间的联调。
- 迁移并完善 `wait_for_nav_ready`、`run_when_ready`，使 Nav2 在地图、TF、扫描、里程计和 FAST-LIO 时间滞后稳定后再启动或执行后续命令。
- 明确并验证坐标系约定：`base_link` 为机器狗机体中心，`base_footprint` 为水平投影，`livox_frame` 为位于机体中心正上方约 0.10 m 的 MID360 坐标系。
- 检查并修正 `/cloud_registered_body`、scan 转换和 `/odom.twist` 的坐标系行为，保证 Nav2 控制器读取到一致的机体系速度反馈。
- 以 GICP 作为 `map -> odom` 的定位源，完成实机初始化、定位稳定性和导航链路验证，避免与 AMCL 同时发布定位 TF。
- 调整优化 FAST-LIO、GICP、Nav2 costmap/controller、虚拟障碍等关键参数，提升实机导航和避障稳定性。
- 整理功能包命名、依赖声明、历史配置、unused 功能包、license/notice 和仓库结构，便于迁移到其他项目。

### 工程创新与技术亮点

- **三维建图到二维导航的稳定桥接**：FAST-LIO 维护三维点云地图和里程计，`lidar_scan_bridge` 输出 Nav2 可用的 `/scan`。
- **清晰的 TF 归属**：GICP 负责 `map -> odom`，FAST-LIO 负责 `odom -> base_footprint -> base_link`，静态外参负责 `base_link -> livox_frame`。
- **避免点云重复变换**：`/cloud_registered_body` 保持在 `livox_frame`，由 bridge 转到 `base_link`，避免重复施加雷达到机体中心的外参。
- **更严谨的 Nav2 速度反馈**：`/odom.twist` 按 `base_footprint` 坐标系发布，并提供 yaw 角速度反馈，降低 MPPI/控制器误判风险。
- **启动鲁棒性检查**：启动前检查地图、TF、scan、odom 新鲜度以及 FAST-LIO 时间滞后，减少半初始化状态下启动 Nav2 的问题。
- **FAST-LIO 输出链路优化**：针对四足机器人机体中心和 Nav2 控制反馈，对 FAST-LIO 的 odom、TF、点云 frame 和速度字段进行适配。
- **便于迁移的仓库结构**：当前链路功能包放在 `src/`，暂不用或历史参考包归档到 `unused_packages/`。

### 工程化特点

- **统一启动入口**：使用 `quadruped_navigation_bringup` 统一启动传感器、建图里程计、定位、导航、避障和运动接口。
- **启动时序可控**：通过 `wait_for_nav_ready` 等待 `/scan`、TF、odom 和 FAST-LIO lag 稳定后再启动 Nav2。
- **实机参数调优**：围绕 FAST-LIO、GICP、Nav2 costmap/controller 和虚拟障碍参数进行实机调试与优化。
- **坐标链路可追踪**：明确 `map -> odom -> base_footprint -> base_link -> livox_frame` 的 TF 归属，便于排查定位和避障问题。
- **功能包边界清晰**：active 包放在 `src/`，历史或可选硬件支持放在 `unused_packages/`，方便后续迁移和复用。

### 主启动方式

```bash
cd Quadruped-ROS2-Navigation
source install/setup.bash
ros2 launch quadruped_navigation_bringup navigation.launch.py use_rviz:=false
```

默认启动使用 `quadruped_navigation_bringup/config/` 中安装的 `map_nt_4_all.yaml` 和 `nt_4_all.pcd`。
如果切换测试环境，可以通过 `map:=...` 和 `pcd_map:=...` 覆盖。

启动后需要在 RViz2 或 `/initialpose` 中给定初始位姿，使 GICP 初始化 `map -> odom`。

常用启动参数：

```bash
start_livox:=true
start_fast_lio:=true
start_scan:=true
start_gicp:=true
start_nav2:=true
start_motion_service:=true
use_rviz:=true
```

不连接机器狗运动接口时：

```bash
start_motion_service:=false
```

### 运行链路

```text
Livox MID360
  -> livox_ros_driver2 发布 /livox/lidar 和 /livox/imu
  -> FAST_LIO 发布 /cloud_registered_body、/odom 和 odom -> base_footprint -> base_link
  -> lidar_scan_bridge 将 body 点云转换到 base_link 并发布 /scan
  -> gicp_localization 在 /initialpose 后发布 map -> odom
  -> Nav2 消费 /map、/odom、/scan 和 TF
  -> my_nav 虚拟障碍层支持路径阻塞后的重规划
  -> agibot_motion_service 消费 /cmd_vel 并下发机器狗运动指令
```

不要在该链路中同时启动 AMCL。当前定位源是 GICP，它负责发布 `map -> odom`。

### 坐标系约定

- `base_link`：机器狗机体中心
- `base_footprint`：`base_link` 的水平地面投影
- `livox_frame`：MID360 坐标系，位于 `base_link` 正上方约 0.10 m

### 编译

```bash
cd Quadruped-ROS2-Navigation
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble
source install/setup.bash
```

可能需要手动安装或配置的依赖：

- `small_gicp`：用于 `gicp_localization`
- AgiBot/ZsiBot SDK：如果替换或移除仓库内 SDK，需要为 `agibot_motion_service` 指定 SDK

### 建图说明

FAST-LIO 是当前 MID360 默认建图和里程计链路。可以在 `src/FAST_LIO/config/mid360.yaml` 中开启 PCD 保存，
运行系统后再按需要保存或转换点云地图。

`LIO-SAM` 已作为可选备份建图源码保留在 `unused_packages/LIO-SAM`，默认不会被 `colcon build` 构建。使用前请将其单独移回 `src/LIO-SAM` 或在独立工作区构建，并安装 GTSAM、按 MID360 话题/内置 IMU/外参重新适配参数。

### 可选或历史功能包

当前 MID360 + 内置 IMU 启动链路没有用到的功能包归档在 `unused_packages/`。该目录带有 `COLCON_IGNORE`，默认不会参与 colcon 发现和构建：

- `LIO-SAM`：可选备份图优化建图源码，默认不参与当前 MID360 + FAST-LIO + GICP + Nav2 主链路构建。
- `rslidar_msg`、`rsLiDAR_sdk`、`rs_driver`：RoboSense 雷达支持。
- `serial`、`yesense_interface`、`yesense_std_ros2`：外置串口/YESENSE IMU 支持。
- `legacy_agibot_driver`：旧 AgiBot bridge 链路参考包。当前链路使用 `agibot_motion_service`。

### 仓库说明

- 当前保留上游 media、PCD 地图、PGM 地图等大文件。
- 根目录 `LICENSE` 和 `NOTICE` 汇总混合许可证信息，各功能包本地许可证文件仍为准。
- ROS2 生成目录 `build/`、`install/`、`log/` 不应提交。

## English

This open-source repository provides a ROS2 mapping, localization, navigation,
and obstacle-avoidance engineering system for a quadruped robot running on real hardware.
It includes Livox MID360 sensor integration, FAST-LIO mapping and odometry,
GICP global localization, Nav2 autonomous navigation, point-cloud to 2D scan
conversion, virtual-obstacle avoidance, map assets, and the AgiBot motion
control interface. The goal is to integrate open-source algorithms, ROS2 navigation
components, and real robot hardware interfaces into a deployable, debuggable,
and portable engineering stack.

### Project Features

- Mapping and odometry: `FAST_LIO` with the Livox MID360 built-in IMU.
- Optional graph-SLAM source package: `unused_packages/LIO-SAM`, not built by default.
- Sensor driver: `livox_ros_driver2`.
- Point cloud to LaserScan bridge: `lidar_scan_bridge`.
- Global localization: `gicp_localization`, publishing `map -> odom`.
- Nav2 navigation: `my_nav` configs, behavior tree, MPPI-compatible odometry feedback, and virtual obstacle layer.
- Top-level startup: `quadruped_navigation_bringup`.
- Robot motion interface: `agibot_motion_service`, consuming `/cmd_vel`.
- Map assets: 2D Nav2 maps and 3D PCD maps under `configs/` and installed bringup configs.

### My Work

- Developed, integrated, deployed, validated on the physical robot, and tuned the mapping, localization, navigation, and obstacle-avoidance functions.
- Built the real-hardware MID360 + FAST-LIO + GICP + Nav2 runtime chain and consolidated startup under `quadruped_navigation_bringup`.
- Integrated and tested the Livox MID360 driver, FAST-LIO, point-cloud to LaserScan conversion, GICP localization, Nav2 navigation, and AgiBot motion interface.
- Migrated and refined readiness helpers (`wait_for_nav_ready`, `run_when_ready`) so Nav2 starts only after map, TF, scan, odometry, and FAST-LIO lag are stable.
- Defined and validated the robot frame convention: `base_link` is the quadruped body center, `base_footprint` is the planar projection, and `livox_frame` is the MID360 frame about 0.10 m above `base_link`.
- Checked and corrected frame-sensitive behavior around `/cloud_registered_body`, scan conversion, and `/odom.twist` so Nav2 receives consistent body-frame velocity feedback.
- Used GICP as the localization source that owns `map -> odom`, and validated initialization, localization stability, and navigation behavior on the real robot without AMCL TF conflicts.
- Tuned FAST-LIO, GICP, Nav2 costmap/controller, and virtual-obstacle parameters to improve real-robot navigation and obstacle-avoidance stability.
- Cleaned package names, dependency declarations, historical configs, unused packages, license notices, and repository structure for migration to other projects.

### Engineering and Technical Highlights

- **3D mapping plus 2D navigation bridge**: FAST-LIO maintains LiDAR-inertial odometry and 3D map data, while `lidar_scan_bridge` produces Nav2-compatible `/scan` data.
- **Clear TF ownership**: GICP owns `map -> odom`; FAST-LIO owns `odom -> base_footprint -> base_link`; static extrinsic uses `base_link -> livox_frame`.
- **Frame-safe point cloud handling**: `/cloud_registered_body` stays in `livox_frame`, then the bridge transforms it to `base_link`, avoiding double application of the LiDAR-to-body transform.
- **Nav2-ready odometry feedback**: `/odom.twist` is expressed in the `base_footprint` child frame, with yaw-rate feedback for MPPI/controller behavior.
- **Startup robustness**: readiness checks gate Nav2 startup using map, TF, scan, odometry freshness, and FAST-LIO lag diagnostics.
- **FAST-LIO output-chain optimization**: FAST-LIO odom, TF, point-cloud frame, and velocity feedback are adapted for the quadruped body center and Nav2 controllers.
- **Portable release structure**: active packages are kept in `src/`; optional or historical packages are archived in `unused_packages/` with clear notes.

### Engineering Characteristics

- **Unified bringup entry point**: `quadruped_navigation_bringup` starts sensors, mapping/odometry, localization, navigation, obstacle avoidance, and motion control.
- **Controlled startup sequence**: `wait_for_nav_ready` waits for `/scan`, TF, odom, and FAST-LIO lag to become stable before launching Nav2.
- **Real-robot parameter tuning**: FAST-LIO, GICP, Nav2 costmap/controller, and virtual-obstacle parameters are tuned around physical robot behavior.
- **Traceable frame chain**: TF ownership is explicit for `map -> odom -> base_footprint -> base_link -> livox_frame`, making localization and obstacle-avoidance issues easier to debug.
- **Clear package boundaries**: active packages are kept in `src/`, while historical or optional hardware-support packages are archived in `unused_packages/` for migration and reuse.

### Main Launch

```bash
cd Quadruped-ROS2-Navigation
source install/setup.bash
ros2 launch quadruped_navigation_bringup navigation.launch.py use_rviz:=false
```

The default launch uses the installed `map_nt_4_all.yaml` and `nt_4_all.pcd`
from `quadruped_navigation_bringup/config/`. Override them with `map:=...` and
`pcd_map:=...` when testing another environment.

After launch, publish an initial pose in RViz2 or through `/initialpose` so GICP
can initialize `map -> odom`.

Useful launch toggles:

```bash
start_livox:=true
start_fast_lio:=true
start_scan:=true
start_gicp:=true
start_nav2:=true
start_motion_service:=true
use_rviz:=true
```

For testing without the robot motion interface:

```bash
start_motion_service:=false
```

### Runtime Chain

```text
Livox MID360
  -> livox_ros_driver2 publishes /livox/lidar and /livox/imu
  -> FAST_LIO publishes /cloud_registered_body, /odom, and odom -> base_footprint -> base_link
  -> lidar_scan_bridge transforms the body cloud to base_link and publishes /scan
  -> gicp_localization publishes map -> odom after /initialpose
  -> Nav2 consumes /map, /odom, /scan, and TF
  -> my_nav virtual obstacle layer supports blocked-path replanning
  -> agibot_motion_service consumes /cmd_vel and sends commands to the robot
```

Do not run AMCL together with GICP in this stack. GICP is the localization source
that owns `map -> odom`.

Frame convention:

- `base_link`: quadruped body center
- `base_footprint`: horizontal ground projection of `base_link`
- `livox_frame`: MID360 frame mounted about 0.10 m above `base_link`

### Build

```bash
cd Quadruped-ROS2-Navigation
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble
source install/setup.bash
```

Manual/system dependencies that may not be fully resolved by rosdep:

- `small_gicp` for `gicp_localization`
- AgiBot/ZsiBot SDK libraries for `agibot_motion_service` if the bundled SDK is removed or replaced

### Mapping Notes

FAST-LIO is the current default mapping/odometry path for MID360. Enable PCD
saving in `src/FAST_LIO/config/mid360.yaml`, run the stack, then save or convert
the generated PCD as needed.

`LIO-SAM` is archived as optional backup mapping source under `unused_packages/LIO-SAM` and is not built by default. To use it, move it back to `src/LIO-SAM` or build it in a separate workspace, install GTSAM, and adapt the topics, MID360 built-in IMU assumptions, and extrinsics first.

### Optional Packages

Packages not used by the current MID360 + built-in IMU launch chain are kept in
`unused_packages/`. This directory has `COLCON_IGNORE`, so it is skipped by default colcon discovery:

- `LIO-SAM`: optional backup graph-SLAM mapping source, not part of the default MID360 + FAST-LIO + GICP + Nav2 build.
- `rslidar_msg`, `rsLiDAR_sdk`, `rs_driver`: RoboSense LiDAR support.
- `serial`, `yesense_interface`, `yesense_std_ros2`: external serial/YESENSE IMU support.
- `legacy_agibot_driver`: reference copy of the old AgiBot bridge path. The current stack uses `agibot_motion_service`.

### Repository Notes

- Large upstream media files, PCD maps, and PGM maps are intentionally kept for now.
- Root `LICENSE` and `NOTICE` summarize the mixed-license workspace. Package-local license files remain authoritative.
- Generated ROS 2 build outputs (`build/`, `install/`, `log/`) should not be committed.
