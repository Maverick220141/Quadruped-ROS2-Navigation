# GICP Navigation Runtime Contract

This is the canonical runtime chain for the MID360 + FAST-LIO + GICP + Nav2 stack.

## Main Data Flow

1. `livox_ros_driver2`
   - Publishes `/livox/lidar` as `livox_ros_driver2/msg/CustomMsg`.
   - Publishes `/livox/imu`.
   - Frame: `livox_frame`.

2. `fast_lio`
   - Subscribes `/livox/lidar` and `/livox/imu`.
   - Publishes `/cloud_registered_body` in `livox_frame`.
   - Publishes `/odom` through the launch remap from `/odom_2d`.
   - Owns odometry-side motion and publishes `odom -> base_footprint -> base_link`.

3. `lidar_scan_bridge`
   - Transforms `/cloud_registered_body` from `livox_frame` to `base_link`.
   - Converts the body-centered cloud to `/scan_raw`.
   - Republishes `/scan_raw` to `/scan` with `frame_id=base_link`.
   - Nav2 costmaps consume `/scan`.

4. `gicp_localization`
   - Subscribes `/cloud_registered_body`.
   - Subscribes `/initialpose`.
   - Publishes `/gicp_pose`.
   - Owns `map -> odom`.
   - It must be the only localization node publishing `map -> odom`.

5. `my_nav`
   - Starts Nav2 navigation without AMCL.
   - Starts `map_server`.
   - Nav2 consumes:
     - `/map`
     - `/scan`
     - `/odom`
     - TF tree `map -> odom -> base_*`

## TF Ownership

- `map -> odom`: `gicp_localization/gicp_localization_node`
- `odom -> base_footprint -> base_link`: FAST-LIO odometry chain
- `base_link -> livox_frame`: static transform in `gicp_navigation.launch.py`; default translation is `z=0.10 m`
- Do not start AMCL with GICP.
- Do not run another static `base_footprint -> base_link` publisher if FAST-LIO or the robot already publishes it.

## Canonical Launch

Use one entrypoint:

```bash
ros2 launch quadruped_navigation_bringup navigation.launch.py
```

Direct package-level launch is still available when custom map paths are needed:

```bash
ros2 launch my_nav gicp_navigation.launch.py \
  map:=/absolute/path/to/map.yaml \
  pcd_map:=/absolute/path/to/map.pcd \
  nav2_params:=/absolute/path/to/nav2_params_gicp.yaml
```

Optional switches:

```bash
start_livox:=true
start_fast_lio:=true
start_scan:=true
start_gicp:=true
start_nav2:=true
use_rviz:=false
```

## Debug Topics

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /cloud_registered_body
ros2 topic hz /scan
ros2 topic echo /gicp_pose --once
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo odom base_link
```

