# System Integrity Check / 系统完整性检查

This document records the intended runtime chain for the current release.

本文档记录当前整理版本的预期运行链路和检查要点。

## Active Entry Point

```bash
ros2 launch quadruped_navigation_bringup navigation.launch.py
```

`quadruped_navigation_bringup` is the only top-level startup package. It includes
the sensor/localization chain from `my_nav/gicp_navigation.launch.py`, starts the
AgiBot motion service when enabled, waits for navigation readiness, and can run
a post-ready command.

## Expected Runtime Chain

```text
/livox/lidar, /livox/imu
  -> FAST_LIO
  -> /cloud_registered_body, /odom, odom -> base_footprint -> base_link
  -> lidar_scan_bridge
  -> /scan
  -> gicp_localization, map -> odom
  -> Nav2
  -> /cmd_vel
  -> agibot_motion_service
```

## TF Ownership

- `map -> odom`: `gicp_localization`
- `odom -> base_footprint -> base_link`: `FAST_LIO`
- `base_link -> livox_frame`: static transform, MID360 about 0.10 m above the body center

Do not start AMCL in this GICP-based chain.

## Current Release Notes

- `/cloud_registered_body` is intentionally published in `livox_frame`.
- `lidar_scan_bridge` transforms the cloud into `base_link` before generating `/scan`.
- `/odom.twist` is expressed in `base_footprint`, with yaw-rate feedback.
- `unused_packages/` keeps optional or historical packages outside the active startup chain.
- Large map/media assets are intentionally kept for now.
