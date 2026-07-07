# Optional and Legacy Packages

These packages are intentionally outside the active `src/` build tree. They are
kept for reference or future hardware variants and are not part of the default
MID360 + FAST-LIO + GICP + Nav2 startup chain.

## Keep As Optional Hardware Support

- `livox_repub`: Livox `CustomMsg` to `PointCloud2` bridge for diagnostics.
- `rslidar_msg`, `rsLiDAR_sdk`, `rs_driver`: RoboSense LiDAR stack.
- `serial`, `yesense_interface`, `yesense_std_ros2`: YESENSE external IMU stack.

## Keep As Legacy Reference

- `legacy_agibot_driver`: old AgiBot bridge implementation retained as a
  reference. The current robot command path is `src/agibot_motion_service`.

## Removed From This Folder During Cleanup

- Empty `BehaviorTree.CPP` placeholder directory.
- Historical `lidar_scan_bridge_legacy_files` and `lidar_scan_bridge_legacy_launch`
  backups that referenced the old `agibot_nav2_bringup` chain.
