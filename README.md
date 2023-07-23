# stella_vslam_ros

[stella_vslam](https://github.com/stella-cv/stella_vslam)'s ROS package.

## Documentation

See [here](https://stella-cv.readthedocs.io/en/latest/ros2_package.html).

## Install instruction

stella_vslam_ros uses submodules. Clone it with `git clone --recursive` or download submodules with `git submodule update --init --recursive`.

## Subscribed topics

### monocular setup

- `camera/image_raw`

### stereo setup

- `camera/left/image_raw`
- `camera/right/image_raw`

### RGBD setup

- `camera/color/image_raw`
- `camera/depth/image_raw`

## Published topics

- `~/camera_pose`

## Parameters

- `odom_frame`
- `map_frame`
- `robot_base_frame`
- `camera_frame`
- `publish_tf`
- `transform_tolerance`
- `use_exact_time` (stereo, RGBD only)

## Tutorial

Users unfamiliar with docker, ROS2, or stella_vslam should refer to the [tutorial](/doc/tutorial.md).

## TIPS

- run_slam_offline node allows you to run SLAM by reading the rosbag2 file directly.
