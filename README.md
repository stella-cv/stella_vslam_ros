# stella_vslam_ros

[stella_vslam](https://github.com/stella-cv/stella_vslam)'s ROS package.

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
- `~/pointcloud`
- `/tf`

## Parameters

- `odom_frame`
- `map_frame`
- `robot_base_frame`
- `camera_frame`
- `publish_tf`
- `publish_pointcloud`
- `transform_tolerance`
- `use_exact_time` (stereo, RGBD only)
