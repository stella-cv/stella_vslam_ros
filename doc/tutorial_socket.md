# Instructions for SocketViewer

`Dockerfile.socket` can be used for easy installation. This chapter provides instructions on building and running examples with SocketViewer support using Docker.

## Building Docker Images

### Step 1: Clone the required github repository

If you are going to use non RGBD setup then exceute following in your terminal.

```shell
~$ git clone --recursive https://github.com/stella-cv/stella_vslam_ros.git
```

(If you are going to use RGBD setup then refer <https://github.com/stella-cv/stella_vslam_ros/issues/32>.)

### Step 2: Create docker image

Execute the following commands:

```shell
~$ cd stella_vslam_ros
~/stella_vslam_ros$ docker build -t stella_vslam-ros-socket -f Dockerfile.socket .
```

You can accelerate the build of the docker image with `--build-arg NUM_THREADS=<number of parallel builds>` option in command line argument as follows:

```shell
# building the docker image with four threads
~/stella_vslam_ros$ docker build -t stella_vslam-ros-socket -f Dockerfile.socket . --build-arg NUM_THREADS=4
```

### Step 3: Docker Image of Server

Execute the following commands:

```shell
~$ git clone --recursive https://github.com/stella-cv/stella_vslam.git
~$ cd stella_vslam/viewer
~/stella_vslam/viewer$ docker build -t stella_vslam-ros-server .
```

### Step 4: Start Docker Containers for "stella_vslam-ros-server"

### On Linux

Launch the server container and access to it with the web browser in advance. Please specify `--net=host` in order to share the network with the host machine.

Execute the following commands:

Terminal1:

```shell
~$ docker run --rm -it --name stella_vslam-ros-server --net=host stella_vslam-ros-server
```

Following message should appear on your terminal.

```shell-session
WebSocket: listening on *:3000
HTTP server: listening on *:3001
```

After launching, access to `http://localhost:3001/` with the web browser.

### Step 5: Start Docker Containers for "stella_vslam-ros-socket"

### Step 5.1: Using USB Camera

Execute the following commands:

Terminal2:

Launch the container of stella_vslam-ros-socket. The shell interface will be launched in the docker container.
Download an ORB vocabulary from GitHub. Publish Images Captured by a USB Camera using package like image_tools.

```shell
~$ xhost +local:
~$ docker run --rm -it --name stella_vslam-ros-socket --device /dev/video0 --net=host stella_vslam-ros-socket
```

```shell-session
root@hostname:/ros2_ws# curl -sL "https://github.com/stella-cv/FBoW_orb_vocab/raw/main/orb_vocab.fbow" -o orb_vocab.fbow
root@hostname:/ros2_ws# ros2 run image_tools cam2image
```

Terminal3:

Republish the ROS topic to `/camera/image_raw`.

```shell
~$ docker exec -it stella_vslam-ros-socket /bin/bash
```

```shell-session
root@hostname:/ros2_ws# source /opt/ros/${ROS_DISTRO}/setup.bash
root@hostname:/ros2_ws# source /ros2_ws/install/setup.bash
root@hostname:/ros2_ws# ros2 run image_transport republish raw in:=image raw out:=/camera/image_raw
```

Terminal4:

Run it in slam mode for run tracking and mapping. Please memtion the appropriate path to your usb camera config.yaml file in below command.

```shell
~$ docker exec -it stella_vslam-ros-socket /bin/bash
```

```shell-session
root@hostname:/ros2_ws# source /opt/ros/${ROS_DISTRO}/setup.bash
root@hostname:/ros2_ws# source /ros2_ws/install/setup.bash
root@hostname:/ros2_ws# ros2 run stella_vslam_ros run_slam -v ./orb_vocab.fbow -c /path/to/config.yaml --map-db-out ./usb_camera.msg
```

You can see the mapping and tracking in your localhost browser.
Once mapping is done, click the [Terminate] button to close the viewer.
you can find usb_camera.msg in the current directory.

Terminal1:

Stop previous running image tools.Then run below again.

```shell-session
root@hostname:/ros2_ws# ros2 run image_tools cam2image
```

Terminal2:

Stop previous running image_transport. Then run below again.

```shell-session
root@hostname:/ros2_ws# ros2 run image_transport republish raw in:=image raw out:=/camera/image_raw
```

Terminal3:

Run localization

```shell-session
root@hostname:/ros2_ws# ros2 run stella_vslam_ros run_slam --disable-mapping -v ./orb_vocab.fbow -c /path/to/config.yaml --map-db-in ./usb_camera.msg
```

### Step 5.2: Using VIDEO file

Execute the following commands:

Terminal2:

Launch the container of stella_vslam-ros-socket. The shell interface will be launched in the docker container.
Download an ORB vocabulary from GitHub and download a sample dataset from Google Drive for mapping and localization respectively.

```shell
~$ docker run --rm -it --name stella_vslam-ros-socket --net=host stella_vslam-ros-socket
```

OR if you would like to mount a folder (ex:Inputs) to access all required files like config.yaml, rosbag.bag etc then you can run below:

```shell
~$ docker run --rm -it --name stella_vslam-ros-socket --net=host -v $PWD/Inputs:/ros2_ws/Inputs stella_vslam-ros-socket
```

```shell-session
root@hostname:/ros2_ws# curl -sL "https://github.com/stella-cv/FBoW_orb_vocab/raw/main/orb_vocab.fbow" -o orb_vocab.fbow
root@hostname:/ros2_ws# FILE_ID="1d8kADKWBptEqTF7jEVhKatBEdN7g0ikY"
root@hostname:/ros2_ws# curl -sc /tmp/cookie "https://drive.google.com/uc?export=download&id=${FILE_ID}" > /dev/null
root@hostname:/ros2_ws# CODE="$(awk '/_warning_/ {print $NF}' /tmp/cookie)"
root@hostname:/ros2_ws# curl -sLb /tmp/cookie "https://drive.google.com/uc?export=download&confirm=${CODE}&id=${FILE_ID}" -o aist_lInputsiving_lab_1.zip
root@hostname:/ros2_ws# unzip aist_living_lab_1.zip

root@hostname:/ros2_ws# FILE_ID="1TVf2D2QvMZPHsFoTb7HNxbXclPoFMGLX"
root@hostname:/ros2_ws# curl -sc /tmp/cookie "https://drive.google.com/uc?export=download&id=${FILE_ID}" > /dev/null
root@hostname:/ros2_ws# CODE="$(awk '/_warning_/ {print $NF}' /tmp/cookie)"
root@hostname:/ros2_ws# curl -sLb /tmp/cookie "https://drive.google.com/uc?export=download&confirm=${CODE}&id=${FILE_ID}" -o aist_living_lab_2.zip
root@hostname:/ros2_ws# unzip aist_living_lab_2.zip 
```

Clone the repository and build it. Then publish video images.

```shell-session
root@hostname:/ros2_ws# git clone https://github.com/mirellameelo/dataset_publisher_ros2.git
root@hostname:/ros2_ws# cd dataset_publisher_ros2
root@hostname:/ros2_ws# colcon build
root@hostname:/ros2_ws# source install/setup.bash
root@hostname:/ros2_ws# cd ..                     
root@hostname:/ros2_ws# ros2 run publisher openvslam_video -m ./aist_living_lab_1/video.mp4 
```

Terminal3:

```shell
~$ docker exec -it stella_vslam-ros-socket /bin/bash
```

```shell-session
root@hostname:/ros2_ws# source /opt/ros/${ROS_DISTRO}/setup.bash
root@hostname:/ros2_ws# source /ros2_ws/install/setup.bash
root@hostname:/ros2_ws# ros2 run image_transport republish raw in:=image raw out:=/camera/image_raw
```

Images will be published in camera/image_raw topic and frame_id is set to camera_link

Terminal4:

```shell
~$ docker exec -it stella_vslam-ros-socket /bin/bash
```

```shell-session
root@hostname:/ros2_ws# source /opt/ros/${ROS_DISTRO}/setup.bash
root@hostname:/ros2_ws# source /ros2_ws/install/setup.bash
root@hostname:/ros2_ws# ros2 run stella_vslam_ros run_slam -v ./orb_vocab.fbow -c /path/to/config.yaml --frame-skip 1 --map-db-out aist_living_lab_1_map.msg
```

You can see the mapping and tracking in your localhost browser.
Once mapping is done, click the [Terminate] button to close the viewer.
you can find aist_living_lab_1_map.msg in the current directory.

Terminal2:

Stop previous running image tools.Then run below again.

```shell-session
root@hostname:/ros2_ws# ros2 run publisher openvslam_video -m ../aist_living_lab_2/video.mp4
```

Terminal3:

Stop previous running image_transport. Then run below again.

```shell-session
root@hostname:/ros2_ws# ros2 run image_transport republish raw in:=image raw out:=/camera/image_raw
```

Terminal4:

Run localization

```shell-session
root@hostname:/ros2_ws# ros2 run stella_vslam_ros run_slam --disable-mapping -v ./orb_vocab.fbow -c /path/to/config.yaml --frame-skip 1 --map-db-in aist_living_lab_1_map.msg
```

### Step 5.3: Using ROSBAG file

Execute the following commands:

Terminal2:

Launch container of stella_vslam-ros-socket. The shell interface will be launched in the docker container.

```shell
~$ docker run --rm -it --name stella_vslam-ros-socket --net=host stella_vslam-ros-socket   
```

OR if you would like to mount a folder (ex:Inputs) to access all required files like config.yaml, rosbag.bag etc then you can run below:

```shell
~$ docker run --rm -it --name stella_vslam-ros-socket --net=host -v $PWD/Inputs:/ros2_ws/Inputs stella_vslam-ros-socket
```

```shell-session
root@hostname:/ros2_ws# ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link
```

Terminal3:

```shell
~$ docker exec -it stella_vslam-ros-socket /bin/bash
```

```shell-session
root@hostname:/ros2_ws# source /opt/ros/${ROS_DISTRO}/setup.bash
root@hostname:/ros2_ws# source /ros2_ws/install/setup.bash
root@hostname:/ros2_ws# ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link cam0
```

Terminal4:

```shell
~$ docker exec -it stella_vslam-ros-socket /bin/bash
```

```shell-session
root@hostname:/ros2_ws# source /opt/ros/${ROS_DISTRO}/setup.bash
root@hostname:/ros2_ws# source /ros2_ws/install/setup.bash
root@hostname:/ros2_ws# ros2 run stella_vslam_ros run_slam -v ./orb_vocab.fbow -c /path/to/config.yaml --frame-skip 1 --map-db-out /path/to/message.msg --ros-args -r /camera/image_raw:=/cam0/image_raw
```

Terminal5:

Build docker image with rosbag installed and launch container of stella_vslam-rosbag.

```shell
~$ docker build -t stella_vslam-rosbag -f Dockerfile.rosbag .    
~$ docker run --rm -it --name stella_vslam-rosbag --net=host stella_vslam-rosbag
# OR
# ~$ docker run --rm -it --name stella_vslam-rosbag --net=host -v $PWD/Inputs:/ros2_ws/Inputs stella_vslam-rosbag
```

```shell-session
root@hostname:/# source /opt/ros/${ROS_DISTRO}/setup.bash
root@hostname:/# rosbags-convert /datasets/EuRoC/MH_04_difficult.bag ./MH_04_difficult
root@hostname:/# ros2 bag play /path/to/rosbagfile.bag ./rosbagfile
root@hostname:/# ros2 bag play ./rosbagfile
```

You can see the mapping and tracking in your localhost browser.
Once mapping is done, click the [Terminate] button to close the viewer.
you can find message.msg in the directory mentioned in Terminal 4.

Terminal2:

Stop previous running tf2_ros.Then run below again.

```shell-session
root@hostname:/ros2_ws# ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link
```

Terminal3:

Stop previous running tf2_ros. Then run below again.

```shell-session
root@hostname:/ros2_ws# ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link cam0
```

Terminal4:

Run localization

```shell-session
root@hostname:/ros2_ws# ros2 run stella_vslam_ros run_slam --disable-mapping -v ./orb_vocab.fbow -c /path/to/config.yaml --frame-skip 1 --map-db-in /path/to/message.msg --ros-args -r /camera/image_raw:=/cam0/image_raw
```

Terminal5:

```shell-session
root@hostname:/# ros2 bag play /path/to/rosbagfile.bag ./rosbagfile
root@hostname:/# ros2 bag play ./rosbagfile
```
