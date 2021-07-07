# openvslam_ros

[OpenVSLAM](https://github.com/OpenVSLAM-Community/openvslam)'s ROS package.

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
- `publish_tf`
- `transform_tolerance`


# Running openvslam on ros2

## Installation using docker

- `Instructions for SocketViewer` 
- `Instructions for PangolinViewer` - `Comming soon`

# Instructions for SocketViewer
`Dockerfile.socket` can be used for easy installation. This chapter provides instructions on building and running examples with SocketViewer support using Docker.

## Building Docker Images

### Docker Image of OpenVSLAM

### Step 1: Clone the required github repository:

If you are going to use non RGBD setup then exceute following in your terminal.

    git clone https://github.com/OpenVSLAM-Community/openvslam_ros.git

If you are going to use RGBD setup then exceute following in your terminal.

    git clone -b wip/approximate-time-rgbd https://github.com/OpenVSLAM-Community/openvslam_ros.git

In case of openvslam using rosbag file, then replace `Dockerfile.socket` with `Dockerfile.socket-rosbag`.

### Step 2: Create docker image

Execute the following commands:

    cd /path/to/openvslam
    git submodule update -i --recursive
    docker build -t openvslam-ros-socket -f Dockerfile.socket .

You can accelerate the build of the docker image with `--build-arg NUM_THREADS=<number of parallel builds> ` option in command line argument.

For example:

    # building the docker image with four threads
    docker build -t openvslam-socket -f Dockerfile.socket . --build-arg NUM_THREADS=4

### Step 3: Docker Image of Server
Copy viewer folder from the main 'openvslam' [repository](https://github.com/OpenVSLAM-Community/openvslam) to your present working direcory i.e 'openvslam_ros'. Then create docker image for socket server.

Execute the following commands:

    git clone https://github.com/OpenVSLAM-Community/openvslam.git openvslam
    cp -r openvslam/viewer ./viewer
    rm -r openvslam/ --force
    cd viewer
    docker build -t openvslam-ros-server .

### Step 4: Starting Docker Containers for "openvslam-ros-server"

### On Linux
Launch the server container and access to it with the web browser in advance. Please specify `--net=host` in order to share the network with the host machine.

Execute the following commands:

`Terminal1:`

    docker run --rm -it --name openvslam-ros-server --net=host openvslam-ros-server

Following message should appear on your terminal.

    WebSocket: listening on *:3000
    HTTP server: listening on *:3001

After launching, access to `http://localhost:3001/` with the web browser.

### Step 5: Starting Docker Containers for "openvslam-ros-socket"

### Step 5.1: Using USB Camera

Execute the following commands:


`Terminal2:`
Launch the container of OpenVSLAM-socket. The shell interface will be launched in the docker container.Download an ORB vocabulary from GitHub. Publish Images Captured by a USB Camera using package like image_tools.

    xhost +local:
    docker run --rm -it --name openvslam-ros-socket --device /dev/video0 --net=host openvslam-ros-socket
    root@hostname:/ros2_ws#curl -sL "https://github.com/OpenVSLAM-Community/FBoW_orb_vocab/raw/main/orb_vocab.fbow" -o orb_vocab.fbow
    root@hostname:/ros2_ws#ros2 run image_tools cam2image
    

`Terminal3:`
Republish the ROS topic to `/camera/image_raw`.

    docker exec -it openvslam-ros-socket /bin/bash
    root@hostname:/ros2_ws#source /opt/ros/galactic/setup.bash
    root@hostname:/ros2_ws#source /ros2_ws/install/setup.bash
    root@hostname:/ros2_ws#ros2 run image_transport republish raw in:=image raw out:=/camera/image_raw

`Terminal4:`
Run it in slam mode for run tracking and mapping. Please memtion the appropriate path to your usb camera config.yaml file in below command.

    docker exec -it openvslam-ros-socket /bin/bash
    root@hostname:/ros2_ws#source /opt/ros/galactic/setup.bash
    root@hostname:/ros2_ws#source /ros2_ws/install/setup.bash
    root@hostname:/ros2_ws#ros2 run openvslam_ros run_slam -v ./orb_vocab.fbow -c /path/to/config.yaml --map-db ./usb_camera.msg

You can see the mapping and tracking in your localhost browser.
Once mapping is done, click the [Terminate] button to close the viewer.
you can find usb_camera.msg in the current directory.

`Terminal1:`
Stop previous running image tools.Then run below again.

    root@hostname:/ros2_ws#ros2 run image_tools cam2image

`Terminal2:`
Stop previous running image_transport. Then run below again.

    root@hostname:/ros2_ws#ros2 run image_transport republish raw in:=image raw out:=/camera/image_raw

`Terminal3:`
Run localization

    root@hostname:/ros2_ws#ros2 run openvslam_ros run_localization -v ./orb_vocab.fbow -c /path/to/config.yaml --map-db ./usb_camera.msg

### Step 5.2: Using VIDEO file

Execute the following commands:

`Terminal2:`
Launch the container of OpenVSLAM-socket. The shell interface will be launched in the docker container.
Download an ORB vocabulary from GitHub and download a sample dataset from Google Drive for mapping and localization respectively.

    docker run --rm -it --name openvslam-ros-socket --net=host openvslam-ros-socket
    OR if you would like to mount a folder (ex:Inputs) to access all required files like config.yaml, rosbag.bag etc then you can run below:
    docker run --rm -it --name openvslam-ros-socket --net=host -v $PWD/Inputs:/ros2_ws/Inputs openvslam-ros-socket
    
    root@hostname:/ros2_ws#curl -sL "https://github.com/OpenVSLAM-Community/FBoW_orb_vocab/raw/main/orb_vocab.fbow" -o orb_vocab.fbow
    root@hostname:/ros2_ws#FILE_ID="1d8kADKWBptEqTF7jEVhKatBEdN7g0ikY"
    root@hostname:/ros2_ws#curl -sc /tmp/cookie "https://drive.google.com/uc?export=download&id=${FILE_ID}" > /dev/null
    root@hostname:/ros2_ws#CODE="$(awk '/_warning_/ {print $NF}' /tmp/cookie)"
    root@hostname:/ros2_ws#curl -sLb /tmp/cookie "https://drive.google.com/uc?export=download&confirm=${CODE}&id=${FILE_ID}" -o aist_lInputsiving_lab_1.zip
    root@hostname:/ros2_ws#unzip aist_living_lab_1.zip

    root@hostname:/ros2_ws#FILE_ID="1TVf2D2QvMZPHsFoTb7HNxbXclPoFMGLX"
    root@hostname:/ros2_ws#curl -sc /tmp/cookie "https://drive.google.com/uc?export=download&id=${FILE_ID}" > /dev/null
    root@hostname:/ros2_ws#CODE="$(awk '/_warning_/ {print $NF}' /tmp/cookie)"
    root@hostname:/ros2_ws#curl -sLb /tmp/cookie "https://drive.google.com/uc?export=download&confirm=${CODE}&id=${FILE_ID}" -o aist_living_lab_2.zip
    root@hostname:/ros2_ws#unzip aist_living_lab_2.zip 

Clone the repository and build it. Then publish video images.
    
    root@hostname:/ros2_ws#git clone https://github.com/mirellameelo/dataset_publisher_ros2.git
    root@hostname:/ros2_ws#cd dataset_publisher_ros2
    root@hostname:/ros2_ws#colcon build
    root@hostname:/ros2_ws#source install/setup.bash
    root@hostname:/ros2_ws#cd ..                     
    root@hostname:/ros2_ws#ros2 run publisher openvslam_video -m ./aist_living_lab_1/video.mp4 

`Terminal3:`

    docker exec -it openvslam-ros-socket /bin/bash
    root@hostname:/ros2_ws#source /opt/ros/galactic/setup.bash
    root@hostname:/ros2_ws#source /ros2_ws/install/setup.bash
    root@hostname:/ros2_ws#ros2 run image_transport republish raw in:=image raw out:=/camera/image_raw

Images will be published in camera/image_raw topic and frame_id is set to camera_link


`Terminal4:`

    docker exec -it openvslam-ros-socket /bin/bash
    root@hostname:/ros2_ws#source /opt/ros/galactic/setup.bash
    root@hostname:/ros2_ws#source /ros2_ws/install/setup.bash
    root@hostname:/ros2_ws#ros2 run openvslam_ros run_slam -v ./orb_vocab.fbow -c /path/to/config.yaml --frame-skip 1 --map-db aist_living_lab_1_map.msg

You can see the mapping and tracking in your localhost browser.
Once mapping is done, click the [Terminate] button to close the viewer.
you can find aist_living_lab_1_map.msg in the current directory.
       

`Terminal2:`
Stop previous running image tools.Then run below again.

    root@hostname:/ros2_ws#ros2 run publisher openvslam_video -m ../aist_living_lab_2/video.mp4

`Terminal3:`
Stop previous running image_transport. Then run below again.

    root@hostname:/ros2_ws#ros2 run image_transport republish raw in:=image raw out:=/camera/image_raw

`Terminal4:`
Run localization

    root@hostname:/ros2_ws#ros2 run openvslam_ros run_localization -v ./orb_vocab.fbow -c /path/to/config.yaml --frame-skip 1 --map-db aist_living_lab_1_map.msg

### Step 5.3: Using ROSBAG file
Execute the following commands:

`Terminal2:`
Launch container of openvslam-ros-socket. The shell interface will be launched in the docker container.
    
    docker run --rm -it --name openvslam-ros-socket --net=host openvslam-ros-socket   
OR if you would like to mount a folder (ex:Inputs) to access all required files like config.yaml, rosbag.bag etc then you can run below:
    
    docker run --rm -it --name openvslam-ros-socket --net=host -v $PWD/Inputs:/ros2_ws/Inputs openvslam-ros-socket
    root@hostname:/ros2_ws#ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link

`Terminal3:`
    
    docker exec -it openvslam-ros-socket /bin/bash
    root@hostname:/ros2_ws#source /opt/ros/${ROS2_DISTRO}/setup.bash
    root@hostname:/ros2_ws#source /ros2_ws/install/setup.bash
    root@hostname:/ros2_ws#ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link cam0
    

`Terminal4:`
    
    docker exec -it openvslam-ros-socket /bin/bash
    root@hostname:/ros2_ws#source /opt/ros/${ROS2_DISTRO}/setup.bash
    root@hostname:/ros2_ws#source /ros2_ws/install/setup.bash
    root@hostname:/ros2_ws#ros2 run openvslam_ros run_slam -v ./orb_vocab.fbow -c /path/to/config.yaml --frame-skip 1 --map-db /path/to/message.msg --ros-args -r /camera/image_raw:=/cam0/image_raw


`Terminal5:`

Build docker image with rosbag installed and launch container of openvslam-rosbag.
    

    docker build -t openvslam-rosbag -f Dockerfile.rosbag .    
    docker run --rm -it --name openvslam-rosbag --net=host openvslam-rosbag
    OR
    docker run --rm -it --name openvslam-rosbag --net=host -v $PWD/Inputs:/ros2_ws/Inputs openvslam-rosbag
  
    root@hostname:/ros2_ws#source /opt/ros/${ROS1_DISTRO}/setup.bash
    root@hostname:/ros2_ws#source /opt/ros/foxy/setup.bash
    root@hostname:/ros2_ws#ros2 bag play -s rosbag_v2 /path/to/rosbagfile.bag


You can see the mapping and tracking in your localhost browser.
Once mapping is done, click the [Terminate] button to close the viewer.
you can find aist_living_lab_1_map.msg in the current directory.
       

`Terminal2:`
Stop previous running tf2_ros.Then run below again.

    root@hostname:/ros2_ws#ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link

`Terminal3:`
Stop previous running tf2_ros. Then run below again.

    root@hostname:/ros2_ws#ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link cam0

`Terminal4:`
Run localization

    root@hostname:/ros2_ws#ros2 run openvslam_ros run_localization -v ./orb_vocab.fbow -c /path/to/config.yaml --frame-skip 1 --map-db /path/to/message.msg --ros-args -r /camera/image_raw:=/cam0/image_raw

`Terminal5:`

    root@hostname:/ros2_ws#ros2 bag play -s rosbag_v2 /path/to/rosbagfile.bag

