FROM ros:humble
ENV DEBIAN_FRONTEND noninteractive

# install dependencies via apt
ENV DEBCONF_NOWARNINGS yes
RUN set -x && \
  apt-get update -y -qq && \
  apt-get upgrade -y -qq --no-install-recommends && \
  : "basic dependencies" && \
  apt-get install -y -qq \
    build-essential \
    pkg-config \
    cmake \
    git \
    wget \
    curl \
    tar \
    unzip && \
  : "g2o dependencies" && \
  apt-get install -y -qq \
    libgoogle-glog-dev \
    libatlas-base-dev \
    libsuitesparse-dev && \
  : "Pangolin dependencies" && \
  apt-get install -y -qq \
    libglew-dev && \
  : "backward-cpp dependencies" && \
  apt install -y -qq binutils-dev && \
  : "other dependencies" && \
  apt-get install -y -qq \
    libyaml-cpp-dev \ 
    libopencv-dev \
    libeigen3-dev && \
  : "remove cache" && \
  apt-get autoremove -y -qq && \
  rm -rf /var/lib/apt/lists/*

ARG CMAKE_INSTALL_PREFIX=/usr/local
ARG NUM_THREADS=1

ENV CPATH=${CMAKE_INSTALL_PREFIX}/include:${CPATH}
ENV C_INCLUDE_PATH=${CMAKE_INSTALL_PREFIX}/include:${C_INCLUDE_PATH}
ENV CPLUS_INCLUDE_PATH=${CMAKE_INSTALL_PREFIX}/include:${CPLUS_INCLUDE_PATH}
ENV LIBRARY_PATH=${CMAKE_INSTALL_PREFIX}/lib:${LIBRARY_PATH}
ENV LD_LIBRARY_PATH=${CMAKE_INSTALL_PREFIX}/lib:${LD_LIBRARY_PATH}

ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Pangolin
ARG PANGOLIN_COMMIT=ad8b5f83222291c51b4800d5a5873b0e90a0cf81
WORKDIR /tmp
RUN set -x && \
  git clone https://github.com/stevenlovegrove/Pangolin.git && \
  cd Pangolin && \
  git checkout ${PANGOLIN_COMMIT} && \
  sed -i -e "193,198d" ./src/utils/file_utils.cpp && \
  mkdir -p build && \
  cd build && \
  cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_PANGOLIN_DEPTHSENSE=OFF \
    -DBUILD_PANGOLIN_FFMPEG=OFF \
    -DBUILD_PANGOLIN_LIBDC1394=OFF \
    -DBUILD_PANGOLIN_LIBJPEG=OFF \
    -DBUILD_PANGOLIN_LIBOPENEXR=OFF \
    -DBUILD_PANGOLIN_LIBPNG=OFF \
    -DBUILD_PANGOLIN_LIBREALSENSE=OFF \
    -DBUILD_PANGOLIN_LIBREALSENSE2=OFF \
    -DBUILD_PANGOLIN_LIBTIFF=OFF \
    -DBUILD_PANGOLIN_LIBUVC=OFF \
    -DBUILD_PANGOLIN_LZ4=OFF \
    -DBUILD_PANGOLIN_OPENNI=OFF \
    -DBUILD_PANGOLIN_OPENNI2=OFF \
    -DBUILD_PANGOLIN_PLEORA=OFF \
    -DBUILD_PANGOLIN_PYTHON=OFF \
    -DBUILD_PANGOLIN_TELICAM=OFF \
    -DBUILD_PANGOLIN_TOON=OFF \
    -DBUILD_PANGOLIN_UVC_MEDIAFOUNDATION=OFF \
    -DBUILD_PANGOLIN_V4L=OFF \
    -DBUILD_PANGOLIN_VIDEO=OFF \
    -DBUILD_PANGOLIN_ZSTD=OFF \
    -DBUILD_PYPANGOLIN_MODULE=OFF \
    .. && \
  make -j${NUM_THREADS} && \
  make install && \
  cd /tmp && \
  rm -rf *
ENV Pangolin_DIR=${CMAKE_INSTALL_PREFIX}/lib/cmake/Pangolin

# backward-cpp
ARG BACKWARD_CPP_COMMIT=5ffb2c879ebdbea3bdb8477c671e32b1c984beaa
WORKDIR /tmp
RUN set -x && \
  git clone https://github.com/bombela/backward-cpp.git && \
  cd backward-cpp && \
  git checkout ${BACKWARD_CPP_COMMIT} && \
  mkdir -p build && \
  cd build && \
  cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} \
    .. && \
  make -j${NUM_THREADS} && \
  make install && \
  cd /tmp && \
  rm -rf *

# stella_vslam
RUN set -x && \
  : "stella_vslam dependencies" && \
  apt-get update -y -qq && \
  apt-get install -y -qq \
    ros-${ROS_DISTRO}-libg2o && \
  : "remove cache" && \
  apt-get autoremove -y -qq && \
  rm -rf /var/lib/apt/lists/*

RUN set -x && \
  git clone --depth 1 https://github.com/stella-cv/stella_vslam.git && \
  cd stella_vslam && \
  git submodule update -i --recursive && \
  mkdir -p build && \
  cd build && \
  CMAKE_PREFIX_PATH=/opt/ros/${ROS_DISTRO}/lib/cmake cmake .. && \
  make -j${NUM_THREADS} && \
  make install && \
  rm -rf CMakeCache.txt CMakeFiles Makefile cmake_install.cmake example src && \
  chmod -R 777 ./*

ARG PANGOLIN_VIEWER_COMMIT=f3564675d420bcdbe6380b29b5e2f2b623987dd0
WORKDIR /tmp
RUN set -x && \
  git clone https://github.com/stella-cv/pangolin_viewer.git && \
  cd pangolin_viewer && \
  git checkout -q ${PANGOLIN_VIEWER_COMMIT} && \
  git submodule update --init --recursive && \
  mkdir -p build && \
  cd build && \
  CMAKE_PREFIX_PATH=/opt/ros/${ROS_DISTRO}/lib/cmake cmake \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} \
    .. && \
  make -j${NUM_THREADS} && \
  make install && \
  cd /tmp && \
  rm -rf *

# ROS2
RUN set -x && \
  apt-get update -y -qq && \
  : "install ROS2 packages" && \
  apt-get install -y -qq \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-rosbag2-storage-mcap \
    python3-pip \
    python3-colcon-common-extensions && \
  pip3 install -U \
    argcomplete && \
  : "remove cache" && \
  apt-get autoremove -y -qq && \
  rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
COPY . /ros2_ws/src/stella_vslam_ros

RUN set -x && \
  : "build ROS2 packages" && \
  bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; \
  colcon build --parallel-workers ${NUM_THREADS} --cmake-args \
    -DUSE_STACK_TRACE_LOGGER=ON"

RUN set -x && \
  sh -c "echo '#'\!'/bin/bash\nset -e\nsource /opt/ros/${ROS_DISTRO}/setup.bash\nsource /ros2_ws/install/setup.bash\nexec \"\$@\"' > /ros_entrypoint.sh" && \
  chmod u+x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
