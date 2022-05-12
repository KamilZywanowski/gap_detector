FROM osrf/ros:noetic-desktop-full 

# SETUP ENVS
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV DEBIAN_FRONTEND noninteractive
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# INSTALL SOME ESSENTIAL PROGRAMS
RUN apt-get update     && \
    apt-get install -y    \
        git ros-noetic-rqt-* ros-noetic-joy ros-noetic-teleop-twist-joy ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc ros-noetic-rgbd-launch ros-noetic-depthimage-to-laserscan ros-noetic-rosserial-arduino ros-noetic-rosserial-python ros-noetic-rosserial-server ros-noetic-rosserial-client ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro ros-noetic-compressed-image-transport ros-noetic-rqt-image-view ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers ros-noetic-dwa-local-planner git python3-catkin-tools vim python3-pip libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev libudev-dev ros-noetic-ddynamic-reconfigure && \
    rm -rf /var/lib/apt/lists/*

# INSTALL NUMBA
RUN pip3 install numba

# INSTALL RTABMAP
RUN apt-get update     && \
    apt-get install -y    \
        git ros-noetic-rtabmap ros-noetic-rtabmap-ros

# CREATE CATKIN WS
RUN mkdir -p /catkin_ws/src/

# INSTALL REALSENSE
WORKDIR /catkin_ws/src
RUN git clone https://github.com/IntelRealSense/librealsense.git
WORKDIR /catkin_ws/src/librealsense
RUN mkdir -p /etc/udev/rules.d
RUN ./scripts/setup_udev_rules.sh
RUN mkdir build
WORKDIR /catkin_ws/src/librealsense/build
RUN cmake ../ -DFORCE_RSUSB_BACKEND=true -DCMAKE_BUILD_TYPE=release -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true
RUN make uninstall && make clean && make && sudo make install

# INSTALL REALSENSE ROS

WORKDIR /catkin_ws/src/
RUN git clone https://github.com/IntelRealSense/realsense-ros.git
WORKDIR /catkin_ws/src/realsense-ros
RUN git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`


# DOWNLOAD GAP DETECTOR
RUN git clone https://github.com/KamilZywanowski/gap_detector.git

# DOWNLOAD CODE
WORKDIR /catkin_ws/src
RUN git clone https://github.com/ANYbotics/elevation_mapping.git
RUN git clone https://github.com/ANYbotics/grid_map.git
RUN git clone https://github.com/ANYbotics/kindr.git
RUN git clone https://github.com/ANYbotics/kindr_ros.git

# ROSDEP
WORKDIR /catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash;apt-get update;rosdep install --from-paths src --ignore-src -r -y"

# BUILD WORKSPACE
RUN /bin/bash -c ". /opt/ros/noetic/setup.bash; catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release;catkin build"

# COPY LOCAL
#COPY /turtlebot3_waffle_demo_basic.rviz /catkin_ws/src/elevation_mapping/elevation_mapping_demos/rviz/
#COPY /turtlesim3_waffle_demo_basic.launch //catkin_ws/src/elevation_mapping/elevation_mapping_demos/launch/

# FILL BASHRC
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc
