FROM arm64v8/ros:humble

# install nav2 dependencies
#RUN apt-get update && apt-get install -y --no-install-recommends \
#    ros-humble-joint-state-publisher-gui \
#    ros-humble-xacro \
#    ros-humble-gazebo-ros-pkgs \
#    ros-humble-robot-localization \ 
#    && rm -rf /var/lib/apt/lists/*

# install nav2 packages
#RUN apt-get update && apt-get install -y --no-install-recommends \
#    ros-humble-navigation2 \
#    ros-humble-nav2-bringup \
#    && rm -rf /var/lib/apt/lists/*

# install miscellaneous ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
     ros-humble-image-publisher \
#    ros-humble-joy-linux \
#    ros-humble-realsense2-camera \
#    ros-humble-cartographer-ros \
#    ros-humble-camera-calibration-parsers \
    && rm -rf /var/lib/apt/lists/*

# user specific packages
#RUN apt-get update && apt-get install -y --no-install-recommends \
    # package-name-here \
#    && rm -rf /var/lib/apt/lists/*

# install some text editors
RUN apt-get update && apt-get install -y --no-install-recommends \
    vim \
    nano \
    gedit \
    && rm -rf /var/lib/apt/lists/*

# non ROS miscellaneous installs
RUN apt-get update && apt-get install -y --no-install-recommends \
    neofetch \
    python3-pip \
    python3-numpy \
    python3-pandas \
    python3-torch \ 
    python3-torchvision \
    && rm -rf /var/lib/apt/lists/*

# Python packages
RUN python3 -m pip install --no-cache-dir --upgrade pip && \
    python3 -m pip install --no-cache-dir --no-dependencies \
	ultralytics \
	tqdm \
	py-cpuinfo


# source setup.bash for each new terminal
RUN printf "\nsource /opt/ros/\$ROS_DISTRO/setup.bash\n" >> ~/.bashrc

# ummmmm
RUN printf "neofetch\n" >> ~/.bashrc

# Enter workspace
WORKDIR /root/workspaces
