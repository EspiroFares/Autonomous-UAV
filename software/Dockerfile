FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-jazzy-mavros \
    ros-jazzy-mavros-extras \
    && rm -rf /var/lib/apt/lists/*

RUN source /opt/ros/jazzy/setup.bash && ros2 run mavros install_geographiclib_datasets.sh  

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

WORKDIR /workspaces/drone_ws
