ARG BASE_IMAGE=docker.m.daocloud.io/osrf/ros:foxy-desktop
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    gazebo11 \
    libgazebo11-dev \
    ros-foxy-gazebo-ros-pkgs \
    ros-foxy-gazebo-plugins \
    ros-foxy-navigation2 \
    ros-foxy-nav2-bringup \
    ros-foxy-pointcloud-to-laserscan \
    ros-foxy-depthimage-to-laserscan \
    ros-foxy-xacro \
    ros-foxy-joint-state-publisher \
    ros-foxy-joint-state-publisher-gui \
    ros-foxy-tf2-tools \
    ros-foxy-rviz2 \
    ros-foxy-cv-bridge \
    ros-foxy-image-transport \
    python3-colcon-common-extensions \
    python3-pip \
    python3-serial \
    python3-opencv \
    python3-numpy \
    python3-yaml \
    xvfb \
    mesa-utils \
    libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir pyserial

WORKDIR /root/sim_ws
