#!/usr/bin/env bash
set -e

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update

sudo apt install ros-noetic-desktop-full
#sudo apt install ros-noetic-desktop
#sudo apt install ros-noetic-ros-base

sudo apt install \
    ros-noetic-joy ros-noetic-ps3joy \
    ros-noetic-dynamixel-sdk \
    ros-noetic-effort-controllers \
    ros-noetic-control \
    ros-noetic-controllers \
    python3-roslaunch \
    libyaml-cpp-dev \
    liburdfdom-tools

source /opt/ros/noetic/setup.bash

# build Workspace
git submodule update --init --resursive
catkin_make
source devel/setup.bash