# mildred
A ROS Based Hexapod

# Getting Started

## Install ROS2
See: https://index.ros.org/doc/ros2/Installation/Linux-Install-Debians/

### Install Extra Dependencies

    sudo apt install sudo apt install \
    liburdfdom-tools

## Extra Dependencies (Mac OS)
    
    brew install yaml-cpp

### Mac Setup

    # This might be required for building
    set -gx LDFLAGS "-L/usr/local/opt/tinyxml2@6.2.0/lib"
    set -gx CPPFLAGS "-I/usr/local/opt/tinyxml2@6.2.0/include"

### Build rviz from source
See: https://github.com/ros2/rviz

## OpenCM9.04 Arduino Setup
Additional Boards Manager URL:

    https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCM9.04/master/arduino/opencm_release/package_opencm9.04_index.json

**Linux**: Allow uploading to the board without root permissions:

    wget https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCM9.04/master/99-opencm-cdc.rules
    sudo mv ./99-opencm-cdc.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules
    sudo udevadm trigger

## Build

    colcon build --symlink-install --base-paths src

## Source the Workspace

###### Using Fish
You must have [bass](https://github.com/edc/bass) installed.

    bass source install/setup.bash
    
###### Using Zsh

    source install/setup.zsh
    
###### Using Bash

    source install/setup.bash

# Other References

## Dynamixel Workbench
http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench

## LEAP Motion SDK
https://developer.leapmotion.com/sdk/v2

## Running

    TBD

## Running (ROS1)

#### Terminal 1

    roscore
    
#### Terminal 2

##### Local Development

    roslaunch mildred mildred.launch

## Helpers

### View Graph
    rosrun rqt_graph rqt_graph
### View Plot
    rosrun rqt_plot rqt_plot
### Get Topic Information
    rostopic
### Services
    rosservice
### Parameter Server
    rosparam
### Console
    rosrun rqt_console rqt_console
#### Set Logger Levels
    rosrun rqt_logger_level rqt_logger_level