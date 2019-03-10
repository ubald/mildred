# mildred
A ROS Based Hexapod

# Getting Started

## Extra Dependencies (Other than ROS complete)

    sudo apt install sudo apt install \
    ros-melodic-moveit \
    ros-melodic-rosserial-arduino \
    ros-melodic-joy ros-melodic-joystick-drivers \  
    ros-melodic-dynamixel-sdk ros-melodic-dynamixel-workbench \
    liburdfdom-tools

###### Using Bash

    source devel/setup.bash
    
###### Using Zsh

    source devel/setup.zsh
    
###### Using Fish
You must have [bass](https://github.com/edc/bass) installed.

    bass source devel/setup.bash

## OpenCM9.04 Arduino Setup
Additional Boards Manager URL:

    https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCM9.04/master/arduino/opencm_release/package_opencm9.04_index.json

**Linux**: Allow uploading to the board without root permissions:

    wget https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCM9.04/master/99-opencm-cdc.rules
    sudo mv ./99-opencm-cdc.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    
## Dynamixel Workbench
http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench

## LEAP Motion SDK
https://developer.leapmotion.com/sdk/v2

## Running

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