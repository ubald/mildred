# mildred
A ROS Based Hexapod

# Getting Started

## Extra Dependencies (Other than ROS complete)

    sudo apt install ros-melodic-rosserial-arduino 

###### Using Bash

    source devel/setup.bash
    
###### Using Zsh

    source devel/setup.zsh
    
###### Using Fish
You must have [bass](https://github.com/edc/bass) installed.

    bass source devel/setup.bash
    
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