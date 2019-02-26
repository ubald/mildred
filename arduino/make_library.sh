#!/usr/bin/env bash
mkdir -p ./libraries
rm -r ./libraries/ros_lib
rosrun rosserial_arduino make_libraries.py ./libraries mildred_core