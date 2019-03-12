#!/usr/bin/env bash
colcon build --base-paths src --symlink-install
ros2 launch mildred_dynamixel dynamixel_controller.launch.py
