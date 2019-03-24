#!/usr/bin/env bash
rosservice call gazebo/delete_model '{model_name: mildred}';
roslaunch mildred_description load_model.launch;
rosrun gazebo_ros spawn_model -param robot_description -urdf -model mildred -x 0 -y 0 -z 0 -robot_namespace mildred;