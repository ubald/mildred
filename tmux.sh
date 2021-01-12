#!/usr/bin/env bash

CWD=$(pwd)
SESSION_NAME="mildred"

tmux detach > /dev/null
set -- $(stty size) # $1 = rows $2 = columns
tmux new-session -d -s $SESSION_NAME -x "$2" -y "$(($1 - 1))"

tmux new-window -t $SESSION_NAME:1 -n 'roscore'
tmux new-window -t $SESSION_NAME:2 -n 'description'
tmux new-window -t $SESSION_NAME:3 -n 'control'
tmux new-window -t $SESSION_NAME:4 -n 'teleop'
tmux new-window -t $SESSION_NAME:5 -n 'gazebo'

tmux select-window -t $SESSION_NAME:1
tmux send-keys "bass source devel/setup.sh" C-m
tmux send-keys "roscore" C-m

tmux select-window -t $SESSION_NAME:2
tmux send-keys "bass source devel/setup.sh" C-m
tmux send-keys "roslaunch mildred_description load_model.launch " C-m

tmux select-window -t $SESSION_NAME:3
tmux send-keys "bass source devel/setup.sh" C-m
tmux send-keys "catkin_make && roslaunch mildred_control control.launch"

tmux select-window -t $SESSION_NAME:4
tmux send-keys "bass source devel/setup.sh" C-m
tmux send-keys "catkin_make && roslaunch mildred_teleop ps4.launch"

tmux select-window -t $SESSION_NAME:5
tmux send-keys "bass source devel/setup.sh" C-m
tmux send-keys "roslaunch mildred_gazebo mildred.launch"

tmux attach -t $SESSION_NAME
