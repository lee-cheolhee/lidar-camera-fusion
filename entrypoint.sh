#!/bin/bash

export ROS_MASTER_URI=http://192.168.101.100:11311
export ROS_IP=192.168.101.100
export ROS_HOSTNAME=$(hostname)

source /opt/ros/noetic/setup.bash

cd /root/catkin_ws && catkin_make
source /root/catkin_ws/devel/setup.bash

exec "$@"
