#!/bin/bash

source /opt/ros/noetic/setup.bash
cd /root/catkin_ws && catkin_make
source /root/catkin_ws/devel/setup.bash
exec "$@"
