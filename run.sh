#!/bin/bash

ENVS="--env=XAUTHORITY=/home/$(id -un)/.Xauthority
      --env=ROS_IP=127.0.0.1
      --env=DISPLAY=$DISPLAY
      --env=ROS_MASTER_URI=http://localhost:11311
      --device=/dev/dri:/dev/dri"

XSOCK=/tmp/.X11-unix
XAUTH=$HOME/.Xauthority
VOLUMES="--volume=$XSOCK:$XSOCK
		 --volume=$XAUTH:/home/$(id -un)/.Xauthority"

xhost +local:docker

docker run \
-it \
$VOLUMES \
$ENVS \
--privileged \
--net host \
--ipc host \
--workdir="/root/catkin_ws/src" \
--name=fusion_ws \
rdv_fusion:0.1 /bin/bash
