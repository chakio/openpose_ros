#!/bin/bash

cd `dirname $0`

xhost +local:user

TAG="chakio/ros-openpose:devel"
docker run -it \
--privileged \
--runtime=nvidia \
--env=DISPLAY=$DISPLAY \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--env="QT_X11_NO_MITSHM=1" \
--rm \
-v "/$(pwd)/global_ros_setting.sh:/ros_setting.sh" \
-v "/$(pwd)/ros_workspace:/home/${USER}/catkin_ws/" \
-v "/$(pwd)/../openpose_ros:/home/${USER}/catkin_ws/src/openpose_ros/" \
-v /etc/group:/etc/group:ro \
-v /etc/passwd:/etc/passwd:ro \
-v /etc/localtime:/etc/localtime:ro \
-v /dev:/dev \
--net host \
$TAG