 #!/bin/bash -x

cd `dirname $0`

docker build  -t chakio/ros-openpose:devel . --build-arg USER=${USER} --build-arg USER_ID=`id -u` --build-arg workspace="/home/${USER}/catkin_ws"