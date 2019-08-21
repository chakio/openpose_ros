xhost +local:user
docker run -it \
--runtime=nvidia \
--env=DISPLAY=$DISPLAY \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--env="QT_X11_NO_MITSHM=1" \
--rm \
-v $HOME/.Xauthority:/root/.Xauthority \
-v $PWD/../../openpose_ros/:/catkin_ws/src/openpose_ros/ \
--privileged \
--net host \
ros:ros-openpose \
