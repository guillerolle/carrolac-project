#!/bin/bash
# Este archivo corre la calibracion en un docker

xhost +local:root &&\
docker run --rm -it --net=host --privileged --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="`pwd`/../docker-home:/home/duser:rw" --volume="/etc/localtime:/etc/localtime:ro" ros2:humble_with_gazebo bash -c "source ~/.bashrc; colcon build" &&\

# Detiene el contenedor
xhost -local:root
