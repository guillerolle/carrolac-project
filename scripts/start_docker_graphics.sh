#!/usr/bin/bash

docker volume create --driver local --opt type=none --opt device="$(pwd)/../site-packages" --opt o=bind docker-site &&\
docker volume create --driver local --opt type=none --opt device="$(pwd)/../dist-packages" --opt o=bind docker-dist &&\
xhost +local:root &&\
docker run --rm -it --net=host --privileged --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"  \
  --volume="`pwd`/../docker-home:/home/duser:rw" \
  --volume="`pwd`/../carrolac-ros2ws:/home/duser/carrolac-ros2ws:rw" \
  --volume="`pwd`/../venvs:/home/duser/venvs:rw" \
  --volume="/etc/localtime:/etc/localtime:ro" \
  --volume="docker-site:/opt/ros/humble/lib/python3.10/site-packages" \
  --volume="docker-dist:/opt/ros/humble/local/lib/python3.10/dist-packages" \
  ros2:humble_with_gazebo &&\
xhost -local:root

# --volume="`pwd`/../carrolac-ros2ws:/home/duser/carrolac-ros2ws:rw" --volume="`pwd`/../venvs/requirements.txt:/home/duser/requirements.txt:rw"
# --volume="`pwd`/../docker-home:/home/duser:rw"


  # --volume="`pwd`/../docker-home/.bashrc:/home/duser/.bashrc:rw" \
  # --volume="`pwd`/../docker-home/.bash_history:/home/duser/.bash_history:rw" \
  # --volume="$(pwd)/../docker-home/venvs:/home/duser/venvs:rw" \