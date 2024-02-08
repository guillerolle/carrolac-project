#!/bin/bash
echo SOURCEANDO .BASHRC...
source ~/venvs/$(hostname)/$(id --user --name)-python310.venv/bin/activate
source /opt/ros/humble/setup.bash;
cd ~/carrolac-ros2ws;
source install/local_setup.bash;
export ROS_LOG_DIR=~/carrolac-ros2ws/log
export PYTHONWARNINGS='ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::pkg_resources'
