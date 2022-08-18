#!/bin/bash
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
source ~/.bashrc

roslaunch mbot_gazebo_smartcity mbot_gazebo_smartcity.launch 
sleep 99999
