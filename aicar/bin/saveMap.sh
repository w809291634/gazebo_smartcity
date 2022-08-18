#!/bin/sh
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosrun map_server map_saver -f /home/zonesion/catkin_ws/src/aicar/maps/map_gmapping
sync
echo "Save Map to aicar/maps/map_gmapping."

sleep 5
