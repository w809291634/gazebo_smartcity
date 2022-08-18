#!/bin/bash
# 测试机械臂取货功能
 
sudo chmod 666 /dev/ttyUSB*
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash

cd /home/zonesion/catkin_ws/src/aicar/script

#roslaunch aicar gps.launch &
roslaunch aicar demo-xcar.launch &
sleep 3
roslaunch aicar arm-controller-noserial.launch &

sleep 26
roslaunch aicar grasp_filter_test.launch &

echo "start app"
#../bin/vnode-xcar > /dev/null&
python test.py -c pure_pursuit -x xcar  -r 1 -v 1 -vt 1
echo "app exit $?"
sleep 99999