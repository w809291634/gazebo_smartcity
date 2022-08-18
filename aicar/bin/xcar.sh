#!/bin/bash
# 红灯停，其他灯行驶
 
sudo chmod 666 /dev/ttyUSB*
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# 启动UWB定位服务
roslaunch aicar gps.launch &
# 启动ROS车导航服务
roslaunch aicar demo-xcar.launch &

# 启动ROS车智云节点
cd /home/zonesion/catkin_ws/src/aicar/script
../bin/vnode-xcar > /dev/null &
# 启动AiCar任务调度服务
python test.py -c pure_pursuit -x xcar  -r 1
# python test.py -c pure_pursuit -x xcar  -r 1 -t 3 -l 3
sleep 99999
