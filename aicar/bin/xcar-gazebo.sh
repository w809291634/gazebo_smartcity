#!/bin/bash
# 红灯停，其他灯行驶
 
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# 复位小车速度
rostopic pub /cmd_vel geometry_msgs/Twist -1 \
'{linear: {}, angular:{}}' 

# 复位小车位置
rosservice call /gazebo/set_model_state \
'{model_state: { model_name: mbot, 
pose: { position: { x: -2.525, y: 0.95 },
orientation: { z: -0.707, w: 0.707 }}, 
reference_frame: world }}'


# 启动UWB定位服务
# roslaunch aicar gps.launch &
# 启动ROS车导航服务
roslaunch aicar demo-xcar-gazebo.launch &

# 启动ROS车智云节点
cd /home/zonesion/catkin_ws/src/aicar/script
../bin/vnode-xcar > /dev/null &
# 启动AiCar任务调度服务
python test.py -c pure_pursuit -x xcar  -r 1
# python test.py -c pure_pursuit -x xcar  -r 1 -t 3 -l 3
sleep 99999
