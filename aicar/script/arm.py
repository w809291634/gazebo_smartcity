#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import Pose
import sys
from geometry_msgs.msg import PoseStamped, Pose
import time
import tf
from std_msgs.msg import Int32
from copy import deepcopy
import math
this = sys.modules[__name__]

def goPose(a, wait=True, tmout = 8):
    '''
       设置机械臂目的位置, 
       a: 长度为6的数组
           a[0],a[1],a[2], 分别为目的x，y,z 坐标
           a[3],a[4],a[5]， 为机械爪的姿态绕x,y，z轴的旋转
    '''
    target_pose = PoseStamped()
    target_pose.header.frame_id = "base_link"  # group.get_pose_reference_frame()
    #target_pose.header.stamp = rospy.Time.now()
    target_pose.pose.position.x = a[0] #x
    target_pose.pose.position.y = a[1] #y
    target_pose.pose.position.z = a[2] #z
    # 自带吸盘的四元数
    q = tf.transformations.quaternion_from_euler(a[3], a[4], a[5]) #RPY

    target_pose.pose.orientation.x = q[0]
    target_pose.pose.orientation.y = q[1]
    target_pose.pose.orientation.z = q[2]
    target_pose.pose.orientation.w = q[3]

    arm.set_pose_target(target_pose, arm.get_end_effector_link())
    return arm.go(True)

def getPose():
    return arm.get_current_pose(arm.get_end_effector_link())
def getRpy():
    return arm.get_current_rpy(arm.get_end_effector_link())
 
        
def set_joint_value_target(joint_positions):
    arm.set_joint_value_target(joint_positions)
    arm.go(True)
    
def init():
    this.arm = MoveGroupCommander("manipulator")
    this.gripper = rospy.Publisher('/xcar/gripper', Int32, queue_size=0)

def setGripper(en):
    '''
        机械爪控制en:True 抓取
                    :False 释放
    '''
    if not en:
        this.gripper.publish(Int32(data=0))
    else:
        this.gripper.publish(Int32(data=-80))



def goHome(wait=True):
    '''
        机械臂回到初始位置
    '''
    arm.set_named_target('home')
    arm.go(wait)
    
from marm_visual_inspection.srv import GenerateSolutions
def test2(x,y,z):
    Object_pose=Pose()
    Object_pose.position.x=x
    Object_pose.position.y=y
    Object_pose.position.z=z
    Object_pose.orientation.x=0
    Object_pose.orientation.y=0
    Object_pose.orientation.z=0
    Object_pose.orientation.w=1
   
    Solutions_client = rospy.ServiceProxy('/grasp_filter_test/GenerateSolutions', GenerateSolutions)  
   
    response = Solutions_client(Object_pose)   
    print response.ik_solutions[0].positions
    if  len(response.ik_solutions[0].positions)>0:
        joint_positions = response.ik_solutions[0].positions
        arm.set_joint_value_target(joint_positions)
        arm.go(True)
        
if __name__ == '__main__':
    print 'arm'
    rospy.init_node("arm_debug", log_level=rospy.INFO)
    init()
    time.sleep(1)
    setGripper(False)
    time.sleep(1)
    setGripper(True)
    time.sleep(1)
    goHome()
    time.sleep(1)
    test2(0.3,0,0.2)
    time.sleep(1)
    goHome()

