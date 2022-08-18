#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import Pose
import sys
from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import PointStamped
import time
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from copy import deepcopy
import math
import threading
this = sys.modules[__name__]

this.xcarPos = None

def locCar():
    '''
        通过ros tf变换，获取小车位置
    '''
    rate = rospy.Rate(10)
    tf_listener = tf.TransformListener()
    waitTf = True
    lastA = None
    currA = 0
    while not rospy.is_shutdown():
        try:
            if waitTf:
                tf_listener.waitForTransform('/odom', '/base_link', rospy.Time(), rospy.Duration(1))
                waitTf = False
        except:
            rospy.loginfo("tf error odom->base_link")
            continue
        try:
            (trans, rot) = tf_listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("tf Error")
            waitTf = True
            continue
        euler = tf.transformations.euler_from_quaternion(rot)
        this.xcarPos = (trans[0], trans[1], euler[2])
        #rospy.loginfo ( this.xcarPos)
        rate.sleep()
def __onOdom(msg):
    a=(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(a)
    this.xcarPos = (msg.pose.pose.position.x,msg.pose.pose.position.y,euler[2])
    #print this.xcarPos 
class Pid:
    def __init__(self, P, I, D, ee=float('inf')):
        self.P = P
        self.I = I
        self.D = D
        self.lastError = 0
        self.sunError = 0
        self.enError = ee

        self.limitMax = None
        self.limitMin = None

        self.limitI = None

    def clac(self, dv, sv, sub=None):
        if sub == None:
            error = dv - sv
        else:
            error = sub(dv, sv)
        derror = error - self.lastError
        if abs(error) < self.enError:
            self.sunError += error
        else:
            self.sunError = 0

        if self.limitI != None and self.sunError > self.limitI:
            self.sunError = self.limitI

        v = self.P * error + self.I * self.sunError + self.D * derror
        self.lastError = error
        if self.limitMax != None and v > self.limitMax:
            v = self.limitMax
        if self.limitMin != None and v < self.limitMin:
            v = self.limitMin

        return v

    def clearError(self):
        self.lastError = 0
        self.sunError = 0


pidVx = Pid(2.2, 0.00, 0.7, 0.05)
pidVx.limitMax = 0.1
pidVx.limitMin = -0.1

pidVy = Pid(2.2, 0.00, 0.7, 0.05)
pidVy.limitMax = 0.1
pidVy.limitMin = -0.1

pidVa = Pid(2.5, 0, 0.1)
pidVa.limitMax = 0.5
pidVa.limitMin = -0.5
pidVa.limitI = 0.7

pidVaTun = Pid(3, 0.05, 1.5, 0.017*3)
pidVaTun.limitMax = 0.5
pidVaTun.limitMin = -0.5



def setSpeed(vx, vy, va):
    '''
        设置小车目标速度
    '''
    t = Twist()
    t.linear.x = vx
    t.linear.y = vy
    t.linear.z = 0
    t.angular.x = 0
    t.angular.y = 0
    t.angular.z = va
    #print 'out v %.2f %.2f %.2f' % (vx, vy, va)
    pubVel.publish(t)

def subAngle(a, b):
    '''
    计算两个角度偏差
    '''
    da = a - b
    if da > math.pi:
        da = da - math.pi * 2
    elif da < -math.pi:
        da = math.pi*2 + da
    return da

def tun(yaw, erra=0.01):
    '''
    控制小车旋转指定的角度yaw
    '''
    print 'tun yaw', yaw
    _lyaw = this.xcarPos[2]
    _cyaw = this.xcarPos[2] + math.pi*2*10
    _dyaw = _cyaw  + yaw

    def _update_yaw(_cyaw, lca):
        ca = this.xcarPos[2]
        da = subAngle(ca, lca)
        _cyaw = _cyaw + da
        return _cyaw, ca

    rate = rospy.Rate(5)
    while _dyaw - _cyaw > math.pi/4:
        setSpeed(0, 0, 0.5)
        rate.sleep()
        _cyaw, _lyaw = _update_yaw(_cyaw, _lyaw)
        print "diff angle", _dyaw - _cyaw
    while _dyaw - _cyaw < -math.pi/4:
        setSpeed(0, 0, -0.5)
        rate.sleep()
        _cyaw, _lyaw = _update_yaw(_cyaw, _lyaw)
        print "diff angle", _dyaw - _cyaw
    count = 0
    pidVaTun.clearError()
    while count < 5:

        print "diff angle", _dyaw - _cyaw
        if abs(_dyaw - _cyaw) < erra:
            setSpeed(0,0,0)
            count += 1
        else:
            count = 0
            va = pidVaTun.clac(_dyaw, _cyaw)
            if va >= 0.01 and va < 0.08:
                va = 0.08
            if va >-0.08 and va <= -0.01:
                va = -0.08
            setSpeed(0,0,va)
        rate.sleep()
        _cyaw, _lyaw = _update_yaw(_cyaw, _lyaw)

def getPos():
    '''
        返回当前小车坐标
    '''
    return this.xcarPos

def go(dx, dy, yaw, errd=0.005, erra=0.01):
    '''
        控制小车前后dx，左右dy,移动的距离及最终旋转的角度yaw
        小车base_link坐标系下的位置，
        yaw为终点位置车头朝向相对当前车头朝向
    '''
    if this.xcarPos == None:
        print 'xcar pos is None'
        return False
    rate = rospy.Rate(5)
    ox = this.xcarPos[0]
    oy = this.xcarPos[1]
    oa = this.xcarPos[2]


    nx = this.xcarPos[0] * math.cos(oa) + this.xcarPos[1]*math.sin(oa)
    ny = this.xcarPos[1] * math.cos(oa) - this.xcarPos[0]*math.sin(oa)



    cx = this.xcarPos[0] * math.cos(oa) + this.xcarPos[1]*math.sin(oa)  - nx
    cy = this.xcarPos[1] * math.cos(oa) - this.xcarPos[0]*math.sin(oa) - ny
    da = subAngle(this.xcarPos[2], oa)

    pidVx.clearError()
    pidVy.clearError()
    pidVa.clearError()


    bt = time.time()
    count = 0
    while count < 5:
        print 'diff %0.3f, %0.3f'%( dx-cx, dy-cy)
        if  abs(cx - dx) >=errd or abs(cy - dy) >= errd:
            count  = 0
            
            if abs(subAngle(oa, this.xcarPos[2])) > math.pi/4:
                setSpeed(0, 0, 0)
                rate.sleep()
                sys.exit(1)
            vx = pidVx.clac(dx, cx)
            vy = pidVy.clac(dy, cy)
            va = pidVa.clac(oa, this.xcarPos[2], subAngle)
            #print 'vvvv', vx, vy, va
            setSpeed(vx, vy, va)
        else:
            count += 1
            setSpeed(0, 0, 0)
        rate.sleep()
        cx = this.xcarPos[0] * math.cos(oa) + this.xcarPos[1] * math.sin(oa) - nx
        cy = this.xcarPos[1] * math.cos(oa) - this.xcarPos[0] * math.sin(oa) - ny
        #print cx,cy
    et = time.time()
    print "go time ", et-bt
    tun(yaw - subAngle(this.xcarPos[2], oa), erra)
    print "tun time", time.time()-et
    return True


def getMapPos():
    pos = PointStamped()
    pos.header.frame_id = "odom"
    pos.point.x = this.xcarPos[0]
    pos.point.y = this.xcarPos[1]
    pos.point.z = 0
    tf_listener = tf.TransformListener()
    tf_listener.waitForTransform('/map', '/odom', rospy.Time(), rospy.Duration(3))
    p = tf_listener.transformPoint("map", pos)
    return p

def init():
    #thread = threading.Thread(target=locCar)
    #thread.setDaemon(True)
    #thread.start()
    rospy.Subscriber("/odom", Odometry, __onOdom, queue_size=1)
    
    this.pubVel = rospy.Publisher('/cmd_vel', Twist, queue_size=0, latch=False)

if __name__ == '__main__':
    rospy.init_node("arm-test", log_level=rospy.INFO)
    init()
    time.sleep(5)
    time.sleep(50)
    