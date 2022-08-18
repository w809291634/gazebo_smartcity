#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from copy import deepcopy
import math
import threading
import numpy as np
import sys
import time
import tf
import os


from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped

from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import NavSatFix
class ACar:
    def __init__(self):
        self.FRAMEID = "map"
        self.__pubVel = rospy.Publisher('/cmd_vel', Twist, queue_size=0, latch=False)
        
        self.__x = 0
        self.__y = 0
        self.__yaw = 0
        

        self.speed_x = 0
        self.speed_y = 0
        self.speed_a = 0
        
        self.__locTime = 0
        self.__running__ = True
        self.threadLoc = threading.Thread(target=self.__locCar)
        self.threadLoc.setDaemon(True)
        self.threadLoc.start()
        
        # self.sensors = {}
        # self.__subSensor = rospy.Subscriber("/xcar/sensors", Int32MultiArray, self.__onSensorCb)
        
        self.__gps_pos = None
        self.__subUWB = rospy.Subscriber("/gps/fix", NavSatFix, self.__onUWBCb)
        self.__lostLocTime = None
    
    # 监听UWB实时位置
    def __onUWBCb(self, msg):
        if msg.position_covariance[0]!= msg.position_covariance[0]:
            self.__gps_pos =  None
            return
        if    msg.position_covariance[0] < 20:
            y = msg.latitude
            x = msg.longitude
            self.__gps_pos = (time.time(), x, y)
        else:
            self.__gps_pos =  None

    # 监听传感器数据（已废弃，在vnode里面实现了）
    # def __onSensorCb(self, msg):
        # self.sensors['batvol'] = msg.data[0]/10.0  #电池电压
        # self.sensors['temp'] = msg.data[1]/10.0    #环境温度
        # self.sensors['humi'] = msg.data[2]         #环境湿度
        # msg.data[3]                     #气压计温度
        # self.sensors['fmbP'] = msg.data[4]         #环境气压
        # self.sensors['light'] = msg.data[5]        #环境光强
        
        # self.sensors['MP503'] = msg.data[6];
        # self.sensors['MP2'] = msg.data[7];
        # self.sensors['Sonar1'] = msg.data[8];
        # self.sensors['Sonar2'] = msg.data[9];
        # self.sensors['Sonar3'] = msg.data[10];
        # self.sensors['Sonar4'] = -1;
        # if len(msg.data)>11:
            # self.sensors['Sonar4'] = msg.data[11];
            
    # 初始化小车位置   
    def initPos(self, x, y, yaw):
    
        self.__x = x
        self.__y = y
        self.__yaw = yaw
        
        pos = PoseWithCovarianceStamped()
        pos.header.frame_id = "map"
        t = time.time()
        pos.header.seq = 1
        pos.header.stamp.secs = int(t)
        pos.header.stamp.nsecs = int((t - int(t))*1000000000)
        pos.pose.pose.position.x = x
        pos.pose.pose.position.y = y
        pos.pose.pose.position.z = 0


        qu = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pos.pose.pose.orientation.x = qu[0]
        pos.pose.pose.orientation.y = qu[1]
        pos.pose.pose.orientation.z = qu[2]
        pos.pose.pose.orientation.w = qu[3]
        pos.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.06853891945200942]

        #this.pubPos = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=0)
        #this.pubPos.publish(pos)
        cmd = "rostopic pub /initialpose geometry_msgs/PoseWithCovarianceStamped  '%s' -1" % str(pos)
        os.system(cmd)
        
    # 设置小车速度：线速度、角速度
    def updateSpeed(self, vx, va=0, vy=0, foce=False):
        if self.__lostLocTime == None or foce:
            self.speed_x = vx
            self.speed_y = vy
            self.speed_a = va
            
            t = Twist()
            t.linear.x = vx
            t.linear.y = vy
            t.linear.z = 0
            t.angular.x = 0
            t.angular.y = 0
            t.angular.z = va
            
            self.__pubVel.publish(t)
        
    # 基于雷达获取当前小车位置（x,y,a）
    def currentPos(self):
        x = self.__x;
        y = self.__y;
        a = self.__yaw
        sx = self.speed_x
        sa = self.speed_a
        #没有考虑y轴速度
        dt = time.time() - self.__locTime
        if dt <= 1:
            x = x + sx * np.cos(a) * dt 
            y = y + sx * np.sin(a) * dt 
            a = a + sa * dt
        return (x,y,a)

    # 基于UWB获取当前小车位置（x,y）
    def getGPSPos(self):
        return self.__gps_pos
        
    def __del__(self):
        self.__running__ = False
        self.threadLoc.join()
        self.pubVel.unregister()

    ############################################# 
    def __locCar(self):
        rate = rospy.Rate(10)
        tf_listener = tf.TransformListener()
        waitTf = True
 
        #start = time.time()
        while self.__running__:
            try:
                if waitTf:
                    tf_listener.waitForTransform(self.FRAMEID, '/base_link', rospy.Time(), rospy.Duration(0.1))
                    waitTf = False
            except:
                rospy.loginfo("tf error "+self.FRAMEID+"->base_link")
                continue
            try:
                (trans, rot) = tf_listener.lookupTransform(self.FRAMEID, '/base_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.loginfo("tf Error")
                waitTf = True
                continue
            euler = tf.transformations.euler_from_quaternion(rot)
            
            dx = trans[0] - self.__x
            dy = trans[1] - self.__y
            d = math.sqrt(dx * dx + dy * dy)
            da = abs(euler[2] - self.__yaw)
            
            if d > 1: #or da > math.pi/4: #定位漂移检测
                if self.__lostLocTime == None:
                    self.__lostLocTime = time.time()
                if time.time() - self.__lostLocTime > 1:
                    #重新初始化定位
                    self.updateSpeed(0,0,0,True)
                    rate.sleep()
                    self.updateSpeed(0,0,0,True)
                    rate.sleep()
                    rate.sleep()
                    self.initPos(self.__x, self.__y, self.__yaw)
                    rate.sleep()
                    rate.sleep()
                    rate.sleep()
                    self.__lostLocTime == time.time()
            else:
                self.__x = trans[0]
                self.__y = trans[1]
                self.__yaw = euler[2]
                self.__lostLocTime = None
                self.__locTime = time.time()
            
            rate.sleep()

    def waitFirstLocation(self, to=30):
        st = time.time()
        while time.time() - st < 30:
            if self.__locTime != 0:
                return True
        return  False
        
if __name__ == '__main__':
    import rospy
    rospy.init_node("xcar", log_level=rospy.INFO)
    car = ACar()
    time.sleep(60)