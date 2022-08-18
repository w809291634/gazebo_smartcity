#!/usr/bin/python
# -*- coding: utf-8 -*-
import sys
import cv2
import numpy as np
import sys
import time
import os
import math
import pyrealsense2 as rs
from geometry_msgs.msg import PointStamped
from threading import Lock
from threading import Lock, Event
import cvwin
this = sys.modules[__name__]

def max_word(lt):
    # 定义一个字典，用于保存每个元素及出现的次数
    d = {}
    # 记录做大的元素(字典的键)
    max_key = None
    for w in lt:
        if w not in d:
            # 统计该元素在列表中出现的次数
            count = lt.count(w)
            # 以元素作为键，次数作为值，保存到字典中
            d[w] = count
            # 记录最大元素
            if d.get(max_key, 0) < count:
                max_key = w
    return max_key,d
def stability_check(lt,tolerance=0.003):        
    '''
    lt 列表
    tolerance 公差 单位m，初始值3mm
    '''
    lt.sort()
    max=lt[len(lt)-1]
    min=lt[0]
    if abs(max-min)>tolerance :
        return -1
    else:
        return 1
class LocObject:
    '''目标识别，并获取目标三维坐标'''
    def __init__(self, findObjCall, MARGIN_PIX=7):
        self.findObj = findObjCall
        self.MARGIN_PIX = MARGIN_PIX
        #self.__camera__init(424,240)
        #self.configCamera(IMAGE_W,IMAGE_H)
        self.IMAGE_W = 424
        self.IMAGE_H = 240

    def startCamera(self, WIDTH=424, HEIGHT=240):
        self.IMAGE_H = HEIGHT
        self.IMAGE_W = WIDTH
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, 30)     #使能深度相机
        config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, 30)    #使能彩色相机
        self.profile = self.pipeline.start(config)
        self.__camera_enable=True
        # 保存相机内参
        frames = self.pipeline.wait_for_frames()            #等待相机坐标系生成
        color_frame = frames.get_color_frame()         #获取彩色相机坐标系
        self.intr = color_frame.profile.as_video_stream_profile().intrinsics
        camera_parameters = {'fx': self.intr.fx, 'fy': self.intr.fy,
                            'ppx': self.intr.ppx, 'ppy': self.intr.ppy,
                            'height': self.intr.height, 'width': self.intr.width,
                            'depth_scale': self.profile.get_device().first_depth_sensor().get_depth_scale()
                            }
     
        # 保存深度参数
        align_to = rs.stream.color                              #统一对齐到彩色相机
        align = rs.align(align_to)
        aligned_frames = align.process(frames)                  #对齐后的相机坐标系
        aligned_depth_frame = aligned_frames.get_depth_frame()  #对齐到彩色相机后的深度相机坐标系
        self.depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
       
        
    def wait_image(self):
       # 图像对齐
        frames = self.pipeline.wait_for_frames()            #等待相机坐标系生成

        self.align_to = rs.stream.color                     #统一对齐到彩色相机
        self.align = rs.align(self.align_to)
        self.aligned_frames = self.align.process(frames)    #对齐后的相机坐标系

        self.aligned_depth_frame = self.aligned_frames.get_depth_frame()        #对齐到彩色相机后的深度相机坐标系
        self.color_frame = self.aligned_frames.get_color_frame()                #彩色相机坐标系

        if self.aligned_depth_frame and self.color_frame:
            self.color_data = np.asanyarray(self.color_frame.get_data())        #/camera/color/image_raw
            self.dep_data= np.asanyarray(self.aligned_depth_frame.get_data()) 
            return True
        return False
        
    def object_detect(self):
        _, box, type,pp = self.findObj(self.color_data)   #_:处理后的图像，带有红色方框.与cv_image同一内存    #box:  返回左上角的坐标,和物体的宽度\高度，红色方框 #type:标签
        if len(box)>0 :
            rect = box                          #取box数据，返回左上角的坐标,和物体的宽度\高度，
            if rect[2]>self.MARGIN_PIX*2 and rect[3] > self.MARGIN_PIX*2:   #宽度大于两倍的边缘和长度大于两倍边缘
                rect2 = (rect[0]+self.MARGIN_PIX, rect[1]+self.MARGIN_PIX, rect[2]-self.MARGIN_PIX*2, rect[3]-self.MARGIN_PIX*2)    #rect2为处理后的方框坐标，绿色方框            
                cv2.rectangle(self.color_data, (rect2[0], rect2[1]),(rect2[0] + rect2[2], rect2[1] + rect2[3]), (0, 255, 0), 2)  #显示绿色方框
            else:
                rect2 = np.array([])        
        else:
            rect2 = np.array([])            
      
        cvwin.imshow("Image window", _)    #打开图像显示
        #cv2.waitKey(10)  
        return rect2,type   #绿色方框，左上角的坐标,和物体的宽度\高度，单位pixels
    
    def clac_pos(self,rect):
        #深度处理
        depimg = self.dep_data
        ret = (x, y, z) = self.getPos(depimg, rect[0] + rect[2] / 2, rect[1] + rect[3] / 2,  self.MARGIN_PIX, self.MARGIN_PIX)
        print rect, "--->", ret
 
        p1 = (x1,y1,z1) = self.getPos(depimg, rect[0], rect[1], self.MARGIN_PIX-1,
                                      self.MARGIN_PIX-1)
        p2 = (x2, y2, z2) = self.getPos(depimg, rect[0]+rect[2], rect[1],  self.MARGIN_PIX - 1,
                                        self.MARGIN_PIX - 1)
        p3 = (x3, y3, z3) = self.getPos(depimg, rect[0], rect[1]+rect[3],  self.MARGIN_PIX - 1,
                                        self.MARGIN_PIX - 1)
        p4 = (x4, y4, z4) = self.getPos(depimg, rect[0]+rect[2], rect[1]+rect[3],  self.MARGIN_PIX - 1,
                                        self.MARGIN_PIX - 1)

        w = ((x2 - x1) + (x4 - x3)) / 2 * ((rect[2]+self.MARGIN_PIX*2.0)/rect[2])
        h = ((y3 - y1) + (y4 - y2)) / 2 * ((rect[3]+self.MARGIN_PIX*2.0)/rect[3])
        size = (w, h)
        
        
        rect2 = (rect[0]+self.MARGIN_PIX, rect[1]+self.MARGIN_PIX, rect[2]-self.MARGIN_PIX*2, rect[3]-self.MARGIN_PIX*2)
        #cv2.rectangle(depimg, (rect2[0], rect2[1]),
        #              (rect2[0] + rect2[2], rect2[1] + rect2[3]), (255, 255, 255), 2)
        #cv2.rectangle(depimg, (rect[0],rect[1]), (rect[0] + rect[2], rect[1] + rect[3]),(255, 255, 255), 2)
            
        if len(ret)>0 and (ret[0] + ret[1] + ret[2]) != 0:
            poi = PointStamped()
            '''计算出来的坐标实际是相对于camera rgb的，此处要转换成camera_link 直接在y上减去一个0.03'''
            poi.header.frame_id = "camera_link" #data.header.frame_id 
            poi.point.x = ret[0]
            poi.point.y = ret[1] #- 0.03 #camera rgb坐标转camera link坐标
            poi.point.z = ret[2]
            return (poi, size)
        return (-1,-1)
    
    
    def getPos(self, deep, xx, yy, wp, hp, maxDeep = 1000):
        '''
            摄像头坐标系
                |
                |
            ----X-------------->x
                |
                |
                V y
        '''
        sumx = sumy = sumz = num = 0
        depths = []
        for m in range(int(yy-hp), int(yy+hp)):
            for n in range(int(xx-wp), int(xx+wp)):
                if m >= self.IMAGE_H or n >= self.IMAGE_W or n < 0 or m < 0:
                    continue
                if deep[m][n] < 135:
                    #print("min %d"%deep[m][n])
                    continue
                if deep[m][n] > maxDeep:
                    #print("max %d"%deep[m][n])
                    continue
                depths.append(deep[m][n])
        if len(depths)>0:
            max_key,d=max_word(depths)
            (x, y, z) = self.deep2pos(xx, yy, max_key)
            return (x,y,z)
        else:
            return (0,0,0)
    def deep2pos(self, x, y, deep):
        '''
        x   物体某点的x坐标 pixel 
        y   物体某点的y坐标 pixel
        deep    该点对应的深度数据 单位mm
        转换相机坐标系
        转换方法2
        [22.698266983032227, 29.696226119995117, 178.0]
        '''
        camera_coordinate = rs.rs2_deproject_pixel_to_point(intrin=self.depth_intrin, pixel=[x, y], depth=deep) #单位mm
        camera_factor = 1000.0 
        Zc= camera_coordinate[2]/ camera_factor     
        Xc= camera_coordinate[0]/ camera_factor 
        Yc= camera_coordinate[1]/ camera_factor 
        return (Xc,Yc,Zc)                       #返回相机坐标系下的坐标点
        
    def __locObject(self):
        '''
            等待目标识别，并获取目标三维坐标
        '''
        pos = size = t = -1
        if self.wait_image():
            rect,t = self.object_detect()
            
            if len(rect)>0:
                pos,size = self.clac_pos(rect)
        return pos, size, t
    
    def stopCamera(self):
        self.pipeline.stop()
        
    def locObject(self, wait=None):
        stTime = time.time()
        __lastTaget = []
        while  len(__lastTaget)<5:
            pos,size, t = self.__locObject()                    #相机坐标系
            if pos == -1 :
                if wait != None and time.time()-stTime > wait:
                    cvwin.destroyWindow("Image window")
                    return (None, (0,0), None)
                continue
            stTime = time.time()
            if len(__lastTaget) == 0:
                __lastTaget = [(pos, size)]
                continue
            
            if abs(pos.point.x - __lastTaget[-1][0].point.x) < 0.006 and \
                    abs(pos.point.y - __lastTaget[-1][0].point.y) < 0.006 and \
                    abs(pos.point.z - __lastTaget[-1][0].point.z) < 0.006 and \
                    abs(size[0] - __lastTaget[-1][1][0]) < 0.01 and \
                    abs(size[1] - __lastTaget[-1][1][1]) < 0.01 :
                __lastTaget.append((pos, size))
            else:
                __lastTaget = [(pos, size)]
               
        sumx = sumy = sumz = 0
        size_w = size_h = 0
        for (pos,size) in __lastTaget:
            sumx += pos.point.x
            sumy += pos.point.y
            sumz += pos.point.z
            size_w += size[0]
            size_h += size[1]

        x = sumx / len(__lastTaget)
        y = sumy / len(__lastTaget)
        z = sumz / len(__lastTaget)
        w = size_w / len(__lastTaget)
        h = size_h / len(__lastTaget)
        poi = PointStamped()
        poi.header.frame_id = pos.header.frame_id # data.header.frame_id
        poi.point.x = x
        poi.point.y = y
        poi.point.z = z
        cvwin.destroyWindow("Image window")
        return (poi, (w,h), t)
                
    def __del__(self):
        pass
        #self.pipeline.

if __name__ == '__main__':
    import threading
    from obj_detection_rk3399 import detection
    obj_model_name = 'pill_detection_20220426'
    obj_class_names = ['box1', 'box2', 'box3', 'box4']
    pilldetect = detection.ObjDetect(obj_model_name, obj_class_names)
    def detect_obj(img):
        _, box, type,pp = pilldetect.detect(img)
        if len(box)>0:
            return _, box[0], type[0], pp[0]
        return _, box, type,pp
    locobj = LocObject( detect_obj)
    locobj.startCamera()
    pos, size, type = locobj.locObject()
    locobj.stopCamera()
    print pos
    import rospy
    import tf
    
    rospy.init_node('testGrap', anonymous=True)
    tf_listener = tf.TransformListener()
    
    point_trans = tf_listener.waitForTransform("base_link", pos.header.frame_id, rospy.Time(0), rospy.Duration(1))
    pos = tf_listener.transformPoint("base_link", pos)
    print 'baselink', pos   
    
