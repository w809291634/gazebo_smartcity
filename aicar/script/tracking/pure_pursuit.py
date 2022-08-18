#!/usr/bin/python
# -*- coding: utf-8 -*-
import numpy as np
import math
import time

# Parameters
#Lf = k * state.v + Lfc
k = 0       #前视距离增益
Lfc = 0.2  # 前视距离

MAX_SPEED = 0.2
MAX_SPEED_A = math.pi/4

def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].

    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle

def goWithPathEx(paths, car):
    cyaw = []           #每个点的方向角              
    p0 = None 
    skip = 0
    noYaw = True
    for path in paths:  #计算每个点的方向角
        for p in path.path:
            if p0 == None:
                p0 = p
            else:
                dis = math.sqrt((p.x - p0.x)**2 + (p.y - p0.y)**2)
                if dis >= 0.1: #防止距离太小角度抖动
                    yaw = math.atan2(p.y-p0.y, p.x-p0.x)
                    cyaw.append(yaw)
                    while skip > 0: 
                        cyaw.append(yaw)
                        skip -= 1
                    p0 = p
                    noYaw = False
                else:
                    skip += 1
    if noYaw:           
        p0 = paths[0].path[0]
        p = paths[-1].path[-1]
        yaw = math.atan2(p.y-p0.y, p.x-p0.x)
    cyaw.append(yaw)
    sizep = len(cyaw)
    cx, cy, ca = car.currentPos()
    da = normalize_angle(ca-cyaw[0])
    
    if abs(da) > (math.pi/180)*60: 
       #当车头方向与起始路径方向夹角>30°，进行车头调整
        while abs(da) > (math.pi/180)*10:
            if da > 0:
                va = - MAX_SPEED_A/2
            elif da < 0:
                va =  MAX_SPEED_A/2
            yield 0, 0, va, paths[0], 0
            cx, cy, ca = car.currentPos()
            da = normalize_angle(ca-cyaw[0])
        yield 0, 0, 0, paths[0], 0
    ip = 0
    idx_i = 0
    idx_j = 0
    while ip < sizep - 1 or dis >= 0.2: 
        cx, cy, ca = car.currentPos()
        # tpos 保存当前跟踪点
        tpos = paths[idx_i].path[idx_j] 
        #当前位置与跟踪点的距离
        dis = math.sqrt((tpos.x - cx)**2 + (tpos.y - cy)**2) 
        #Lf 计算前示距离
        Lf = k * car.speed_x + Lfc
        #计算下一跟踪点
        while dis < Lf and ip < sizep-1:                    
            ip += 1
            idx_j += 1
            if idx_j >= len(paths[idx_i].path) :
                idx_j = 0
                idx_i += 1
            lastpos = tpos
            tpos = paths[idx_i].path[idx_j]
            
            dis += math.sqrt((tpos.x - lastpos.x)**2 + (tpos.y - lastpos.y)**2)
        
            
        cx, cy, ca = car.currentPos()
        # 跟踪点到小车当前位置与地图x轴夹角
        alpha = math.atan2(tpos.y - cy, tpos.x - cx)
        # 跟踪点与小车距离
        dis = math.sqrt((tpos.x - cx)**2 + (tpos.y - cy)**2)
        # de 小车航向与alpha的误差，即小车航向与当前位置到目标点夹角
        de = normalize_angle(alpha-ca)
        
        #bate目标航向与当前位置到目标点夹角
        bate = normalize_angle(cyaw[ip]- alpha) 
        #da目标点航向与小车当前航向夹角
        da = normalize_angle(cyaw[ip]- ca) 
        #dw横向误差
        dw = dis * math.cos(bate) 
        v = MAX_SPEED
        #小车移向目标点的速度
        vx = v * math.cos(de)  
        vy = v * math.sin(de)
        va = da/(dis / v)
        #限速
        if va > MAX_SPEED_A:
            va = MAX_SPEED_A
        elif va < -MAX_SPEED_A:
            va = -MAX_SPEED_A
        #返回小车的速度、路径线、当前路径点等数据    
        yield vx, vy, va, paths[idx_i], idx_j
