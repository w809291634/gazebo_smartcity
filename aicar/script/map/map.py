#!/usr/bin/python
# -*- coding: utf-8 -*-
##############################################################################################
# 文件：map.py
# 作者：Zonesion Xuzhy 20220412
# 说明：制作电子地图，实现图像坐标系与电子地图坐标系转换
# 修改：
# 注释：
##############################################################################################
import cv2 as cv
from copy import deepcopy
import os
import math

dir = os.path.split(os.path.realpath(__file__))[0]

# 定义电子地图尺寸和相关坐标系
'''
    电子地图坐标系
    
                ^  X
                |
                |
         <------ORIGIN
          Y
'''
MAP_IMAGE   = "map.jpg"                                         # 地图图片文件
MAP_WIDTH   = 6.4                                               # 地图宽度
MAP_HEIGHT  = 6.4                                               # 地图高度

MAP_ROAD_WIDTH = 0.6                                            # 地图道路宽度
MAP_HALF_WIDTH  = MAP_WIDTH/2                                   # 地图一半的宽度
MAP_HALF_HEIGHT = MAP_HEIGHT/2                                  # 地图一半的高度

# 图像坐标系
image = cv.imread(dir+"/"+MAP_IMAGE, cv.IMREAD_COLOR)
IMAGE_WIDTH = image.shape[1]                                    # 图片宽度
IMAGE_HEIGHT =  image.shape[0]                                  # 图片高度
ORIGIN = (IMAGE_HEIGHT/2, IMAGE_WIDTH/2)                        # 地图原点在图片中的位置

# 获取图片的拷贝
def getImage():
    return deepcopy(image)

# 电子地图坐标系转换为图像坐标系
def mapp2img(p):
    img_x = int(ORIGIN[1] - p.y * IMAGE_WIDTH/MAP_WIDTH)
    img_y = int(ORIGIN[0] - p.x * IMAGE_HEIGHT/MAP_HEIGHT)
    if img_x < 0:
        img_x = 0
    if img_x >= IMAGE_WIDTH:
        img_x = IMAGE_WIDTH-1
    if img_y < 0:
        img_y = 0
    if img_y >= IMAGE_HEIGHT:
        img_y = IMAGE_HEIGHT - 1
    return (img_x, img_y)

# 图像坐标系转换为电子地图坐标系
def imgp2map(ix, iy):
    y = (ORIGIN[1] - ix)/(IMAGE_WIDTH/MAP_WIDTH)
    x = (ORIGIN[0] - iy)/(IMAGE_HEIGHT/MAP_HEIGHT)
    return x,y

# 在电子地图上画路径线
def drawPath(img, paths, color, width=2):
    start = 0
    for path in paths:
        for p in path.path:
            if start == 0:
                p1 = mapp2img(p)
                start = 1
            else:
                p2 = mapp2img(p)
                co = color
                if path.trafficlight:
                    co = (255-color[0], 255-color[1], 255-color[2])
                cv.line(img, p1, p2, co, width)
                p1 = p2
                
# 在电子地图上画路径点
def drawPoint(img, pos, radius=3, color =(255,0,255), thickness=-1, name=""):
    cp = mapp2img(pos)
    cv.circle(img, cp, radius, color, thickness)
    if len(name)>0:
        cv.putText(img, name, (cp[0]-16, cp[1]-4), cv.FONT_HERSHEY_PLAIN , 1, color, thickness=2)

# 在电子地图上画圆  
def drawCircle(img, x, y, r, color=(255,255,0)):
    class POS:
        def __init__(self, x, y):
            self.x = x
            self.y = y
    p = POS(x,y)
    p = mapp2img(p)
    
    r = int(r * IMAGE_WIDTH/MAP_WIDTH)
    cv.circle(img, p, r, color, thickness=2)
	
# 在电子地图上画小车  
def drawCar(img, x,y,a, color=(255,255,0)):
    class POS:
        def __init__(self, x, y):
            self.x = x
            self.y = y
    lines = [(0.15,0), (-0.1,0.1),(0,0),(-0.1,-0.1),(0.15,0)]
    nlines = []
    for px,py in lines:
        nx = px * math.cos(a) - py * math.sin(a)
        ny = px * math.sin(a) + py * math.cos(a) 
        cp = mapp2img(POS(x+nx, y+ny))
        nlines.append(cp)
    p1 = nlines[0]
    for p2 in nlines[1:]:
        cv.line(img, p1, p2, color, 2)
        p1 = p2
		
# 在电子地图上显示雷达坐标
def drawRadarPos(img, x,y,a, color=(255,255,0)):
    class POS:
        def __init__(self, x, y):
            self.x = x
            self.y = y
    p=mapp2img(POS(x,y))  #小车坐标
    point='radar:'+'('+'%.2f'%x+','+'%.2f'%y+','+'%.2f'%a+')'
    cv.putText(img, point, (p[0]-100, p[1]-20), cv.FONT_HERSHEY_PLAIN , 1, color, thickness=2)

# 在电子地图上显示UWB坐标
def drawUWBPos(img, x,y,u_x,u_y, color=(255,255,0)):
    class POS:
        def __init__(self, x, y):
            self.x = x
            self.y = y
    p=mapp2img(POS(x,y))  #小车坐标
    point='  uwb:'+'('+'%.2f'%u_x+','+'%.2f'%u_y+')'
    cv.putText(img, point, (p[0]-100, p[1]-40), cv.FONT_HERSHEY_PLAIN , 1, color, thickness=2)

# 单元测试：加载电子地图，鼠标点击地图任意位置在终端打印坐标
if __name__ == '__main__':
    def OnMouseAction(evt, x, y, flags, param):
        if evt == cv.EVENT_LBUTTONDOWN:
            px,py  = imgp2map(x,y)
            print "(%d,%d)--->(%.3f,%.3f)"%(x,y,px,py)

    cv.imshow("map", getImage())
    cv.setMouseCallback('map',OnMouseAction) 

    # 按“q”键退出
    while True:
        key=cv.waitKey(1)
        if key==ord('q'):
            break
    cv.destroyAllWindows()
    
 



