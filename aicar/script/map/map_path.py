#!/usr/bin/python
# -*- coding: utf-8 -*-
##############################################################################################
# 文件：map_path.py
# 作者：Zonesion Xuzhy 20220412
# 说明：路径规划算法
# 修改：
# 注释：
##############################################################################################
import cv2 as cv
import random
import math
import copy
import sys
import argparse
import map

# 路径点定义
class Point:
    def __init__(self, x, y, tag="", yaw=0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.__tag = tag
        
    def equal(self, p):
        return abs(self.x - p.x)<0.001 and abs(self.y - p.y)<0.001
    
    def tag(self):
        return self.__tag

# 路径线定义
# path： 路径点数组， 
# name:  道路名字
# trafficlight: 是否有交通灯需要识别
# voice：语音播报
class Path:
    def __init__(self, path, name="", trafficlight=False, voice=None, leftp=[], straightp=[], rightp=[]):

        self.path = path
        self.name = name
        self.trafficlight = trafficlight
        self.voice = voice
        
        self.leftp=leftp
        self.straightp = straightp
        self.rightp = rightp
        
    def tag(self):
        t = []
        for p in self.path:
            t.append(p.tag())
        return "-->".join(t)
        
# 将x，y逆时针旋转angle后的新坐标   
def __rotaPoint(p, angle, ro="", rn=""):
    nx = p.x*math.cos(angle) - p.y * math.sin(angle)
    ny = p.x*math.sin(angle) + p.y * math.cos(angle)
    nyaw = p.yaw + angle
    nn = p.tag().replace(ro, rn)
    return Point(nx, ny, nn, nyaw)

# 通过旋转创建镜像的道路：ang: 旋转度数
def __rotaPath(ps, ang, ro="", rn=""):
    nps = []
    for p in ps:
        np = __rotaPoint(p, ang, ro, rn)
        nps.append(np)
    return tuple(nps)

# 通过旋转创建镜像的道路：ang: 旋转度数
def __rotaPaths(pss, ang, ro, rn):
    npss = []
    for ps in pss:
        p = Path(__rotaPath(ps.path, ang, ro,rn), ps.name.replace(ro,rn), ps.trafficlight, ps.voice)
        npss.append(p)
    return tuple(npss)
    
# 地图中心为（0，0）点方向正北为x轴
# 地图高6.4m，宽6.4m
# 宽与y轴平行，高与x轴平行
MAP_ROAD_WIDTH  = 0.6                                           # 道路宽度
MAP_HALF_WIDTH  = 3.2                                           # 地图一半的宽度
MAP_HALF_HEIGHT = 3.2                                           # 地图一半的高度

PARK_WIDTH  = 0.5                                               # 停车位宽度
PARK_HEIGHT = 0.45                                              # 停车位高度
# 南向停车位位置定义
Park01 = Point(-MAP_HALF_HEIGHT+PARK_HEIGHT+PARK_HEIGHT/2, MAP_ROAD_WIDTH*2-PARK_WIDTH/2, "SP1", -math.pi/2) # 停车位1位置
Park02 = Point(Park01.x-PARK_HEIGHT, Park01.y, "SP2", -math.pi/2) # 停车位2位置
Park03 = Point(Park01.x, -Park01.y, "SP3", math.pi/2)           # 停车位3位置
Park04 = Point(Park02.x, -Park02.y, "SP4", math.pi/2)           # 停车位4位置
# 所有停车位位置定义
Parks = [Park01, Park02, Park03, Park04]
Parks += [__rotaPoint(Park01,math.pi/2, "S", "E"),__rotaPoint(Park02,math.pi/2, "S", "E"),__rotaPoint(Park03,math.pi/2, "S", "E"),__rotaPoint(Park04,math.pi/2, "S", "E")]
Parks += [__rotaPoint(Park01,math.pi, "S", "N"),__rotaPoint(Park02,math.pi, "S", "N"),__rotaPoint(Park03,math.pi, "S", "N"),__rotaPoint(Park04,math.pi, "S", "N")]
Parks += [__rotaPoint(Park01,-math.pi/2, "S", "W"),__rotaPoint(Park02,-math.pi/2, "S", "W"),__rotaPoint(Park03,-math.pi/2, "S", "W"),__rotaPoint(Park04,-math.pi/2, "S", "W")]

# 取送货点位置定义
ROAD_POINT_X0 = Point(-MAP_ROAD_WIDTH*7/2-0.05, MAP_ROAD_WIDTH*4/2+0.15, "SX0",math.pi/2)
ROAD_POINT_X1 = Point(-MAP_ROAD_WIDTH*7/2-0.05, -MAP_ROAD_WIDTH*4/2-0.15, "SX1",-math.pi/2)
FPoints = [ROAD_POINT_X0, ROAD_POINT_X1]
FPoints += [__rotaPoint(ROAD_POINT_X0,math.pi/2, "S", "E"), __rotaPoint(ROAD_POINT_X1,math.pi/2, "S", "E")]
FPoints += [__rotaPoint(ROAD_POINT_X0,math.pi, "S", "N"), __rotaPoint(ROAD_POINT_X1,math.pi, "S", "N")]
FPoints += [__rotaPoint(ROAD_POINT_X0,-math.pi/2, "S", "W"), __rotaPoint(ROAD_POINT_X1,-math.pi/2, "S", "W")]

# 前16个为停车位，后8个为取送货点
Parks = tuple(Parks+FPoints)

# 中心转盘关键点定义
INTERSECTION_01 = Point(-MAP_ROAD_WIDTH*3/4, MAP_ROAD_WIDTH*3/4, "C01")
INTERSECTION_0102 = Point(-MAP_ROAD_WIDTH*4/4, 0, "C0102") #01 02 中间点
INTERSECTION_02 = Point(-MAP_ROAD_WIDTH*3/4, -MAP_ROAD_WIDTH*3/4, "C02")
INTERSECTION_0203 = Point(0, -MAP_ROAD_WIDTH*4/4, "C0203") #02 03 中间点
INTERSECTION_03 = Point(MAP_ROAD_WIDTH*3/4, -MAP_ROAD_WIDTH*3/4, "C03")
INTERSECTION_0304 = Point(MAP_ROAD_WIDTH*4/4, 0, "C0304") #03 04 中间点
INTERSECTION_04 = Point(MAP_ROAD_WIDTH*3/4, MAP_ROAD_WIDTH*3/4, "C04")
INTERSECTION_0401 = Point(0, MAP_ROAD_WIDTH*4/4, "C0401") #04 01 中间点

INTERSECTION_11 = Point(-MAP_ROAD_WIDTH*3/2, MAP_ROAD_WIDTH*3/2, "C11")
INTERSECTION_12 = Point(-MAP_ROAD_WIDTH*3/2, MAP_ROAD_WIDTH*1/2, "C12")
INTERSECTION_13 = Point(-MAP_ROAD_WIDTH*3/2, -MAP_ROAD_WIDTH*1/2, "C13")
INTERSECTION_14 = Point(-MAP_ROAD_WIDTH*3/2, -MAP_ROAD_WIDTH*3/2, "C14")
INTERSECTION_15 = Point(-MAP_ROAD_WIDTH*1/2, -MAP_ROAD_WIDTH*3/2, "C15")
INTERSECTION_16 = Point(MAP_ROAD_WIDTH*1/2, -MAP_ROAD_WIDTH*3/2, "C16")
INTERSECTION_17 = Point(MAP_ROAD_WIDTH*3/2, -MAP_ROAD_WIDTH*3/2, "C17")
INTERSECTION_18 = Point(MAP_ROAD_WIDTH*3/2, -MAP_ROAD_WIDTH*1/2, "C18")
INTERSECTION_19 = Point(MAP_ROAD_WIDTH*3/2, MAP_ROAD_WIDTH*1/2, "C19")
INTERSECTION_1a = Point(MAP_ROAD_WIDTH*3/2, MAP_ROAD_WIDTH*3/2, "C1A")
INTERSECTION_1b = Point(MAP_ROAD_WIDTH*1/2, MAP_ROAD_WIDTH*3/2, "C1B")
INTERSECTION_1c = Point(-MAP_ROAD_WIDTH*1/2, MAP_ROAD_WIDTH*3/2, "C1C")

# 南向关键路径点定义
ROAD_POINT_A1 = Point(-MAP_ROAD_WIDTH*5/2, MAP_ROAD_WIDTH*3/2, "SA1")
ROAD_POINT_A2 = Point(-MAP_ROAD_WIDTH*7/2, MAP_ROAD_WIDTH*3/2, "SA2")
ROAD_POINT_B1 = Point(-MAP_ROAD_WIDTH*5/2, MAP_ROAD_WIDTH*1/2, "SB1")
ROAD_POINT_B2 = Point(-MAP_ROAD_WIDTH*7/2, MAP_ROAD_WIDTH*1/2, "SB2")
ROAD_POINT_B3 = Point(Park01.x, MAP_ROAD_WIDTH*1/2, "SB3")
ROAD_POINT_B4 = Point(Park02.x+0.2, MAP_ROAD_WIDTH*1/2, "SB4")
ROAD_POINT_C3 = Point(Park04.x+0.2, -MAP_ROAD_WIDTH*1/2, "SC3")
ROAD_POINT_C2 = Point(Park03.x, -MAP_ROAD_WIDTH*1/2, "SC2")
ROAD_POINT_C1 = Point(Park03.x+MAP_ROAD_WIDTH/2, -MAP_ROAD_WIDTH*1/2, "SC1")
ROAD_POINT_C0 = Point(-MAP_ROAD_WIDTH*4/2, -MAP_ROAD_WIDTH*1/2, "SC0")             # 红绿灯等待点
ROAD_POINT_D1 = Point(ROAD_POINT_C1.x+MAP_ROAD_WIDTH, -MAP_ROAD_WIDTH*3/2,"SD1")
ROAD_POINT_D0 = Point(-MAP_ROAD_WIDTH*5/2, -MAP_ROAD_WIDTH*3/2, "SD0")             # 红绿灯等待点

# 所有关键路径点
Center = [INTERSECTION_01, INTERSECTION_0102, INTERSECTION_02, INTERSECTION_0203, INTERSECTION_03, INTERSECTION_0304, INTERSECTION_04, INTERSECTION_0401,
          INTERSECTION_11, INTERSECTION_12, INTERSECTION_13, INTERSECTION_14, INTERSECTION_15, INTERSECTION_16, INTERSECTION_17, INTERSECTION_18, INTERSECTION_19, INTERSECTION_1a, INTERSECTION_1b, INTERSECTION_1c]
SRoads = [ROAD_POINT_A1, ROAD_POINT_A2, ROAD_POINT_B1, ROAD_POINT_B2,
         ROAD_POINT_B3, ROAD_POINT_B4, ROAD_POINT_C3, ROAD_POINT_C2,
         ROAD_POINT_C1, ROAD_POINT_C0, ROAD_POINT_D1, ROAD_POINT_D0]
ERoads = [__rotaPoint(p,math.pi/2, "S", "E") for p in SRoads]
NRoads = [__rotaPoint(p,math.pi, "S", "N") for p in SRoads]
WRoads = [__rotaPoint(p,-math.pi/2, "S", "W") for p in SRoads]
Roads = tuple(Center+SRoads+ERoads+NRoads+WRoads)

# 南向道路路径线
south_paths = (
    Path((ROAD_POINT_A1, ROAD_POINT_A2), "SA1->SA2"),
    Path((ROAD_POINT_A1, ROAD_POINT_B2), "SA1->SB2"),
    Path((ROAD_POINT_B1, ROAD_POINT_A2), "SB1->SA2"),
    Path((ROAD_POINT_B1, ROAD_POINT_B2), "SB1->SB2"),
    Path((ROAD_POINT_B2, ROAD_POINT_B3), "SB2->SB3"),
    Path((ROAD_POINT_B3, ROAD_POINT_B4), "SB3->SB4"),
    Path((ROAD_POINT_B4, ROAD_POINT_C3), "SB4->SC3"),
    Path((ROAD_POINT_C3, ROAD_POINT_C2), "SC3->SC2"),
    Path((ROAD_POINT_C2, ROAD_POINT_C1), "SC2->SC1"),
    Path((ROAD_POINT_C1, ROAD_POINT_D1), "SC1->SD1"),
    Path((ROAD_POINT_C1, ROAD_POINT_C0), "SC1->SC0", trafficlight = True),    # 红绿灯,SC0为红绿灯等待点
    Path((ROAD_POINT_D1, ROAD_POINT_D0), "SD1->SD0", trafficlight = False),   # 红绿灯
    
    # 进入停车位
    Path((ROAD_POINT_A2, Park01), "SA2->SP1"),
    Path((ROAD_POINT_A2, Park02), "SA2->SP2"),
    Path((ROAD_POINT_C2, Park03), "SC2->SP3"),
    Path((ROAD_POINT_C3, Park04), "SC3->SP4"),

    # 离开停车位
    Path((Park01, ROAD_POINT_B3), "SP1->SB3"),
    Path((Park02, ROAD_POINT_B4), "SP2->SB4"),
    Path((Park03, ROAD_POINT_D1), "SP3->SD1"),
    Path((Park04, ROAD_POINT_C3, ROAD_POINT_C2), "SP4->SC3->SC2"),
    
    # 取送货路线
    Path((ROAD_POINT_A2, ROAD_POINT_X0), "SA2-SX0"),
    Path((ROAD_POINT_X0, ROAD_POINT_A2), "SX0-SA2"),
    Path((ROAD_POINT_X0, ROAD_POINT_B2), "SX0-SB2"),
    Path((ROAD_POINT_C1, ROAD_POINT_X1), "SC1-SX1"),
    Path((ROAD_POINT_X1, ROAD_POINT_C1), "SX1-SC1"),
    Path((ROAD_POINT_X1, Point(ROAD_POINT_X1.x,ROAD_POINT_D1.y), ROAD_POINT_D1), "SX1-SD1"),
)
# 通过南向路径生成其他3个方向路径
east_paths = __rotaPaths(south_paths, math.pi/2, "S", "E")
north_paths = __rotaPaths(south_paths, math.pi, "S", "N")
west_paths = __rotaPaths(south_paths, -math.pi/2, "S", "W")

# 地图所有路径线数据
MAP_PATHS = (
    # 中心转盘路径定义, 中心转盘有4个入口
    Path((INTERSECTION_13, INTERSECTION_02, INTERSECTION_0203), "C13->C02->C0203"),  #1CPATH_13_02  3进不能从02出
    Path((INTERSECTION_16, INTERSECTION_03, INTERSECTION_0304), "C16->C03->C0304"),  #CPATH_16_03   16进不能从03出
    Path((INTERSECTION_19, INTERSECTION_04, INTERSECTION_0401), "C19->C04->C0401"), 
    Path((INTERSECTION_1c, INTERSECTION_01, INTERSECTION_0102), "C1c->C01->C0102"),
    # 中心转盘路径定义, 中心转盘有4个出口
    Path((INTERSECTION_0401, INTERSECTION_01, INTERSECTION_12), "C0401->C01->C12"),
    Path((INTERSECTION_0102, INTERSECTION_02, INTERSECTION_15), "C0102->C02->C15"),
    Path((INTERSECTION_0203, INTERSECTION_03, INTERSECTION_18), "C0203->C03->C18"),
    Path((INTERSECTION_0304, INTERSECTION_04, INTERSECTION_1b), "C0304->C04->C1b"),
    # 中心转盘路径
    Path((INTERSECTION_0203, INTERSECTION_03, INTERSECTION_0304), "C0203->C03->C0304"),
    Path((INTERSECTION_0304, INTERSECTION_04, INTERSECTION_0401), "C0304->C04->C0401"),
    Path((INTERSECTION_0401, INTERSECTION_01, INTERSECTION_0102), "C0401->C01->C0102"),
    Path((INTERSECTION_0102, INTERSECTION_02, INTERSECTION_0203), "C0102->C02->C0203"),

    # 南向道路路径 入口
    Path((INTERSECTION_11, ROAD_POINT_A1), "C11->SA1"),   
    Path((INTERSECTION_12, ROAD_POINT_B1), "C12->SB1"),        
    # 南向道路路径 出口
    Path((ROAD_POINT_C0, INTERSECTION_13), "SC0->C13"),
    Path((ROAD_POINT_D1, ROAD_POINT_D0, INTERSECTION_14), "SD1->SD0->C14", voice=u"前方右转。"),
    
    # 东向道路路径 入口
    Path((INTERSECTION_14, __rotaPoint(ROAD_POINT_A1, math.pi/2, "S", "E")), "C14->EA1"),
    Path((INTERSECTION_15, __rotaPoint(ROAD_POINT_B1, math.pi/2, "S", "E")), "C15->EB1"),
    # 东向道路路径 出口
    Path((__rotaPoint(ROAD_POINT_C0, math.pi/2, "S", "E"), INTERSECTION_16), "EC0->C16"), 
    Path((__rotaPoint(ROAD_POINT_D1, math.pi/2, "S", "E"),__rotaPoint(ROAD_POINT_D0, math.pi/2, "S", "E"), INTERSECTION_17), "ED1->ED0->C17", voice=u"前方右转。"),
    
    # 北向道路路径 入口
    Path((INTERSECTION_17, __rotaPoint(ROAD_POINT_A1, math.pi, "S", "N")), "C17->NA1"),
    Path((INTERSECTION_18, __rotaPoint(ROAD_POINT_B1, math.pi, "S", "N")), "C18->NB1"),
    # 北向道路路径 出口
    Path((__rotaPoint(ROAD_POINT_C0, math.pi, "S", "N"), INTERSECTION_19), "NC0->C19"),
    Path((__rotaPoint(ROAD_POINT_D1, math.pi, "S", "N"), __rotaPoint(ROAD_POINT_D0, math.pi, "S", "N"), INTERSECTION_1a), "ND1->ND0->C1A", voice=u"前方右转。"),

    # 西向道路路径 入口
    Path((INTERSECTION_1a, __rotaPoint(ROAD_POINT_A1, -math.pi/2, "S", "W")), "C1a->WA1"),   
    Path((INTERSECTION_1b, __rotaPoint(ROAD_POINT_B1, -math.pi/2, "S", "W")), "C1b->WB1"),    
    # 西向道路路径 出口
    Path((__rotaPoint(ROAD_POINT_C0, -math.pi/2, "S", "W"),INTERSECTION_1c),  "WC0->C1c"),  
    Path((__rotaPoint(ROAD_POINT_D1, -math.pi/2, "S", "W"), __rotaPoint(ROAD_POINT_D0,-math.pi/2, "S", "W"), INTERSECTION_11), "WD1->WD0->C11", voice=u"前方右转。"),
) + south_paths + east_paths + north_paths + west_paths

# 深度优先返回第一条找到的路径
def __getFirstPathByDepth(start, end):
    pathused = [ 0 for x in range(len(MAP_PATHS))]              # 保存以访问的路径
    
    def _getpath(s, e, used):
        for i in range(len(MAP_PATHS)):
            if used[i] == 0 and MAP_PATHS[i].path[0].equal(s):
                used[i] = 1
                if MAP_PATHS[i].path[-1].equal(e):
                    return [i]
                else:
                    for park in Parks:                          # 如果为停车位，返回路径为空
                        if park.equal(MAP_PATHS[i].path[-1]):
                            return []
                    ret = _getpath(MAP_PATHS[i].path[-1], e, used)
                    if len(ret) != 0:
                        ret.append(i)
                        return ret
        return []
        
    return _getpath(start, end, pathused)[::-1]

# 广度优先返回第一条找到的路径
def __getFirstPathBySpan(start, end, checkPark = True):
    pathused = [ 0 for x in range(len(MAP_PATHS))]              # 保存以访问的路径
    wkque = []
    
    # 获取当前节点子节点：pa:从根节点到当前节点路径，s：当前节点
    def _getnext(pa, s, used):
        class node:
            def __init__(self, no, path):
                self.no = no
                self.path = path
                
        sps = []
        for i in range(len(MAP_PATHS)):
            if used[i] == 0 and MAP_PATHS[i].path[0].equal(s):
                path = copy.deepcopy(pa)                        # 复制节点路径
                path.append(i)                                  # 生成字节的路径
                sps.append(node(i, path))
        return sps
    wkque = wkque + _getnext([], start, pathused)
    while len(wkque)>0:
        i = wkque[0]
        del wkque[0]
        if MAP_PATHS[i.no].path[-1].equal(end):  
            return i.path
        else:
            isPark = False
            if checkPark:
                for park in Parks:                              # 如果为停车位，返回路径为空
                    if park.equal(MAP_PATHS[i.no].path[-1]):
                        isPark = True
            if not isPark:
                nd = _getnext(i.path, MAP_PATHS[i.no].path[-1], pathused)
                wkque = wkque + nd
    return []

# 深度优先算法搜索路线
def getFirstPathByDepth(start, end):
    path = []
    pi = __getFirstPathByDepth(start, end)
    for i in pi:
        path.append(MAP_PATHS[i])
    return path

# 广度优先算法搜索路线
def getFirstPathBySpan(start, end, checkPark=True):
    path = []
    pi = __getFirstPathBySpan(start, end, checkPark)
    for i in pi:
        path.append(MAP_PATHS[i])
    return path

# 计算线段上的点坐标
def linePoint(p1, p2, per):
    x = p1.x + (p2.x - p1.x) * per
    y = p1.y + (p2.y - p1.y) * per
    return Point(x,y)

# 计算两点距离
def distence(a, b):
    dx = a.x-b.x
    dy = a.y - b.y
    return math.sqrt(dx*dx + dy*dy)

# 生成贝塞尔曲线
def curve(p1, p2, p3, number=10):
    ret = []
    for t in range(number):
        per = float(t)/number
        sp1 = linePoint(p1, p2, per)
        ep1 = linePoint(p2, p3, per)
        cp  = linePoint(sp1, ep1, per)
        ret.append(cp)
    ret.append(p3)
    return ret

# 将一条路径转换为贝塞尔曲线路径
def path2curve(path, DISTENCE=0.3):
    if len(path.path) <= 2:
        return path
    npoint = []
    p1 = path.path[0]
    p2 = path.path[1]
    
    npoint.append(p1)
    for p3 in path.path[2:]:
        cp1 = linePoint(p1, p2, 0.5)                            # 计算p1，p2 中点坐标
        cp3 = linePoint(p2, p3, 0.5)                            # 计算p2，p3 中点坐标

        d12 = distence(cp1,p2)      
        if d12 > DISTENCE:      
            # 如果cp1离p2的距离大于DISTENCE，将cp1设置为离p2 DISTENCE处
            cp1 = linePoint(p1, p2, (d12*2-DISTENCE)/(d12*2))
        
        d23 = distence(p2,cp3)
        if d23 > DISTENCE:
            # 如果cp3离p2的距离大于DISTENCE，将cp3设置为离p2 DISTENCE处
            cp3 = linePoint(p2, p3, (DISTENCE)/(d23*2))
            
        npoint += curve(cp1, p2, cp3)
        p1 = p2
        p2 = p3
        
    npoint.append(p3)
    return Path(npoint) 
    
# 将制定的路线生成贝塞尔曲线路线
def path2curveEx(path, DISTENCE=0.3):
    cpath = copy.deepcopy(path)
    cpath[0].path = list(path2curve(path[0]).path)
    if len(path) > 1:
        lp = path[0]
        idx = 0
        del cpath[0].path[-1]                                   # 删除最后一个点
        for pp in path[1:]:
            if distence(lp.path[-1], pp.path[0]) > 0.001:
                print 'err', idx, 'distance', distence(lp.path[-1], pp.path[0])
            pt = [lp.path[-2], lp.path[-1], pp.path[1]]
            cp = Path(path2curve(Path(pt), DISTENCE).path[1:-1])# 连接前后路径曲线化
            
            cpath[idx].path += cp.path[:len(cp.path)/2]
            cpath[idx+1].path = cp.path[len(cp.path)/2:]
            cpath[idx+1].path += list(path2curve(pp, DISTENCE).path[1:-1]) # 当前路径曲线化
            idx += 1
            lp = pp
        cpath[idx].path.append(lp.path[-1])                     # 增加终点坐标
    return cpath
   
# 打印所有路径线
def dumpPath(path):
    sp = []
    for p in path:
        if p.trafficlight:
            sp.append( "["+p.tag()+"]" )
        else:
            sp.append( p.tag() )
    print " ".join(sp)

# 单元测试：加载电子地图，显示路径点、路径线
if __name__ == '__main__':
    parser = argparse.ArgumentParser(" ".join(sys.argv))
    parser.add_argument('-t','--type', default='0')
    parser.add_argument('-s','--start', default='0')
    parser.add_argument('-e','--end', default='1')
    args = parser.parse_args()
    start = int(args.start)
    end = int(args.end)
    
    def showParks():                                            # 显示所有停车点
        img = map.getImage()
        for p in Parks:
            map.drawPoint(img, p, name=p.tag())
        cv.imshow("map", img)
        cv.waitKey(0)
        cv.destroyWindow("map")

    def showRoads():                                            # 显示所有路径点
        img = map.getImage()
        for p in Roads:
            map.drawPoint(img, p, name=p.tag())
        cv.imshow("map", img)
        cv.waitKey(0)
        cv.destroyWindow("map")
    
    def showPaths():                                            # 显示所有路径
        img = map.getImage()
        for p in MAP_PATHS:
            map.drawPath(img, (p,), (255,0,255), 2)
        cv.imshow("map", img)
        cv.waitKey(0)
        cv.destroyWindow("map")
        
    def showPath(st, end):                                      # 显示指定路径
        img = map.getImage()
        path = getFirstPathBySpan(st, end)
        dumpPath(path)
        map.drawPath(img, path, (255,0,255), 2)
        cv.imshow("map", img)
        cv.waitKey(0)
        cv.destroyWindow("map")
        
    def showPath2(st, end):                                     # 显示曲线化后的指定路径
        img = map.getImage()
        path = getFirstPathBySpan(st, end)
        dumpPath(path)
        map.drawPath(img, path2curveEx(path, 0.3), (255,0,255), 2)
        cv.imshow("map", img)
        cv.waitKey(0)
        cv.destroyWindow("map")
    
    if args.type == "0" :                                       # 显示所有停车点
        showParks()
        
    if args.type == "1" :                                       # 显示所有路径点
        showRoads()
    
    if args.type == "2" :                                       # 显示所有路径线
        showPaths()
    
    if args.type == "3" :                                       # 显示指定路径线
        showPath(Parks[start], Parks[end]);
    
    if args.type == "4":                                        # 显示曲线化后的指定路径线
        showPath2(Parks[start], Parks[end]);
















