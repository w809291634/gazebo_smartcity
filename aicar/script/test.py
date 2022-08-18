#!/usr/bin/python
# -*- coding: utf-8 -*-
##############################################################################################
# 文件：map_path.py
# 作者：Zonesion Xuzhy 20220412
# 说明：路径规划算法
# 修改：
# 注释：
##############################################################################################
import os
import math
import time
from map import map_path
from map import map
import copy
import sys
import threading
from tracking.move_to_pose import move_to_pose
import cvwin  
import random
this = sys.modules[__name__]
Parks =  map_path.Parks 

this.pluse_radar = None
this.pluse_sona = None
import sound
import platform as pf

CarPlate = u"鄂AWH001"

# 交通标志检测模式:
# 1：只检测红灯，其他情况通行，默认模式
# 2：车辆行驶严格按交通指示牌行驶
lightDetectMode = 1 
# 调度模式：0，自动巡航，1，任务调度
this.workMode = 0
# 小车控制：0，小车正常运行，1，强制停止小车
this.carStop = 0
# 小车任务：
TASK_DEFAULT    = 0 #默认导航任务
TASK_NAVIGATION = 1 #导航任务
TASK_PICKUP     = 2 #取货任务
TASK_DELIVERY   = 3 #送货任务
SUPPORT_TASKS = [TASK_DEFAULT,TASK_NAVIGATION,TASK_PICKUP,TASK_DELIVERY]

# 小车对象：
class AiCar:
    def __init__(self, car, goWithPath, trafficsign=None, arm=None):
        self.car = car
        #self.pluse_radar = None
        self.nvPath = []    #原始导航路径
        self.cvPath = []    #曲线化之后的路径
        self.flowPos = None #当前路径前置点 
        
        self.targetPos = None
        self.autoMove = False       #自动驾驶中
        self.goWithPath = goWithPath
    
        self.randomRun = False      #是否开启随机路径选择
        self.next_command = None    #
        
        t = threading.Thread(target=self.__ui_map)
        t.setDaemon(True)
        t.start()
        
        t = threading.Thread(target=self.__run)
        t.setDaemon(True)
        t.start()

        self.tags = []              #交通标志识别结果
        self.tags_pp = []
        
        self.trafficsign = trafficsign
        if self.trafficsign != None:
            self.trafficsign.register(self.__trafficsignCallback)
     
        self.arm = arm
        self.objType = 0    #抓取目标类别索引0：随机
        
        self.onTargetFinishCall = None
        self.callParam = ()
        self.ObsCount = 0
        self.lastObsTime = None #最后检测到障碍物时间
        if arm != None:
            #加载目标检测模型
            from obj_detection_rk3399 import detection
            obj_model_name = 'pill_detection_20220426'
            self.obj_class_names = ['box1', 'box2', 'box3', 'box4']
            self.__objDetection =  detection.ObjDetect(obj_model_name, self.obj_class_names)
            import locObj
            self.locObj = locObj.LocObject(self.__findObj)
            import xcar_odom
            xcar_odom.init()
            self.xcar_odom = xcar_odom 
            from marm_visual_inspection.srv import GenerateSolutions
            import rospy
            self.Solutions_client = rospy.ServiceProxy('/grasp_filter_test/GenerateSolutions', GenerateSolutions)  
    
    # 调用目标检测模型识别物品
    def __findObj(self, frame):
        img, rect, types, pp = self.__objDetection.detect(frame)
        if len(types) == 0:
            return img, [], '', 0
        if self.objType == 0:
            return img, rect[0], types[0],pp[0]
        else:
            i = 0
            for t in types:
                idx = self.__objDetection.getClassIdx(t)+1
                if idx == self.objType:
                    break
                i += 1
            if i >= len(types):
                return img, [], ''
            return img, rect[i], types[i],pp[i]
    
    # 控制小车靠近物品并抓取
    def grapObject(self, clstype=0, cb=None):
        if self.arm == None:
            return -1 
        import tf
        import rospy
        from geometry_msgs.msg import PoseStamped, Pose
        arm = self.arm
        xcar_odom = self.xcar_odom 
        dx = this.pluse_radar.getObstacleDistance(0)
      
        xcar_odom.go(dx-0.35,0, 0, 0.03, 0.05) #往前走0.1m 太远可能检测不到目标
        self.objType = clstype
        self.locObj.startCamera(640,480)
        pos, size, type = self.locObj.locObject(40)
        self.locObj.stopCamera()
        if pos == None:
            sound.say(u"没有检测到目标。", False)
            if cb != None:
                cb(0)
            return -2
      
        '''移动车辆到目标物体附件待抓取'''
        
        tf_listener = tf.TransformListener()
     
        point_trans = tf_listener.waitForTransform("base_link", pos.header.frame_id, rospy.Time(0), rospy.Duration(1))
        pos = tf_listener.transformPoint("base_link", pos)
        print 'baselink', pos
        xpos = (pos.point.x - 0.35, pos.point.y)
        xcar_odom.go(xpos[0],xpos[1], 0, 0.02, 0.05)
        self.locObj.startCamera()
        pos, size, type = self.locObj.locObject(40)
        self.locObj.stopCamera()
        if pos == None:
            sound.say(u"没有检测到目标。", False)
            if cb != None:
                cb(0)
            return -2
        if cb != None:
            tidx = self.obj_class_names.index(type)+1
            cb(tidx)
        point_trans = tf_listener.waitForTransform("base_link", pos.header.frame_id, rospy.Time(0), rospy.Duration(1))
        pos = tf_listener.transformPoint("base_link", pos)
        print 'baselink', pos   
        x = pos.point.x 
        y = pos.point.y
        z = pos.point.z + size[1]/2 + 0.02#从顶部抓取  
       
        print 'arm go', x, y, z
        Object_pose=Pose()
        Object_pose.position.x=x
        Object_pose.position.y=y
        Object_pose.position.z=z
        Object_pose.orientation.x=0
        Object_pose.orientation.y=0
        Object_pose.orientation.z=0
        Object_pose.orientation.w=1
       
        response = self.Solutions_client(Object_pose)   
        if  len(response.ik_solutions[0].positions)>0:
            joint_positions = response.ik_solutions[1].positions
            print joint_positions
            arm.setGripper(False)
            time.sleep(1)
            arm.set_joint_value_target(joint_positions)
            time.sleep(1)
            joint_positions = response.ik_solutions[0].positions
            arm.set_joint_value_target(joint_positions)
            time.sleep(1)
            arm.setGripper(True)
            time.sleep(1)
            arm.goHome()
            time.sleep(1)
        else:
            print 'arm  go fail'
            
        xcar_odom.go(-xpos[0],-xpos[1],0, 0.05, 0.1)
        return 0 
    
    # 控制小车靠近货柜释放物品
    def dropObject(self):
        '''
            释放目标
        '''
        if self.arm == None:
            return 
      
        from geometry_msgs.msg import PoseStamped, Pose
        
        dx = this.pluse_radar.getObstacleDistance(0)
        
        arm = self.arm
        xcar_odom = self.xcar_odom 
        pos = PoseStamped() 
        pos.pose.position.x = dx - 0.15
        pos.pose.position.y = 0
        pos.pose.position.z = 0.0
            
        xcar_odom.go(pos.pose.position.x, pos.pose.position.y, 0, 0.03,0.15)
        #x = 0.3
        #y = 0
        #z = 0.15 + pos.pose.position.z   # 物体高度
        
        #joint_positions = (0.0, 1.047, 0.873, 0.524, 0) #释放位置舵机角度
        joint_positions = (0, 0.65, 0.62, 0.524, 0) 
        arm.set_joint_value_target(joint_positions)
        time.sleep(1)
        joint_positions = (0, 0.75, 0.72, 0.624, 0) 
        arm.set_joint_value_target(joint_positions)

        time.sleep(1)
        arm.setGripper(False)
        time.sleep(1)
        arm.goHome()
        time.sleep(1)
        arm.setGripper(True)
        time.sleep(1)
        xcar_odom.go(-(pos.pose.position.x), -pos.pose.position.y, 0, 0.05, 0.2)
    
    # 设置小车初始位置
    def setInitPosIdx(self, pidx, playsound=True):
        self.car.initPos(Parks[pidx].x, Parks[pidx].y, Parks[pidx].yaw)
        self.currentEndPos = Parks[pidx]
        if playsound:
            sound.say(u"设置初始位置为%d号停车位"%(pidx+1))
        
    # 交通标志识别回调函数
    def __trafficsignCallback(self, frame, rets, types, pp):
        
        self.tags = types
        self.tags_pp  = pp
        
    def setCarPos(self, p):
        self.car.initPos(p.x, p.y, p.yaw)
        self.currentEndPos = p
    
    # 设置小车目标位置
    def setTargetPos(self,pos, onFinishCall=None, *param):
        self.onTargetFinishCall = onFinishCall
        self.callParam = param
        self.targetPos = pos
                           
    # 调用电子地图显示信息 
    def __ui_map(self):
        lnvPath = None
        gpath = []
        
        sp1 = self.car.currentPos()
        while True:
            if lnvPath != self.cvPath:              #如果全局路径改变，绘制全局路径
                lnvPath = self.cvPath
                bgimg = map.getImage()
                map.drawPath(bgimg, self.cvPath, (0,255,0), 2)
                img = copy.deepcopy(bgimg)
                sp1 = self.car.currentPos()
                map.drawCar(img, sp1[0], sp1[1], sp1[2])
                cvwin.imshow("map", img)
                
                gpath = []
            
            sp2 = self.car.currentPos()
            dx = sp2[0] - sp1[0]
            dy = sp2[1] - sp1[1]
            da = sp2[2] - sp1[2]
            d = math.sqrt(dx*dx + dy*dy)
            if d > 0.01 or abs(da)> 0.26:
                img = copy.deepcopy(bgimg)
                if self.flowPos != None:            #绘制路径跟随点
                    map.drawPoint(img, self.flowPos)
                
                if self.autoMove:                   #绘制小车已行驶的路径
                    gpath.append(map_path.Point(sp2[0], sp2[1]))
                    map.drawPath(img, (map_path.Path(gpath),), (0,0,255), 2)

                map.drawCar(img, sp2[0], sp2[1], sp2[2])    #绘制小车
                gpos = self.car.getGPSPos()
                if gpos != None and time.time()-gpos[0]<1 and gpos[1]!=float('NaN') and gpos[2]!=float('NaN'):
                    #计算gps位置与小车当前位置的距离
                    d2 = (gpos[1]-sp2[0])*(gpos[1]-sp2[0]) + (gpos[2]-sp2[1])*(gpos[2]-sp2[1])
                    if d2==d2 and d2 <0.35: #d2==d2 nan 判断
                        d = math.sqrt(d2)
                        #已小车当前位置为圆心，d为半径画圆
                        map.drawCircle(img, sp2[0], sp2[1], d)
                        if display_coord == 2:
                            map.drawUWBPos(img, sp2[0], sp2[1], gpos[1], gpos[2] )    #绘制小车uwb坐标 
                            map.drawRadarPos(img, sp2[0], sp2[1], sp2[2])    #绘制小车雷达坐标

                if display_coord == 1 :   
                    map.drawRadarPos(img, sp2[0], sp2[1], sp2[2])    #绘制小车雷达坐标 
                cvwin.imshow("map", img)
            time.sleep(0.1)

    # 监听进入到新的路径线
    def __onPathIn(self, xpath):
        #print xpath.name, xpath.trafficlight, xpath.voice
        self.currentEndPos = xpath.path[-1]
        if  xpath.trafficlight:
            sound.say(u"前方红绿灯。")
        if xpath.voice != None:
            sound.say(xpath.voice)
        self.next_command = None

    # 监听离开当前路径线          
    def __onPathOut(self, xpath):
        if xpath.trafficlight:
            '''根据目的停车位计算出口是左转还是直行'''
            #bug 依赖目的停车位无通用性
            def get_quadrant(p): #计算p点所在的象限
                if p.x*p.y > 0:
                    if p.x > 0:
                        ret = 1 #
                    else:
                        ret = 3
                else:
                    if p.x > 0:
                        ret = 4
                    else:
                        ret = 2
                return ret
            sp = get_quadrant(self.currentEndPos) #当前位置所在象限
            ep = get_quadrant(self.targetPos)   #目标位置所在象限
            #根据目标停车位将目标象限细分位8个区域
            if ep == 1:
                if abs(self.targetPos.x)>abs(self.targetPos.y):
                    eep = 1
                else:
                    eep = 2
            elif ep == 2:
                if abs(self.targetPos.y)>abs(self.targetPos.x):
                    eep = 3
                else:
                    eep = 4
            elif ep == 3:
                if abs(self.targetPos.x)>abs(self.targetPos.y):
                    eep = 5
                else:
                    eep = 6
            elif ep == 4: 
                if abs(self.targetPos.y)>abs(self.targetPos.x):
                    eep = 7
                else:
                    eep = 8
            #直行判断
            straight={
                1: (4,5),
                2: (6,7),
                3: (8,1),
                4: (2,3),
            }
            left = {
                1: (6,7),
                2: (8,1),
                3: (2,3),
                4: (4,5)
            }
            around = {
                1: (8,1),
                2: (2,3),
                3: (4,5),
                4: (6,7)
            }
            if eep in straight[sp]:
                v = ('straight', u'前方直行')
            if eep in left[sp]:
                v = ('left', u'前方左转')
            if eep in around[sp]:
                v = ('left', u'前方掉头')

            def getLightStatus():
                if 'right' in self.tags:
                    #sound.say(u"识别到右转")
                    return 'right'
                elif 'left' in self.tags:
                    #sound.say(u"识别到左转")
                    return 'left'
                elif 'red' in self.tags:
                    #sound.say(u"识别到红灯")
                    return 'red'
                elif 'straight' in self.tags:
                    #sound.say(u"识别到直行")
                    return 'straight'
                return ''
            
            lastLightStatus = ""
            lightStatus = getLightStatus()
            if lightStatus != "":
                while True:
                    if lightStatus != lastLightStatus:
                        lastLightStatus = lightStatus
                        l2v = {
                            "red":u"红灯",
                            "right":u"右转",
                            "left":u"左转",
                            "straight":u"直行"
                        }
                        if lightStatus in l2v.keys():
                            sound.say(u"识别到"+l2v[lightStatus])
                        
                    if 'red' == lightStatus or (lightDetectMode == 2 and lightStatus != v[0]):
                        self.car.updateSpeed(0, 0, 0)
                        time.sleep(1)
                    else:
                        break
                    lightStatus = getLightStatus()
            else:
                sound.say(v[1])
    
    # 监听正在行驶的路段是否需要介入应用
    def __onPathRun(self, xpath):
        if self.next_command == None:
            #['left', 'right', 'stop', 'straight', 'red', 'green']
            if 'left' in self.tags and len(xpath.leftp) > 0:
                self.next_command = 'left' #xpath.leftp[0]
            if 'right' in self.tags and len(xpath.rightp) > 0:
                self.next_command = 'right' #xpath.rightp[0]
            if 'straight' in self.tags and len(xpath.straightp) > 0:
                self.next_command = 'straight' #xpath.straightp[0]  
            #if 'stop' in self.tags:
        x,y,yaw = self.car.currentPos() 
        dx = self.currentEndPos.x - x
        dy = self.currentEndPos.y - y
        d = math.sqrt(dx*dx + dy*dy)
       
        if d < 0.4: #小地图版本随机下一个路径点
            if self.next_command ==None and self.randomRun:
                '''随机获取下一路径点'''
                np = xpath.leftp + xpath.straightp + xpath.rightp
                if len(np) != 0:
                    npath = np[random.randint(0,len(np)-1)]
                    np = npath.leftp + npath.straightp + npath.rightp
                    while len(np) == 1:
                        npath = np[0]
                        np = npath.leftp + npath.straightp + npath.rightp
                    self.setTargetPos(npath.path[-1])
            elif self.next_command !=None:
                if self.next_command == 'left':
                    sound.say(u"前方左转。")
                    self.setTargetPos(xpath.leftp[0].path[-1])
                if self.next_command == 'right':
                    sound.say(u"前方右转。")
                    self.setTargetPos(xpath.rightp[0].path[-1])
                if self.next_command == 'straight':
                    sound.say(u"前方直行。")
                    self.setTargetPos(xpath.straightp[0].path[-1])    
                self.next_command = None
                
    def setRandomPath(self, en):
        self.randomRun = en

    def __ObstacleAvoidance(self, vx, vy, va):
        '''
            避障
            vx，vy，va当前预设速度
        '''
        if vx == 0 and vy == 0 and va == 0:
            return vx, vy, va
        if vx == 0 and vy == 0:
            return vx, vy, va  #原地旋转，没有避障
        
        v_yaw = math.atan2(vy,vx) #小车速度航向
        a1 = v_yaw - 0.7
        a2 = v_yaw + 0.7
        dis_radar = -1
        if this.pluse_radar != None:
            dis_radar, angle = this.pluse_radar.getMiniDistance(a1, a2)
        dis_sona = -1
        if this.pluse_sona != None:
            dis_sona = this.pluse_sona.getDistance()
            
        if dis_radar>0 and dis_radar < 0.6 or   \
            abs(v_yaw)<0.2 and dis_sona>0 and dis_sona < 0.4:
            self.ObsCount += 1
            if self.ObsCount >= 3:
                self.lastObsTime = time.time()
                print dis_radar, angle,dis_sona
                vx = vy = va = 0
        elif dis_radar>0 and dis_radar < 1 or \
            abs(v_yaw)<0.2 and dis_sona>0 and dis_sona < 0.8:
            vx = vx * 0.6
            vy = vy * 0.6
            va = va * 0.6
        if self.lastObsTime != None:
            if time.time() - self.lastObsTime < 3:
                vx = vy = va = 0
            else:
                self.lastObsTime = None
                self.ObsCount = 0
                
        return vx, vy, va
    
    # 按照路线执行任务
    def __runPath(self, path=None):
        '''path 为未曲线化之前的路径'''
        while True:
            path = map_path.getFirstPathBySpan(self.currentEndPos, self.targetPos, True)
            if len(path) == 0:
                path = map_path.getFirstPathBySpan(self.currentEndPos, self.targetPos, False)

            ps = self.car.currentPos()
            path.insert(0, map_path.Path((map_path.Point(ps[0], ps[1]) ,self.currentEndPos)))
            self.targetPos = path[-1].path[-1]

            self.nvPath = path
            self.cvPath = map_path.path2curveEx(path, 0.3)
            self.autoMove = True
       
            #sound.say(u'准备出发，全程%.1f米,预计用时%.0f分钟。'%(d,d/(0.2*60)))
            endRun = False
            lpath = None  
            
             
            for vx, vy, va, xpath, idxpoint in self.goWithPath(self.cvPath, self.car):
                #print "%.2f,%.2f %.2f"%(vx, vy, va),
                self.flowPos = xpath.path[idxpoint]
                
                vx, vy, va = self.__ObstacleAvoidance(vx,vy,va)
                
                if 'stop' in self.tags:
                    vx = vy = va = 0
                if xpath.trafficlight and 'red' in self.tags:
                    vx = vx * 0.7
                        
                if self.targetPos != path[-1].path[-1]: #目标位置改变
                    #if self.currentEndPos != path[-1].path[-1]: #当前路段终点不是停车场直接退出，否则走完当前路段
                    #    endRun = True
                    break
                if lpath != xpath:
                    if lpath != None:
                        idx = self.cvPath.index(lpath)
                        self.__onPathOut(self.nvPath[idx])
                    idx = self.cvPath.index(xpath)
                    self.__onPathIn(self.nvPath[idx])
                    lpath = xpath
                self.__onPathRun(self.nvPath[idx])
                #print "--->%.2f,%.2f %.2f"%(vx, vy, va)
                if this.carStop != 0:
                    vx = va = vy = 0
                self.car.updateSpeed(vx, va, vy)
                time.sleep(0.1)
            self.flowPos = None
            if self.targetPos == path[-1].path[-1]:
                dst = self.targetPos
                move_to_pose(self.car,dst.x, dst.y, dst.yaw, 0.05,0.1)
                self.car.updateSpeed(0,0,0)
                time.sleep(0.1)
                self.car.updateSpeed(0,0,0)
                time.sleep(0.1)
                self.car.updateSpeed(0,0,0)
                time.sleep(0.1)
                self.autoMove = False
                if self.targetPos == dst:
                    if self.onTargetFinishCall:
                        self.onTargetFinishCall(*self.callParam)
                        if self.targetPos == dst: 
                            self.onTargetFinishCall = None
                if self.targetPos == dst: #目的任务没有改变
                    self.targetPos = None 
                break
            
    def __run(self):
        while True:
            if self.targetPos != None:
                #path = map_path.getFirstPathBySpan(self.currentEndPos, self.targetPos, False)
                #ps = self.car.currentPos()
                #path.insert(0, map_path.Path((map_path.Point(ps[0], ps[1]) ,self.currentEndPos)))
                #self.targetPos = path[-1].path[-1]
                #target = self.targetPos
                self.__runPath()
                
            else:
                time.sleep(1)

def rk1808Check():
    if pf.uname()[0] == 'Linux':
        dev = ""
        with os.popen("ifconfig | grep enx10 | awk '{print $1}'") as f:
            dev = f.read()
        if len(dev) != 0:
            os.system("sudo ifconfig %s 192.168.180.100"%dev)
            return True
    return False
     
# 主线程
def testAiCar():
    import argparse
    sys.path.append("tracking")
    sys.path.append("car")
    sys.path.append("/usr/local/lib")
    
    parser = argparse.ArgumentParser(" ".join(sys.argv))
    parser.add_argument('-s','--start', default='0')
    parser.add_argument('-e','--end', default='0')
    parser.add_argument('-c','--control', default='pure_pursuit')
    parser.add_argument('-x','--xcar', default='vcar')
    #trafficsign是否开启交通标志检测 0:有rk1808则开启，没有则不开启，1：强制用rk1808，2：强制用cpu,3:关闭交通标志检测
    parser.add_argument('-t','--trafficsign', default='0') 
    # 是否与智云小车节点通信：0关闭；1开启
    parser.add_argument('-r','--ros', default='0')  
    # 红绿灯识别规则：1红灯停其他行；2严格按照红绿灯交规行走；3忽略红绿灯
    parser.add_argument('-l','--lightmode', default='1')  
    # 是否开启视觉识别机械臂抓取系统：0关闭；1开启
    parser.add_argument('-v','--vision', default='0')       
    parser.add_argument('-vt','--vision_test', default='0') 
    # 开启雷达&UWB数据显示：0关闭；1显示雷达；2显示雷达和UWB
    parser.add_argument('-p','--display_coord', default='0') 

    args = parser.parse_args()
    start = int(args.start)
    end = int(args.end)
    
    sound.say(u"系统启动中。", True)
    global lightDetectMode
    if args.lightmode == '2':
        lightDetectMode = 2
    global display_coord
    if args.display_coord == '1':
        display_coord = 1
    elif args.display_coord == '2':
        display_coord = 2
    else:
        display_coord=0
    rosInit = False
    trafficsign = None      
    arm  = None
    
    # 真实小车
    if args.xcar == "xcar":
        if not rosInit:
            import rospy
            rospy.init_node("xcar-demo", log_level=rospy.INFO)
            rosInit = True
        
        # 检查雷达，并初始化
        import radar
        this.pluse_radar = radar.PluseRadar()
        if not this.pluse_radar.waitRadar():
            print u"雷达系统异常。"
            sound.say(u"雷达系统异常。", True)
            sound.say(u"请检查雷达系统，并重新启动应用。", True)
            sys.exit(0)
        # 检查超声波，并初始化
        import sona
        this.pluse_sona = sona.PluseSona() 
        
        # 检查交通标志检测系统的硬件连接（摄像头、NPU），并初始化硬件
        if (not os.path.exists("/dev/video5")) and os.path.exists("/dev/video0") or \
                os.path.exists("/dev/video7"):
            if args.trafficsign == "0":
                #自动检测rk1808
                if rk1808Check():
                    import trafficsign 
                    trafficsign.init(useRK1808=True)
            elif args.trafficsign == "1":
                if rk1808Check():
                    import trafficsign
                    trafficsign.init(useRK1808=True)
                else:
                    print u"警告！没有检测到NPU,交通标志检测功能将关闭。"
                    sound.say(u"警告！没有检测到NPU,交通标志检测功能将关闭。", True)
            elif args.trafficsign == "2":
                import trafficsign
                trafficsign.init(useRK1808=False)
        else:
            print u"警告！没有检测到广角摄像头,交通标志检测功能将关闭。"
            sound.say(u"警告！没有检测到广角摄像头,交通标志检测功能将关闭。", True)
        
        # 检查机械臂的摄像头，并初始化机械臂
        if args.vision != '0':
            #检测是否安装景深摄像头
            if (not os.path.exists("/dev/video5")) and 0 != os.system("lsusb -d 8086:0ad3") :
                sound.say(u"景深摄像头异常", True)
                sound.say(u"请检查景深摄像头连接，并重新启动应用。", True)
                sys.exit(0)
            import arm
             
            arm.init()
            arm.goHome()
            
    # 虚拟小车
    if args.xcar == "vcar":
        if args.trafficsign != "3":
            import trafficsign
            trafficsign.init(useRK1808=False)

    # 加载路径执行模块
    goWithPathEx = __import__(args.control).goWithPathEx
    # 加载智能小车模块
    ACar = __import__(args.xcar).ACar

    car = ACar()
    # 执行小车任务调度服务
    acar = AiCar(car, goWithPathEx, trafficsign, arm)
    acar.setInitPosIdx(start, False) #在检测定位系统前，先初始化一次位置
    if not car.waitFirstLocation():
        print u"定位系统异常，请检查定位系统！"
        sound.say(u"定位系统异常，请检查定位系统！", True)
        sys.exit(0)
    sound.say(u"一切正常。", True)
    acar.setInitPosIdx(start)
    if end != 0: 
        #指定目的地址
        acar.setTargetPos(Parks[end%len(Parks)])
    
    # 与小车节点vnode通信
    if args.ros == "1":
        import rospy
        from std_msgs.msg import String
        from std_msgs.msg import Float32MultiArray
        rospy.init_node("xcar-demo", log_level=rospy.INFO)
        rosInit = True

        def __pubPos():
            global lastSetPos
            lastSetPos = 0

            pub = rospy.Publisher('/demo/acar/pos', Float32MultiArray, queue_size=0, latch=False)
            pubST = rospy.Publisher('/demo/acar/to_server/target', String, queue_size=0, latch=False)
            pubPlate = rospy.Publisher('/demo/acar/plate', String, queue_size=0, latch=True)
            def __onTarget(msg):
             
                if len(msg.data) > 0 :
                    #任务&任务参数1[&任务参数2][&任务参数3]...
                    #任务： 1：导航  2：取货，3：送货
                    its = msg.data.split("&")
                    if len(its) > 1:
                        start_pi = int(its[1])
                        task = int(its[0])
                        otype = 0 #取货类别
                        if task == TASK_PICKUP and len(its)>2:
                            otype = int(its[2])
                    else:
                        start_pi = int(its[0])
                        task = TASK_DEFAULT #默认任务
                   
                    
                    if task == TASK_PICKUP or task == TASK_DELIVERY:
                        if start_pi>0 and start_pi<=len(map_path.FPoints):
                            def onTaskCall_2(*pa):
                                dst = acar.targetPos
                                _task = pa[0]
                                clstype = pa[1]
                                def onTypeCall(t):
                                    report_dat=str(_task)+'&'+str(pa[2])+'&'+str(t)
                                    pubST.publish(str(report_dat))
                                    
                                if _task == TASK_PICKUP: #抓取
                                    acar.grapObject(clstype, onTypeCall)
                                elif _task == TASK_DELIVERY: #释放
                                    acar.dropObject()
                                if acar.targetPos == dst: #在抓取过程中没有改变目的地址
                                    #上报结果
                                    pubST.publish("0")
                            acar.setTargetPos(map_path.FPoints[start_pi-1], onTaskCall_2, task, otype,start_pi)
                    elif start_pi>0 and start_pi<=len(Parks):
                        def onTaskCall_3(*pa):
                            pubST.publish("0")
                        acar.setTargetPos(Parks[start_pi-1], onTaskCall_3)
                    
            rospy.Subscriber('/demo/acar/to_client/target', String, __onTarget)
            
            def __onMode(msg):
                mode = int(msg.data)
                if mode == 0:
                    this.workMode  = mode
                    sound.say(u"已切换为巡航模式。")
                elif mode == 1:
                    this.workMode  = mode
                    sound.say(u"已切换为调度模式。")
                else:
                    sound.say(u"不支持的模式。")
                
            rospy.Subscriber('/demo/acar/mode', String, __onMode)
            
            def __onCarStop(msg):
                this.carStop = int(msg.data)
            rospy.Subscriber('/demo/acar/stop', String, __onCarStop)
            
            def __onVoice(msg):
                print('type', type(msg.data),'dat', msg.data.decode('utf-8'))
                sound.say(msg.data.decode('utf-8'))
                
            rospy.Subscriber('/demo/acar/voice', String, __onVoice)
            pubPlate.publish(CarPlate)
            rate = rospy.Rate(1)
            a = Float32MultiArray()
           
            while True:
                rate.sleep()
                p1 = car.currentPos()
                st = (p1[0], p1[1], p1[2], car.speed_x, car.speed_a)
                a.data = st
                pub.publish(a)
                 
        t = threading.Thread(target=__pubPos)
        t.setDaemon(True)
        t.start()
        
    # 机械臂测试 
    if args.vision_test == "1":
        acar.grapObject()
        acar.dropObject()
        sys.exit(0)
    
    lastTargetTime = None
    lastTargetPos = None
    lastTargetTime = time.time()-25
    while True:
        if lastTargetPos != acar.targetPos:
            if lastTargetPos != None and acar.targetPos == None:
                lastTargetTime = time.time()
            lastTargetPos = acar.targetPos

        if this.workMode == 0 and acar.targetPos == None and lastTargetTime != None and time.time()-lastTargetTime>30:
            pidx = [0,2,6,8,10,12,14]
            acar.setTargetPos(Parks[random.choice(pidx)])
            lastTargetTime = None
        time.sleep(1)

# 主程序
if __name__ == '__main__':
    testAiCar()