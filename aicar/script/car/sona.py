#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Range
import time

class SonaCb:
    def __init__(self):
        self.range = -1
        self.update_time = 0
    def __call__(self, msg): 
        self.range = msg.range
        self.update_time = time.time()
class PluseSona:
    
    def __init__(self):
        self.sonas = []
        for i in range(1,5):
            self.sonas.append(SonaCb())
            rospy.Subscriber('/xcar/sonar%d'%i, Range, self.sonas[i-1])


        
    def getDistance(self, dir=0):
        if dir >=0 and dir <= 3:
            if time.time() - self.sonas[dir].update_time < 2:
                return self.sonas[dir].range
        return -1
        
if __name__ == '__main__':
    import rospy
    import time
    rospy.init_node("sano-demo", log_level=rospy.INFO)
    p = PluseSona()
    for i in range(60):
        print p.getDistance()
        time.sleep(1)
        