#!/usr/bin/python
# -*- coding: utf-8 -*-
import sys
import cv2
import time
import threading
#import queue

this = sys.modules[__name__]

this.wins = {}
this.delWins = []#queue.Queue(maxsize=0)

def imshow(win, img):
    wi = {
        "image":img,
        "update":True
    }
    this.wins[win] = wi
    

def destroyWindow(win):
    this.delWins.append(win)
        
def __run():
    
    while True:
        '''
        while True:
            try:
                w = this.delWins.get_nowait()
                del this.wins[w]
                cv2.destroyWindow(w)
            except:
                break
        '''
        while len(this.delWins) >0:
            w = this.delWins[0]
            
            del this.delWins[0]
            del this.wins[w]

            cv2.destroyWindow(w)
        for k in this.wins.keys():
            wi = this.wins[k]
            if wi['update']:
                wi['update'] = False
                cv2.imshow(k, wi['image'])
        cv2.waitKey(100)
        
t = threading.Thread(target=__run)
t.setDaemon(True)
t.start()