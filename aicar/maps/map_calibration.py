#!/usr/bin/python
# -*- coding: utf-8 -*-
import math
import sys
from Tkinter import *
from PIL import Image,ImageDraw, ImageTk
import platform

OS = platform.system()
def coor2point(a, origin, resolution, height):
    x = int((a[0]-origin[0])/resolution)
    y = height-int((a[1]-origin[1])/resolution)
    return (x,y)
    
def rotaPoint(p, angle):
    '''将x，y逆时针旋转angle后的新坐标'''
    nx = p[0]*math.cos(angle) - p[1] * math.sin(angle)
    ny = p[0]*math.sin(angle) + p[1] * math.cos(angle)
    return (nx,ny)
class Application(Frame):
    def __init__(self, master=None):
        Frame.__init__(self,master)
        self.pack(expand=YES,fill=BOTH)
        self.master.title(u'地图校准')
        width,height=self.master.maxsize()
        self.master.geometry("{}x{}".format(int(width*0.7),int(height*0.7)))
        
        file = open("map_gmapping.yaml")
        lines = file.read().split("\n")
        file.close()
        self.yaml = []
        for line in lines:
            it = line.split(":")
            if len(it) == 2:
                name = it[0].strip()
                val = it[1].strip()
                self.yaml.append((name,val))
                if name == 'resolution':
                    resolution = float(val)
                if name == 'origin':
                    origin = eval(val)
        self.resolution = resolution
        self.origin = origin
            
        self.fm1 = Frame(self)
        self.label_x = Label(self.fm1, text="偏移X")
        self.label_x.pack(side=LEFT,fill='y',padx=20)
        self.entry_x = Entry(self.fm1, width="10")
        self.entry_x.bind('<Return>', self.on_config_change)
        self.entry_x.insert(0, "0")
        self.entry_x.pack(side=LEFT,fill='y')
        self.label_y = Label(self.fm1, text="偏移Y")
        self.label_y.pack(side=LEFT,fill='y',padx=20)
        self.entry_y = Entry(self.fm1, width="10")
        self.entry_y.bind('<Return>', self.on_config_change)
        self.entry_y.insert(0, "0")
        self.entry_y.pack(side=LEFT,fill='y')
        self.label_r = Label(self.fm1, text="旋转")
        self.label_r.pack(side=LEFT,fill='y',padx=20)
        self.entry_r = Entry(self.fm1, width="10")
        self.entry_r.bind('<Return>', self.on_config_change)
        self.entry_r.insert(0, "0")
        self.entry_r.pack(side=LEFT,fill='y')
        
        self.bun_1 = Button(self.fm1, text=" 缩小 ", command=self.on_btn_1)
        self.bun_1.pack(side=LEFT,padx=20)
        
        self.bun_2 = Button(self.fm1, text=" 放大 ", command=self.on_btn_2)
        self.bun_2.pack(side=LEFT,padx=20)
        
        self.bun_3 = Button(self.fm1, text=u" 保存 ", command=self.on_btn_3)
        self.bun_3.pack(side=LEFT,padx=20)
        
        self.fm1.pack(side=TOP)
         
        self.fm2 = Frame(self)
        self.image = Image.open("map_gmapping.pgm")
        self.imageRGB = self.image.convert('RGBA') 
 
        #print self.image.getdata()[0]
        self.resize = 100
        self.imgLabel = Label(self.fm2)
        self.draw_image()
        self.imgLabel.pack(anchor=CENTER)
        
        if OS == "Linux" :
            self.imgLabel.bind_all('<4>', self._on_mousewheel,  add='+')
            self.imgLabel.bind_all('<5>', self._on_mousewheel,  add='+')
        else:
            # Windows and MacOS
            self.imgLabel.bind_all("<MouseWheel>", self._on_mousewheel,  add='+')
 
        self.fm2.pack(anchor=CENTER)
    
    def on_config_change(self, evt):
        self.draw_image()
    
    def on_btn_1(self):
        self.resize -= 1
        if self.resize < 10:
            self.resize = 10
        self.draw_image()
    def on_btn_2(self):
        self.resize += 1
        if self.resize > 150:
            self.resize = 150
        self.draw_image()
    def on_btn_3(self):
        r = float(self.entry_r.get())
        x = float(self.entry_x.get())
        y = float(self.entry_y.get())
        if r> 0.000001  or r < -0.000001:
            self.image = self.image.rotate(r)
            self.image.save("map_gmapping.pgm")
            self.entry_r.delete(0, END)
            self.entry_r.insert(0,'0')
        if x> 0.000001  or x < -0.000001 or y> 0.000001  or y < -0.000001:
            origin = [self.origin[0]+x, self.origin[1]+y, self.origin[2]]
            file = open("map_gmapping.yaml","w")
            for n,v in self.yaml:
                if n == 'origin':
                    v = str(origin)
                file.write("%s: %s\n"%(n, v))
            file.close()
            self.origin = origin
            self.entry_x.delete(0, END)
            self.entry_x.insert(0,'0')
            self.entry_y.delete(0, END)
            self.entry_y.insert(0,'0')
        self.draw_image()
    

    def coor2point(self, p):
        x = float(self.entry_x.get())
        y = float(self.entry_y.get())
        origin = [self.origin[0]+x, self.origin[1]+y]
        cp =  coor2point(p, origin, self.resolution,self.image.size[1])
        cx = cp[0]*self.resize/100
        cy = cp[1]*self.resize/100
        return (cx, cy)
        
    def draw_image(self):
        w = int(self.image.size[0]*self.resize/100)
        h = int(self.image.size[1]*self.resize/100)
       
        im = self.imageRGB.resize((w, h), Image.ANTIALIAS)
        r = float(self.entry_r.get())
        x = float(self.entry_x.get())
        y = float(self.entry_y.get())
        im = im.rotate(r)
        draw = ImageDraw.Draw(im)
        ''' 中心坐标 '''
        cx,cy = self.coor2point((0,0))
        draw.ellipse((cx-10,cy-10,cx+10,cy+10), outline ='red')
        
        '''4个台子校准坐标'''
        awidth = 0.6 #台子宽度
        adis = 1.9425 #台子边缘到中心点距离
        a = (((-adis,adis), (-adis-awidth, adis+awidth)), 
             ((-adis,-adis), (-adis-awidth, -adis-awidth)),
             ((adis,-adis),  (adis+awidth, -adis-awidth)),
             ((adis,adis), (adis+awidth, adis+awidth))
            )
        for p1,p2 in a:
            c1 = self.coor2point(p1)
            c2 = self.coor2point(p2)
            draw.rectangle((c1[0],c1[1],c2[0],c2[1]), outline='red') 
        
        '''8个灯杆校准坐标'''
        dg_width = 0.2
        dg_height = 0.3
        dg_p01 = (-2.8, 1.3) # 1号灯杆距离中心参考坐标
        dg01 = ((dg_p01[0], dg_p01[1]), (dg_p01[0] - dg_width, dg_p01[1]+dg_height))
        dg02 = ((dg_p01[0], -dg_p01[1]), (dg_p01[0]- dg_width, -dg_p01[1]-dg_height))
        for r in [0,math.pi/2, math.pi, -math.pi/2]:
            p1 = self.coor2point(rotaPoint(dg01[0], r))
            p2 = self.coor2point(rotaPoint(dg01[1], r))
            p3 = self.coor2point(rotaPoint(dg02[0], r))
            p4 = self.coor2point(rotaPoint(dg02[1], r))
            draw.rectangle((p1[0],p1[1],p2[0],p2[1]), outline='red')
            draw.rectangle((p3[0],p3[1],p4[0],p4[1]), outline='red')
            
        '''16个停车位位置参考'''
        MAP_ROAD_WIDTH = 0.6
        MAP_HALF_WIDTH  = 3.2        #地图一半的宽度
        MAP_HALF_HEIGHT = 3.2       #地图一半的高度

        PARK_WIDTH  =  0.5      #停车位宽度
        PARK_HEIGHT = 0.45       #停车位高度
        '''南向停车位位置定义'''
        Park01 = (-MAP_HALF_HEIGHT+PARK_HEIGHT+PARK_HEIGHT/2, MAP_ROAD_WIDTH*2-PARK_WIDTH/2)       #停车位1位置
        Park02 = (Park01[0]-PARK_HEIGHT, Park01[1])                                                  #停车位2位置
        Park03 = (Park01[0], -Park01[1])                                                  #停车位3位置
        Park04 = (Park02[0], -Park02[1]) 
        i = 1
        for r in [0,math.pi/2, math.pi, -math.pi/2]:
            
            p01 = (Park01[0]-PARK_HEIGHT/2,Park01[1]-PARK_WIDTH/2)
            p02 = (Park01[0]+PARK_HEIGHT/2,Park01[1]+PARK_WIDTH/2)
            
            p1 = self.coor2point(rotaPoint(p01, r))
            p2 = self.coor2point(rotaPoint(p02, r))
            draw.rectangle((p1[0],p1[1],p2[0],p2[1]), outline='green')
            pt = self.coor2point(rotaPoint(Park01, r))
            draw.text((pt[0]-8,pt[1]-8), "P%02d"%i, fill='red')
            i += 4
            
            p03 = (Park01[0]-PARK_HEIGHT/2 - PARK_HEIGHT,Park01[1]-PARK_WIDTH/2)
            p04 = (Park01[0]+PARK_HEIGHT/2- PARK_HEIGHT,Park01[1]+PARK_WIDTH/2)
            p3 = self.coor2point(rotaPoint(p03, r))
            p4 = self.coor2point(rotaPoint(p04, r))
            draw.rectangle((p3[0],p3[1],p4[0],p4[1]), outline='green')
            
            
            p05 = (Park01[0]-PARK_HEIGHT/2, -Park01[1]+PARK_WIDTH/2)
            p06 = (Park01[0]+PARK_HEIGHT/2, -Park01[1]-PARK_WIDTH/2)
            p5 = self.coor2point(rotaPoint(p05, r))
            p6 = self.coor2point(rotaPoint(p06, r))
            draw.rectangle((p5[0],p5[1],p6[0],p6[1]), outline='green')
            
            p07 = (Park01[0]-PARK_HEIGHT/2 - PARK_HEIGHT,-Park01[1]+PARK_WIDTH/2)
            p08 = (Park01[0]+PARK_HEIGHT/2- PARK_HEIGHT,-Park01[1]-PARK_WIDTH/2)
            p7 = self.coor2point(rotaPoint(p07, r))
            p8 = self.coor2point(rotaPoint(p08, r))
            draw.rectangle((p7[0],p7[1],p8[0],p8[1]), outline='green')
         
            
        render = ImageTk.PhotoImage(im)
        self.imgLabel.configure(image=render)
        self.imgLabel.image=render
       
 
    
    def _on_mousewheel(self, evt):
        if OS == 'Linux':
            if evt.num == 5:
               self.resize -= 1
            if evt.num == 4:
               self.resize += 1
        else:
             
            self.resize += evt.delta/120
        if self.resize < 10:
            self.resize = 10
        if self.resize > 150:
            self.resize = 150
        self.draw_image()
            
if __name__ == '__main__':
    app = Application()
    app.mainloop()