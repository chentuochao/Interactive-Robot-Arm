import math
import os
import random
import sys
import time

import pygame
import serial
import music

pi=3.141593

class Robotarm(object):
    def __init__(self, port="COM22", rate=9600,initial_pos=[30,10,15,15,55,90,0,0,90,30,1],angel_range=[[30,10,15,15,55,0,0,0,0],[145,135,95,70,125,180,180,180,180]] ):
        self.PORT = port
        self.RATE = rate
        self.angel_range = angel_range
        self.ser = serial.Serial(self.PORT, self.RATE, timeout=0.5)

        if sys.platform.lower().find('linux') != -1 or sys.platform.lower().find('darwin') != -1:
            self.file = '../music/rap.wav'
        elif sys.platform.lower().find('win32') != -1:
            self.file = '..\\music\\rap.wav'
        else:
            self.file = None
        # playsound(self.file)
        if self.ser.isOpen()==0:
            print("Serial Open Error!!")
        else:
            print("参数设置：串口=%s ，波特率=%d" % (self.PORT, self.RATE))
            send_bytes = bytes(initial_pos)
            self.ser.write(send_bytes)

    def initial_set(self):
        while 1:
            send_bytes = bytes([60,10,15,15,55,0,0,0,90,30,1])
            self.ser.write(send_bytes)
            #print("send successfully")
            data = self.readline()
            print(data)
            if data!=b'':
                #print(data[0])
                if data[0]==98:
                    break
        print("begin")

    def send_control(self, angels): #angels is the vector representing the angels of different servos [大拇指，食指，中指，无名指，小指，手腕旋转，第一节手臂，第二节手臂，转动速度（5-30，越小越快）,最后一个通信参数（默认为1)]
        intangel=[0,0,0,0,0,0,0,0,0,0,0]
        if len(angels)!=11:
            print("WRONG INPUT!!!")
            return
        for i in range(0,11):
            intangel[i]=round(angels[i])
        for i in range(0,9):
            if intangel[i]>self.angel_range[1][i] or intangel[i]<self.angel_range[0][i]:
                print("Angel"+str(i)+": Invalid angle!!!")
                print("It should be within ["+str(self.angel_range[0][i] )+','+str(self.angel_range[1][i] )+']' )
                return
        # print(intangel)
        send_bytes=bytes(intangel)
        self.ser.write(send_bytes)
        if angels[10] == 1:
            self.wait()

    def control(self, control_index):  #control_index包括：[柱坐标系theta,柱坐标系r,柱坐标系z,手腕角度,手指弯曲程度（5个），转动速度（5-30，越小越快）,最后一个通信参数(默认为1)]
        l1=10.5
        l2=7
        if len(control_index)!=11:
            print("WRONG INPUT!!!")
            return
        theta0=control_index[0]
        r=control_index[1]
        z=control_index[2]
        middle1=(math.pow(r,2)+math.pow(z,2)+math.pow(l1,2)-math.pow(l2,2))/(2*l1*math.sqrt(math.pow(r,2)+math.pow(z,2)))
        middle2=(math.pow(r,2)+math.pow(z,2)+math.pow(l2,2)-math.pow(l1,2))/(2*l2*math.sqrt(math.pow(r,2)+math.pow(z,2)))
        if middle1>1 or middle2>1 or middle1<-1 or middle2<-1:
            print("Position Error!!!")
            return
        #print(middle1,middle2)
        if z==0:
            theta1_1=math.asin(middle1)-math.pi/2
            theta1_2=math.pi-math.asin(middle1)-math.pi/2
            theta2_1=math.asin(middle2)-math.pi/2
            theta2_2=math.pi-math.asin(middle2)-math.pi/2 
        else:
            theta1_1=math.asin(middle1)-math.atan(r/z)
            theta1_2=math.pi-math.asin(middle1)-math.atan(r/z)
            theta2_1=math.asin(middle2)-math.atan(r/z)
            theta2_2=math.pi-math.asin(middle2)-math.atan(r/z)
        theta1_min=min(theta1_1, theta1_2)
        theta1_max=max(theta1_1, theta1_2)
        theta2_min=min(theta2_1, theta2_2)
        theta2_max=max(theta2_1, theta2_2)
        if theta1_min<-0.01*math.pi:
            theta1=theta1_max
            theta2=theta2_min
        else:
            theta1=theta1_min
            theta2=theta2_max

        theta1=math.degrees(theta1)
        theta2=90 + math.degrees(theta2) - theta1
        theta3=control_index[3]
        #print(theta1,theta2)
        if theta0>180.01 or theta0<-0.01 or theta1>180.01 or theta1<-0.01 or theta2>180.01 or theta2<-0.01 or theta3>180.01 or theta3<-0.01:
            print("Out of angle range!!!")
            return
        finger=[0,0,0,0,0]
        for i in range(0,5):
            finger[i]=(self.angel_range[1][i]-self.angel_range[0][i])*control_index[4+i]+self.angel_range[0][i]
        speed = control_index[9]
        self.send_control([finger[0],finger[1],finger[2],finger[3],finger[4],theta3,180-theta2,theta1,theta0,speed,control_index[10]])
        
    def prepare(self):
        self.control([90,7,11,90,0.8,0.8,0.8,0.8,0.8,30,1])
    
    def fuck(self):
        '''
        The gesture for "Fuck you !"
        '''
        mode=random.randint(0,1)
        print(mode)
        if mode == 0:
            self.control([90, 10.5, 7, 0, 1, 1, 0, 1, 1, 12, 1])
        else:
            #self.control([170, 6, 15, 90, 1, 0, 1, 1, 1, 12, 1])
            self.send_control([self.angel_range[0][0],self.angel_range[0][1],self.angel_range[0][2],self.angel_range[0][3],self.angel_range[0][4], 70,0,40,90,   12,1])
            self.send_control([self.angel_range[1][0],self.angel_range[0][1],self.angel_range[1][2],self.angel_range[1][3],self.angel_range[1][4], 70,0,40,170,   5,1])
            for i in range(0, 2):
                self.send_control([self.angel_range[1][0],self.angel_range[0][1],self.angel_range[1][2],self.angel_range[1][3],self.angel_range[1][4], 90,70,40,170,   10,1])
                print("sdsda")
                self.send_control([self.angel_range[1][0],self.angel_range[0][1],self.angel_range[1][2],self.angel_range[1][3],self.angel_range[1][4], 90,30,40,170,   10,1])
                #self.control([160, 4, 15, 90, 1, 0, 1, 1, 1, 15, 1])
                #self.control([160, 8, 15, 90, 1, 0, 1, 1, 1, 15, 1])
            self.send_control([self.angel_range[1][0],self.angel_range[0][1],self.angel_range[1][2],self.angel_range[1][3],self.angel_range[1][4], 90,45,40,170,   7,1])
            #self.control([160, 6, 15, 90, 1, 0, 1, 1, 1, 12, 1])

    def Robot_win(self):
        '''
        If you win, the robot will praise you
        '''
        self.control([90, 10.5, 7, 180, 1, 0, 0, 1, 1, 12, 1])

    def Human_win(self):
        '''
        If you win, the robot will praise you
        '''
        self.control([90, 17, 3, 90, 0, 1, 1, 1, 1, 10, 1])

    def shake_hand(self):
        while 1:
            data = self.readline()
            if data!=b'' and data[0]==115:
                print(data)
                break

    def Rock(self):
        '''
        Rock and playing music
        '''
        music.play(self.file)
        self.control([10,10.5,7,90,1,0,1,1,0,15,1])
        self.send_control([145,10,95,70,55, 90,0,0,10,   5,1])
        self.send_control([145,10,95,70,55, 90,30,0,10,   5,1])
        self.send_control([145,10,95,70,55, 90,0,0,10,   5,1])
        self.send_control([145,10,95,70,55, 90,30,0,10,   5,1])
        '''
        self.send_control([145,10,95,70,55, 90,180,70,10,   20,1])
        self.send_control([145,10,95,70,55, 90,150,60,10,   20,1])
        self.send_control([145,10,95,70,55, 90,180,70,10,   20,1])
        self.send_control([145,10,95,70,55, 90,150,60,10,   20,1])
        
        self.send_control([145,10,95,70,55, 90,180,70,10,   20,1])
        self.send_control([145,10,95,70,55, 90,150,60,10,   20,1])
        self.send_control([145,10,95,70,55, 90,180,70,10,   20,1])
        self.send_control([145,10,95,70,55, 90,150,60,10,   20,1])
        '''
        self.send_control([60,10,15,15,55, 90,45,0,90,   20,1])
        print("Waiting to shake hand")
        self.shake_hand()
        self.send_control([60,115,85,60,115, 90,45,0,90,   10,1])
        time.sleep(0.5)
        self.send_control([60,115,85,60,115, 90,45,50,90,   10,1])
        self.send_control([145,135,95,70,125, 180,100,50,90,   10,1])
        time.sleep(0.5)
        self.send_control([145,135,95,70,125, 180,50,0,90,   15,1])
    
    def come_on(self, num = 2):
        '''
        勾引小gay gay
        '''
        for i in range(0, num):
            self.control([90, 15, 6, 0, 1, 0,   1, 1, 1, 10, 1])
            self.control([90, 15, 6, 0, 1, 0.9, 1, 1, 1, 10, 1])

        

    
    def reset(self):
        final_index=[90,10.5,7,90,0,0,0,0,0,30,0]
        self.control(final_index)

    def st_jd_b(self):
        #self.control([90,16,5,90,0.6,0.6,0.6,0.6,0.6,7,0])
        mode=random.randint(0,2)
        if mode==0:  #stone
            self.control([90,16,5,90,1,1,1,1,1,8,0])
            print("stone")
        elif mode==1:  #cut
            self.control([90,16,5,90,1,0,0,1,1,8,0])
            print("cut")
        elif mode==2:  #napkin
            self.control([90,16,5,90,0,0,0,0,0,8,0])
            print("napkin")
        return mode

    def wait(self):
        while 1:
            data = self.readline()
            if data!=b'' and data[0]==102:
                print(data)
                break

    def read(self,length):
        s=self.ser.read(length)
        return s

    def readline(self):
        s=self.ser.readline()
        return s
    
    def close(self):
        self.ser.close()
