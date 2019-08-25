import serial
import time
import math
import os
import random
pi=3.141593

class Robotarm(object):
    def __init__(self, port="COM22", rate=9600,initial_pos=[60,10,15,15,55,0,0,0,90,30,1]):
        self.PORT = port
        self.RATE = rate
        self.ser = serial.Serial(self.PORT, self.RATE, timeout=0.5)
        if self.ser.isOpen()==0:
            print("Serial Open Error!!")
        else:
            self.send_control(initial_pos)

    def send_control(self, angels): #angels is the vector representing the angels of different servos [大拇指，食指，中指，无名指，小指，手腕旋转，第一节手臂，第二节手臂，转动速度（5-30，越小越快）,最后一个通信参数（默认为1)]
        
        intangel=[0,0,0,0,0,0,0,0,0,0,0]
        if len(angels)!=11:
            print("WRONG INPUT!!!")
            return
        for i in range(0,11):
            intangel[i]=round(angels[i])
        print(intangel)
        send_bytes=bytes(intangel)
        print(send_bytes)
        self.ser.write(send_bytes)

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
        if middle1>1 or middle2>1 or middle1<0 or middle2<0:
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
        angel_range=[[60,10,15,15,55,10,10,10,10],  #min
        [145,135,95,70,125,170,170,170,170]]    #max
        finger=[0,0,0,0,0]
        for i in range(0,5):
            finger[i]=(angel_range[1][i]-angel_range[0][i])*control_index[4+i]+angel_range[0][i]
        speed = control_index[9]
        self.send_control([finger[0],finger[1],finger[2],finger[3],finger[4],theta3,180-theta2,theta1,theta0,speed,control_index[10]])
        
    def prepare(self):
        self.control([90,0,15,0,0.8,0.8,0.8,0.8,0.8,30,1])
    def prepare2(self):
        self.control([90,16,5,0,0.6,0.6,0.6,0.6,0.6,7,1])

    def st_jd_b(self):
        mode=random.randint(0,2)
        if mode==0:  #stone
            self.control([90,16,5,0,1,1,1,1,1,5,1])
            print("stone")
        elif mode==1:  #cut
            self.control([90,16,5,0,1,0,0,1,1,5,1])
            print("cut")
        elif mode==2:  #napkin
            self.control([90,16,5,0,0,0,0,0,0,5,1])
            print("napkin")
        return mode


    def read(self,length):
        s=self.ser.read(length)
        return s

    def readline(self):
        s=self.ser.readline()
        return s
    
    def close(self):
        self.ser.close()
