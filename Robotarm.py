import serial
import time

class Robotarm(object):
    def __init__(self, port="COM22", rate=9600,initial_pos=[60,35,20,35,55,90,90,90]):
        self.PORT = port
        self.RATE = rate
        self.ser = serial.Serial(self.PORT, self.RATE, timeout=0.5)
        if self.ser.isOpen()==0:
            print("Serial Open Error!!")
        else:
            print("参数设置：串口=%s ，波特率=%d" % (self.PORT, self.RATE))
            self.control(initial_pos)
    
    def control(self, angels):
        if len(angels)!=8:
            print("WRONG INPUT!!!")
        send_bytes=bytes(angels)
        #print(send_bytes)
        self.ser.write(send_bytes)

    def read(self,length):
        s=self.ser.read(length)
        return s

    def readline(self):
        s=self.ser.readline()
        return s
    
    def close(self):
        self.ser.close()
