import argparse
import time
from Robotarm import Robotarm

parser = argparse.ArgumentParser()
parser.add_argument("-p", "--port",help="Input serial port",type=str,default="/dev/ttyACM0")
parser.add_argument('-r', "--rate",help="Input baudrate",type=int, default=9600)
args = parser.parse_args()

serialPort =  args.port  #"COM22" 
baudRate =  args.rate 
#range of angel for each servos
angel_range=[[60,10,15,15,55,10,10,10,10],  #min
[145,135,95,70,125,170,170,170,170]]    #max

angels=[60,10,15,15,55,90,0,0,90,30,1] #angels is the vector representing the angels of different servos []
arm=Robotarm(serialPort, baudRate, angels)
final_index=[90,10.5,7,90,0,0,0,0,0,30,1]
#control_index=[90,10.5,7,90,0,0,0,0,0]
control_index=[90,10.5,11,90,1,1,1,1,1,30,1]

def wait():
        while 1:
            data = arm.readline()
            if data!=b'' and data[0]==102:
                print(data)
                break

def main():
    try:
        while 1:
            arm.control(control_index)
            data = arm.readline()
            if data!=b'':
                print(data[0])
                if data[0]==98:
                    break
        print("begin")
        while 1:
            arm.prepare()
            wait()
            arm.prepare2()
            wait()
            #time.sleep(5)
            arm.st_jd_b()
            wait()
            time.sleep(3)
    except KeyboardInterrupt:
        arm.control(final_index)
        arm.close()

if __name__ == "__main__":
    main()
