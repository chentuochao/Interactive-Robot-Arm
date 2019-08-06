import argparse
import time
from Robotarm import Robotarm

parser = argparse.ArgumentParser()
parser.add_argument("-p", "--port",help="Input serial port",type=str,default="COM22")
parser.add_argument('-r', "--rate",help="Input baudrate",type=int, default=9600)
args = parser.parse_args()

serialPort =  args.port  #"COM22"  # 串口
baudRate =  args.rate # 波特率
#range of angel for each servos
angel_range=[[60,35,20,35,55,10,10,10],  #min
[145,120,120,95,120,170,170,170]]    #max

angels=[60,35,20,35,55,90,90,90] #angels is the vector representing the angels of different servos [大拇指，食指，中指，无名指，小指，手腕旋转，第一节手臂，第二节手臂]
arm=Robotarm(serialPort, baudRate, angels)

def main():
    try:
        while 1:
            arm.control(angels)    #send the control instruction 
            time.sleep(0.5)
            #data = arm.readline()
            #print(data)
            #input your code
    except KeyboardInterrupt:
        arm.close()

if __name__ == "__main__":
    main()
