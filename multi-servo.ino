 bux#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
////以这种方式调用，它使用默认地址0x40。
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
//也可以用不同的地址调用它

/* Depending on your servo make, the pulse width min and max may vary, you  want these to be as small/large as possible without hitting the hard stop
 for max range. You'll have to tweak them as necessary to match the servos you
have!*/
/*根据你的伺服制作，脉冲宽度最小和最大可能变化，你想要这些尽可能小大而不碰到
硬停止，对于最大范围。你必须调整它们以匹配你的伺服系统！*/
#define SERVOMIN  103 // this is the 'minimum' pulse length count (out of 4096)
//这是“最小”脉冲长度计数（在4096）中
#define SERVOMAX  512 // this is the 'maximum' pulse length count (out of 4096)
//这是“最大”脉冲长度计数（在4096中）
#define NUM 20 
double angel[9][2]={{60,145},{10,135},{15,95},{15,70},{55,125},{0,180},{0,180},{0,180},{0,180}};
unsigned char buffer[9];
float now[9]={60,10,15,15,55,90,0,0,90};    //current angle
//float now[9]={145,135,95,70,125,90,90,90,90};    //current angle
unsigned int goal[9]={60,10,15,15,55,90,0,0,90};  // the goal of each angel
// int goal[9]={145,135,95,70,125,90,90,90,90};    //current angle

float next[9];   // next step
int steps=0;
// our servo # counter
//uint8_t servonum = 0;
bool if_return=0;

void setup() {
   Serial.begin(9600);
   //Serial.println("16 channel Servo test!");

   pwm.begin();
   
   pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
   ////模拟伺服在60赫兹更新下运行

   for (int32_t m=0;m<9;m++)
   {
     angel[m][0]=angel[m][0]/180.0*(SERVOMAX-SERVOMIN)+SERVOMIN;
     angel[m][1]=angel[m][1]/180.0*(SERVOMAX-SERVOMIN)+SERVOMIN;
   }
   for (int32_t m=0; m<9;m++)
    {
      pwm.setPWM(m, 0, now[m]/180.0*(SERVOMAX-SERVOMIN)+SERVOMIN);  //initialize the angel
    }
    Serial.println("begin");
   
}

void control(float *next)
{
     for (int m=0; m<9;m++)
    {
      //Serial.print('|');
      //Serial.print(next[m]);
      double myangel=next[m]/180.0*(SERVOMAX-SERVOMIN)+SERVOMIN;
      //Serial.print(m);
      //Serial.println(myangel);
      if(myangel>angel[m][1] || myangel<angel[m][0])
      { 
        //Serial.println("warning!!!");
        continue;
      }
      pwm.setPWM(m, 0, myangel);
    }
    //Serial.print('\n');
}

void loop() {
    if(Serial.available())
    {
      Serial.readBytes(buffer,11);
      
      for(int i=0;i<9;++i)
      { 
        //Serial.print(buffer[i]);
        goal[i] = buffer[i];
        //Serial.print(now[i]);
        //Serial.print(goal[i]);
        //Serial.print('|');
        buffer[i]='\0';
      }
      steps=buffer[9];
      if_return=buffer[10];
    }
    
    if(steps>0)
    {
      for(int i=0;i<9;++i)
      { 
        next[i] = now[i] + (goal[i] - now[i])/steps;
        now[i] = next[i];
      }
      steps--;
      if(steps==0 && if_return==1) Serial.println("finished!!");
      control(next);
    }
    
    delay(60);

}
