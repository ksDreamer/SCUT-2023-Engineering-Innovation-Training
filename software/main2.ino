 /*********************************PPM*********************************/
//PPM引脚用？
#include "ppm.h"
#define THROTTLE        3
#define ROLL            1
#define PITCH           2
#define YAW             4
#define SA              5
#define SB              6
#define SC              7    
#define SD              8     
#define MiddlePoint     1508
#define PPM             13
/*******************************servo********************/
#include <Servo.h>
Servo LF;
Servo LB;
Servo RF;
Servo RB;
Servo back1;//上
Servo back2;//下
#define S0             10
#define S1             11
#define S2             8
#define S3             9
#define S4             16
#define S5             17


/****************************interval*****************/
// Loop interval time
const long interval = 50;
unsigned long previousMillis = 0;

int counter=0;
float anglex=90;
float angley=90;
int addressx[4]={0,0,0,0};
int addressy[4]={0,0,0,0};

//Varible Initialization
short throttle,roll,pitch,yaw,sb,sa,sc,sd;
/*********************************TB6612*********************************/
#define AIN1  22
#define AIN2  23
#define BIN1  24
#define BIN2  25
#define CIN1  26
#define CIN2  27
#define DIN1  28
#define DIN2  29
#define EIN1  30
#define EIN2  31
#define FIN1  32
#define FIN2  33
#define PWMA  2
#define PWMB  3
#define PWMC  4
#define PWMD  5
#define PWME  6
#define PWMF  7
//#define PWM0  46  给舵机拿了
//#define PWM1  45
#define PWM2  44
/*********************************PID*********************************/
//速度
int lastError=0;
int allError=0;
int delta=0;
//角度
float Velocity,Position=0,Motor;            //速度和位置测量值
float Target_Position=0,Target_Velocity=0;  //目标速度和目标位置
float Position_KP=120,Position_KI=15,Position_KD=800,Velocity_KP=20,Velocity_KI=30;
int last_time=0,current_time,last_position;
const int time_gap=20;
//角度(视觉)
float pos_kp=100,pos_ki=15,pos_kd=500;
float serx=0,sery=0;
/*********************************人工设置*********************************/
const int DeadZone=40;//ppm+fric
const int Domain=470;
const int maxLinearSpeed=220;
/**********************************************我是大分割线**********************************************/
void setup() {
  //PPM
  Serial.begin(38400);
  ppm.begin(PPM, false);

  //servo
  LF.attach(S0);
  RF.attach(S1);
  LB.attach(S2);
  RB.attach(S3);
  back1.attach(S4);
  back2.attach(S5);
  
  
  LF.write(90);
  RF.write(90);
  LB.write(90);
  RB.write(90);
  back1.write(90);
  back2.write(90);
  
  
  
  //TB6612
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(CIN1, OUTPUT);
  pinMode(CIN2, OUTPUT);
  pinMode(DIN1, OUTPUT);
  pinMode(DIN2, OUTPUT);
  pinMode(EIN1, OUTPUT);
  pinMode(EIN2, OUTPUT);
  pinMode(FIN1, OUTPUT);
  pinMode(FIN2, OUTPUT);
  
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(PWMC, OUTPUT);
  pinMode(PWMD, OUTPUT);
  pinMode(PWME, OUTPUT);
  pinMode(PWMF, OUTPUT);
  //pinMode(PWM0, OUTPUT);
  //pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  
  digitalWrite(AIN1, 1);
  digitalWrite(AIN2, 0);
  digitalWrite(BIN1, 1);
  digitalWrite(BIN2, 0);
  digitalWrite(CIN1, 1);
  digitalWrite(CIN2, 0);
  digitalWrite(DIN1, 1);
  digitalWrite(DIN2, 0);
  digitalWrite(EIN1, 1);
  digitalWrite(EIN2, 0);
  digitalWrite(FIN1, 1);
  digitalWrite(FIN2, 0);
  
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  analogWrite(PWMC, 0);
  analogWrite(PWMD, 0);
  analogWrite(PWME, 0);
  analogWrite(PWMF, 0);
  
 }
/**********************************************我是大分割线**********************************************/
void loop() 
{
  
  ReadPPM();
  //PrintPPM();
  mecanum(roll,pitch,yaw);
  //mecanum(255,255,255);
  //SetMotor(255,255,255,255);
  ServoControl();
  
  
}
/***********************************************servo*************************************************/
void ServoControl()
{
  //前
  if(sc==1)
  {
    LF.write(45);
    RF.write(135);
  }
  else 
  {
    LF.write(180);
    RF.write(0);
  }
  //锁止
  if(sd==1)
  {
    back2.write(0);
  }
  else if(sd==-1 && sb==-1)
  {
    back2.write(135);
  }
  //后
  if(sb==1 && sd==1)
  {
    back1.write(85);
  }
  else if(sb==-1)
  {
    back1.write(0);
  }
  //框
  if(sa==1)
  {
    LB.write(173);
    RB.write(2);
  }
  else 
  {
    LB.write(55);
    RB.write(120);
  }
}
/**********************************************我是大分割线**********************************************/
void mecanum(float xSpeed, float ySpeed, float aSpeed)
{
    float speed1 = (ySpeed + xSpeed + aSpeed); 
    float speed2 = (ySpeed - xSpeed - aSpeed);
    float speed3 = (ySpeed + xSpeed - aSpeed);
    float speed4 = (ySpeed - xSpeed + aSpeed);
    
    float max = abs(speed1);
    if (max < abs(speed2))  max = abs(speed2);
    if (max < abs(speed3))  max = abs(speed3);
    if (max < abs(speed4))  max = abs(speed4);
    
    if (max > maxLinearSpeed)
    {
        speed1 = speed1 / max * maxLinearSpeed;
        speed2 = speed2 / max * maxLinearSpeed;
        speed3 = speed3 / max * maxLinearSpeed;
        speed4 = speed4 / max * maxLinearSpeed;
    }
    
    SetMotor(speed2, speed3, speed4, speed1);

    //display
    /*Serial.print(speed1); Serial.print("  ");
    Serial.print(speed2); Serial.print("  ");
    Serial.print(speed3); Serial.print("  ");
    Serial.print(speed4); Serial.print("  ");
    Serial.println();*/
}
/*********************************我是分割线*********************************/
void SetMotor(int speed1,int speed2,int speed3,int speed4)
{
  SetPWM(1,speed1);
  SetPWM(2,-speed2);
  SetPWM(3,speed3);
  SetPWM(4,-speed4);
}
/*********************************我是分割线*********************************/
void SetPWM(int motor, int pwm)
{
  if(pwm>255)
    pwm=255;
  else if(pwm<-255)
    pwm=-255;
    
  if(motor==1&&pwm>=0)
  {
    digitalWrite(AIN1, 1);
    digitalWrite(AIN2, 0);
    analogWrite(PWMA, pwm);
  }
  else if(motor==1&&pwm<0)
  {
    digitalWrite(AIN1, 0);
    digitalWrite(AIN2, 1);
    analogWrite(PWMA, -pwm);
  }
  else if(motor==2&&pwm>=0)
  {
    digitalWrite(BIN1, 0);
    digitalWrite(BIN2, 1);
    analogWrite(PWMB, pwm);
  }
  else if(motor==2&&pwm<0)
  {
    digitalWrite(BIN1, 1);
    digitalWrite(BIN2, 0);
    analogWrite(PWMB, -pwm);
  }
  else if(motor==3&&pwm>=0)
  {
    digitalWrite(CIN1, 1);
    digitalWrite(CIN2, 0);
    analogWrite(PWMC, pwm);
  }
  else if(motor==3&&pwm<0)
  {
    digitalWrite(CIN1, 0);
    digitalWrite(CIN2, 1);
    analogWrite(PWMC, -pwm);
  }
  else if(motor==4&&pwm>=0)
  {
    digitalWrite(DIN1, 0);
    digitalWrite(DIN2, 1);
    analogWrite(PWMD, pwm);
  }
  else if(motor==4&&pwm<0)
  {
    digitalWrite(DIN1, 1);
    digitalWrite(DIN2, 0);
    analogWrite(PWMD, -pwm);
  }
  else if(motor==5&&pwm>=0)
  {
    digitalWrite(EIN1, 0);
    digitalWrite(EIN2, 1);
    analogWrite(PWME, pwm);
  }
  else if(motor==5&&pwm<0)
  {
    digitalWrite(EIN1, 1);
    digitalWrite(EIN2, 0);
    analogWrite(PWME, -pwm);
  }
}

/**********************************************我是大分割线**********************************************/
void servocontrol(int y,int x)
{   float a,b;//比例
    a=y*-0.001;
    b=x*-0.001;
    angley+=a;
    if(angley>120) angley=120;//俯角
    else if(angley<20) angley=20;//仰角
    anglex+=b;
    if(anglex>180) anglex=180;
    else if(anglex<0) anglex=0;
    //L1.write(angley);
    //L2.write(anglex);
    /*Serial.print("anglex:");
    Serial.print(anglex);
    Serial.print("\n");*/
    delay(10);
  }
/*********************************我是分割线*********************************/
void ReadPPM()
{
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Acquiring all the channels values
    throttle      =   (ppm.read_channel(THROTTLE)-MiddlePoint)*0.545;
    roll          =   (ppm.read_channel(ROLL)-MiddlePoint)*0.545;
    pitch         =   (ppm.read_channel(PITCH)-MiddlePoint)*0.545;
    yaw           =   (ppm.read_channel(YAW)-MiddlePoint)*0.545;
    sa            =   ppm.read_channel(SA)-MiddlePoint;
    sb            =   ppm.read_channel(SB)-MiddlePoint;
    sc            =   ppm.read_channel(SC)-MiddlePoint;
    sd            =   ppm.read_channel(SD)-MiddlePoint;
    
    //Dead Zone
    if(abs(throttle)<DeadZone) throttle=0;
    if(abs(roll)<DeadZone)     roll=0;
    if(abs(pitch)<DeadZone)    pitch=0;
    if(abs(yaw)<DeadZone)      yaw=0;

    //Switch Transform   范围是300还是100
    if(sa<-300)                 sa=-1;
    else if(sa>-100 && sa<100)  sa=0;
    else if(sa>300)             sa=1;
    
    if(sb<-300)                 sb=-1;
    else if(sb>-100 && sb<100)  sb=0;
    else if(sb>300)             sb=1;

    if(sc<-300)                 sc=-1;
    else if(sc>-100 && sc<100)  sc=0;
    else if(sc>300)             sc=1;

    if(sd<-300)                 sd=-1;
    else if(sd>-100 && sd<100)  sd=0;
    else if(sd>300)             sd=1;
  }
}
/*********************************我是分割线*********************************/
void PrintPPM()
{
  Serial.print("Throttle:");        Serial.print(throttle);       Serial.print(" ");
  Serial.print("Roll:");            Serial.print(roll);           Serial.print(" ");
  Serial.print("Pitch:");           Serial.print(pitch);          Serial.print(" ");
  Serial.print("Yaw:");             Serial.print(yaw);            Serial.print(" ");
  Serial.print("SA:");              Serial.print(sa);             Serial.print(" ");
  Serial.print("SB:");              Serial.print(sb);             Serial.print(" ");
  Serial.print("SC:");              Serial.print(sc);             Serial.print(" ");
  Serial.print("SD:");              Serial.print(sd);             Serial.print(" ");
  Serial.println(); 
}
/*********************************我是分割线*********************************/
