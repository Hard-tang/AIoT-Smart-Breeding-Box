#include<Arduino.h>
#include <AccelStepper.h> 
#include "gimbal_control.h"

/* GPIO配置 */
const int enablePin = 11;  // 使能控制引脚

const int xdirPin = 2;     // x方向控制引脚
const int xstepPin = 3;    // x步进控制引脚
const int ydirPin = 4;     // y方向控制引脚
const int ystepPin = 5;    // y步进控制引脚
const int zdirPin = 6;     // z方向控制引脚
const int zstepPin = 7;    // z步进控制引脚
const int x_stopPin = 8;   // x方向光电门限位引脚
const int y_stopPin = 9;   // y方向光电门限位引脚
const int z_stopPin = 10;  // z方向光电门限位引脚
const int x_max = 13950;   // x方向最大步进数
const int y_max = 15250;   // y方向最大步进数
const int z_max = 11000;   // z方向最大步进数

AccelStepper stepperX(1,xstepPin,xdirPin);//建立步进电机对象X
AccelStepper stepperY(1,ystepPin,ydirPin);//建立步进电机对象Y
AccelStepper stepperZ(1,zstepPin,zdirPin);//建立步进电机对象Z

/* 定义全局变量 */
// 信号停止状态标志
int x_stop = 1;
int y_stop = 1;
int z_stop = 1;

// 步进电机步数
int x_steps = 0;
int y_steps = 0;
int z_steps = 0;

// 运动速度
float x_vel = 100000;
float y_vel = 100000;
float z_vel = 100000;

// 运动加速度
float x_acc = 20000;
float y_acc = 20000;
float z_acc = 20000;



void setup() {
  driver_init();

}

void loop() {
  Serial_app();
  stepper_run();
}


/* 上位机通讯程序 */
void Serial_app(){//上位机交互程序
  String inString ="" ; //串口接收命令存放区
  while(Serial.available()>0){ //检查缓冲区是否存在数据
    inString += char(Serial.read()); //读取缓冲区
    delay(10);      // 延时函数用于等待字符完全进入缓冲区
  }

  if(inString!=""){ //  命令解析区
    String command_head= "" ; //  命令头缓存
    String command_text= "" ; //  命令内容缓存
    String command_mid= "" ;
    command_head = inString.substring(0,7);// 命令头格式"/000000"
    command_mid = inString.substring(7,8);
    command_text = inString.substring(8,inString.length());

    
    /*解析命令头*/

    /* 对全体电机写入速度 */
    if(command_head == "/000000"&&command_mid == " ")
    {
      x_vel = command_text.toInt();
      y_vel = command_text.toInt();
      z_vel = command_text.toInt();
      stepperX.setMaxSpeed(x_vel);
      stepperY.setMaxSpeed(y_vel);
      stepperZ.setMaxSpeed(z_vel);
      Serial.print("全体速度设置为：");
      Serial.println(x_vel);
    }


    /* 对X电机写入速度 */
    if(command_head == "/000001"&&command_mid == " ") 
    {
      x_vel = command_text.toInt();
      stepperX.setMaxSpeed(x_vel);
      Serial.print("X电机速度设置为：");
      Serial.println(x_vel);
    }

    /* 对Y电机写入速度 */
    if(command_head == "/000002"&&command_mid == " ") 
    {
      y_vel = command_text.toInt();
      stepperY.setMaxSpeed(y_vel);
      Serial.print("Y电机速度设置为：");
      Serial.println(y_vel);
    }

    /* 对Z电机写入速度 */
    if(command_head == "/000003"&&command_mid == " ") 
    {
      z_vel = command_text.toInt();
      stepperZ.setMaxSpeed(z_vel);
      Serial.print("Z电机速度设置为：");
      Serial.println(z_vel);
    }


    /* 对全体电机写入加速度 */
    if(command_head == "/100000"&&command_mid == " ") 
    {
      x_acc = command_text.toInt();
      y_acc = command_text.toInt();
      z_acc = command_text.toInt();
      stepperX.setAcceleration(x_acc);
      stepperY.setAcceleration(y_acc);
      stepperZ.setAcceleration(z_acc);
      Serial.print("全体电机加速度设置为：");
      Serial.println(x_acc);
    }

    /* 对X电机写入加速度 */
    if(command_head == "/100001"&&command_mid == " ") 
    {
      x_acc = command_text.toInt();
      stepperX.setAcceleration(x_acc);
      Serial.print("X电机加速度设置为：");
      Serial.println(x_acc);
    }

    /* 对Y电机写入加速度 */
    if(command_head == "/100002"&&command_mid == " ") 
    {
      y_acc = command_text.toInt();
      stepperY.setAcceleration(y_acc);
      Serial.print("Y电机加速度设置为：");
      Serial.println(y_acc);
    }

    /* 对Z电机写入加速度 */
    if(command_head == "/100003"&&command_mid == " ") 
    {
      z_acc = command_text.toInt();
      stepperZ.setAcceleration(z_acc);
      Serial.print("Z电机加速度设置为：");
      Serial.println(z_acc);
    }


    /* 对全体电机写入绝对位置 */
    if(command_head == "/200000"&&command_mid == " ") 
    {
      x_steps = command_text.toInt();
      y_steps = command_text.toInt();
      z_steps = command_text.toInt();
      stepperX.moveTo(x_steps);
      stepperY.moveTo(y_steps);
      stepperZ.moveTo(z_steps);
      Serial.print("全体电机位置设置为：");
      Serial.println(x_steps);
    }

    /* 对X电机写入绝对位置 */
    if(command_head == "/200001"&&command_mid == " ") 
    {
      x_steps = command_text.toInt();
      stepperX.moveTo(x_steps);
      Serial.print("X电机位置设置为：");
      Serial.println(x_steps);
    }

    /* 对Y电机写入绝对位置 */
    if(command_head == "/200002"&&command_mid == " ") 
    {
      y_steps = command_text.toInt();
      stepperY.moveTo(y_steps);
      Serial.print("Y电机位置设置为：");
      Serial.println(y_steps);
    }

    /* 对Z电机写入绝对位置 */
    if(command_head == "/200003"&&command_mid == " ") 
    {
      z_steps = command_text.toInt();
      stepperZ.moveTo(z_steps);
      Serial.print("Z电机位置设置为：");
      Serial.println(z_steps);
    }


    /* 获取全体电机绝对位置 */
    if(command_head == "/300000") 
    {
      Serial.print("X电机位置：");
      Serial.println(stepperX.currentPosition());
      Serial.print("Y电机位置：");
      Serial.println(stepperY.currentPosition());
      Serial.print("Z电机位置：");
      Serial.println(stepperZ.currentPosition());
    }

    /* 重设X电机绝对位置 */
    if(command_head == "/300001") 
    {
      stepperX.setCurrentPosition(0);
      Serial.print("X电机坐标系已经重建");
      Serial.println(x_steps);
    }

    /* 重设Y电机绝对位置 */
    if(command_head == "/300002") 
    {
      stepperY.setCurrentPosition(0);
      Serial.print("X电机坐标系已经重建");
      Serial.println(0);
    }

    /* 重设Z电机绝对位置 */
    if(command_head == "/300003") 
    {
      stepperZ.setCurrentPosition(0);
      Serial.print("X电机坐标系已经重建");
      Serial.println(z_steps);
    }



    /* 启动自校准程序 */
    if(command_head == "/400000") 
    {
      stepper_calibration();
    }
  }
}


