#include <Arduino.h>
#include <Esp32PcntEncoder.h>
#include "PIDController.h"
#include "Kinematics.h"

// 电机控制引脚定义
#define AIN1 14
#define AIN2 27
#define PWMA 12
#define BIN1 26
#define BIN2 25
#define PWMB 13

// 第二个TB6612模块（控制电机C和D）
#define CIN1 18
#define CIN2 19
#define PWMC 5
#define DIN1 17
#define DIN2 16
#define PWMD 23
//速度上限
#define Speed_Limit 200

Esp32PcntEncoder encoders[2]; // 创建一个数组用于存储两个编码器
PIDController pid_controller[2];//创建一个数组用于PID控制
Kinematics kinematics;

float target_linear_speed = 50.0; //单位 mm/s
float target_angular_speed = 0.5; //单位 弧度/s
float out_left_speed = 0.0;       //输出的是左右轮速度，不是反馈的左右轮速度
float out_right_speed = 0.0;

void setMotorSpeed(int in1, int in2, int pwm, int speed);
void moveForward(int speed);
void moveBackward(int speed);
void turnLeft(int speed);
void turnRight(int speed);
void stopMotors();




void setup()
{
  // 初始化串口
  Serial.begin(115200); // 初始化串口通信，设置通信速率为115200

  // 初始化电机控制引脚
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(CIN1, OUTPUT);
  pinMode(CIN2, OUTPUT);
  pinMode(PWMC, OUTPUT);
  pinMode(DIN1, OUTPUT);
  pinMode(DIN2, OUTPUT);
  pinMode(PWMD, OUTPUT);


  // 初始化编码器
  encoders[0].init(0, 33, 32); // 初始化第一个编码器，A1=33，B1=32
  encoders[1].init(1, 2, 4); // 初始化第二个编码器，A2=2，B2=4

  //初始化PID控制器
  pid_controller[0].update_pid(0.625,0.125,0.0);
  pid_controller[1].update_pid(0.625,0.125,0.0);
  pid_controller[0].out_limit(-Speed_Limit,Speed_Limit);
  pid_controller[1].out_limit(-Speed_Limit,Speed_Limit);
  //初始化运动学参数
  kinematics.set_wheel_distance(175);
  kinematics.set_motor_param(0,0.166812);
  kinematics.set_motor_param(1,0.166812);
  //测试运动学逆解
  kinematics.kinematics_inverse(target_linear_speed,target_angular_speed,
                                &out_left_speed,&out_right_speed);
  Serial.printf("OUT:left_speed=%f,right_speed=%f\n",out_left_speed,out_right_speed);
  pid_controller[0].uptate_target(out_left_speed);
  pid_controller[1].uptate_target(out_right_speed);
                          
}

void loop()
{
  delay(10); // 等待10毫秒
  kinematics.update_motor_speed(millis(),encoders[0].getTicks(),encoders[1].getTicks());
  int temp0 = pid_controller[0].update(kinematics.get_motor_speed(0));
  int temp1 = pid_controller[1].update(kinematics.get_motor_speed(1));
  setMotorSpeed(AIN1, AIN2, PWMA, temp1);
  setMotorSpeed(BIN1, BIN2, PWMB, temp0);
  setMotorSpeed(CIN1, CIN2, PWMC, temp1);
  setMotorSpeed(DIN1, DIN2, PWMD, temp0);   
  // 打印两个电机的速度
  //Serial.printf("speed1=%f,speed2=%f\n",current_speed[0],current_speed[1]);
  Serial.printf("x,y,yaw=%f,%f,%f\n",kinematics.get_odom().x,kinematics.get_odom().y,
                                     kinematics.get_odom().angle);
}

void setMotorSpeed(int in1, int in2, int pwm, int speed) {
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  analogWrite(pwm, abs(speed));
}

void moveForward(int speed) {
  setMotorSpeed(AIN1, AIN2, PWMA, speed);
  setMotorSpeed(BIN1, BIN2, PWMB, speed);
  setMotorSpeed(CIN1, CIN2, PWMC, speed);
  setMotorSpeed(DIN1, DIN2, PWMD, speed);
}

void moveBackward(int speed) {
  setMotorSpeed(AIN1, AIN2, PWMA, -speed);
  setMotorSpeed(BIN1, BIN2, PWMB, -speed);
  setMotorSpeed(CIN1, CIN2, PWMC, -speed);
  setMotorSpeed(DIN1, DIN2, PWMD, -speed);
}

void turnLeft(int speed) {
  setMotorSpeed(AIN1, AIN2, PWMA, speed);
  setMotorSpeed(BIN1, BIN2, PWMB, -speed);
  setMotorSpeed(CIN1, CIN2, PWMC, speed);
  setMotorSpeed(DIN1, DIN2, PWMD, -speed);
}

void turnRight(int speed) {
  setMotorSpeed(AIN1, AIN2, PWMA, -speed);
  setMotorSpeed(BIN1, BIN2, PWMB, speed);
  setMotorSpeed(CIN1, CIN2, PWMC, -speed);
  setMotorSpeed(DIN1, DIN2, PWMD, speed);
}

void stopMotors() {
  setMotorSpeed(AIN1, AIN2, PWMA, 0);
  setMotorSpeed(BIN1, BIN2, PWMB, 0);
  setMotorSpeed(CIN1, CIN2, PWMC, 0);
  setMotorSpeed(DIN1, DIN2, PWMD, 0);
}

