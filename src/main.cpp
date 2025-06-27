#include <Arduino.h>
#include <Esp32PcntEncoder.h>

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

Esp32PcntEncoder encoders[2]; // 创建一个数组用于存储两个编码器

int64_t last_ticks[2] = {0,0};//用于存储上一次读取的编码器数值
int16_t delta_ticks[2] = {0,0};//用于存储这一次读取的编码器数值
int64_t last_update_time = 0;//用于存储上一次更新电机速度的时间
float current_speed[2] = {0,0};//用于存储当前电机速度

void setMotorSpeed(int in1, int in2, int pwm, int speed);
void moveForward(int speed);
void moveBackward(int speed);
void turnLeft(int speed);
void turnRight(int speed);
void stopMotors();


void setup()
{
  // 1.初始化串口
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


  // 2.设置编码器
  encoders[0].init(0, 33, 32); // 初始化第一个编码器，A1=33，B1=32
  encoders[1].init(1, 2, 4); // 初始化第二个编码器，A2=2，B2=4

  moveForward(200);
}

void loop()
{
  delay(10); // 等待10毫秒
  //计算时间差
  int16_t dt = millis() - last_update_time;

  //计算编码器当前与上一次读取的数值之差
  delta_ticks[0] = encoders[0].getTicks() - last_ticks[0];
  delta_ticks[1] = encoders[1].getTicks() - last_ticks[1];
  //更新速度
  current_speed[0] = delta_ticks[0] * 0.166812 / dt;
  current_speed[1] = delta_ticks[1] * 0.166812 / dt;
  //更新last_tick
  last_ticks[0] = encoders[0].getTicks();
  last_ticks[1] = encoders[1].getTicks();
  last_update_time = millis();

  // 打印两个电机的速度
  Serial.printf("speed1=%f,speed2=%f\n",current_speed[0],current_speed[1]);
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

