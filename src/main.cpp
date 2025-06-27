#include "Wire.h"
#include <MPU6050_light.h>

// 电机控制引脚定义A
// 第一个TB6612模块（控制电机A和B）
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

// 共用待机引脚
#define STBY1 33
#define STBY2 32


float leftSpeed = 0.0;//左速度
float rightSpeed = 0.0;//右速度
float Speed = 200;

MPU6050 mpu(Wire);

//I2C扫描函数
void scanI2C() {
  Serial.println("Scanning I2C devices...");
  byte error, address;
  int devices = 0;
  
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("Device found at 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println();
      devices++;
    }
  }
  
  if (devices == 0) {
    Serial.println("No I2C devices found!");
  }
  Serial.println();
}

//设置电机速度
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

//电机前进函数
void moveForward(int speed) {
  setMotorSpeed(AIN1, AIN2, PWMA, speed);
  setMotorSpeed(BIN1, BIN2, PWMB, speed);
  setMotorSpeed(CIN1, CIN2, PWMC, speed);
  setMotorSpeed(DIN1, DIN2, PWMD, speed);
}

//电机后退函数
void moveBackward(int speed) {
  setMotorSpeed(AIN1, AIN2, PWMA, -speed);
  setMotorSpeed(BIN1, BIN2, PWMB, -speed);
  setMotorSpeed(CIN1, CIN2, PWMC, -speed);
  setMotorSpeed(DIN1, DIN2, PWMD, -speed);
}

//电机左转函数
void turnLeft(int speed) {
  setMotorSpeed(AIN1, AIN2, PWMA, speed);
  setMotorSpeed(BIN1, BIN2, PWMB, -speed);
  setMotorSpeed(CIN1, CIN2, PWMC, speed);
  setMotorSpeed(DIN1, DIN2, PWMD, -speed);
}

//电机右转函数
void turnRight(int speed) {
  setMotorSpeed(AIN1, AIN2, PWMA, -speed);
  setMotorSpeed(BIN1, BIN2, PWMB, speed);
  setMotorSpeed(CIN1, CIN2, PWMC, -speed);
  setMotorSpeed(DIN1, DIN2, PWMD, speed);
}

//电机停止函数
void stopMotors() {
  setMotorSpeed(AIN1, AIN2, PWMA, 0);
  setMotorSpeed(BIN1, BIN2, PWMB, 0);
  setMotorSpeed(CIN1, CIN2, PWMC, 0);
  setMotorSpeed(DIN1, DIN2, PWMD, 0);
}



void setup() {
  Serial.begin(115200);

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


  Wire.begin(21,22);//SDA,SCL
  delay(100); // 添加延时确保I2C总线稳定
  scanI2C(); // 添加扫描
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");
  
}

unsigned long timer = 0;

void loop() {

  //MPU6050部分
  mpu.update();

  if(millis() - timer > 1000){ // print data every second
    Serial.print(F("TEMPERATURE: "));Serial.println(mpu.getTemp());
    Serial.print(F("ACCELERO  X: "));Serial.print(mpu.getAccX());
    Serial.print("\tY: ");Serial.print(mpu.getAccY());
    Serial.print("\tZ: ");Serial.println(mpu.getAccZ());
  
    Serial.print(F("GYRO      X: "));Serial.print(mpu.getGyroX());
    Serial.print("\tY: ");Serial.print(mpu.getGyroY());
    Serial.print("\tZ: ");Serial.println(mpu.getGyroZ());
  
    Serial.print(F("ACC ANGLE X: "));Serial.print(mpu.getAccAngleX());
    Serial.print("\tY: ");Serial.println(mpu.getAccAngleY());
    
    Serial.print(F("ANGLE     X: "));Serial.print(mpu.getAngleX());
    Serial.print("\tY: ");Serial.print(mpu.getAngleY());
    Serial.print("\tZ: ");Serial.println(mpu.getAngleZ());
    Serial.println(F("=====================================================\n"));
    timer = millis();
  }



  moveForward(Speed*0.7);
  delay(1000);
  turnLeft(Speed*0.7);
  delay(1000);
  moveBackward(Speed*0.7);
  delay(1000);
  turnRight(Speed*0.7);
  delay(1000);



}




