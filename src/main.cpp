/*---------------------------------------------头文件引用区------------------------------------------------------*/
#include <Arduino.h>
#include "Esp32McpwmMotor.h"
#include <Esp32PcntEncoder.h>
#include "PIDController.h"
#include "Kinematics.h"
// micro-ROS 相关已拆分到单独文件
#include "Micro_ROS.h"
//oled显示屏相关库
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
//引入MPU6050相关库
#include <Adafruit_MPU6050.h> // 添加MPU6050库
#include <Adafruit_Sensor.h>
#include "ExponentialFilter.h"
#include <BasicLinearAlgebra.h> // 添加线性代数库

/*---------------------------------------------宏定义区------------------------------------------------------*/
#define SCREEN_WIDTH 128    // OLED宽度
#define SCREEN_HEIGHT 64    // OLED高度
#define PUBLISH_TIMEOUT 3000  // 心跳检测：3秒发布不成功认为断开
#define AGENT_CHECK_INTERVAL 1000  // 心跳检测：每1秒主动检查一次Agent连接

/*---------------------------------------------变量声明区------------------------------------------------------*/
Esp32McpwmMotor motor[2];
Esp32PcntEncoder encoders[2]; // 创建一个数组用于存储两个编码器
PIDController pid_controller[2];//创建一个数组用于PID控制
Kinematics kinematics;//运动学对象
//声明一些相关的结构体对象
int64_t last_ticks[2] = {0,0};//用于存储上一次读取的编码器数值
int16_t delta_ticks[2] = {0,0};//用于存储这一次读取的编码器数值
int64_t last_update_time = 0;//用于存储上一次更新电机速度的时间
float current_speed[2] = {0,0};//用于存储当前电机速度
float target_linear_speed = 20.0; //单位 mm/s
float target_angular_speed = 0.8; //单位 弧度/s
float out_left_speed = 0.0;       //输出的是左右轮速度，不是反馈的左右轮速度
float out_right_speed = 0.0;
//创建一个OLED显示屏对象
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
bool oledInitialized = false;
// MPU6050相关变量
Adafruit_MPU6050 mpu;
bool mpu_initialized = false;
float imu_yaw = 0.0; // IMU计算的偏航角
unsigned long last_imu_time = 0;
ExponentialFilter<float> yawFilter(0.2, 0); // 偏航角滤波器，参数可调
// MPU6050 数据变量 - 添加这些定义
float imu_angular_velocity_x = 0.0;
float imu_angular_velocity_y = 0.0;
float imu_angular_velocity_z = 0.0;
float imu_linear_acceleration_x = 0.0;
float imu_linear_acceleration_y = 0.0;
float imu_linear_acceleration_z = 0.0;
// 目标速度平滑滤波变量
float smoothed_left_target = 0;
float smoothed_right_target = 0;
unsigned long last_smooth_time = 0;

// 添加外部声明
extern bool microros_connected;
extern bool wifi_connected;
extern String ip_address;

/*------------------------------------------函数声明区---------------------------------------------------------*/
void motorSpeedControl();//函数用于控制电机速度（闭环控制）
// micro-ROS 的回调和任务在 Micro_ROS.* 中实现
//oled初始化函数
void InitOLED();
//oled显示函数
void updateDisplay();
// 更新IMU数据并计算偏航角
void updateIMU();
//IMU校准函数
void calibrateIMU();
// 检测是否应该完全停止函数
bool shouldStopCompletely();

/*------------------------------------------SetUP函数---------------------------------------------------------*/
void setup() {
  // 1.初始化串口
  Serial.begin(115200); // 初始化串口通信，设置通信速率为115200

  // 2.设置编码器
  encoders[0].init(0, 2, 15); // 初始化第一个编码器，使用GPIO2和15连接编码器A相和B相
  encoders[1].init(1, 34, 35); // 初始化第二个编码器，使用GPIO34和35连接编码器A相和B相

  //3.初始化电机的引脚设置(id,pwm,in1,in2)
  motor[0].attachMotor(0, 4, 17, 16);//左前A
  motor[0].attachMotor(1, 14, 27, 26);//右前D
  motor[1].attachMotor(0, 19, 5, 18);//左后B
  motor[1].attachMotor(1, 32, 33, 25);//右后C
  
  //4.设置电机速度。这里初始化为0
  motor[0].updateMotorSpeed(0, 0);
  motor[0].updateMotorSpeed(1, 0);
  motor[1].updateMotorSpeed(0, 0);
  motor[1].updateMotorSpeed(1, 0);

  //5.初始化PID控制器
  //0.6, 0.004, 0.06还行
  //0.625,0.125,0.0
  //0.15,0.001,0.01
  pid_controller[0].update_pid(0.15, 0.00015, 0.03);//pid参数
  pid_controller[1].update_pid(0.15, 0.00015, 0.03);
  pid_controller[0].out_limit(-800,800);//设置输出限制
  pid_controller[1].out_limit(-800,800);

  pid_controller[0].update_target(0);//设置目标值为0
  pid_controller[1].update_target(0);


  //6.初始化运动学参数
  //轮子转一圈编码器的脉冲数为1600
  //轮子直径为83mm，则每个脉冲的前进距离为80*3.14/1600=0.1570796mm
  kinematics.set_wheel_distance(175.0); // 设置两个轮子之间的距离为175mm
  kinematics.set_motor_param(0,0.1570796);
  kinematics.set_motor_param(1,0.1570796);
  //7.初始化里程计消息
  msg_odom.pose.pose.orientation.x = 0;
  msg_odom.pose.pose.orientation.y = 0;
  msg_odom.pose.pose.orientation.z = 0;
  msg_odom.pose.pose.orientation.w = 1;
  //8.创建一个任务来启动micro-ros的task
  //扩大任务栈到40KB以防止内存不足
  xTaskCreate(microros_task,"micros_task",40960,NULL,1,NULL);

  //9.初始化I2C总线
  Wire.begin(21, 22); // SDA=GPIO21, SCL=GPIO22
  Wire.setClock(100000); // 降低I2C速度以提高兼容性


  //10.初始化OLED显示屏
  InitOLED();

  
  //11.初始化MPU6050
  Serial.println("Initializing MPU6050...");
  if (!mpu.begin(0x68)) {
    Serial.println("Failed to find MPU6050 at 0x68, trying 0x69...");
    if (!mpu.begin(0x69)) {
      Serial.println("MPU6050 initialization failed!");
    } else {
      mpu_initialized = true;
      Serial.println("MPU6050 found at 0x69!");
    }
  } else {
    mpu_initialized = true;
    Serial.println("MPU6050 found at 0x68!");
  }
  
  if (mpu_initialized) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("MPU6050 configured successfully");
    calibrateIMU();//初始校准
  }

 
}


/*------------------------------------------LOOP函数---------------------------------------------------------*/
void loop() {
  // 更新IMU数据
  updateIMU();

  Serial.printf("tick_0=%d,tick_1=%d\n",encoders[0].getTicks(),encoders[1].getTicks());
  kinematics.update_motor_speed(millis(),encoders[0].getTicks(),encoders[1].getTicks());
  Serial.printf("left_tick=%d,right_tick=%d\n",encoders[0].getTicks(),encoders[1].getTicks());

  // 获取当前速度
  float left_speed = kinematics.get_motor_speed(0);
  float right_speed = kinematics.get_motor_speed(1);
  // 计算PID输出 - 每个电机使用独立的PID
  int left_pwm = pid_controller[0].update(left_speed);
  int right_pwm = pid_controller[1].update(right_speed);

  motor[0].updateMotorSpeed(0, left_pwm);//左前A
  motor[0].updateMotorSpeed(1, right_pwm);//右前D
  motor[1].updateMotorSpeed(0, left_pwm);//左后B
  motor[1].updateMotorSpeed(1, right_pwm);//右后C

 
  // 打印两个电机的速度
  //Serial.printf("speed1=%f,speed2=%f\n",current_speed[0],current_speed[1]);
  Serial.printf("x,y,yaw=%f,%f,%f\n",kinematics.get_odom().x,kinematics.get_odom().y,
                                     kinematics.get_odom().angle);

  // 每500毫秒更新一次OLED显示
  static unsigned long lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate > 500) {
    updateDisplay();
    lastDisplayUpdate = millis();
  }

  //delay(10); // 添加小延迟，避免循环过快

}




/*-----------------------------------------函数定义区----------------------------------------------------------*/
//电机控制函数
void motorSpeedControl(){//函数用于控制电机速度（闭环控制）
  //计算时间差
  int16_t dt = millis() - last_update_time;

  //计算编码器当前与上一次读取的数值之差
  delta_ticks[0] = encoders[0].getTicks() - last_ticks[0];
  delta_ticks[1] = encoders[1].getTicks() - last_ticks[1];
  //更新速度
  //要乘上每个编码器脉冲对应的前进距离0.1570796mm
  current_speed[0] = delta_ticks[0] * 0.1570796 / dt * 1000;
  current_speed[1] = delta_ticks[1] * 0.1570796 / dt * 1000;
  //更新last_tick
  last_ticks[0] = encoders[0].getTicks();
  last_ticks[1] = encoders[1].getTicks();
  last_update_time = millis();

  //调用PID获取动态的输出值
  int temp0 = pid_controller[0].update(current_speed[0]);
  int temp1 = pid_controller[1].update(current_speed[1]);
  //Serial.printf("temp0=%d,temp1=%d\n",temp0,temp1);
  motor[0].updateMotorSpeed(0,temp0);
  motor[0].updateMotorSpeed(1,temp1);
  motor[1].updateMotorSpeed(0,temp0);
  motor[1].updateMotorSpeed(1,temp1);

}






//初始化oled
void InitOLED() {
  Wire.begin(21, 22);
  Wire.setClock(400000); // 提高I2C速度
  
   // 尝试常见地址
  const uint8_t addresses[] = {0x3C, 0x3D};
  for (int i = 0; i < sizeof(addresses)/sizeof(addresses[0]); i++) {
    Wire.beginTransmission(addresses[i]);
    if (Wire.endTransmission() == 0) {
      Serial.printf("检测到OLED在地址0x%X\n", addresses[i]);
      if (display.begin(SSD1306_SWITCHCAPVCC, addresses[i])) {
        oledInitialized = true;
        Serial.println("OLED初始化成功");
        
        // 显示启动画面
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0,0);
        display.println("Robot Controller");
        display.setCursor(0,10);
        display.println("Starting...");
        display.setCursor(0,20);
        display.println("Connect WiFi...");
        display.display();
        delay(1000);
        
        return;
      }
    }
  }
  Serial.println("OLED初始化失败!");
}


//oled显示函数
void updateDisplay() {
  if (!oledInitialized) return;
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  // 第1行：显示WiFi状态
  display.setCursor(0, 0);
  display.print("WiFi: ");
  display.print(WiFi.SSID());
  
  // 第2行：显示IP地址
  display.setCursor(0, 10);
  display.print("IP: ");
  display.print(WiFi.localIP().toString());
  
  // 第3行: micro-ROS Agent连接状态
  display.setCursor(0, 20);
  display.print("Agent:");
  if (microros_connected) {
    display.print("CONN");
  } else {
    display.print("DISCONN");
  }
  
  // 第4行: 左电机速度
  float left_speed = kinematics.get_motor_speed(0);
  display.setCursor(0, 30);
  display.print("L:");
  display.print(left_speed, 0);
  display.print("mm/s");
  
  // 第5行: 右电机速度
  float right_speed = kinematics.get_motor_speed(1);
  display.setCursor(0, 40);
  display.print("R:");
  display.print(right_speed, 0);
  display.print("mm/s");
  
  // 第6行: 运行时间
  display.setCursor(0, 50);
  display.print("Up:");
  display.print(millis() / 1000);
  display.print("s");
  
  display.display();
}

// 更新IMU数据并计算偏航角
void updateIMU() {
  if (!mpu_initialized) return;
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // 保存原始数据供Micro-ROS使用
  imu_angular_velocity_x = g.gyro.x;
  imu_angular_velocity_y = g.gyro.y;
  imu_angular_velocity_z = g.gyro.z;
  
  imu_linear_acceleration_x = a.acceleration.x;
  imu_linear_acceleration_y = a.acceleration.y;
  imu_linear_acceleration_z = a.acceleration.z;
  
  // 计算时间差（秒）
  unsigned long current_time = millis();
  float dt = (current_time - last_imu_time) / 1000.0f;
  last_imu_time = current_time;
  
  // 确保dt在合理范围内
  if (dt <= 0 || dt > 0.1) {
    dt = 0.01; // 默认10ms
  }
  
  // 计算偏航角变化（陀螺仪Z轴积分）
  float yaw_rate = g.gyro.z; // 弧度/秒
  imu_yaw += yaw_rate * dt;
  
  // 使用滤波器平滑数据
  yawFilter.Filter(imu_yaw);
  imu_yaw = yawFilter.Current();
  
  // 角度归一化到[-π, π]
  while (imu_yaw > PI) imu_yaw -= 2 * PI;
  while (imu_yaw < -PI) imu_yaw += 2 * PI;
  
  // 调试输出
  static unsigned long last_debug = 0;
  if (millis() - last_debug > 500) {
    Serial.printf("IMU Yaw: %.2f rad (%.1f°)\n", imu_yaw, imu_yaw * 180 / PI);
    Serial.printf("Gyro: X=%.2f, Y=%.2f, Z=%.2f rad/s\n", 
                  imu_angular_velocity_x, imu_angular_velocity_y, imu_angular_velocity_z);
    Serial.printf("Accel: X=%.2f, Y=%.2f, Z=%.2f m/s²\n", 
                  imu_linear_acceleration_x, imu_linear_acceleration_y, imu_linear_acceleration_z);
    last_debug = millis();
  }
}

//IMU校准函数
void calibrateIMU() {
  if (!mpu_initialized) return;
  
  Serial.println("Calibrating MPU6050...");
  
  // 收集500个样本计算平均值
  const int samples = 500;
  float gx_sum = 0.0, gy_sum = 0.0, gz_sum = 0.0;
  float ax_sum = 0.0, ay_sum = 0.0, az_sum = 0.0;
  
  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    gx_sum += g.gyro.x;
    gy_sum += g.gyro.y;
    gz_sum += g.gyro.z;
    
    ax_sum += a.acceleration.x;
    ay_sum += a.acceleration.y;
    az_sum += a.acceleration.z;
    
    delay(5);
  }
  
  // 计算零偏
  float gyro_bias_x = gx_sum / samples;
  float gyro_bias_y = gy_sum / samples;
  float gyro_bias_z = gz_sum / samples;
  
  float accel_bias_x = ax_sum / samples;
  float accel_bias_y = ay_sum / samples;
  float accel_bias_z = az_sum / samples - 9.81; // 减去重力加速度
  
  Serial.println("Calibration results:");
  Serial.printf("Gyro bias: X=%.6f, Y=%.6f, Z=%.6f rad/s\n", gyro_bias_x, gyro_bias_y, gyro_bias_z);
  Serial.printf("Accel bias: X=%.6f, Y=%.6f, Z=%.6f m/s^2\n", accel_bias_x, accel_bias_y, accel_bias_z);
  
  // 保存校准值（在实际应用中应该保存到EEPROM）
}

// 检测是否应该完全停止
bool shouldStopCompletely() {
    return (fabs(target_linear_speed) < 0.01 && fabs(target_angular_speed) < 0.01);
}
