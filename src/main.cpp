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
// MPU6050 互补滤波相关变量
float complementary_angle = 0.0;
float accel_angle = 0.0;
float gyro_bias_z = 0.0;
bool gyro_calibrated = false;
unsigned long last_comp_filter_time = 0;
float complementary_alpha = 0.98; // 互补滤波参数，可调整


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
// 电机校准函数
void calibrate_motors();
// 函数声明
void updateComplementaryFilter();
void calibrateGyroBias();
float calculateAccelAngle(float accel_x, float accel_y, float accel_z);

/*------------------------------------------SetUP函数---------------------------------------------------------*/
void setup() {

  // 1.初始化串口
  Serial.begin(115200); // 初始化串口通信，设置通信速率为115200

  // 2.设置编码器v1.0
  encoders[0].init(0, 15, 2); // 初始化第一个编码器，使用GPIO2和15连接编码器A相和B相
  encoders[1].init(1, 35, 34); // 初始化第二个编码器，使用GPIO34和35连接编码器A相和B相(35->12,34->13)

  //3.初始化电机的引脚设置(id,pwm,in1,in2)v1.0
  motor[0].attachMotor(0, 4, 16, 17);//左前A
  motor[0].attachMotor(1, 14, 26, 27);//右前D
  motor[1].attachMotor(0, 19, 18, 5);//左后B->4,17,16
  motor[1].attachMotor(1, 32, 25, 33);//右后C->14,27,26

  //4.设置电机速度。这里初始化为0
  motor[0].updateMotorSpeed(0, 0);
  motor[0].updateMotorSpeed(1, 0);
  motor[1].updateMotorSpeed(0, 0);
  motor[1].updateMotorSpeed(1, 0);

  //5.初始化I2C总线
  Wire.begin(21, 22); // SDA=GPIO21, SCL=GPIO22
  Wire.setClock(100000); // 降低I2C速度以提高兼容性

  //6.初始化OLED显示屏
  InitOLED();

  //7.电机校准
  encoders[0].clearCount();  // 使用clearCount而不是reset
  encoders[1].clearCount();
  calibrate_motors();

  //8.初始化MPU6050
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
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);  // 改为4G范围
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);       // 500度/秒
    mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);    // 降低带宽减少噪声
    
    Serial.println("MPU6050 configured successfully");
    
    // 先校准IMU，再校准陀螺仪零偏
    calibrateIMU();
    calibrateGyroBias();
    
    // 初始化互补滤波角度
    complementary_angle = 0.0;
    imu_yaw = 0.0;
    last_comp_filter_time = millis();
  }


  // 9. 重置编码器计数 - 校准后清零
  encoders[0].reset();  
  encoders[1].reset();

  //10. 初始化运动学参数
  //轮子转一圈编码器的脉冲数为1300
  //轮子直径为65mm，则每个脉冲的前进距离为65*3.14/100=0.1570796mm
  kinematics.set_wheel_distance(175.0); // 设置两个轮子之间的距离为175mm
  kinematics.set_motor_param(0,0.1570796);
  kinematics.set_motor_param(1,0.1570796);

  //11. 重置里程计
  kinematics.reset_odom();

  //12. 初始化PID控制器
  //0.6, 0.004, 0.06还行
  //0.625,0.125,0.0
  //0.15,0.001,0.01
  pid_controller[0].update_pid(0.245, 0.000048, 0.058);//pid参数
  pid_controller[1].update_pid(0.25, 0.00005, 0.06);
  pid_controller[0].out_limit(-800,800);//设置输出限制
  pid_controller[1].out_limit(-800,800);

  pid_controller[0].set_integral_limits(40.0f, 800.0f);// 设置积分限制和摩擦补偿
  pid_controller[1].set_integral_limits(40.0f, 800.0f);
  
  pid_controller[0].set_friction_compensation(35.0f);
  pid_controller[1].set_friction_compensation(35.0f);

  pid_controller[0].update_target(0);//设置目标值为0
  pid_controller[1].update_target(0);

  //13.创建一个任务来启动micro-ros的task
  //扩大任务栈到40KB以防止内存不足
  xTaskCreate(microros_task,"micros_task",40960,NULL,1,NULL);
 
}


/*------------------------------------------LOOP函数---------------------------------------------------------*/
void loop() {
  // 更新IMU数据
  updateIMU();

  //Serial.printf("tick_0=%d,tick_1=%d\n",encoders[0].getTicks(),encoders[1].getTicks());
  kinematics.update_motor_speed(millis(),encoders[0].getTicks(),encoders[1].getTicks());
  //Serial.printf("left_tick=%d,right_tick=%d\n",encoders[0].getTicks(),encoders[1].getTicks());

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
  //Serial.printf("x,y,yaw=%f,%f,%f\n",kinematics.get_odom().x,kinematics.get_odom().y,
  //                                   kinematics.get_odom().angle);

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
  
  // 第6行: IMU偏航角
  display.setCursor(0, 50);
  display.print("Yaw:");
  display.print(imu_yaw * 180 / PI, 1);
  display.print("°");

  // 第7行: 里程计的角度
  display.setCursor(64, 50);
  display.print("Odom:");
  display.print(kinematics.get_odom().angle * 180 / PI, 1);
  display.print("°");
  
  display.display();
}

// 更新IMU数据并计算偏航角
void updateIMU() {
  updateComplementaryFilter(); // 使用改进的互补滤波
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
  
  // Serial.println("Calibration results:");
  // Serial.printf("Gyro bias: X=%.6f, Y=%.6f, Z=%.6f rad/s\n", gyro_bias_x, gyro_bias_y, gyro_bias_z);
  // Serial.printf("Accel bias: X=%.6f, Y=%.6f, Z=%.6f m/s^2\n", accel_bias_x, accel_bias_y, accel_bias_z);
  
  // 保存校准值（在实际应用中应该保存到EEPROM）
}

// 检测是否应该完全停止
bool shouldStopCompletely() {
    return (fabs(target_linear_speed) < 0.01 && fabs(target_angular_speed) < 0.01);
}

// 电机校准函数
void calibrate_motors() {
  Serial.println("开始电机校准...");
  
  // 测试两个电机在相同PWM下的速度差异
  const int test_pwm = 200;
  const int test_duration = 3000; // 3秒
  
  // 记录初始编码器值
  int32_t left_start = encoders[0].getTicks();
  int32_t right_start = encoders[1].getTicks();
  
  // 运行电机
  motor[0].updateMotorSpeed(0, test_pwm);
  motor[0].updateMotorSpeed(1, test_pwm);
  motor[1].updateMotorSpeed(0, test_pwm);
  motor[1].updateMotorSpeed(1, test_pwm);
  
  delay(test_duration);
  
  // 停止电机
  motor[0].updateMotorSpeed(0, 0);
  motor[0].updateMotorSpeed(1, 0);
  motor[1].updateMotorSpeed(0, 0);
  motor[1].updateMotorSpeed(1, 0);
  
  delay(500); // 等待完全停止
  
  // 读取编码器计数
  int32_t left_ticks = encoders[0].getTicks() - left_start;
  int32_t right_ticks = encoders[1].getTicks() - right_start;
  
  Serial.printf("校准结果 - 左轮脉冲: %d, 右轮脉冲: %d\n", left_ticks, right_ticks);
  
  // 计算补偿系数（如果需要）
  if (abs(left_ticks - right_ticks) > 100) {
    float compensation = (float)left_ticks / right_ticks;
    Serial.printf("建议右轮PID参数乘以补偿系数: %.3f\n", compensation);
  }
  
  delay(1000);
}

// 陀螺仪零偏校准函数
void calibrateGyroBias() {
  if (!mpu_initialized) return;
  
  Serial.println("Calibrating gyroscope bias... Please keep the robot stationary!");
  
  const int calibration_samples = 1000;
  float gx_sum = 0.0, gy_sum = 0.0, gz_sum = 0.0;
  
  for (int i = 0; i < calibration_samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    gx_sum += g.gyro.x;
    gy_sum += g.gyro.y;
    gz_sum += g.gyro.z;
    
    delay(5);
    
    // 显示校准进度
    if (i % 100 == 0) {
      Serial.printf("Calibration progress: %d%%\n", (i * 100) / calibration_samples);
    }
  }
  
  gyro_bias_z = gz_sum / calibration_samples;
  gyro_calibrated = true;
  
  Serial.printf("Gyro biases - X: %.6f, Y: %.6f, Z: %.6f rad/s\n", 
                gx_sum / calibration_samples, 
                gy_sum / calibration_samples, 
                gyro_bias_z);
  Serial.println("Gyroscope calibration completed!");
}


// 改进的互补滤波函数
void updateComplementaryFilter() {
  if (!mpu_initialized || !gyro_calibrated) return;
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // 保存原始数据供Micro-ROS使用
  imu_angular_velocity_x = g.gyro.x;
  imu_angular_velocity_y = g.gyro.y;
  imu_angular_velocity_z = g.gyro.z - gyro_bias_z; // 应用零偏校准
  
  imu_linear_acceleration_x = a.acceleration.x;
  imu_linear_acceleration_y = a.acceleration.y;
  imu_linear_acceleration_z = a.acceleration.z;
  
  // 计算时间差
  unsigned long current_time = millis();
  float dt = (current_time - last_comp_filter_time) / 1000.0f;
  last_comp_filter_time = current_time;
  
  // 确保dt在合理范围内
  if (dt <= 0 || dt > 0.1) {
    dt = 0.01; // 默认10ms
  }
  
  // 重要修改：对于偏航角，只使用陀螺仪积分
  // 加速度计无法提供偏航角的绝对参考，因此不使用加速度计修正偏航角
  float gyro_rate = imu_angular_velocity_z;
  
  // 纯陀螺仪积分（会漂移，但短期准确）
  complementary_angle += gyro_rate * dt;
  
  // 使用滤波器平滑
  yawFilter.Filter(complementary_angle);
  imu_yaw = yawFilter.Current();
  
  // 角度归一化到[-π, π]
  while (imu_yaw > PI) imu_yaw -= 2 * PI;
  while (imu_yaw < -PI) imu_yaw += 2 * PI;
  
  // 调试输出
  static unsigned long last_debug = 0;
  if (millis() - last_debug > 1000) {
    last_debug = millis();
    Serial.printf("Yaw: %.2f°, Gyro Z: %.3f rad/s, dt: %.3f s\n",
                 imu_yaw * 180 / PI, gyro_rate, dt);
  }
}

// #include <Arduino.h>
// #include <Esp32PcntEncoder.h>

// Esp32PcntEncoder encoders[2]; // 创建一个数组用于存储两个编码器

// void setup()
// {
//   // 1.初始化串口
//   Serial.begin(115200); // 初始化串口通信，设置通信速率为115200

//   // 2.设置编码器
//   encoders[0].init(0, 13, 12); // 初始化第一个编码器，使用GPIO 13和12连接
//   encoders[1].init(1, 18, 5); // 初始化第二个编码器，使用GPIO 18和5连接
// }

// void loop()
// {
//   delay(10); // 等待10毫秒

//   // 读取并打印两个编码器的计数器数值
//   Serial.printf("tick1=%d,tick2=%d\n", encoders[0].getTicks(), encoders[1].getTicks());
// }
