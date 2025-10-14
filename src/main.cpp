/*---------------------------------------------头文件引用区------------------------------------------------------*/
#include <Arduino.h>
#include "Esp32McpwmMotor.h"
#include <Esp32PcntEncoder.h>
#include "PIDController.h"
#include "Kinematics.h"
//引入micro-ROS和wifi相关库
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
//引入字符串内存分配管理工具
#include <micro_ros_utilities/string_utilities.h>
//引入消息接口
#include <geometry_msgs/msg/twist.h>//消息接口
//引入里程计消息接口
#include <nav_msgs/msg/odometry.h>
//oled显示屏相关库
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
//#include <U8g2lib.h>
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
// 使用char数组而不是const char*
char ssid[] = "fudoyusei";
char password[] = "12345678";
IPAddress agent_ip(192, 168, 36, 191);//同一个wifi下的上位机IP地址

//引入消息接口
rcl_subscription_t sub_cmd_vel;//创建一个消息的订阅者
geometry_msgs__msg__Twist msg_cmd_vel;//订阅到的数据存储在这里
//引入里程计消息接口
rcl_publisher_t pub_odom;//创建一个里程计发布者
nav_msgs__msg__Odometry msg_odom;//里程计消息存储到这里
rcl_timer_t timer;//定时器，定时调用某个函数

Esp32McpwmMotor motor[2];
Esp32PcntEncoder encoders[2]; // 创建一个数组用于存储两个编码器
PIDController pid_controller[2];//创建一个数组用于PID控制
Kinematics kinematics;//运动学对象

//声明一些相关的结构体对象
rcl_allocator_t allocator;//内存分配器。用于动态内存分配管理
rclc_support_t support;//用于存储时钟，内存分配器和上下文，用于提供支持
rclc_executor_t executor;//执行器，用于管理订阅和计时器回调的执行
rcl_node_t node;        //结点，用于创建结点

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

//心跳检测相关变量
bool wifi_connected = false;
String ip_address = "Connecting...";
bool microros_connected = false;
unsigned long last_successful_publish = 0;
unsigned long last_agent_check = 0;



/*------------------------------------------函数声明区---------------------------------------------------------*/
void motorSpeedControl();//函数用于控制电机速度（闭环控制）
//定时器回调函数
void timer_callback(rcl_timer_t* timer,int64_t last_call_time);
//话题的回调函数
void twist_callback(const void* msg_in);
//单独创建一个任务运行micro-ROS 相当于一个线程
void microros_task(void* args);
//oled初始化函数
void InitOLED();
//oled显示函数
void updateDisplay();
// 更新IMU数据并计算偏航角
void updateIMU();
//IMU校准函数
void calibrateIMU();




/*------------------------------------------SetUP函数---------------------------------------------------------*/
void setup() {
  // 1.初始化串口
  Serial.begin(115200); // 初始化串口通信，设置通信速率为115200

  // 2.设置编码器
  encoders[0].init(0, 15, 2); // 初始化第一个编码器，使用GPIO 32和33连接
  encoders[1].init(1, 27, 14); // 初始化第二个编码器，使用GPIO 2和4连接
  //3.初始化电机的引脚设置
  // motor[0].attachMotor(0, 12, 14, 27);
  // motor[0].attachMotor(1, 13, 26, 25);
  // motor[1].attachMotor(0, 5, 19, 18); 
  // motor[1].attachMotor(1, 23, 17, 16);

  //3.(id,pwm,in1,in2)
  motor[0].attachMotor(0, 4, 17, 16);//左前A
  motor[0].attachMotor(1, 26, 25, 33);//右前D
  motor[1].attachMotor(0, 19, 5, 18);//左后B
  motor[1].attachMotor(1, 34, 35, 32);//右后C

  
  //4.设置电机速度。这里初始化为0
  motor[0].updateMotorSpeed(0, 0);
  motor[0].updateMotorSpeed(1, 0);
  motor[1].updateMotorSpeed(0, 0);
  motor[1].updateMotorSpeed(1, 0);

  //5.初始化PID控制器
  pid_controller[0].update_pid(0.625,0.125,0.0);//pid参数
  pid_controller[1].update_pid(0.625,0.125,0.0);

  pid_controller[0].out_limit(-800,800);//设置输出限制
  pid_controller[1].out_limit(-800,800);

  pid_controller[0].uptate_target(0);//设置目标值为0
  pid_controller[1].uptate_target(0);

  // pid_controller[0].set_friction_compensation(50.0);//设置静摩擦补偿值
  // pid_controller[1].set_friction_compensation(50.0);

  //6.初始化运动学参数
  kinematics.set_wheel_distance(175.0); // 设置两个轮子之间的距离为175mm
  kinematics.set_motor_param(0,0.1051566);
  kinematics.set_motor_param(1,0.1051566);
    
  msg_odom.pose.pose.orientation.x = 0;
  msg_odom.pose.pose.orientation.y = 0;
  msg_odom.pose.pose.orientation.z = 0;
  msg_odom.pose.pose.orientation.w = 1;

  //7.创建一个任务来启动micro-ros的task
  xTaskCreate(microros_task,"micros_task",16384,NULL,1,NULL);

  //8.初始化I2C总线
  Wire.begin(21, 22); // SDA=GPIO21, SCL=GPIO22
  Wire.setClock(100000); // 降低I2C速度以提高兼容性
  
  
  //9.初始化OLED显示屏
  InitOLED();

  
  //10.初始化MPU6050
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
    
    // 初始校准
    calibrateIMU();
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
  current_speed[0] = delta_ticks[0] * 0.1051566 / dt * 1000;
  current_speed[1] = delta_ticks[1] * 0.1051566 / dt * 1000;
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

//定时器回调函数
void timer_callback(rcl_timer_t* timer,int64_t last_call_time)
{
  //完成里程计的发布
  odom_t odom = kinematics.get_odom();//获取当前的里程计
  int64_t stamp = rmw_uros_epoch_millis();//获取当前的时间
  msg_odom.header.stamp.sec = static_cast<int32_t>(stamp/1000);//秒部分
  msg_odom.header.stamp.nanosec = static_cast<int32_t>((stamp%1000)*1e6);//纳秒部分
  msg_odom.pose.pose.position.x = odom.x;
  msg_odom.pose.pose.position.y = odom.y;
  msg_odom.pose.pose.orientation.w = cos(odom.angle*0.5);
  msg_odom.pose.pose.orientation.x = 0;
  msg_odom.pose.pose.orientation.y = 0;
  msg_odom.pose.pose.orientation.z = sin(odom.angle*0.5);
  msg_odom.twist.twist.linear.x = odom.linear_speed;
  msg_odom.twist.twist.angular.z = odom.angular_speed;//修复

  //借助里程计的发布进行连接心跳检测
  //发布里程计，把数据发出去
  if(rcl_publish(&pub_odom,&msg_odom,NULL)==RCL_RET_OK)//里程计发布成功
  {
    last_successful_publish = millis();
    // 只有在发布成功时才认为连接正常
    if (!microros_connected) {
      microros_connected = true;
      Serial.println("Agent connection established!");
    }
  }
  else
  {
    Serial.println("error:odom pub failed");
  }
  
  // 主动检查Agent连接状态（每1秒检查一次）
  if (millis() - last_agent_check > AGENT_CHECK_INTERVAL) {
    last_agent_check = millis();
    
    // 检查是否超时
    if (millis() - last_successful_publish > PUBLISH_TIMEOUT) {
      if (microros_connected) {
        Serial.println("Agent connection lost - timeout!");
        microros_connected = false;
      }
    } else {
      // 检查网络连接状态
      if (WiFi.status() != WL_CONNECTED) {
        if (microros_connected) {
          Serial.println("WiFi disconnected!");
          microros_connected = false;
        }
      }
    }
  }

}


//话题的回调函数
void twist_callback(const void* msg_in)
{
  //将收到的消息指针转换成geometry_msgs_Twist类型的指针
  const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist*)msg_in;
  target_linear_speed = msg->linear.x;
  target_angular_speed = msg->angular.z;

  // // 确保有足够转向速度
  // const float MIN_ANGULAR = 1.0; // 最小角速度(rad/s)
  // if (fabs(target_angular_speed) > 0 && fabs(target_angular_speed) < MIN_ANGULAR) {
  //     target_angular_speed = (target_angular_speed > 0) ? MIN_ANGULAR : -MIN_ANGULAR;
  // }

  //测试运动学逆解
  kinematics.kinematics_inverse(target_linear_speed,target_angular_speed,
                                &out_left_speed,&out_right_speed);

  Serial.printf("OUT:left_speed=%f,right_speed=%f\n",out_left_speed,out_right_speed);

  pid_controller[0].uptate_target(out_left_speed);
  pid_controller[1].uptate_target(out_right_speed);


}


//单独创建一个任务运行micro-ROS 相当于一个线程
void microros_task(void* args)
{

  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nWiFi connection failed!");
    vTaskDelete(NULL); // 删除任务
    return;
  }
  
  Serial.println("\nWiFi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  
  // 更新WiFi状态和IP地址
  wifi_connected = true;  
  ip_address = WiFi.localIP().toString();  // 确保这里使用ip_address
  updateDisplay();

  // 直接调用函数，不检查返回值
  Serial.println("Setting up micro-ROS transport...");
  set_microros_wifi_transports(ssid, password, agent_ip, 8888);
  delay(2000);
  
  // 2. 初始化内存分配器
  allocator = rcl_get_default_allocator();
  
  // 3. 初始化支持模块
  rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
  if (ret != RCL_RET_OK) {
    Serial.printf("rclc_support_init error: %d\n", ret);
    vTaskDelete(NULL);
    return;
  }
  
  // 4. 初始化结点
  ret = rclc_node_init_default(&node, "robot_motion_control", "", &support);
  if (ret != RCL_RET_OK) {
    Serial.printf("rclc_node_init_default error: %d\n", ret);
    vTaskDelete(NULL);
    return;
  }
  
  // 5. 初始化执行器
  unsigned int num_handles = 2;
  ret = rclc_executor_init(&executor, &support.context, num_handles, &allocator);
  if (ret != RCL_RET_OK) {
    Serial.printf("rclc_executor_init error: %d\n", ret);
    vTaskDelete(NULL);
    return;
  }
  
  // 6. 初始化订阅者
  ret = rclc_subscription_init_best_effort(
    &sub_cmd_vel,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"
  );
  if (ret != RCL_RET_OK) {
    Serial.printf("rclc_subscription_init_best_effort error: %d\n", ret);
    vTaskDelete(NULL);
    return;
  }
  
  ret = rclc_executor_add_subscription(&executor, &sub_cmd_vel, &msg_cmd_vel, &twist_callback, ON_NEW_DATA);
  if (ret != RCL_RET_OK) {
    Serial.printf("rclc_executor_add_subscription error: %d\n", ret);
    vTaskDelete(NULL);
    return;
  }
  
  // 7. 初始化里程计消息
  msg_odom.header.frame_id = micro_ros_string_utilities_set(msg_odom.header.frame_id, "odom");
  msg_odom.child_frame_id = micro_ros_string_utilities_set(msg_odom.child_frame_id, "base_footprint");
  
  // 8. 初始化发布者和定时器
  ret = rclc_publisher_init_best_effort(
    &pub_odom,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "/odom"
  );
  if (ret != RCL_RET_OK) {
    Serial.printf("rclc_publisher_init_best_effort error: %d\n", ret);
    vTaskDelete(NULL);
    return;
  }
  
  ret = rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(50), timer_callback);
  if (ret != RCL_RET_OK) {
    Serial.printf("rclc_timer_init_default error: %d\n", ret);
    vTaskDelete(NULL);
    return;
  }
  
  ret = rclc_executor_add_timer(&executor, &timer);
  if (ret != RCL_RET_OK) {
    Serial.printf("rclc_executor_add_timer error: %d\n", ret);
    vTaskDelete(NULL);
    return;
  }
  
  // 9. 时间同步
  int sync_attempts = 0;
  while (!rmw_uros_epoch_synchronized() && sync_attempts < 10) {
    Serial.println("Synchronizing time with agent...");
    rmw_uros_sync_session(1000);
    delay(100);
    sync_attempts++;
  }
  
  if (!rmw_uros_epoch_synchronized()) {
    Serial.println("Time synchronization failed!");
    vTaskDelete(NULL);
    return;
  }
  
  Serial.println("micro-ROS setup complete!");

  // 初始化连接状态并立即更新OLED显示
  microros_connected = true;
  last_successful_publish = millis();
  last_agent_check = millis();
  updateDisplay();
  
  // 循环执行
  while (true) {
    // 检查WiFi连接状态
    if (WiFi.status() != WL_CONNECTED) {
        if (microros_connected) {
            Serial.println("WiFi connection lost in main loop!");
            microros_connected = false;
        }
    }
    rcl_ret_t rc = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    if (rc != RCL_RET_OK) {
      Serial.printf("rclc_executor_spin_some error: %d\n", rc);
      // 如果执行器出现错误，可能表示连接问题
      if (microros_connected) {
          microros_connected = false;
          Serial.println("Executor error - agent may be disconnected");
      }
      delay(100);
    }
  }
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
  
  // 计算时间差（秒）
  unsigned long current_time = millis();
  float dt = (current_time - last_imu_time) / 1000.0f;
  last_imu_time = current_time;
  
  // 确保dt在合理范围内
  if (dt <= 0 || dt > 0.1) {
    dt = 0.01; // 默认10ms
  }
  
  // 计算偏航角变化（陀螺仪Z轴积分）
  // 注意：根据您的安装方向，可能需要调整符号
  float yaw_rate = g.gyro.z; // 弧度/秒
  
  // 应用积分计算偏航角
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
