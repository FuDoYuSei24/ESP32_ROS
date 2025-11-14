#include "Micro_ROS.h"
#include "Kinematics.h"
#include "MPU6050_Node.h"
#include "Ultrasonic_Node.h"
#include <rmw/types.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>

// WiFi credentials and agent IP (moved from main.cpp)
// char ssid[] = "沙河汤臣一品";
// char password[] = "20050202";
// IPAddress agent_ip(192, 168, 0, 134);
char ssid[] = "fudoyusei";
char password[] = "12345678";
IPAddress agent_ip(192, 168, 119, 191);

// micro-ROS related globals (definitions)
rcl_subscription_t sub_cmd_vel;
geometry_msgs__msg__Twist msg_cmd_vel;
rcl_publisher_t pub_odom;
nav_msgs__msg__Odometry msg_odom;
rcl_timer_t timer;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;
rcl_node_t node;
bool microros_connected = false;
unsigned long last_successful_publish = 0;
unsigned long last_agent_check = 0;
bool wifi_connected = false;
String ip_address = "";
extern Kinematics kinematics;
extern void updateDisplay();

// twist 回调：接收 /cmd_vel 并把速度下发给运动学和 PID 目标
void twist_callback(const void* msg_in)
{
  const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist*)msg_in;
  float target_linear_speed = msg->linear.x;
  float target_angular_speed = msg->angular.z;

  float out_left_speed = 0.0f;
  float out_right_speed = 0.0f;
  kinematics.kinematics_inverse(target_linear_speed, target_angular_speed,
                                &out_left_speed, &out_right_speed);

  // 保留调试信息，但减少频率或删除
  // Serial.printf("New command: linear=%.2f, angular=%.2f -> OUT:left_speed=%f,right_speed=%f\n",
  //              target_linear_speed, target_angular_speed, out_left_speed, out_right_speed);

  // 订阅回调只负责将目标速度传给 PID/运动学层，具体的 PID 目标更新在其他模块完成（main.cpp 中仍有 PIDController）
  // 这里仍保留 msg_cmd_vel 以便于外部检查或复用
  extern PIDController pid_controller[2]; // 需要声明外部变量
  pid_controller[0].update_target(out_left_speed);
  pid_controller[1].update_target(out_right_speed);
  msg_cmd_vel = *msg;

}

// 定时器回调：发布里程计并做心跳检测
void timer_callback(rcl_timer_t* timer,int64_t last_call_time)
{
  odom_t odom = kinematics.get_odom();
  int64_t stamp = rmw_uros_epoch_millis();
  msg_odom.header.stamp.sec = static_cast<int32_t>(stamp/1000);
  msg_odom.header.stamp.nanosec = static_cast<int32_t>((stamp%1000)*1e6);
  msg_odom.pose.pose.position.x = odom.x;
  msg_odom.pose.pose.position.y = odom.y;
  msg_odom.pose.pose.orientation.w = cos(odom.angle*0.5);
  msg_odom.pose.pose.orientation.x = 0;
  msg_odom.pose.pose.orientation.y = 0;
  msg_odom.pose.pose.orientation.z = sin(odom.angle*0.5);
  msg_odom.twist.twist.linear.x = odom.linear_speed;
  msg_odom.twist.twist.angular.z = odom.angular_speed;

  if(rcl_publish(&pub_odom,&msg_odom,NULL)==RCL_RET_OK)
  {
    last_successful_publish = millis();
    if (!microros_connected) {
      microros_connected = true;
      Serial.println("Agent connection established!");
    }
  }
  else
  {
    // 只在调试时开启错误打印
    // Serial.println("error:odom pub failed");
  }

  if (millis() - last_agent_check > AGENT_CHECK_INTERVAL) {
    last_agent_check = millis();
    if (millis() - last_successful_publish > PUBLISH_TIMEOUT) {
      if (microros_connected) {
        Serial.println("Agent connection lost - timeout!");
        microros_connected = false;
      }
    } else {
      if (WiFi.status() != WL_CONNECTED) {
        if (microros_connected) {
          Serial.println("WiFi disconnected!");
          microros_connected = false;
        }
      }
    }
  }
}

// microros 任务：连接 WiFi、设置 transport、初始化 micro-ROS、启动执行器循环
void microros_task(void* args)
{
  //1.wifi链接
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
    vTaskDelete(NULL);
    return;
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  //2.更新显示（main.cpp 中的 updateDisplay）
  updateDisplay();
  //3.设置micro-ROS的WiFi传输
  set_microros_wifi_transports(ssid, password, agent_ip, 8888);
  delay(2000);
  //4.初始化内存分配器
  allocator = rcl_get_default_allocator();
  //5.初始化支持模块
  rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
  if (ret != RCL_RET_OK) {
    Serial.printf("rclc_support_init error: %d\n", ret);
    vTaskDelete(NULL);
    return;
  }
  //6.初始化节点、执行器、订阅者、发布者和定时器
  ret = rclc_node_init_default(&node, "robot_motion_control", "", &support);
  if (ret != RCL_RET_OK) {
    Serial.printf("rclc_node_init_default error: %d\n", ret);
    vTaskDelete(NULL);
    return;
  }

  //7.初始化执行器
  unsigned int num_handles = 4;//运动订阅者 + 里程计定时器 + IMU定时器 + 超声波定时器
  ret = rclc_executor_init(&executor, &support.context, num_handles, &allocator);
  if (ret != RCL_RET_OK) {
    Serial.printf("rclc_executor_init error: %d\n", ret);
    vTaskDelete(NULL);
    return;
  }

  //8.初始化订阅者
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
  //9.将订阅者添加到执行器
  ret = rclc_executor_add_subscription(&executor, &sub_cmd_vel, &msg_cmd_vel, &twist_callback, ON_NEW_DATA);
  if (ret != RCL_RET_OK) {
    Serial.printf("rclc_executor_add_subscription error: %d\n", ret);
    vTaskDelete(NULL);
    return;
  }
  //10.初始化里程计信息
  msg_odom.header.frame_id = micro_ros_string_utilities_set(msg_odom.header.frame_id, "odom");
  msg_odom.child_frame_id = micro_ros_string_utilities_set(msg_odom.child_frame_id, "base_footprint");
  //11.初始化里程计发布者
  
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
  //12.初始化定时器
  ret = rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(50), timer_callback);
  if (ret != RCL_RET_OK) {
    Serial.printf("rclc_timer_init_default error: %d\n", ret);
    vTaskDelete(NULL);
    return;
  }
  //13.将定时器添加到执行器
  ret = rclc_executor_add_timer(&executor, &timer);
  if (ret != RCL_RET_OK) {
    Serial.printf("rclc_executor_add_timer error: %d\n", ret);
    vTaskDelete(NULL);
    return;
  }
  //14.MPU6050节点初始化
  init_mpu6050_node(&node, &support, &executor);
  //15.超声波节点初始化
  //init_ultrasonic_node(&node, &support, &executor);

  //16.时间同步 - 增强版本
  int sync_attempts = 0;
  bool time_synced = false;
  while (!rmw_uros_epoch_synchronized() && sync_attempts < 50) { // 增加尝试次数
    rmw_uros_sync_session(1000);
    delay(200); // 稍微延长延迟
    sync_attempts++;
    
    // 检查是否同步成功
    if (rmw_uros_epoch_synchronized()) {
      time_synced = true;
      break;
    }
  }

  if (!time_synced) {
    Serial.println("Time synchronization failed, but continuing...");
  } else {
    Serial.println("Time synchronization completed successfully!");
  }

  Serial.println("micro-ROS setup complete!");
  microros_connected = true;
  last_successful_publish = millis();
  last_agent_check = millis();
  updateDisplay();

  while (true) {
  static unsigned long last_status_check = 0;
  if (millis() - last_status_check > 5000) {
    last_status_check = millis();
    
    // 减少状态检查的打印频率或删除
    // Serial.printf("=== System Status ===\n");
    // Serial.printf("WiFi: %s\n", WiFi.status() == WL_CONNECTED ? "CONNECTED" : "DISCONNECTED");
    // Serial.printf("Agent: %s\n", microros_connected ? "CONNECTED" : "DISCONNECTED");
    // Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());
    // Serial.printf("Time synced: %s\n", rmw_uros_epoch_synchronized() ? "YES" : "NO");
    // Serial.printf("Current time: %lld\n", rmw_uros_epoch_millis());
    // Serial.printf("=====================\n");
  }
  
  rcl_ret_t rc = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  if (rc != RCL_RET_OK) {
    // 只在调试时开启错误打印
    // Serial.printf("Executor spin error: %d\n", rc);
    if (microros_connected) {
      microros_connected = false;
      Serial.println("Agent connection lost due to executor error");
    }
    delay(100);
  }
 }
}