#include "Ultrasonic_Node.h"
#include <Arduino.h>
#include <rmw/types.h>

// 引脚定义
const int TRIG_PIN = 12;
const int ECHO_PIN = 13;

// 超声波参数
const float SOUND_SPEED = 0.0343; // 声速 (cm/μs)
const float MAX_DISTANCE = 400.0; // 最大检测距离 (cm)
const float MIN_DISTANCE = 2.0;   // 最小检测距离 (cm)

// Micro-ROS变量
rcl_publisher_t pub_ultrasonic;
sensor_msgs__msg__Range msg_ultrasonic;
rcl_timer_t ultrasonic_timer;

// 读取超声波距离
float read_ultrasonic_distance() {
  // 发送触发信号
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // 读取回声时间
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms超时
  
  // 计算距离 (cm)
  if (duration == 0) {
    return -1.0; // 超时或没有检测到物体
  }
  
  float distance = duration * SOUND_SPEED / 2.0;
  
  // 限制在有效范围内
  if (distance < MIN_DISTANCE || distance > MAX_DISTANCE) {
    return -1.0;
  }
  
  return distance;
}

// 超声波定时器回调
void ultrasonic_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  (void) timer;
  (void) last_call_time;
  
  Serial.println("ULTRASONIC TIMER CALLBACK TRIGGERED!");
  
  // 读取距离
  float distance = read_ultrasonic_distance();
  Serial.printf("Distance reading: %.1f cm\n", distance);
  
  // 获取当前时间戳
  int64_t stamp = rmw_uros_epoch_millis();
  msg_ultrasonic.header.stamp.sec = static_cast<int32_t>(stamp / 1000);
  msg_ultrasonic.header.stamp.nanosec = static_cast<uint32_t>((stamp % 1000) * 1000000);
  
  // 设置测量距离
  if (distance >= 0) {
    msg_ultrasonic.range = distance / 100.0;
  } else {
    msg_ultrasonic.range = msg_ultrasonic.max_range + 0.1f;
  }
  
  Serial.printf("Publishing range: %.3f m\n", msg_ultrasonic.range);
  
  // 发布数据
  rcl_ret_t ret = rcl_publish(&pub_ultrasonic, &msg_ultrasonic, NULL);
  if (ret != RCL_RET_OK) {
    Serial.printf("Ultrasonic publish FAILED: %d\n", ret);
  } else {
    Serial.println("Ultrasonic publish SUCCESS");
  }
}

// 初始化超声波节点
void init_ultrasonic_node(rcl_node_t* node, rclc_support_t* support, rclc_executor_t* executor) {
   // 初始化引脚
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  Serial.println("Initializing ultrasonic sensor pins...");
  msg_ultrasonic.radiation_type = sensor_msgs__msg__Range__ULTRASOUND; // 必须指定为超声波类型
  msg_ultrasonic.field_of_view = 0.5236f; // 视场角（弧度，例如30度=0.5236rad）
  msg_ultrasonic.min_range = 0.02f; // 最小测距（米）
  msg_ultrasonic.max_range = 4.0f;  // 最大测距（米）
  msg_ultrasonic.range = 0.0f;      // 初始距离（回调中更新） 
  
  // 初始化发布者 - 使用正确的rmw QoS配置
  rmw_qos_profile_t qos = rmw_qos_profile_default;  // 使用rmw命名空间的QoS结构体
  qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;  // 使用rmw命名空间的宏
  qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  qos.depth = 10;

  rcl_ret_t ret = rclc_publisher_init(
    &pub_ultrasonic,
    node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    "/ultrasonic",
    &qos
  );
  
  if (ret != RCL_RET_OK) {
    Serial.println("Failed to initialize ultrasonic publisher");
    return;
  }

  // 初始化定时器 (每200ms发布一次)
  ret = rclc_timer_init_default(&ultrasonic_timer, support, RCL_MS_TO_NS(200), ultrasonic_timer_callback);
  if (ret != RCL_RET_OK) {
    Serial.println("Failed to initialize ultrasonic timer");
    return;
  }

  // 添加定时器到执行器
  ret = rclc_executor_add_timer(executor, &ultrasonic_timer);
  if (ret != RCL_RET_OK) {
    Serial.println("Failed to add ultrasonic timer to executor");
    return;
  }

  // 预初始化帧ID（仅执行一次）
  msg_ultrasonic.header.frame_id = micro_ros_string_utilities_set(
    msg_ultrasonic.header.frame_id, "ultrasonic_sensor"
  );

  Serial.println("Ultrasonic node initialized successfully");
}