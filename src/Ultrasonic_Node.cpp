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
  // 确保引脚状态正确
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  // 发送10μs的高电平脉冲
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // 读取回声时间 - 增加超时时间并添加重试机制
  long duration = 0;
  int attempts = 0;
  
  while (duration == 0 && attempts < 3) {
    duration = pulseIn(ECHO_PIN, HIGH, 40000); // 40ms超时（更长的超时）
    attempts++;
    if (duration == 0) {
      delay(10); // 短暂延迟后重试
    }
  }
  
  Serial.printf("Ultrasonic - Duration: %ld μs, Attempts: %d\n", duration, attempts);
  
  // 计算距离 (cm)
  if (duration == 0) {
    Serial.println("Ultrasonic - No echo received (timeout)");
    return -1.0;
  }
  
  float distance = duration * SOUND_SPEED / 2.0;
  Serial.printf("Ultrasonic - Calculated distance: %.1f cm\n", distance);
  
  // 检查距离是否在有效范围内
  if (distance < MIN_DISTANCE) {
    Serial.printf("Ultrasonic - Distance too small: %.1f cm < %.1f cm\n", distance, MIN_DISTANCE);
    return -1.0;
  }
  
  if (distance > MAX_DISTANCE) {
    Serial.printf("Ultrasonic - Distance too large: %.1f cm > %.1f cm\n", distance, MAX_DISTANCE);
    return -1.0;
  }
  
  Serial.printf("Ultrasonic - Valid distance: %.1f cm\n", distance);
  return distance;
}

// 超声波定时器回调
void ultrasonic_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  (void) timer;
  (void) last_call_time;
  
  Serial.println("=== ULTRASONIC CALLBACK START ===");
  
  // 检查发布者是否有效（使用更安全的方法）
  bool publisher_valid = true;
  // 我们可以通过尝试获取发布者类型支持来间接检查有效性
  // 如果发布者无效，发布时会返回错误
  
  // 读取距离
  float distance = read_ultrasonic_distance();
  Serial.printf("Raw distance: %.1f cm\n", distance);
  
  // 检查时间同步状态
  bool time_synced = rmw_uros_epoch_synchronized();
  Serial.printf("Time synchronized: %s\n", time_synced ? "YES" : "NO");
  
  int64_t stamp = rmw_uros_epoch_millis();
  Serial.printf("Current timestamp: %lld ms\n", stamp);
  
  if (stamp > 0) {
    msg_ultrasonic.header.stamp.sec = static_cast<int32_t>(stamp / 1000);
    msg_ultrasonic.header.stamp.nanosec = static_cast<uint32_t>((stamp % 1000) * 1000000);
  } else {
    // 使用相对时间
    static int32_t fallback_sec = 0;
    static uint32_t fallback_nsec = 0;
    fallback_nsec += 200000000;
    if (fallback_nsec >= 1000000000) {
      fallback_sec++;
      fallback_nsec -= 1000000000;
    }
    msg_ultrasonic.header.stamp.sec = fallback_sec;
    msg_ultrasonic.header.stamp.nanosec = fallback_nsec;
    Serial.printf("Using fallback time: %d.%09d\n", fallback_sec, fallback_nsec);
  }
  
  // 确保帧ID设置
  if (msg_ultrasonic.header.frame_id.data == NULL || 
      msg_ultrasonic.header.frame_id.size == 0) {
    msg_ultrasonic.header.frame_id = micro_ros_string_utilities_set(
      msg_ultrasonic.header.frame_id, "ultrasonic_sensor"
    );
    Serial.println("Frame ID was NULL, reinitialized");
  }
  
  // 设置距离值
  if (distance >= MIN_DISTANCE && distance <= MAX_DISTANCE) {
    msg_ultrasonic.range = distance / 100.0;
    Serial.printf("Valid range: %.3f m\n", msg_ultrasonic.range);
  } else {
    msg_ultrasonic.range = msg_ultrasonic.max_range + 0.01f;
    Serial.printf("Invalid range, setting to: %.3f m\n", msg_ultrasonic.range);
  }
  
  // 检查消息内容
  Serial.printf("Message details:\n");
  Serial.printf("  Frame ID: %s\n", msg_ultrasonic.header.frame_id.data);
  Serial.printf("  Stamp: %d.%09d\n", msg_ultrasonic.header.stamp.sec, msg_ultrasonic.header.stamp.nanosec);
  Serial.printf("  Range: %.3f m\n", msg_ultrasonic.range);
  Serial.printf("  Min range: %.3f m\n", msg_ultrasonic.min_range);
  Serial.printf("  Max range: %.3f m\n", msg_ultrasonic.max_range);
  Serial.printf("  Radiation type: %d\n", msg_ultrasonic.radiation_type);
  Serial.printf("  Field of view: %.3f rad\n", msg_ultrasonic.field_of_view);
  
  // 发布数据
  rcl_ret_t ret = rcl_publish(&pub_ultrasonic, &msg_ultrasonic, NULL);
  Serial.printf("Publish return code: %d\n", ret);
  
  if (ret != RCL_RET_OK) {
    Serial.printf("Publish FAILED with error: %d\n", ret);
    // 打印具体的错误信息
    switch (ret) {
      case RCL_RET_OK: Serial.println("  -> RCL_RET_OK"); break;
      case RCL_RET_ERROR: Serial.println("  -> RCL_RET_ERROR"); break;
      case RCL_RET_TIMEOUT: Serial.println("  -> RCL_RET_TIMEOUT"); break;
      case RCL_RET_NODE_INVALID: Serial.println("  -> RCL_RET_NODE_INVALID"); break;
      case RCL_RET_PUBLISHER_INVALID: Serial.println("  -> RCL_RET_PUBLISHER_INVALID"); break;
      case RCL_RET_BAD_ALLOC: Serial.println("  -> RCL_RET_BAD_ALLOC"); break;
      case RCL_RET_INVALID_ARGUMENT: Serial.println("  -> RCL_RET_INVALID_ARGUMENT"); break;
      default: Serial.printf("  -> Unknown error code: %d\n", ret); break;
    }
  } else {
    Serial.println("Publish SUCCESS - message should be visible on ROS2");
  }
  
  Serial.println("=== ULTRASONIC CALLBACK END ===\n");
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