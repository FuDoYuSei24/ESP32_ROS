// 包含公共 Micro_ROS 接口（从 include/ 目录）
#include <Micro_ROS.h>
#include "MPU6050_Node.h"

// MPU6050相关变量
extern bool mpu_initialized;
extern float imu_yaw;
extern float imu_angular_velocity_x, imu_angular_velocity_y, imu_angular_velocity_z;
extern float imu_linear_acceleration_x, imu_linear_acceleration_y, imu_linear_acceleration_z;
extern void updateIMU();

// Micro-ROS变量
rcl_publisher_t pub_imu;
sensor_msgs__msg__Imu msg_imu;
rcl_timer_t imu_timer;

void imu_timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
  if (!mpu_initialized) return;

  updateIMU();

  int64_t stamp = rmw_uros_epoch_millis();
  msg_imu.header.stamp.sec = static_cast<int32_t>(stamp/1000);
  msg_imu.header.stamp.nanosec = static_cast<int32_t>((stamp%1000)*1e6);
  
  msg_imu.header.frame_id = micro_ros_string_utilities_set(msg_imu.header.frame_id, "imu_link");
  
  // 四元数（只有偏航角）
  msg_imu.orientation.x = 0.0;
  msg_imu.orientation.y = 0.0;
  msg_imu.orientation.z = sin(imu_yaw * 0.5);
  msg_imu.orientation.w = cos(imu_yaw * 0.5);
  
  // 角速度
  msg_imu.angular_velocity.x = imu_angular_velocity_x;
  msg_imu.angular_velocity.y = imu_angular_velocity_y;
  msg_imu.angular_velocity.z = imu_angular_velocity_z;
  
  // 线加速度
  msg_imu.linear_acceleration.x = imu_linear_acceleration_x;
  msg_imu.linear_acceleration.y = imu_linear_acceleration_y;
  msg_imu.linear_acceleration.z = imu_linear_acceleration_z;
  
  rcl_ret_t ret = rcl_publish(&pub_imu, &msg_imu, NULL);
  if (ret == RCL_RET_OK) {
    // 发布成功
  }
}

void init_mpu6050_node(rcl_node_t* node, rclc_support_t* support, rclc_executor_t* executor)
{
  // 初始化IMU发布者
  rcl_ret_t ret = rclc_publisher_init_best_effort(
    &pub_imu,
    node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "/imu/data"
  );
  
  if (ret != RCL_RET_OK) {
    Serial.println("Failed to initialize IMU publisher");
    return;
  }

  // 初始化IMU定时器
  ret = rclc_timer_init_default(&imu_timer, support, RCL_MS_TO_NS(100), imu_timer_callback);
  if (ret != RCL_RET_OK) {
    Serial.println("Failed to initialize IMU timer");
    return;
  }

  // 添加定时器到执行器
  ret = rclc_executor_add_timer(executor, &imu_timer);
  if (ret != RCL_RET_OK) {
    Serial.println("Failed to add IMU timer to executor");
    return;
  }

  Serial.println("MPU6050 node initialized successfully");
}