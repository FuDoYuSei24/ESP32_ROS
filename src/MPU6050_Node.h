#ifndef MPU6050_NODE_H
#define MPU6050_NODE_H

#include <rcl/rcl.h>
#include <rclc/rclc.h>
// 需要 executor 的类型声明
#include <rclc/executor.h>
#include "Micro_ROS.h" 
#include <sensor_msgs/msg/imu.h>

// 声明MPU6050相关的ROS2变量
extern rcl_publisher_t pub_imu;
extern sensor_msgs__msg__Imu msg_imu;
extern rcl_timer_t imu_timer;

void imu_timer_callback(rcl_timer_t* timer, int64_t last_call_time);
void init_mpu6050_node(rcl_node_t* node, rclc_support_t* support, rclc_executor_t* executor);

#endif