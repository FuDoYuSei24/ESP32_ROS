// Micro_ROS.h
#ifndef MICRO_ROS_H
#define MICRO_ROS_H

#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <micro_ros_utilities/string_utilities.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/range.h>  // 添加超声波消息支持
#include "PIDController.h"

// 这些宏在原文件中用于心跳检测，保留在这里以便 micro-ROS 相关代码使用
#ifndef PUBLISH_TIMEOUT
#define PUBLISH_TIMEOUT 3000
#endif
#ifndef AGENT_CHECK_INTERVAL
#define AGENT_CHECK_INTERVAL 1000
#endif

// 外部符号引用（在 main.cpp 中定义的对象）
class Kinematics; // 前向声明，实际类型在 main.cpp 中定义并包含 Kinematics.h
extern Kinematics kinematics;
extern void updateDisplay(); // main.cpp 中定义的 OLED 更新函数

// micro-ROS 相关的全局变量与函数声明
extern char ssid[];
extern char password[];
extern IPAddress agent_ip;
extern rcl_subscription_t sub_cmd_vel;
extern geometry_msgs__msg__Twist msg_cmd_vel;
extern rcl_publisher_t pub_odom;
extern nav_msgs__msg__Odometry msg_odom;
extern rcl_timer_t timer;
extern rcl_allocator_t allocator;
extern rclc_support_t support;
extern rclc_executor_t executor;
extern rcl_node_t node;
extern bool microros_connected;
extern unsigned long last_successful_publish;
extern unsigned long last_agent_check;
extern bool wifi_connected;
extern String ip_address;
extern PIDController pid_controller[2];

// micro-ROS 回调和任务
void timer_callback(rcl_timer_t* timer, int64_t last_call_time);
void twist_callback(const void* msg_in);
void microros_task(void* args);




#endif // MICRO_ROS_H
