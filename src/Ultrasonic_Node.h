#ifndef ULTRASONIC_NODE_H
#define ULTRASONIC_NODE_H

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "Micro_ROS.h" 
#include <sensor_msgs/msg/range.h>

// 声明超声波相关的ROS2变量
extern rcl_publisher_t pub_ultrasonic;
extern sensor_msgs__msg__Range msg_ultrasonic;
extern rcl_timer_t ultrasonic_timer;

// 超声波相关函数
void ultrasonic_timer_callback(rcl_timer_t* timer, int64_t last_call_time);
void init_ultrasonic_node(rcl_node_t* node, rclc_support_t* support, rclc_executor_t* executor);
float read_ultrasonic_distance();

#endif