#include "Kinematics.h"
#include <cmath> // 添加 cmath 头文件支持数学函数

// 辅助函数：计算数组平均值
float average(float arr[], int size) {
    float sum = 0;
    for (int i = 0; i < size; i++) sum += arr[i];
    return sum / size;
}

//设置轮子之间的距离
void Kinematics::set_wheel_distance(float distance)
{
    wheel_distance = distance;
}

//设置指定电机的速度
void Kinematics::set_motor_param(uint8_t id,float per_pluse_distancee)
{
    motor_param[id].per_pulse_distance = per_pluse_distancee;
}

//运动学正解
void Kinematics::kinematics_forward(float left_speed,float right_speed,float* out_linear_speed,float* out_angular_speed)
{
    *out_linear_speed = (left_speed + right_speed) / 2000.0;
    *out_angular_speed = (right_speed - left_speed) / wheel_distance;
}

//运动学逆解
void Kinematics::kinematics_inverse(float linear_speed,float angular_speed,float* out_left_speed,float* out_right_speed)
{
    // 只做一次转换：m/s → mm/s
    float linear_mm_s = linear_speed * 1000.0;

    *out_left_speed = linear_mm_s - (angular_speed * wheel_distance) / 2.0;
    *out_right_speed = linear_mm_s + (angular_speed * wheel_distance) / 2.0;

    // 处理纯线速度情况，避免浮点误差
    if(angular_speed==0){
        *out_left_speed = *out_right_speed;
    }
}

//输入：左右轮脉冲数，当前时间；输出：更新的电机速度和编码器数据
void Kinematics::update_motor_speed(uint64_t current_time,int32_t left_tick,int32_t right_tick)
{ 
    //计算时间差
    uint32_t dt = current_time - last_update_time;//单位：ms
    if(dt == 0) return; // 避免除以0

    // 使用移动平均滤波计算速度
    static float left_speed_history[3] = {0};
    static float right_speed_history[3] = {0};
    static int history_index = 0;

    //计算编码器当前与上一次读取的数值之差
    delta_ticks[0] =  left_tick - motor_param[0].last_encoder_ticks;
    delta_ticks[1] = right_tick - motor_param[1].last_encoder_ticks;

    // 计算原始速度
    float left_raw_speed = delta_ticks[0] * motor_param[0].per_pulse_distance / dt * 1000;
    float right_raw_speed = delta_ticks[1] * motor_param[1].per_pulse_distance / dt * 1000;
    
    // 移动平均滤波
    left_speed_history[history_index] = left_raw_speed;
    right_speed_history[history_index] = right_raw_speed;
    history_index = (history_index + 1) % 3;
    
    motor_param[0].motor_speed = (left_speed_history[0] + left_speed_history[1] + left_speed_history[2]) / 3;
    motor_param[1].motor_speed = (right_speed_history[0] + right_speed_history[1] + right_speed_history[2]) / 3;

    //更新last_tick
    motor_param[0].last_encoder_ticks = left_tick;
    motor_param[1].last_encoder_ticks = right_tick;
    last_update_time = current_time;

    //更新里程计
    update_odom(dt);

 
}

//获取电机速度,返回值是速度
int16_t Kinematics::get_motor_speed(uint8_t id)
{
    if(id<0||id>1)
    {
        return -1;
    }
    //Serial.printf("motor%d:speed:%d\n",id,motor_param[id].motor_speed);
    return motor_param[id].motor_speed;
}

odom_t& Kinematics::get_odom(){
    return odom;
}

// Kinematics.cpp - 修复角度归一化
void Kinematics::TransAngleInPI(float& angle)
{
    while (angle > PI) angle -= 2*PI;
    while (angle < -PI) angle += 2*PI;
}

void Kinematics::update_odom(uint16_t dt)
{
     float dt_s = float(dt) / 1000.0f; // 单位：ms->s
  
  // 获取实时的角速度和线速度
  this->kinematics_forward(motor_param[0].motor_speed, motor_param[1].motor_speed,
                          &odom.linear_speed, &odom.angular_speed);
  
  // 使用IMU的偏航角（如果可用）
  if (mpu_initialized) {
    // 直接使用IMU计算的偏航角
    odom.angle = imu_yaw;
    
  } 
  else {
    // 没有IMU时使用编码器积分
    float delta_angle = odom.angular_speed * dt_s;
    odom.angle += delta_angle;
    TransAngleInPI(odom.angle); // 角度归一化
  }

  
  // 计算机器人行走的距离
  float delta_distance = odom.linear_speed * dt_s;
  
  // 计算坐标：行走距离分解到xy轴
  odom.x += delta_distance * std::cos(odom.angle);
  odom.y += delta_distance * std::sin(odom.angle);
}

void Kinematics::reset_odom()
{
    odom.x = 0;
    odom.y = 0;
    odom.angle = 0;
    odom.linear_speed = 0;
    odom.angular_speed = 0;
    
    // 同时重置编码器相关的历史数据
    motor_param[0].last_encoder_ticks = 0;
    motor_param[1].last_encoder_ticks = 0;
    delta_ticks[0] = 0;
    delta_ticks[1] = 0;
    last_update_time = 0;
}
