#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

#include "Arduino.h"
extern bool mpu_initialized;  // 在 main.cpp 中定义
extern float imu_yaw;         // 在 main.cpp 中定义

typedef struct{
    float per_pulse_distance;//每个脉冲的前进距离
    int16_t motor_speed;//单位用mm/s
    int64_t last_encoder_ticks;//上一次电机的编码器读数
}motor_param_t;

typedef struct{
    float x;
    float y;
    float angle;
    float linear_speed;
    float angular_speed;
}odom_t;//里程计结构体

/*
*1.运动学正逆解（两个轮子的实时速度->当前实时的角速度和线速度/
*               当前目标的角速度和线速度->两个轮子的目标速度）
*/
class Kinematics
{
    private:
        motor_param_t motor_param[2];
        int16_t delta_ticks[2] = {0,0};//用于存储这一次读取的编码器数值
        uint64_t last_update_time = 0;//用于存储上一次更新电机速度的时间
        float wheel_distance = 0.0;//两个轮子之间的距离
        odom_t odom;//用于存储里程计信息

    public:
        Kinematics(/* args */) = default;
        ~Kinematics() = default;

        void update_odom(uint16_t dt);//更新里程计函数

        odom_t& get_odom(); //获取里程计函数

        void TransAngleInPI(float& angle);

        void set_wheel_distance(float distance);//设置两个轮子之间的距离

        void set_motor_param(uint8_t id,float per_pluse_distance);//设置电机的速度
        //1.运动学正解：将左右轮速度转换为线速度和角速度
        void kinematics_forward(float left_speed,float right_speed,
                                float* out_linear_speed,float* out_angular_speed);
        //2.运动学逆解：将目标线速度和目标角速度转换为左右轮的目标速度
        void kinematics_inverse(float linear_speed,float angular_speed,
                                float* out_left_speed,float* out_right_speed);
        //更新电机速度和编码器数据
        void update_motor_speed(uint64_t current_time,int32_t left_tick,int32_t right_tick);
        //获取电机速度,返回值是速度
        int16_t get_motor_speed(uint8_t id);


};

#endif