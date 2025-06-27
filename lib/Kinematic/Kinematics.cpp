#include "Kinematics.h"

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

}

//运动学逆解
void Kinematics::kinematics_inverse(float linear_speed,float angular_speed,float* out_left_speed,float* out_right_speed)
{

}

//更新电机的速度
void Kinematics::update_motor_speed(uint64_t current_time,int32_t left_tick,int32_t right_tick)
{

}

//获取电机速度,返回值是速度
int16_t Kinematics::get_motor_speed(uint8_t id)
{
    if(id<0||id>1)
    {
        return -1;
    }
    return motor_param[id].motor_speed;
}