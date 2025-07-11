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
    *out_linear_speed = (left_speed + right_speed) / 2000.0;
    *out_angular_speed = (right_speed - left_speed) / wheel_distance;
}

//运动学逆解
void Kinematics::kinematics_inverse(float linear_speed,float angular_speed,float* out_left_speed,float* out_right_speed)
{
    // 只做一次转换：m/s → mm/s
    float linear_mm_s = linear_speed * 1000.0;

    // // 添加转向辅助：当纯转向时添加基础线速度
    // if (fabs(linear_speed) < 0.01 && fabs(angular_speed) > 0.1) {
    //     linear_mm_s = 50.0; // 50mm/s的基础速度
    // }

    *out_left_speed = linear_mm_s - (angular_speed * wheel_distance) / 2.0;
    *out_right_speed = linear_mm_s + (angular_speed * wheel_distance) / 2.0;
}

//输入：左右轮脉冲数，当前时间；输出：更新的电机速度和编码器数据
void Kinematics::update_motor_speed(uint64_t current_time,int32_t left_tick,int32_t right_tick)
{ 
    //计算时间差
    uint32_t dt = current_time - last_update_time;//单位：ms
    if(dt == 0) return; // 避免除以0

    //计算编码器当前与上一次读取的数值之差
    delta_ticks[0] =  left_tick - motor_param[0].last_encoder_ticks;
    delta_ticks[1] = right_tick - motor_param[1].last_encoder_ticks;
    //更新当前速度
    motor_param[0].motor_speed = delta_ticks[0] * 0.1051566 / dt * 1000;
    motor_param[1].motor_speed = delta_ticks[1] * 0.1051566 / dt * 1000;
    //Serial.printf("left_tick=%d,right_tick=%d\n",delta_ticks[0],delta_ticks[1]);

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
    Serial.printf("motor%d:speed:%d\n",id,motor_param[id].motor_speed);
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
    float dt_s = float(dt) / 1000.0f;//单位：ms->s
    //获取实时的角速度和线速度
    //拿左右轮的实时速度进行运动学正解
    this->kinematics_forward(motor_param[0].motor_speed,motor_param[1].motor_speed,
                            &odom.linear_speed,&odom.angular_speed);
    //odom.linear_speed = odom.linear_speed/1000.0;

    //角度积分
    float delta_angle = odom.angular_speed * dt_s;
    odom.angle += delta_angle;
    TransAngleInPI(odom.angle); // 直接修改原值


    //计算机器人行走的距离
    float delta_distance = odom.linear_speed * dt_s;
    //计算坐标：行走距离分解到xy轴
    odom.x += delta_distance * std::cos(odom.angle);
    odom.y += delta_distance * std::sin(odom.angle);
}