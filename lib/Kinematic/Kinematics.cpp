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
    *out_linear_speed = (left_speed + right_speed) / 2;
    *out_angular_speed = (right_speed - left_speed) / wheel_distance;
}

//运动学逆解
void Kinematics::kinematics_inverse(float linear_speed,float angular_speed,float* out_left_speed,float* out_right_speed)
{
    *out_left_speed = linear_speed - angular_speed * wheel_distance / 2;
    *out_right_speed = linear_speed + angular_speed * wheel_distance / 2;

}

//输入：左右轮脉冲数，当前时间；输出：更新的电机速度和编码器数据
void Kinematics::update_motor_speed(uint64_t current_time,int32_t left_tick,int32_t right_tick)
{ 
    //计算时间差
    int16_t dt = current_time - last_update_time;//单位：ms

    //计算编码器当前与上一次读取的数值之差
    delta_ticks[0] =  left_tick - motor_param[0].last_encoder_ticks;
    delta_ticks[1] = right_tick - motor_param[1].last_encoder_ticks;
    //更新当前速度
    motor_param[0].motor_speed = delta_ticks[0] * 0.166812 / dt * 1000;
    motor_param[1].motor_speed = delta_ticks[1] * 0.166812 / dt * 1000;
    //更新last_tick
    motor_param[0].last_encoder_ticks = left_tick;
    motor_param[1].last_encoder_ticks = right_tick;
    last_update_time = current_time;

    //更新里程计
    update_odom(dt);

    //调用PID获取动态的输出值
    //int temp0 = pid_controller[0].update(current_speed[0]);
    //int temp1 = pid_controller[1].update(current_speed[1]);
    //setMotorSpeed(AIN1, AIN2, PWMA, temp1);
    //setMotorSpeed(BIN1, BIN2, PWMB, temp0);
    //setMotorSpeed(CIN1, CIN2, PWMC, temp1);
    //setMotorSpeed(DIN1, DIN2, PWMD, temp0);
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

odom_t& Kinematics::get_odom(){
    return odom;
}

void Kinematics::TransAngleInPI(float angle,float& out_angle)
{
    if(angle > PI)
    {
        out_angle -= 2*PI;
    }else if (angle < -PI)
    {
        out_angle += 2*PI;
    }
}

void Kinematics::update_odom(uint16_t dt)
{
    float dt_s = float(dt) / 1000.0;//单位：ms->s
    //获取实时的角速度和线速度
    //拿左右轮的实时速度进行运动学正解
    this->kinematics_forward(motor_param[0].motor_speed,motor_param[1].motor_speed,
                            &odom.linear_speed,&odom.angular_speed);
    odom.linear_speed = odom.linear_speed/1000.0;

    //角度积分
    odom.angle += odom.angular_speed * dt_s;
    TransAngleInPI(odom.angle,odom.angle);

    //计算机器人行走的距离
    float delta_distance = odom.linear_speed * dt_s;
    //计算坐标：行走距离分解到xy轴
    odom.x += delta_distance * std::cos(odom.angle);
    odom.y += delta_distance * std::sin(odom.angle);
}