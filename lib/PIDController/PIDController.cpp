#include "PIDController.h"
#include <Arduino.h>

//构造函数
PIDController::PIDController(float kp,float ki,float kd){
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

//float update(float current)//提供当前值，返回下次输出值，也就是PID的结果
//void uptate_target(float target);//更新目标值
//void update_pid(float kp,float ki,float kd);//更新PID的参数
//void reset();//重置PID
//void out_limit(float );//限制输出范围

float PIDController::update(float current){

    error_ = target_ - current;//计算error。目标值值减去当前值
    error_sum_ += error_;//对error进行积分

    //限制积分范围
    if(error_sum_ > intergral_up_) error_sum_ = intergral_up_;
    if(error_sum_ < -intergral_up_) error_sum_ = -1*intergral_up_;

    derror_ = prev_error_ - error_;//计算误差变化率
    prev_error_ = error_;//更新上一次误差

    //计算PID结果
    float output = kp_ * error_ + ki_ * error_sum_ + kd_ * derror_;
    //限制PID结果范围
    if(output > out_max_) output = out_max_;
    if(output < out_min_) output = out_min_;

    //  // 添加静摩擦补偿
    // if (fabs(output) > 0 && fabs(output) < friction_compensation_) {
    //     output = (output > 0) ? friction_compensation_ : -friction_compensation_;
    // }
    
    // 减少调试输出频率
    static int count = 0;
    if (count++ > 100) {
        count = 0;
        Serial.printf("output=%f\n",output);
    }

    Serial.printf("output=%f\n",output);
    return output;

}

void PIDController::uptate_target(float target){
    target_ = target;
}

void PIDController::update_pid(float kp,float ki,float kd){
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PIDController::reset(){
    error_ = 0;
    error_sum_ = 0;
    derror_ = 0;
    prev_error_ = 0;
    kp_ = 0;
    ki_ = 0;
    kd_ = 0;
    intergral_up_ = 2500;
    out_max_ = 0;
    out_min_ = 0;
}

void PIDController::out_limit(float min,float max){
    out_min_ = min;
    out_max_ = max;
}