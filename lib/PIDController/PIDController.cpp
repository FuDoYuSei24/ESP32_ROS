#include "PIDController.h"
#include <Arduino.h>

//构造函数
PIDController::PIDController(float kp,float ki,float kd){
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    reset();
}


float PIDController::update(float current){

    // // 死区处理：当目标速度接近0且当前速度很小时，直接返回0
    // if (fabs(target_) < 1.0f && fabs(current) < 5.0f) {
    //     error_sum_ = 0; // 重置积分项
    //     derror_ = 0;
    //     prev_error_ = 0;
    //     return 0;
    // }
    error_ = target_ - current;//计算error。目标值值减去当前值
    
   // 抗积分饱和：只在误差较小时积分
    if (fabs(error_) < integral_threshold_) {
        error_sum_ += error_;
        // 积分限幅
        if (error_sum_ > integral_max_) error_sum_ = integral_max_;
        if (error_sum_ < -integral_max_) error_sum_ = -integral_max_;
    } else {
        error_sum_ = 0; // 大误差时清零积分，防止饱和
    }


    derror_ = prev_error_ - error_;//计算误差变化率
    prev_error_ = error_;//更新上一次误差

    //计算PID结果
    float output = kp_ * error_ + ki_ * error_sum_ + kd_ * derror_;

     // 静摩擦补偿
    if (fabs(target_) > 0.1f && fabs(output) < friction_compensation_) {
        output = (output > 0) ? friction_compensation_ : -friction_compensation_;
    }

    //限制PID结果范围
    if(output > out_max_) output = out_max_;
    if(output < out_min_) output = out_min_;

    
    // 减少调试输出频率
    static int count = 0;
    if (count++ > 100) {
        count = 0;
        //Serial.printf("output=%f\n",output);
    }

    //Serial.printf("output=%f\n",output);
    return output;

}

void PIDController::update_target(float target){
    target_ = target;

    // 目标为0时重置积分，防止停止时积分饱和
    if (fabs(target_) < 0.1f) {
        error_sum_ = 0;
    }
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
    out_max_ = 0;
    out_min_ = 0;

}

void PIDController::out_limit(float min,float max){
    out_min_ = min;
    out_max_ = max;
}

// 新增方法：设置积分限制和摩擦补偿
void PIDController::set_integral_limits(float threshold, float max) {
    integral_threshold_ = threshold;
    integral_max_ = max;
}

void PIDController::set_friction_compensation(float compensation) {
    friction_compensation_ = compensation;
}

void PIDController::clearCount() {
    //清除编码器的值
    
}