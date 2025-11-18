#ifndef __PID_CONTROLLER_H__
#define __PID_CONTROLLER_H__

class PIDController{

    public:
        PIDController() = default;
        PIDController(float kp,float ki,float kd);
    
    private:
        float target_;
        float out_min_;//输出最小值
        float out_max_;//输出最大值
        float kp_;
        float ki_;
        float kd_;
        //pid中间值
        float error_;//误差
        float error_sum_;//误差积分
        float derror_;//误差变化率
        float prev_error_;//上一次的误差
        // float friction_compensation_ = 30.0; // 静摩擦补偿值
        // 新增：积分限制参数
        float integral_threshold_ = 50.0f;  // 积分生效的误差阈值
        float integral_max_ = 1000.0f;      // 积分上限
        
        // 新增：静摩擦补偿
        float friction_compensation_ = 30.0f;



    public:
       float update(float current);//提供当前值，返回下次输出值，也就是PID的结果
       void update_target(float target);//更新目标值
       void update_pid(float kp,float ki,float kd);//更新PID的参数
       void reset();//重置PID
       void out_limit(float min,float max);//限制输出范围
       // 新增方法
       void set_integral_limits(float threshold, float max);
       void set_friction_compensation(float compensation);
       // 新增：获取目标值用于调试
       float get_target() const { return target_; }
       void clearCount();



};

#endif