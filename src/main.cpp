#include <Arduino.h>
#include <Esp32PcntEncoder.h>
#include "PIDController.h"
#include "Kinematics.h"

//引入micro-ROS和wifi相关库
#include <wifi.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
//引入字符串内存分配管理工具
#include <micro_ros_utilities/string_utilities.h>

//引入消息接口
#include <geometry_msgs/msg/twist.h>//消息接口
rcl_subscription_t sub_cmd_vel;//创建一个消息的订阅者
geometry_msgs__msg__Twist msg_cmd_vel;//订阅到的数据存储在这里

//引入里程计消息接口
#include <nav_msgs/msg/odometry.h>
rcl_publisher_t pub_odom;//创建一个里程计发布者
nav_msgs__msg__Odometry msg_odom;//里程计消息存储到这里
rcl_timer_t timer;//定时器，定时调用某个函数




Esp32PcntEncoder encoders[2]; // 创建一个数组用于存储两个编码器
PIDController pid_controller[2];//创建一个数组用于PID控制
Kinematics kinematics;

float target_linear_speed = 50.0; //单位 mm/s
float target_angular_speed = 0.9; //单位 弧度/s
float out_left_speed = 0.0;       //输出的是左右轮速度，不是反馈的左右轮速度
float out_right_speed = 0.0;

//定时器回调函数
void timer_callback(rcl_timer_t* timer,int64_t last_call_time)
{
  //完成里程计的发布
  odom_t odom = kinematics.get_odom();//获取当前的里程计
  int64_t stamp = rmw_uros_epoch_millis();//获取当前的时间
  msg_odom.header.stamp.sec = static_cast<int32_t>(stamp/1000);//秒部分
  msg_odom.header.stamp.nanosec = static_cast<int32_t>((stamp%1000)*1e6);//纳秒部分
  msg_odom.pose.pose.position.x = odom.x;
  msg_odom.pose.pose.position.y = odom.y;
  msg_odom.pose.pose.orientation.w = cos(odom.angle*0.5);
  msg_odom.pose.pose.orientation.x = 0;
  msg_odom.pose.pose.orientation.y = 0;
  msg_odom.pose.pose.orientation.z = sin(odom.angle*0.5);
  msg_odom.twist.twist.linear.x = odom.linear_speed;
  msg_odom.twist.twist.angular.z = odom.angular_speed;//修复
  //发布里程计，把数据发出去
  if(rcl_publish(&pub_odom,&msg_odom,NULL)!=RCL_RET_OK)
  {
    Serial.println("error:odom pub failed");
  }

}

//话题的回调函数
void twist_callback(const void* msg_in)
{
  //将收到的消息指针转换成geometry_msgs_Twist类型的指针
  const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist*)msg_in;
  target_linear_speed = msg->linear.x * 1000;
  target_angular_speed = msg->angular.z;

  //测试运动学逆解
  kinematics.kinematics_inverse(target_linear_speed,target_angular_speed,
                                &out_left_speed,&out_right_speed);
  Serial.printf("OUT:left_speed=%f,right_speed=%f\n",out_left_speed,out_right_speed);
  pid_controller[0].uptate_target(out_left_speed);
  pid_controller[1].uptate_target(out_right_speed);


}

//声明一些相关的结构体对象
rcl_allocator_t allocator;//内存分配器。用于动态内存分配管理
rclc_support_t support;//用于存储时钟，内存分配器和上下文，用于提供支持
rclc_executor_t executor;//执行器，用于管理订阅和计时器回调的执行
rcl_node_t node;        //结点，用于创建结点

//单独创建一个任务运行micro-ROS 相当于一个线程
void microros_task(void* args)
{
  //1.设置传输协议并延迟一段时间等待设置的完成
  IPAddress agent_ip;
  agent_ip.fromString("192.168.0.134");//设置上位机的地址
  set_microros_wifi_transports("沙河汤臣一品","20050202",agent_ip,8888);//设置esp32连接的wifi以及要发送数据的上位机的ip和端口号
  delay(2000);//等待wifi连接
  //2.初始化内存分配器
  allocator = rcl_get_default_allocator();//获取默认的内存分配器
  //3.初始化支持模块
  rclc_support_init(&support,0,NULL,&allocator);//初始化支持
  //4.初始化结点
  rclc_node_init_default(&node,"robot_motion_control","",&support);
  //5.初始化执行器
  unsigned int num_handles = 2;//订阅和计时器的数量，这是一个需要更改的参数
  rclc_executor_init(&executor,&support.context,num_handles,&allocator);
  //6.初始化订阅者，并将其添加到执行器中
  rclc_subscription_init_best_effort(&sub_cmd_vel,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs,msg,Twist),"/cmd_vel");
  rclc_executor_add_subscription(&executor,&sub_cmd_vel,&msg_cmd_vel,&twist_callback,ON_NEW_DATA);
  //7.初始化msg
  msg_odom.header.frame_id = micro_ros_string_utilities_set(msg_odom.header.frame_id,"odom");
  msg_odom.child_frame_id = micro_ros_string_utilities_set(msg_odom.child_frame_id,"base_footprint");
  //8.初始化发布者和定时器
  rclc_publisher_init_best_effort(&pub_odom,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs,msg,Odometry),"/odom");
  rclc_timer_init_default(&timer,&support,RCL_MS_TO_NS(50),timer_callback);
  rclc_executor_add_timer(&executor,&timer);
  //9.时间同步
  while(!rmw_uros_epoch_synchronized())
  {
    rmw_uros_sync_session(1000);
    delay(10);
  }

  //循环执行
  rclc_executor_spin(&executor);
}





// 电机控制引脚定义
#define AIN1 14
#define AIN2 27
#define PWMA 12
#define BIN1 26
#define BIN2 25
#define PWMB 13

// 第二个TB6612模块（控制电机C和D）
#define CIN1 18
#define CIN2 19
#define PWMC 5
#define DIN1 17
#define DIN2 16
#define PWMD 23
//速度上限
#define Speed_Limit 200



void setMotorSpeed(int in1, int in2, int pwm, int speed);
void moveForward(int speed);
void moveBackward(int speed);
void turnLeft(int speed);
void turnRight(int speed);
void stopMotors();




void setup()
{
  // 初始化串口
  Serial.begin(115200); // 初始化串口通信，设置通信速率为115200

  // 初始化电机控制引脚
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(CIN1, OUTPUT);
  pinMode(CIN2, OUTPUT);
  pinMode(PWMC, OUTPUT);
  pinMode(DIN1, OUTPUT);
  pinMode(DIN2, OUTPUT);
  pinMode(PWMD, OUTPUT);


  // 初始化编码器
  encoders[0].init(0, 33, 32); // 初始化第一个编码器，A1=33，B1=32
  encoders[1].init(1, 2, 4); // 初始化第二个编码器，A2=2，B2=4

  //初始化PID控制器
  pid_controller[0].update_pid(0.625,0.125,0.0);
  pid_controller[1].update_pid(0.625,0.125,0.0);
  pid_controller[0].out_limit(-Speed_Limit,Speed_Limit);
  pid_controller[1].out_limit(-Speed_Limit,Speed_Limit);
  //初始化运动学参数
  kinematics.set_wheel_distance(175);
  kinematics.set_motor_param(0,0.1051566);
  kinematics.set_motor_param(1,0.1051566);
  
  msg_odom.pose.pose.orientation.x = 0;
  msg_odom.pose.pose.orientation.y = 0;
  msg_odom.pose.pose.orientation.z = 0;
  msg_odom.pose.pose.orientation.w = 1;

  //创建一个任务来启动micro-ros的task
  xTaskCreate(microros_task,"micros_task",10240,NULL,1,NULL);
                          
}

void loop()
{
  delay(10); // 等待10毫秒
  kinematics.update_motor_speed(millis(),encoders[0].getTicks(),encoders[1].getTicks());
  int temp0 = pid_controller[0].update(kinematics.get_motor_speed(0));
  int temp1 = pid_controller[1].update(kinematics.get_motor_speed(1));
  setMotorSpeed(AIN1, AIN2, PWMA, temp1);
  setMotorSpeed(BIN1, BIN2, PWMB, temp0);
  setMotorSpeed(CIN1, CIN2, PWMC, temp1);
  setMotorSpeed(DIN1, DIN2, PWMD, temp0);   
  // 打印两个电机的速度
  //Serial.printf("speed1=%f,speed2=%f\n",current_speed[0],current_speed[1]);
  Serial.printf("x,y,yaw=%f,%f,%f\n",kinematics.get_odom().x,kinematics.get_odom().y,
                                     kinematics.get_odom().angle);
}

void setMotorSpeed(int in1, int in2, int pwm, int speed) {
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  analogWrite(pwm, abs(speed));
}

void moveForward(int speed) {
  setMotorSpeed(AIN1, AIN2, PWMA, speed);
  setMotorSpeed(BIN1, BIN2, PWMB, speed);
  setMotorSpeed(CIN1, CIN2, PWMC, speed);
  setMotorSpeed(DIN1, DIN2, PWMD, speed);
}

void moveBackward(int speed) {
  setMotorSpeed(AIN1, AIN2, PWMA, -speed);
  setMotorSpeed(BIN1, BIN2, PWMB, -speed);
  setMotorSpeed(CIN1, CIN2, PWMC, -speed);
  setMotorSpeed(DIN1, DIN2, PWMD, -speed);
}

void turnLeft(int speed) {
  setMotorSpeed(AIN1, AIN2, PWMA, speed);
  setMotorSpeed(BIN1, BIN2, PWMB, -speed);
  setMotorSpeed(CIN1, CIN2, PWMC, speed);
  setMotorSpeed(DIN1, DIN2, PWMD, -speed);
}

void turnRight(int speed) {
  setMotorSpeed(AIN1, AIN2, PWMA, -speed);
  setMotorSpeed(BIN1, BIN2, PWMB, speed);
  setMotorSpeed(CIN1, CIN2, PWMC, -speed);
  setMotorSpeed(DIN1, DIN2, PWMD, speed);
}

void stopMotors() {
  setMotorSpeed(AIN1, AIN2, PWMA, 0);
  setMotorSpeed(BIN1, BIN2, PWMB, 0);
  setMotorSpeed(CIN1, CIN2, PWMC, 0);
  setMotorSpeed(DIN1, DIN2, PWMD, 0);
}

