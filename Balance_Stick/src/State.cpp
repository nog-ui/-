#include "State.hpp"

// State.cpp
// 全局变量的定义与初始值。由各模块引用以获取/更新运行时状态。

float ax = 0, gx = 0;                 // IMU 原始读数
float angleX = 0, gyroX = 0;          // 角度和角速度（可作为快捷变量）

float bias = 0;                       // 偏差（目标角 - 测量角）
float integrate = 0;                  // 积分项
int motor_pwm = 0;                    // 计算得到的电机 PWM

bool Gestures_SW = false;            // 手势识别状态
bool fallDown_SW = false;            // 倾倒检测状态

float compAngle = 0;                 // 互补滤波输出角度
unsigned long lastTime = 0;          // 上次滤波更新时间

float gyroX_lp = 0.0f;               // 陀螺低通滤波器状态

// 默认参数
const int default_deadzone = 46;
int deadzone = default_deadzone;

const float default_kp = 120;
const float default_ki = 0.060;
const float default_kd = 68.80;
const float default_balanceAngle = 0;

// 运行时 PID 参数（可以由 Web 界面修改并保存）
float kp = default_kp;
float ki = default_ki;
float kd = default_kd;
float keepAngle = default_balanceAngle;

// 速度环默认参数（串级控制内环）
const float default_vkp = 0.8;
const float default_vki = 0.0;
const float default_vkd = 0.1;

// 速度环运行时参数
float vkp = default_vkp;
float vki = default_vki;
float vkd = default_vkd;
float targetGyro = 0.0f;      // 目标角速度（由位置环输出）
float vintegrate = 0.0f;      // 速度环积分项
bool cascadeEnabled = false;  // 串级控制使能开关（默认关闭，保持单环兼容）

// 电机引脚默认（可按实际硬件调整）
int motor_pin_a = 7;
int motor_pin_b = 10;
int motor_pin_c = -1;
int motor_pin_d = -1;

// LEDC 默认配置
const int default_ledc_freq = 5000;
const int default_ledc_bit = 8;
int ledc_freq = default_ledc_freq;
int ledc_bit = default_ledc_bit;
int ledc_channel_a = 0;
int ledc_channel_b = 1;

// 电机方向反转（默认不反转）
bool motor_inverted = false;
// 手动控制状态（用于 /deadzone 页面）
bool manual_control_enabled = false;
int manual_pwm = 0;
// 实际应用到电机的 PWM（由 MotorControl 更新，用于显示）
int applied_pwm = 0;
