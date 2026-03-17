#pragma once

// State.hpp
// 全局状态变量声明：IMU 读数、PID 参数、运行时中间量等。
// 这些变量由 `State.cpp` 定义并在各模块间共享。

#include <Arduino.h>

// IMU 角度与陀螺数据（由 IMU 模块更新）
extern float ax;   // 加速度计计算的角度值（X 轴）
extern float gx;   // 陀螺仪角速度（X 轴）
extern float angleX; // 当前角度（如需要可与 ax 区分）
extern float gyroX;  // 当前陀螺值（低层使用）

// 控制与中间量
extern float bias;      // 参考角与测量角之差
extern float integrate; // 积分项
extern int motor_pwm;   // 当前计算出的 PWM 值
extern int applied_pwm; // 实际输出给电机的 PWM（可能为 manual_pwm 或 motor_pwm）

// 状态开关
extern bool Gestures_SW; // 手势使能开关
extern bool fallDown_SW; // 倾倒检测开关

// 互补滤波与定时
extern float compAngle;      // 互补滤波输出角度
extern unsigned long lastTime; // 上次时间戳（ms）

// 低通滤波器状态
extern float gyroX_lp;

// 参数默认值与运行时参数
extern const int default_deadzone; // 死区默认值
extern int deadzone;               // 当前死区设置

extern const float default_kp;
extern const float default_ki;
extern const float default_kd;
extern const float default_balanceAngle;

extern float kp;
extern float ki;
extern float kd;
extern float keepAngle;

// 电机硬件配置（可根据板子改动）
extern int motor_pin_a; // 正向 PWM 或 H 桥输入 A
extern int motor_pin_b; // 反向 PWM 或 H 桥输入 B
extern int motor_pin_c; // 可选：使能/方向引脚 C
extern int motor_pin_d; // 可选：使能/方向引脚 D

// LEDC 配置（可修改以匹配硬件）
extern const int default_ledc_freq;
extern const int default_ledc_bit;
extern int ledc_freq;
extern int ledc_bit;
extern int ledc_channel_a;
extern int ledc_channel_b;

// 电机方向反转配置（true 表示反转）
extern bool motor_inverted;
// 手动控制用于调试死区：当为 true 时使用 manual_pwm 驱动电机
extern bool manual_control_enabled;
extern int manual_pwm;
