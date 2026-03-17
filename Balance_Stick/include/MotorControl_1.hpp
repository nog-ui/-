#pragma once

// MotorControl.hpp
// 对外提供简单电机控制封装：初始化、运行、熄火。

// 初始化电机硬件（配置 PWM/引脚等）
void initMotor();

// 根据 pwm 值驱动电机（范围 -255..255 或更宽，取决于实现）
void runMotor(int pwm);

// 熄火，将电机驱动置为安全状态
void flameoutMotor();
