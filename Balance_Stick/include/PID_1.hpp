#pragma once

// PID.hpp
// PID 控制与显示效果相关的接口：计算电机输出、LED 指示、死区补偿。

// 计算 PID 输出并更新全局 `motor_pwm`
void pwm_calculation();

// 根据当前 PWM 更新 LED 指示效果（视觉反馈）
void led_effect();

// 应用死区补偿，输入/输出为 -255..255（或更宽）
int applyDeadzoneComp(int u);
