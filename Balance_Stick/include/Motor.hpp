#pragma once
#include <Arduino.h>

// Motor.hpp
// 最小化的 Motor 类接口（占位实现）。如果有更完整的驱动库，应替换此文件。

class Motor {
public:
  // 构造：传入控制引脚（实现可忽略不使用的参数）
  Motor(int pinA, int pinB, int pinC, int pinD);

  // 运行电机：pwm 可为负表示反向
  void run(int pwm);

  // 熄火/安全停止
  void flameout();

private:
  int _a, _b, _c, _d;
  int _last_pwm;
};
