#include "Motor.hpp"
#include "State.hpp"

// Motor.cpp
// 基于 ESP32 LEDC 的 PWM 实现

Motor::Motor(int pinA, int pinB, int pinC, int pinD)
 : _a(pinA), _b(pinB), _c(pinC), _d(pinD), _last_pwm(0)
{
  // 配置 LEDC（可能多次调用也安全）
  ledcSetup(ledc_channel_a, ledc_freq, ledc_bit);
  ledcSetup(ledc_channel_b, ledc_freq, ledc_bit);

  // 绑定引脚到对应通道
  if (_a >= 0) ledcAttachPin(_a, ledc_channel_a);
  if (_b >= 0) ledcAttachPin(_b, ledc_channel_b);

  if (_c >= 0) pinMode(_c, OUTPUT);
  if (_d >= 0) pinMode(_d, OUTPUT);

  // 初始停止
  ledcWrite(ledc_channel_a, 0);
  ledcWrite(ledc_channel_b, 0);
}

// run
// pwm 期望在 -255..255；映射到 LEDC 分辨率
void Motor::run(int pwm){
  _last_pwm = pwm;
  int max_duty = (1 << ledc_bit) - 1;

  long abs_pwm = pwm >= 0 ? pwm : -pwm;
  if (abs_pwm > 255) abs_pwm = 255;
  int duty = (int)((abs_pwm * max_duty) / 255);

  if (pwm >= 0) {
    if (_a >= 0) ledcWrite(ledc_channel_a, duty);
    if (_b >= 0) ledcWrite(ledc_channel_b, 0);
  } else {
    if (_a >= 0) ledcWrite(ledc_channel_a, 0);
    if (_b >= 0) ledcWrite(ledc_channel_b, duty);
  }
}

// flameout
// 将所有输出置为安全状态
void Motor::flameout(){
  if (_a >= 0) ledcWrite(ledc_channel_a, 0);
  if (_b >= 0) ledcWrite(ledc_channel_b, 0);
  if (_c >= 0) digitalWrite(_c, LOW);
  if (_d >= 0) digitalWrite(_d, LOW);
  _last_pwm = 0;
}
