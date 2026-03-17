#include "MotorControl.hpp"
#include "Motor.hpp"
#include "State.hpp"

// MotorControl.cpp
// 统一管理项目中的 Motor 实例，提供对外初始化和运行接口。

static Motor* motor = nullptr;

// initMotor
// 配置指示灯引脚并根据状态中的引脚配置构造 Motor 实例
void initMotor(){
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);

  // 如果已有实例，先熄火并删除
  if (motor) {
    motor->flameout();
    delete motor;
    motor = nullptr;
  }

  motor = new Motor(motor_pin_a, motor_pin_b, motor_pin_c, motor_pin_d);
  if (motor) motor->flameout();
}

// runMotor
// 将计算出的 pwm 值转发到底层 Motor 实现
void runMotor(int pwm){
  if (!motor) return;

  // 手动控制优先（用于死区测试）
  if (manual_control_enabled) {
    int m = manual_pwm;
    if (motor_inverted) m = -m;
    applied_pwm = m;
    motor->run(m);
    return;
  }

  // 根据配置做反转支持（物理接反时可用）
  int out = motor_inverted ? -pwm : pwm;
  applied_pwm = out;
  motor->run(out);
}

// flameoutMotor
// 安全熄火接口
void flameoutMotor(){
  if (motor) motor->flameout();
}
