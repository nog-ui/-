#include <Arduino.h>
#include "PID.hpp"
#include "IMU.hpp"
#include "State.hpp"
#include "MotorControl.hpp"

// pwm_calculation
// 执行 PID 相关计算：读取 IMU 状态、更新积分项、计算 motor_pwm
// 支持两种模式：
//   - 单环模式（默认）：位置PID直接输出PWM
//   - 串级模式：位置环输出目标角速度，速度环输出PWM
void pwm_calculation() {
  // 更新手势与倾倒检测状态
  detect_gestures();
  detect_fall_down();

  // 若检测到倾倒或手势不允许操作，则清零并退出
  if ((!fallDown_SW) || (!Gestures_SW)) {
    motor_pwm = 0;
    integrate = 0;
    bias = 0;
    targetGyro = 0;
    vintegrate = 0;
    return;
  }

  // 使用全局的最新 IMU 值进行计算
  angleX = ax; // 加速度推导角度
  gyroX  = gx; // 陀螺速率

  // 对陀螺进行低通滤波以获得平滑的 D 项
  const float d_alpha = 0.70f;
  gyroX_lp = d_alpha * gyroX_lp + (1.0f - d_alpha) * gyroX;

  // 卡尔曼滤波融合角度（最优估计）
  // 如需切换回互补滤波，将 kalmanFilter 改为 complementaryFilter
  float fAngle = kalmanFilter(angleX, gyroX);

  bias = fAngle - keepAngle;

  // 积分限幅以避免饱和
  integrate += bias;
  integrate = constrain(integrate, -200, 200);

  if (cascadeEnabled) {
    // ========== 串级控制模式 ==========
    
    // 外环（位置环）：计算目标角速度
    // 位置PID输出作为速度环的设定值
    targetGyro = kp * bias + ki * integrate;
    // 限制目标角速度范围，防止过大
    targetGyro = constrain(targetGyro, -200, 200);
    
    // 内环（速度环）：根据角速度误差计算PWM
    float gyroError = targetGyro - gyroX_lp;
    
    // 速度环积分限幅
    vintegrate += gyroError;
    vintegrate = constrain(vintegrate, -100, 100);
    
    // 速度环PID计算
    motor_pwm = vkp * gyroError + vki * vintegrate + vkd * (gyroError - bias);
    
  } else {
    // ========== 单环控制模式（默认，保持兼容） ==========
    
    // PID 线性组合
    motor_pwm = kp * bias
              + ki * integrate
              + kd * gyroX_lp;
  }
}

// LED 显示引脚（与电机无关，仅用于状态指示）
const int ledpin1 = 13;
const int ledpin2 = 12;

// led_effect
// 根据 `motor_pwm` 的方向改变对应 LED 的占空比，作为视觉反馈
void led_effect() {
  int ledPWM = abs(motor_pwm);
  if (motor_pwm < 0) analogWrite(ledpin1, ledPWM);
  else analogWrite(ledpin2, ledPWM);
}

// applyDeadzoneComp
// 根据配置的 `deadzone` 对输出进行补偿，保证小幅输出能驱动电机
int applyDeadzoneComp(int u){
  const int U_MAX = 255;

  if (u == 0) return 0;

  int s = (u > 0) ? 1 : -1;
  int a = abs(u);

  // 如果开启死区补偿且输出在死区以内，将其拉到死区边界
  if (deadzone > 0 && a < deadzone) a = deadzone;

  if (a > U_MAX) a = U_MAX;
  return s * a;
}
