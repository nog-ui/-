#include <Wire.h>
#include <MPU6050_tockn.h>
#include "IMU.hpp"
#include "State.hpp"

// IMU.cpp
// 负责与 MPU6050 的交互，提供原始读数与改进的滤波功能。

static MPU6050 mpu6050(Wire);

// 低通滤波器状态变量
static float ax_filtered = 0.0f;     // 加速度计角度低通滤波输出
static float gx_filtered = 0.0f;     // 陀螺仪角速度低通滤波输出
static float last_ax = 0.0f;         // 上一次的加速度计角度
static float last_gx = 0.0f;         // 上一次的陀螺仪角速度

// 滤波器参数
const float ACCEL_LPF_ALPHA = 0.15f;  // 加速度计低通滤波系数（越小越平滑）
const float GYRO_LPF_ALPHA = 0.3f;    // 陀螺仪低通滤波系数
const float OUTLIER_THRESHOLD = 30.0f; // 异常值阈值（度）

// ========== 卡尔曼滤波器状态变量 ==========
// 状态向量: [角度, 陀螺偏差]
static float kalman_angle = 0.0f;      // 卡尔曼滤波估计的角度
static float kalman_bias = 0.0f;       // 陀螺仪偏差估计

// 状态协方差矩阵 P (2x2)
static float P[2][2] = {
  {1.0f, 0.0f},
  {0.0f, 1.0f}
};

// 过程噪声协方差矩阵 Q (2x2)
// Q[0][0]: 角度过程噪声 - 角度的不确定性增长速度
// Q[1][1]: 陀螺偏差过程噪声 - 偏差的随机游走
const float Q_angle = 0.001f;   // 角度过程噪声（较小=更信任模型）
const float Q_bias = 0.003f;    // 陀螺偏差过程噪声

// 测量噪声协方差 R
// 加速度计测量噪声（越大越不信任加速度计）
const float R_measure = 0.3f;

// 初始化 I2C 与 MPU6050，计算陀螺偏置，配置硬件滤波器
void initIMU(){
  Wire.begin(4, 5);
  mpu6050.begin();
  
  // 配置 MPU6050 硬件数字低通滤波器（DLPF）
  // DLPF_CFG = 3: 带宽 ~44Hz（加速度计）/42Hz（陀螺仪），延迟 4.9ms
  // 这可以有效减少高频噪声
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);  // CONFIG 寄存器
  Wire.write(0x03);  // DLPF_CFG = 3
  Wire.endTransmission();
  
  mpu6050.calcGyroOffsets(true);
  lastTime = millis();
  
  // 初始化滤波器状态
  mpu6050.update();
  ax_filtered = mpu6050.getAngleX();
  gx_filtered = mpu6050.getGyroX();
  last_ax = ax_filtered;
  last_gx = gx_filtered;
}

// 读取传感器数据并更新全局变量 `ax`、`gx`
// 应用低通滤波和异常值检测
void get_mpu6050_val() {
  mpu6050.update();
  
  // 读取原始数据
  float ax_raw = mpu6050.getAngleX();
  float gx_raw = mpu6050.getGyroX();
  
  // 异常值检测：如果变化超过阈值，则拒绝该读数
  if(abs(ax_raw - last_ax) < OUTLIER_THRESHOLD) {
    // 加速度计低通滤波（一阶IIR滤波器）
    ax_filtered = ACCEL_LPF_ALPHA * ax_raw + (1.0f - ACCEL_LPF_ALPHA) * ax_filtered;
    last_ax = ax_raw;
  }
  // 如果是异常值，保持上一次的滤波值
  
  // 陀螺仪低通滤波
  if(abs(gx_raw - last_gx) < 500.0f) { // 陀螺异常阈值：500度/秒
    gx_filtered = GYRO_LPF_ALPHA * gx_raw + (1.0f - GYRO_LPF_ALPHA) * gx_filtered;
    last_gx = gx_raw;
  }
  
  // 更新全局变量为滤波后的值
  ax = ax_filtered;
  gx = gx_filtered;
  gyroX_lp = gx_filtered;  // 同时更新低通滤波陀螺值
}

// 改进的互补滤波：融合加速度计角度与陀螺角速度
// 动态调整 alpha 系数以适应不同运动状态
float complementaryFilter(float accelAngle, float gyroRate) {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  // 防止dt过大（系统刚启动或长时间未调用）
  if(dt > 0.1f) dt = 0.01f;

  // 动态调整 alpha：根据角速度大小决定信任陀螺还是加速度计
  // 快速运动时更信任陀螺仪，静止时更信任加速度计
  float alpha;
  float absGyroRate = abs(gyroRate);
  
  if(absGyroRate < 5.0f) {
    // 几乎静止：增加加速度计权重，快速收敛
    alpha = 0.96f;
  } else if(absGyroRate < 50.0f) {
    // 正常运动：标准权重
    alpha = 0.985f;
  } else {
    // 快速运动：更信任陀螺仪
    alpha = 0.995f;
  }
  
  // 互补滤波计算
  compAngle = alpha * (compAngle + gyroRate * dt)
            + (1 - alpha) * accelAngle;

  return compAngle;
}

// ========== 卡尔曼滤波器实现 ==========
// 基于一维卡尔曼滤波器，融合加速度计角度和陀螺仪角速度
// 状态: [角度, 陀螺偏差]，测量: 加速度计角度
float kalmanFilter(float accelAngle, float gyroRate) {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;
  
  // 防止dt过大
  if(dt > 0.1f || dt <= 0.0f) dt = 0.01f;
  
  // ===== 第1步：预测（先验估计）=====
  // 状态预测：angle = angle + (gyroRate - bias) * dt
  kalman_angle += (gyroRate - kalman_bias) * dt;
  
  // 协方差预测：P = P + Q
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;
  
  // ===== 第2步：更新（后验估计）=====
  // 计算卡尔曼增益
  float S = P[0][0] + R_measure;        // 创新协方差
  float K[2];                           // 卡尔曼增益
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;
  
  // 计算创新（测量残差）
  float innovation = accelAngle - kalman_angle;
  
  // 更新状态估计
  kalman_angle += K[0] * innovation;
  kalman_bias += K[1] * innovation;
  
  // 更新协方差矩阵
  float P00_temp = P[0][0];
  float P01_temp = P[0][1];
  
  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;
  
  // 返回滤波后的角度
  return kalman_angle;
}

// 倾倒检测：根据 ax 值判断是否处于有效工作角度范围
void detect_fall_down() {
  fallDown_SW = ((ax > -35) && (ax < 35));
}

// 简单手势检测：依赖 Y 轴陀螺阈值
void detect_gestures() {
  float gyroY = mpu6050.getGyroY();
  if (gyroY > 450)      Gestures_SW = true;
  else if (gyroY < -450) Gestures_SW = false;
}
