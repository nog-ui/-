#pragma once

// IMU.hpp
// 封装 MPU6050 的初始化与数据读取，提供互补滤波及简单手势/倾倒检测接口。

// 初始化 IMU（初始化 I2C、MPU 底层并计算陀螺零偏，配置硬件滤波器）
void initIMU();

// 读取并更新全局 IMU 值（ax/gx），应用低通滤波和异常值检测
void get_mpu6050_val();

// 改进的互补滤波：融合加速度角度与陀螺角速度，动态调整滤波系数
float complementaryFilter(float accelAngle, float gyroRate);

// 卡尔曼滤波器：最优融合加速度计和陀螺仪数据（推荐用于高精度应用）
float kalmanFilter(float accelAngle, float gyroRate);

// 倾倒检测（更新 `fallDown_SW`）
void detect_fall_down();

// 手势检测（更新 `Gestures_SW`）
void detect_gestures();
