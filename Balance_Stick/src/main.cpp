#include <Arduino.h>
#include <WiFi.h>
#include "esp_wifi.h"

#include "State.hpp"
#include "IMU.hpp"
#include "PID.hpp"
#include "MotorControl.hpp"
#include "Config.hpp"
#include "WebServer.hpp"

// main.cpp
// 程序入口，负责全局模块初始化并在主循环中驱动各模块。

// ================== WiFi AP ==================
const char* AP_SSID = "ESP32C3_Balance";
const char* AP_PSK  = "";

// setup
// 初始化顺序：串口 -> AP -> IMU -> 参数 -> 电机 -> Web
void setup(){
  Serial.begin(115200);

  WiFi.softAP(AP_SSID, AP_PSK);
  esp_wifi_set_max_tx_power(60);

  initIMU();       // 初始化传感器
  loadPrefs();     // 加载已保存参数
  initMotor();     // 初始化电机与指示灯
  initWeb();       // 启动 Web 服务（HTTP + WS）

  flameoutMotor(); // 确保电机处于安全状态
  Gestures_SW = false;

  // 指示初始化完成（点亮 LED）
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
  digitalWrite(13, HIGH);
}

// loop
// 周期性读取传感器、计算 PID、驱动电机并推送状态到客户端
void loop(){
  static unsigned long lastSend = 0;

  get_mpu6050_val();
  pwm_calculation();
  motor_pwm = applyDeadzoneComp(motor_pwm);
  runMotor(motor_pwm);
  led_effect();

  webCleanup();

  if (millis() - lastSend > 100) {
    sendStatusToClients();
    lastSend = millis();
  }
}
