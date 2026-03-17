#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include "WebServer.hpp"
#include "WebContent.hpp"
#include "State.hpp"
#include "Config.hpp"

// WebServer.cpp
// 封装 Web 服务（HTTP + WebSocket），负责接收前端命令并向所有客户端广播状态。

static AsyncWebServer server(80);
static AsyncWebSocket ws("/ws");

// sendStatusToClients
// 将当前关键状态打包为 JSON 字符串并广播给所有 WebSocket 客户端
void sendStatusToClients() {
  StaticJsonDocument<512> doc;
  doc["angle"] = angleX;
  doc["cangle"] = compAngle;
  doc["kp"] = kp;
  doc["ki"] = ki;
  doc["kd"] = kd;
  doc["keepAngle"] = keepAngle;
  doc["deadzone"] = deadzone;

  doc["pwm"] = motor_pwm;
  doc["bias"] = bias;
  doc["integrate"] = integrate;
  doc["applied_pwm"] = applied_pwm;
  doc["manual_enabled"] = manual_control_enabled;
  
  // 速度环参数（串级控制）
  doc["vkp"] = vkp;
  doc["vki"] = vki;
  doc["vkd"] = vkd;
  doc["targetGyro"] = targetGyro;
  doc["vintegrate"] = vintegrate;
  doc["cascadeEnabled"] = cascadeEnabled;

  String msg;
  serializeJson(doc, msg);
  ws.textAll(msg);
}

// handleWebSocketMessage
// 收到来自前端的消息（JSON），处理 update/save/reset/calibrate 等命令
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (!(info->final && info->index == 0 && info->len == len)) return;

  data[len] = 0;
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, (char*)data)) return;

  String type = doc["type"] | "";

  if (type == "update" || type == "save") {
    // 更新运行参数
    kp = doc["kp"] | kp;
    ki = doc["ki"] | ki;
    kd = doc["kd"] | kd;
    keepAngle = doc["keepAngle"] | keepAngle;
    deadzone = doc["deadzone"] | deadzone;
    
    // 速度环参数（串级控制）
    if (doc.containsKey("vkp")) vkp = doc["vkp"];
    if (doc.containsKey("vki")) vki = doc["vki"];
    if (doc.containsKey("vkd")) vkd = doc["vkd"];
    if (doc.containsKey("cascadeEnabled")) cascadeEnabled = doc["cascadeEnabled"];

    if (type == "save") {
      savePrefs();
    }
  }

  if (type == "reset") {
    // 恢复默认参数
    kp = default_kp;
    ki = default_ki;
    kd = default_kd;
    keepAngle = default_balanceAngle;
    deadzone = default_deadzone;
  }

  if (type == "calibrate") {
    // 使用当前互补角作为新的平衡点并保存
    keepAngle = compAngle;
    integrate = 0;
    savePrefs();
  }

  // 处理完后立即回复当前状态
  sendStatusToClients();
}

// WebSocket 事件回调：连接时发送一次状态，收到数据时处理
void onEvent(AsyncWebSocket *serverPtr, AsyncWebSocketClient *client,
             AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) sendStatusToClients();
  else if (type == WS_EVT_DATA) handleWebSocketMessage(arg, data, len);
}

// initWeb
// 注册 WebSocket 回调、HTTP 路由并启动服务器
void initWeb(){
  ws.onEvent(onEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html; charset=utf-8", index_html);
  });
  // motor 管理页面与 API
  server.on("/motor", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html; charset=utf-8", motor_html);
  });

  server.on("/motor/state", HTTP_GET, [](AsyncWebServerRequest *request){
    String s = "{\"inverted\":";
    s += (motor_inverted ? "true" : "false");
    s += "}";
    request->send(200, "application/json; charset=utf-8", s);
  });

  server.on("/motor/toggle", HTTP_POST, [](AsyncWebServerRequest *request){
    motor_inverted = !motor_inverted;
    String s = "{\"inverted\":";
    s += (motor_inverted ? "true" : "false");
    s += "}";
    request->send(200, "application/json; charset=utf-8", s);
  });

  server.on("/motor/set", HTTP_POST, [](AsyncWebServerRequest *request){
    if (request->hasParam("inv", true)){
      String v = request->getParam("inv", true)->value();
      motor_inverted = (v != "0");
    }
    request->send(200, "text/plain", "ok");
  });

  server.on("/motor/save", HTTP_POST, [](AsyncWebServerRequest *request){
    savePrefs();
    request->send(200, "application/json", "{\"saved\":true}");
  });

  // deadzone 手动调试页面
  server.on("/deadzone", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html; charset=utf-8", deadzone_html);
  });

  server.on("/deadzone/state", HTTP_GET, [](AsyncWebServerRequest *request){
    String s = "{";
    s += "\"pwm\":" + String(manual_pwm) + ",";
    s += "\"enabled\":" + String(manual_control_enabled?1:0);
    s += "}";
    request->send(200, "application/json", s);
  });

  server.on("/deadzone/set", HTTP_POST, [](AsyncWebServerRequest *request){
    // 解析简单 body 格式：value=NN
    String body;
    if (request->contentLength()) {
      body = request->arg(0); // AsyncWebServerRequest::arg 可以用于表单/ body
    }
    int v = manual_pwm;
    if (request->hasParam("value", true)) {
      v = request->getParam("value", true)->value().toInt();
    } else if (body.length()) {
      int idx = body.indexOf('=');
      if (idx >= 0) v = body.substring(idx+1).toInt();
    }
    v = constrain(v, -255, 255);
    manual_pwm = v;
    // 不自动启用手动控；界面通过 enable 切换
    request->send(200, "application/json", String("{\"pwm\":") + String(manual_pwm) + String("}"));
  });

  server.on("/deadzone/enable", HTTP_POST, [](AsyncWebServerRequest *request){
    int en = 0;
    if (request->hasParam("enabled", true)) en = request->getParam("enabled", true)->value().toInt();
    else if (request->contentLength()) {
      String body = request->arg(0);
      int idx = body.indexOf('=');
      if (idx >= 0) en = body.substring(idx+1).toInt();
    }
    manual_control_enabled = (en != 0);
    request->send(200, "application/json", String("{\"enabled\":") + String(manual_control_enabled?1:0) + String("}"));
  });
  server.begin();
}

// webCleanup
// 在主循环中周期性调用以清理已断开的客户端
void webCleanup(){
  ws.cleanupClients();
}
