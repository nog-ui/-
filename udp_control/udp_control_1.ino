#include <WiFi.h>
#include <Wire.h>
#include <MPU6050_tockn.h>
#include "Motor.hpp"

#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <ArduinoJson.h>

// ================== WiFi AP 模式 ==================
const char* AP_SSID = "ESP32C3_Balance";
const char* AP_PSK  = "";

// ================== WebServer & WebSocket ==================
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ================== Preferences（永久存储） ==================
Preferences prefs;

// ================== MPU & Motor ==================
MPU6050 mpu6050(Wire);
Motor motor(7, 10, 0, 1);

const int ledpin1 = 13, ledpin2 = 12;

// ================== 状态变量 ==================
float ax, gx;
float angleX, gyroX;
float integrate = 0;
float bias = 0;
int motor_pwm = 0;

// 上电默认：不允许控制（等待手势激活）
bool Gestures_SW = false;
bool fallDown_SW = false;

// ================== 卡尔曼滤波参数 ==================
float kalmanAngle = 0;
float kalmanBias = 0;
float P[2][2] = {{0, 0}, {0, 0}};
float Q_angle = 0.001;
float Q_bias = 0.003;
float R_measure = 0.03;
float dt;
unsigned long lastTime;

// ================== 默认参数（恢复默认用） ==================
const float default_kp = 15;
const float default_ki = 0.03;
const float default_kd = 9.9;
const float default_balanceAngle = -1.85;

// ================== PID 参数（可网页调节） ==================
float kp = default_kp, ki = default_ki, kd = default_kd;
float keepAngle = default_balanceAngle;

// ============================================================
// HTML 页面（按键加减 + 保存 + 恢复默认 + 校准 + 显示调试信息）
// ============================================================
const char index_html[] PROGMEM = R"HTML(
<!DOCTYPE HTML><html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ESP32C3 平衡杆 PID 调参</title>
<style>
  body{font-family:Arial;text-align:center;margin:0;background:#f2f2f2;}
  .card{background:white;margin:15px auto;padding:20px;width:90%;max-width:520px;
        border-radius:12px;box-shadow:0 2px 10px rgba(0,0,0,0.2);}
  .top{
    background:#143642;
    color:white;
    padding:14px 10px;
    text-align:center;
  }

  .logoText{
    font-size:22px;
    letter-spacing:2px;
    font-weight:700;
    line-height:1.1;
  }

  .brandMain{
    font-weight:800;
  }

  .brandSub{
    font-weight:500;
    opacity:0.92;
    margin-left:4px;
  }

  .logoDesc{
    font-size:13px;
    margin-top:6px;
    opacity:0.85;
    letter-spacing:1px;
  }

  .row{
    display:grid;
    grid-template-columns: 110px 64px 120px 64px;
    align-items:center;
    justify-content:center;
    gap:10px;
    margin:12px 0;
  }


  .label{
    font-size:18px;
    white-space:nowrap;      /* 防止“角度”换行 */
    justify-self:center;     /* 标签列居中 */
    text-align:center;
  }

  .val{
    font-size:18px;
    font-weight:bold;
    color:#0f8b8d;
    width:120px;
    text-align:center;
    justify-self:center;     /* 数值列居中 */
  }

  .row button{
    justify-self:center;     /* 按钮列居中 */
  }
  @media (max-width: 360px){
    .row{
      grid-template-columns: 80px 56px 110px 56px;
    }
    .val{ width:110px; }
    button{ width:56px; height:42px; font-size:24px; }
  }



  button{
    width:64px;               /* ✅ 变宽 */
    height:44px;              /* ✅ 稍微变高 */
    font-size:26px;           /* + - 更醒目 */
    border:none;
    border-radius:10px;
    background:#0f8b8d;
    color:white;

    display:flex;             /* ✅ 关键：用 flex 让符号居中 */
    align-items:center;       /* ✅ 垂直居中 */
    justify-content:center;   /* ✅ 水平居中 */
    line-height:1;            /* ✅ 防止字体基线影响上下偏移 */
    padding:0;                /* ✅ 清掉默认内边距 */
  }
  button:active{
    transform:scale(0.98);
  }

  .actionBtn{width:140px;height:45px;font-size:18px;margin:8px;}
  .danger{background:#b00020;}
  .green{background:#2e7d32;}
  .inlineLine{
    display:flex;
    justify-content:center;
    gap:18px;
    flex-wrap:wrap;
    font-size:16px;
    margin: 6px 0 10px 0;
  }
  .inlineLine span{
    font-weight:bold;
    color:#0f8b8d;
  }

  .pwm{
    font-size:16px;
    margin: 6px 0 10px 0;
  }
  .pwm span{
    font-weight:bold;
  }

  .twoCol{
    display:flex;
    justify-content:center;
    gap:24px;
    flex-wrap:wrap;
    font-size:16px;
    margin-top: 6px; 
  }
  .twoCol span{
    font-weight:bold;
    color:#0f8b8d;
  }

  .pwmPos{ color:#d10000 !important; }   /* 正数：红色 */
  .pwmNeg{ color:#000000 !important; }   /* 负数：黑色 */


</style>
</head>

<body>
<div class="top">
  <div class="logoText">
    <span class="brandMain">极点电子</span>
  </div>
  <div class="logoDesc">平衡杆PID调参面板</div>
</div>


<div class="card">
  <h3>角度 & 调试</h3>

  <div class="inlineLine">
    <div>目前角度: <span id="angle">--</span></div>
    <div>卡尔曼角度: <span id="kangle">--</span></div>
  </div>

  <hr>

  <div class="pwm">
    电机PWM: <span id="pwm">--</span>
  </div>

  <div class="twoCol">
    <div>bias: <span id="bias">--</span></div>
    <div>integrate: <span id="integ">--</span></div>
  </div>
</div>



<div class="card">
  <h3>参数</h3>

  <div class="row">
    <div class="label">Kp</div>
    <button onclick="change('kp', -0.1)">-</button>
    <div class="val" id="kpVal">--</div>
    <button onclick="change('kp', 0.1)">+</button>
  </div>

  <div class="row">
    <div class="label">Ki</div>
    <button onclick="change('ki', -0.001)">-</button>
    <div class="val" id="kiVal">--</div>
    <button onclick="change('ki', 0.001)">+</button>
  </div>

  <div class="row">
    <div class="label">Kd</div>
    <button onclick="change('kd', -0.1)">-</button>
    <div class="val" id="kdVal">--</div>
    <button onclick="change('kd', 0.1)">+</button>
  </div>

  <div class="row">
    <div class="label">角度</div>
    <button onclick="change('keepAngle', -0.05)">-</button>
    <div class="val" id="keepVal">--</div>
    <button onclick="change('keepAngle', 0.05)">+</button>
  </div>

  <br>

  <div style="display:flex; justify-content:center; gap:12px; flex-wrap:wrap;">
    <button class="actionBtn" onclick="save()">保存参数</button>
    <button class="actionBtn danger" onclick="reset()">恢复默认</button>
  </div>

  <br>

  <div style="display:flex; justify-content:center;">
    <button class="actionBtn green" onclick="calibrate()">一键校准</button>
  </div>

</div>


<script>
var websocket;
var params = {kp:0, ki:0, kd:0, keepAngle:0};

window.addEventListener('load', function(){
  var gateway = "ws://" + window.location.hostname + "/ws";
  websocket = new WebSocket(gateway);
  websocket.onmessage = onMessage;
  websocket.onclose = function(){ setTimeout(function(){location.reload();},2000); };
});

function onMessage(event){
  var data = JSON.parse(event.data);

  if(data.angle !== undefined){
    document.getElementById("angle").innerHTML = data.angle.toFixed(2);
    document.getElementById("kangle").innerHTML = data.kangle.toFixed(2);
    var pwmEl = document.getElementById("pwm");
    pwmEl.innerHTML = data.pwm;
    
    // 正数红色，负数黑色，0 用黑色（你也可以改灰色）
    pwmEl.classList.remove("pwmPos", "pwmNeg");
    if (data.pwm > 0) pwmEl.classList.add("pwmPos");
    else pwmEl.classList.add("pwmNeg");
    document.getElementById("bias").innerHTML = data.bias.toFixed(2);
    document.getElementById("integ").innerHTML = data.integrate.toFixed(2);
  }

  if(data.kp !== undefined){
    params.kp = data.kp;
    params.ki = data.ki;
    params.kd = data.kd;
    params.keepAngle = data.keepAngle;

    document.getElementById("kpVal").innerHTML = data.kp.toFixed(2);
    document.getElementById("kiVal").innerHTML = data.ki.toFixed(3);
    document.getElementById("kdVal").innerHTML = data.kd.toFixed(2);
    document.getElementById("keepVal").innerHTML = data.keepAngle.toFixed(2);
  }
}

function change(key, step){
  params[key] += step;

  if(key=="kp") params[key] = Math.max(0, Math.min(40, params[key]));
  if(key=="ki") params[key] = Math.max(0, Math.min(2, params[key]));
  if(key=="kd") params[key] = Math.max(0, Math.min(40, params[key]));
  if(key=="keepAngle") params[key] = Math.max(-20, Math.min(20, params[key]));

  websocket.send(JSON.stringify({type:"update", kp:params.kp, ki:params.ki, kd:params.kd, keepAngle:params.keepAngle}));
}

function save(){
  websocket.send(JSON.stringify({type:"save", kp:params.kp, ki:params.ki, kd:params.kd, keepAngle:params.keepAngle}));
  alert("Saved!");
}

function reset(){
  websocket.send(JSON.stringify({type:"reset"}));
}

function calibrate(){
  websocket.send(JSON.stringify({type:"calibrate"}));
}
</script>

</body>
</html>
)HTML";

// ============================================================
// 读取 MPU6050 数据
// ============================================================
void get_mpu6050_val() {
  mpu6050.update();
  ax = mpu6050.getAngleX();
  gx = mpu6050.getGyroX();
}

// ============================================================
// 卡尔曼滤波
// ============================================================
float kalmanFilter(float angle, float rate) {
  unsigned long now = millis();
  dt = (now - lastTime) / 1000.0;
  lastTime = now;

  float predictedRate = rate - kalmanBias;
  kalmanAngle += dt * predictedRate;

  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  float S = P[0][0] + R_measure;
  float K0 = P[0][0] / S;
  float K1 = P[1][0] / S;

  float y = angle - kalmanAngle;
  kalmanAngle += K0 * y;
  kalmanBias += K1 * y;

  float P00_temp = P[0][0];
  float P01_temp = P[0][1];
  P[0][0] = P00_temp - K0 * P00_temp;
  P[0][1] = P01_temp - K0 * P01_temp;
  P[1][0] = P[1][0] - K1 * P00_temp;
  P[1][1] = P[1][1] - K1 * P01_temp;

  return kalmanAngle;
}

// ============================================================
// 倾倒检测 + 手势检测
// ============================================================
void detect_fall_down() {
  fallDown_SW = ((ax > -35) && (ax < 35));
}

void detect_gestures() {
  float gyroY = mpu6050.getGyroY();
  if (gyroY > 450) {
    Gestures_SW = true;
  } else if (gyroY < -450) {
    Gestures_SW = false;
  }
}

// ============================================================
// PID 计算
// ============================================================
void pwm_calculation() {
  // // 上电 2 秒保护：防止 MPU 初始漂移导致电机乱转
  // if (millis() < 2000) {
  //   motor_pwm = 0;
  //   integrate = 0;
  //   bias = 0;
  //   return;
  // }

  detect_gestures();
  detect_fall_down();

  // 只有检测到手势允许 + 未倾倒 才输出
  if ((!fallDown_SW) || (!Gestures_SW)) {
    motor_pwm = 0;
    integrate = 0;
    bias = 0;
  } else {
    angleX = mpu6050.getAngleX();
    gyroX = mpu6050.getGyroX();
    float kAngle = kalmanFilter(angleX, gyroX);

    bias = kAngle - keepAngle;
    integrate += bias;
    integrate = constrain(integrate, -200, 200);

    motor_pwm = kp * bias + ki * integrate + kd * gyroX;
  }
}

// ============================================================
// LED 效果
// ============================================================
void led_effect() {
  int ledPWM = abs(motor_pwm);
  if (motor_pwm < 0) analogWrite(ledpin1, ledPWM);
  else analogWrite(ledpin2, ledPWM);
}

// ============================================================
// WebSocket 推送实时状态
// ============================================================
void sendStatusToClients() {
  StaticJsonDocument<256> doc;
  doc["angle"] = mpu6050.getAngleX();
  doc["kangle"] = kalmanAngle;
  doc["kp"] = kp;
  doc["ki"] = ki;
  doc["kd"] = kd;
  doc["keepAngle"] = keepAngle;

  doc["pwm"] = motor_pwm;
  doc["bias"] = bias;
  doc["integrate"] = integrate;

  String msg;
  serializeJson(doc, msg);
  ws.textAll(msg);
}

// ============================================================
// WebSocket 接收参数
// ============================================================
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (!(info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)) return;

  data[len] = 0;
  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, (char*)data);
  if (err) return;

  String type = doc["type"] | "";

  if (type == "update" || type == "save") {
    kp = doc["kp"] | kp;
    ki = doc["ki"] | ki;
    kd = doc["kd"] | kd;
    keepAngle = doc["keepAngle"] | keepAngle;

    if (type == "save") {
      prefs.putFloat("kp", kp);
      prefs.putFloat("ki", ki);
      prefs.putFloat("kd", kd);
      prefs.putFloat("keepAngle", keepAngle);
    }
  }

  if (type == "reset") {
    kp = default_kp;
    ki = default_ki;
    kd = default_kd;
    keepAngle = default_balanceAngle;

    prefs.putFloat("kp", kp);
    prefs.putFloat("ki", ki);
    prefs.putFloat("kd", kd);
    prefs.putFloat("keepAngle", keepAngle);
  }

  if (type == "calibrate") {
    keepAngle = kalmanAngle;
    integrate = 0;
    prefs.putFloat("keepAngle", keepAngle);
  }

  sendStatusToClients();
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    sendStatusToClients();
  } else if (type == WS_EVT_DATA) {
    handleWebSocketMessage(arg, data, len);
  }
}

// ============================================================
// setup
// ============================================================
void setup() {
  Serial.begin(115200);

  pinMode(ledpin1, OUTPUT);
  pinMode(ledpin2, OUTPUT);

  WiFi.softAP(AP_SSID, AP_PSK);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  Wire.begin(4, 5);
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  lastTime = millis();

  prefs.begin("pid", false);
  kp = prefs.getFloat("kp", default_kp);
  ki = prefs.getFloat("ki", default_ki);
  kd = prefs.getFloat("kd", default_kd);
  keepAngle = prefs.getFloat("keepAngle", default_balanceAngle);

  motor.flameout();

  ws.onEvent(onEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    AsyncWebServerResponse *response = request->beginResponse_P(200, "text/html; charset=utf-8", index_html);
    request->send(response);
  });


  server.begin();

  motor.flameout();  // 防止因部分引脚开机启动默认为高电平，导致马达转动
  analogWrite(ledpin1, 125);
  analogWrite(ledpin2, 125);
  // 上电默认关闭手势控制
  Gestures_SW = false;

}

// ============================================================
// loop
// ============================================================
unsigned long lastSend = 0;

void loop() {
  get_mpu6050_val();
  pwm_calculation();
  motor.run(motor_pwm);
  led_effect();

  ws.cleanupClients();

  if (millis() - lastSend > 100) {
    sendStatusToClients();
    lastSend = millis();
  }
}
