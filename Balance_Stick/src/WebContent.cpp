// WebContent.cpp
// 将网页模板打包为 C 字符串并以 PROGMEM 存放，节省运行时 RAM。

#include <Arduino.h>
#include "WebContent.hpp"

// 注意：这是一个较长的 HTML 模板，保持为原始字符串字面量避免转义问题
const char index_html[] PROGMEM = R"HTML(
<!DOCTYPE HTML><html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ESP32C3 平衡杆 PID 调参</title>
<style>
  body{font-family:Arial;text-align:center;margin:0;background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);}
  .card{background:white;margin:15px auto;padding:20px;width:90%;max-width:520px;
        border-radius:12px;box-shadow:0 10px 40px rgba(0, 0, 0, 0.2);}
  .top{
    background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
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

  .logoDesc{
    font-size:13px;
    margin-top:6px;
    opacity:0.85;
    letter-spacing:1px;
  }

  h2, h3{
    color:#333;
    margin:15px 0 10px 0;
    font-size:16px;
    border-bottom:1px solid #ddd;
    padding-bottom:8px;
  }

  .row{
    display:grid;
    grid-template-columns: 80px 80px 80px 80px;
    align-items:center;
    justify-content:center;
    gap:10px;
    margin:12px 0;
  }

  .label{
    font-size:18px;
    white-space:nowrap;
    justify-self:center;
    text-align:center;
  }

  .val{
    font-size:18px;
    font-weight:bold;
    color:#667eea;
    width:120px;
    text-align:center;
    justify-self:center;
  }

  .row button{
    justify-self:center;
  }
  @media (max-width: 320px){
    .row{
      grid-template-columns: 80px 80px 80px 80px;
    }
    .val{ width:110px; }
    button{ width:56px; height:42px; font-size:24px; }
  }

  button{
    width:64px;
    height:44px;
    font-size:26px;
    border:none;
    border-radius:10px;
    background:#667eea;
    color:white;
    display:flex;
    align-items:center;
    justify-content:center;
    line-height:1;
    padding:0;
    cursor:pointer;
    transition:all 0.3s ease;
    font-weight:600;
  }
  button:hover{
    background:#5568d3;
    transform:translateY(-2px);
    box-shadow:0 5px 15px rgba(102, 126, 234, 0.4);
  }
  button:active{
    transform:scale(0.98);
  }

  .actionBtn{width:140px;height:45px;font-size:18px;margin:8px;background:#667eea;}
  .actionBtn:hover{background:#5568d3;}
  .danger{background:#ff6b6b;}
  .danger:hover{background:#ff5252;}
  .green{background:#51cf66;}
  .green:hover{background:#40c057;}
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
    color:#667eea;
  }

  .pwm{
    font-size:16px;
    margin: 6px 0 10px 0;
  }
  .pwm span{
    font-weight:bold;
    color:#667eea;
  }

  .pwmPos{ color:#d10000 !important; }
  .pwmNeg{ color:#666 !important; }
</style>
</head>

<body>
<div class="top">
  <div class="logoText">
    极点电子
  </div>
  <div class="logoDesc">平衡杆PID调参面板</div>
</div>


<div class="card">
  <h2>角度 & 调试</h2>

  <div class="inlineLine">
    <div>目前角度: <span id="angle">--</span></div>
    <div>互补角度: <span id="cangle">--</span></div>
  </div>

  <hr>

  <div class="pwm">
    电机PWM: <span id="pwm">--</span>
  </div>

  <div class="inlineLine" id="cascadeInfo" style="display:none;">
    <div>目标角速度: <span id="targetGyro">--</span></div>
    <div>速度积分: <span id="vintegrate">--</span></div>
  </div>

  <hr style="margin-top:20px;">

  <h2 style="margin-top:20px;">参数</h2>

  <div class="row">
    <div class="label">Kp</div>
    <button onclick="change('kp', -0.1)">-</button>
    <div class="val" id="kpVal">--</div>
    <button onclick="change('kp', 0.1)">+</button>
  </div>

  <div class="row">
    <div class="label">Ki</div>
    <button onclick="change('ki', -0.01)">-</button>
    <div class="val" id="kiVal">--</div>
    <button onclick="change('ki', 0.01)">+</button>
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

  <div class="row">
    <div class="label">死区</div>
    <button onclick="change('deadzone', -1)">-</button>
    <div class="val" id="deadzoneVal">--</div>
    <button onclick="change('deadzone', 1)">+</button>
  </div>

  <br>

  <div style="display:flex; justify-content:center; align-items:center; gap:10px; margin:10px 0;">
    <label for="cascadeToggle" style="font-size:16px; font-weight:600;">启用串级控制:</label>
    <input type="checkbox" id="cascadeToggle" onchange="toggleCascade()" style="width:24px; height:24px; accent-color:#667eea; cursor:pointer;">
  </div>

  <div id="cascadeParams" style="display:none;">
    <h3 style="margin-top:15px;">速度环参数</h3>

    <div class="row">
      <div class="label">Vkp</div>
      <button onclick="change('vkp', -0.1)">-</button>
      <div class="val" id="vkpVal">--</div>
      <button onclick="change('vkp', 0.1)">+</button>
    </div>

    <div class="row">
      <div class="label">Vki</div>
      <button onclick="change('vki', -0.01)">-</button>
      <div class="val" id="vkiVal">--</div>
      <button onclick="change('vki', 0.01)">+</button>
    </div>

    <div class="row">
      <div class="label">Vkd</div>
      <button onclick="change('vkd', -0.1)">-</button>
      <div class="val" id="vkdVal">--</div>
      <button onclick="change('vkd', 0.1)">+</button>
    </div>
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
var ws = null;
var reconnectInterval = 2000;
var reconnectAttempts = 0;
var maxReconnectAttempts = 10;
var params = {kp:0, ki:0, kd:0, keepAngle:0, deadzone:0, vkp:0, vki:0, vkd:0};

var paramLimits = {
  kp: {min:0, max:1000},
  ki: {min:0, max:10},
  kd: {min:0, max:1000},
  keepAngle: {min:-20, max:20},
  deadzone: {min:0, max:200},
  vkp: {min:0, max:10},
  vki: {min:0, max:1},
  vkd: {min:0, max:10}
};

function connectWebSocket(){
  var gateway = "ws://" + window.location.hostname + "/ws";
  ws = new WebSocket(gateway);
  
  ws.onmessage = onMessage;
  
  ws.onopen = function(){
    reconnectAttempts = 0;
    console.log('WebSocket connected');
  };
  
  ws.onclose = function(){
    if(reconnectAttempts < maxReconnectAttempts){
      reconnectAttempts++;
      var delay = reconnectInterval * reconnectAttempts;
      console.log('WebSocket disconnected. Reconnecting in ' + delay + 'ms...');
      setTimeout(connectWebSocket, delay);
    } else {
      console.error('WebSocket: Max reconnection attempts reached');
      document.body.innerHTML += '<div style="color:red;padding:20px;text-align:center;">连接已断开，请刷新页面</div>';
    }
  };
  
  ws.onerror = function(error){
    console.error('WebSocket error:', error);
  };
}

window.addEventListener('load', connectWebSocket);

function onMessage(event){
  var data = JSON.parse(event.data);

  if(data.angle !== undefined){
    document.getElementById("angle").innerHTML = data.angle.toFixed(2);
    document.getElementById("cangle").innerHTML = data.cangle.toFixed(2);
    var pwmEl = document.getElementById("pwm");
    pwmEl.innerHTML = data.pwm;
    
    pwmEl.classList.remove("pwmPos", "pwmNeg");
    if (data.pwm > 0) pwmEl.classList.add("pwmPos");
    else pwmEl.classList.add("pwmNeg");
    
    // 串级控制信息
    if (data.cascadeEnabled) {
      document.getElementById("cascadeInfo").style.display = "flex";
      document.getElementById("targetGyro").innerHTML = data.targetGyro.toFixed(2);
      document.getElementById("vintegrate").innerHTML = data.vintegrate.toFixed(2);
    } else {
      document.getElementById("cascadeInfo").style.display = "none";
    }
  }

  if(data.kp !== undefined){
    params.kp = data.kp;
    params.ki = data.ki;
    params.kd = data.kd;
    params.keepAngle = data.keepAngle;
    params.deadzone = data.deadzone;
    params.vkp = data.vkp;
    params.vki = data.vki;
    params.vkd = data.vkd;

    document.getElementById("kpVal").innerHTML = data.kp.toFixed(2);
    document.getElementById("kiVal").innerHTML = data.ki.toFixed(3);
    document.getElementById("kdVal").innerHTML = data.kd.toFixed(2);
    document.getElementById("keepVal").innerHTML = data.keepAngle.toFixed(2);
    document.getElementById("deadzoneVal").innerHTML = data.deadzone;
    
    // 速度环参数
    document.getElementById("vkpVal").innerHTML = data.vkp.toFixed(2);
    document.getElementById("vkiVal").innerHTML = data.vki.toFixed(3);
    document.getElementById("vkdVal").innerHTML = data.vkd.toFixed(2);
    
    // 串级控制开关状态
    document.getElementById("cascadeToggle").checked = data.cascadeEnabled;
    document.getElementById("cascadeParams").style.display = data.cascadeEnabled ? "block" : "none";
  }
}

function change(key, step){
  params[key] += step;

  if(paramLimits[key]){
    params[key] = Math.max(paramLimits[key].min, Math.min(paramLimits[key].max, params[key]));
  }

  if(ws && ws.readyState === WebSocket.OPEN){
    ws.send(JSON.stringify({type:"update", kp:params.kp, ki:params.ki, kd:params.kd, keepAngle:params.keepAngle, deadzone:params.deadzone, vkp:params.vkp, vki:params.vki, vkd:params.vkd}));
  }
}

function toggleCascade(){
  var enabled = document.getElementById("cascadeToggle").checked;
  document.getElementById("cascadeParams").style.display = enabled ? "block" : "none";
  
  if(ws && ws.readyState === WebSocket.OPEN){
    ws.send(JSON.stringify({type:"update", cascadeEnabled:enabled}));
  }
}

function save(){
  if(!ws || ws.readyState !== WebSocket.OPEN){
    alert("未连接到服务器");
    return;
  }
  ws.send(JSON.stringify({type:"save", kp:params.kp, ki:params.ki, kd:params.kd, keepAngle:params.keepAngle, deadzone:params.deadzone}));
  alert("已保存！");
}

function reset(){
  if(!ws || ws.readyState !== WebSocket.OPEN){
    alert("未连接到服务器");
    return;
  }
  ws.send(JSON.stringify({type:"reset"}));
}

function calibrate(){
  if(!ws || ws.readyState !== WebSocket.OPEN){
    alert("未连接到服务器");
    return;
  }
  ws.send(JSON.stringify({type:"calibrate"}));
}
</script>

</body>
</html>
)HTML";

const char motor_html[] PROGMEM = R"HTML(
<!DOCTYPE HTML>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>电机方向设置</title>
<style>
  * {
    margin: 0;
    padding: 0;
    box-sizing: border-box;
  }
  body {
    font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, Cantarell, sans-serif;
    background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
    min-height: 100vh;
    display: flex;
    align-items: center;
    justify-content: center;
    padding: 20px;
  }
  .container {
    background: white;
    border-radius: 12px;
    box-shadow: 0 10px 40px rgba(0, 0, 0, 0.2);
    padding: 40px;
    max-width: 400px;
    width: 100%;
  }
  h2 {
    color: #333;
    margin-bottom: 30px;
    text-align: center;
    font-size: 28px;
  }
  .info-box {
    background: #f5f7fa;
    border-left: 4px solid #667eea;
    padding: 15px 20px;
    margin-bottom: 25px;
    border-radius: 6px;
    text-align: center;
  }
  .info-label {
    color: #666;
    font-size: 14px;
    margin-bottom: 8px;
    display: block;
  }
  #state {
    font-size: 24px;
    font-weight: bold;
    color: #667eea;
  }
  .button-group {
    display: flex;
    flex-direction: column;
    gap: 12px;
    margin-bottom: 25px;
  }
  .btn {
    padding: 12px 18px;
    font-size: 18px;
    border-radius: 8px;
    border: none;
    background: #667eea;
    color: #fff;
    cursor: pointer;
    transition: all 0.3s ease;
    font-weight: 600;
  }
  .btn:hover {
    background: #5568d3;
    transform: translateY(-2px);
    box-shadow: 0 5px 15px rgba(102, 126, 234, 0.4);
  }
  .btn:active {
    transform: translateY(0);
  }
  .danger {
    background: #ff6b6b;
  }
  .danger:hover {
    background: #ff5252;
  }
  .info-text {
    margin-top: 20px;
    color: #666;
    font-size: 14px;
    line-height: 1.6;
    text-align: center;
  }
  @media (max-width: 480px) {
    .container {
      padding: 30px 20px;
    }
    h2 {
      font-size: 24px;
      margin-bottom: 20px;
    }
    .btn {
      padding: 10px 16px;
      font-size: 16px;
    }
  }
</style>
</head>
<body>
<div class="container">
  <h2>电机方向</h2>
  <div class="info-box">
    <span class="info-label">当前状态:</span>
    <span id="state">--</span>
  </div>
  <div class="button-group">
    <button class="btn" id="toggle">切换方向</button>
    <button class="btn" id="save">保存设置</button>
    <button class="btn danger" id="revert">恢复默认</button>
  </div>
  <div class="info-text">
    <p>提示: 若电机转向不对，点击切换方向测试，满意后点击保存设置</p>
  </div>
</div>
<script>
var motorState = null;
var retryDelay = 1000;
var maxRetries = 3;

function fetchState(retryCount){
  retryCount = retryCount || 0;
  var el = document.getElementById('state');
  
  if(retryCount === 0){
    el.innerText = '加载中';
  }
  
  fetch('/motor/state', {method:'GET', cache:'no-cache'})
    .then(function(r){
      if (!r.ok) {
        throw new Error('HTTP ' + r.status + ': ' + r.statusText);
      }
      return r.json();
    })
    .then(function(j){
      if (j && j.inverted !== undefined) {
        motorState = j.inverted;
        el.innerText = j.inverted ? '已转换' : '未转换';
      }
    })
    .catch(function(e){
      console.error('Fetch state error:', e);
      if(retryCount < maxRetries){
        setTimeout(function(){
          fetchState(retryCount + 1);
        }, retryDelay);
      } else {
        el.innerText = '错误';
      }
    });
}

document.getElementById('toggle').addEventListener('click', function(){
  fetch('/motor/toggle', {method:'POST'})
    .then(function(r){
      if(!r.ok) throw new Error('Toggle failed: ' + r.status);
      return r.json();
    })
    .then(function(j){
      motorState = j.inverted;
      document.getElementById('state').innerText = j.inverted ? '已转换' : '未转换';
    })
    .catch(function(e){
      console.error('Toggle error:', e);
      setTimeout(fetchState, retryDelay);
    });
});

document.getElementById('save').addEventListener('click', function(){
  fetch('/motor/save', {method:'POST'})
    .then(function(r){
      if(!r.ok) throw new Error('Save failed: ' + r.status);
      alert('已保存');
    })
    .catch(function(e){
      console.error('Save error:', e);
      alert('保存失败');
    });
});

document.getElementById('revert').addEventListener('click', function(){
  fetch('/motor/set', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body:'inv=0'})
    .then(function(r){
      if(!r.ok) throw new Error('Set failed: ' + r.status);
      return fetch('/motor/save', {method:'POST'});
    })
    .then(function(r){
      if(!r.ok) throw new Error('Save failed: ' + r.status);
      fetchState();
    })
    .catch(function(e){
      console.error('Revert error:', e);
      alert('操作失败');
    });
});

window.addEventListener('load', fetchState);
</script>
</body>
</html>
)HTML";

const char deadzone_html[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Deadzone 调试</title>
  <style>
    * {
      margin: 0;
      padding: 0;
      box-sizing: border-box;
    }
    body {
      font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, Cantarell, sans-serif;
      background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
      min-height: 100vh;
      display: flex;
      align-items: center;
      justify-content: center;
      padding: 20px;
    }
    .container {
      background: white;
      border-radius: 12px;
      box-shadow: 0 10px 40px rgba(0, 0, 0, 0.2);
      padding: 40px;
      max-width: 400px;
      width: 100%;
    }
    h2 {
      color: #333;
      margin-bottom: 30px;
      text-align: center;
      font-size: 28px;
    }
    .info-box {
      background: #f5f7fa;
      border-left: 4px solid #667eea;
      padding: 15px 20px;
      margin-bottom: 25px;
      border-radius: 6px;
    }
    .info-label {
      color: #666;
      font-size: 14px;
      margin-bottom: 8px;
      display: block;
    }
    #pwm {
      font-size: 32px;
      font-weight: bold;
      color: #667eea;
    }
    .control-group {
      display: flex;
      gap: 10px;
      margin-bottom: 25px;
    }
    button {
      flex: 1;
      padding: 12px 20px;
      border: none;
      border-radius: 6px;
      font-size: 16px;
      font-weight: 600;
      cursor: pointer;
      transition: all 0.3s ease;
    }
    .step-btn {
      background: #667eea;
      color: white;
    }
    .step-btn:hover {
      background: #5568d3;
      transform: translateY(-2px);
      box-shadow: 0 5px 15px rgba(102, 126, 234, 0.4);
    }
    .step-btn:active {
      transform: translateY(0);
    }
    .stop-btn {
      background: #ff6b6b;
      color: white;
    }
    .stop-btn:hover {
      background: #ff5252;
      transform: translateY(-2px);
      box-shadow: 0 5px 15px rgba(255, 107, 107, 0.4);
    }
    .stop-btn:active {
      transform: translateY(0);
    }
    .checkbox-box {
      display: flex;
      align-items: center;
      gap: 12px;
      padding: 15px;
      background: #f5f7fa;
      border-radius: 6px;
    }
    input[type="checkbox"] {
      width: 20px;
      height: 20px;
      cursor: pointer;
      accent-color: #667eea;
    }
    label {
      color: #333;
      font-size: 16px;
      font-weight: 500;
      cursor: pointer;
      flex: 1;
    }
    @media (max-width: 480px) {
      .container {
        padding: 30px 20px;
      }
      h2 {
        font-size: 24px;
      }
      button {
        padding: 10px 16px;
        font-size: 14px;
      }
    }
  </style>
</head>
<body>
  <div class="container">
    <h2>Deadzone 调试</h2>
    <div class="info-box">
      <span class="info-label">当前 PWM:</span>
      <span id="pwm">0</span>
    </div>
    <div class="control-group">
      <button class="step-btn" onclick="step(-1)">-</button>
      <button class="stop-btn" onclick="set0()">停止</button>
      <button class="step-btn" onclick="step(1)">+</button>
    </div>
    <div class="checkbox-box">
      <input type="checkbox" id="enable">
      <label for="enable">启用手动控制</label>
    </div>
  </div>
  <script>
    var pwm = 0;
    var ws = null;
    var reconnectTime = 1000;
    
    function connectWS(){
      var gw = 'ws://' + location.hostname + '/ws';
      ws = new WebSocket(gw);
      ws.onmessage = function(e){
        try{
          var d = JSON.parse(e.data);
          if (d.applied_pwm !== undefined) {
            pwm = d.applied_pwm;
            document.getElementById('pwm').innerText = pwm;
          }
          if (d.manual_enabled !== undefined) {
            document.getElementById('enable').checked = !!d.manual_enabled;
          }
        }catch(err){}
      };
      ws.onclose = function(){
        setTimeout(connectWS, reconnectTime);
      };
    }
    
    function step(delta){
      var v = pwm + delta;
      v = Math.max(-255, Math.min(255, v));
      fetch('/deadzone/set', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body: 'value=' + v})
        .then(function(r){ return r.json(); })
        .then(function(j){ 
          if(j && j.pwm !== undefined) {
            pwm = j.pwm;
            document.getElementById('pwm').innerText = pwm;
          }
        })
        .catch(function(e){ console.error('step failed:', e); });
    }
    
    function set0(){
      fetch('/deadzone/set', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body: 'value=0'})
        .then(function(r){ return r.json(); })
        .then(function(j){ 
          if(j && j.pwm !== undefined) {
            pwm = j.pwm;
            document.getElementById('pwm').innerText = pwm;
          }
        })
        .catch(function(e){ console.error('set0 failed:', e); });
    }
    
    document.addEventListener('DOMContentLoaded', function(){
      connectWS();
      document.getElementById('enable').addEventListener('change', function(e){
        fetch('/deadzone/enable', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body: 'enabled=' + (e.target.checked?1:0)})
          .then(function(){})
          .catch(function(e){ console.error('enable toggle failed:', e); });
      });
    });
  </script>
</body>
</html>
)HTML";
