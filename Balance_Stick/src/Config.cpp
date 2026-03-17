#include <Preferences.h>
#include "Config.hpp"
#include "State.hpp"

// Config.cpp
// 使用 Preferences API 将 PID 参数在 NVS 中持久化

static Preferences prefs;

// 从 NVS 读取参数并更新全局变量
void loadPrefs(){
  prefs.begin("pid", false);
  kp = prefs.getFloat("kp", default_kp);
  ki = prefs.getFloat("ki", default_ki);
  kd = prefs.getFloat("kd", default_kd);
  keepAngle = prefs.getFloat("keepAngle", default_balanceAngle);
  deadzone = prefs.getFloat("deadzone", default_deadzone);
  // motor inversion（0/1 存储）
  motor_inverted = prefs.getInt("motor_inv", 0) != 0;
  
  // 速度环参数（串级控制）
  vkp = prefs.getFloat("vkp", default_vkp);
  vki = prefs.getFloat("vki", default_vki);
  vkd = prefs.getFloat("vkd", default_vkd);
  cascadeEnabled = prefs.getBool("cascade_enabled", false);
}

// 将当前参数写回 NVS
void savePrefs(){
  prefs.putFloat("kp", kp);
  prefs.putFloat("ki", ki);
  prefs.putFloat("kd", kd);
  prefs.putFloat("keepAngle", keepAngle);
  prefs.putFloat("deadzone", deadzone);
  prefs.putInt("motor_inv", motor_inverted ? 1 : 0);
  
  // 速度环参数（串级控制）
  prefs.putFloat("vkp", vkp);
  prefs.putFloat("vki", vki);
  prefs.putFloat("vkd", vkd);
  prefs.putBool("cascade_enabled", cascadeEnabled);
}
