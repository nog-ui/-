#pragma once

// Config.hpp
// Preferences 读写封装：从 NVS 加载/保存 PID 与其他运行参数。

// 从非易失存储加载参数到全局变量（kp/ki/kd/keepAngle/deadzone）
void loadPrefs();

// 将当前全局参数写回非易失存储
void savePrefs();
