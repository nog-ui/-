# Balance_Stick

ESP32-C3 平衡杆控制器项目（基于 PlatformIO + Arduino 框架）。

文档

- 新手介绍（算法 + 功能 + 上手）：`新手介绍.md`

快速开始

- 安装 PlatformIO（VS Code 推荐 PlatformIO IDE 扩展）
- 克隆仓库后在项目根目录运行：

```bash
pio run -e airm2m_core_esp32c3
```

上传固件（连接设备并在项目根目录运行）：

```bash
pio run -e airm2m_core_esp32c3 -t upload
```

开发说明

- 源代码：`src/main.cpp`
- 库：`lib/`（本地库）、通过 `platformio.ini` 指定的外部依赖
- 单元测试目录：`test/`

持续集成

本仓库包含 GitHub Actions 工作流，会在每次 push/PR 时执行 `pio run` 验证构建。

如果需要我可以继续添加代码格式化、静态检查、或者更多 CI 步骤。欢迎告诉我你的优先项。
