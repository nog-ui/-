# 贡献指南

欢迎贡献！建议流程：

1. Fork 仓库并新建分支（例如 `feat/xxx` 或 `fix/yyy`）。
2. 保持提交小而清晰，写明变更目的。  
3. 提交前在本地运行构建：

```bash
pio run -e airm2m_core_esp32c3
```

4. 发起 Pull Request，CI 会自动构建验证。

如需添加单元测试或修改硬件映射，请在 PR 描述中说明测试方法和硬件版本。
