#pragma once

// WebServer.hpp
// 对外暴露 WebServer 相关函数：初始化、周期性清理以及向所有客户端广播状态。

// 初始化 WebServer（注册 WS、路由并启动服务）
void initWeb();

// 清理 WebSocket 客户端（应在主循环中定期调用）
void webCleanup();

// 向所有已连接客户端推送当前状态（JSON）
void sendStatusToClients();
