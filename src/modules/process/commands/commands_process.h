#pragma once

#include "core/command/commands_api.h"

// ─────────────────────────────────────────────────────────────────────────────
// Process commands —— 加工运行 / 控制器连接 / 仿真模式
//
// 每个命令仅负责"对话/触发 → 转发到 IProcessFacade"，不直接持有
// 加工业务状态。需要参数交互的（如 jog、setFeedOverride）由执行面板
// 发出信号后交给应用层编排。
// ─────────────────────────────────────────────────────────────────────────────

namespace lcnc::process {

/// 启动加工运行（仿真或实控）。
class CmdRunStart : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "process.runStart";
    explicit CmdRunStart(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};

/// 暂停加工运行。
class CmdRunPause : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "process.runPause";
    explicit CmdRunPause(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};

/// 停止加工运行（不释放资源）。
class CmdRunStop : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "process.runStop";
    explicit CmdRunStop(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};

/// 紧急停止：立刻置为 EmergencyStop 状态。
class CmdEmergencyStop : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "process.emergencyStop";
    explicit CmdEmergencyStop(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};

/// 复位急停。
class CmdResetEmergencyStop : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "process.resetEmergencyStop";
    explicit CmdResetEmergencyStop(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};

/// 各轴回零。
class CmdHome : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "process.home";
    explicit CmdHome(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};

/// 通过对话框输入控制器地址并连接。
class CmdConnectController : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "process.connectController";
    explicit CmdConnectController(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};

/// 断开控制器连接。
class CmdDisconnectController : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "process.disconnectController";
    explicit CmdDisconnectController(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};

/// 切换仿真模式（Action 自身 checkable，会与模块信号双向同步）。
class CmdToggleSimulationMode : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "process.toggleSimulationMode";
    explicit CmdToggleSimulationMode(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};

} // namespace lcnc::process
