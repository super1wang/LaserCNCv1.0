#pragma once

#include "core/command/commands_api.h"

// ─────────────────────────────────────────────────────────────────────────────
// Process commands —— 加工运行 / 控制器连接 / 参数设置
//
// 每个命令仅负责"对话/触发 → 转发到 IProcessFacade"，不直接持有
// 加工业务状态。需要参数交互的（如 jog、setFeedOverride）由执行面板
// 发出信号后交给应用层编排。
// ─────────────────────────────────────────────────────────────────────────────

namespace lcnc::process {

/// 新建流程树。
class CmdNewProcess : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "process.newProcess";
    explicit CmdNewProcess(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};

/// 从 TOML 文件加载流程树。
class CmdLoadProcess : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "process.loadProcess";
    explicit CmdLoadProcess(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};

/// 保存当前流程树为 TOML 文件。
class CmdSaveProcess : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "process.saveProcess";
    explicit CmdSaveProcess(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};

/// 打开统一参数设置对话框（qg_dlgsetting）。
class CmdOpenProcessSettings : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "process.settings";
    explicit CmdOpenProcessSettings(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};

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

/// 异步连接当前已启用的全部外设（运动控制器、激光器等）。
class CmdConnectController : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "process.connectController";
    explicit CmdConnectController(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};

/// 异步断开当前已连接的全部外设。
class CmdDisconnectController : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "process.disconnectController";
    explicit CmdDisconnectController(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};

/// 把当前选中的轮廓按选择顺序追加到切割链表（手动设置加工顺序）。
class CmdManualAppendSelectedToCuttingOrder : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "process.manualAppendSelected";
    explicit CmdManualAppendSelectedToCuttingOrder(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};

/// 按 ProcessModule::autoSortAxis 当前选项做"主方向 + 最近邻"自动排序。
class CmdAutoSortCuttingOrder : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "process.autoSortCutting";
    explicit CmdAutoSortCuttingOrder(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};

/// 切换"切割路径显示"——在 OCC 视图中用虚线绘制空程路径。
class CmdToggleTravelPath : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "process.toggleTravelPath";
    explicit CmdToggleTravelPath(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};

} // namespace lcnc::process
