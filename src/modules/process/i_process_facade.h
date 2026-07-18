#pragma once

#include "core/kernel/i_service.h"

#include <QString>

class QObject;

namespace lcnc {

enum class ProcessRunState {
    Idle,
    Running,
    Paused,
    Error,
    EmergencyStop,
};

/**
 * @brief Process（激光加工执行）模块对外门面接口。
 *
 * 暴露控制器连接、仿真模式开关、运行/暂停/停止以及状态信息读取等
 * UI 实际触达的动作；具体的运动学控制仍走模块内部控制器。
 */
class IProcessFacade : public IService
{
public:
    ~IProcessFacade() override = default;

    /// 用于让调用方挂接 ProcessModule 的 Qt 信号（statusMessageChanged 等）。
    virtual QObject* asQObject() = 0;

    virtual bool isConnected() const = 0;

    /// 异步连接全部已配置的外设（运动控制器、激光器等）。
    /// 通过 TaskManager 在线程池中执行，进度通过 ProcessModule 信号上报。
    virtual void connectAllDevices() = 0;
    /// 异步断开全部已连接的外设。
    virtual void disconnectAllDevices() = 0;

    /// 仿真模式开关（true 表示纯软件仿真，不发送下位机指令）。
    virtual bool simulationMode() const = 0;
    virtual void setSimulationMode(bool enabled) = 0;

    virtual ProcessRunState state() const = 0;

    /// 加工流程：开始 / 暂停 / 停止。
    virtual void runStart() = 0;
    virtual void runPause() = 0;
    virtual void runStop()  = 0;
    virtual void emergencyStop() = 0;
    virtual void resetEmergencyStop() = 0;
    virtual void home() = 0;

    virtual void newProcess() = 0;
    virtual bool loadProcess(const QString& filePath) = 0;
    virtual bool saveProcess(const QString& filePath) = 0;

    /// 当前状态描述（用于状态栏）。
    virtual QString statusMessage() const = 0;
};

} // namespace lcnc
