#pragma once

#include "core/kernel/i_service.h"

#include <QString>

class QObject;

namespace lcnc {

/**
 * @brief Process（激光加工执行）模块对外门面接口（Phase 7）。
 *
 * 暴露控制器连接、仿真模式开关、运行/暂停/停止以及状态信息读取等
 * UI 实际触达的动作；具体的运动学控制仍走 @c ProcessModule 私有 API。
 * 本接口继承 @ref IService；ProcessModule 不再直接继承 IService 以避免多重继承。
 */
class IProcessFacade : public IService
{
public:
    ~IProcessFacade() override = default;

    /// 用于让调用方挂接 ProcessModule 的 Qt 信号（statusMessageChanged 等）。
    virtual QObject* asQObject() = 0;

    /// 连接到指定控制器端点（形如 "tcp://127.0.0.1:5000"）。
    /// 返回是否连接成功。
    virtual bool connectController(const QString& endpoint) = 0;
    /// 断开当前控制器连接。
    virtual void disconnectController() = 0;

    /// 仿真模式开关（true 表示纯软件仿真，不发送下位机指令）。
    virtual bool simulationMode() const = 0;
    virtual void setSimulationMode(bool enabled) = 0;

    /// 加工流程：开始 / 暂停 / 停止。
    virtual void runStart() = 0;
    virtual void runPause() = 0;
    virtual void runStop()  = 0;

    /// 当前状态描述（用于状态栏）。
    virtual QString statusMessage() const = 0;
};

} // namespace lcnc
