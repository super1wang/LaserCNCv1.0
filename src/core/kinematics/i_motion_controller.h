#pragma once

#include <QObject>
#include <QHash>
#include <QString>

#include "core/kernel/i_service.h"

namespace lcnc {

/**
 * @brief 运动控制器抽象接口（仿真控制器 / 真实硬件控制器统一外观）。
 *
 * 设计原则：
 *   - 零 OCC 几何 / 零 Qt UI 依赖；只描述 jog / 绝对移动 / 回零 / 急停 这几件事；
 *   - 控制器是"姿态的唯一写入方"——所有运动结果都体现为 @ref MachinePose 上的轴值更新；
 *   - 通过 IService 暴露给 Kernel；同一时刻只允许一个活动控制器（仿真 OR 硬件）。
 *
 * QObject 信号通过派生类 @ref MotionControllerBase 暴露，避免本接口被
 * 强制成为 QObject（保持 IService 的纯虚特性）。
 */
class IMotionController : public IService
{
public:
    ~IMotionController() override = default;

    /// 控制器内部 id（"sim" / "grbl" / ...）。
    virtual QString id() const = 0;

    /// 启用控制器（建立连接 / 启动仿真线程等）。返回 false 表示失败。
    virtual bool start() = 0;

    /// 停用控制器（断开连接 / 停止仿真）。
    virtual void stop() = 0;

    /// 当前是否处于运行/连接状态。
    virtual bool isRunning() const = 0;

    /// 增量 jog；delta 单位与轴一致（mm 或 °）。
    virtual bool jog(const QString& axis, double delta) = 0;

    /// 绝对位置移动。
    virtual bool moveTo(const QString& axis, double absolutePos) = 0;

    /// 回零；axis 为空表示所有轴。
    virtual bool home(const QString& axis = QString()) = 0;

    /// 急停（立刻清状态，禁止后续运动直至 reset）。
    virtual void emergencyStop() = 0;
};

} // namespace lcnc
