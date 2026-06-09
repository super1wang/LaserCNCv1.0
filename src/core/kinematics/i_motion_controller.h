#pragma once

#include <QObject>
#include <QHash>
#include <QMap>
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

    /// 可选采集接口：返回控制器当前轴位置；未实现时返回空表。
    virtual QMap<QString, double> axisPositions() const { return {}; }

    /// 可选轴使能接口；未实现的控制器默认认为轴已使能。
    virtual bool setAxisEnabled(const QString& axis, bool enabled)
    {
        Q_UNUSED(axis);
        Q_UNUSED(enabled);
        return true;
    }
    virtual bool axisEnabled(const QString& axis) const
    {
        Q_UNUSED(axis);
        return true;
    }

    /// 可选回零状态接口；未实现时调用方按自身状态判断。
    virtual bool axisHomed(const QString& axis) const
    {
        Q_UNUSED(axis);
        return false;
    }

    /// 可选控制器 IO 接口；未实现时返回 false，调用方可回退到独立 IO 设备。
    virtual bool setDigitalOutput(const QString& channel, bool value, QString* errorMessage = nullptr)
    {
        Q_UNUSED(channel);
        Q_UNUSED(value);
        if (errorMessage)
            *errorMessage = QStringLiteral("motion controller digital output is not supported");
        return false;
    }
    virtual bool digitalInput(const QString& channel, bool* value, QString* errorMessage = nullptr) const
    {
        Q_UNUSED(channel);
        if (value)
            *value = false;
        if (errorMessage)
            *errorMessage = QStringLiteral("motion controller digital input is not supported");
        return false;
    }
    virtual bool setAnalogOutput(const QString& channel, double value, QString* errorMessage = nullptr)
    {
        Q_UNUSED(channel);
        Q_UNUSED(value);
        if (errorMessage)
            *errorMessage = QStringLiteral("motion controller analog output is not supported");
        return false;
    }
    virtual bool analogInput(const QString& channel, double* value, QString* errorMessage = nullptr) const
    {
        Q_UNUSED(channel);
        if (value)
            *value = 0.0;
        if (errorMessage)
            *errorMessage = QStringLiteral("motion controller analog input is not supported");
        return false;
    }

    /// 可选控制器程序接口；ACS/GTN 适配器可将已翻译程序装入缓冲区执行。
    virtual bool executeProgram(const QString& program,
                                int bufferIndex,
                                bool waitForFinish,
                                int timeoutMs,
                                QString* errorMessage = nullptr)
    {
        Q_UNUSED(program);
        Q_UNUSED(bufferIndex);
        Q_UNUSED(waitForFinish);
        Q_UNUSED(timeoutMs);
        if (errorMessage)
            *errorMessage = QStringLiteral("motion controller program execution is not supported");
        return false;
    }
    virtual bool programRunning(int bufferIndex, bool* running, QString* errorMessage = nullptr) const
    {
        Q_UNUSED(bufferIndex);
        if (running)
            *running = false;
        if (errorMessage)
            *errorMessage = QStringLiteral("motion controller program state is not supported");
        return false;
    }
    virtual bool supportsProgramPause() const
    {
        return false;
    }
    virtual bool pauseProgram(int bufferIndex, QString* errorMessage = nullptr)
    {
        Q_UNUSED(bufferIndex);
        if (errorMessage)
            *errorMessage = QStringLiteral("motion controller program pause is not supported");
        return false;
    }
    virtual bool resumeProgram(int bufferIndex, QString* errorMessage = nullptr)
    {
        Q_UNUSED(bufferIndex);
        if (errorMessage)
            *errorMessage = QStringLiteral("motion controller program resume is not supported");
        return false;
    }
};

} // namespace lcnc
