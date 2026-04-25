#pragma once

#include <QObject>

#include "core/kinematics/i_motion_controller.h"

namespace lcnc { class MachinePose; }

namespace lcnc::process {

/**
 * @brief 仿真模式下的运动控制器实现。
 *
 * 行为：
 *   - jog/moveTo 直接写入 @ref MachinePose（GUI 线程同步），由 pose 信号驱动 CAM
 *     做局部 AIS 刷新（参见 CamModule 中的 16ms coalescer）；
 *   - home 把所有轴值清零；
 *   - emergencyStop 阻断后续指令直到 start() 重新启用；
 *   - 不直接持有 ToolpathSimulator —— 刀路仿真的播放/暂停仍走 CAM 命令；
 *     该播放过程内部最终也会调用 setAxisPosition → MachinePose，
 *     与本控制器位于同一姿态总线，自然无冲突。
 *
 * 该类位于 modules/process/controllers/，明确把"运动指令入口"
 * 归属到 process 模块，避免 cam 模块反向感知控制器。
 */
class SimulationMotionController : public QObject, public lcnc::IMotionController
{
    Q_OBJECT
public:
    explicit SimulationMotionController(QObject* parent = nullptr);
    ~SimulationMotionController() override;

    QString id() const override { return QStringLiteral("sim"); }

    bool start() override;
    void stop() override;
    bool isRunning() const override { return m_running; }

    bool jog(const QString& axis, double delta) override;
    bool moveTo(const QString& axis, double absolutePos) override;
    bool home(const QString& axis = QString()) override;
    void emergencyStop() override;

private:
    /// 取共享 MachinePose（首次调用时通过 Kernel::services() 解析）。
    lcnc::MachinePose* pose() const;

private:
    bool m_running{false};
    bool m_estop{false};
    mutable lcnc::MachinePose* m_poseCache{nullptr};
};

} // namespace lcnc::process
