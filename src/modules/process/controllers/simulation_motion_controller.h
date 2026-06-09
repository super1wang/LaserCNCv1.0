#pragma once

#include <QMap>
#include <QMutex>
#include <QObject>
#include <QSet>

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
class SimulationMotionController : public QObject
{
    Q_OBJECT
public:
    explicit SimulationMotionController(QObject* parent = nullptr);
    ~SimulationMotionController() override;

    virtual QString id() const { return QStringLiteral("sim"); }

    bool start();
    void stop();
    bool isRunning() const { return m_running; }

    bool jog(const QString& axis, double delta);
    bool moveTo(const QString& axis, double absolutePos);
    bool home(const QString& axis = QString());
    void emergencyStop();
    QMap<QString, double> axisPositions() const;
    bool setAxisEnabled(const QString& axis, bool enabled);
    bool axisEnabled(const QString& axis) const;
    bool axisHomed(const QString& axis) const;
    bool setDigitalOutput(const QString& channel, bool value, QString* errorMessage = nullptr);
    bool digitalInput(const QString& channel, bool* value, QString* errorMessage = nullptr) const;
    bool setAnalogOutput(const QString& channel, double value, QString* errorMessage = nullptr);
    bool analogInput(const QString& channel, double* value, QString* errorMessage = nullptr) const;
    bool executeProgram(const QString& program,
                        int bufferIndex,
                        bool waitForFinish,
                        int timeoutMs,
                        QString* errorMessage = nullptr);
    bool programRunning(int bufferIndex, bool* running, QString* errorMessage = nullptr) const;
    bool supportsProgramPause() const { return true; }
    bool pauseProgram(int bufferIndex, QString* errorMessage = nullptr);
    bool resumeProgram(int bufferIndex, QString* errorMessage = nullptr);

private:
    /// 取共享 MachinePose（首次调用时通过 Kernel::services() 解析）。
    lcnc::MachinePose* pose() const;

private:
    bool m_running{false};
    bool m_estop{false};
    QSet<QString> m_disabledAxes;
    mutable QMutex m_ioMutex;
    QMap<QString, bool> m_digitalValues;
    QMap<QString, double> m_analogValues;
    QMap<int, QString> m_loadedPrograms;
    QSet<int> m_runningPrograms;
    QSet<int> m_pausedPrograms;
    mutable lcnc::MachinePose* m_poseCache{nullptr};
};

} // namespace lcnc::process
