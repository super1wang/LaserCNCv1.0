#pragma once

#include <QObject>
#include <QList>
#include <QMap>
#include <QString>

#include "core/kinematics/machine_kinematics.h"
#include "core/kernel/i_module.h"
#include "core/kernel/i_service.h"
#include "modules/process/i_process_facade.h"
#include "modules/process/settings/process_settings.h"

class QTimer;

/**
 * @brief Process module singleton — manages execution process, peripherals, and parameters.
 *
 * Phase 5 is still in progress, so this module currently provides a thin
 * placeholder API and state container. Ribbon/UI should depend on this module
 * instead of embedding process logic directly.
 *
 * 微内核集成：同样实现 @ref lcnc::IModule + @ref lcnc::IService，依赖 "cam"
 * 以获取机台轴定义。
 */
class ProcessModule : public QObject, public lcnc::IModule, public lcnc::IProcessFacade
{
    Q_OBJECT
public:
    /// 公开构造：由 Kernel 拥有。
    explicit ProcessModule(QObject* parent = nullptr);

    /// IProcessFacade：用于让调用方挂接 ProcessModule 的 Qt 信号。
    QObject* asQObject() override { return this; }

    enum class State {
        Idle,
        Running,
        Paused,
        Error,
        EmergencyStop,
    };

    // ── IModule ─────────────────────────────────────────────────────
    /// id="process"，依赖 ["cam"]。
    lcnc::ModuleInfo info() const override;
    bool init(lcnc::IKernel& kernel) override;
    bool start() override;
    void stop() override;

    bool connectController(const QString& endpoint) override;
    void disconnectController() override;
    bool isConnected() const;

    void setSimulationMode(bool on) override;
    bool simulationMode() const override;

    void setAxisDefinitions(const QList<MachineAxisDef>& axes);

    void jog(const QString& axisName, int direction, int speedLevel);
    void home();

    /// 启动加工运行（仿真或控制器）。
    /// @note 以 @c run 前缀区分于 @ref lcnc::IModule::start 生命周期调用。
    void runStart() override;
    /// 暂停当前运行。
    void runPause() override;
    /// 停止当前运行（不释放资源）。
    void runStop() override;
    void emergencyStop();

    State state() const;
    QMap<QString, double> currentAxisPositions() const;
    void setAxisPosition(const QString& axisName, double value);

    void setFeedOverride(double factor);
    double feedOverride() const;

    QString statusMessage() const override;

signals:
    void connectionChanged(bool connected);
    void simulationModeChanged(bool enabled);
    void stateChanged(State state);
    void axisPositionChanged(const QString& axisName, double value);
    void feedOverrideChanged(double factor);
    void statusMessageChanged(const QString& message);

private slots:
    void onSimulationTick();

private:
    void initializeAxisPositions();
    void setState(State state, const QString& statusMessage);
    void setStatusMessage(const QString& message);

    /// 生命周期由 Kernel 接管。
    bool                  m_initialized{false};

    bool m_connected{false};
    bool m_simulationMode{true};
    State m_state{State::Idle};
    QList<MachineAxisDef> m_axisDefinitions;
    QMap<QString, double> m_axisPositions;
    QTimer* m_simTimer{nullptr};
    double m_feedOverride{1.0};
    double m_simPhase{0.0};
    QString m_statusMessage;
    lcnc::ProcessSettings m_settings;
};
