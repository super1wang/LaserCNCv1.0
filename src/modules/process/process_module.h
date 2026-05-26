#pragma once

#include <QObject>
#include <QList>
#include <QMap>
#include <QString>
#include <memory>

#include "core/kinematics/machine_kinematics.h"
#include "core/kernel/i_module.h"
#include "core/kernel/i_service.h"
#include "modules/process/i_process_facade.h"
#include "modules/process/settings/process_settings.h"

class QTimer;
class ProcessTreeView;

namespace lcnc::process {
class ProcessDeviceManager;
class SimulationMotionController;
}

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

    using State = lcnc::ProcessRunState;

    // ── IModule ─────────────────────────────────────────────────────
    /// id="process"，依赖 ["cam"]。
    lcnc::ModuleInfo info() const override;
    bool init(lcnc::IKernel& kernel) override;
    bool start() override;
    void stop() override;

    bool connectController(const QString& endpoint) override;
    void disconnectController() override;
    bool isConnected() const override;

    void setSimulationMode(bool on) override;
    bool simulationMode() const override;

    void setAxisDefinitions(const QList<MachineAxisDef>& axes);

    void jog(const QString& axisName, int direction, int speedLevel);
    void home() override;

    /// 启动加工运行（仿真或控制器）。
    /// @note 以 @c run 前缀区分于 @ref lcnc::IModule::start 生命周期调用。
    void runStart() override;
    /// 暂停当前运行。
    void runPause() override;
    /// 停止当前运行（不释放资源）。
    void runStop() override;
    void emergencyStop() override;
    /// 复位急停 —— 仅当当前状态为 EmergencyStop 时把状态切回 Idle 并刷新
    /// 状态栏；不重连控制器、不重置轴位置。
    void resetEmergencyStop() override;

    void newProcess() override;
    bool loadProcess(const QString& filePath) override;
    bool saveProcess(const QString& filePath) override;

    State state() const;
    QMap<QString, double> currentAxisPositions() const;
    void setAxisPosition(const QString& axisName, double value);
    void setAxisPositions(const QMap<QString, double>& positions);

    void setFeedOverride(double factor);
    double feedOverride() const;

    lcnc::ProcessSettings& settings() { return m_settings; }
    const lcnc::ProcessSettings& settings() const { return m_settings; }

    lcnc::process::ProcessDeviceManager* deviceManager() const { return m_deviceManager.get(); }
    void reloadDeviceSettings();

    void setProcessTreeView(ProcessTreeView* treeView);
    ProcessTreeView* processTreeView() const;

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
    ProcessTreeView* m_processTreeView{nullptr};
    lcnc::ProcessSettings m_settings;
    std::unique_ptr<lcnc::process::ProcessDeviceManager> m_deviceManager;
    std::unique_ptr<lcnc::process::SimulationMotionController> m_simController;
};
