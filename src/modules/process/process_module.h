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
#include "modules/process/workflow/process_flow_document.h"

class QTimer;
class Service;

namespace lcnc::process {
class SimulationMotionController;
class ProcessWorkflowExecutor;
}

namespace lcnc {
class IKernel;
class MachineConfigurationService;
}

/**
 * @brief Process module — manages execution process, peripherals, and parameters.
 *
 * 使用旧 System/Service 类作为外设和参数统一管理器，
 * 通过唯一的 qg_dlgsetting 对话框提供共同参数界面。
 * 运动仿真依赖 SimulatorCmhpMotionController。
 */
class ProcessModule : public QObject, public lcnc::IModule, public lcnc::IProcessFacade
{
    Q_OBJECT
public:
    explicit ProcessModule(QObject* parent = nullptr);

    /// IProcessFacade：用于让调用方挂接 ProcessModule 的 Qt 信号。
    QObject* asQObject() override { return this; }

    using State = lcnc::ProcessRunState;

    // ── IModule ─────────────────────────────────────────────────────
    lcnc::ModuleInfo info() const override;
    bool init(lcnc::IKernel& kernel) override;
    bool start() override;
    void stop() override;

    bool connectController(const QString& endpoint) override;
    void disconnectController() override;
    bool isConnected() const override;

    void connectAllDevices() override;
    void disconnectAllDevices() override;

    void setSimulationMode(bool on) override;
    bool simulationMode() const override;

    void setAxisDefinitions(const QList<MachineAxisDef>& axes);

    void jog(const QString& axisName, int direction, int speedLevel, double distance = 0.0);
    void setAxisEnabled(const QString& axisName, bool enabled);
    QMap<QString, bool> axisEnabledStates() const { return m_axisEnabled; }
    void setDigitalOutput(const QString& outputName, bool value);
    QMap<QString, bool> digitalOutputStates() const { return m_digitalOutputs; }
    void home() override;

    void runStart() override;
    void runPause() override;
    void runStop() override;
    void emergencyStop() override;
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

    lcnc::process::ProcessFlowDocument& processFlowDocument() { return m_processFlowDocument; }
    const lcnc::process::ProcessFlowDocument& processFlowDocument() const { return m_processFlowDocument; }
    Service* service() const { return m_service.get(); }
    QString statusMessage() const override;

signals:
    void connectionChanged(bool connected);
    void simulationModeChanged(bool enabled);
    void stateChanged(State state);
    void axisPositionChanged(const QString& axisName, double value);
    void axisEnabledChanged(const QString& axisName, bool enabled);
    void digitalOutputChanged(const QString& outputName, const QString& channel, bool value);
    void feedOverrideChanged(double factor);
    void statusMessageChanged(const QString& message);
    void processLogMessage(const QString& level, const QString& message);
    void processFlowChanged();
    /// 单个外设连接进度（设备名、百分比、当前步骤描述）。
    void deviceConnectProgress(const QString& deviceName, int percent, const QString& step);
    /// 全部外设连接/断开完成（是否全部成功、汇总消息）。
    void deviceConnectFinished(bool allSuccess, const QString& summary);

private slots:
    void onSimulationTick();

private:
    void initializeAxisPositions();
    void initializeAxisEnabledStates();
    void safeStopProcessOutputs();
    void setState(State state, const QString& statusMessage);
    void setStatusMessage(const QString& message);

    bool                  m_initialized{false};
    bool                  m_connected{false};
    bool                  m_simulationMode{true};
    bool                  m_homing{false};
    State                 m_state{State::Idle};
    QList<MachineAxisDef> m_axisDefinitions;
    QMap<QString, double> m_axisPositions;
    QMap<QString, bool>   m_axisEnabled;
    QMap<QString, bool>   m_digitalOutputs;
    QTimer*               m_simTimer{nullptr};
    double                m_feedOverride{1.0};
    double                m_simPhase{0.0};
    QString               m_statusMessage;
    lcnc::IKernel*        m_kernel{nullptr};
    lcnc::process::ProcessFlowDocument m_processFlowDocument;
    std::unique_ptr<Service> m_service;
    std::unique_ptr<lcnc::process::SimulationMotionController> m_motionController;
    std::unique_ptr<lcnc::process::ProcessWorkflowExecutor> m_workflowExecutor;
};
