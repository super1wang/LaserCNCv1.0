#pragma once

#include <QObject>
#include <QList>
#include <QMap>
#include <QString>
#include <atomic>
#include <memory>

#include "core/kinematics/machine_kinematics.h"
#include "core/kernel/i_module.h"
#include "core/kernel/i_service.h"
#include "modules/process/cutting/process_cutting_plan_service.h"
#include "modules/process/i_process_facade.h"
#include "modules/process/steps/process_step_context.h"
#include "modules/process/workflow/process_flow_document.h"

class QTimer;
class Service;

namespace lcnc::process {
class CallbackProcessCuttingService;
class LegacyProcessIoService;
class LegacyProcessMotionService;
class NormalCuttingManager;
class ProcessCuttingPlanService;
class ProcessMonitorService;
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
 * 运动指令统一走 service->GetMotionControl()（仿真模式下由 MCFactory
 * 返回 SimulatorCMHP / ACS 仿真器）。
 */
/**
 * @brief 主界面 IO 栏一条数字量输出按钮的描述符。
 *
 * 由 ProcessModule 从 settings 的 `[Setting.Digital.DigitalOUT]` 子表中筛
 * 选出 `enabled && showInMain` 的项构造，并通过
 * digitalOutputDescriptorsChanged 信号广播给 UI 动态重建按钮。
 */
struct DigitalOutputDescriptor
{
    QString name;     ///< 显示文本（中文名）
    QString channel;  ///< toml key（如 "aLaser"），用于 setDigitalOutput 调用
    bool    active{true};    ///< 高/低电平有效；UI 仅展示，写入逻辑由 MotionControl 解析时处理
};

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
    void moveAxisAbsolute(const QString& axisName, double position, int speedLevel);
    void startContinuousJog(const QString& axisName, int direction, int speedLevel);
    void stopContinuousJog(const QString& axisName);
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

    /// 主界面 IO 栏要显示的数字量输出列表（来自 settings + 当前缓存值）。
    QList<DigitalOutputDescriptor> mainPanelDigitalOutputs() const;
    /// 在 settings 变更（设置对话框 Apply/OK）后调用，重新发射 IO 描述符 + 启动期反馈。
    void refreshIOFromSettings();

    /// NormalCuttingManager 在执行普通切割期间调用，旁路 onSimulationTick 的
    /// Lissajous 正弦波 + 硬件状态轮询，避免与刀路驱动写入 setAxisPosition 抢占。
    void setNormalCuttingActive(bool active);

    /// 暴露给 NormalCuttingManager 等需要工艺数据的内部组件。
    lcnc::process::ProcessCuttingPlanService* cuttingPlanService() const { return m_cuttingPlanService.get(); }

    // ── Ribbon「加工顺序」状态 ────────────────────────────────────────────
    /// 自动排序使用的主轴方向（Ribbon 下拉同步而来）。
    lcnc::process::AutoSortAxis autoSortAxis() const { return m_autoSortAxis; }
    void setAutoSortAxis(lcnc::process::AutoSortAxis a);
    /// 从 Ribbon QComboBox 当前文本（"X+"/"X-"/"Y+"/...）回写。
    void setAutoSortAxisFromText(const QString& text);

    /// 切割路径虚线显示开关；ProcessModule 仅维护状态，绘制在 CAM 端。
    bool isTravelPathVisible() const { return m_travelPathVisible; }
    void setTravelPathVisible(bool on);

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
    /// 主界面 IO 栏的数字量输出按钮列表已变更（settings 修改、初次加载等）。
    void digitalOutputDescriptorsChanged(const QList<DigitalOutputDescriptor>& descriptors);

private slots:
    void onSimulationTick();
    void pollHardwareStatus();          // 联机后周期性采集硬件轴位/使能

private:
    void initializeAxisPositions();
    void initializeAxisEnabledStates();
    void safeStopProcessOutputs();
    void triggerSafeStopOutputs();
    void setState(State state, const QString& statusMessage);
    void setStatusMessage(const QString& message);
    void seedDefaultIOTables();
    void startDeviceMonitoring();
    void stopDeviceMonitoring();
    bool validateProcessingEnvironment(QString* errorMessage);

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
    QTimer*               m_hwStatusTimer{nullptr};   ///< 硬件状态轮询（联机模式下生效）
    bool                  m_hwPollInFlight{false};    ///< 防止后台采集任务堆积
    double                m_feedOverride{1.0};
    double                m_simPhase{0.0};
    QString               m_statusMessage;
    lcnc::IKernel*        m_kernel{nullptr};
    lcnc::process::ProcessFlowDocument m_processFlowDocument;
    lcnc::process::ProcessStepContext m_stepContext;
    std::unique_ptr<Service> m_service;
    std::unique_ptr<lcnc::process::LegacyProcessMotionService> m_motionStepService;
    std::unique_ptr<lcnc::process::LegacyProcessIoService> m_ioStepService;
    std::unique_ptr<lcnc::process::CallbackProcessCuttingService> m_cuttingStepService;
    std::unique_ptr<lcnc::process::ProcessCuttingPlanService> m_cuttingPlanService;
    std::unique_ptr<lcnc::process::NormalCuttingManager> m_normalCuttingManager;
    std::unique_ptr<lcnc::process::ProcessMonitorService> m_monitorService;
    std::unique_ptr<lcnc::process::ProcessWorkflowExecutor> m_workflowExecutor;
    std::atomic_bool      m_normalCuttingActive{false};  ///< 见 setNormalCuttingActive

    // ── Ribbon「加工顺序」状态镜像 ────────────────────────────────────────
    lcnc::process::AutoSortAxis m_autoSortAxis{lcnc::process::AutoSortAxis::XPos};
    bool                        m_travelPathVisible{false};
};

Q_DECLARE_METATYPE(DigitalOutputDescriptor)
Q_DECLARE_METATYPE(QList<DigitalOutputDescriptor>)
