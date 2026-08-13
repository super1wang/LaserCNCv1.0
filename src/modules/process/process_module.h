#pragma once

#include <QObject>
#include <QList>
#include <QMap>
#include <QSet>
#include <QString>
#include <atomic>
#include <cstdint>
#include <memory>

#include "core/kinematics/machine_kinematics.h"
#include "core/kernel/i_module.h"
#include "core/kernel/i_service.h"
#include "core/task/task_manager.h"
#include "core/task/module_task_scope.h"
#include "modules/process/cutting/process_cutting_plan_service.h"
#include "modules/process/i_process_facade.h"
#include "modules/process/runtime/process_run_coordinator.h"
#include "modules/process/steps/process_step_context.h"
#include "modules/process/runtime/process_runtime_configuration.h"

class QTimer;
class ProcessDeviceRuntime;

namespace lcnc::process {
class CallbackProcessCuttingService;
class ProcessIoWorkflowService;
class ProcessMotionWorkflowService;
class NormalCuttingManager;
class ProcessCuttingPlanService;
class ProcessMonitorService;
class ProcessPreflightService;
class ProcessConnectionService;
class ProcessManualMotionService;
class ProcessInteractiveIoService;
class ProcessStatusService;
class ProcessWorkflowExecutor;
class ProcessSettingsService;
class ProcessWorkflowService;
class DeviceCommandQueue;
struct DeviceCommandResult;
struct DeviceStatusSnapshot;
struct DevicePeripheralSnapshot;
struct ProcessSettingsChangeSet;
}

namespace lcnc {
class IKernel;
class MachineConfigurationService;
}

/**
 * @brief Process module — manages execution process, peripherals, and parameters.
 *
 * 使用 System/ProcessDeviceRuntime 作为现有设备适配器，参数界面由动态属性表提供。
 * 运动、IO 与设备生命周期指令经 ProcessDeviceRuntime 的 typed 命令面和
 * DeviceCommandQueue 执行；仿真控制器由 runtime 选择。
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
    ~ProcessModule() override;

    /// IProcessFacade：用于让调用方挂接 ProcessModule 的 Qt 信号。
    QObject* asQObject() override { return this; }

    using State = lcnc::ProcessRunState;

    // ── IModule ─────────────────────────────────────────────────────
    lcnc::ModuleInfo info() const override;
    bool init(lcnc::IKernel& kernel) override;
    bool start() override;
    void stop() override;

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
    void moveToConfiguredPosition(bool loading);

    void runStart() override;
    void runPause() override;
    void runStop() override;
    void resetStop() override;

    void newProcess() override;
    bool loadProcess(const QString& filePath) override;
    bool saveProcess(const QString& filePath) override;

    State state() const;
    QMap<QString, double> currentAxisPositions() const;
    void setAxisPosition(const QString& axisName, double value);
    void setAxisPositions(const QMap<QString, double>& positions);

    /// Re-publish the latest controller feedback after a view rebuild and ask
    /// the active controller for a fresh sample.  This never derives values
    /// from project/file state or machine configuration.
    void synchronizeAxisFeedback();

    void setFeedOverride(double factor);
    double feedOverride() const;

    lcnc::process::ProcessSettingsService* settingsService() const { return m_settingsService.get(); }
    QString statusMessage() const override;

    /// 主界面 IO 栏要显示的数字量输出列表（来自 settings + 当前缓存值）。
    QList<DigitalOutputDescriptor> mainPanelDigitalOutputs() const;
    /// 在 settings 变更（设置对话框 Apply/OK）后调用，重新发射 IO 描述符 + 启动期反馈。
    void refreshIOFromSettings();
    /// Queues committed settings for background application to Process devices.
    void applySettingsChanges(const lcnc::process::ProcessSettingsChangeSet& changes);

    /// NormalCuttingManager 在执行普通切割期间调用，旁路 onSimulationTick 的
    /// Lissajous 正弦波 + 硬件状态轮询，避免与刀路驱动写入 setAxisPosition 抢占。
    /// 仅写入原子标志，可从工作流线程调用。
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

    /// 切割链表序号显示开关；ProcessModule 仅维护状态，绘制在 CAM 端。
    bool isContourOrderLabelVisible() const { return m_contourOrderLabelVisible; }
    void setContourOrderLabelVisible(bool on);

signals:
    void connectionChanged(bool connected);
    void simulationModeChanged(bool enabled);
    void stateChanged(State state);
    void axisPositionChanged(const QString& axisName, double value);
    void axisEnabledChanged(const QString& axisName, bool enabled);
    void digitalOutputChanged(const QString& outputName, const QString& channel, bool value);
    void feedOverrideChanged(double factor);
    void statusMessageChanged(const QString& message);
    /// 新加工运行已开始；暂停恢复不会发射此信号。
    void processingRunStarted();
    /// 普通切割轮廓进度：已完成轮廓数、总轮廓数。
    void processingProgressChanged(int completedContours, int totalContours);
    void processLogMessage(const QString& level, const QString& message);
    void processFlowChanged();
    /// 单个外设连接进度（设备名、百分比、当前步骤描述）。
    void deviceConnectProgress(const QString& deviceName, int percent, const QString& step);
    /// 全部外设连接/断开完成（是否全部成功、汇总消息）。
    void deviceConnectFinished(bool allSuccess, const QString& summary);
    /// 主界面 IO 栏的数字量输出按钮列表已变更（settings 修改、初次加载等）。
    void digitalOutputDescriptorsChanged(const QList<DigitalOutputDescriptor>& descriptors);
    /// Low-frequency serial/peripheral health snapshot.  Emitted on the GUI
    /// thread after polling completes on the peripheral worker.
    void peripheralStatusChanged(const QString& deviceName,
                                 bool connected,
                                 bool initialized,
                                 const QString& diagnostic);

private slots:
    void onSimulationTick();

private:
    enum class DeviceOperation { None, Connecting, Disconnecting, Homing, PresetMove };
    enum class StopOutcome { Stopped, Error, Idle };

    void initializeAxisPositions();
    void initializeAxisEnabledStates();
    void clearSafeOutputCache();
    /// The sole software stop path for operator Stop, workflow faults and completion.
    void requestStop(StopOutcome outcome, const QString& statusMessage);
    void setState(State state, const QString& statusMessage);
    void setStatusMessage(const QString& message);
    void startDeviceMonitoring();
    void stopDeviceMonitoring();
    void applyHardwareStatus(const lcnc::process::DeviceCommandResult& result,
                             const lcnc::process::DeviceStatusSnapshot& batch);
    void applyPeripheralStatus(const lcnc::process::DeviceCommandResult& result,
                               const lcnc::process::DevicePeripheralSnapshot& sample);
    bool validateProcessingConfiguration(QString* errorMessage, bool requireIdle = true);
    void startWorkflowAfterPreflight();
    /// 请求本模块任务取消并等待；false 表示仍有 worker 未在期限内退出。
    bool cancelOwnedTasks(int timeoutMs);

    bool                  m_initialized{false};
    bool                  m_connected{false};
#if (defined(LCNC_PROCESS_HAS_ACS) && LCNC_PROCESS_HAS_ACS) || \
    (defined(LCNC_PROCESS_HAS_GTN) && LCNC_PROCESS_HAS_GTN)
    bool                  m_simulationMode{false};
#else
    bool                  m_simulationMode{true};
#endif
    bool                  m_homing{false};
    bool                  m_preflightInFlight{false};
    bool                  m_stopInFlight{false};
    bool                  m_stopRecoveryRequired{false};
    bool                  m_stopRecoveryInFlight{false};
    StopOutcome           m_pendingStopOutcome{StopOutcome::Stopped};
    QString               m_pendingStopMessage;
    std::uint64_t         m_runRequestGeneration{0};
    State                 m_state{State::Idle};
    lcnc::process::ProcessRunCoordinator m_runCoordinator;
    QList<MachineAxisDef> m_axisDefinitions;
    QMap<QString, double> m_axisPositions;
    QMap<QString, bool>   m_axisEnabled;
    /// Non-interpolated rotary axes that must remain at their mode posture
    /// throughout the active workflow.
    QMap<QString, double> m_activeLockedAxisTargets;
    QMap<QString, bool>   m_digitalOutputs;
    QTimer*               m_simTimer{nullptr};
    QString               m_lastPeripheralDiagnostic;
    double                m_feedOverride{1.0};
    double                m_simPhase{0.0};
    QString               m_statusMessage;
    lcnc::IKernel*        m_kernel{nullptr};
    lcnc::process::ProcessStepContext m_stepContext;
    // Background connect/disconnect/home tasks retain a shared ProcessDeviceRuntime lease so
    // a bounded module shutdown cannot destroy vendor objects while an SDK call
    // is still returning.
    std::unique_ptr<lcnc::process::ProcessSettingsService> m_settingsService;
    // Must outlive ProcessDeviceRuntime and every controller which borrows these runtime facts.
    lcnc::process::ProcessRuntimeConfiguration m_runtimeConfiguration;
    // Declared before ProcessDeviceRuntime so reverse member destruction releases the
    // ProcessDeviceRuntime (which borrows this settings object) first.
    std::shared_ptr<ProcessDeviceRuntime> m_service;
    // Serializes all newly migrated vendor device operations on one dedicated
    // thread. ProcessDeviceRuntime ownership remains here until the remaining call paths
    // have moved behind this boundary.
    std::unique_ptr<lcnc::process::DeviceCommandQueue> m_deviceCommandQueue;
    std::shared_ptr<lcnc::process::ProcessPreflightService> m_preflightService;
    std::unique_ptr<lcnc::process::ProcessConnectionService> m_connectionService;
    std::unique_ptr<lcnc::process::ProcessManualMotionService> m_manualMotionService;
    std::unique_ptr<lcnc::process::ProcessInteractiveIoService> m_interactiveIoService;
    std::unique_ptr<lcnc::process::ProcessStatusService> m_statusService;
    std::unique_ptr<lcnc::process::ProcessMotionWorkflowService> m_motionStepService;
    std::unique_ptr<lcnc::process::ProcessIoWorkflowService> m_ioStepService;
    std::unique_ptr<lcnc::process::CallbackProcessCuttingService> m_cuttingStepService;
    std::unique_ptr<lcnc::process::ProcessCuttingPlanService> m_cuttingPlanService;
    std::unique_ptr<lcnc::process::NormalCuttingManager> m_normalCuttingManager;
    std::unique_ptr<lcnc::process::ProcessMonitorService> m_monitorService;
    std::unique_ptr<lcnc::process::ProcessWorkflowExecutor> m_workflowExecutor;
    std::unique_ptr<lcnc::process::ProcessWorkflowService> m_workflowService;
    std::atomic_bool      m_normalCuttingActive{false};  ///< 见 setNormalCuttingActive
    lcnc::ModuleTaskScope m_taskScope;
    DeviceOperation       m_deviceOperation{DeviceOperation::None};

    // ── Ribbon「加工顺序」状态镜像 ────────────────────────────────────────
    lcnc::process::AutoSortAxis m_autoSortAxis{lcnc::process::AutoSortAxis::XPos};
    bool                        m_travelPathVisible{false};
    bool                        m_contourOrderLabelVisible{false};
};

Q_DECLARE_METATYPE(DigitalOutputDescriptor)
Q_DECLARE_METATYPE(QList<DigitalOutputDescriptor>)
