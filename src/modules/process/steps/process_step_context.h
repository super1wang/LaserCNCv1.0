#pragma once

#include "modules/process/execution/process_workflow_executor.h"
#include "modules/process/runtime/process_cancellation_token.h"
#include "modules/process/runtime/process_execution_context.h"
#include "modules/process/runtime/process_interrupt_context.h"

#include <QMap>
#include <QString>
#include <QVariant>
#include <functional>

namespace lcnc::process {

class IProcessMotionService
{
public:
    virtual ~IProcessMotionService() = default;
    virtual bool moveAxis(const QString& axis,
                          const QString& mode,
                          double target,
                          double velocity,
                          int timeoutMs,
                          QString* errorMessage) = 0;
    virtual bool moveAxes(const QVariantList& axes,
                          const QString& mode,
                          int timeoutMs,
                          QString* errorMessage) = 0;
    virtual bool stopMotion(QString* errorMessage = nullptr) = 0;
};

class IProcessIoService
{
public:
    virtual ~IProcessIoService() = default;
    virtual bool setOutput(const QString& signalType,
                           const QString& ioName,
                           const QVariant& value,
                           QString* errorMessage) = 0;
    virtual bool waitInput(const QString& signalType,
                           const QString& ioName,
                           const QVariant& targetValue,
                           int timeoutMs,
                           int pollIntervalMs,
                           QString* errorMessage) = 0;
    /// 读取一次输入信号当前值（digital -> bool，analog -> double），不阻塞等待。
    virtual bool readInput(const QString& signalType,
                           const QString& ioName,
                           QVariant* value,
                           QString* errorMessage) = 0;
};

class IProcessCuttingService
{
public:
    virtual ~IProcessCuttingService() = default;
    virtual ProcessToolpathSnapshot toolpathSnapshot() const = 0;
    virtual bool executeNormalCutting(const QString& nodeId,
                                      const QVariantMap& parameters,
                                      ProcessInterruptContext* interrupt,
                                      QString* errorMessage) = 0;
};

struct ProcessStepContext
{
    ProcessExecutionContext* executionContext{nullptr};
    IProcessMotionService* motion{nullptr};
    IProcessIoService* io{nullptr};
    IProcessCuttingService* cutting{nullptr};
    // 统一中断上下文：所有步骤通过 interrupt->checkpoint(nodeId, label, env)
    // 注册可暂停/可停止位置；同时支持按 nodeId 取回最后一次断点环境用于断点续跑。
    // `cancellationToken` 是历史命名，与 interrupt 指向同一对象，保留以兼容旧代码。
    ProcessInterruptContext* interrupt{nullptr};
    ProcessCancellationToken* cancellationToken{nullptr};
    // 流程级变量环境（由执行器在 start 时从 Start 节点声明播种），If 条件可读取。
    QMap<QString, QVariant>* variables{nullptr};
    // If 节点执行后写入的条件结果（nodeId -> true/false），执行器据此跳过 false 分支的子节点。
    QMap<QString, bool>* ifResults{nullptr};
    std::function<void(const QString&)> logMessage;
    std::function<void(const QString&, double)> requestAxisPosition;
    std::function<void(const QString&, bool)> requestDigitalOutput;
};

} // namespace lcnc::process
