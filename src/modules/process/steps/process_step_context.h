#pragma once

#include "modules/process/execution/process_workflow_executor.h"
#include "modules/process/runtime/process_execution_context.h"

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
};

class IProcessCuttingService
{
public:
    virtual ~IProcessCuttingService() = default;
    virtual ProcessToolpathSnapshot toolpathSnapshot() const = 0;
    virtual bool executeNormalCutting(const QVariantMap& parameters,
                                      QString* errorMessage) = 0;
};

struct ProcessStepContext
{
    ProcessExecutionContext* executionContext{nullptr};
    IProcessMotionService* motion{nullptr};
    IProcessIoService* io{nullptr};
    IProcessCuttingService* cutting{nullptr};
    std::function<void(const QString&)> logMessage;
    std::function<void(const QString&, double)> requestAxisPosition;
    std::function<void(const QString&, bool)> requestDigitalOutput;
};

} // namespace lcnc::process
