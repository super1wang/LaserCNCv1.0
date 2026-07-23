#pragma once

#include "modules/process/steps/process_step_context.h"

class Service;

namespace lcnc::process {

class DeviceCommandQueue;

class LegacyProcessMotionService final : public IProcessMotionService
{
public:
    explicit LegacyProcessMotionService(Service* service = nullptr,
                                        DeviceCommandQueue* deviceQueue = nullptr);
    void setService(Service* service) { m_service = service; }
    void setDeviceCommandQueue(DeviceCommandQueue* deviceQueue) { m_deviceQueue = deviceQueue; }

    bool moveAxis(const QString& axis,
                  const QString& mode,
                  double target,
                  double velocity,
                  int timeoutMs,
                  QString* errorMessage) override;
    bool moveAxes(const QVariantList& axes,
                  const QString& mode,
                  int timeoutMs,
                  QString* errorMessage) override;
    bool stopMotion(QString* errorMessage = nullptr) override;

private:
    Service* m_service{nullptr};
    DeviceCommandQueue* m_deviceQueue{nullptr};
};

class LegacyProcessIoService final : public IProcessIoService
{
public:
    explicit LegacyProcessIoService(Service* service = nullptr,
                                    DeviceCommandQueue* deviceQueue = nullptr);
    void setService(Service* service) { m_service = service; }
    void setDeviceCommandQueue(DeviceCommandQueue* deviceQueue) { m_deviceQueue = deviceQueue; }

    bool setOutput(const QString& signalType,
                   const QString& ioName,
                   const QVariant& value,
                   QString* errorMessage) override;
    bool waitInput(const QString& signalType,
                   const QString& ioName,
                   const QVariant& targetValue,
                   int timeoutMs,
                   int pollIntervalMs,
                   QString* errorMessage) override;

private:
    Service* m_service{nullptr};
    DeviceCommandQueue* m_deviceQueue{nullptr};
};

class CallbackProcessCuttingService final : public IProcessCuttingService
{
public:
    using ExecutorFn = std::function<bool(const QString& nodeId,
                                          const QVariantMap& parameters,
                                          ProcessInterruptContext* interrupt,
                                          QString* errorMessage)>;

    void setSnapshotProvider(std::function<ProcessToolpathSnapshot()> provider);
    void setExecutor(ExecutorFn executor);

    ProcessToolpathSnapshot toolpathSnapshot() const override;
    bool executeNormalCutting(const QString& nodeId,
                              const QVariantMap& parameters,
                              ProcessInterruptContext* interrupt,
                              QString* errorMessage) override;

private:
    std::function<ProcessToolpathSnapshot()> m_snapshotProvider;
    ExecutorFn m_executor;
};

} // namespace lcnc::process
