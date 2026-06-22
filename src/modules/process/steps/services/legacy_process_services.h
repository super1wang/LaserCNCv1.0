#pragma once

#include "modules/process/steps/process_step_context.h"

class Service;

namespace lcnc::process {

class LegacyProcessMotionService final : public IProcessMotionService
{
public:
    explicit LegacyProcessMotionService(Service* service = nullptr);
    void setService(Service* service) { m_service = service; }

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
};

class LegacyProcessIoService final : public IProcessIoService
{
public:
    explicit LegacyProcessIoService(Service* service = nullptr);
    void setService(Service* service) { m_service = service; }

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
};

class CallbackProcessCuttingService final : public IProcessCuttingService
{
public:
    void setSnapshotProvider(std::function<ProcessToolpathSnapshot()> provider);
    void setExecutor(std::function<bool(bool dryRun, QString* errorMessage)> executor);

    ProcessToolpathSnapshot toolpathSnapshot() const override;
    bool executeNormalCutting(const QVariantMap& parameters, QString* errorMessage) override;

private:
    std::function<ProcessToolpathSnapshot()> m_snapshotProvider;
    std::function<bool(bool dryRun, QString* errorMessage)> m_executor;
};

} // namespace lcnc::process
