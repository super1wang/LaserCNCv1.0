#pragma once

#include "modules/process/instructions/process_command.h"

#include <QObject>

namespace lcnc::process { class ProcessDeviceCoordinator; }

namespace lcnc::process {

/**
 * @brief Executes controller-neutral Process commands through active device adapters.
 */
class ProcessExecutionService : public QObject
{
    Q_OBJECT
public:
    explicit ProcessExecutionService(ProcessDeviceCoordinator& devices, QObject* parent = nullptr);

    bool executeDryRun(const ProcessCommandBuffer& buffer, QString* errorMessage = nullptr);
    int executedCommandCount() const { return m_executedCommandCount; }

signals:
    void executionMessage(const QString& message);
    void executionError(const QString& message);

private:
    bool executeOne(const ProcessCommand& command, QString* errorMessage);

    ProcessDeviceCoordinator& m_devices;
    int m_executedCommandCount{0};
};

} // namespace lcnc::process