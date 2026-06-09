#pragma once

#include "modules/process/instructions/process_command.h"

#include <QObject>

namespace lcnc::process {

/**
 * @brief Executes controller-neutral Process commands with simple dry-run semantics.
 *
 * Simplified version without ProcessDeviceCoordinator dependency.
 */
class ProcessExecutionService : public QObject
{
    Q_OBJECT
public:
    explicit ProcessExecutionService(QObject* parent = nullptr);

    bool executeDryRun(const ProcessCommandBuffer& buffer, QString* errorMessage = nullptr);
    int executedCommandCount() const { return m_executedCommandCount; }

signals:
    void executionMessage(const QString& message);
    void executionError(const QString& message);

private:
    bool executeOne(const ProcessCommand& command, QString* errorMessage);

    int m_executedCommandCount{0};
};

} // namespace lcnc::process
