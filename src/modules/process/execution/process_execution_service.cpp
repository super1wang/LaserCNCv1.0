#include "modules/process/execution/process_execution_service.h"

#include "core/logging/logger.h"

namespace lcnc::process {

ProcessExecutionService::ProcessExecutionService(QObject* parent)
    : QObject(parent)
{
}

bool ProcessExecutionService::executeDryRun(const ProcessCommandBuffer& buffer, QString* errorMessage)
{
    Q_UNUSED(errorMessage);
    m_executedCommandCount = 0;
    for (const ProcessCommand& command : buffer.commands()) {
        if (!executeOne(command, errorMessage)) {
            emit executionError(errorMessage ? *errorMessage : tr("执行 Process 指令失败"));
            return false;
        }
        ++m_executedCommandCount;
    }

    LCNC_INFO(lcnc::LogCode::Generic,
              "process.execution: dry-run executed {} commands",
              m_executedCommandCount);
    emit executionMessage(tr("Process 指令干运行完成: %1 条").arg(m_executedCommandCount));
    return true;
}

bool ProcessExecutionService::executeOne(const ProcessCommand& command, QString* errorMessage)
{
    Q_UNUSED(errorMessage);
    switch (command.type) {
    case ProcessCommandType::SetFeed:
    case ProcessCommandType::SetLaserPower:
    case ProcessCommandType::LaserOn:
    case ProcessCommandType::LaserOff:
    case ProcessCommandType::Dwell:
    case ProcessCommandType::WaitSignal:
    case ProcessCommandType::Home:
    case ProcessCommandType::Stop:
    case ProcessCommandType::MoveLinear:
    case ProcessCommandType::SetDigitalOutput:
        return true;
    }
    return true;
}

} // namespace lcnc::process
