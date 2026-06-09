#include "modules/process/execution/process_execution_service.h"

#include "core/logging/logger.h"
#include "modules/process/device/process_device_coordinator.h"

#include <cmath>

namespace lcnc::process {

ProcessExecutionService::ProcessExecutionService(ProcessDeviceCoordinator& devices, QObject* parent)
    : QObject(parent)
    , m_devices(devices)
{
}

bool ProcessExecutionService::executeDryRun(const ProcessCommandBuffer& buffer, QString* errorMessage)
{
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
    switch (command.type) {
    case ProcessCommandType::SetFeed:
        return true;
    case ProcessCommandType::SetLaserPower:
        return m_devices.setLaserEnergy(command.laserEnergy, errorMessage);
    case ProcessCommandType::LaserOn:
        return m_devices.setLaserOn(true, errorMessage);
    case ProcessCommandType::LaserOff:
        return m_devices.setLaserOn(false, errorMessage);
    case ProcessCommandType::Dwell:
    case ProcessCommandType::WaitSignal:
    case ProcessCommandType::Home:
    case ProcessCommandType::Stop:
        return true;
    case ProcessCommandType::MoveLinear:
        if (!m_devices.moveAxisTo(QStringLiteral("X"), command.x, errorMessage))
            return false;
        if (!m_devices.moveAxisTo(QStringLiteral("Y"), command.y, errorMessage))
            return false;
        if (!m_devices.moveAxisTo(QStringLiteral("Z"), command.z, errorMessage))
            return false;
        if (std::abs(command.r1) > 1e-9 && !m_devices.moveAxisTo(QStringLiteral("A"), command.r1, errorMessage))
            return false;
        if (std::abs(command.r2) > 1e-9 && !m_devices.moveAxisTo(QStringLiteral("C"), command.r2, errorMessage))
            return false;
        return true;
    case ProcessCommandType::SetDigitalOutput:
        return m_devices.setDigitalOutput(command.channel, command.boolValue, errorMessage);
    }
    return true;
}

} // namespace lcnc::process