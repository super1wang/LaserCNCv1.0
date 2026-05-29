#include "modules/process/instructions/process_command.h"

#include <QObject>

namespace lcnc::process {

QString ProcessCommandBuffer::summary() const
{
    int moveCount = 0;
    int laserCount = 0;
    for (const ProcessCommand& command : m_commands) {
        if (command.type == ProcessCommandType::MoveLinear)
            ++moveCount;
        if (command.type == ProcessCommandType::LaserOn
            || command.type == ProcessCommandType::LaserOff
            || command.type == ProcessCommandType::SetLaserPower) {
            ++laserCount;
        }
    }
    return QObject::tr("commands=%1, moves=%2, laser=%3")
        .arg(m_commands.size())
        .arg(moveCount)
        .arg(laserCount);
}

} // namespace lcnc::process