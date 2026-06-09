#include "modules/process/instructions/i_controller_translator.h"

#include <QObject>

namespace {

QString n(double value)
{
    return QString::number(value, 'g', 12);
}

} // namespace

namespace lcnc::process {

bool AcsControllerTranslator::translate(const ProcessCommandBuffer& buffer,
                                        QStringList* output,
                                        QString* errorMessage) const
{
    if (!output)
        return true;

    output->clear();
    double currentFeed = 10.0;
    for (const ProcessCommand& command : buffer.commands()) {
        switch (command.type) {
        case ProcessCommandType::SetFeed:
            if (command.feedRate > 0.0)
                currentFeed = command.feedRate;
            output->append(QStringLiteral("VEL=%1").arg(n(currentFeed)));
            break;
        case ProcessCommandType::SetLaserPower:
            output->append(QStringLiteral("LaserEnergy=%1").arg(n(command.laserEnergy)));
            output->append(QStringLiteral("LaserFreq=%1").arg(n(command.laserFrequency)));
            output->append(QStringLiteral("LaserPulse=%1").arg(n(command.laserPulseWidth)));
            break;
        case ProcessCommandType::LaserOn:
            output->append(QStringLiteral("Laser=1"));
            output->append(QStringLiteral("TILL Laser"));
            break;
        case ProcessCommandType::LaserOff:
            output->append(QStringLiteral("Laser=0"));
            output->append(QStringLiteral("TILL ^Laser"));
            break;
        case ProcessCommandType::MoveLinear:
            output->append(QStringLiteral("LINE(0,1,2,3,4)/V, %1,%2,%3,%4,%5,%6")
                               .arg(n(command.x),
                                    n(command.y),
                                    n(command.z),
                                    n(command.r1),
                                    n(command.r2),
                                    n(command.feedRate > 0.0 ? command.feedRate : currentFeed)));
            break;
        case ProcessCommandType::SetDigitalOutput:
            if (command.channel.trimmed().isEmpty()) {
                if (errorMessage)
                    *errorMessage = QObject::tr("ACS 数字输出通道为空");
                return false;
            }
            output->append(QStringLiteral("%1=%2")
                               .arg(command.channel.trimmed(), command.boolValue ? QStringLiteral("1") : QStringLiteral("0")));
            break;
        case ProcessCommandType::Dwell:
            output->append(QStringLiteral("WAIT %1").arg(n(command.durationMs / 1000.0)));
            break;
        case ProcessCommandType::WaitSignal:
            if (!command.channel.trimmed().isEmpty())
                output->append(QStringLiteral("TILL %1").arg(command.channel.trimmed()));
            break;
        case ProcessCommandType::Home:
            output->append(QStringLiteral("! HOME is executed by controller home buffers"));
            break;
        case ProcessCommandType::Stop:
            output->append(QStringLiteral("STOP"));
            break;
        }
    }
    if (output->isEmpty() || output->last().trimmed().compare(QStringLiteral("STOP"), Qt::CaseInsensitive) != 0)
        output->append(QStringLiteral("STOP"));
    return true;
}

} // namespace lcnc::process