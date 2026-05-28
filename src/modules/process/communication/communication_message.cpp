#include "modules/process/communication/communication_message.h"

namespace lcnc::process {

QString communicationMessageDirectionText(CommunicationMessageDirection direction)
{
    switch (direction) {
    case CommunicationMessageDirection::Sent:
        return QStringLiteral("TX");
    case CommunicationMessageDirection::Received:
        return QStringLiteral("RX");
    case CommunicationMessageDirection::Info:
        return QStringLiteral("Info");
    case CommunicationMessageDirection::Error:
        return QStringLiteral("Error");
    }
    return QStringLiteral("Info");
}

} // namespace lcnc::process