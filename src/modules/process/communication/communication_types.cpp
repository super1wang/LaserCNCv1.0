#include "modules/process/communication/communication_types.h"

namespace lcnc::process {

QString communicationProtocolToString(CommunicationProtocol protocol)
{
    switch (protocol) {
    case CommunicationProtocol::Mock:
        return QStringLiteral("Mock");
    case CommunicationProtocol::Tcp:
        return QStringLiteral("TCP");
    case CommunicationProtocol::Http:
        return QStringLiteral("HTTP");
    case CommunicationProtocol::Serial:
        return QStringLiteral("Serial");
    }
    return QStringLiteral("Mock");
}

CommunicationProtocol communicationProtocolFromString(const QString& text, bool* ok)
{
    const QString normalized = text.trimmed().toCaseFolded();
    if (ok)
        *ok = true;

    if (normalized == QStringLiteral("mock"))
        return CommunicationProtocol::Mock;
    if (normalized == QStringLiteral("tcp"))
        return CommunicationProtocol::Tcp;
    if (normalized == QStringLiteral("http"))
        return CommunicationProtocol::Http;
    if (normalized == QStringLiteral("serial") || normalized == QStringLiteral("串口"))
        return CommunicationProtocol::Serial;

    if (ok)
        *ok = false;
    return CommunicationProtocol::Mock;
}

QString communicationStateToString(CommunicationState state)
{
    switch (state) {
    case CommunicationState::Disconnected:
        return QStringLiteral("Disconnected");
    case CommunicationState::Connecting:
        return QStringLiteral("Connecting");
    case CommunicationState::Connected:
        return QStringLiteral("Connected");
    case CommunicationState::Error:
        return QStringLiteral("Error");
    }
    return QStringLiteral("Disconnected");
}

} // namespace lcnc::process