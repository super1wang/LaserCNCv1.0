#pragma once

#include <QString>

namespace lcnc::process {

enum class CommunicationProtocol
{
    Mock,
    Tcp,
    Http,
    Serial
};

enum class CommunicationState
{
    Disconnected,
    Connecting,
    Connected,
    Error
};

QString communicationProtocolToString(CommunicationProtocol protocol);
CommunicationProtocol communicationProtocolFromString(const QString& text, bool* ok = nullptr);
QString communicationStateToString(CommunicationState state);

} // namespace lcnc::process