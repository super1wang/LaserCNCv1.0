#include "modules/process/communication/communication_channel_base.h"

namespace lcnc::process {

CommunicationChannelBase::CommunicationChannelBase(QObject* parent)
    : ICommunicationChannel(parent)
{
}

void CommunicationChannelBase::setState(CommunicationState state)
{
    if (m_state == state)
        return;
    m_state = state;
    emit stateChanged(m_state);
}

void CommunicationChannelBase::reportSent(const QByteArray& payload, const QString& text)
{
    CommunicationMessage message;
    message.deviceId = m_endpoint.normalizedDeviceId();
    message.direction = CommunicationMessageDirection::Sent;
    message.payload = payload;
    message.text = text;
    emit messageSent(message);
}

void CommunicationChannelBase::reportReceived(const QByteArray& payload, const QString& text)
{
    CommunicationMessage message;
    message.deviceId = m_endpoint.normalizedDeviceId();
    message.direction = CommunicationMessageDirection::Received;
    message.payload = payload;
    message.text = text;
    emit messageReceived(message);
}

void CommunicationChannelBase::reportError(const QString& message)
{
    setState(CommunicationState::Error);
    emit errorOccurred(message);
}

} // namespace lcnc::process