#pragma once

#include "modules/process/communication/i_communication_channel.h"

namespace lcnc::process {

class CommunicationChannelBase : public ICommunicationChannel
{
    Q_OBJECT
public:
    explicit CommunicationChannelBase(QObject* parent = nullptr);

    CommunicationEndpoint endpoint() const override { return m_endpoint; }
    void configure(const CommunicationEndpoint& endpoint) override { m_endpoint = endpoint; }
    CommunicationState state() const override { return m_state; }

protected:
    void setState(CommunicationState state);
    void reportSent(const QByteArray& payload, const QString& text = QString());
    void reportReceived(const QByteArray& payload, const QString& text = QString());
    void reportError(const QString& message);

    CommunicationEndpoint m_endpoint;
    CommunicationState m_state{CommunicationState::Disconnected};
};

} // namespace lcnc::process