#include "modules/process/communication/protocols/mock_channel.h"

#include <QPointer>
#include <QTimer>

namespace lcnc::process {

MockChannel::MockChannel(QObject* parent)
    : CommunicationChannelBase(parent)
{
}

bool MockChannel::connectChannel(QString*)
{
    setState(CommunicationState::Connecting);
    QPointer<MockChannel> self(this);
    QTimer::singleShot(0, this, [self] {
        if (self)
            self->setState(CommunicationState::Connected);
    });
    return true;
}

void MockChannel::disconnectChannel()
{
    setState(CommunicationState::Disconnected);
}

bool MockChannel::sendData(const QByteArray& payload, QString* errorMessage)
{
    if (state() != CommunicationState::Connected) {
        const QString message = tr("Mock channel is not connected");
        if (errorMessage)
            *errorMessage = message;
        reportError(message);
        return false;
    }

    reportSent(payload);
    QPointer<MockChannel> self(this);
    QTimer::singleShot(20, this, [self, payload] {
        if (self)
            self->reportReceived(payload, QStringLiteral("echo: %1").arg(QString::fromUtf8(payload)));
    });
    return true;
}

} // namespace lcnc::process