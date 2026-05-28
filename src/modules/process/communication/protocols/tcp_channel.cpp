#include "modules/process/communication/protocols/tcp_channel.h"

#include <QAbstractSocket>
#include <QTcpSocket>

namespace lcnc::process {

TcpChannel::TcpChannel(QObject* parent)
    : CommunicationChannelBase(parent)
    , m_socket(new QTcpSocket(this))
{
    connect(m_socket, &QTcpSocket::connected, this, [this] {
        setState(CommunicationState::Connected);
    });
    connect(m_socket, &QTcpSocket::disconnected, this, [this] {
        setState(CommunicationState::Disconnected);
    });
    connect(m_socket, &QTcpSocket::readyRead, this, [this] {
        const QByteArray payload = m_socket->readAll();
        if (!payload.isEmpty())
            reportReceived(payload);
    });
    connect(m_socket, &QAbstractSocket::errorOccurred, this, [this](QAbstractSocket::SocketError) {
        reportError(m_socket->errorString());
    });
}

TcpChannel::~TcpChannel() = default;

bool TcpChannel::connectChannel(QString* errorMessage)
{
    const QString host = m_endpoint.host.trimmed();
    if (host.isEmpty() || m_endpoint.port <= 0) {
        const QString message = tr("TCP endpoint requires host and port");
        if (errorMessage)
            *errorMessage = message;
        reportError(message);
        return false;
    }

    setState(CommunicationState::Connecting);
    m_socket->abort();
    m_socket->connectToHost(host, static_cast<quint16>(m_endpoint.port));
    return true;
}

void TcpChannel::disconnectChannel()
{
    if (m_socket->state() == QAbstractSocket::UnconnectedState) {
        setState(CommunicationState::Disconnected);
        return;
    }
    m_socket->disconnectFromHost();
}

bool TcpChannel::sendData(const QByteArray& payload, QString* errorMessage)
{
    if (state() != CommunicationState::Connected) {
        const QString message = tr("TCP channel is not connected");
        if (errorMessage)
            *errorMessage = message;
        reportError(message);
        return false;
    }

    const qint64 written = m_socket->write(payload);
    if (written < 0) {
        const QString message = m_socket->errorString();
        if (errorMessage)
            *errorMessage = message;
        reportError(message);
        return false;
    }
    reportSent(payload);
    return true;
}

} // namespace lcnc::process