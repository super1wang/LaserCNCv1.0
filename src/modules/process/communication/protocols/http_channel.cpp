#include "modules/process/communication/protocols/http_channel.h"

#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>

namespace lcnc::process {

HttpChannel::HttpChannel(QObject* parent)
    : CommunicationChannelBase(parent)
    , m_network(new QNetworkAccessManager(this))
{
}

HttpChannel::~HttpChannel() = default;

bool HttpChannel::connectChannel(QString*)
{
    setState(CommunicationState::Connected);
    return true;
}

void HttpChannel::disconnectChannel()
{
    setState(CommunicationState::Disconnected);
}

bool HttpChannel::sendData(const QByteArray& payload, QString* errorMessage)
{
    if (state() != CommunicationState::Connected) {
        const QString message = tr("HTTP channel is not connected");
        if (errorMessage)
            *errorMessage = message;
        reportError(message);
        return false;
    }

    const QUrl url = m_endpoint.url();
    if (!url.isValid()) {
        const QString message = tr("Invalid HTTP endpoint: %1").arg(url.toString());
        if (errorMessage)
            *errorMessage = message;
        reportError(message);
        return false;
    }

    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, QStringLiteral("application/octet-stream"));
    request.setTransferTimeout(m_endpoint.timeoutMs > 0 ? m_endpoint.timeoutMs : 3000);
    for (auto it = m_endpoint.headers.cbegin(); it != m_endpoint.headers.cend(); ++it)
        request.setRawHeader(it.key().toUtf8(), it.value().toUtf8());

    QNetworkReply* reply = m_network->post(request, payload);
    reportSent(payload, url.toString());
    connect(reply, &QNetworkReply::finished, this, [this, reply] {
        const QByteArray body = reply->readAll();
        if (reply->error() == QNetworkReply::NoError)
            reportReceived(body, body.isEmpty() ? QStringLiteral("HTTP %1").arg(reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt()) : QString());
        else
            reportError(reply->errorString());
        reply->deleteLater();
    });
    return true;
}

} // namespace lcnc::process