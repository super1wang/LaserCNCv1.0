#include "modules/process/communication/communication_endpoint.h"

namespace lcnc::process {

CommunicationEndpoint CommunicationEndpoint::defaults()
{
    return CommunicationEndpoint{};
}

QString CommunicationEndpoint::normalizedDeviceId() const
{
    const QString trimmed = deviceId.trimmed();
    return trimmed.isEmpty() ? QStringLiteral("external") : trimmed;
}

QUrl CommunicationEndpoint::url() const
{
    QUrl result;
    result.setScheme(QStringLiteral("http"));
    result.setHost(host.trimmed().isEmpty() ? QStringLiteral("127.0.0.1") : host.trimmed());
    if (port > 0)
        result.setPort(port);
    const QString normalizedPath = path.trimmed().startsWith(QLatin1Char('/'))
        ? path.trimmed()
        : QStringLiteral("/") + path.trimmed();
    result.setPath(normalizedPath.isEmpty() ? QStringLiteral("/") : normalizedPath);
    return result;
}

QString CommunicationEndpoint::displayName() const
{
    switch (protocol) {
    case CommunicationProtocol::Mock:
        return QStringLiteral("Mock:%1").arg(normalizedDeviceId());
    case CommunicationProtocol::Tcp:
        return QStringLiteral("TCP:%1:%2").arg(host, QString::number(port));
    case CommunicationProtocol::Http:
        return url().toString();
    case CommunicationProtocol::Serial:
        return QStringLiteral("Serial:%1@%2").arg(serialPortName, QString::number(baudRate));
    }
    return normalizedDeviceId();
}

} // namespace lcnc::process