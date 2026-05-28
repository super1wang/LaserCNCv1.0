#pragma once

#include "modules/process/communication/communication_types.h"

#include <QMap>
#include <QString>
#include <QUrl>

namespace lcnc::process {

struct CommunicationEndpoint
{
    QString deviceId{QStringLiteral("external")};
    CommunicationProtocol protocol{CommunicationProtocol::Mock};
    QString host{QStringLiteral("127.0.0.1")};
    int port{5000};
    QString path{QStringLiteral("/")};
    QString serialPortName{QStringLiteral("COM1")};
    int baudRate{115200};
    int timeoutMs{3000};
    QMap<QString, QString> headers;

    static CommunicationEndpoint defaults();
    QString normalizedDeviceId() const;
    QUrl url() const;
    QString displayName() const;
};

} // namespace lcnc::process