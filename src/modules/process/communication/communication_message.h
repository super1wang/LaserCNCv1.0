#pragma once

#include <QByteArray>
#include <QDateTime>
#include <QString>

namespace lcnc::process {

enum class CommunicationMessageDirection
{
    Sent,
    Received,
    Info,
    Error
};

struct CommunicationMessage
{
    QDateTime timestamp{QDateTime::currentDateTime()};
    QString deviceId;
    CommunicationMessageDirection direction{CommunicationMessageDirection::Info};
    QByteArray payload;
    QString text;

    QString displayText() const
    {
        if (!text.isEmpty())
            return text;
        return QString::fromUtf8(payload);
    }
};

QString communicationMessageDirectionText(CommunicationMessageDirection direction);

} // namespace lcnc::process