#pragma once

#include "modules/process/communication/communication_types.h"

#include <QObject>
#include <QString>

namespace lcnc::process {

class CommunicationManager;

class CommunicationDeviceBase : public QObject
{
    Q_OBJECT
public:
    CommunicationDeviceBase(QString deviceId, CommunicationManager* manager, QObject* parent = nullptr);

    QString deviceId() const { return m_deviceId; }
    CommunicationState communicationState() const;

protected:
    bool sendCommunicationData(const QByteArray& payload, QString* errorMessage = nullptr);
    CommunicationManager* communicationManager() const { return m_manager; }

private:
    QString m_deviceId;
    CommunicationManager* m_manager{nullptr};
};

} // namespace lcnc::process