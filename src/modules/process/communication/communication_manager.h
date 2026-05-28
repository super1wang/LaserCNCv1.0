#pragma once

#include "modules/process/communication/communication_endpoint.h"
#include "modules/process/communication/communication_log_model.h"

#include <QObject>
#include <QHash>

namespace lcnc::process {

class ICommunicationChannel;

class CommunicationManager : public QObject
{
    Q_OBJECT
public:
    explicit CommunicationManager(QObject* parent = nullptr);
    ~CommunicationManager() override;

    CommunicationLogModel* logModel() const { return m_logModel; }
    bool configureEndpoint(const CommunicationEndpoint& endpoint, QString* errorMessage = nullptr);
    bool connectDevice(const QString& deviceId, QString* errorMessage = nullptr);
    void disconnectDevice(const QString& deviceId);
    bool sendData(const QString& deviceId, const QByteArray& payload, QString* errorMessage = nullptr);
    CommunicationState state(const QString& deviceId) const;

signals:
    void deviceStateChanged(const QString& deviceId, lcnc::process::CommunicationState state);
    void messageLogged(const lcnc::process::CommunicationMessage& message);

private:
    ICommunicationChannel* createChannel(CommunicationProtocol protocol, QString* errorMessage) const;
    ICommunicationChannel* channel(const QString& deviceId) const;
    void attachChannel(const QString& deviceId, ICommunicationChannel* channel);
    void logMessage(const CommunicationMessage& message);
    QString normalizedDeviceId(const QString& deviceId) const;

    QHash<QString, ICommunicationChannel*> m_channels;
    CommunicationLogModel* m_logModel{nullptr};
};

} // namespace lcnc::process