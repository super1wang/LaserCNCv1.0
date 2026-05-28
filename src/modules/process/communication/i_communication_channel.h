#pragma once

#include "modules/process/communication/communication_endpoint.h"
#include "modules/process/communication/communication_message.h"
#include "modules/process/communication/communication_types.h"

#include <QObject>

namespace lcnc::process {

class ICommunicationChannel : public QObject
{
    Q_OBJECT
public:
    explicit ICommunicationChannel(QObject* parent = nullptr) : QObject(parent) {}
    ~ICommunicationChannel() override = default;

    virtual CommunicationEndpoint endpoint() const = 0;
    virtual void configure(const CommunicationEndpoint& endpoint) = 0;
    virtual CommunicationState state() const = 0;
    virtual bool connectChannel(QString* errorMessage = nullptr) = 0;
    virtual void disconnectChannel() = 0;
    virtual bool sendData(const QByteArray& payload, QString* errorMessage = nullptr) = 0;

signals:
    void stateChanged(lcnc::process::CommunicationState state);
    void messageSent(const lcnc::process::CommunicationMessage& message);
    void messageReceived(const lcnc::process::CommunicationMessage& message);
    void errorOccurred(const QString& message);
};

} // namespace lcnc::process