#pragma once

#include "modules/process/communication/communication_channel_base.h"

class QSerialPort;

namespace lcnc::process {

class SerialChannel : public CommunicationChannelBase
{
    Q_OBJECT
public:
    explicit SerialChannel(QObject* parent = nullptr);
    ~SerialChannel() override;

    bool connectChannel(QString* errorMessage = nullptr) override;
    void disconnectChannel() override;
    bool sendData(const QByteArray& payload, QString* errorMessage = nullptr) override;

private:
    QSerialPort* m_port{nullptr};
};

} // namespace lcnc::process