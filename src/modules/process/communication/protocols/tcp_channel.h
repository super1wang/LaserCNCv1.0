#pragma once

#include "modules/process/communication/communication_channel_base.h"

class QTcpSocket;

namespace lcnc::process {

class TcpChannel : public CommunicationChannelBase
{
    Q_OBJECT
public:
    explicit TcpChannel(QObject* parent = nullptr);
    ~TcpChannel() override;

    bool connectChannel(QString* errorMessage = nullptr) override;
    void disconnectChannel() override;
    bool sendData(const QByteArray& payload, QString* errorMessage = nullptr) override;

private:
    QTcpSocket* m_socket{nullptr};
};

} // namespace lcnc::process