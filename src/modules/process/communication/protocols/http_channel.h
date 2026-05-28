#pragma once

#include "modules/process/communication/communication_channel_base.h"

class QNetworkAccessManager;

namespace lcnc::process {

class HttpChannel : public CommunicationChannelBase
{
    Q_OBJECT
public:
    explicit HttpChannel(QObject* parent = nullptr);
    ~HttpChannel() override;

    bool connectChannel(QString* errorMessage = nullptr) override;
    void disconnectChannel() override;
    bool sendData(const QByteArray& payload, QString* errorMessage = nullptr) override;

private:
    QNetworkAccessManager* m_network{nullptr};
};

} // namespace lcnc::process