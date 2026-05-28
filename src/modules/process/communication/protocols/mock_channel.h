#pragma once

#include "modules/process/communication/communication_channel_base.h"

namespace lcnc::process {

class MockChannel : public CommunicationChannelBase
{
    Q_OBJECT
public:
    explicit MockChannel(QObject* parent = nullptr);

    bool connectChannel(QString* errorMessage = nullptr) override;
    void disconnectChannel() override;
    bool sendData(const QByteArray& payload, QString* errorMessage = nullptr) override;
};

} // namespace lcnc::process