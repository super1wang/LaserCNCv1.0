#include "modules/process/communication/communication_manager.h"

#include "modules/process/communication/i_communication_channel.h"
#include "modules/process/communication/protocols/http_channel.h"
#include "modules/process/communication/protocols/mock_channel.h"
#if LCNC_PROCESS_HAS_SERIAL_COMM
#include "modules/process/communication/protocols/serial_channel.h"
#endif
#include "modules/process/communication/protocols/tcp_channel.h"

namespace lcnc::process {

CommunicationManager::CommunicationManager(QObject* parent)
    : QObject(parent)
    , m_logModel(new CommunicationLogModel(this))
{
}

CommunicationManager::~CommunicationManager() = default;

bool CommunicationManager::configureEndpoint(const CommunicationEndpoint& endpoint, QString* errorMessage)
{
    const QString deviceId = endpoint.normalizedDeviceId();
    ICommunicationChannel* nextChannel = createChannel(endpoint.protocol, errorMessage);
    if (!nextChannel) {
        CommunicationMessage message;
        message.deviceId = deviceId;
        message.direction = CommunicationMessageDirection::Error;
        message.text = errorMessage ? *errorMessage : tr("Unsupported communication protocol");
        logMessage(message);
        return false;
    }

    nextChannel->configure(endpoint);
    if (auto* previous = channel(deviceId)) {
        previous->disconnectChannel();
        previous->deleteLater();
    }
    attachChannel(deviceId, nextChannel);
    m_channels.insert(deviceId, nextChannel);

    CommunicationMessage message;
    message.deviceId = deviceId;
    message.direction = CommunicationMessageDirection::Info;
    message.text = tr("Configured %1").arg(endpoint.displayName());
    logMessage(message);
    return true;
}

bool CommunicationManager::connectDevice(const QString& deviceId, QString* errorMessage)
{
    auto* item = channel(deviceId);
    if (!item) {
        const QString message = tr("Communication endpoint is not configured: %1").arg(deviceId);
        if (errorMessage)
            *errorMessage = message;
        return false;
    }
    return item->connectChannel(errorMessage);
}

void CommunicationManager::disconnectDevice(const QString& deviceId)
{
    if (auto* item = channel(deviceId))
        item->disconnectChannel();
}

bool CommunicationManager::sendData(const QString& deviceId, const QByteArray& payload, QString* errorMessage)
{
    auto* item = channel(deviceId);
    if (!item) {
        const QString message = tr("Communication endpoint is not configured: %1").arg(deviceId);
        if (errorMessage)
            *errorMessage = message;
        return false;
    }
    return item->sendData(payload, errorMessage);
}

CommunicationState CommunicationManager::state(const QString& deviceId) const
{
    const auto* item = channel(deviceId);
    return item ? item->state() : CommunicationState::Disconnected;
}

ICommunicationChannel* CommunicationManager::createChannel(CommunicationProtocol protocol,
                                                           QString* errorMessage) const
{
    switch (protocol) {
    case CommunicationProtocol::Mock:
        return new MockChannel();
    case CommunicationProtocol::Tcp:
        return new TcpChannel();
    case CommunicationProtocol::Http:
        return new HttpChannel();
    case CommunicationProtocol::Serial:
#if LCNC_PROCESS_HAS_SERIAL_COMM
        return new SerialChannel();
#else
        if (errorMessage)
            *errorMessage = tr("Serial communication is not enabled in this build");
        return nullptr;
#endif
    }
    if (errorMessage)
        *errorMessage = tr("Unsupported communication protocol");
    return nullptr;
}

ICommunicationChannel* CommunicationManager::channel(const QString& deviceId) const
{
    const auto it = m_channels.find(normalizedDeviceId(deviceId));
    return it == m_channels.end() ? nullptr : it.value();
}

void CommunicationManager::attachChannel(const QString& deviceId, ICommunicationChannel* channel)
{
    channel->setParent(this);
    connect(channel, &ICommunicationChannel::stateChanged, this, [this, deviceId](CommunicationState state) {
        emit deviceStateChanged(deviceId, state);
    });
    connect(channel, &ICommunicationChannel::messageSent, this, &CommunicationManager::logMessage);
    connect(channel, &ICommunicationChannel::messageReceived, this, &CommunicationManager::logMessage);
    connect(channel, &ICommunicationChannel::errorOccurred, this, [this, deviceId](const QString& text) {
        CommunicationMessage message;
        message.deviceId = deviceId;
        message.direction = CommunicationMessageDirection::Error;
        message.text = text;
        logMessage(message);
    });
}

void CommunicationManager::logMessage(const CommunicationMessage& message)
{
    if (m_logModel)
        m_logModel->addMessage(message);
    emit messageLogged(message);
}

QString CommunicationManager::normalizedDeviceId(const QString& deviceId) const
{
    const QString trimmed = deviceId.trimmed();
    return trimmed.isEmpty() ? QStringLiteral("external") : trimmed;
}

} // namespace lcnc::process