#include "modules/process/communication/communication_device_base.h"

#include "modules/process/communication/communication_manager.h"

namespace lcnc::process {

CommunicationDeviceBase::CommunicationDeviceBase(QString deviceId, CommunicationManager* manager, QObject* parent)
    : QObject(parent)
    , m_deviceId(deviceId.trimmed().isEmpty() ? QStringLiteral("external") : deviceId.trimmed())
    , m_manager(manager)
{
}

CommunicationState CommunicationDeviceBase::communicationState() const
{
    return m_manager ? m_manager->state(m_deviceId) : CommunicationState::Disconnected;
}

bool CommunicationDeviceBase::sendCommunicationData(const QByteArray& payload, QString* errorMessage)
{
    if (!m_manager) {
        if (errorMessage)
            *errorMessage = tr("Communication manager is not available");
        return false;
    }
    return m_manager->sendData(m_deviceId, payload, errorMessage);
}

} // namespace lcnc::process