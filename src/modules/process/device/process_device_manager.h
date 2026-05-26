#pragma once

#include <QString>
#include <QStringList>

namespace lcnc { class ProcessSettings; }

namespace lcnc::process {

class ProcessDeviceManager
{
public:
    ProcessDeviceManager();

    QStringList availableMotionControllers() const;
    QStringList availableLaserDevices() const;

    void syncFromSettings(const lcnc::ProcessSettings& settings);

    QString activeMotionController() const { return m_activeMotionController; }
    QString activeLaserDevice() const { return m_activeLaserDevice; }

    bool isMotionControllerAvailable(const QString& name) const;
    bool isLaserDeviceAvailable(const QString& name) const;

private:
    QString m_activeMotionController;
    QString m_activeLaserDevice;
};

} // namespace lcnc::process
