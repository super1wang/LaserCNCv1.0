#include "modules/process/settings/process_settings.h"

#include "core/logging/logger.h"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>

#include <cmath>

namespace lcnc {

QString ProcessSettings::defaultFilePath()
{
    const QString exeDir = QFileInfo(QCoreApplication::applicationFilePath()).absolutePath();
    return QDir(exeDir).filePath(QStringLiteral("config/process.toml"));
}

bool ProcessSettings::loadDefault()
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "ProcessSettings::loadDefault begin");
    const bool ok = load(defaultFilePath());
    LCNC_DEBUG(lcnc::LogCode::Generic, "ProcessSettings::loadDefault done ok={}", ok);
    return ok;
}

bool ProcessSettings::saveDefault() const
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "ProcessSettings::saveDefault begin");
    const bool ok = save(defaultFilePath());
    LCNC_DEBUG(lcnc::LogCode::Generic, "ProcessSettings::saveDefault done ok={}", ok);
    return ok;
}

void ProcessSettings::setControllerEndpoint(const QString& endpoint)
{
    if (m_controllerEndpoint == endpoint) return;
    m_controllerEndpoint = endpoint;
    saveDefault();
}

void ProcessSettings::setSimulationMode(bool enabled)
{
    if (m_simulationMode == enabled) return;
    m_simulationMode = enabled;
    saveDefault();
}

void ProcessSettings::setMotionControllerName(const QString& name)
{
    if (m_motionControllerName == name) return;
    m_motionControllerName = name;
    saveDefault();
}

void ProcessSettings::setLaserDeviceName(const QString& name)
{
    if (m_laserDeviceName == name) return;
    m_laserDeviceName = name;
    saveDefault();
}

void ProcessSettings::setLaserEnergy(double value)
{
    if (std::abs(m_laserEnergy - value) < 1e-9) return;
    m_laserEnergy = value;
    saveDefault();
}

void ProcessSettings::setLaserFrequency(double value)
{
    if (std::abs(m_laserFrequency - value) < 1e-9) return;
    m_laserFrequency = value;
    saveDefault();
}

void ProcessSettings::setLaserPulseWidth(double value)
{
    if (std::abs(m_laserPulseWidth - value) < 1e-9) return;
    m_laserPulseWidth = value;
    saveDefault();
}

void ProcessSettings::readFrom(const toml::value& root)
{
    using namespace lcnc::toml_io;
    m_controllerEndpoint = get_qstring(root, "controllerEndpoint", QString());
    m_simulationMode     = get_bool(root, "simulationMode", true);
    m_motionControllerName = get_qstring(root, "motionController", QStringLiteral("SimulatorCMHP"));
    m_laserDeviceName = get_qstring(root, "laserDevice", QStringLiteral("Simulator"));
    m_laserEnergy = get_double(root, "laserEnergy", 0.0);
    m_laserFrequency = get_double(root, "laserFrequency", 0.0);
    m_laserPulseWidth = get_double(root, "laserPulseWidth", 0.0);
}

void ProcessSettings::writeTo(toml::value& root) const
{
    using namespace lcnc::toml_io;
    root["controllerEndpoint"] = qs(m_controllerEndpoint);
    root["simulationMode"]     = m_simulationMode;
    root["motionController"] = qs(m_motionControllerName);
    root["laserDevice"] = qs(m_laserDeviceName);
    root["laserEnergy"] = m_laserEnergy;
    root["laserFrequency"] = m_laserFrequency;
    root["laserPulseWidth"] = m_laserPulseWidth;
}

} // namespace lcnc
