#include "modules/process/settings/process_settings.h"

#include "core/logging/logger.h"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>

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

void ProcessSettings::readFrom(const toml::value& root)
{
    using namespace lcnc::toml_io;
    m_controllerEndpoint = get_qstring(root, "controllerEndpoint", QString());
    m_simulationMode     = get_bool(root, "simulationMode", true);
}

void ProcessSettings::writeTo(toml::value& root) const
{
    using namespace lcnc::toml_io;
    root["controllerEndpoint"] = qs(m_controllerEndpoint);
    root["simulationMode"]     = m_simulationMode;
}

} // namespace lcnc
