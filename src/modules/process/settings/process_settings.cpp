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

void ProcessSettings::setAxisTravelX(double value)
{
    if (std::abs(m_axisTravelX - value) < 1e-9) return;
    m_axisTravelX = value;
    saveDefault();
}

void ProcessSettings::setAxisTravelY(double value)
{
    if (std::abs(m_axisTravelY - value) < 1e-9) return;
    m_axisTravelY = value;
    saveDefault();
}

void ProcessSettings::setAxisTravelZ(double value)
{
    if (std::abs(m_axisTravelZ - value) < 1e-9) return;
    m_axisTravelZ = value;
    saveDefault();
}

void ProcessSettings::setAxisMaxVelocity(double value)
{
    if (std::abs(m_axisMaxVelocity - value) < 1e-9) return;
    m_axisMaxVelocity = value;
    saveDefault();
}

void ProcessSettings::setAxisAcceleration(double value)
{
    if (std::abs(m_axisAcceleration - value) < 1e-9) return;
    m_axisAcceleration = value;
    saveDefault();
}

void ProcessSettings::setToolFeedRate(double value)
{
    if (std::abs(m_toolFeedRate - value) < 1e-9) return;
    m_toolFeedRate = value;
    saveDefault();
}

void ProcessSettings::setToolKerfWidth(double value)
{
    if (std::abs(m_toolKerfWidth - value) < 1e-9) return;
    m_toolKerfWidth = value;
    saveDefault();
}

void ProcessSettings::setPierceDelayMs(int value)
{
    if (m_pierceDelayMs == value) return;
    m_pierceDelayMs = value;
    saveDefault();
}

void ProcessSettings::setIoDefaultChannel(const QString& channel)
{
    const QString trimmed = channel.trimmed();
    if (m_ioDefaultChannel == trimmed) return;
    m_ioDefaultChannel = trimmed;
    saveDefault();
}

void ProcessSettings::setIoDefaultValue(bool value)
{
    if (m_ioDefaultValue == value) return;
    m_ioDefaultValue = value;
    saveDefault();
}

void ProcessSettings::setAssistGas(const QString& gas)
{
    const QString trimmed = gas.trimmed();
    if (m_assistGas == trimmed) return;
    m_assistGas = trimmed;
    saveDefault();
}

void ProcessSettings::setGasPressure(double value)
{
    if (std::abs(m_gasPressure - value) < 1e-9) return;
    m_gasPressure = value;
    saveDefault();
}

void ProcessSettings::setWaterCoolingEnabled(bool enabled)
{
    if (m_waterCoolingEnabled == enabled) return;
    m_waterCoolingEnabled = enabled;
    saveDefault();
}

void ProcessSettings::setWaterMinFlow(double value)
{
    if (std::abs(m_waterMinFlow - value) < 1e-9) return;
    m_waterMinFlow = value;
    saveDefault();
}

void ProcessSettings::setMonitorEnabled(bool enabled)
{
    if (m_monitorEnabled == enabled) return;
    m_monitorEnabled = enabled;
    saveDefault();
}

void ProcessSettings::setMonitorIntervalMs(int value)
{
    if (m_monitorIntervalMs == value) return;
    m_monitorIntervalMs = value;
    saveDefault();
}

void ProcessSettings::setLoadingPositionX(double value)
{
    if (std::abs(m_loadingPositionX - value) < 1e-9) return;
    m_loadingPositionX = value;
    saveDefault();
}

void ProcessSettings::setLoadingPositionY(double value)
{
    if (std::abs(m_loadingPositionY - value) < 1e-9) return;
    m_loadingPositionY = value;
    saveDefault();
}

void ProcessSettings::setLoadingPositionZ(double value)
{
    if (std::abs(m_loadingPositionZ - value) < 1e-9) return;
    m_loadingPositionZ = value;
    saveDefault();
}

void ProcessSettings::setCameraName(const QString& name)
{
    const QString trimmed = name.trimmed();
    if (m_cameraName == trimmed) return;
    m_cameraName = trimmed;
    saveDefault();
}

void ProcessSettings::setCameraExposureMs(int value)
{
    if (m_cameraExposureMs == value) return;
    m_cameraExposureMs = value;
    saveDefault();
}

void ProcessSettings::setInternetHost(const QString& host)
{
    const QString trimmed = host.trimmed();
    if (m_internetHost == trimmed) return;
    m_internetHost = trimmed;
    saveDefault();
}

void ProcessSettings::setInternetPort(int value)
{
    if (m_internetPort == value) return;
    m_internetPort = value;
    saveDefault();
}

void ProcessSettings::setCommunicationDeviceId(const QString& value)
{
    const QString trimmed = value.trimmed().isEmpty() ? QStringLiteral("laser") : value.trimmed();
    if (m_communicationDeviceId == trimmed) return;
    m_communicationDeviceId = trimmed;
    saveDefault();
}

void ProcessSettings::setCommunicationProtocol(const QString& value)
{
    const QString trimmed = value.trimmed().isEmpty() ? QStringLiteral("Mock") : value.trimmed();
    if (m_communicationProtocol == trimmed) return;
    m_communicationProtocol = trimmed;
    saveDefault();
}

void ProcessSettings::setCommunicationHost(const QString& value)
{
    const QString trimmed = value.trimmed();
    if (m_communicationHost == trimmed) return;
    m_communicationHost = trimmed;
    saveDefault();
}

void ProcessSettings::setCommunicationPort(int value)
{
    if (m_communicationPort == value) return;
    m_communicationPort = value;
    saveDefault();
}

void ProcessSettings::setCommunicationPath(const QString& value)
{
    const QString trimmed = value.trimmed().isEmpty() ? QStringLiteral("/") : value.trimmed();
    if (m_communicationPath == trimmed) return;
    m_communicationPath = trimmed;
    saveDefault();
}

void ProcessSettings::setCommunicationSerialPort(const QString& value)
{
    const QString trimmed = value.trimmed().isEmpty() ? QStringLiteral("COM1") : value.trimmed();
    if (m_communicationSerialPort == trimmed) return;
    m_communicationSerialPort = trimmed;
    saveDefault();
}

void ProcessSettings::setCommunicationBaudRate(int value)
{
    if (m_communicationBaudRate == value) return;
    m_communicationBaudRate = value;
    saveDefault();
}

void ProcessSettings::setCommunicationTimeoutMs(int value)
{
    if (m_communicationTimeoutMs == value) return;
    m_communicationTimeoutMs = value;
    saveDefault();
}

QString ProcessSettings::uiSettingValue(const QString& key, const QString& defaultValue) const
{
    return m_uiSettingValues.value(key, defaultValue);
}

void ProcessSettings::setUiSettingValues(const QMap<QString, QString>& values)
{
    if (m_uiSettingValues == values) return;
    m_uiSettingValues = values;
    saveDefault();
}

void ProcessSettings::readFrom(const toml::value& root)
{
    using namespace lcnc::toml_io;
    m_controllerEndpoint = get_qstring(root, "controllerEndpoint", QString());
    m_simulationMode     = get_bool(root, "simulationMode", true);
    m_motionControllerName = get_qstring(root, "motionController", QStringLiteral("PureSimulation"));
    m_laserDeviceName = get_qstring(root, "laserDevice", QStringLiteral("Simulator"));
    m_laserEnergy = get_double(root, "laserEnergy", 0.0);
    m_laserFrequency = get_double(root, "laserFrequency", 0.0);
    m_laserPulseWidth = get_double(root, "laserPulseWidth", 0.0);
    m_axisTravelX = get_double(root, "axisTravelX", 300.0);
    m_axisTravelY = get_double(root, "axisTravelY", 300.0);
    m_axisTravelZ = get_double(root, "axisTravelZ", 100.0);
    m_axisMaxVelocity = get_double(root, "axisMaxVelocity", 50.0);
    m_axisAcceleration = get_double(root, "axisAcceleration", 200.0);
    m_toolFeedRate = get_double(root, "toolFeedRate", 10.0);
    m_toolKerfWidth = get_double(root, "toolKerfWidth", 0.05);
    m_pierceDelayMs = get_int(root, "pierceDelayMs", 0);
    m_ioDefaultChannel = get_qstring(root, "ioDefaultChannel", QStringLiteral("DO0"));
    m_ioDefaultValue = get_bool(root, "ioDefaultValue", true);
    m_assistGas = get_qstring(root, "assistGas", QStringLiteral("Air"));
    m_gasPressure = get_double(root, "gasPressure", 0.0);
    m_waterCoolingEnabled = get_bool(root, "waterCoolingEnabled", false);
    m_waterMinFlow = get_double(root, "waterMinFlow", 0.0);
    m_monitorEnabled = get_bool(root, "monitorEnabled", true);
    m_monitorIntervalMs = get_int(root, "monitorIntervalMs", 500);
    m_loadingPositionX = get_double(root, "loadingPositionX", 0.0);
    m_loadingPositionY = get_double(root, "loadingPositionY", 0.0);
    m_loadingPositionZ = get_double(root, "loadingPositionZ", 0.0);
    m_cameraName = get_qstring(root, "cameraName", QStringLiteral("SimulatorCamera"));
    m_cameraExposureMs = get_int(root, "cameraExposureMs", 30);
    m_internetHost = get_qstring(root, "internetHost", QStringLiteral("127.0.0.1"));
    m_internetPort = get_int(root, "internetPort", 0);
    m_communicationDeviceId = get_qstring(root, "communicationDeviceId", QStringLiteral("laser"));
    m_communicationProtocol = get_qstring(root, "communicationProtocol", QStringLiteral("Mock"));
    m_communicationHost = get_qstring(root, "communicationHost", QStringLiteral("127.0.0.1"));
    m_communicationPort = get_int(root, "communicationPort", 5000);
    m_communicationPath = get_qstring(root, "communicationPath", QStringLiteral("/"));
    m_communicationSerialPort = get_qstring(root, "communicationSerialPort", QStringLiteral("COM1"));
    m_communicationBaudRate = get_int(root, "communicationBaudRate", 115200);
    m_communicationTimeoutMs = get_int(root, "communicationTimeoutMs", 3000);

    m_uiSettingValues.clear();
    auto readUiSettingTable = [this, &root](const char* tableName) {
        if (!root.is_table() || !root.contains(tableName) || !root.at(tableName).is_table())
            return;
        const auto& uiRoot = root.at(tableName).as_table();
        for (const auto& pagePair : uiRoot) {
            if (!pagePair.second.is_table())
                continue;
            const QString page = QString::fromStdString(pagePair.first);
            for (const auto& fieldPair : pagePair.second.as_table()) {
                const QString field = QString::fromStdString(fieldPair.first);
                const toml::value& value = fieldPair.second;
                QString text;
                if (value.is_string())
                    text = QString::fromStdString(value.as_string());
                else if (value.is_boolean())
                    text = value.as_boolean() ? QStringLiteral("true") : QStringLiteral("false");
                else if (value.is_integer())
                    text = QString::number(value.as_integer());
                else if (value.is_floating())
                    text = QString::number(value.as_floating(), 'g', 15);
                else
                    continue;
                m_uiSettingValues.insert(QStringLiteral("%1.%2").arg(page, field), text);
            }
        }
    };
    readUiSettingTable("legacySetting");
    readUiSettingTable("uiSetting");
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
    root["axisTravelX"] = m_axisTravelX;
    root["axisTravelY"] = m_axisTravelY;
    root["axisTravelZ"] = m_axisTravelZ;
    root["axisMaxVelocity"] = m_axisMaxVelocity;
    root["axisAcceleration"] = m_axisAcceleration;
    root["toolFeedRate"] = m_toolFeedRate;
    root["toolKerfWidth"] = m_toolKerfWidth;
    root["pierceDelayMs"] = m_pierceDelayMs;
    root["ioDefaultChannel"] = qs(m_ioDefaultChannel);
    root["ioDefaultValue"] = m_ioDefaultValue;
    root["assistGas"] = qs(m_assistGas);
    root["gasPressure"] = m_gasPressure;
    root["waterCoolingEnabled"] = m_waterCoolingEnabled;
    root["waterMinFlow"] = m_waterMinFlow;
    root["monitorEnabled"] = m_monitorEnabled;
    root["monitorIntervalMs"] = m_monitorIntervalMs;
    root["loadingPositionX"] = m_loadingPositionX;
    root["loadingPositionY"] = m_loadingPositionY;
    root["loadingPositionZ"] = m_loadingPositionZ;
    root["cameraName"] = qs(m_cameraName);
    root["cameraExposureMs"] = m_cameraExposureMs;
    root["internetHost"] = qs(m_internetHost);
    root["internetPort"] = m_internetPort;
    root["communicationDeviceId"] = qs(m_communicationDeviceId);
    root["communicationProtocol"] = qs(m_communicationProtocol);
    root["communicationHost"] = qs(m_communicationHost);
    root["communicationPort"] = m_communicationPort;
    root["communicationPath"] = qs(m_communicationPath);
    root["communicationSerialPort"] = qs(m_communicationSerialPort);
    root["communicationBaudRate"] = m_communicationBaudRate;
    root["communicationTimeoutMs"] = m_communicationTimeoutMs;

    if (!m_uiSettingValues.isEmpty()) {
        toml::value uiSetting(toml::table{});
        for (auto it = m_uiSettingValues.cbegin(); it != m_uiSettingValues.cend(); ++it) {
            const int dot = it.key().indexOf(QLatin1Char('.'));
            if (dot <= 0 || dot >= it.key().size() - 1)
                continue;
            const QString page = it.key().left(dot);
            const QString field = it.key().mid(dot + 1);
            uiSetting[qs(page)][qs(field)] = qs(it.value());
        }
        root["uiSetting"] = uiSetting;
    }
}

} // namespace lcnc
