#pragma once

#include "core/settings/toml_config.h"

#include <QMap>
#include <QString>

namespace lcnc {

/**
 * @brief Process（激光加工执行）模块持久化设置占位实现。
 *
 * Phase 6.6：建立模块自带 settings 的统一约定（CamConfig 已迁移；
 * AppSettings 由内核拥有；本类为 ProcessModule 预留入口）。
 *
 * 当前仅持久化两项：
 *   - controllerEndpoint：上次连接的控制器端点（"tcp://host:port"）
 *   - simulationMode：仿真模式默认开关
 *
 * 持久化文件：`<exeDir>/config/process.toml`，路径由 @ref defaultFilePath 提供。
 * 后续按需扩充字段；新增字段需同时更新 readFrom/writeTo。
 */
class ProcessSettings : public TomlConfig
{
public:
    ProcessSettings() = default;

    /// 从默认路径加载；文件不存在视作首次启动，使用内置默认值。
    bool loadDefault();
    /// 保存到默认路径。
    bool saveDefault() const;

    QString controllerEndpoint() const { return m_controllerEndpoint; }
    void    setControllerEndpoint(const QString& endpoint);

    bool simulationMode() const { return m_simulationMode; }
    void setSimulationMode(bool enabled);

    QString motionControllerName() const { return m_motionControllerName; }
    void setMotionControllerName(const QString& name);

    QString laserDeviceName() const { return m_laserDeviceName; }
    void setLaserDeviceName(const QString& name);

    double laserEnergy() const { return m_laserEnergy; }
    void setLaserEnergy(double value);

    double laserFrequency() const { return m_laserFrequency; }
    void setLaserFrequency(double value);

    double laserPulseWidth() const { return m_laserPulseWidth; }
    void setLaserPulseWidth(double value);

    double axisTravelX() const { return m_axisTravelX; }
    void setAxisTravelX(double value);

    double axisTravelY() const { return m_axisTravelY; }
    void setAxisTravelY(double value);

    double axisTravelZ() const { return m_axisTravelZ; }
    void setAxisTravelZ(double value);

    double axisMaxVelocity() const { return m_axisMaxVelocity; }
    void setAxisMaxVelocity(double value);

    double axisAcceleration() const { return m_axisAcceleration; }
    void setAxisAcceleration(double value);

    double toolFeedRate() const { return m_toolFeedRate; }
    void setToolFeedRate(double value);

    double toolKerfWidth() const { return m_toolKerfWidth; }
    void setToolKerfWidth(double value);

    int pierceDelayMs() const { return m_pierceDelayMs; }
    void setPierceDelayMs(int value);

    QString ioDefaultChannel() const { return m_ioDefaultChannel; }
    void setIoDefaultChannel(const QString& channel);

    bool ioDefaultValue() const { return m_ioDefaultValue; }
    void setIoDefaultValue(bool value);

    QString assistGas() const { return m_assistGas; }
    void setAssistGas(const QString& gas);

    double gasPressure() const { return m_gasPressure; }
    void setGasPressure(double value);

    bool waterCoolingEnabled() const { return m_waterCoolingEnabled; }
    void setWaterCoolingEnabled(bool enabled);

    double waterMinFlow() const { return m_waterMinFlow; }
    void setWaterMinFlow(double value);

    bool monitorEnabled() const { return m_monitorEnabled; }
    void setMonitorEnabled(bool enabled);

    int monitorIntervalMs() const { return m_monitorIntervalMs; }
    void setMonitorIntervalMs(int value);

    double loadingPositionX() const { return m_loadingPositionX; }
    void setLoadingPositionX(double value);

    double loadingPositionY() const { return m_loadingPositionY; }
    void setLoadingPositionY(double value);

    double loadingPositionZ() const { return m_loadingPositionZ; }
    void setLoadingPositionZ(double value);

    QString cameraName() const { return m_cameraName; }
    void setCameraName(const QString& name);

    int cameraExposureMs() const { return m_cameraExposureMs; }
    void setCameraExposureMs(int value);

    QString internetHost() const { return m_internetHost; }
    void setInternetHost(const QString& host);

    int internetPort() const { return m_internetPort; }
    void setInternetPort(int value);

    QString communicationDeviceId() const { return m_communicationDeviceId; }
    void setCommunicationDeviceId(const QString& value);

    QString communicationProtocol() const { return m_communicationProtocol; }
    void setCommunicationProtocol(const QString& value);

    QString communicationHost() const { return m_communicationHost; }
    void setCommunicationHost(const QString& value);

    int communicationPort() const { return m_communicationPort; }
    void setCommunicationPort(int value);

    QString communicationPath() const { return m_communicationPath; }
    void setCommunicationPath(const QString& value);

    QString communicationSerialPort() const { return m_communicationSerialPort; }
    void setCommunicationSerialPort(const QString& value);

    int communicationBaudRate() const { return m_communicationBaudRate; }
    void setCommunicationBaudRate(int value);

    int communicationTimeoutMs() const { return m_communicationTimeoutMs; }
    void setCommunicationTimeoutMs(int value);

    QString uiSettingValue(const QString& key, const QString& defaultValue = QString()) const;
    const QMap<QString, QString>& uiSettingValues() const { return m_uiSettingValues; }
    void setUiSettingValues(const QMap<QString, QString>& values);

    QString legacySettingValue(const QString& key, const QString& defaultValue = QString()) const
    {
        return uiSettingValue(key, defaultValue);
    }
    const QMap<QString, QString>& legacySettingValues() const { return uiSettingValues(); }
    void setLegacySettingValues(const QMap<QString, QString>& values) { setUiSettingValues(values); }

protected:
    void readFrom(const toml::value& root) override;
    void writeTo(toml::value& root) const override;
    const char* configName() const override { return "ProcessSettings"; }

private:
    static QString defaultFilePath();

    QString m_controllerEndpoint;
    bool    m_simulationMode{true};
    QString m_motionControllerName{QStringLiteral("PureSimulation")};
    QString m_laserDeviceName{QStringLiteral("Simulator")};
    double  m_laserEnergy{0.0};
    double  m_laserFrequency{0.0};
    double  m_laserPulseWidth{0.0};
    double  m_axisTravelX{300.0};
    double  m_axisTravelY{300.0};
    double  m_axisTravelZ{100.0};
    double  m_axisMaxVelocity{50.0};
    double  m_axisAcceleration{200.0};
    double  m_toolFeedRate{10.0};
    double  m_toolKerfWidth{0.05};
    int     m_pierceDelayMs{0};
    QString m_ioDefaultChannel{QStringLiteral("DO0")};
    bool    m_ioDefaultValue{true};
    QString m_assistGas{QStringLiteral("Air")};
    double  m_gasPressure{0.0};
    bool    m_waterCoolingEnabled{false};
    double  m_waterMinFlow{0.0};
    bool    m_monitorEnabled{true};
    int     m_monitorIntervalMs{500};
    double  m_loadingPositionX{0.0};
    double  m_loadingPositionY{0.0};
    double  m_loadingPositionZ{0.0};
    QString m_cameraName{QStringLiteral("SimulatorCamera")};
    int     m_cameraExposureMs{30};
    QString m_internetHost{QStringLiteral("127.0.0.1")};
    int     m_internetPort{0};
    QString m_communicationDeviceId{QStringLiteral("laser")};
    QString m_communicationProtocol{QStringLiteral("Mock")};
    QString m_communicationHost{QStringLiteral("127.0.0.1")};
    int     m_communicationPort{5000};
    QString m_communicationPath{QStringLiteral("/")};
    QString m_communicationSerialPort{QStringLiteral("COM1")};
    int     m_communicationBaudRate{115200};
    int     m_communicationTimeoutMs{3000};
    QMap<QString, QString> m_uiSettingValues;
};

} // namespace lcnc
