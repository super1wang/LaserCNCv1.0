#pragma once

#include <QMap>
#include <QString>
#include <QStringList>
#include <QVector>

namespace lcnc {

struct ProcessMonitorFaultAction_Compat { enum { Continue, Pause, Stop }; };

struct ProcessMonitorSettings_Compat {
    int faultAction{1}; // Pause
    bool interLockEnabled{true}, safetyLightCurtainEnabled{false};
    bool pressureMonitorEnabled{false}, waterLeakageMonitorEnabled{false};
    bool waterTankMonitorEnabled{false}, waterPressureMonitorEnabled{false};
    double waterPressureConversions{1}, waterPressureLimitMpa{1.6};
    bool waterLevelMonitorEnabled{false};
    double waterLevelConversions{1}, waterLevelLimitMm{10.0};
};

class ProcessMonitorIoTableEntry_Compat {
public:
    QString name, ioIndex, direction;
    bool enabled{true};
};

class ProcessIoTableEntry_Compat {
public:
    QString name, ioIndex;
    bool output{false};
};

class ProcessSettings {
public:
    ProcessSettings() = default;

    // ── Basic settings ──
    bool simulationMode() const { return m_simulationMode; }
    void setSimulationMode(bool v) { m_simulationMode = v; }

    QString motionControllerName() const { return m_motionControllerName; }
    void setMotionControllerName(const QString& v) { m_motionControllerName = v; }

    QString laserDeviceName() const { return m_laserDeviceName; }
    void setLaserDeviceName(const QString& v) { m_laserDeviceName = v; }

    QString controllerEndpoint() const { return m_controllerEndpoint; }
    void setControllerEndpoint(const QString& v) { m_controllerEndpoint = v; }

    // ── Laser params ──
    double laserEnergy() const { return m_laserEnergy; }
    double laserFrequency() const { return m_laserFrequency; }
    double laserPulseWidth() const { return m_laserPulseWidth; }
    void setLaserEnergy(double v) { m_laserEnergy = v; }
    void setLaserFrequency(double v) { m_laserFrequency = v; }
    void setLaserPulseWidth(double v) { m_laserPulseWidth = v; }

    // ── Monitor ──
    bool monitorEnabled() const { return m_monitorEnabled; }
    int monitorIntervalMs() const { return m_monitorIntervalMs; }
    ProcessMonitorSettings_Compat monitorSettings() const { return m_monitorSettings; }
    void setMonitorEnabled(bool v) { m_monitorEnabled = v; }

    // ── UI settings (legacy .ui fields) ──
    QString uiSettingValue(const QString& key, const QString& fallback = {}) const {
        return m_uiSettings.value(key, fallback);
    }
    void setUiSettingValues(const QMap<QString, QString>& values) { m_uiSettings = values; }
    const QMap<QString, QString>& uiSettingValues() const { return m_uiSettings; }

    // ── IO tables ──
    QVector<ProcessIoTableEntry_Compat> customDigitalIoTable() const { return m_digitalIo; }
    QVector<ProcessIoTableEntry_Compat> customAnalogIoTable() const { return m_analogIo; }
    void setCustomDigitalIoTable(const QVector<ProcessIoTableEntry_Compat>& v) { m_digitalIo = v; }
    void setCustomAnalogIoTable(const QVector<ProcessIoTableEntry_Compat>& v) { m_analogIo = v; }

    QString ioDefaultChannel() const { return QStringLiteral("DO0"); }

    // ── Communication ──
    QString communicationDeviceId() const { return m_commDeviceId; }
    void setCommunicationDeviceId(const QString& v) { m_commDeviceId = v; }
    QString communicationProtocol() const { return m_commProtocol; }
    void setCommunicationProtocol(const QString& v) { m_commProtocol = v; }
    QString communicationHost() const { return m_commHost; }
    void setCommunicationHost(const QString& v) { m_commHost = v; }
    int communicationPort() const { return m_commPort; }
    void setCommunicationPort(int v) { m_commPort = v; }
    QString communicationPath() const { return m_commPath; }
    void setCommunicationPath(const QString& v) { m_commPath = v; }
    QString communicationSerialPort() const { return m_commSerialPort; }
    void setCommunicationSerialPort(const QString& v) { m_commSerialPort = v; }
    int communicationBaudRate() const { return m_commBaudRate; }
    void setCommunicationBaudRate(int v) { m_commBaudRate = v; }
    int communicationTimeoutMs() const { return m_commTimeoutMs; }
    void setCommunicationTimeoutMs(int v) { m_commTimeoutMs = v; }

    // ── Persistence ──
    void loadDefault() {}
    void saveDefault() {}

private:
    bool m_simulationMode{true};
    QString m_motionControllerName{"SimulatorCMHP"};
    QString m_laserDeviceName{"Simulator"};
    QString m_controllerEndpoint;
    double m_laserEnergy{0}, m_laserFrequency{0}, m_laserPulseWidth{0};
    bool m_monitorEnabled{true};
    int m_monitorIntervalMs{500};
    ProcessMonitorSettings_Compat m_monitorSettings;
    QMap<QString, QString> m_uiSettings;
    QVector<ProcessIoTableEntry_Compat> m_digitalIo;
    QVector<ProcessIoTableEntry_Compat> m_analogIo;
    QString m_commDeviceId, m_commProtocol{"TCP"}, m_commHost{"127.0.0.1"}, m_commPath, m_commSerialPort{"COM1"};
    int m_commPort{5000}, m_commBaudRate{115200}, m_commTimeoutMs{3000};
};

} // namespace lcnc
