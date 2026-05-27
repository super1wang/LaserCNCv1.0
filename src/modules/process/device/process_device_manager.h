#pragma once

#include <QString>
#include <QStringList>
#include <QVector>
#include <memory>

namespace lcnc { class ProcessSettings; }

namespace lcnc::process {

class ILaserDevice;
class IProcessIo;

enum class ProcessDeviceKind
{
    MotionController,
    Laser,
    Io
};

enum class ProcessDeviceAvailability
{
    Simulation,
    Available,
    Unavailable
};

struct ProcessDeviceDescriptor
{
    ProcessDeviceKind kind{ProcessDeviceKind::MotionController};
    QString name;
    QString displayName;
    ProcessDeviceAvailability availability{ProcessDeviceAvailability::Unavailable};
    QStringList capabilities;
};

class ProcessDeviceManager
{
public:
    ProcessDeviceManager();
    ~ProcessDeviceManager();

    QVector<ProcessDeviceDescriptor> descriptors(ProcessDeviceKind kind) const;
    QVector<ProcessDeviceDescriptor> allDescriptors() const { return m_descriptors; }
    QStringList availableMotionControllers() const;
    QStringList availableLaserDevices() const;

    void syncFromSettings(const lcnc::ProcessSettings& settings);

    QString activeMotionController() const { return m_activeMotionController; }
    QString activeLaserDevice() const { return m_activeLaserDevice; }

    ILaserDevice* laserDevice() const { return m_laserDevice.get(); }
    IProcessIo* processIo() const { return m_processIo.get(); }

    bool isMotionControllerAvailable(const QString& name) const;
    bool isLaserDeviceAvailable(const QString& name) const;
    bool isDeviceUsable(ProcessDeviceKind kind, const QString& name) const;
    const ProcessDeviceDescriptor* descriptor(ProcessDeviceKind kind, const QString& name) const;

private:
    void registerBuiltInDevices();
    QStringList names(ProcessDeviceKind kind) const;
    QString normalizedName(const QString& name) const;

    QVector<ProcessDeviceDescriptor> m_descriptors;
    QString m_activeMotionController;
    QString m_activeLaserDevice;
    std::unique_ptr<ILaserDevice> m_laserDevice;
    std::unique_ptr<IProcessIo> m_processIo;
};

} // namespace lcnc::process
