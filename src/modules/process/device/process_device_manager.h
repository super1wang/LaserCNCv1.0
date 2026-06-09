#pragma once

#include "modules/process/device/i_process_device.h"

#include <QString>
#include <QStringList>
#include <QVector>
#include <functional>
#include <memory>

class QObject;

namespace lcnc {
class IMotionController;
class ProcessSettings;
}

namespace lcnc::process {

class ILaserDevice;
class IProcessIo;

/**
 * @brief Runtime role assigned to a user-added Process peripheral instance.
 */
enum class ProcessDeviceRole
{
    None,
    ActiveMotion,
    ActiveLaser,
    ActiveIo
};

/**
 * @brief Registered Process peripheral type descriptor.
 */
struct ProcessDeviceDescriptor
{
    ProcessDeviceKind kind{ProcessDeviceKind::MotionController};
    QString name;
    QString displayName;
    ProcessDeviceAvailability availability{ProcessDeviceAvailability::Unavailable};
    QStringList capabilities;
    QString unavailableReason;
    QString settingsPageId;
};

/**
 * @brief User-added Process peripheral instance tracked by the manager and UI.
 */
struct ProcessDeviceSession
{
    QString instanceId;
    ProcessDeviceKind kind{ProcessDeviceKind::MotionController};
    ProcessDeviceRole role{ProcessDeviceRole::None};
    QString descriptorName;
    QString displayName;
    bool enabled{true};
    ProcessDeviceConnectionState connectionState{ProcessDeviceConnectionState::Disconnected};
    QString lastError;
};

using ProcessDeviceConnectProgress = std::function<void(int current, int total, const QString& step)>;

/**
 * @brief Registers Process peripherals and manages active runtime device sessions.
 */
class ProcessDeviceManager
{
public:
    ProcessDeviceManager();
    ~ProcessDeviceManager();

    QVector<ProcessDeviceDescriptor> descriptors(ProcessDeviceKind kind) const;
    QVector<ProcessDeviceDescriptor> allDescriptors() const { return m_descriptors; }
    QVector<ProcessDeviceSession> deviceSessions() const { return m_sessions; }
    QStringList availableMotionControllers() const;
    QStringList availableLaserDevices() const;

    void syncFromSettings(const lcnc::ProcessSettings& settings);
    void setMotionController(lcnc::IMotionController* controller);

    QString activeMotionController() const { return m_activeMotionController; }
    QString activeLaserDevice() const { return m_activeLaserDevice; }

    ILaserDevice* laserDevice() const { return m_laserDevice.get(); }
    IProcessIo* processIo() const { return m_processIo.get(); }
    IProcessDevice* processDevice(const QString& instanceId) const;

    QString addDevice(ProcessDeviceKind kind, const QString& descriptorName, QString* errorMessage = nullptr);
    bool removeDevice(const QString& instanceId, QString* errorMessage = nullptr);
    bool connectDevice(const QString& instanceId, QString* errorMessage = nullptr);
    void disconnectDevice(const QString& instanceId);
    bool connectAllDevices(QStringList* errorMessages = nullptr, ProcessDeviceConnectProgress progress = {});
    void disconnectAllDevices();

    std::unique_ptr<lcnc::IMotionController> createMotionController(const lcnc::ProcessSettings& settings,
                                                                    QObject* parent,
                                                                    QString* errorMessage = nullptr) const;

    bool isMotionControllerAvailable(const QString& name) const;
    bool isLaserDeviceAvailable(const QString& name) const;
    bool isDeviceUsable(ProcessDeviceKind kind, const QString& name) const;
    const ProcessDeviceDescriptor* descriptor(ProcessDeviceKind kind, const QString& name) const;

private:
    void registerBuiltInDevices();
    void registerDefaultSessions();
    void updateMotionDeviceDescriptor();
    void updateLaserDeviceDescriptor();
    ProcessDeviceSession* sessionById(const QString& instanceId);
    const ProcessDeviceSession* sessionById(const QString& instanceId) const;
    IProcessDevice* runtimeDeviceForSession(const ProcessDeviceSession& session) const;
    QStringList names(ProcessDeviceKind kind) const;
    QString normalizedName(const QString& name) const;
    QString makeInstanceId(ProcessDeviceKind kind, const QString& descriptorName) const;

    QVector<ProcessDeviceDescriptor> m_descriptors;
    QVector<ProcessDeviceSession> m_sessions;
    QString m_activeMotionController;
    QString m_activeLaserDevice;
    std::unique_ptr<ILaserDevice> m_laserDevice;
    std::unique_ptr<IProcessIo> m_processIo;
    std::unique_ptr<IProcessDevice> m_motionDevice;
    lcnc::IMotionController* m_motionController{nullptr};
};

} // namespace lcnc::process
