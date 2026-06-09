#pragma once

#include <QList>
#include <QString>
#include <QStringList>

namespace lcnc::process {

/**
 * @brief Process peripheral category used by the device registry and UI.
 */
enum class ProcessDeviceKind
{
    MotionController,
    Laser,
    Io,
    Aux
};

/**
 * @brief Build/runtime availability of a registered peripheral type.
 */
enum class ProcessDeviceAvailability
{
    Simulation,
    Available,
    Unavailable
};

/**
 * @brief Runtime connection state shared by all Process peripherals.
 */
enum class ProcessDeviceConnectionState
{
    Disconnected,
    Connecting,
    Connected,
    Disconnecting,
    Error
};

/**
 * @brief One displayable peripheral status value.
 */
struct ProcessDeviceStatusItem
{
    QString name;
    QString value;
};

QString processDeviceKindName(ProcessDeviceKind kind);
QString processDeviceAvailabilityName(ProcessDeviceAvailability availability);
QString processDeviceConnectionStateName(ProcessDeviceConnectionState state);

/**
 * @brief Common Process peripheral interface for connection, state and diagnostics.
 *
 * This interface intentionally does not inherit IService. Concrete module
 * classes can expose Kernel services through adapters without introducing
 * multiple IService inheritance paths.
 */
class IProcessDevice
{
public:
    virtual ~IProcessDevice() = default;

    virtual QString deviceId() const = 0;
    virtual QString displayName() const = 0;
    virtual ProcessDeviceKind kind() const = 0;
    virtual QStringList capabilities() const = 0;

    virtual bool connectDevice(QString* errorMessage = nullptr) = 0;
    virtual void disconnectDevice() = 0;
    virtual bool isConnected() const = 0;
    virtual ProcessDeviceConnectionState connectionState() const = 0;
    virtual QString lastError() const = 0;
    virtual QList<ProcessDeviceStatusItem> statusItems() const = 0;
};

} // namespace lcnc::process