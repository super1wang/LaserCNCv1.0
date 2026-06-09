#include "modules/process/device/i_process_device.h"

namespace lcnc::process {

QString processDeviceKindName(ProcessDeviceKind kind)
{
    switch (kind) {
    case ProcessDeviceKind::MotionController: return QStringLiteral("Motion");
    case ProcessDeviceKind::Laser: return QStringLiteral("Laser");
    case ProcessDeviceKind::Io: return QStringLiteral("IO");
    case ProcessDeviceKind::Aux: return QStringLiteral("Aux");
    }
    return QStringLiteral("Unknown");
}

QString processDeviceAvailabilityName(ProcessDeviceAvailability availability)
{
    switch (availability) {
    case ProcessDeviceAvailability::Simulation: return QStringLiteral("Simulation");
    case ProcessDeviceAvailability::Available: return QStringLiteral("Available");
    case ProcessDeviceAvailability::Unavailable: return QStringLiteral("Unavailable");
    }
    return QStringLiteral("Unknown");
}

QString processDeviceConnectionStateName(ProcessDeviceConnectionState state)
{
    switch (state) {
    case ProcessDeviceConnectionState::Disconnected: return QStringLiteral("Disconnected");
    case ProcessDeviceConnectionState::Connecting: return QStringLiteral("Connecting");
    case ProcessDeviceConnectionState::Connected: return QStringLiteral("Connected");
    case ProcessDeviceConnectionState::Disconnecting: return QStringLiteral("Disconnecting");
    case ProcessDeviceConnectionState::Error: return QStringLiteral("Error");
    }
    return QStringLiteral("Unknown");
}

} // namespace lcnc::process