#include "core/kinematics/machine_topology.h"

#include <gp_Ax1.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include <cmath>

namespace lcnc {

QString machineAxisRoleName(MachineAxisRole role)
{
    switch (role) {
    case MachineAxisRole::LinearX:           return QStringLiteral("LinearX");
    case MachineAxisRole::LinearY:           return QStringLiteral("LinearY");
    case MachineAxisRole::LinearZ:           return QStringLiteral("LinearZ");
    case MachineAxisRole::WorkpieceRotary:   return QStringLiteral("WorkpieceRotary");
    case MachineAxisRole::TableTilt:         return QStringLiteral("TableTilt");
    case MachineAxisRole::TableSpin:         return QStringLiteral("TableSpin");
    case MachineAxisRole::HeadTiltPrimary:   return QStringLiteral("HeadTiltPrimary");
    case MachineAxisRole::HeadTiltSecondary: return QStringLiteral("HeadTiltSecondary");
    case MachineAxisRole::Unspecified:
    default:                                 return QStringLiteral("Unspecified");
    }
}

MachineAxisRole machineAxisRoleFromName(const QString& name)
{
    const QString normalized = name.trimmed();
    for (MachineAxisRole role : {
             MachineAxisRole::LinearX,
             MachineAxisRole::LinearY,
             MachineAxisRole::LinearZ,
             MachineAxisRole::WorkpieceRotary,
             MachineAxisRole::TableTilt,
             MachineAxisRole::TableSpin,
             MachineAxisRole::HeadTiltPrimary,
             MachineAxisRole::HeadTiltSecondary}) {
        if (machineAxisRoleName(role).compare(normalized, Qt::CaseInsensitive) == 0)
            return role;
    }
    return MachineAxisRole::Unspecified;
}

QString machiningModeName(MachiningMode mode)
{
    switch (mode) {
    case MachiningMode::RotaryTube4Axis:       return QStringLiteral("RotaryTube4Axis");
    case MachiningMode::SimultaneousTable5Axis:return QStringLiteral("SimultaneousTable5Axis");
    case MachiningMode::SimultaneousHead5Axis: return QStringLiteral("SimultaneousHead5Axis");
    case MachiningMode::Planar3Axis:
    default:                                   return QStringLiteral("Planar3Axis");
    }
}

int machiningModeSolverVersion(MachiningMode mode)
{
    switch (mode) {
    case MachiningMode::Planar3Axis:             return 3;
    case MachiningMode::RotaryTube4Axis:
    case MachiningMode::SimultaneousTable5Axis:  return 3;
    case MachiningMode::SimultaneousHead5Axis:  return 2;
    }
    return 0;
}

MachiningMode machiningModeFromName(const QString& name, MachiningMode fallback)
{
    const QString normalized = name.trimmed();
    for (MachiningMode mode : {
             MachiningMode::Planar3Axis,
             MachiningMode::RotaryTube4Axis,
             MachiningMode::SimultaneousTable5Axis,
             MachiningMode::SimultaneousHead5Axis}) {
        if (machiningModeName(mode).compare(normalized, Qt::CaseInsensitive) == 0)
            return mode;
    }
    return fallback;
}

bool MachineAxisLayout::append(const QString& name, MachineAxisRole role)
{
    const QString normalized = name.trimmed().toUpper();
    if (normalized.isEmpty() || count >= kMaxAxes || indexOfName(normalized) >= 0)
        return false;
    axes[count++] = {normalized, role};
    return true;
}

int MachineAxisLayout::indexOfName(const QString& name) const
{
    const QString normalized = name.trimmed().toUpper();
    for (int index = 0; index < count; ++index)
        if (axes[index].name == normalized)
            return index;
    return -1;
}

int MachineAxisLayout::indexOfRole(MachineAxisRole role) const
{
    for (int index = 0; index < count; ++index)
        if (axes[index].role == role)
            return index;
    return -1;
}

QStringList MachineAxisLayout::axisNames() const
{
    QStringList result;
    result.reserve(count);
    for (int index = 0; index < count; ++index)
        result.append(axes[index].name);
    return result;
}

bool MachineAxisLayout::isValid(QString* errorMessage) const
{
    if (count == 0 || count > kMaxAxes) {
        if (errorMessage) *errorMessage = QStringLiteral("Interpolated axis count must be between 1 and 5");
        return false;
    }
    QHash<QString, bool> names;
    QHash<MachineAxisRole, bool> roles;
    for (int index = 0; index < count; ++index) {
        const QString name = axes[index].name.trimmed().toUpper();
        if (name.isEmpty() || names.contains(name)) {
            if (errorMessage) *errorMessage = QStringLiteral("Interpolated axis names must be non-empty and unique");
            return false;
        }
        if (axes[index].role == MachineAxisRole::Unspecified || roles.contains(axes[index].role)) {
            if (errorMessage) *errorMessage = QStringLiteral("Interpolated axis roles must be specified and unique");
            return false;
        }
        names.insert(name, true);
        roles.insert(axes[index].role, true);
    }
    return true;
}

bool MachineAxisLayout::operator==(const MachineAxisLayout& other) const
{
    if (count != other.count) return false;
    for (int index = 0; index < count; ++index) {
        if (axes[index].name != other.axes[index].name
            || axes[index].role != other.axes[index].role) return false;
    }
    return true;
}

bool SolvedMachinePose::isActive(int index) const
{
    return index >= 0 && index < MachineAxisLayout::kMaxAxes
        && (activeMask & static_cast<std::uint8_t>(1u << index));
}

double SolvedMachinePose::value(int index) const
{
    return index >= 0 && index < MachineAxisLayout::kMaxAxes ? values[index] : 0.0;
}

void SolvedMachinePose::setValue(int index, double axisValue, bool active)
{
    if (index < 0 || index >= MachineAxisLayout::kMaxAxes)
        return;
    values[index] = axisValue;
    const auto bit = static_cast<std::uint8_t>(1u << index);
    if (active) activeMask |= bit;
    else activeMask &= static_cast<std::uint8_t>(~bit);
}

gp_Trsf WorkpieceSetupTransform::toTransform() const
{
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    gp_Trsf rx, ry, rz, translation;
    rx.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0)), rotationXDeg * kDegToRad);
    ry.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0)), rotationYDeg * kDegToRad);
    rz.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), rotationZDeg * kDegToRad);
    translation.SetTranslation(gp_Vec(x, y, z));
    return translation.Multiplied(rz.Multiplied(ry.Multiplied(rx)));
}

bool WorkpieceSetupTransform::isIdentity(double tolerance) const
{
    return std::abs(x) <= tolerance && std::abs(y) <= tolerance && std::abs(z) <= tolerance
        && std::abs(rotationXDeg) <= tolerance
        && std::abs(rotationYDeg) <= tolerance
        && std::abs(rotationZDeg) <= tolerance;
}

bool MachineModeDefinition::isValid(QString* errorMessage) const
{
    if (solverId.trimmed().isEmpty()) {
        if (errorMessage) *errorMessage = QStringLiteral("Machine mode solver id is empty");
        return false;
    }
    if (solverVersion <= 0) {
        if (errorMessage) *errorMessage = QStringLiteral("Machine mode solver version is invalid");
        return false;
    }
    if (!interpolatedAxes.isValid(errorMessage))
        return false;

    const auto hasRole = [this](MachineAxisRole role) {
        return interpolatedAxes.indexOfRole(role) >= 0;
    };
    const bool hasCartesian = hasRole(MachineAxisRole::LinearX)
        && hasRole(MachineAxisRole::LinearY)
        && hasRole(MachineAxisRole::LinearZ);
    bool modeLayoutMatches = false;
    switch (mode) {
    case MachiningMode::Planar3Axis:
        modeLayoutMatches = interpolatedAxes.count == 3 && hasCartesian;
        break;
    case MachiningMode::RotaryTube4Axis:
        modeLayoutMatches = interpolatedAxes.count == 4 && hasCartesian
            && (hasRole(MachineAxisRole::WorkpieceRotary)
                || hasRole(MachineAxisRole::TableSpin));
        break;
    case MachiningMode::SimultaneousTable5Axis:
        modeLayoutMatches = interpolatedAxes.count == 5 && hasCartesian
            && hasRole(MachineAxisRole::TableTilt)
            && hasRole(MachineAxisRole::TableSpin);
        break;
    case MachiningMode::SimultaneousHead5Axis:
        modeLayoutMatches = interpolatedAxes.count == 5 && hasCartesian
            && hasRole(MachineAxisRole::HeadTiltPrimary)
            && hasRole(MachineAxisRole::HeadTiltSecondary);
        break;
    }
    if (!modeLayoutMatches) {
        if (errorMessage) *errorMessage = QStringLiteral("Interpolated axis roles do not match the machining mode");
        return false;
    }
    for (auto it = lockedAxisTargets.cbegin(); it != lockedAxisTargets.cend(); ++it) {
        if (it.key().trimmed().isEmpty() || !std::isfinite(it.value())
            || interpolatedAxes.indexOfName(it.key()) >= 0) {
            if (errorMessage) *errorMessage = QStringLiteral("Locked-axis targets must be finite and outside the interpolated layout");
            return false;
        }
    }
    return true;
}

} // namespace lcnc
