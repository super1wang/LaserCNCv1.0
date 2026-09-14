#include "core/kinematics/controller_kinematics_snapshot.h"

#include "core/kinematics/machine_calibration_service.h"
#include "core/kinematics/machine_configuration_service.h"

#include <algorithm>
#include <cmath>

namespace lcnc::kinematics {
namespace {

const MachineAxisRuntimeConfig* roleAxis(
    const QVector<MachineAxisRuntimeConfig>& axes, MachineAxisRole role)
{
    const MachineAxisRuntimeConfig* found = nullptr;
    for (const MachineAxisRuntimeConfig& axis : axes) {
        if (axis.axis.role != role) continue;
        if (found) return nullptr;
        found = &axis;
    }
    return found;
}

bool isDescendant(const QVector<MachineAxisRuntimeConfig>& axes,
                  const QString& child, const QString& ancestor)
{
    QString cursor = child;
    for (int depth = 0; depth <= axes.size(); ++depth) {
        const auto it = std::find_if(axes.cbegin(), axes.cend(), [&cursor](const auto& axis) {
            return axis.axis.name.compare(cursor, Qt::CaseInsensitive) == 0;
        });
        if (it == axes.cend()) return false;
        if (it->axis.parentAxis.compare(ancestor, Qt::CaseInsensitive) == 0) return true;
        if (it->axis.parentAxis.isEmpty() || it->axis.parentAxis == QStringLiteral("BASE")) return false;
        cursor = it->axis.parentAxis;
    }
    return false;
}

std::array<double, 3> pointArray(const gp_Pnt& point)
{
    return {point.X(), point.Y(), point.Z()};
}

} // namespace

bool buildControllerKinematicsSnapshot(
    const MachineConfigurationService& machine,
    const MachineCalibrationService* calibration,
    ControllerCalibrationRequirement calibrationRequirement,
    ControllerKinematicsSnapshot* snapshot,
    QString* error)
{
    if (!snapshot) return false;
    const QVector<MachineAxisRuntimeConfig> axes = machine.axisConfigurations();
    const auto* x = roleAxis(axes, MachineAxisRole::LinearX);
    const auto* y = roleAxis(axes, MachineAxisRole::LinearY);
    const auto* z = roleAxis(axes, MachineAxisRole::LinearZ);
    const auto* primary = roleAxis(axes, MachineAxisRole::TableTilt);
    const auto* slave = roleAxis(axes, MachineAxisRole::TableSpin);
    if (!x || !y || !z || !primary || !slave
        || !isDescendant(axes, slave->axis.name, primary->axis.name)) {
        if (error) *error = QStringLiteral("The active topology is not a supported serial five-axis table");
        return false;
    }
    const QString p = primary->axis.name.trimmed().toUpper();
    const QString s = slave->axis.name.trimmed().toUpper();
    short modelType = -1;
    if (p == QStringLiteral("A") && s == QStringLiteral("C")) modelType = 3;
    else if (p == QStringLiteral("B") && s == QStringLiteral("C")) modelType = 0;
    else if (p == QStringLiteral("A") && s == QStringLiteral("B")) modelType = 1;
    else if (p == QStringLiteral("C") && s == QStringLiteral("A")) modelType = 20;
    else if (p == QStringLiteral("C") && s == QStringLiteral("B")) modelType = 21;
    if (modelType < 0) {
        if (error) *error = QStringLiteral("The rotary-axis order does not map to a supported GTN table model");
        return false;
    }

    ControllerKinematicsSnapshot value;
    value.modelType = modelType;
    value.primaryAxisName = primary->axis.name;
    value.slaveAxisName = slave->axis.name;
    // GTN MCS is the right-handed machine-world frame used by CAM TCPs.
    // Controller-axis coordinates are joint values, not a Cartesian frame:
    // projecting into them erases linear-axis signs and can mirror a rotary
    // vector (Z-down + C-world-up used to become Z-up + C-down).
    // 中文翻译：GTN MCS 与 CAM TCP 共用右手机床世界系；不能用轴值坐标基投影方向。
    const auto controllerDirection = [](
        const gp_Dir& worldDirection, std::array<double, 3>* result) {
        if (!result) return false;
        const gp_Pnt axisVector(worldDirection.X(), worldDirection.Y(), worldDirection.Z());
        const double magnitude = std::sqrt(
            axisVector.X() * axisVector.X()
            + axisVector.Y() * axisVector.Y()
            + axisVector.Z() * axisVector.Z());
        if (!std::isfinite(magnitude) || magnitude <= 1e-12)
            return false;
        *result = {axisVector.X() / magnitude,
                   axisVector.Y() / magnitude,
                   axisVector.Z() / magnitude};
        return true;
    };
    gp_Pnt primaryWorld, slaveWorld;
    if (!machine.axisCoordinatesToWorld(primary->axis.origin, &primaryWorld)
        || !machine.axisCoordinatesToWorld(slave->axis.origin, &slaveWorld)) {
        if (error) *error = QStringLiteral("Cannot convert rotary centers to right-handed machine MCS");
        return false;
    }
    value.primaryAxisPointMcs = pointArray(primaryWorld);
    value.slaveAxisPointMcs = pointArray(slaveWorld);
    if (!controllerDirection(x->axis.direction, &value.axisVectorsMcs[0])
        || !controllerDirection(y->axis.direction, &value.axisVectorsMcs[1])
        || !controllerDirection(z->axis.direction, &value.axisVectorsMcs[2])
        || !controllerDirection(primary->axis.direction, &value.axisVectorsMcs[3])
        || !controllerDirection(slave->axis.direction, &value.axisVectorsMcs[4])) {
        if (error) *error = QStringLiteral(
            "The machine-axis directions cannot be represented in controller MCS");
        return false;
    }
    value.toolLocationPointMcs = {machine.headToolGeometry().installationOffsetX,
                                  machine.headToolGeometry().installationOffsetY,
                                  machine.headToolGeometry().installationOffsetZ};
    const MachineAxisRuntimeConfig* ordered[5] = {x, y, z, primary, slave};
    for (int index = 0; index < 5; ++index) {
        // The machine configuration stores controller-native indices.  A GTN
        // Group snapshot therefore consumes the configured physical axis number
        // directly; converting it again would shift both jogging and Group/RTCP.
        if (ordered[index]->controllerIndex < 1 || ordered[index]->controllerIndex > 8
            || !std::isfinite(ordered[index]->resolution)
            || ordered[index]->resolution <= 0.0) {
            if (error) *error = QStringLiteral("A five-axis controller index or scale is invalid");
            return false;
        }
        value.physicalAxisIndices[index] =
            static_cast<short>(ordered[index]->controllerIndex);
        value.scales[index].alpha = 1.0;
        value.scales[index].beta = ordered[index]->resolution;
    }
    value.directions[0] = isDescendant(axes, slave->axis.name, x->axis.name) ? 1 : 0;
    value.directions[1] = isDescendant(axes, slave->axis.name, y->axis.name) ? 1 : 0;
    value.directions[2] = isDescendant(axes, slave->axis.name, z->axis.name) ? 1 : 0;

    MachineCalibrationRecord active;
    QString calibrationError;
    // ACS (non-RTCP) executes the CAM-solved physical axes. An optional RTCP
    // record must neither override that model nor block it during migration.
    if (calibrationRequirement != ControllerCalibrationRequirement::None
        && calibration && calibration->activeRecord(&active, &calibrationError)) {
        if (active.schemaVersion != MachineCalibrationRecord::kCurrentSchemaVersion) {
            if (error) *error = QStringLiteral(
                "Legacy calibration uses ambiguous axis-coordinate MCS; regenerate or remeasure it in right-handed machine-world MCS");
            return false;
        }
        const QString activeMachineFingerprint =
            calibration->activeMachineConfigurationFingerprint();
        if (active.machineIdentity.compare(machine.presetName(), Qt::CaseInsensitive) != 0
            || activeMachineFingerprint.isEmpty()
            || activeMachineFingerprint != machine.configurationFingerprint()) {
            if (calibrationRequirement != ControllerCalibrationRequirement::None) {
                if (error) {
                    *error = QStringLiteral(
                        "The active calibration does not match the current machine configuration");
                }
                return false;
            }
            active = {};
        }
    }
    if (!active.calibrationId.isEmpty()) {
        value.primaryAxisPointMcs = active.primaryAxis.pointMcs;
        value.slaveAxisPointMcs = active.slaveAxis.pointMcs;
        value.axisVectorsMcs[3] = active.primaryAxis.unitVectorMcs;
        value.axisVectorsMcs[4] = active.slaveAxis.unitVectorMcs;
        value.toolLocationPointMcs = active.tool.toolLocationPointMcs;
        value.calibrationFingerprint = active.calibrationFingerprint;
        value.toolCalibrationFingerprint = active.tool.fingerprint;
        value.calibrationMachineVerified = active.verification.state
            == CalibrationVerificationState::MachineVerified;
        value.calibrationConfigurationDerived = active.verification.state
            == CalibrationVerificationState::ConfigurationDerived;
    } else if (calibrationRequirement != ControllerCalibrationRequirement::None) {
        if (error) *error = calibrationError.isEmpty()
            ? QStringLiteral("RTCP requires an active physical calibration")
            : calibrationError;
        return false;
    }
    if (calibrationRequirement == ControllerCalibrationRequirement::MachineVerified
        && !value.calibrationMachineVerified) {
        if (error) *error = QStringLiteral("RTCP requires a MachineVerified calibration record");
        return false;
    }
    if (calibrationRequirement
            == ControllerCalibrationRequirement::MachineVerifiedOrConfigurationDerived
        && !value.calibrationMachineVerified && !value.calibrationConfigurationDerived) {
        if (error) {
            *error = QStringLiteral(
                "RTCP requires a MachineVerified or configuration-derived calibration record");
        }
        return false;
    }
    value.machineKinematicsFingerprint = machine.configurationFingerprint();
    *snapshot = value;
    return true;
}

} // namespace lcnc::kinematics
