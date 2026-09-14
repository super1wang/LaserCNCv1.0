#include "core/kinematics/toolpath_kinematics_solver.h"

#include "core/kinematics/ik_solver.h"
#include "core/kinematics/machine_kinematics.h"
#include "core/kinematics/machine_relative_kinematics.h"

#include <gp_Ax1.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace lcnc {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kReachTolerance = 1e-5;
// Discretised CAD normals can produce a few thousandths of a degree beyond a
// mathematically exact rotary limit.  Keep the command inside the configured
// limit, but do not hide a material soft-limit violation.
constexpr double kRotaryLimitNumericalToleranceDeg = 0.01;

gp_Trsf rotationOf(const MachineAxisDef& axis, double degrees)
{
    gp_Trsf transform;
    transform.SetRotation(gp_Ax1(axis.origin, axis.direction), degrees * kPi / 180.0);
    return transform;
}

double equivalentNear(double value, double reference)
{
    while (value - reference > 180.0) value -= 360.0;
    while (value - reference < -180.0) value += 360.0;
    return value;
}

const MachineAxisDef* axisForRole(const MachineKinematics* machine, MachineAxisRole role)
{
    if (!machine) return nullptr;
    for (const MachineAxisDef& axis : machine->axes())
        if (axis.role == role) return &axis;
    return nullptr;
}

double projectedLinear(const MachineAxisDef& axis, const gp_Pnt& point)
{
    return gp_Vec(gp_Pnt(0, 0, 0), point).Dot(gp_Vec(axis.direction));
}

SolvedMachinePose poseFromLegacy(const MachineCoord& coordinate,
                                 const MachineAxisLayout& layout)
{
    SolvedMachinePose pose;
    pose.valid = coordinate.valid;
    for (int index = 0; index < layout.count; ++index) {
        double value = 0.0;
        switch (layout.axes[index].role) {
        case MachineAxisRole::LinearX: value = coordinate.x; break;
        case MachineAxisRole::LinearY: value = coordinate.y; break;
        case MachineAxisRole::LinearZ: value = coordinate.z; break;
        default:
            if (layout.axes[index].name == coordinate.r1Name) value = coordinate.r1;
            else if (layout.axes[index].name == coordinate.r2Name) value = coordinate.r2;
            break;
        }
        pose.setValue(index, value);
    }
    if (!pose.valid) pose.failureReason = QStringLiteral("Inverse kinematics failed");
    return pose;
}

bool solveRelativeLinearCoordinates(const ToolpathKinematicsRequest& request,
                                    const QString& toolCarrierAxis,
                                    const QString& workpieceCarrierAxis,
                                    const gp_Pnt& workpiecePoint,
                                    const QMap<QString, double>& fixedAxisPositions,
                                    MachineCoord& coordinate,
                                    QString* failureReason)
{
    const MachineAxisDef* linearX = axisForRole(request.machine, MachineAxisRole::LinearX);
    const MachineAxisDef* linearY = axisForRole(request.machine, MachineAxisRole::LinearY);
    const MachineAxisDef* linearZ = axisForRole(request.machine, MachineAxisRole::LinearZ);
    if (!linearX || !linearY || !linearZ) {
        if (failureReason)
            *failureReason = QStringLiteral("The configured machine does not define three semantic linear axes");
        return false;
    }

    const QStringList linearAxes{linearX->name, linearY->name, linearZ->name};
    lcnc::kinematics::RelativeLinearSolveResult relative;
    lcnc::kinematics::RelativeLinearSolveError error =
        lcnc::kinematics::RelativeLinearSolveError::None;
    if (!lcnc::kinematics::solveRelativeLinearAxes(
            *request.machine, toolCarrierAxis, workpieceCarrierAxis,
            gp_Pnt(0.0, 0.0, 0.0), workpiecePoint, fixedAxisPositions,
            linearAxes, relative, &error)) {
        if (failureReason) {
            switch (error) {
            case lcnc::kinematics::RelativeLinearSolveError::SingularRelativeMotion:
                *failureReason = QStringLiteral("The tool/workpiece relative linear motion is singular for the configured axis tree");
                break;
            case lcnc::kinematics::RelativeLinearSolveError::AxisLimitExceeded:
                *failureReason = QStringLiteral("The relative tool/workpiece solution exceeds a configured linear-axis limit");
                break;
            case lcnc::kinematics::RelativeLinearSolveError::ForwardResidualExceeded:
                *failureReason = QStringLiteral("The relative tool/workpiece forward residual exceeds tolerance");
                break;
            default:
                *failureReason = QStringLiteral("The configured tool/workpiece carrier chains are incomplete");
                break;
            }
        }
        return false;
    }

    coordinate.x = relative.axisPositions.value(linearX->name);
    coordinate.y = relative.axisPositions.value(linearY->name);
    coordinate.z = relative.axisPositions.value(linearZ->name);
    return true;
}

MachineCoord tablePreviousFromPose(const SolvedMachinePose& pose,
                                   const MachineAxisLayout& layout,
                                   const QString& childSpinAxisName,
                                   const QString& parentTiltAxisName)
{
    MachineCoord coordinate;
    if (!pose.valid) return coordinate;

    const int spinIndex = layout.indexOfName(childSpinAxisName);
    const int tiltIndex = layout.indexOfName(parentTiltAxisName);
    if (spinIndex < 0 || tiltIndex < 0
        || !pose.isActive(spinIndex) || !pose.isActive(tiltIndex))
        return coordinate;

    // The persisted/controller layout is X/Y/Z/tilt/spin, while the table IK
    // decomposes the physical chain child-spin then parent-tilt.  Do not infer
    // r1/r2 from layout order here: that loses cross-contour continuity and
    // allows the solver to replace a stable +90 degree table tilt with -90.
    // 中文翻译：控制器轴布局为倾斜轴、旋转轴，转台逆解连续姿态必须按子旋转轴、父倾斜轴的物理链路恢复。
    coordinate.r1Name = childSpinAxisName;
    coordinate.r1 = pose.value(spinIndex);
    coordinate.r2Name = parentTiltAxisName;
    coordinate.r2 = pose.value(tiltIndex);
    coordinate.valid = true;
    return coordinate;
}

class Planar3AxisSolver final : public IToolpathKinematicsSolver
{
public:
    QString id() const override { return QStringLiteral("Planar3Axis"); }
    int version() const override { return machiningModeSolverVersion(MachiningMode::Planar3Axis); }

    std::vector<SolvedMachinePose> solve(const ToolpathKinematicsRequest& request) const override
    {
        std::vector<SolvedMachinePose> result;
        if (!request.machine || !request.points) return result;
        result.reserve(request.points->size());
        const gp_Trsf setup = request.workpieceSetup.toTransform();
        for (std::size_t pointIndex = 0; pointIndex < request.points->size(); ++pointIndex) {
            const ToolpathPoint& point = request.points->at(pointIndex);
            gp_Pnt position = point.position.Transformed(setup);
            // Planar processing deliberately uses a fixed machine-Z beam.
            // CAD face normals describe the source geometry only; they must
            // not turn an otherwise valid three-axis contour into an IK
            // failure.  The fixed normal is applied for both cutting and
            // lead-in requests through this common solver entry point.
            // 中文翻译：三轴加工固定使用机床 Z 向刀束；CAD 面法线只描述源几何，不能导致三轴轮廓逆解失败。
            SolvedMachinePose pose;
            bool valid = true;
            for (int axisIndex = 0; axisIndex < request.definition.interpolatedAxes.count; ++axisIndex) {
                const MachineAxisSlot& slot = request.definition.interpolatedAxes.axes[axisIndex];
                const MachineAxisDef* axis = axisForRole(request.machine, slot.role);
                if (!axis || axis->motionType != MachineAxisDef::Linear) { valid = false; break; }
                const double value = projectedLinear(*axis, position);
                if (value < axis->minVal - 1e-6 || value > axis->maxVal + 1e-6) { valid = false; break; }
                pose.setValue(axisIndex, value);
            }
            pose.valid = valid;
            if (!valid) pose.failureReason = QStringLiteral("Point %1 exceeds a configured linear-axis limit").arg(pointIndex);
            result.push_back(std::move(pose));
        }
        return result;
    }
};

class RotaryTube4AxisSolver final : public IToolpathKinematicsSolver
{
public:
    QString id() const override { return QStringLiteral("RotaryTube4Axis"); }
    int version() const override { return machiningModeSolverVersion(MachiningMode::RotaryTube4Axis); }

    std::vector<SolvedMachinePose> solve(const ToolpathKinematicsRequest& request) const override
    {
        std::vector<SolvedMachinePose> result;
        if (!request.machine || !request.points) return result;
        result.reserve(request.points->size());
        const MachineAxisDef* rotary = axisForRole(request.machine, MachineAxisRole::WorkpieceRotary);
        const MachineAxisDef* tilt = nullptr;
        double lockedTilt = 0.0;
        if (!rotary) {
            rotary = axisForRole(request.machine, MachineAxisRole::TableSpin);
            tilt = axisForRole(request.machine, MachineAxisRole::TableTilt);
            if (tilt) lockedTilt = request.definition.lockedAxisTargets.value(tilt->name, 90.0);
        }
        if (!rotary) return result;
        const MachineAxisDef* toolCarrier = axisForRole(
            request.machine, MachineAxisRole::LinearZ);
        if (!toolCarrier) return result;

        gp_Trsf setup = request.workpieceSetup.toTransform();
        gp_Trsf locked;
        if (tilt) locked = rotationOf(*tilt, lockedTilt);
        double previousAngle = 0.0;
        bool hasPrevious = false;
        const int rotaryIndex = request.definition.interpolatedAxes.indexOfName(rotary->name);
        if (request.previousPose && request.previousPose->valid && rotaryIndex >= 0) {
            previousAngle = request.previousPose->value(rotaryIndex);
            hasPrevious = true;
        }

        for (std::size_t pointIndex = 0; pointIndex < request.points->size(); ++pointIndex) {
            const gp_Pnt position = request.points->at(pointIndex).position.Transformed(setup);
            gp_Vec normal(request.points->at(pointIndex).normal);
            normal.Transform(setup);
            gp_Vec target(0, 0, 1);
            if (tilt) {
                const gp_Trsf inverseLocked = locked.Inverted();
                target.Transform(inverseLocked);
            }
            const gp_Vec axisVector(rotary->direction);
            const double parallelError = std::abs(normal.Dot(axisVector) - target.Dot(axisVector));
            SolvedMachinePose pose;
            if (parallelError > kReachTolerance) {
                pose.failureReason = QStringLiteral("Point %1 normal is unreachable by rotary axis %2")
                    .arg(pointIndex).arg(rotary->name);
                result.push_back(std::move(pose));
                continue;
            }
            gp_Vec from = normal - axisVector * normal.Dot(axisVector);
            gp_Vec to = target - axisVector * target.Dot(axisVector);
            if (from.Magnitude() <= 1e-10 || to.Magnitude() <= 1e-10) {
                pose.failureReason = QStringLiteral("Point %1 lies in a rotary singular direction").arg(pointIndex);
                result.push_back(std::move(pose));
                continue;
            }
            from.Normalize(); to.Normalize();
            const double dot = std::clamp(from.Dot(to), -1.0, 1.0);
            double angle = std::acos(dot) * 180.0 / kPi;
            if (from.Crossed(to).Dot(axisVector) < 0.0) angle = -angle;
            if (hasPrevious) angle = equivalentNear(angle, previousAngle);
            if (angle < rotary->minVal - kRotaryLimitNumericalToleranceDeg
                || angle > rotary->maxVal + kRotaryLimitNumericalToleranceDeg) {
                pose.failureReason = QStringLiteral("Point %1 requires %2=%3 outside soft limits")
                    .arg(pointIndex).arg(rotary->name).arg(angle, 0, 'f', 3);
                result.push_back(std::move(pose));
                continue;
            }
            angle = std::clamp(angle, rotary->minVal, rotary->maxVal);
            bool valid = rotaryIndex >= 0;
            for (int axisIndex = 0; axisIndex < request.definition.interpolatedAxes.count; ++axisIndex) {
                const MachineAxisSlot& slot = request.definition.interpolatedAxes.axes[axisIndex];
                if (axisIndex == rotaryIndex) { pose.setValue(axisIndex, angle); continue; }
                if (slot.role == MachineAxisRole::LinearX
                    || slot.role == MachineAxisRole::LinearY
                    || slot.role == MachineAxisRole::LinearZ) {
                    continue;
                }
                valid = false;
                break;
            }
            MachineCoord coordinate;
            coordinate.r1Name = rotary->name;
            coordinate.r1 = angle;
            coordinate.valid = valid;
            QMap<QString, double> fixedPositions{{rotary->name, angle}};
            if (tilt)
                fixedPositions.insert(tilt->name, lockedTilt);
            QString relativeFailure;
            if (valid && !solveRelativeLinearCoordinates(
                    request, toolCarrier->name, rotary->name, position,
                    fixedPositions, coordinate, &relativeFailure)) {
                valid = false;
            }
            if (valid) {
                const SolvedMachinePose solved = poseFromLegacy(
                    coordinate, request.definition.interpolatedAxes);
                for (int axisIndex = 0;
                     axisIndex < request.definition.interpolatedAxes.count; ++axisIndex) {
                    if (axisIndex != rotaryIndex)
                        pose.setValue(axisIndex, solved.value(axisIndex));
                }
            }
            pose.valid = valid;
            if (!valid) {
                pose.failureReason = relativeFailure.isEmpty()
                    ? QStringLiteral("Point %1 exceeds a configured axis limit").arg(pointIndex)
                    : QStringLiteral("Point %1: %2").arg(pointIndex).arg(relativeFailure);
            }
            else { previousAngle = angle; hasPrevious = true; }
            result.push_back(std::move(pose));
        }
        return result;
    }
};

class Head5AxisSolver final : public IToolpathKinematicsSolver
{
public:
    QString id() const override { return QStringLiteral("SimultaneousHead5Axis"); }
    int version() const override { return machiningModeSolverVersion(MachiningMode::SimultaneousHead5Axis); }

    std::vector<SolvedMachinePose> solve(const ToolpathKinematicsRequest& request) const override
    {
        std::vector<SolvedMachinePose> result;
        if (!request.machine || !request.points || request.headToolGeometry.focusLength <= 0.0)
            return result;
        const MachineAxisDef* primary = axisForRole(request.machine, MachineAxisRole::HeadTiltPrimary);
        const MachineAxisDef* secondary = axisForRole(request.machine, MachineAxisRole::HeadTiltSecondary);
        if (!primary || !secondary) return result;
        const int primaryIndex = request.definition.interpolatedAxes.indexOfRole(MachineAxisRole::HeadTiltPrimary);
        const int secondaryIndex = request.definition.interpolatedAxes.indexOfRole(MachineAxisRole::HeadTiltSecondary);
        gp_Vec zeroBeam(request.headToolGeometry.zeroBeamX,
                        request.headToolGeometry.zeroBeamY,
                        request.headToolGeometry.zeroBeamZ);
        if (zeroBeam.Magnitude() <= 1e-10) return result;
        zeroBeam.Normalize();
        const bool secondaryIsChild = secondary->parentAxis == primary->name;
        const auto totalRotation = [&](double primaryAngle, double secondaryAngle) {
            const gp_Trsf primaryRotation = rotationOf(*primary, primaryAngle);
            const gp_Trsf secondaryRotation = rotationOf(*secondary, secondaryAngle);
            return secondaryIsChild ? primaryRotation.Multiplied(secondaryRotation)
                                    : secondaryRotation.Multiplied(primaryRotation);
        };
        const auto directionAt = [&](double primaryAngle, double secondaryAngle) {
            gp_Vec direction = zeroBeam;
            direction.Transform(totalRotation(primaryAngle, secondaryAngle));
            direction.Normalize();
            return direction;
        };

        double previousPrimary = 0.0, previousSecondary = 0.0;
        bool hasPrevious = request.previousPose && request.previousPose->valid;
        if (hasPrevious) {
            previousPrimary = request.previousPose->value(primaryIndex);
            previousSecondary = request.previousPose->value(secondaryIndex);
        }
        const gp_Trsf setup = request.workpieceSetup.toTransform();
        result.reserve(request.points->size());
        for (std::size_t pointIndex = 0; pointIndex < request.points->size(); ++pointIndex) {
            gp_Pnt position = request.points->at(pointIndex).position.Transformed(setup);
            gp_Vec normal(request.points->at(pointIndex).normal); normal.Transform(setup); normal.Normalize();
            const gp_Vec desired = -normal;
            double bestPrimary = 0.0, bestSecondary = 0.0;
            double bestError = std::numeric_limits<double>::max();
            const std::array<std::pair<double, double>, 5> seeds{{
                {hasPrevious ? previousPrimary : 0.0, hasPrevious ? previousSecondary : 0.0},
                {0.0, 0.0},
                {primary->minVal, 0.0}, {primary->maxVal, 0.0},
                {0.0, secondary->maxVal}}};
            for (auto seed : seeds) {
                double a = std::clamp(seed.first, primary->minVal, primary->maxVal);
                double b = std::clamp(seed.second, secondary->minVal, secondary->maxVal);
                for (int iteration = 0; iteration < 60; ++iteration) {
                    const gp_Vec current = directionAt(a, b);
                    const gp_Vec residual = current - desired;
                    if (residual.Magnitude() < 1e-8) break;
                    constexpr double deltaDeg = 0.01;
                    const gp_Vec da = (directionAt(a + deltaDeg, b) - current) / deltaDeg;
                    const gp_Vec db = (directionAt(a, b + deltaDeg) - current) / deltaDeg;
                    const double aa = da.Dot(da), ab = da.Dot(db), bb = db.Dot(db);
                    const double ar = da.Dot(residual), br = db.Dot(residual);
                    const double determinant = aa * bb - ab * ab;
                    if (std::abs(determinant) < 1e-14) break;
                    const double stepA = std::clamp(-(bb * ar - ab * br) / determinant, -10.0, 10.0);
                    const double stepB = std::clamp(-(-ab * ar + aa * br) / determinant, -10.0, 10.0);
                    a = std::clamp(a + stepA, primary->minVal, primary->maxVal);
                    b = std::clamp(b + stepB, secondary->minVal, secondary->maxVal);
                }
                const double error = (directionAt(a, b) - desired).Magnitude();
                const double continuityPenalty = hasPrevious
                    ? 1e-9 * (std::abs(a - previousPrimary) + std::abs(b - previousSecondary)) : 0.0;
                if (error + continuityPenalty < bestError) {
                    bestError = error + continuityPenalty;
                    bestPrimary = a; bestSecondary = b;
                }
            }

            SolvedMachinePose pose;
            if (bestError > kReachTolerance) {
                pose.failureReason = QStringLiteral("Point %1 is singular or unreachable for the configured head")
                    .arg(pointIndex);
                result.push_back(std::move(pose));
                continue;
            }
            const gp_Trsf rotation = totalRotation(bestPrimary, bestSecondary);
            gp_Vec focusOffset = zeroBeam * request.headToolGeometry.focusLength
                + gp_Vec(request.headToolGeometry.installationOffsetX,
                         request.headToolGeometry.installationOffsetY,
                         request.headToolGeometry.installationOffsetZ);
            focusOffset.Transform(rotation);
            const gp_Pnt compensated = position.Translated(-focusOffset);
            bool valid = primaryIndex >= 0 && secondaryIndex >= 0;
            for (int axisIndex = 0; axisIndex < request.definition.interpolatedAxes.count; ++axisIndex) {
                const MachineAxisRole role = request.definition.interpolatedAxes.axes[axisIndex].role;
                if (role == MachineAxisRole::HeadTiltPrimary) { pose.setValue(axisIndex, bestPrimary); continue; }
                if (role == MachineAxisRole::HeadTiltSecondary) { pose.setValue(axisIndex, bestSecondary); continue; }
                const MachineAxisDef* axis = axisForRole(request.machine, role);
                if (!axis) { valid = false; break; }
                const double value = projectedLinear(*axis, compensated);
                if (value < axis->minVal - 1e-6 || value > axis->maxVal + 1e-6) { valid = false; break; }
                pose.setValue(axisIndex, value);
            }
            pose.valid = valid;
            if (!valid) pose.failureReason = QStringLiteral("Point %1 exceeds a configured head/linear-axis limit").arg(pointIndex);
            else { previousPrimary = bestPrimary; previousSecondary = bestSecondary; hasPrevious = true; }
            result.push_back(std::move(pose));
        }
        return result;
    }
};

class Table5AxisSolver final : public IToolpathKinematicsSolver
{
public:
    explicit Table5AxisSolver(MachiningMode mode) : m_mode(mode) {}
    QString id() const override { return machiningModeName(m_mode); }
    int version() const override { return machiningModeSolverVersion(m_mode); }
    std::vector<SolvedMachinePose> solve(const ToolpathKinematicsRequest& request) const override
    {
        std::vector<SolvedMachinePose> result;
        if (!request.machine || !request.points) return result;
        result.reserve(request.points->size());
        const gp_Trsf setup = request.workpieceSetup.toTransform();
        const MachineAxisDef* tilt = axisForRole(request.machine, MachineAxisRole::TableTilt);
        const MachineAxisDef* spin = axisForRole(request.machine, MachineAxisRole::TableSpin);
        const MachineAxisDef* toolCarrier = axisForRole(
            request.machine, MachineAxisRole::LinearZ);
        if (!tilt || !spin || !toolCarrier) return result;
        MachineCoord previous = request.previousPose
            ? tablePreviousFromPose(*request.previousPose, request.definition.interpolatedAxes,
                                    spin->name, tilt->name)
            : MachineCoord{};
        for (const ToolpathPoint& point : *request.points) {
            gp_Pnt position = point.position.Transformed(setup);
            gp_Vec normal(point.normal); normal.Transform(setup);
            // The table solver decomposes the transform in application order:
            // child spin first, then parent tilt.  In an AC/BC table the
            // TableSpin role is the child of TableTilt, even though the output
            // layout deliberately remains X/Y/Z/tilt/spin for controllers.
            // 中文翻译：转台逆解按子旋转轴、父倾斜轴的实际变换顺序分解。
            MachineCoord coordinate = IKSolver::solveTableContinuous(
                request.machine, position, gp_Dir(normal), spin->name, tilt->name,
                previous.valid ? &previous : nullptr);
            QString relativeFailure;
            if (coordinate.valid) {
                const QMap<QString, double> fixedPositions{
                    {spin->name, coordinate.r1},
                    {tilt->name, coordinate.r2}};
                if (!solveRelativeLinearCoordinates(
                        request, toolCarrier->name, spin->name, position,
                        fixedPositions, coordinate, &relativeFailure)) {
                    coordinate.valid = false;
                }
            }
            SolvedMachinePose pose = poseFromLegacy(
                coordinate, request.definition.interpolatedAxes);
            if (!coordinate.valid && !relativeFailure.isEmpty())
                pose.failureReason = relativeFailure;
            result.push_back(std::move(pose));
            if (coordinate.valid) previous = coordinate;
        }
        return result;
    }
private:
    MachiningMode m_mode;
};

} // namespace

ToolpathSolverRegistry::ToolpathSolverRegistry()
{
    m_solvers.insert(MachiningMode::Planar3Axis, std::make_shared<Planar3AxisSolver>());
    m_solvers.insert(MachiningMode::RotaryTube4Axis, std::make_shared<RotaryTube4AxisSolver>());
    m_solvers.insert(MachiningMode::SimultaneousTable5Axis,
                     std::make_shared<Table5AxisSolver>(MachiningMode::SimultaneousTable5Axis));
    m_solvers.insert(MachiningMode::SimultaneousHead5Axis,
                     std::make_shared<Head5AxisSolver>());
}

const IToolpathKinematicsSolver* ToolpathSolverRegistry::solver(MachiningMode mode) const
{
    const auto it = m_solvers.constFind(mode);
    return it == m_solvers.cend() ? nullptr : it.value().get();
}

std::vector<SolvedMachinePose> ToolpathSolverRegistry::solve(
    const ToolpathKinematicsRequest& request) const
{
    const IToolpathKinematicsSolver* selected = solver(request.mode);
    if (!selected || selected->id() != request.definition.solverId
        || selected->version() != request.definition.solverVersion)
        return {};
    return selected->solve(request);
}

} // namespace lcnc
