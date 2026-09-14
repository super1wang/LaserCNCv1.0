#include "core/kinematics/machine_calibration.h"

#include "core/kinematics/machine_kinematics.h"

#include <QMap>
#include <QStringList>

#include <cmath>

namespace lcnc::kinematics {
namespace {

constexpr double kParallelTolerance = 1e-10;
constexpr double kAxisIntersectionTolerance = 0.1;
constexpr double kCorrectionResidualTolerance = 1e-5;

void setError(CalibrationPlanError* error, CalibrationPlanError value)
{
    if (error)
        *error = value;
}

const MachineAxisDef* axisForRole(const MachineKinematics& kinematics,
                                  lcnc::MachineAxisRole role)
{
    for (const MachineAxisDef& axis : kinematics.axes()) {
        if (axis.role == role)
            return &axis;
    }
    return nullptr;
}

bool solveDenseSystem(QList<QList<double>>& matrix,
                      QList<double>& rhs,
                      QList<double>& solution)
{
    const int size = matrix.size();
    solution.fill(0.0, size);
    for (int pivot = 0; pivot < size; ++pivot) {
        int best = pivot;
        for (int row = pivot + 1; row < size; ++row) {
            if (std::abs(matrix[row][pivot]) > std::abs(matrix[best][pivot]))
                best = row;
        }
        if (std::abs(matrix[best][pivot]) < kParallelTolerance)
            return false;
        if (best != pivot) {
            matrix.swapItemsAt(best, pivot);
            rhs.swapItemsAt(best, pivot);
        }
        for (int row = pivot + 1; row < size; ++row) {
            const double factor = matrix[row][pivot] / matrix[pivot][pivot];
            for (int column = pivot; column < size; ++column)
                matrix[row][column] -= factor * matrix[pivot][column];
            rhs[row] -= factor * rhs[pivot];
        }
    }
    for (int row = size - 1; row >= 0; --row) {
        double value = rhs[row];
        for (int column = row + 1; column < size; ++column)
            value -= matrix[row][column] * solution[column];
        solution[row] = value / matrix[row][row];
    }
    return true;
}

} // namespace

bool tableRotationCenterFromReferences(const MachineAxisDef& tiltAxis,
                                       const gp_Pnt& tiltReferenceCenter,
                                       const MachineAxisDef& spinAxis,
                                       const gp_Pnt& spinReferenceCenter,
                                       gp_Pnt& center,
                                       double* axisSeparation)
{
    const gp_Vec tiltDirection(tiltAxis.direction);
    const gp_Vec spinDirection(spinAxis.direction);
    const double directionDot = tiltDirection.Dot(spinDirection);
    const double denominator = 1.0 - directionDot * directionDot;
    if (std::abs(denominator) < kParallelTolerance)
        return false;

    const gp_Vec between(spinReferenceCenter, tiltReferenceCenter);
    const double tiltDot = tiltDirection.Dot(between);
    const double spinDot = spinDirection.Dot(between);
    const double tiltParameter =
        (directionDot * spinDot - tiltDot) / denominator;
    const double spinParameter =
        (spinDot - directionDot * tiltDot) / denominator;
    const gp_Pnt pointOnTilt = tiltReferenceCenter.Translated(
        tiltDirection * tiltParameter);
    const gp_Pnt pointOnSpin = spinReferenceCenter.Translated(
        spinDirection * spinParameter);
    center = pointOnTilt;
    if (axisSeparation)
        *axisSeparation = pointOnTilt.Distance(pointOnSpin);
    return true;
}

bool buildTableCalibrationPlan(const MachineKinematics& kinematics,
                               const gp_Pnt& tiltReferenceCenter,
                               const gp_Pnt& spinReferenceCenter,
                               const gp_Pnt& cutterReferenceCenter,
                               const gp_Pnt& physicalRotationCenter,
                               const gp_Pnt& physicalTcp,
                               TableCalibrationPlan& plan,
                               CalibrationPlanError* error)
{
    plan = {};
    setError(error, CalibrationPlanError::None);

    const MachineAxisDef* tiltAxis = axisForRole(
        kinematics, lcnc::MachineAxisRole::TableTilt);
    const MachineAxisDef* spinAxis = axisForRole(
        kinematics, lcnc::MachineAxisRole::TableSpin);
    const MachineAxisDef* toolCarrier = axisForRole(
        kinematics, lcnc::MachineAxisRole::LinearZ);
    if (!tiltAxis || !spinAxis || !toolCarrier) {
        setError(error, CalibrationPlanError::MissingSemanticAxis);
        return false;
    }

    plan.tiltAxisName = tiltAxis->name;
    plan.spinAxisName = spinAxis->name;
    plan.toolCarrierAxisName = toolCarrier->name;
    const QStringList toolChain = kinematics.axisChain(toolCarrier->name);
    if (toolChain.isEmpty()) {
        setError(error, CalibrationPlanError::InvalidToolCarrierChain);
        return false;
    }

    if (!tableRotationCenterFromReferences(
            *tiltAxis, tiltReferenceCenter, *spinAxis, spinReferenceCenter,
            plan.modelRotationCenter, &plan.rotaryAxisSeparation)) {
        setError(error, CalibrationPlanError::ParallelRotaryAxes);
        return false;
    }
    if (plan.rotaryAxisSeparation > kAxisIntersectionTolerance) {
        setError(error, CalibrationPlanError::SkewRotaryReferences);
        return false;
    }

    plan.wholeMachineTranslation = gp_Vec(
        plan.modelRotationCenter, physicalRotationCenter);
    const gp_Pnt alignedCutter = cutterReferenceCenter.Translated(
        plan.wholeMachineTranslation);
    plan.cutterTranslation = gp_Vec(alignedCutter, physicalTcp);

    QMap<QString, double> basePositions;
    for (const MachineAxisDef& axis : kinematics.axes())
        basePositions.insert(axis.name, axis.currentPos);

    QStringList carrierLinearAxes;
    QList<gp_Vec> motionColumns;
    QMap<QString, double> zeroCarrierPositions = basePositions;
    for (const QString& axisName : toolChain) {
        const MachineAxisDef* axis = kinematics.findAxis(axisName);
        if (axis && axis->motionType == MachineAxisDef::Linear
            && axis->role != lcnc::MachineAxisRole::Unspecified) {
            carrierLinearAxes.append(axis->name);
            zeroCarrierPositions.insert(axis->name, 0.0);
        }
    }
    if (carrierLinearAxes.isEmpty() || carrierLinearAxes.size() > 3) {
        setError(error, CalibrationPlanError::DegenerateToolCarrierMotion);
        return false;
    }

    const gp_Pnt toolZero = gp_Pnt(0.0, 0.0, 0.0).Transformed(
        kinematics.computeAxisTransform(toolCarrier->name, zeroCarrierPositions));
    for (const QString& axisName : carrierLinearAxes) {
        QMap<QString, double> unitPositions = zeroCarrierPositions;
        unitPositions.insert(axisName, 1.0);
        const gp_Pnt toolUnit = gp_Pnt(0.0, 0.0, 0.0).Transformed(
            kinematics.computeAxisTransform(toolCarrier->name, unitPositions));
        motionColumns.append(gp_Vec(toolZero, toolUnit));
    }

    const int count = motionColumns.size();
    QList<QList<double>> normalMatrix(count, QList<double>(count, 0.0));
    QList<double> rhs(count, 0.0);
    for (int row = 0; row < count; ++row) {
        rhs[row] = motionColumns[row].Dot(plan.cutterTranslation);
        for (int column = 0; column < count; ++column)
            normalMatrix[row][column] = motionColumns[row].Dot(motionColumns[column]);
    }
    QList<double> coefficients;
    if (!solveDenseSystem(normalMatrix, rhs, coefficients)) {
        setError(error, CalibrationPlanError::DegenerateToolCarrierMotion);
        return false;
    }

    gp_Vec reproduced(0.0, 0.0, 0.0);
    for (int index = 0; index < count; ++index) {
        const gp_Vec correction = motionColumns[index] * coefficients[index];
        reproduced += correction;
        plan.carrierCorrections.append({carrierLinearAxes[index], correction});
    }
    plan.cutterResidual = (plan.cutterTranslation - reproduced).Magnitude();
    if (plan.cutterResidual > kCorrectionResidualTolerance) {
        plan.carrierCorrections.clear();
        setError(error, CalibrationPlanError::UnreachableToolCorrection);
        return false;
    }
    return true;
}

gp_Vec inheritedCarrierCorrection(const MachineKinematics& kinematics,
                                  const QString& assignedAxis,
                                  const QList<AxisCarrierCorrection>& corrections)
{
    gp_Vec inherited(0.0, 0.0, 0.0);
    for (const AxisCarrierCorrection& correction : corrections) {
        if (kinematics.isAxisDescendantOf(assignedAxis, correction.axisName))
            inherited += correction.translation;
    }
    return inherited;
}

} // namespace lcnc::kinematics
