#include "core/kinematics/machine_relative_kinematics.h"

#include "core/kinematics/machine_kinematics.h"

#include <gp_Vec.hxx>

#include <algorithm>
#include <array>
#include <cmath>

namespace lcnc::kinematics {
namespace {

constexpr double kRankTolerance = 1e-10;
constexpr double kForwardTolerance = 1e-5;

void setError(RelativeLinearSolveError* error, RelativeLinearSolveError value)
{
    if (error)
        *error = value;
}

gp_Pnt transformedPoint(const MachineKinematics& kinematics,
                        const QString& carrierAxis,
                        const gp_Pnt& localPoint,
                        const QMap<QString, double>& positions)
{
    return localPoint.Transformed(
        kinematics.computeAxisTransform(carrierAxis, positions));
}

bool solveThreeByThree(double matrix[3][3], double rhs[3], double solution[3])
{
    for (int pivot = 0; pivot < 3; ++pivot) {
        int best = pivot;
        for (int row = pivot + 1; row < 3; ++row) {
            if (std::abs(matrix[row][pivot]) > std::abs(matrix[best][pivot]))
                best = row;
        }
        if (std::abs(matrix[best][pivot]) < kRankTolerance)
            return false;
        if (best != pivot) {
            for (int column = pivot; column < 3; ++column)
                std::swap(matrix[pivot][column], matrix[best][column]);
            std::swap(rhs[pivot], rhs[best]);
        }
        for (int row = pivot + 1; row < 3; ++row) {
            const double factor = matrix[row][pivot] / matrix[pivot][pivot];
            for (int column = pivot; column < 3; ++column)
                matrix[row][column] -= factor * matrix[pivot][column];
            rhs[row] -= factor * rhs[pivot];
        }
    }

    for (int row = 2; row >= 0; --row) {
        double value = rhs[row];
        for (int column = row + 1; column < 3; ++column)
            value -= matrix[row][column] * solution[column];
        solution[row] = value / matrix[row][row];
    }
    return true;
}

bool buildRelativeMatrix(const MachineKinematics& kinematics,
                         const QString& toolCarrierAxis,
                         const QString& workpieceCarrierAxis,
                         const gp_Pnt& toolLocalPoint,
                         const gp_Pnt& workpieceLocalPoint,
                         const QMap<QString, double>& fixedAxisPositions,
                         const QStringList& linearAxisNames,
                         double matrix[3][3],
                         gp_Vec& zeroResidual)
{
    if (kinematics.axisChain(toolCarrierAxis).isEmpty()
        || kinematics.axisChain(workpieceCarrierAxis).isEmpty()
        || linearAxisNames.size() != 3) {
        return false;
    }

    const gp_Pnt toolZero = transformedPoint(
        kinematics, toolCarrierAxis, toolLocalPoint, fixedAxisPositions);
    const gp_Pnt workpieceZero = transformedPoint(
        kinematics, workpieceCarrierAxis, workpieceLocalPoint, fixedAxisPositions);
    zeroResidual = gp_Vec(workpieceZero, toolZero);

    for (int column = 0; column < 3; ++column) {
        const QString axisName = linearAxisNames[column].trimmed().toUpper();
        const MachineAxisDef* axis = kinematics.findAxis(axisName);
        if (!axis || axis->motionType != MachineAxisDef::Linear)
            return false;
        QMap<QString, double> unitPositions = fixedAxisPositions;
        unitPositions.insert(axisName, 1.0);
        const gp_Pnt toolUnit = transformedPoint(
            kinematics, toolCarrierAxis, toolLocalPoint, unitPositions);
        const gp_Pnt workpieceUnit = transformedPoint(
            kinematics, workpieceCarrierAxis, workpieceLocalPoint, unitPositions);
        const gp_Vec unitResidual(workpieceUnit, toolUnit);
        const gp_Vec coefficient = unitResidual - zeroResidual;
        matrix[0][column] = coefficient.X();
        matrix[1][column] = coefficient.Y();
        matrix[2][column] = coefficient.Z();
    }
    return true;
}

} // namespace

bool solveRelativeLinearAxes(const MachineKinematics& kinematics,
                             const QString& toolCarrierAxis,
                             const QString& workpieceCarrierAxis,
                             const gp_Pnt& toolLocalPoint,
                             const gp_Pnt& workpieceLocalPoint,
                             const QMap<QString, double>& fixedAxisPositions,
                             const QStringList& linearAxisNames,
                             RelativeLinearSolveResult& result,
                             RelativeLinearSolveError* error)
{
    result = {};
    setError(error, RelativeLinearSolveError::None);

    double matrix[3][3]{};
    gp_Vec zeroResidual;
    if (!buildRelativeMatrix(kinematics, toolCarrierAxis, workpieceCarrierAxis,
                             toolLocalPoint, workpieceLocalPoint, fixedAxisPositions,
                             linearAxisNames, matrix, zeroResidual)) {
        setError(error, kinematics.axisChain(toolCarrierAxis).isEmpty()
                           || kinematics.axisChain(workpieceCarrierAxis).isEmpty()
                       ? RelativeLinearSolveError::InvalidCarrierChain
                       : RelativeLinearSolveError::MissingLinearAxis);
        return false;
    }

    double rhs[3]{-zeroResidual.X(), -zeroResidual.Y(), -zeroResidual.Z()};
    double solution[3]{};
    if (!solveThreeByThree(matrix, rhs, solution)) {
        setError(error, RelativeLinearSolveError::SingularRelativeMotion);
        return false;
    }

    QMap<QString, double> solvedPositions = fixedAxisPositions;
    for (int index = 0; index < 3; ++index) {
        const QString axisName = linearAxisNames[index].trimmed().toUpper();
        const MachineAxisDef* axis = kinematics.findAxis(axisName);
        if (!axis || solution[index] < axis->minVal - 1e-6
            || solution[index] > axis->maxVal + 1e-6) {
            setError(error, RelativeLinearSolveError::AxisLimitExceeded);
            return false;
        }
        result.axisPositions.insert(axisName,
                                    std::clamp(solution[index], axis->minVal, axis->maxVal));
        solvedPositions.insert(axisName, result.axisPositions.value(axisName));
    }

    const gp_Pnt toolWorld = transformedPoint(
        kinematics, toolCarrierAxis, toolLocalPoint, solvedPositions);
    const gp_Pnt workpieceWorld = transformedPoint(
        kinematics, workpieceCarrierAxis, workpieceLocalPoint, solvedPositions);
    result.forwardResidual = toolWorld.Distance(workpieceWorld);
    if (result.forwardResidual > kForwardTolerance) {
        result.axisPositions.clear();
        setError(error, RelativeLinearSolveError::ForwardResidualExceeded);
        return false;
    }
    return true;
}

bool hasFullRankRelativeLinearMotion(const MachineKinematics& kinematics,
                                     const QString& toolCarrierAxis,
                                     const QString& workpieceCarrierAxis,
                                     const QStringList& linearAxisNames)
{
    double matrix[3][3]{};
    gp_Vec zeroResidual;
    if (!buildRelativeMatrix(kinematics, toolCarrierAxis, workpieceCarrierAxis,
                             gp_Pnt(0.0, 0.0, 0.0), gp_Pnt(0.0, 0.0, 0.0), {},
                             linearAxisNames, matrix, zeroResidual)) {
        return false;
    }
    double rhs[3]{};
    double solution[3]{};
    return solveThreeByThree(matrix, rhs, solution);
}

} // namespace lcnc::kinematics
