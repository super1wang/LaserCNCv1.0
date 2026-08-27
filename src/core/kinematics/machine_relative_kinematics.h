#pragma once

#include <QMap>
#include <QString>
#include <QStringList>

#include <gp_Pnt.hxx>

class MachineKinematics;

namespace lcnc::kinematics {

enum class RelativeLinearSolveError
{
    None,
    InvalidCarrierChain,
    MissingLinearAxis,
    SingularRelativeMotion,
    AxisLimitExceeded,
    ForwardResidualExceeded
};

struct RelativeLinearSolveResult
{
    QMap<QString, double> axisPositions;
    double forwardResidual{0.0};
};

/// Solves the three configured linear-axis coordinates from the relative
/// motion between the tool carrier and workpiece carrier. This supports linear
/// axes on either branch, including split layouts such as:
///   tool: BASE->Y->Z, workpiece: BASE->X->B->C.
bool solveRelativeLinearAxes(const MachineKinematics& kinematics,
                             const QString& toolCarrierAxis,
                             const QString& workpieceCarrierAxis,
                             const gp_Pnt& toolLocalPoint,
                             const gp_Pnt& workpieceLocalPoint,
                             const QMap<QString, double>& fixedAxisPositions,
                             const QStringList& linearAxisNames,
                             RelativeLinearSolveResult& result,
                             RelativeLinearSolveError* error = nullptr);

/// Configuration-time rank check at the zero rotary posture.
bool hasFullRankRelativeLinearMotion(const MachineKinematics& kinematics,
                                     const QString& toolCarrierAxis,
                                     const QString& workpieceCarrierAxis,
                                     const QStringList& linearAxisNames);

} // namespace lcnc::kinematics
