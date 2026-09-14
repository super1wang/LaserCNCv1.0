#pragma once

#include <QList>
#include <QString>

#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

struct MachineAxisDef;
class MachineKinematics;

namespace lcnc::kinematics {

struct AxisCarrierCorrection
{
    QString axisName;
    gp_Vec translation;
};

struct TableCalibrationPlan
{
    QString tiltAxisName;
    QString spinAxisName;
    QString toolCarrierAxisName;
    gp_Pnt modelRotationCenter;
    gp_Vec wholeMachineTranslation;
    gp_Vec cutterTranslation;
    QList<AxisCarrierCorrection> carrierCorrections;
    double rotaryAxisSeparation{0.0};
    double cutterResidual{0.0};
};

enum class CalibrationPlanError
{
    None,
    MissingSemanticAxis,
    InvalidToolCarrierChain,
    ParallelRotaryAxes,
    SkewRotaryReferences,
    DegenerateToolCarrierMotion,
    UnreachableToolCorrection
};

/// Finds the point on the configured parent tilt-axis line nearest to the
/// child spin-axis line. For an A axis parallel to X this preserves A.Y/Z and
/// obtains the missing X from the child reference; for B parallel to Y it
/// preserves B.X/Z and obtains Y from the child reference.
bool tableRotationCenterFromReferences(const MachineAxisDef& tiltAxis,
                                       const gp_Pnt& tiltReferenceCenter,
                                       const MachineAxisDef& spinAxis,
                                       const gp_Pnt& spinReferenceCenter,
                                       gp_Pnt& center,
                                       double* axisSeparation = nullptr);

/// Builds a topology-driven two-stage table calibration plan. The complete
/// machine is translated first; the cutter correction is then decomposed only
/// through the actual linear axes on the configured tool-carrier chain.
bool buildTableCalibrationPlan(const MachineKinematics& kinematics,
                               const gp_Pnt& tiltReferenceCenter,
                               const gp_Pnt& spinReferenceCenter,
                               const gp_Pnt& cutterReferenceCenter,
                               const gp_Pnt& physicalRotationCenter,
                               const gp_Pnt& physicalTcp,
                               TableCalibrationPlan& plan,
                               CalibrationPlanError* error = nullptr);

gp_Vec inheritedCarrierCorrection(const MachineKinematics& kinematics,
                                  const QString& assignedAxis,
                                  const QList<AxisCarrierCorrection>& corrections);

} // namespace lcnc::kinematics
