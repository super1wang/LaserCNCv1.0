#include "core/algorithms/cam/collision_scan_policy.h"

#include <QCoreApplication>
#include <QTextStream>

#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

#include <cmath>

namespace {

int fail(const QString& message)
{
    QTextStream(stderr) << message << '\n';
    return 1;
}

} // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    lcnc::cam::CollisionValidationSnapshot validation;
    validation.state = lcnc::cam::CollisionValidationState::Disabled;
    validation.complete = true;
    if (validation.blocksExecution())
        return fail(QStringLiteral("Disabled collision detection must not block execution"));

    validation.state = lcnc::cam::CollisionValidationState::Warning;
    if (!validation.blocksExecution(true) || validation.blocksExecution(false))
        return fail(QStringLiteral("Clearance warning policy is not configurable"));

    validation.state = lcnc::cam::CollisionValidationState::Collision;
    if (!validation.blocksExecution(false))
        return fail(QStringLiteral("Confirmed collision must always block execution"));

    lcnc::cam::CamMotionPlanSnapshot plan;
    plan.revision = 42;
    lcnc::cam::CamMotionNode rapid;
    rapid.phase = lcnc::cam::CamMotionPhase::Rapid;
    rapid.rapidPhase = lcnc::cam::RapidSegmentPhase::Retract;
    rapid.contourId = 7;
    rapid.axisMask = 0x1f;
    plan.nodes.append(rapid);
    plan.collision = validation;
    if (plan.nodes.size() != 1 || plan.nodes.constFirst().contourId != 7
        || plan.nodes.constFirst().phase != lcnc::cam::CamMotionPhase::Rapid)
        return fail(QStringLiteral("Motion node contract did not preserve final CAM data"));
    if (plan.nodes.constFirst().rapidPhase != lcnc::cam::RapidSegmentPhase::Retract)
        return fail(QStringLiteral("Motion node contract lost the CAM rapid segment phase"));

    lcnc::cam::CamMotionNode cutting;
    cutting.phase = lcnc::cam::CamMotionPhase::Cutting;
    cutting.tcpX = 10.0;
    cutting.tcpY = 0.0;
    cutting.tcpZ = 0.0;
    cutting.normalX = 1.0;
    cutting.normalY = 0.0;
    cutting.normalZ = 0.0;
    gp_Trsf rotation;
    rotation.SetRotation(gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)),
                         1.5707963267948966);
    gp_Trsf installation;
    installation.SetTranslation(gp_Vec(0.0, 0.0, 150.0));
    const gp_Trsf workpieceTransform = installation.Multiplied(rotation);
    lcnc::cam_algo::transformMotionNodeGeometry(&cutting, workpieceTransform);
    if (std::abs(cutting.tcpX) > 1e-9
        || std::abs(cutting.tcpY - 10.0) > 1e-9
        || std::abs(cutting.tcpZ - 150.0) > 1e-9
        || std::abs(cutting.normalX) > 1e-9
        || std::abs(cutting.normalY - 1.0) > 1e-9
        || std::abs(cutting.normalZ) > 1e-9) {
        return fail(QStringLiteral(
            "Workpiece installation was not applied to collision motion geometry"));
    }

    MachineAxisDef xAxis;
    xAxis.name = QStringLiteral("X");
    xAxis.currentPos = 417.0; // Simulated live controller feedback.
    MachineAxisDef aAxis;
    aAxis.name = QStringLiteral("A");
    aAxis.motionType = MachineAxisDef::Rotary;
    aAxis.currentPos = -37.0; // Must not leak into an offline CAM snapshot.
    lcnc::MachineModeDefinition definition;
    definition.interpolatedAxes.append(
        QStringLiteral("X"), lcnc::MachineAxisRole::LinearX);
    definition.lockedAxisTargets.insert(QStringLiteral("A"), 90.0);
    const QList<MachineAxisDef> baseline =
        lcnc::cam_algo::offlinePlanningAxisBaseline({xAxis, aAxis}, definition);
    if (baseline.size() != 2
        || std::abs(baseline.at(0).currentPos) > 1e-9
        || std::abs(baseline.at(1).currentPos - 90.0) > 1e-9) {
        return fail(QStringLiteral(
            "Offline CAM planning retained live controller axis positions"));
    }
    MachineKinematics offlineKinematics;
    offlineKinematics.setAxes(baseline);
    std::array<double, lcnc::MachineAxisLayout::kMaxAxes> plannedValues{};
    plannedValues[0] = 12.5;
    lcnc::cam_algo::applyOfflineMotionPose(
        &offlineKinematics, baseline, definition.interpolatedAxes,
        plannedValues, 0x01u);
    const MachineAxisDef* offlineX = offlineKinematics.findAxis(QStringLiteral("X"));
    const MachineAxisDef* offlineA = offlineKinematics.findAxis(QStringLiteral("A"));
    if (!offlineX || !offlineA
        || std::abs(offlineX->currentPos - 12.5) > 1e-9
        || std::abs(offlineA->currentPos - 90.0) > 1e-9) {
        return fail(QStringLiteral(
            "Offline motion pose did not use solved and locked axis values"));
    }
    return 0;
}
