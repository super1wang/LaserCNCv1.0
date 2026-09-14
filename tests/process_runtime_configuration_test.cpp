#include "core/algorithms/cam/initial_approach_axis_planner.h"
#include "modules/process/runtime/process_runtime_configuration.h"
#include "modules/process/runtime/rapid_motion_utilities.h"

#include <QCoreApplication>
#include <QTextStream>

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
    lcnc::process::ProcessRuntimeConfiguration configuration;

    configuration.setEnabledAxes({QStringLiteral("x"), QStringLiteral("A"), QStringLiteral("X"), QStringLiteral(" base ")});
    if (!configuration.isAxisEnabled(lcnc::process::Axis::X) || !configuration.isAxisEnabled(lcnc::process::Axis::A)
        || configuration.isAxisEnabled(lcnc::process::Axis::Y) || configuration.enabledAxes().size() != 2)
        return fail(QStringLiteral("Axis normalization or selection is invalid"));

    configuration.setExtensionAxes({QStringLiteral("x1"), QStringLiteral("U")});
    if (!configuration.isExtensionAxis(QStringLiteral("X1"))
        || !configuration.isExtensionAxis(QStringLiteral("u"))
        || configuration.isExtensionAxis(QStringLiteral("A1")))
        return fail(QStringLiteral("Extension-axis matching is invalid"));

    if (!configuration.simulationMode())
        return fail(QStringLiteral("Simulation default must be enabled"));
    configuration.setSimulationMode(false);
    configuration.setCustomerId("MaiTong");
    configuration.setPermission(lcnc::process::PermissionLevel::Factory);
    if (configuration.simulationMode() || configuration.customerId() != "MaiTong"
        || configuration.permission() != lcnc::process::PermissionLevel::Factory)
        return fail(QStringLiteral("Runtime configuration update is invalid"));

    lcnc::cam::RapidMoveSegment rapid;
    rapid.target.axes = {1.25, -2.5, 3.75, 91.0, -182.0};
    rapid.target.activeMask = 0x1f;
    rapid.target.rotaryAxis1Name = QStringLiteral("A");
    rapid.target.rotaryAxis2Name = QStringLiteral("C");
    rapid.target.tcpX = 100.0;
    rapid.target.tcpY = 200.0;
    rapid.target.tcpZ = 300.0;
    rapid.target.tcpMcsX = -10.0;
    rapid.target.tcpMcsY = -20.0;
    rapid.target.tcpMcsZ = -30.0;
    rapid.target.tcpMcsValid = true;
    const auto rapidPose = lcnc::process::solvedRapidPose(rapid);
    if (rapidPose.x != 1.25 || rapidPose.y != -2.5 || rapidPose.z != 3.75
        || rapidPose.r1 != 91.0 || rapidPose.r2 != -182.0
        || rapidPose.mask != 0x1f || rapidPose.r1Name != QStringLiteral("A")
        || rapidPose.r2Name != QStringLiteral("C")) {
        return fail(QStringLiteral("Process modified a CAM-certified rapid pose"));
    }
    if (!rapidPose.tcpMcsValid || rapidPose.tcpMcsX != -10.0
        || rapidPose.tcpMcsY != -20.0 || rapidPose.tcpMcsZ != -30.0)
        return fail(QStringLiteral("Process substituted scene-world TCP for RTCP reference"));
    rapid.target.tcpMcsValid = false;
    if (lcnc::process::solvedRapidPose(rapid).tcpMcsValid)
        return fail(QStringLiteral("Process manufactured a missing RTCP reference"));

    lcnc::cam::RapidTransition committedTransition;
    committedTransition.fromContourId = 0;
    committedTransition.toContourId = 10;
    committedTransition.segments.append(rapid);
    if (lcnc::process::canReuseCommittedTransition(0, committedTransition))
        return fail(QStringLiteral("A fresh run reused an old initial transition"));
    committedTransition.fromContourId = 7;
    if (!lcnc::process::canReuseCommittedTransition(7, committedTransition)
        || lcnc::process::canReuseCommittedTransition(8, committedTransition)) {
        return fail(QStringLiteral("Inter-contour transition matching is invalid"));
    }

    lcnc::MachineAxisLayout layout;
    layout.count = 5;
    layout.axes[0] = {QStringLiteral("Y"), lcnc::MachineAxisRole::LinearY};
    layout.axes[1] = {QStringLiteral("X"), lcnc::MachineAxisRole::LinearX};
    layout.axes[2] = {QStringLiteral("Z"), lcnc::MachineAxisRole::LinearZ};
    layout.axes[3] = {QStringLiteral("A"), lcnc::MachineAxisRole::TableTilt};
    layout.axes[4] = {QStringLiteral("C"), lcnc::MachineAxisRole::TableSpin};
    lcnc::SolvedMachinePose source;
    source.values = {-5.0, 10.0, -10.0, 20.0, 30.0};
    source.activeMask = 0x1f;
    source.valid = true;
    lcnc::SolvedMachinePose target;
    target.values = {25.0, 40.0, -20.0, 50.0, 60.0};
    target.activeMask = 0x1f;
    target.valid = true;
    const auto manualPlan = lcnc::cam_algo::planInitialApproachAxes(
        layout, source, target, 50.0, 1.0,
        lcnc::cam_algo::InitialApproachAxisMode::Manual);
    if (!manualPlan.isValid() || manualPlan.waypoints.size() != 3
        || manualPlan.waypoints.at(0).phase != lcnc::cam::RapidSegmentPhase::Retract
        || manualPlan.waypoints.at(0).pose.values[2] != 50.0
        || manualPlan.waypoints.at(1).phase != lcnc::cam::RapidSegmentPhase::Traverse
        || manualPlan.waypoints.at(1).pose.values[0] != target.values[0]
        || manualPlan.waypoints.at(1).pose.values[1] != target.values[1]
        || manualPlan.waypoints.at(1).pose.values[2] != 50.0
        || manualPlan.waypoints.at(2).phase != lcnc::cam::RapidSegmentPhase::Approach
        || manualPlan.waypoints.at(2).pose.values != target.values) {
        return fail(QStringLiteral("Manual initial approach is not strict Z/traverse/Z"));
    }

    lcnc::SolvedMachinePose highSource = source;
    highSource.values[2] = 80.0;
    const auto automaticPlan = lcnc::cam_algo::planInitialApproachAxes(
        layout, highSource, target, 50.0, 1.0,
        lcnc::cam_algo::InitialApproachAxisMode::AutomaticSafeZone);
    if (!automaticPlan.isValid() || automaticPlan.waypoints.size() != 3
        || automaticPlan.waypoints.at(0).phase != lcnc::cam::RapidSegmentPhase::SafeXY
        || automaticPlan.waypoints.at(0).pose.values[0] != target.values[0]
        || automaticPlan.waypoints.at(0).pose.values[1] != target.values[1]
        || automaticPlan.waypoints.at(0).pose.values[2] != 80.0
        || automaticPlan.waypoints.at(0).pose.values[3] != highSource.values[3]
        || automaticPlan.waypoints.at(0).pose.values[4] != highSource.values[4]
        || automaticPlan.waypoints.at(1).phase != lcnc::cam::RapidSegmentPhase::SafeAC
        || automaticPlan.waypoints.at(1).pose.values[3] != target.values[3]
        || automaticPlan.waypoints.at(1).pose.values[4] != target.values[4]
        || automaticPlan.waypoints.at(1).pose.values[2] != 80.0
        || automaticPlan.waypoints.at(2).phase != lcnc::cam::RapidSegmentPhase::Approach) {
        return fail(QStringLiteral("Automatic initial approach is not SafeXY/SafeAC/Z"));
    }
    const auto automaticRetractPlan = lcnc::cam_algo::planInitialApproachAxes(
        layout, source, target, 50.0, 1.0,
        lcnc::cam_algo::InitialApproachAxisMode::AutomaticSafeZone);
    if (!automaticRetractPlan.isValid() || automaticRetractPlan.waypoints.size() != 4
        || automaticRetractPlan.waypoints.at(0).phase
            != lcnc::cam::RapidSegmentPhase::Retract
        || automaticRetractPlan.waypoints.at(0).pose.values[2] != 50.0
        || automaticRetractPlan.waypoints.at(1).phase
            != lcnc::cam::RapidSegmentPhase::SafeXY
        || automaticRetractPlan.waypoints.at(2).phase
            != lcnc::cam::RapidSegmentPhase::SafeAC
        || automaticRetractPlan.waypoints.at(3).phase
            != lcnc::cam::RapidSegmentPhase::Approach) {
        return fail(QStringLiteral("Automatic initial approach did not retract before SafeXY/SafeAC"));
    }
    lcnc::SolvedMachinePose signedTarget = target;
    signedTarget.values[2] = -40.0;
    const auto signedSafetyPlan = lcnc::cam_algo::planInitialApproachAxes(
        layout, source, signedTarget, -30.0, 1.0,
        lcnc::cam_algo::InitialApproachAxisMode::Manual);
    if (!signedSafetyPlan.isValid()
        || signedSafetyPlan.resolvedSafetyAxisZ != -30.0
        || signedSafetyPlan.waypoints.at(0).pose.values[2] != -30.0) {
        return fail(QStringLiteral("Signed controller safety coordinate was modified"));
    }

    lcnc::SolvedMachinePose reversedSource = source;
    reversedSource.values[2] = 20.0;
    lcnc::SolvedMachinePose reversedTarget = target;
    reversedTarget.values[2] = 40.0;
    const auto reversedManual = lcnc::cam_algo::planInitialApproachAxes(
        layout, reversedSource, reversedTarget, -50.0, -1.0,
        lcnc::cam_algo::InitialApproachAxisMode::Manual);
    if (!reversedManual.isValid() || reversedManual.waypoints.size() != 3
        || reversedManual.resolvedSafetyAxisZ != -50.0
        || reversedManual.waypoints.at(0).phase != lcnc::cam::RapidSegmentPhase::Retract
        || reversedManual.waypoints.at(0).pose.values[2] != -50.0
        || reversedManual.waypoints.at(1).pose.values[2] != -50.0
        || reversedManual.waypoints.at(2).pose.values[2] != 40.0) {
        return fail(QStringLiteral("Reversed Z did not retract toward negative machine coordinates"));
    }
    lcnc::SolvedMachinePose reversedHighSource = reversedSource;
    reversedHighSource.values[2] = -80.0;
    const auto reversedAutomatic = lcnc::cam_algo::planInitialApproachAxes(
        layout, reversedHighSource, reversedTarget, -50.0, -1.0,
        lcnc::cam_algo::InitialApproachAxisMode::AutomaticSafeZone);
    if (!reversedAutomatic.isValid() || reversedAutomatic.waypoints.size() != 3
        || reversedAutomatic.waypoints.at(0).phase != lcnc::cam::RapidSegmentPhase::SafeXY
        || reversedAutomatic.waypoints.at(0).pose.values[2] != -80.0
        || reversedAutomatic.waypoints.at(1).phase != lcnc::cam::RapidSegmentPhase::SafeAC
        || reversedAutomatic.waypoints.at(1).pose.values[2] != -80.0
        || reversedAutomatic.waypoints.at(2).pose.values[2] != 40.0) {
        return fail(QStringLiteral("Reversed Z automatic planning moved downward before traverse"));
    }

    lcnc::SolvedMachinePose positiveReverseSource = reversedSource;
    positiveReverseSource.values[2] = 100.0;
    lcnc::SolvedMachinePose positiveReverseTarget = reversedTarget;
    positiveReverseTarget.values[2] = 80.0;
    const auto positiveReverseSafety = lcnc::cam_algo::planInitialApproachAxes(
        layout, positiveReverseSource, positiveReverseTarget, 50.0, -1.0,
        lcnc::cam_algo::InitialApproachAxisMode::Manual);
    if (!positiveReverseSafety.isValid()
        || positiveReverseSafety.resolvedSafetyAxisZ != 50.0
        || positiveReverseSafety.waypoints.at(0).pose.values[2] != 50.0) {
        return fail(QStringLiteral("Reversed Z converted an absolute controller coordinate"));
    }

    const auto forwardCandidates =
        lcnc::cam_algo::planAutomaticSafetyZCandidates(
            -10.0, -20.0, 1.0, -100.0, 100.0, 5.0, 5.0);
    if (!forwardCandidates.isValid()
        || forwardCandidates.axisCoordinates.front() != -10.0
        || forwardCandidates.axisCoordinates.back() != 100.0) {
        return fail(QStringLiteral("Forward automatic Z safety-domain candidates are invalid"));
    }
    const auto reversedCandidates =
        lcnc::cam_algo::planAutomaticSafetyZCandidates(
            20.0, 40.0, -1.0, -100.0, 100.0, 5.0, 5.0);
    if (!reversedCandidates.isValid()
        || reversedCandidates.axisCoordinates.front() != 20.0
        || reversedCandidates.axisCoordinates.back() != -100.0) {
        return fail(QStringLiteral("Reversed automatic Z safety-domain candidates are invalid"));
    }
    const auto limitFailure =
        lcnc::cam_algo::planAutomaticSafetyZCandidates(
            90.0, 98.0, 1.0, -100.0, 100.0, 5.0, 5.0);
    if (limitFailure.isValid())
        return fail(QStringLiteral("Automatic Z search ignored the physical upper limit"));
    if (lcnc::cam_algo::shouldRetryAutomaticSafetyZCandidate(
            lcnc::cam::CollisionValidationState::Collision, true,
            lcnc::cam::RapidSegmentPhase::Retract)
        || lcnc::cam_algo::shouldRetryAutomaticSafetyZCandidate(
            lcnc::cam::CollisionValidationState::Indeterminate, false,
            lcnc::cam::RapidSegmentPhase::Traverse)
        || !lcnc::cam_algo::shouldRetryAutomaticSafetyZCandidate(
            lcnc::cam::CollisionValidationState::Collision, true,
            lcnc::cam::RapidSegmentPhase::SafeXY)) {
        return fail(QStringLiteral("Automatic Z retry did not reject a shared blocked retract prefix"));
    }

    return 0;
}
