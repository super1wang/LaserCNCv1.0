#include "core/algorithms/cam/laser_toolpath.h"
#include "core/kinematics/machine_configuration_service.h"
#include "core/kinematics/machine_kinematics.h"
#include "core/kinematics/machine_pose.h"
#include "core/kinematics/toolpath_kinematics_solver.h"
#include "modules/process/process_module.h"
#include "modules/process/runtime/axis_map.h"
#include "modules/process/runtime/machine_pose5.h"

#include <QCoreApplication>
#include <QDebug>
#include <QFile>
#include <QTemporaryDir>
#include <QTextStream>

#include <algorithm>
#include <cmath>

namespace {

bool isNear(double lhs, double rhs, double tolerance = 1e-6)
{
    return std::abs(lhs - rhs) <= tolerance;
}

bool require(bool condition, const char* message)
{
    if (!condition) QTextStream(stderr) << message << '\n';
    return condition;
}

} // namespace

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);
    bool ok = true;

    lcnc::MachineConfigurationService configuration;
    configuration.applyPreset(QStringLiteral("XYZ"));
    ok &= require(configuration.supportedMachiningModes()
                      == QList<lcnc::MachiningMode>{lcnc::MachiningMode::Planar3Axis},
                  "XYZ capability matrix is incorrect");

    configuration.applyPreset(QStringLiteral("XYZA"));
    ok &= require(configuration.defaultMachiningMode() == lcnc::MachiningMode::RotaryTube4Axis,
                  "XYZA default mode is not RotaryTube4Axis");
    ok &= require(configuration.supportsMachiningMode(lcnc::MachiningMode::Planar3Axis)
                  && configuration.supportsMachiningMode(lcnc::MachiningMode::RotaryTube4Axis),
                  "XYZA reduced capability matrix is incorrect");
    configuration.applyPreset(QStringLiteral("VERTICAL_AC_TABLE"));
    ok &= require(configuration.defaultMachiningMode() == lcnc::MachiningMode::SimultaneousTable5Axis,
                  "A five-axis table machine did not default to simultaneous five-axis mode");

    // Old XYZA configurations could retain the +/-120 degree A tilt limits
    // from the AC-table preset.  Loading one must upgrade A to the continuous
    // workpiece-rotary planning envelope.
    QTemporaryDir temporaryConfigDir;
    const QString legacyXyzaConfigPath = temporaryConfigDir.filePath(QStringLiteral("machine.toml"));
    QFile legacyXyzaConfig(legacyXyzaConfigPath);
    ok &= require(legacyXyzaConfig.open(QIODevice::WriteOnly | QIODevice::Text),
                  "Could not create the legacy XYZA configuration fixture");
    if (legacyXyzaConfig.isOpen()) {
        legacyXyzaConfig.write(R"(machinePreset = "XYZA"
[[modeDefinitions]]
mode = "RotaryTube4Axis"

[[modeDefinitions.lockedAxes]]
name = "A"
target = 90.0

[[axes]]
name = "A"
motionType = "Rotary"
role = "WorkpieceRotary"
parent = "BASE"
min = -120.0
max = 120.0
)");
        legacyXyzaConfig.close();
        lcnc::MachineConfigurationService migratedXyza;
        ok &= require(migratedXyza.load(legacyXyzaConfigPath),
                      "Could not load the legacy XYZA configuration fixture");
        const auto migratedAxes = migratedXyza.axisConfigurations();
        ok &= require(migratedAxes.size() == 1
                      && isNear(migratedAxes.front().axis.minVal, -9999.0)
                      && isNear(migratedAxes.front().axis.maxVal, 9999.0),
                      "XYZA workpiece rotary limits were not migrated to continuous rotation");
        ok &= require(migratedXyza.modeDefinition(lcnc::MachiningMode::RotaryTube4Axis)
                          .lockedAxisTargets.isEmpty(),
                      "XYZA rotary tube mode accepted a stale table-axis lock override");
    }

    configuration.applyPreset(QStringLiteral("VERTICAL_AC_TABLE"));
    const auto reduced = configuration.modeDefinition(lcnc::MachiningMode::RotaryTube4Axis);
    ok &= require(reduced.interpolatedAxes.count == 4
                  && reduced.interpolatedAxes.axes[3].name == QStringLiteral("C")
                  && reduced.solverVersion == lcnc::machiningModeSolverVersion(
                      lcnc::MachiningMode::RotaryTube4Axis)
                  && isNear(reduced.lockedAxisTargets.value(QStringLiteral("A")), 90.0),
                  "AC-table four-axis definition is incorrect");
    const auto reducedMap = lcnc::process::AxisMap::from(&configuration, reduced.interpolatedAxes);
    ok &= require(reducedMap.activeCount() == 4
                  && reducedMap.axisName(lcnc::process::AxisMap::R1) == QStringLiteral("C")
                  && reducedMap.axisTupleText(lcnc::process::MachinePose5::Bx
                         | lcnc::process::MachinePose5::By | lcnc::process::MachinePose5::Bz
                         | lcnc::process::MachinePose5::Br1) == QStringLiteral("(0, 1, 2, 4)"),
                  "AC-table reduced four-axis controller tuple is incorrect");
    const auto fiveAxis = configuration.modeDefinition(lcnc::MachiningMode::SimultaneousTable5Axis);
    const auto fiveMap = lcnc::process::AxisMap::from(&configuration, fiveAxis.interpolatedAxes);
    ok &= require(fiveMap.activeCount() == 5 && fiveMap.isFiveAxis(),
                  "AC-table five-axis controller map is incomplete");
    ok &= require(fiveMap.axis(lcnc::process::AxisMap::R1).jerk > 0.0
                  && fiveMap.axis(lcnc::process::AxisMap::R2).jerk > 0.0,
                  "AC-table rotary axes must have a positive ACS jerk value");

    // The preset service reconfigures one MachineKinematics instance in place.
    // Pose must refresh its axis set in that case so C feedback drives the
    // mounted workpiece for both reduced four-axis and five-axis execution.
    MachineKinematics liveKinematics;
    configuration.applyPreset(QStringLiteral("XYZA"));
    liveKinematics.setAxes(configuration.axisDefinitions(), configuration.presetName());
    lcnc::MachinePose livePose;
    livePose.setKinematics(&liveKinematics);
    configuration.applyPreset(QStringLiteral("VERTICAL_AC_TABLE"));
    liveKinematics.setAxes(configuration.axisDefinitions(), configuration.presetName());
    livePose.setKinematics(&liveKinematics);
    ok &= require(livePose.supportedAxes().contains(QStringLiteral("C"))
                  && livePose.setAxisValue(QStringLiteral("C"), 30.0, false)
                  && isNear(livePose.axisValue(QStringLiteral("C")), 30.0),
                  "In-place AC-table configuration did not refresh the pose C axis");

    // Reconfiguring an axis definition while opening a new workpiece must not
    // publish a synthetic zero feedback and visually return the mounted
    // workpiece to home until the next controller polling cycle.
    ProcessModule process;
    QList<MachineAxisDef> processAxes = configuration.axisDefinitions();
    process.setAxisDefinitions(processAxes);
    process.setAxisPositions({{QStringLiteral("X"), 123.0},
                              {QStringLiteral("A"), 17.0},
                              {QStringLiteral("C"), -42.0}});
    processAxes.front().maxVal += 1.0; // force a non-topological definition refresh
    process.setAxisDefinitions(processAxes);
    const QMap<QString, double> refreshedPositions = process.currentAxisPositions();
    ok &= require(isNear(refreshedPositions.value(QStringLiteral("X")), 123.0)
                  && isNear(refreshedPositions.value(QStringLiteral("A")), 17.0)
                  && isNear(refreshedPositions.value(QStringLiteral("C")), -42.0),
                  "Axis definition refresh reset live controller feedback to zero");
    QMap<QString, double> replayedPositions;
    QObject::connect(&process, &ProcessModule::axisPositionChanged,
                     [&replayedPositions](const QString& axis, double value) {
                         replayedPositions.insert(axis, value);
                     });
    process.synchronizeAxisFeedback();
    ok &= require(isNear(replayedPositions.value(QStringLiteral("X")), 123.0)
                  && isNear(replayedPositions.value(QStringLiteral("A")), 17.0)
                  && isNear(replayedPositions.value(QStringLiteral("C")), -42.0),
                  "Rebuilt coordinate view did not receive cached controller feedback");

    // Rapid preview points arrive from planning in machine/world coordinates.
    // The display stores them in workpiece-local coordinates and applies the
    // current WPC transform once through AIS.  This round trip is what keeps
    // a rapid curve on a workpiece held at a non-zero table posture.
    MachineKinematics displayedMachine;
    displayedMachine.setAxes(configuration.axisDefinitions(), configuration.presetName());
    displayedMachine.mountWorkpiece(QStringLiteral("fixture"), QStringLiteral("C"));
    displayedMachine.setAxisPosition(QStringLiteral("A"), 35.0);
    displayedMachine.setAxisPosition(QStringLiteral("C"), -42.0);
    gp_Trsf fixtureSetup;
    fixtureSetup.SetTranslation(gp_Vec(12.0, -8.0, 25.0));
    displayedMachine.setWorkpieceSetupTransform(fixtureSetup);
    const gp_Trsf liveFixtureTransform =
        displayedMachine.computeWpcTransform(QStringLiteral("fixture"));
    const gp_Pnt plannedWorldPoint = gp_Pnt(4.0, 7.0, 9.0).Transformed(liveFixtureTransform);
    const gp_Pnt displayLocalPoint = plannedWorldPoint.Transformed(liveFixtureTransform.Inverted());
    displayedMachine.setAxisPosition(QStringLiteral("C"), 28.0);
    const gp_Trsf movedFixtureTransform =
        displayedMachine.computeWpcTransform(QStringLiteral("fixture"));
    const gp_Pnt redisplayedWorldPoint = displayLocalPoint.Transformed(movedFixtureTransform);
    const gp_Pnt expectedMovedWorldPoint = gp_Pnt(4.0, 7.0, 9.0).Transformed(movedFixtureTransform);
    ok &= require(redisplayedWorldPoint.Distance(expectedMovedWorldPoint) < 1e-6
                  && redisplayedWorldPoint.Distance(plannedWorldPoint) > 1e-3,
                  "Rapid preview did not follow the non-zero workpiece posture");

    configuration.applyPreset(QStringLiteral("VERTICAL_BC_TABLE"));
    const auto bcReduced = configuration.modeDefinition(lcnc::MachiningMode::RotaryTube4Axis);
    ok &= require(configuration.supportsMachiningMode(lcnc::MachiningMode::SimultaneousTable5Axis)
                  && bcReduced.interpolatedAxes.axes[3].name == QStringLiteral("C")
                  && isNear(bcReduced.lockedAxisTargets.value(QStringLiteral("B")), 90.0),
                  "BC-table capability or reduced mode is incorrect");

    lcnc::HeadToolGeometry calibratedHead;
    calibratedHead.focusLength = 100.0;
    for (const QString& preset : {QStringLiteral("AB_HEAD"), QStringLiteral("AC_HEAD")}) {
        configuration.applyPreset(preset);
        configuration.setHeadToolGeometry({});
        ok &= require(configuration.supportedMachiningModes()
                          == QList<lcnc::MachiningMode>{lcnc::MachiningMode::Planar3Axis},
                      "Uncalibrated head unexpectedly enabled five-axis machining");
        configuration.setHeadToolGeometry(calibratedHead);
        ok &= require(configuration.supportsMachiningMode(
                          lcnc::MachiningMode::SimultaneousHead5Axis),
                      "Calibrated head did not enable software-TCP mode");
    }

    lcnc::MachineConfigurationService invalidConfiguration;
    invalidConfiguration.applyPreset(QStringLiteral("XYZ"));
    auto invalidAxes = invalidConfiguration.axisConfigurations();
    invalidAxes[1].controllerIndex = invalidAxes[0].controllerIndex;
    invalidConfiguration.setAxisConfigurations(invalidAxes);
    QString validationError;
    ok &= require(!invalidConfiguration.validateConfiguration(&validationError)
                  && !validationError.isEmpty(),
                  "Duplicate controller indices were accepted");
    invalidConfiguration.applyPreset(QStringLiteral("XYZ"));
    invalidAxes = invalidConfiguration.axisConfigurations();
    invalidAxes[0].axis.role = lcnc::MachineAxisRole::LinearX;
    invalidAxes[1].axis.role = lcnc::MachineAxisRole::LinearX;
    invalidConfiguration.setAxisConfigurations(invalidAxes);
    ok &= require(!invalidConfiguration.validateConfiguration(&validationError),
                  "Duplicate axis roles were accepted");

    // A rotary role alone is not a usable four-axis topology.  Capability
    // discovery must require the complete XYZ Cartesian basis as well.
    invalidConfiguration.applyPreset(QStringLiteral("XYZA"));
    invalidAxes = invalidConfiguration.axisConfigurations();
    invalidAxes.erase(std::remove_if(invalidAxes.begin(), invalidAxes.end(),
        [](const lcnc::MachineAxisRuntimeConfig& axis) {
            return axis.axis.role == lcnc::MachineAxisRole::LinearZ;
        }), invalidAxes.end());
    invalidConfiguration.setAxisConfigurations(invalidAxes);
    ok &= require(!invalidConfiguration.supportsMachiningMode(
                      lcnc::MachiningMode::RotaryTube4Axis)
                  && invalidConfiguration.supportedMachiningModes().isEmpty(),
                  "Incomplete XYZ+rotary topology was exposed as four-axis capable");

    lcnc::MachineModeDefinition invalidMode;
    invalidMode.mode = lcnc::MachiningMode::RotaryTube4Axis;
    invalidMode.solverId = QStringLiteral("RotaryTube4Axis");
    invalidMode.solverVersion = lcnc::machiningModeSolverVersion(invalidMode.mode);
    invalidMode.interpolatedAxes.append(QStringLiteral("X"), lcnc::MachineAxisRole::LinearX);
    invalidMode.interpolatedAxes.append(QStringLiteral("Y"), lcnc::MachineAxisRole::LinearY);
    invalidMode.interpolatedAxes.append(QStringLiteral("Z"), lcnc::MachineAxisRole::LinearZ);
    ok &= require(!invalidMode.isValid(&validationError),
                  "Four-axis mode accepted a three-axis layout");

    MachineKinematics xyz;
    xyz.loadPreset(QStringLiteral("XYZ"));
    ToolpathPoint planarPoint;
    planarPoint.position = gp_Pnt(10, 20, 30);
    planarPoint.normal = gp_Dir(0, 0, 1);
    std::vector<ToolpathPoint> planarPoints{planarPoint};
    lcnc::ToolpathKinematicsRequest planarRequest;
    planarRequest.machine = &xyz;
    planarRequest.mode = lcnc::MachiningMode::Planar3Axis;
    planarRequest.definition.mode = planarRequest.mode;
    planarRequest.definition.solverId = QStringLiteral("Planar3Axis");
    planarRequest.definition.solverVersion =
        lcnc::machiningModeSolverVersion(planarRequest.mode);
    planarRequest.definition.interpolatedAxes.append(QStringLiteral("X"), lcnc::MachineAxisRole::LinearX);
    planarRequest.definition.interpolatedAxes.append(QStringLiteral("Y"), lcnc::MachineAxisRole::LinearY);
    planarRequest.definition.interpolatedAxes.append(QStringLiteral("Z"), lcnc::MachineAxisRole::LinearZ);
    planarRequest.workpieceSetup.x = 5.0;
    planarRequest.points = &planarPoints;
    lcnc::ToolpathSolverRegistry registry;
    const auto planarPoses = registry.solve(planarRequest);
    ok &= require(planarPoses.size() == 1 && planarPoses.front().valid
                  && isNear(planarPoses.front().value(0), 15.0)
                  && isNear(planarPoses.front().value(1), 20.0)
                  && isNear(planarPoses.front().value(2), 30.0),
                  "Planar solver projection/setup transform failed");
    // Planar mode always cuts with the fixed machine-Z beam.  Source face
    // normals must not make a three-axis contour unreachable.
    planarPoints.front().normal = gp_Dir(0, 1, 0);
    const auto fixedBeamPlanar = registry.solve(planarRequest);
    ok &= require(fixedBeamPlanar.size() == 1 && fixedBeamPlanar.front().valid
                  && isNear(fixedBeamPlanar.front().value(0), 15.0)
                  && isNear(fixedBeamPlanar.front().value(1), 20.0)
                  && isNear(fixedBeamPlanar.front().value(2), 30.0),
                  "Planar solver rejected a contour with a non-Z source normal");
    if (MachineAxisDef* zAxis = xyz.findAxis(QStringLiteral("Z")))
        zAxis->direction = gp_Dir(0, 0, -1);
    planarRequest.workpieceSetup = {};
    planarPoints.front().position = gp_Pnt(0, 0, -30);
    const auto reversedZ = registry.solve(planarRequest);
    ok &= require(reversedZ.size() == 1 && reversedZ.front().valid
                  && isNear(reversedZ.front().value(2), 30.0),
                  "Planar solver ignored the configured reverse Z direction");

    MachineKinematics xyza;
    xyza.loadPreset(QStringLiteral("XYZA"));
    ToolpathPoint rotaryPoint;
    rotaryPoint.position = gp_Pnt(0, 10, 0);
    rotaryPoint.normal = gp_Dir(0, 1, 0);
    std::vector<ToolpathPoint> rotaryPoints{rotaryPoint};
    lcnc::ToolpathKinematicsRequest rotaryRequest;
    rotaryRequest.machine = &xyza;
    rotaryRequest.mode = lcnc::MachiningMode::RotaryTube4Axis;
    rotaryRequest.definition.mode = rotaryRequest.mode;
    rotaryRequest.definition.solverId = QStringLiteral("RotaryTube4Axis");
    rotaryRequest.definition.solverVersion =
        lcnc::machiningModeSolverVersion(rotaryRequest.mode);
    rotaryRequest.definition.interpolatedAxes.append(QStringLiteral("X"), lcnc::MachineAxisRole::LinearX);
    rotaryRequest.definition.interpolatedAxes.append(QStringLiteral("Y"), lcnc::MachineAxisRole::LinearY);
    rotaryRequest.definition.interpolatedAxes.append(QStringLiteral("Z"), lcnc::MachineAxisRole::LinearZ);
    rotaryRequest.definition.interpolatedAxes.append(QStringLiteral("A"), lcnc::MachineAxisRole::WorkpieceRotary);
    rotaryRequest.points = &rotaryPoints;
    const auto rotaryPoses = registry.solve(rotaryRequest);
    ok &= require(rotaryPoses.size() == 1 && rotaryPoses.front().valid
                  && isNear(rotaryPoses.front().value(3), 90.0)
                  && isNear(rotaryPoses.front().value(2), 10.0),
                  "Rotary tube solver did not solve the configured A axis");

    // CAD discretisation may put a normal a few thousandths of a degree past
    // an exact physical limit.  It must be clamped to the safe configured
    // boundary rather than rejecting an otherwise reachable tube point.
    if (MachineAxisDef* rotaryAxis = xyza.findAxis(QStringLiteral("A"))) {
        rotaryAxis->minVal = -120.0;
        rotaryAxis->maxVal = 120.0;
    }
    const double boundaryAngleRad = 120.009 * 3.14159265358979323846 / 180.0;
    rotaryPoints.front().position = gp_Pnt(0, 0, 0);
    rotaryPoints.front().normal = gp_Dir(0, -std::sin(boundaryAngleRad),
                                         std::cos(boundaryAngleRad));
    const auto boundaryPose = registry.solve(rotaryRequest);
    ok &= require(boundaryPose.size() == 1 && boundaryPose.front().valid
                  && isNear(boundaryPose.front().value(3), -120.0),
                  "Rotary tube solver did not clamp numerical limit overshoot");

    rotaryPoints.front().normal = gp_Dir(1, 1, 0);
    const auto unreachable = registry.solve(rotaryRequest);
    ok &= require(unreachable.size() == 1 && !unreachable.front().valid
                  && !unreachable.front().failureReason.isEmpty(),
                  "Rotary tube solver accepted an unreachable normal");

    // Regression: tube sidewalls are solved by the AC table's C-child/A-parent
    // decomposition.  Passing the roles in output-layout order (A then C)
    // reverses that decomposition and rejects the first point.
    // 中文翻译：回归验证 AC 转台管材侧壁法线可由子 C、父 A 的顺序求解。
    MachineKinematics acTable;
    acTable.loadPreset(QStringLiteral("VERTICAL_AC_TABLE"));
    lcnc::MachineConfigurationService acTableConfiguration;
    acTableConfiguration.applyPreset(QStringLiteral("VERTICAL_AC_TABLE"));
    ToolpathPoint tableSidewallPoint;
    tableSidewallPoint.position = gp_Pnt(0, 10, 0);
    tableSidewallPoint.normal = gp_Dir(0, 1, 0);
    std::vector<ToolpathPoint> tableSidewallPoints{tableSidewallPoint};
    lcnc::ToolpathKinematicsRequest tableRequest;
    tableRequest.machine = &acTable;
    tableRequest.mode = lcnc::MachiningMode::SimultaneousTable5Axis;
    tableRequest.definition = acTableConfiguration.modeDefinition(tableRequest.mode);
    tableRequest.points = &tableSidewallPoints;
    const auto tableSidewallPose = registry.solve(tableRequest);
    ok &= require(tableSidewallPose.size() == 1 && tableSidewallPose.front().valid,
                  "AC-table five-axis solver rejected a reachable tube sidewall normal");

    // Regression: the controller layout is XYZ/A/C, but table IK internally
    // uses child C then parent A.  A previous pose at +90 degrees must be
    // restored in that physical order so opposite sidewall normals rotate C
    // instead of flipping the clamping A axis to -90 degrees.
    // 中文翻译：回归验证跨轮廓连续姿态按 C→A 物理链路恢复，保持 A=+90 并由 C 处理对侧。
    const int tableTiltIndex = tableRequest.definition.interpolatedAxes.indexOfRole(
        lcnc::MachineAxisRole::TableTilt);
    const int tableSpinIndex = tableRequest.definition.interpolatedAxes.indexOfRole(
        lcnc::MachineAxisRole::TableSpin);
    lcnc::SolvedMachinePose stableTablePose;
    for (int axisIndex = 0; axisIndex < tableRequest.definition.interpolatedAxes.count; ++axisIndex)
        stableTablePose.setValue(axisIndex, 0.0);
    stableTablePose.setValue(tableTiltIndex, 90.0);
    stableTablePose.valid = true;
    ToolpathPoint oppositeTableSidewallPoint = tableSidewallPoint;
    oppositeTableSidewallPoint.normal = gp_Dir(0, -1, 0);
    std::vector<ToolpathPoint> oppositeTableSidewallPoints{
        tableSidewallPoint, oppositeTableSidewallPoint};
    tableRequest.points = &oppositeTableSidewallPoints;
    tableRequest.previousPose = &stableTablePose;
    const auto stableTablePoses = registry.solve(tableRequest);
    ok &= require(stableTablePoses.size() == 2
                  && stableTablePoses[0].valid && stableTablePoses[1].valid
                  && isNear(stableTablePoses[0].value(tableTiltIndex), 90.0)
                  && isNear(stableTablePoses[1].value(tableTiltIndex), 90.0)
                  && std::abs(stableTablePoses[1].value(tableSpinIndex)
                              - stableTablePoses[0].value(tableSpinIndex)) >= 90.0,
                  "AC-table continuity did not keep the +90 degree clamping tilt on opposite sidewalls");

    // Regression: a lead-in belongs to its contour.  It must be solved before
    // the first cutting point in the same continuity chain; copying only the
    // first point's rotary axes onto an independently solved lead-in leaves
    // its XYZ coordinates in a conflicting table posture.
    // 中文翻译：回归验证下刀点与轮廓首点共同继承连续姿态，不能把独立求解结果的旋转轴覆盖为首点姿态。
    LaserContour leadInContour;
    leadInContour.points = {oppositeTableSidewallPoint, tableSidewallPoint};
    leadInContour.leadInSolution.valid = true;
    leadInContour.leadInSolution.point = oppositeTableSidewallPoint;
    leadInContour.leadInSolution.point.position = gp_Pnt(2.0, -12.0, 0.0);
    const lcnc::SolvedMachinePose contourInitialPose = stableTablePoses.front();
    std::vector<ToolpathPoint> expectedLeadPoints{leadInContour.leadInSolution.point};
    tableRequest.points = &expectedLeadPoints;
    tableRequest.previousPose = nullptr;
    const auto independentLeadPose = registry.solve(tableRequest);
    tableRequest.previousPose = &contourInitialPose;
    const auto expectedLeadPose = registry.solve(tableRequest);
    QString leadInSolveError;
    std::vector<LaserContour*> leadInContours{&leadInContour};
    ok &= require(independentLeadPose.size() == 1 && independentLeadPose.front().valid
                  && expectedLeadPose.size() == 1 && expectedLeadPose.front().valid
                  && (std::abs(independentLeadPose.front().value(0)
                               - expectedLeadPose.front().value(0)) > 1e-6
                      || std::abs(independentLeadPose.front().value(1)
                                  - expectedLeadPose.front().value(1)) > 1e-6
                      || std::abs(independentLeadPose.front().value(2)
                                  - expectedLeadPose.front().value(2)) > 1e-6)
                  && LaserToolpathBuilder::solveToolpathForOrder(
                      leadInContours, &acTable, gp_Trsf(), tableRequest.definition,
                      {}, {}, &leadInSolveError, &contourInitialPose)
                  && leadInContour.leadInSolution.point.machineCoord.valid
                  && isNear(leadInContour.leadInSolution.point.machineCoord.solvedPose.value(0),
                            expectedLeadPose.front().value(0))
                  && isNear(leadInContour.leadInSolution.point.machineCoord.solvedPose.value(1),
                            expectedLeadPose.front().value(1))
                  && isNear(leadInContour.leadInSolution.point.machineCoord.solvedPose.value(2),
                            expectedLeadPose.front().value(2))
                  && isNear(leadInContour.leadInSolution.point.machineCoord.solvedPose.value(tableTiltIndex),
                            expectedLeadPose.front().value(tableTiltIndex))
                  && isNear(leadInContour.leadInSolution.point.machineCoord.solvedPose.value(tableSpinIndex),
                            expectedLeadPose.front().value(tableSpinIndex)),
                  "Lead-in was not solved in the contour continuity branch");

    MachineKinematics head;
    head.loadPreset(QStringLiteral("AB_HEAD"));
    ToolpathPoint headPoint;
    headPoint.position = gp_Pnt(10, 20, 0);
    headPoint.normal = gp_Dir(0, 0, 1);
    std::vector<ToolpathPoint> headPoints{headPoint};
    lcnc::ToolpathKinematicsRequest headRequest;
    headRequest.machine = &head;
    headRequest.mode = lcnc::MachiningMode::SimultaneousHead5Axis;
    headRequest.definition.mode = headRequest.mode;
    headRequest.definition.solverId = QStringLiteral("SimultaneousHead5Axis");
    headRequest.definition.solverVersion =
        lcnc::machiningModeSolverVersion(headRequest.mode);
    headRequest.definition.interpolatedAxes.append(QStringLiteral("X"), lcnc::MachineAxisRole::LinearX);
    headRequest.definition.interpolatedAxes.append(QStringLiteral("Y"), lcnc::MachineAxisRole::LinearY);
    headRequest.definition.interpolatedAxes.append(QStringLiteral("Z"), lcnc::MachineAxisRole::LinearZ);
    headRequest.definition.interpolatedAxes.append(QStringLiteral("B"), lcnc::MachineAxisRole::HeadTiltSecondary);
    headRequest.definition.interpolatedAxes.append(QStringLiteral("A"), lcnc::MachineAxisRole::HeadTiltPrimary);
    headRequest.headToolGeometry.focusLength = 100.0;
    headRequest.points = &headPoints;
    const auto headPose = registry.solve(headRequest);
    ok &= require(headPose.size() == 1 && headPose.front().valid
                  && isNear(headPose.front().value(0), 10.0)
                  && isNear(headPose.front().value(1), 20.0)
                  && isNear(headPose.front().value(2), 100.0),
                  "Head software TCP did not compensate the focus length");

    headPoints.front().normal = gp_Dir(0, 1, 0);
    const auto tiltedHeadPose = registry.solve(headRequest);
    ok &= require(tiltedHeadPose.size() == 1 && tiltedHeadPose.front().valid
                  && std::abs(tiltedHeadPose.front().value(4)) > 45.0,
                  "Head software TCP did not solve a tilted reachable posture");

    return ok ? 0 : 1;
}
