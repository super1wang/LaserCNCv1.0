#include "modules/process/device/motion_control/gtn_motion_control.h"

#include "core/kernel/kernel.h"
#include "core/kinematics/machine_configuration_service.h"
#include "core/kinematics/machine_calibration_service.h"
#include "core/logging/logger.h"
#include "modules/process/runtime/rtcp_target_validation.h"

#include "magic_enum.hpp"
#include <algorithm>
#include <cmath>

using lcnc::process::GtnSealResult;
using lcnc::process::GtnExactCommand;
using lcnc::process::PreparedDeviceProgram;
using lcnc::process::DigitalOUT;

lcnc::cam::ControllerQualificationSnapshot GTNMotionControl::currentExactQualification() const
{
    lcnc::cam::ControllerQualificationSnapshot result;
    result.sourceId = QStringLiteral("gtn/qualification-service-unavailable");
    return result; // installation of an authority is a separate qualification task
}

bool GTNMotionControl::exactApi(short result, const char* operation, QString* error)
{
    if (result == 0) return true;
    m_bErrorOccurred = true;
    if (error) *error = QStringLiteral("GTN exact %1 failed: %2").arg(QString::fromLatin1(operation)).arg(result);
    LogError("ExactGroup", operation, "", result);
    return false;
}

bool GTNMotionControl::admit(const PreparedDeviceProgram& program, int ordinal, QString* error)
{
    const auto fail = [&](const char* text) { if (error) *error = QString::fromLatin1(text); return false; };
    const auto live = currentExactQualification();
    if (!lcnc::cam::controllerQualificationIsQualified(live)
        || lcnc::cam::controllerQualificationSnapshotHash(live) != program.plan().context.controllerCapabilityHash)
        return fail("GTN current controller qualification is unavailable/stale");
    if (!program.realMachine() || !lcnc::process::validateGtnGroupProfile(program, error)) return false;
    if (!requireInitializedConnection("ExactAdmission") || m_bStop.load() || ErrorOccurred())
        return fail("GTN exact admission requires healthy initialized controller without Stop/fault");
    int fault = 0;
    if (!IsMachiningStatusNormal(fault) || fault != 0) return fail("GTN machining fault blocks exact admission");
    if (ordinal < 0 || ordinal >= program.sections().size()) return fail("Invalid GTN section");
    const auto& p = program.recipe().gtnLowering;
    const auto& g = p.group;
    const auto& first = program.plan().blocks[program.sections()[ordinal].firstBlock];
    const auto& entry = first.hasEntryBoundary ? first.entryBoundary : first.physicalKnots.front();
    for (int i = 0; i < 5; ++i) {
        const auto named = magic_enum::enum_cast<lcnc::process::Axis>(p.axes[i].name.toStdString());
        if (!named) return fail("GTN physical axis name unavailable");
        const auto motor = m_mapMotorValue.find(*named);
        if (motor == m_mapMotorValue.end() || motor->second.AxisIndex != p.axes[i].controllerAxis
            || motor->second.Resolution != g.kinematics.scales[i].beta)
            return fail("GTN current axis mapping/scale changed");
        long status = 0;
        double encoder = 0;
        if (!exactApi(GTN_GetSts(m_iCore, static_cast<short>(p.axes[i].controllerAxis), &status), "GetSts", error)
            || !exactApi(GTN_GetEncPos(m_iCore, static_cast<short>(p.axes[i].controllerAxis), &encoder), "GetEncPos", error)) return false;
        if ((status & 0x400) || !std::isfinite(encoder)
            || std::abs(encoder / motor->second.Resolution - entry.axes[p.axes[i].physicalIndex]) > g.startPositionTolerance)
            return fail("GTN start position does not match the CAM entry boundary");
    }
    const DigitalOUT names[] = {DigitalOUT::Laser, DigitalOUT::Blow, DigitalOUT::Blow2};
    for (int i = 0; i < 3; ++i) {
        const auto found = m_mapDigitalOUT.find(names[i]);
        const auto& out = g.outputs[i];
        if (out.index == 0) { if (found != m_mapDigitalOUT.end()) return fail("GTN IO mapping changed"); continue; }
        if (found == m_mapDigitalOUT.end()) return fail("GTN frozen IO is missing on the device");
        const auto& actual = found->second;
        const int on = actual.bExpand ? (actual.bInversion ? 0 : 1) : (actual.bInversion ? 1 : 0);
        if (actual.iIO != out.index || actual.bExpand != out.expanded || on != out.onValue)
            return fail("GTN current IO differs from frozen IO");
    }
    auto* kernel = lcnc::Kernel::tryCurrent();
    auto machine = kernel ? kernel->services().getService<lcnc::MachineConfigurationService>() : nullptr;
    auto calibration = kernel ? kernel->services().getService<lcnc::kinematics::MachineCalibrationService>() : nullptr;
    lcnc::kinematics::ControllerKinematicsSnapshot current;
    if (!machine || !lcnc::kinematics::buildControllerKinematicsSnapshot(*machine, calibration.get(),
        p.mode == lcnc::cam::ControllerMotionMode::RTCP
            ? lcnc::kinematics::ControllerCalibrationRequirement::MachineVerified
            : lcnc::kinematics::ControllerCalibrationRequirement::None, &current, error)) return false;
    if (current.machineKinematicsFingerprint != g.kinematics.machineKinematicsFingerprint
        || current.calibrationFingerprint != g.kinematics.calibrationFingerprint
        || current.toolCalibrationFingerprint != g.kinematics.toolCalibrationFingerprint)
        return fail("GTN live kinematics/calibration changed");
    return true;
}

bool GTNMotionControl::acquire(const PreparedDeviceProgram& program, QString* error)
{
    if (!requireInitializedConnection("ExactAcquire") || m_bStop.load() || m_groupReady)
        return exactApi(-1, "Acquire ownership/connection/Stop", error);
    m_exactProfile = program.recipe().gtnLowering;
    const auto& p = m_exactProfile;
    const auto& g = p.group;
    const auto& k = g.kinematics;
    m_groupIndex = static_cast<short>(g.groupIndex);
    m_commandListIndex = static_cast<short>(g.listIndex);
    TListInfo immediate{};
    TCommandListStatus oldList{};
    TGroupStatus oldGroup{};
    if (!exactApi(GTN_GetCommandListStatus(m_iCore, m_commandListIndex, &oldList), "Read prior list", error)
        || !exactApi(GTN_GetGroupStatus(m_iCore, m_groupIndex, &oldGroup), "Read prior Group", error)) return false;
    if (oldList.execute || oldList.remainderSegCount || oldGroup.run)
        return exactApi(-1, "Prior Group/list is not drained", error);
    if (!stopConfiguredOutputs("ExactAcquireOutputsOff")) return exactApi(-1, "Safe outputs", error);
    long mask = 0;
    for (int i = 0; i < 5; ++i) {
        const auto name = magic_enum::enum_cast<lcnc::process::Axis>(p.axes[i].name.toStdString());
        if (!name) return exactApi(-1, "Axis name", error);
        m_cuttingAxes[i] = *name;
        mask |= AxisMaskByIndex(p.axes[i].controllerAxis);
    }
    m_groupReady = true; // retain potential partial ownership until confirmed release
    if (!stopAndReleaseFiveAxisGroup(mask, "ExactAcquireCleanGroup")) return exactApi(-1, "Prior Group release", error);
    clearGroupRuntimeState();
    if (!synchronizeAxisProfilesToEncoders(mask, "ExactAcquire", true)) return exactApi(-1, "Profile encoder synchronization", error);
    m_groupReady = true;
    m_exactListSealed = false;
    m_exactStartAttempted = false;
    const auto call = [&](short result, const char* api) { return !m_bStop.load() && exactApi(result, api, error); };
    const auto audit = [&](const QString& name, double requested, double effective, const QString& unit) {
        LCNC_INFO(lcnc::LogCode::Generic, "{}", lcnc::process::gtnParameterAudit(name, requested, effective, unit, p).toStdString());
        return std::isfinite(effective) && requested == effective;
    };
    for (int i = 0; i < 5; ++i) {
        TProfileScale scale{};
        scale.count = k.scales[i].count; scale.alpha[0] = k.scales[i].alpha; scale.beta[0] = k.scales[i].beta;
        if (!call(GTN_SetAxisScale(m_iCore, k.physicalAxisIndices[i], &scale, &immediate), "SetAxisScale")
            || !call(GTN_AddAxisToGroup(m_iCore, m_groupIndex, k.physicalAxisIndices[i], static_cast<short>(i + 1), &immediate), "AddAxisToGroup")) return false;
        TAxisMotionConstraint limits{};
        limits.velMax = g.axisVelocity[i]; limits.accMax = limits.decMax = g.axisAcceleration[i]; limits.jerkMax = g.axisJerk[i];
        limits.reverseLimitMode = 1;
        limits.dvMax = g.axisDvMax[i];
        limits.reserve1[AXIS_MOTION_CONSTRAINT_RESERVE1_DV_MAX_LIMIT] = 1;
        TAxisMotionConstraint read{};
        TProfileScale readScale{};
        if (!call(GTN_SetAxisMotionConstraint(m_iCore, k.physicalAxisIndices[i], &limits, &immediate), "SetAxisMotionConstraint")
            || !call(GTN_GetAxisMotionConstraint(m_iCore, k.physicalAxisIndices[i], &read), "GetAxisMotionConstraint")
            || !call(GTN_GetAxisScale(m_iCore, k.physicalAxisIndices[i], &readScale), "GetAxisScale")) return false;
        if (readScale.count != scale.count || readScale.alpha[0] != scale.alpha[0] || readScale.beta[0] != scale.beta[0]
            || read.decMax != limits.decMax || read.reverseLimitMode != limits.reverseLimitMode
            || read.reserve1[AXIS_MOTION_CONSTRAINT_RESERVE1_DV_MAX_LIMIT] != 1
            || !audit(QStringLiteral("axis%1.dvMax").arg(i), limits.dvMax, read.dvMax, i < 3 ? "mm/s" : "deg/s")
            || !audit(QStringLiteral("axis%1.velocity").arg(i), limits.velMax, read.velMax, i < 3 ? "mm/s" : "deg/s")
            || !audit(QStringLiteral("axis%1.acceleration").arg(i), limits.accMax, read.accMax, i < 3 ? "mm/s2" : "deg/s2")
            || !audit(QStringLiteral("axis%1.jerk").arg(i), limits.jerkMax, read.jerkMax, i < 3 ? "mm/s3" : "deg/s3"))
            return exactApi(-1, "Axis profile readback mismatch", error);
    }
    TKinematicTransform transform{};
    transform.type = KIN_TYPE_FIVE_AXIS;
    auto& model = transform.kinPrm.fiveAxis;
    model.type = k.modelType; model.dirMode = k.directionMode;
    for (int j = 0; j < 3; ++j) {
        model.primaryAxisPoint[j] = k.primaryAxisPointMcs[j]; model.slaveAxisPoint[j] = k.slaveAxisPointMcs[j];
        model.toolLocationPoint[j] = k.toolLocationPointMcs[j];
    }
    for (int i = 0; i < 5; ++i) {
        model.dir[i] = k.directions[i];
        for (int j = 0; j < 3; ++j) model.axisVector[i][j] = k.axisVectorsMcs[i][j];
    }
    const bool rtcp = p.mode == lcnc::cam::ControllerMotionMode::RTCP;
    const short coord = rtcp ? COORD_SYSTEM_MCS : COORD_SYSTEM_ACS;
    const short orientationMode = rtcp ? ORI_MODE_ROTATE_AXIS_POS : ORI_MODE_NONE;
    const short profileCoord = rtcp ? COORD_SYSTEM_PCS : COORD_SYSTEM_ACS;
    TCartesianParameter identity{};
    if (!call(GTN_SetGroupKinematicTransform(m_iCore, m_groupIndex, &transform, &immediate), "SetGroupKinematicTransform")
        || !call(GTN_SetGroupCartesianTransform(m_iCore, m_groupIndex, COORD_SYSTEM_PCS, 1, &identity, &immediate), "SetPCS")
        || !call(GTN_SetGroupCartesianTransform(m_iCore, m_groupIndex, COORD_SYSTEM_TCS, 1, &identity, &immediate), "SetTCS")
        || !call(GTN_SetGroupCommandPosDefine(m_iCore, m_groupIndex, coord, orientationMode, 0, &immediate), "SetCommandPosDefine")
        || !call(GTN_SetGroupProfileCoordinateSystem(m_iCore, m_groupIndex, profileCoord, &immediate), "SetProfileCoordinate")
        || !call(GTN_SetCommandListLinkGroup(m_iCore, m_commandListIndex, 1UL << (m_groupIndex - 1)), "LinkGroup")
        || !call(GTN_SetGroupLinkCommandList(m_iCore, m_groupIndex, 1UL << (m_commandListIndex - 1)), "LinkList")) return false;
    TGroupMotionConstraint limits{};
    limits.velMax = g.pathVelocityLimit; limits.accMax = limits.decMax = g.pathAccelerationLimit; limits.jerkMax = g.pathJerkLimit;
    TGroupOrientationConstraint orientation{};
    orientation.oriVelMax = g.orientationVelocity; orientation.oriAccMax = orientation.oriDecMax = g.orientationAcceleration;
    orientation.oriJerkMax = g.orientationJerk;
    TVelProfileMode smooth{};
    smooth.mode = VEL_PROFILE_MODE_SMOOTH; smooth.parameter.smooth.accTime = g.smoothTimeMs; smooth.parameter.smooth.k = g.smoothK;
    TGroupLookAheadParameter look{};
    look.lookAheadNum = g.lookAheadSegments; look.time = g.lookAheadTime; look.radiusRatio = g.lookAheadRadiusRatio;
    double ratios[5]; long refMask = 0;
    for (int i = 0; i < 5; ++i) { ratios[i] = p.referenceRatios[i]; if (ratios[i] > 0) refMask |= 1L << i; }
    if (!call(GTN_SetGroupMotionConstraint(m_iCore, m_groupIndex, &limits, &immediate), "SetMotionConstraint")
        || !call(GTN_SetGroupOrientationConstraint(m_iCore, m_groupIndex, &orientation, &immediate), "SetOrientationConstraint")
        || !call(GTN_GroupEnable(m_iCore, m_groupIndex, &immediate), "GroupEnable")
        || !call(GTN_SetGroupVelProfileMode(m_iCore, m_groupIndex, &smooth, &immediate), "SetVelProfile")
        || !call(GTN_GroupLookAheadEnable(m_iCore, m_groupIndex, &immediate), "LookAheadEnable")
        || !call(GTN_SetGroupLookAheadParameter(m_iCore, m_groupIndex, &look, &immediate), "SetLookAhead")
        || !call(GTN_SetGroupCartesianCoordinateAxisLimit(m_iCore, m_groupIndex, 2, &immediate), "CartesianAxisLimit")
        || !call(GTN_SetGroupCommandVelRefAxis(m_iCore, m_groupIndex, refMask, &immediate), "SetVelRefAxis")
        || !call(GTN_SetGroupCommandVelRefRatio(m_iCore, m_groupIndex, 1, ratios, 5, &immediate), "SetVelRefRatio")
        || !call(GTN_SetVelOverride(m_iCore, 0, 1.0, &immediate), "SetVelOverride")) return false;
    short readCoord = -1, readOri = -1, readConfig = -1, readProfile = -1;
    double readRatios[5]{}, overrideValue = 0; long readRefMask = 0;
    TGroupMotionConstraint readLimits{}; TGroupOrientationConstraint readOrientation{}; TVelProfileMode readSmooth{};
    TKinematicTransform readTransform{};
    if (!call(GTN_GetGroupKinematicTransform(m_iCore, m_groupIndex, &readTransform), "GetKinematicTransform")) return false;
    const auto& readModel = readTransform.kinPrm.fiveAxis;
    if (readTransform.type != transform.type || readModel.type != model.type || readModel.dirMode != model.dirMode)
        return exactApi(-1, "Kinematic model readback mismatch", error);
    for (int i = 0; i < 5; ++i) {
        if (readModel.dir[i] != model.dir[i]) return exactApi(-1, "Kinematic direction readback mismatch", error);
        for (int j = 0; j < 3; ++j)
            if (readModel.axisVector[i][j] != model.axisVector[i][j]) return exactApi(-1, "Kinematic vector readback mismatch", error);
    }
    for (int j = 0; j < 3; ++j)
        if (readModel.primaryAxisPoint[j] != model.primaryAxisPoint[j] || readModel.slaveAxisPoint[j] != model.slaveAxisPoint[j]
            || readModel.toolLocationPoint[j] != model.toolLocationPoint[j]) return exactApi(-1, "Kinematic point readback mismatch", error);
    if (!call(GTN_GetGroupCommandPosDefine(m_iCore, m_groupIndex, &readCoord, &readOri, &readConfig), "GetCommandPosDefine")
        || !call(GTN_GetGroupProfileCoordinateSystem(m_iCore, m_groupIndex, &readProfile), "GetProfileCoordinate")
        || !call(GTN_GetGroupCommandVelRefAxis(m_iCore, m_groupIndex, &readRefMask), "GetVelRefAxis")
        || !call(GTN_GetGroupCommandVelRefRatio(m_iCore, m_groupIndex, 1, readRatios, 5), "GetVelRefRatio")
        || !call(GTN_GetGroupMotionConstraint(m_iCore, m_groupIndex, &readLimits), "GetMotionConstraint")
        || !call(GTN_GetGroupOrientationConstraint(m_iCore, m_groupIndex, &readOrientation), "GetOrientationConstraint")
        || !call(GTN_GetGroupVelProfileMode(m_iCore, m_groupIndex, &readSmooth), "GetVelProfile")
        || !call(GTN_GetVelOverride(m_iCore, 0, &overrideValue), "GetVelOverride")) return false;
    if (readCoord != coord || readOri != orientationMode || readConfig != 0 || readProfile != profileCoord
        || readLimits.decMax != limits.decMax || readOrientation.oriDecMax != orientation.oriDecMax
        || readRefMask != refMask || readSmooth.mode != smooth.mode || !std::equal(ratios, ratios + 5, readRatios)
        || !audit("path.velocity", limits.velMax, readLimits.velMax, p.metric == lcnc::process::GtnFeedMetric::RotaryDegrees ? "deg/s" : "mm/s")
        || !audit("path.acceleration", limits.accMax, readLimits.accMax, "qualified-path-unit/s2")
        || !audit("path.jerk", limits.jerkMax, readLimits.jerkMax, "qualified-path-unit/s3")
        || !audit("orientation.velocity", orientation.oriVelMax, readOrientation.oriVelMax, "deg/s")
        || !audit("orientation.acceleration", orientation.oriAccMax, readOrientation.oriAccMax, "deg/s2")
        || !audit("orientation.jerk", orientation.oriJerkMax, readOrientation.oriJerkMax, "deg/s3")
        || !audit("smooth.time", g.smoothTimeMs, readSmooth.parameter.smooth.accTime, "ms")
        || !audit("smooth.k", g.smoothK, readSmooth.parameter.smooth.k, "ratio")
        || !audit("device.override", 1.0, overrideValue, "ratio")) return exactApi(-1, "Group readback mismatch", error);
    // SDK exposes no LookAhead parameter getter; record accepted submission,
    // not fabricated hardware readback. Qualification covers this API contract.
    LCNC_INFO(lcnc::LogCode::Generic, "gtn.lookahead requested_segments={} requested_time={} requested_radius_ratio={} effective=SDK-accepted readback=unavailable source={} revision={}",
        g.lookAheadSegments, g.lookAheadTime, g.lookAheadRadiusRatio, p.sourceId.toStdString(), p.revision);
    m_groupRtcpActive = rtcp;
    m_groupRtcpConfigurationDerived = false;
    return ResetFiveAxisGroupProgram() || exactApi(-1, "Reset exact list", error);
}

bool GTNMotionControl::validateRtcp(const std::array<double, 5>& target, const std::array<double, 5>& predicted, QString* error)
{
    if (!m_groupReady || !m_groupRtcpActive || !lcnc::process::finiteRtcpPose(target)
        || !lcnc::process::finiteRtcpPose(predicted)) return exactApi(-1, "RTCP target state", error);
    TGroupPosTransformInput input{}; TGroupPosTransformOutput output{};
    input.coordSystem = COORD_SYSTEM_MCS; input.oriMode = input.targetOriMode = ORI_MODE_ROTATE_AXIS_POS;
    for (int i = 0; i < 5; ++i) { input.inputPos[i] = target[i]; input.preAcsPos[i] = predicted[i]; }
    if (!exactApi(GTN_GroupPosTransform(m_iCore, m_groupIndex, &input, &output), "RTCP transform", error)) return false;
    const std::array<double, 5> actual{output.acsPos[0], output.acsPos[1], output.acsPos[2], output.acsPos[3], output.acsPos[4]};
    double maximumError = 0;
    if (output.singularity || !lcnc::process::rtcpAxesAgree(predicted, actual, m_exactProfile.group.rtcpAxisTolerance, &maximumError))
        return exactApi(-1, "RTCP target disagrees with CAM", error);
    return true; // every target; no mutable stride/tolerance settings
}

bool GTNMotionControl::append(const lcnc::process::GtnEncodedSection& section, const GtnExactCommand& command, QString* error)
{
    if (!m_groupReady || m_groupListStarted || m_exactListSealed || m_bStop.load()) return exactApi(-1, "Exact list not writable", error);
    const auto list = [&] { TListInfo value{}; value.list = m_commandListIndex; value.segNum = ++m_groupSegmentNumber;
        value.reserve2[LISTINFO_RESERVE2_USERTAG] = command.userTag; return value; };
    if (command.kind == GtnExactCommand::Kind::LinearAbsolute) {
        TGroupMotionConstraint dynamics{};
        dynamics.velMax = command.dynamics.velocity; dynamics.accMax = dynamics.decMax = command.dynamics.acceleration;
        dynamics.jerkMax = command.dynamics.jerk;
        auto parameters = list();
        if (!exactApi(GTN_SetGroupMotionConstraint(m_iCore, m_groupIndex, &dynamics, &parameters), "Queue exact dynamics", error)) return false;
        TGroupMoveParameter move{};
        move.velocity = dynamics.velMax; move.acceleration = move.deceleration = dynamics.accMax;
        move.orientationDir = static_cast<short>(m_exactProfile.group.orientationDirection);
        const auto found = std::find_if(section.commands().cbegin(), section.commands().cend(),
            [&](const auto& item) { return item.userTag == command.userTag; });
        if (found == section.commands().cend()) return exactApi(-1, "Unknown exact command identity", error);
        bool lastMotion = true;
        for (auto next = found + 1; next != section.commands().cend(); ++next) {
            if (next->kind == GtnExactCommand::Kind::LinearAbsolute) { lastMotion = false; break; }
            if (next->fence.requiredStop) move.endVelocityMode = 1;
        }
        if (lastMotion) move.endVelocityMode = 1;
        double target[8]{}; short direction[8]{};
        std::copy(command.target.cbegin(), command.target.cend(), target);
        auto motion = list();
        if (!exactApi(GTN_MoveLinearAbsolute(m_iCore, m_groupIndex, target, direction, &move, &motion), "Exact MoveLinearAbsolute", error)) return false;
        m_groupListHasMotion = true;
        m_groupCommandPosition = command.target; m_groupCommandPositionValid = true;
        LCNC_DEBUG(lcnc::LogCode::Generic, "gtn.exact tag={} block={} knot={} velocity={} acceleration={} jerk={} metric={} source={} revision={} effective=encoded",
            command.userTag, command.blockId, command.knotIndex, dynamics.velMax, dynamics.accMax, dynamics.jerkMax,
            static_cast<int>(command.dynamics.metric), m_exactProfile.sourceId.toStdString(), m_exactProfile.revision);
    } else if (command.fence.changesLaserState) {
        const auto& tool = section.tool();
        const auto output = [&](int index, bool on) {
            const auto& out = m_exactProfile.group.outputs[index];
            if (!out.index) return !on;
            TDigitalOutputBit bit{};
            bit.doType = out.expanded ? MC_EXT_DO : MC_GPO;
            bit.doIndex = static_cast<short>(out.index); bit.doValue = static_cast<short>(on ? out.onValue : out.offValue);
            auto info = list();
            return exactApi(GTN_WriteDigitalOutputBit(m_iCore, &bit, &info), "Queue frozen IO", error);
        };
        const auto delay = [&](double seconds) {
            if (seconds == 0) return true;
            TDelay wait{}; wait.delayTime = seconds * 1000.0;
            auto info = list();
            return exactApi(GTN_SetDelay(m_iCore, &wait, &info), "Queue frozen delay", error);
        };
        const bool on = command.fence.laserEnabledAfterFence;
        if (on && (!output(tool.blow2 ? 1 : 2, false) || !output(tool.blow2 ? 2 : 1, true) || !delay(tool.blowDelay))) return false;
        if (!delay(on ? tool.beforeOn : tool.beforeOff) || !output(0, on) || !delay(on ? tool.afterOn : tool.afterOff)) return false;
        if (!on && tool.stopBlow && !output(tool.blow2 ? 2 : 1, false)) return false;
    } // structural marker / requiredStop emits no accidental IO or extra point
    m_groupListHasData = true;
    return true;
}

GtnSealResult GTNMotionControl::seal(QString* error)
{
    if (!m_groupReady || m_groupListStarted || m_bStop.load() || !m_groupListHasMotion) {
        exactApi(-1, "Exact DataEnd state", error); return GtnSealResult::Failed;
    }
    const short result = GTN_CommandListDataEnd(m_iCore, m_commandListIndex);
    if (result == 10700) return GtnSealResult::Pending;
    if (!exactApi(result, "Exact DataEnd", error)) return GtnSealResult::Failed;
    m_exactListSealed = true;
    return GtnSealResult::Sealed;
}

bool GTNMotionControl::start(QString* error)
{
    if (!m_exactListSealed || m_exactStartAttempted || !m_groupReady || m_bStop.load() || ErrorOccurred())
        return exactApi(-1, "Exact Start admission/state", error);
    m_exactStartAttempted = true; // non-idempotent; never retry any return code
    TListInfo immediate{};
    const short result = GTN_StartCommandList(m_iCore, m_commandListIndex, &immediate);
    if (!exactApi(result, "Exact StartCommandList", error)) return false;
    m_groupListStarted = true;
    m_groupCompletionSettling = false;
    return true;
}

bool GTNMotionControl::poll(bool& running, QString* error)
{
    running = false;
    if (!m_groupReady || !m_groupListStarted || m_bStop.load()) return exactApi(-1, "Exact poll state/Stop", error);
    running = IsFiveAxisGroupProgramRunning();
    if (ErrorOccurred()) { if (error) *error = GroupExecutionError(); if (error && error->isEmpty()) *error = QStringLiteral("GTN execution fault"); return false; }
    return true;
}

bool GTNMotionControl::stopRelease(bool latch, QString* error)
{
    TListInfo immediate{};
    const auto outputsOff = [&] {
        bool safe = true;
        for (const auto& out : m_exactProfile.group.outputs) {
            if (!out.index) continue;
            TDigitalOutputBit bit{}; bit.doType = out.expanded ? MC_EXT_DO : MC_GPO;
            bit.doIndex = static_cast<short>(out.index); bit.doValue = static_cast<short>(out.offValue);
            safe = exactApi(GTN_WriteDigitalOutputBit(m_iCore, &bit, &immediate), "Exact safe output", error) && safe;
        }
        return safe;
    };
    const bool initialSafe = outputsOff();
    const bool stopped = m_groupReady
        ? stopAndReleaseFiveAxisGroup(configuredAxisMask(), "ExactStopConfirm", false) : true;
    // A running list could have issued another gate after the first OFF.
    const bool finalSafe = outputsOff();
    const bool success = initialSafe && stopped && finalSafe && (!m_groupReady
        || stopAndReleaseFiveAxisGroup(configuredAxisMask(), "ExactRelease", true));
    if (success) clearGroupRuntimeState();
    if (latch || !success) { m_stopFaultLatched = true; m_bErrorOccurred = true; }
    if (!success && error && error->isEmpty()) *error = QStringLiteral("GTN stop/release failed; ownership retained");
    m_exactListSealed = false;
    return success;
}
