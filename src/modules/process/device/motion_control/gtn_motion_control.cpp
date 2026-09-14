#include "gtn_motion_control.h"

#include "modules/process/device/runtime/device_wait.h"
#include <boost/lexical_cast.hpp>
#include <fstream>
//#include "bdaqctrl.h"
#include "core/kernel/kernel.h"
#include "core/kinematics/controller_kinematics_snapshot.h"
#include "core/kinematics/machine_calibration_service.h"
#include "core/kinematics/machine_configuration_service.h"
#include "core/logging/logger.h"
#include "modules/process/settings/process_settings_service.h"
#include "modules/process/system/process_numeric_constants.h"
#include "modules/process/runtime/process_runtime_configuration.h"
#include "modules/process/runtime/motion_feedback_validation.h"
#include "modules/process/runtime/command_list_start.h"
#include "modules/process/runtime/confirmed_motion_stop.h"
#include "modules/process/runtime/controller_recovery_sequence.h"
#include "modules/process/runtime/group_motion_filter.h"
#include "modules/process/runtime/gtn_axis_status_policy.h"
#include "modules/process/runtime/rtcp_target_validation.h"

#include "magic_enum.hpp"

#include <QScopeGuard>
#include <QCryptographicHash>
#include <QDir>
#include <QFile>

#include <algorithm>
#include <thread>
#include <limits>

using lcnc::process::AnalogOUT;
using lcnc::process::Axis;
using lcnc::process::DigitalOUT;
using std::string;
using std::vector;
using toml::table;

namespace {
constexpr double kMinTrapSmoothTimeMs = 0.0;
constexpr double kMaxTrapSmoothTimeMs = 50.0;
constexpr double kMinJogSmooth = 0.0;
constexpr double kMaxJogSmoothExclusive = 1.0;
constexpr long kGtnDriverFaultMask = lcnc::process::kGtnMachiningDriverFaultMask;
constexpr long kGtnPositiveLimitBit = lcnc::process::kGtnMachiningPositiveLimitBit;
constexpr long kGtnNegativeLimitBit = lcnc::process::kGtnMachiningNegativeLimitBit;
constexpr long kGtnLimitMask = kGtnPositiveLimitBit | kGtnNegativeLimitBit;
constexpr auto kFifoFlushTimeout = std::chrono::seconds(2);
constexpr auto kFifoFlushRetryInterval = std::chrono::milliseconds(5);
constexpr short kCommandListDataPending = 10700;
// 固高《新架构功能编程手册》的指令表停止原因：10 表示指令流跑空。
// A finite list terminated by GTN_CommandListDataEnd is therefore complete,
// not faulted, when both the list and Group have stopped normally.
constexpr short kCommandListStopInfoNone = 0;
constexpr short kCommandListStopInfoStreamEmpty = 10;
constexpr const char* kGtnAdapterRevision = "normal-rtcp-transition-v11.4";

constexpr bool isNormalCommandListCompletion(short listStopInfo,
	short listExecute, short groupRun, short groupState)
{
	return listStopInfo == kCommandListStopInfoStreamEmpty
		&& listExecute == 0 && groupRun == 0
		&& groupState == GROUP_STATE_STANDBY;
}

static_assert(isNormalCommandListCompletion(10, 0, 0, GROUP_STATE_STANDBY));
static_assert(!isNormalCommandListCompletion(10, 1, 0, GROUP_STATE_STANDBY));
static_assert(!isNormalCommandListCompletion(10, 0, 0, GROUP_STATE_ERROR_STOP));

constexpr long classifyGtnAxisFault(long status,
	bool hardwarePositiveLimit,
	bool hardwareNegativeLimit,
	bool softwarePositiveLimit,
	bool softwareNegativeLimit)
{
	long fault = status & kGtnDriverFaultMask;
	// A software limit is a directional boundary rather than a resettable
	// controller alarm. Allow motion away from it; the GTN controller still
	// rejects motion farther outside the configured range. Hardware or
	// unclassified limit states remain fail-closed.
	if ((status & kGtnPositiveLimitBit)
		&& (hardwarePositiveLimit || !softwarePositiveLimit))
		fault |= kGtnPositiveLimitBit;
	if ((status & kGtnNegativeLimitBit)
		&& (hardwareNegativeLimit || !softwareNegativeLimit))
		fault |= kGtnNegativeLimitBit;
	return fault;
}

static_assert(classifyGtnAxisFault(0x220, false, false, true, false) == 0);
static_assert(classifyGtnAxisFault(0x220, true, false, false, false) == 0x20);
static_assert(classifyGtnAxisFault(0x240, false, false, false, true) == 0);
static_assert(classifyGtnAxisFault(0x2, false, false, false, false) == 0x2);

bool validTrapSmoothTime(double value)
{
	return std::isfinite(value)
		&& value >= kMinTrapSmoothTimeMs && value <= kMaxTrapSmoothTimeMs
		&& std::floor(value) == value;
}

bool validJogSmooth(double value)
{
	return std::isfinite(value)
		&& value >= kMinJogSmooth && value < kMaxJogSmoothExclusive;
}
}

GTNMotionControl::GTNMotionControl(lcnc::process::ProcessSettingsService& settings,
	                                 lcnc::process::ProcessRuntimeConfiguration& runtimeConfiguration)
	: MotionControl(settings, runtimeConfiguration)
	, m_bConnectFlag(false)
	, m_strName("GTN")
	, m_iCore(1)
	, m_dPreX(0)
	, m_dPreY(0)
	, m_dPreZ(0)
	, m_dPreR1(0)
	, m_dPreR2(0)
	, m_bErrorOccurred(false)
	, m_dLaserOnBWait(0)
	, m_dLaserOnAWait(0)
	, m_dLaserOffBWait(0)
	, m_dLaserOffAWait(0)
	, m_dDiameter(1)
	
	, m_dBlowDelay(0)
	, m_bStop(false)
	, m_iWriteBuf(0)
	, m_inRunBuf(1)
	, m_bCrdStarted(false)
	
{
	m_vecMotors.reserve(8);  // 预分配8个元素空间
}

GTNMotionControl::~GTNMotionControl(void)
{
	ReleaseHomeParameters();
// 	if (!IsConnected())
// 	{
// 		return;
// 	}
// 	short sRtn;
// 	for (Axis axis : m_vecMotors)
// 	{
// 		string strIndex = boost::lexical_cast<string>(m_mapMotorValue[axis].AxisIndex);
// 		int iAxisIndex = m_mapMotorValue[axis].AxisIndex;
// 		//sRtn = GTN_LmtsOffEx(m_iCore, iAxisIndex, -1, 1);
// 		if (0 != GTN_LmtsOffEx(m_iCore, iAxisIndex, -1, 1))
// 		{
// 			LCNC_ERR(lcnc::LogCode::Generic, "{}", fmt::format("GTN Disconnect failed.retrun:{}", sRtn));
// 		}
// 	}
}

const string& GTNMotionControl::GetName() const
{
	return m_strName;
}

void GTNMotionControl::LogError(string strhandle, string command, string name, short error)
{
	if (m_groupExecutionError.isEmpty()) {
		// 中文翻译：GTN 调用失败：操作 %1，API %2，轴 %3，返回值 %4
		m_groupExecutionError = QCoreApplication::translate("GTNMotionControl",
			"GTN call failed: operation %1, API %2, axis %3, result %4")
			.arg(QString::fromStdString(strhandle), QString::fromStdString(command),
				QString::fromStdString(name.empty() ? "-" : name)).arg(error);
	}
	LCNC_ERR(lcnc::LogCode::Generic,
		"gtn.api: operation={} api={} axis={} result={}",
		strhandle, command, name.empty() ? "-" : name, error);
}

long GTNMotionControl::AxisMaskByIndex(int iAxisIndex) const
{
	if (iAxisIndex < 1 || iAxisIndex > 8)
		return 0;
	return 1 << (iAxisIndex - 1);
}

long GTNMotionControl::AxisMask(Axis eAxis) const
{
	auto it = m_mapMotorValue.find(eAxis);
	if (it == m_mapMotorValue.end())
		return 0;
	return AxisMaskByIndex(it->second.AxisIndex);
}

long GTNMotionControl::configuredAxisMask() const
{
	long mask = 0;
	for (const auto& entry : m_mapMotorValue)
		mask |= AxisMaskByIndex(entry.second.AxisIndex);
	return mask;
}

bool GTNMotionControl::requireInitializedConnection(const char* operation)
{
	if (m_stopFaultLatched) {
		m_bErrorOccurred = true;
		LogError(operation, "previous stop failed; explicit stop reset or reconnect required before motion", "", -1);
		return false;
	}
	if (m_bConnectFlag && m_connectionInitialized)
		return true;
	m_bErrorOccurred = true;
	LogError(operation, "connection initialization incomplete; motion prohibited", "", -1);
	return false;
}

bool GTNMotionControl::waitForStoppedMotion(long axisMask, bool includeGroup, const char* operation)
{
	using lcnc::process::MotionStopObservation;
	TCommandListStatus listStatus{};
	TGroupStatus groupStatus{};
	long movingMask = 0;
	bool listReadValid = false;
	bool groupReadValid = false;
	bool axisReadsValid = false;
	unsigned int samples = 0;
	const auto began = std::chrono::steady_clock::now();
	const auto confirmation = lcnc::process::waitForConfirmedMotionStop([&] {
		++samples;
		listReadValid = false;
		groupReadValid = false;
		axisReadsValid = false;
		bool groupStopped = true;
		if (includeGroup) {
			short result = GTN_GetCommandListStatus(m_iCore, m_commandListIndex, &listStatus);
			listReadValid = result == 0;
			if (result != 0) {
				LogError(operation, "GTN_GetCommandListStatus(stop-confirmation)", "", result);
				return MotionStopObservation::ReadFailed;
			}
			result = GTN_GetGroupStatus(m_iCore, m_groupIndex, &groupStatus);
			groupReadValid = result == 0;
			if (result != 0) {
				LogError(operation, "GTN_GetGroupStatus(stop-confirmation)", "", result);
				return MotionStopObservation::ReadFailed;
			}
			groupStopped = listStatus.execute == 0 && groupStatus.run == 0
				&& (groupStatus.state == GROUP_STATE_STANDBY
					|| groupStatus.state == GROUP_STATE_DISABLED
					|| groupStatus.state == GROUP_STATE_ERROR_STOP);
		}
		movingMask = 0;
		for (const auto& entry : m_mapMotorValue) {
			const auto& motor = entry.second;
			const long mask = AxisMaskByIndex(motor.AxisIndex);
			if (!(axisMask & mask))
				continue;
			long status = 0;
			const short result = GTN_GetSts(m_iCore, motor.AxisIndex, &status);
			if (result != 0) {
				LogError(operation, "GTN_GetSts(stop-confirmation)", motor.Name, result);
				return MotionStopObservation::ReadFailed;
			}
			if (status & 0x400)
				movingMask |= mask;
		}
		axisReadsValid = true;
		return groupStopped && movingMask == 0
			? MotionStopObservation::Stopped : MotionStopObservation::Moving;
	}, std::chrono::seconds(30), std::chrono::milliseconds(10));
	const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::steady_clock::now() - began).count();
	const bool confirmed = confirmation == lcnc::process::MotionStopConfirmation::Confirmed;
	LCNC_INFO(lcnc::LogCode::Generic,
		"gtn.api: operation={} phase=stop_confirmation group={} list={} check_group={} axis_mask=0x{:x} moving_mask=0x{:x} list_execute={} list_stop_info={} group_run={} group_state={} group_stop_info={} read_valid=[list:{},group:{},axes:{}] samples={} elapsed_ms={} confirmed={} result={}",
		operation, m_groupIndex, m_commandListIndex, includeGroup, axisMask, movingMask,
		listStatus.execute, listStatus.stopInfo, groupStatus.run, groupStatus.state,
		groupStatus.stopInfo, listReadValid, groupReadValid, axisReadsValid, samples, elapsedMs, confirmed,
		confirmed ? 0 : (confirmation == lcnc::process::MotionStopConfirmation::ReadFailed ? -1 : -2));
	if (!confirmed) {
		m_stopFaultLatched = true;
		m_bErrorOccurred = true;
		LogError(operation, confirmation == lcnc::process::MotionStopConfirmation::ReadFailed
			? "stop status unknown; preserve ownership and evidence"
			: "stop timeout; preserve ownership and evidence", "", -2);
	}
	return confirmed;
}

bool GTNMotionControl::stopAndReleaseFiveAxisGroup(long axisMask, const char* operation, bool allowRelease)
{
	bool completed = false;
	const auto preserveStopFailure = qScopeGuard([this, &completed] {
		if (!completed) {
			m_stopFaultLatched = true;
			m_bErrorOccurred = true;
		}
	});
	TListInfo immediate{};
	TCommandListStatus listStatus{};
	TGroupStatus groupStatus{};
	bool success = true;
	const auto checked = [&](const char* api, short result) {
		LCNC_INFO(lcnc::LogCode::Generic,
			"gtn.api: operation={} phase=stop_release api={} group={} list={} result={}",
			operation, api, m_groupIndex, m_commandListIndex, result);
		if (result != 0) {
			LogError(operation, api, "", result);
			m_bErrorOccurred = true;
			return false;
		}
		return true;
	};
	const bool listRead = checked("GTN_GetCommandListStatus", GTN_GetCommandListStatus(
		m_iCore, m_commandListIndex, &listStatus));
	const bool groupRead = checked("GTN_GetGroupStatus", GTN_GetGroupStatus(
		m_iCore, m_groupIndex, &groupStatus));
	success = listRead && groupRead;
	// GroupStop only stops immediate Group moves. A list in Delay/IO wait
	// can have an idle Group while still being able to execute later outputs.
	if (!listRead || listStatus.execute != 0)
		success = checked("GTN_StopCommandList", GTN_StopCommandList(
			m_iCore, m_commandListIndex, 0, &immediate)) && success;
	if (listRead && listStatus.execute == 0 && groupRead
		&& (groupStatus.run != 0 || groupStatus.state == GROUP_STATE_MOVING
			|| groupStatus.state == GROUP_STATE_STOPPING || groupStatus.state == GROUP_STATE_HOMING))
		success = checked("GTN_GroupStop", GTN_GroupStop(m_iCore, m_groupIndex, &immediate)) && success;
	// A point/jog move can be independent of the Group. Read it explicitly;
	// normal completed-list cleanup must not issue a new stop to idle axes.
	bool axesKnown = true;
	bool axesMoving = false;
	for (const auto& entry : m_mapMotorValue) {
		const auto& motor = entry.second;
		if (!(axisMask & AxisMaskByIndex(motor.AxisIndex)))
			continue;
		long status = 0;
		const short result = GTN_GetSts(m_iCore, motor.AxisIndex, &status);
		if (result != 0) {
			LogError(operation, "GTN_GetSts(before-stop)", motor.Name, result);
			axesKnown = false;
		} else if (status & 0x400) {
			axesMoving = true;
		}
	}
	success = axesKnown && success;
	if (axisMask != 0 && (!axesKnown || axesMoving))
		success = checked("GTN_Stop", GTN_Stop(m_iCore, axisMask, 0)) && success;
	const bool stopped = waitForStoppedMotion(axisMask, true, operation);
	if (!success || !stopped) {
		m_bErrorOccurred = true;
		LCNC_ERR(lcnc::LogCode::Generic,
			"gtn.api: operation={} phase=stop_release action=retain_group_and_list preserve_evidence=true result=-1",
			operation);
		return false;
	}
	if (!allowRelease) {
		LCNC_WARN(lcnc::LogCode::Generic,
			"gtn.api: operation={} phase=stop_release action=preserve_group_and_list reason=earlier_stop_failed stop_confirmed=true cleanup=false preserve_evidence=true result=0",
			operation);
		completed = true;
		return true;
	}
	// Cleanup is strictly conditional on successful requests and valid stopped
	// samples. Abort the cleanup chain at its first failed call.
	if (!checked("GTN_GroupDisable", GTN_GroupDisable(m_iCore, m_groupIndex, &immediate))
		|| !checked("GTN_UngroupAllAxes", GTN_UngroupAllAxes(m_iCore, m_groupIndex, &immediate))
		|| !checked("GTN_ClearCommandListData", GTN_ClearCommandListData(m_iCore, m_commandListIndex, &immediate))
		|| !checked("GTN_ClearCommandListStatus", GTN_ClearCommandListStatus(m_iCore, m_commandListIndex, &immediate)))
		return false;
	completed = true;
	return true;
}

bool GTNMotionControl::stopConfiguredOutputs(const char* operation)
{
	bool success = true;
	for (const auto output : {DigitalOUT::Laser, DigitalOUT::Blow, DigitalOUT::Blow2}) {
		const auto found = m_mapDigitalOUT.find(output);
		if (found == m_mapDigitalOUT.end())
			continue;
		const auto& io = found->second;
		const bool valid = io.iIO > 0 && io.iIO <= (io.bExpand ? 12 : 16);
		const bool off = valid && DigitalOutputSet(found->second, 0, true);
		LCNC_INFO(lcnc::LogCode::Generic,
			"gtn.api: operation={} phase=configured_safe_output output={} hardware_id={} index={} expanded={} inverted={} logical_value=0 success={}",
			operation, magic_enum::enum_name(output), io.qstrID.toStdString(), io.iIO,
			io.bExpand, io.bInversion, off);
		if (!off) {
			m_bErrorOccurred = true;
			LogError(operation, "configured safe output OFF failed", std::string(magic_enum::enum_name(output)), -1);
			success = false;
		}
	}
	return success;
}

bool GTNMotionControl::validateStationaryFeedback(
	long axisMask, const char* operation, bool reportMismatch)
{
	bool success = true;
	for (const auto& entry : m_mapMotorValue) {
		const auto& motor = entry.second;
		if ((axisMask & AxisMaskByIndex(motor.AxisIndex)) == 0)
			continue;
		double encoder = 0.0;
		double profile = 0.0;
		short result = GTN_GetEncPos(m_iCore, motor.AxisIndex, &encoder);
		if (result == 0)
			result = GTN_GetPrfPos(m_iCore, motor.AxisIndex, &profile);
		if (result != 0) {
			LogError(operation, "GTN_GetEncPos+GTN_GetPrfPos(feedback-check)", motor.Name, result);
			return false;
		}
		if (!lcnc::process::stationaryFeedbackMatches(profile, encoder, motor.Resolution)) {
			success = false;
			if (reportMismatch) {
				m_feedbackFault.capture(QString::fromStdString(motor.Name), motor.AxisIndex,
					profile, encoder, motor.Resolution, motor.Rotary);
				LCNC_ERR(lcnc::LogCode::Generic,
					"gtn.api: operation={} phase=feedback_mismatch axis={} index={} profile_pulse={} encoder_pulse={} resolution={} tolerance_axis_units=0.05 minimum_tolerance_pulse=2 action=reject_next_motion result=-1",
					operation, motor.Name, motor.AxisIndex, profile, encoder, motor.Resolution);
			}
		}
	}
	return success;
}

void GTNMotionControl::logAxisPositionEvidence(long axisMask, const char* operation)
{
	// Diagnostic-only reads at boundaries, never in the status refresh loop.
	// Each value has its own return code; NaN means unavailable, not zero.
	for (const auto& entry : m_mapMotorValue) {
		const auto& motor = entry.second;
		if (!(axisMask & AxisMaskByIndex(motor.AxisIndex))) continue;
		const double unknown = std::numeric_limits<double>::quiet_NaN();
		double profile = unknown, encoder = unknown, axisProfile = unknown, axisEncoder = unknown;
		double profileVelocity = unknown, encoderVelocity = unknown;
		unsigned long profileClock = 0, encoderClock = 0;
		const short pr = GTN_GetPrfPos(m_iCore, motor.AxisIndex, &profile, 1, &profileClock);
		const short er = GTN_GetEncPos(m_iCore, motor.AxisIndex, &encoder, 1, &encoderClock);
		const short apr = GTN_GetAxisPrfPos(m_iCore, motor.AxisIndex, &axisProfile);
		const short aer = GTN_GetAxisEncPos(m_iCore, motor.AxisIndex, &axisEncoder);
		const short pvr = GTN_GetPrfVel(m_iCore, motor.AxisIndex, &profileVelocity);
		const short evr = GTN_GetEncVel(m_iCore, motor.AxisIndex, &encoderVelocity);
		LCNC_INFO(lcnc::LogCode::Generic,
			"gtn.api: operation={} phase=position_evidence core={} axis={} index={} resolution={} unit={} raw_profile_pulse={} raw_encoder_pulse={} mapped_axis_profile={} mapped_axis_encoder={} profile_velocity_pulse_per_ms={} encoder_velocity_pulse_per_ms={} profile_clock={} encoder_clock={} read_results=[{},{},{},{},{},{}] sampling=sequential action=read_only",
			operation, m_iCore, motor.Name, motor.AxisIndex, motor.Resolution,
			motor.Rotary ? "deg" : "mm", profile, encoder, axisProfile, axisEncoder,
			profileVelocity, encoderVelocity, profileClock, encoderClock, pr, er, apr, aer, pvr, evr);
	}
}

bool GTNMotionControl::synchronizeAxisProfilesToEncoders(
	long axisMask, const char* operation, bool requireConsistentFeedback)
{
	bool synchronized = false;
	const auto preserveSyncFailure = qScopeGuard([this, &synchronized] {
		if (!synchronized) {
			m_stopFaultLatched = true;
			m_bErrorOccurred = true;
		}
	});
	if (m_stopFaultLatched) {
		logAxisPositionEvidence(axisMask, operation);
		LCNC_WARN(lcnc::LogCode::Generic,
			"gtn.api: operation={} phase=profile_sync action=skip reason=stop_fault_latched preserve_profile=true mask=0x{:x} result=0",
			operation, axisMask);
		synchronized = !requireConsistentFeedback;
		return synchronized;
	}
	if (!m_connectionInitialized) {
		LCNC_WARN(lcnc::LogCode::Generic,
			"gtn.api: operation={} phase=profile_sync action=skip reason=connection_initialization_incomplete preserve_profile=true mask=0x{:x} result=0",
			operation, axisMask);
		synchronized = !requireConsistentFeedback;
		return synchronized;
	}
	if (m_groupFeedbackFaultLatched) {
		logAxisPositionEvidence(axisMask, operation);
		LCNC_WARN(lcnc::LogCode::Generic,
			"gtn.api: operation={} phase=profile_sync action=skip reason=feedback_fault_latched preserve_profile=true mask=0x{:x} result=0",
			operation, axisMask);
		synchronized = !requireConsistentFeedback;
		return synchronized;
	}
	if (axisMask == 0) {
		synchronized = true;
		return true;
	}
	// Preflight every axis before changing any profile. A new machining batch
	// must not absorb an opposite-sign encoder or a multi-turn tracking error.
	// 中文翻译：加工批次不得用反向反馈重设规划起点，避免轮廓间偏差不断放大。
	if (requireConsistentFeedback && !validateStationaryFeedback(axisMask, operation, true)) {
		m_bErrorOccurred = true;
		m_groupFeedbackFaultLatched = true;
		logAxisPositionEvidence(axisMask, operation);
		return false;
	}

	double maximumDifference = 0.0;
	int synchronizedAxes = 0;
	for (const auto& entry : m_mapMotorValue) {
		const int physicalAxis = entry.second.AxisIndex;
		const long physicalMask = AxisMaskByIndex(physicalAxis);
		if ((axisMask & physicalMask) == 0)
			continue;

		double encoderPosition = 0.0;
		double profilePosition = 0.0;
		short result = GTN_GetEncPos(m_iCore, physicalAxis, &encoderPosition);
		if (result != 0)
			return LogError(operation, "GTN_GetEncPos(profile-sync)",
				entry.second.Name, result), false;
		result = GTN_GetPrfPos(m_iCore, physicalAxis, &profilePosition);
		if (result != 0)
			return LogError(operation, "GTN_GetPrfPos(profile-sync)",
				entry.second.Name, result), false;
		maximumDifference = std::max(
			maximumDifference, std::abs(profilePosition - encoderPosition));
		LCNC_INFO(lcnc::LogCode::Generic,
			"gtn.api: operation={} phase=profile_write axis={} index={} profile_before_pulse={} encoder_sample_pulse={} requested_profile_pulse={} source=encoder_feedback motion_command=false result=pending",
			operation, entry.second.Name, physicalAxis, profilePosition, encoderPosition, encoderPosition);
		result = GTN_SetPrfPosEx(m_iCore, physicalAxis, encoderPosition);
		LCNC_INFO(lcnc::LogCode::Generic,
			"gtn.api: operation={} phase=profile_write api=GTN_SetPrfPosEx axis={} index={} requested_profile_pulse={} result={}",
			operation, entry.second.Name, physicalAxis, encoderPosition, result);
		if (result != 0)
			return LogError(operation, "GTN_SetPrfPosEx(profile-sync)",
				entry.second.Name, result), false;
		++synchronizedAxes;
	}

	const short result = GTN_SynchAxisPos(m_iCore, axisMask);
	if (result != 0)
		return LogError(operation, "GTN_SynchAxisPos(profile-sync)", "", result), false;
	LCNC_INFO(lcnc::LogCode::Generic,
		"gtn.api: operation={} phase=profile_sync api=GTN_GetEncPos+GTN_GetPrfPos+GTN_SetPrfPosEx+GTN_SynchAxisPos mask=0x{:x} axes={} maximum_difference_pulse={} result=0",
		operation, axisMask, synchronizedAxes, maximumDifference);
	synchronized = true;
	return true;
}

void GTNMotionControl::ReleaseHomeParameters()
{
	for (auto& pair : m_mapMotorValue)
	{
		delete pair.second.pTHomePrm;
		pair.second.pTHomePrm = nullptr;
	}
}

bool GTNMotionControl::ClearGSNAlarm(Axis eAxis)
{
	short sRtn;
	int AxisIndex = m_mapMotorValue[eAxis].AxisIndex;
	THomePrm tHomePrm;
	sRtn = GTN_ClearAlarm(m_iCore, AxisIndex, 1, 1);
	if (0!= sRtn)
	{
		LogError("ClearGSNAlarm", "GTN_ClearAlarm", magic_enum::enum_name(eAxis).data(), sRtn);
		return false;
	}
	sRtn = GTN_ClrSts(m_iCore, AxisIndex, 1);
	if (0 != sRtn)
	{
		LogError("ClearGSNAlarm", "GTN_ClrSts", magic_enum::enum_name(eAxis).data(), sRtn);
		return false;
	}
	return true;
}

void GTNMotionControl::CreateMotor(Axis eAxis, const table& tAxis)
{
	m_mapMotorValue[eAxis].Name			= magic_enum::enum_name(eAxis).data();
	m_mapMotorValue[eAxis].AxisIndex	= tAxis.at("iIndex")		.as_integer();
	m_mapMotorValue[eAxis].Resolution	= tAxis.at("fResolution")	.as_floating();
	m_mapMotorValue[eAxis].Rotary		= tAxis.at("bRotation")		.as_boolean();
	m_mapMotorValue[eAxis].Velocity		= tAxis.at("fVel")			.as_floating();
	m_mapMotorValue[eAxis].Acceleration	= tAxis.at("fAcc")			.as_floating();
	m_mapMotorValue[eAxis].Deceleration	= tAxis.count("fDec") ? tAxis.at("fDec").as_floating() : tAxis.at("fAcc").as_floating();
	m_mapMotorValue[eAxis].SmoothTime	= tAxis.count("fTrapSmoothTime")
		? tAxis.at("fTrapSmoothTime").as_floating() : 10.0;
	m_mapMotorValue[eAxis].JogSmooth	= tAxis.count("fJogSmooth")
		? tAxis.at("fJogSmooth").as_floating() : 0.5;
	m_mapMotorValue[eAxis].NegLimit		= tAxis.at("fLeftLimit")	.as_floating();
	m_mapMotorValue[eAxis].PosLimit		= tAxis.at("fRightLimit")	.as_floating();

	if (tAxis.count("Home") && tAxis.at("Home").is_table())
		SetAxisHomePrm(eAxis, tAxis.at("Home").as_table());

	m_vecMotors.emplace_back(eAxis);
	LCNC_INFO(lcnc::LogCode::Generic,
		"gtn.api: operation=CreateMotor axis={} index={} resolution={} velocity={} acceleration={} trap_smooth_time_ms={} jog_smooth={} limits=[{},{}]",
		magic_enum::enum_name(eAxis), m_mapMotorValue[eAxis].AxisIndex,
		m_mapMotorValue[eAxis].Resolution, m_mapMotorValue[eAxis].Velocity,
		m_mapMotorValue[eAxis].Acceleration, m_mapMotorValue[eAxis].SmoothTime,
		m_mapMotorValue[eAxis].JogSmooth, m_mapMotorValue[eAxis].NegLimit,
		m_mapMotorValue[eAxis].PosLimit);
}

bool GTNMotionControl::Connect()
{
	short sRtn;
	if (m_bConnectFlag)
	{
		return m_connectionInitialized;
	}
	// The runtime normally creates axis/IO tables after Connect. Recovery must
	// already know its owned physical axes and safe outputs before opening.
	rebuildAxes();
	setDigitalTable();
	m_connectionInitialized = false;
	bool recoveryConfirmed = false;
	auto failConnect = [this, &recoveryConfirmed](const string& command, short error,
		const string& axisName = {}) -> bool {
		LogError("Connect", command, axisName, error);
		m_bErrorOccurred = true;
		// An initialization failure always retains the open handle. Disconnect
		// must confirm both stop and configured safe outputs before closing it.
		// Subsequent Connect cannot report readiness for this partial session.
		LCNC_ERR(lcnc::LogCode::Generic,
			"gtn.api: operation=Connect phase=failed initialized=false stop_confirmed={} handle_retained={} result={}",
			recoveryConfirmed, m_bConnectFlag, error);
		return false;
	};

	LCNC_INFO(lcnc::LogCode::Generic,
		"gtn.api: operation=Connect phase=begin core={} address=5 type=2", m_iCore);
	m_cuttingCoordinateReady = false;
	m_coordinateBufferInitialized = false;
	m_bCrdStarted = false;
	clearGroupRuntimeState();
	sRtn = GTN_Open(5, 2);//连接运动控制器
	if (sRtn != 0)
	{
		LogError("Connect", "GTN_Open", "", sRtn);
		m_bConnectFlag = false;
		return false;
	}
	LCNC_INFO(lcnc::LogCode::Generic,
		"gtn.api: operation=Connect api=GTN_Open address=5 type=2 result=0");
	m_bConnectFlag = true;
	const QString configurationPath = QDir::current().absoluteFilePath(QStringLiteral("gtn_core1.cfg"));
	QFile configurationFile(configurationPath);
	const bool configurationReadable = configurationFile.open(QIODevice::ReadOnly);
	const QByteArray configurationHash = configurationReadable
		? QCryptographicHash::hash(configurationFile.readAll(), QCryptographicHash::Sha256).toHex()
		: QByteArray("unavailable");
	configurationFile.close();
	LCNC_INFO(lcnc::LogCode::Generic,
		"gtn.api: operation=Connect phase=configuration_file path={} sha256={} readable={}",
		configurationPath.toStdString(), configurationHash.toStdString(), configurationReadable);
	// Group ownership survives GTN_Close/GTN_Open on the tested controller.  A
	// previous process that stopped without Disable/Ungroup therefore makes the
	// first GTN_PrfTrap below fail with result 1.  Recover the configured Group
	// before selecting any single-axis profile.
	m_groupIndex = static_cast<short>(m_settings.rawValue(
		lcnc::process::ProcessConfigArea::Devices, QStringLiteral("GTN"),
		QStringLiteral("iFiveAxisGroup"), 1).toInt());
	m_commandListIndex = static_cast<short>(m_settings.rawValue(
		lcnc::process::ProcessConfigArea::Devices, QStringLiteral("GTN"),
		QStringLiteral("iFiveAxisCommandList"), 1).toInt());
	if (m_groupIndex >= 1 && m_groupIndex <= 2
		&& m_commandListIndex >= 1 && m_commandListIndex <= 4) {
		// A previous process may have left a running list while Group.run is
		// zero (Delay/IO). Preserve ownership until both are confirmed stopped.
		m_groupReady = true;
		if (!stopAndReleaseFiveAxisGroup(configuredAxisMask(), "ConnectRecovery"))
			return failConnect("confirmed Group/CommandList recovery", -1);
		clearGroupRuntimeState();
		LCNC_INFO(lcnc::LogCode::Generic,
			"gtn.api: operation=Connect phase=stale_group_recovery group={} list={} confirmed=true result=0",
			m_groupIndex, m_commandListIndex);
	} else {
		return failConnect("invalid recovery Group/CommandList index", -1);
	}
	// Configuration reload must not race motion inherited from an old session.
	recoveryConfirmed = true;
	sRtn = GTN_LoadConfig(m_iCore, const_cast<char*>("gtn_core1.cfg"));
	if (sRtn != 0)
		return failConnect("GTN_LoadConfig", sRtn);
	LCNC_INFO(lcnc::LogCode::Generic,
		"gtn.api: operation=Connect api=GTN_LoadConfig core={} file=gtn_core1.cfg result=0",
		m_iCore);
	// 只对已配置的轴操作，避免对超出控制卡轴数的轴（如4轴卡的轴5-8）发送指令导致连接失败
	for (Axis axis : m_vecMotors)
	{
		int i = m_mapMotorValue[axis].AxisIndex;
		const string axisName(magic_enum::enum_name(axis));
		sRtn = GTN_ClrSts(m_iCore, i, 1);					//清除单轴状态
		if (sRtn != 0)
			return failConnect("GTN_ClrSts", sRtn, axisName);
		sRtn = GTN_SetStopDec(m_iCore, i, 1, 100);			//设置平滑停止减速度和急停减速度pulse/ms2
		if (sRtn != 0)
			return failConnect("GTN_SetStopDec", sRtn, axisName);
		sRtn = GTN_PrfTrap(m_iCore, i);						//设定点位运动
		if (sRtn != 0)
			return failConnect("GTN_PrfTrap", sRtn, axisName);
		sRtn = GTN_SetAxisMotionSmooth(m_iCore, i, 1, 0.5);
		if (sRtn != 0)
			return failConnect("GTN_SetAxisMotionSmooth", sRtn, axisName);
		sRtn = GTN_LmtsOnEx(m_iCore, i, -1, 1);				//控制轴限位有效
		if (sRtn != 0)
			return failConnect("GTN_LmtsOnEx", sRtn, axisName);
		LCNC_INFO(lcnc::LogCode::Generic,
			"gtn.api: operation=Connect phase=axis_init axis={} index={} APIs=GTN_ClrSts+GTN_SetStopDec+GTN_PrfTrap+GTN_SetAxisMotionSmooth+GTN_LmtsOnEx result=0",
			axisName, i);
	}
	sRtn = GTN_ExtModuleInit(m_iCore);	
	if (sRtn != 0)
		return failConnect("GTN_ExtModuleInit", sRtn);
	if (!stopConfiguredOutputs("ConnectAfterRecovery")) {
		return failConnect("configured safe outputs OFF", -1);
	}

	m_bConnectFlag = true;
	m_connectionInitialized = true;
	m_stopFaultLatched = false;
	m_groupFeedbackFaultLatched = false;
	m_feedbackFault = {};
	m_groupExecutionError.clear();
	m_bErrorOccurred = false;
	LCNC_INFO(lcnc::LogCode::Generic,
		"gtn.api: operation=Connect api=GTN_ExtModuleInit core={} adapter_revision={} result=0 phase=complete",
		m_iCore, kGtnAdapterRevision);
	return true;
}

bool GTNMotionControl::Disconnect()
{
	if (!m_bConnectFlag)
		return true;

	LCNC_INFO(lcnc::LogCode::Generic,
		"gtn.api: operation=Disconnect phase=begin core={} axes={}", m_iCore, m_vecMotors.size());
	const bool outputsOff = stopConfiguredOutputs("Disconnect");
	const bool motionStopped = StopMotion();
	// A list may contain later output commands. Repeat OFF only after the stop
	// attempt, and retain the session if either attempt could not be confirmed.
	const bool finalOutputsOff = stopConfiguredOutputs("DisconnectAfterStop");
	if (!outputsOff || !motionStopped || !finalOutputsOff) {
		m_bErrorOccurred = true;
		LCNC_ERR(lcnc::LogCode::Generic,
			"gtn.api: operation=Disconnect phase=retained outputs_off={} motion_stopped={} final_outputs_off={} limits_preserved=true handle_retained=true result=-1",
			outputsOff, motionStopped, finalOutputsOff);
		return false;
	}
	// Disconnect never disables hardware limits. Closing the PC handle does
	// not disable the controller's persistent axis protection.
	const short sRtn = GTN_Close();
	if (0 != sRtn)
	{
		LogError("Disconnect", "GTN_Close", "", sRtn);
		return false;
	}
	
	m_bConnectFlag = false;
	m_connectionInitialized = false;
	m_cuttingCoordinateReady = false;
	m_coordinateBufferInitialized = false;
	m_bCrdStarted = false;
	clearGroupRuntimeState();
	LCNC_INFO(lcnc::LogCode::Generic,
		"gtn.api: operation=Disconnect api=GTN_Close result=0 phase=complete limits_preserved=true success=true");
	return true;
}

bool GTNMotionControl::IsConnected()
{
	return m_bConnectFlag;
}

bool GTNMotionControl::Reboot()
{
	if (!Disconnect())
	{
		LCNC_ERR(lcnc::LogCode::Generic, "GTN Reboot blocked: disconnect did not confirm a safe stop.");
		return false;
	}
	Sleep(500);
	return Connect();
}

bool GTNMotionControl::Home()
{
	if (!requireInitializedConnection("HomeAll"))
		return false;
	// 顺序 Z -> Y -> X -> B -> C -> A。Axis 枚举只包含标准六轴，
	// 不再引用已移除的 X1/Y1/A1 扩展轴。
	int arr[] = { 2, 1, 0, 4, 5, 3 };

	for (const int axisValue : arr)
	{
		Axis eAxis = static_cast<Axis>(axisValue);
		if (!m_runtimeConfiguration.isAxisEnabled(eAxis))
			continue;

		if (m_bStop)
			return false;

		if (!Home(eAxis))
			return false;
		Sleep(10);

		if (m_bStop)
		return false;
	}
	return true;
}

bool GTNMotionControl::Home(Axis eAxis)
{
	if (!requireInitializedConnection("Home"))
		return false;
	short sRtn;
	int AxisIndex = m_mapMotorValue[eAxis].AxisIndex;
	const bool wasEnabled = IsEnabled(eAxis);
	if (!Enable(eAxis))
		return false;
	// The reference GTN implementation documents that the first power-on
	// enable needs settling time for encoder/direction initialization before
	// Smart Home. Re-homing an already enabled axis does not need this delay.
	if (!wasEnabled)
		Sleep(5000);
	if (!ClearGSNAlarm(eAxis)) return false;

	sRtn = GTN_LmtsOffEx(m_iCore, AxisIndex, -1, 1);	//控制轴限位失效
	if (sRtn)
		return LogError("Home", "GTN_LmtsOffEx", magic_enum::enum_name(eAxis).data(), sRtn), false;
	// 限位已关闭，以下所有退出路径必须经过 GTN_LmtsOnEx 恢复限位
	bool bRet = false;
	try
	{
		do {
			sRtn = GTN_ZeroPos(m_iCore, AxisIndex, 1);
			if (sRtn) { LogError("Home", "GTN_ZeroPos", magic_enum::enum_name(eAxis).data(), sRtn); break; }

			THomePrm tHomePrm;
			if (m_mapMotorValue[eAxis].pTHomePrm)
				tHomePrm = *m_mapMotorValue[eAxis].pTHomePrm;	// 使用配置文件中读取的回零参数
			else
			{
				sRtn = GTN_GetHomePrm(m_iCore, AxisIndex, &tHomePrm);	// 退回读控制器默认参数
				if (sRtn != 0) { LogError("Home", "GTN_GetHomePrm", magic_enum::enum_name(eAxis).data(), sRtn); break; }
			}

			sRtn = GTN_GoHome(m_iCore, AxisIndex, &tHomePrm);	//启动Smart Home回原点
			if (sRtn != 0) { LogError("Home", "GTN_GoHome", magic_enum::enum_name(eAxis).data(), sRtn); break; }

			DWORD dwTimeout = GetTickCount64() + 120000;	// 120秒超时
			THomeStatus tHomeSts{};
			bool bAborted = false;
			do
			{
				if (m_bStop)
				{
					// 中文翻译：回原点被用户停止
					LogError("Home", "Return to origin stopped by user", magic_enum::enum_name(eAxis).data(), -1);
					bAborted = true; break;
				}
				if (GetTickCount64() > dwTimeout)
				{
					// 中文翻译：回原点超时
					LogError("Home", "Return to origin timeout", magic_enum::enum_name(eAxis).data(), -2);
					bAborted = true; break;
				}
				sRtn = GTN_GetHomeStatus(m_iCore, AxisIndex, &tHomeSts);	//获取回原点状态
				if (sRtn != 0)
				{
					LogError("Home", "GTN_GetHomeStatus", magic_enum::enum_name(eAxis).data(), sRtn);
					bAborted = true;
					break;
				}
				Sleep(20);
			} while (tHomeSts.run);	// 等待搜索原点停止
			if (bAborted) break;

			// 中文翻译：回原点报错
			if (tHomeSts.error) { LogError("Home", "Return to origin and report error", magic_enum::enum_name(eAxis).data(), tHomeSts.error); break; }

			sRtn = GTN_ZeroPos(m_iCore, AxisIndex, 1);
			if (sRtn != 0) { LogError("Home", "GTN_ZeroPos1", magic_enum::enum_name(eAxis).data(), sRtn); break; }

			bRet = true;
		} while (false);
	}
	catch (...)
	{
		// 中文翻译：回原点异常
		LogError("Home", "Return to origin exception", magic_enum::enum_name(eAxis).data(), -999);
		bRet = false;
	}

	sRtn = GTN_LmtsOnEx(m_iCore, AxisIndex, -1, 1);	//控制轴限位有效（无论成功/失败均须恢复）
	if (sRtn != 0)
		return LogError("Home", "GTN_LmtsOnEx", magic_enum::enum_name(eAxis).data(), sRtn), false;

	if (!IsEnabled(eAxis))
		return false;
	return bRet;
}

bool GTNMotionControl::IsHomed()
{
	for (Axis axis : m_vecMotors)
	{
		if (!IsHomed(axis))
			return false;
	}
	return true;
}

bool GTNMotionControl::IsHomed(Axis eAxis)
{
	if (!IsConnected())
	{
		return false;
	}
	int HomeState = 0;
	short sRtn;
	THomeStatus tHomeSts{};
	sRtn = GTN_GetHomeStatus(m_iCore, m_mapMotorValue[eAxis].AxisIndex, &tHomeSts);//获取回原点状态 
	if (sRtn !=0 )
	{
		LogError("IsHomed", "GTN_GetHomeStatus", magic_enum::enum_name(eAxis).data(), sRtn);
		return false;
	}

	if (tHomeSts.stage != 100)
		return false;
	if (tHomeSts.error)
		return false;
	return true;
}

// [P3 removed] GTNMotionControl::IsHomeBufferRunning

bool GTNMotionControl::Enable()
{
	if (!requireInitializedConnection("EnableAll"))
		return false;
	for (Axis axis : m_vecMotors)
	{
		if (!Enable(axis))
			return false;
	}
	return true;
}

bool GTNMotionControl::Enable(Axis eAxis)
{
	if (!requireInitializedConnection("Enable"))
		return false;
	if (m_groupFeedbackFaultLatched)
		return LogError("Enable", "feedback fault latched; profile write blocked", magic_enum::enum_name(eAxis).data(), -1), false;
	short sRtn = 0;
	if (IsEnabled(eAxis))
		return true;

	int AxisIndex = m_mapMotorValue[eAxis].AxisIndex;
	long mask = AxisMaskByIndex(AxisIndex);
	if (!mask)
		return LogError("Enable", "AxisMaskByIndex", magic_enum::enum_name(eAxis).data(), -1), false;
	sRtn = GTN_AxisOn(m_iCore, AxisIndex);
	if (sRtn != 0)
		return LogError("Enable", "GTN_AxisOn", magic_enum::enum_name(eAxis).data(), sRtn), false;
	double APos;
	sRtn = GTN_GetEncPos(m_iCore, AxisIndex, &APos);//实际位置
	if (0 != sRtn)
		return LogError("Enable", "GTN_GetEncPos", magic_enum::enum_name(eAxis).data(), sRtn), false;
	sRtn = GTN_SetPrfPos(m_iCore, AxisIndex, APos);//规划位置GTN_SetPrfPosEx
	if (0 != sRtn)
		return LogError("Enable", "GTN_SetPrfPos", magic_enum::enum_name(eAxis).data(), sRtn), false;
	sRtn = GTN_SynchAxisPos(m_iCore, mask);
	if (sRtn != 0)
		return LogError("Enable", "GTN_SynchAxisPos", magic_enum::enum_name(eAxis).data(), sRtn) ,false;
	LCNC_INFO(lcnc::LogCode::Generic,
		"gtn.api: operation=Enable api=GTN_AxisOn+GTN_SetPrfPos+GTN_SynchAxisPos axis={} encoder_position={} mask=0x{:x} result=0",
		magic_enum::enum_name(eAxis), APos, mask);
	return true;
}

bool GTNMotionControl::Disable()
{
	for (Axis axis : m_vecMotors)
	{
		if (!Disable(axis))
			return false;
	}
	return true;
}

bool GTNMotionControl::Disable(Axis eAxis)
{
	if (!IsEnabled(eAxis))
		return true;

	short sRtn = 0;
	int AxisIndex = m_mapMotorValue[eAxis].AxisIndex;
	sRtn = GTN_AxisOff(m_iCore, AxisIndex);
	if (sRtn != 0)
		return LogError("Disable", "GTN_AxisOff", magic_enum::enum_name(eAxis).data(), sRtn),false;
	LCNC_INFO(lcnc::LogCode::Generic,
		"gtn.api: operation=Disable api=GTN_AxisOff axis={} result=0",
		magic_enum::enum_name(eAxis));
	return true;
}

bool GTNMotionControl::IsEnabled()
{
	for (Axis axis : m_vecMotors)
	{
		if (!IsEnabled(axis))
			return false;
	}
	return true;
}

bool GTNMotionControl::IsEnabled(Axis eAxis)
{
	if (!IsConnected())
	{
		return false;
	}
	long lAxisStatus;
	short sRtn;
	sRtn = GTN_GetSts(m_iCore, m_mapMotorValue[eAxis].AxisIndex, &lAxisStatus);
	if (sRtn != 0)
		return LogError("IsEnabled", "GTN_GetSts", magic_enum::enum_name(eAxis).data(), sRtn), false;
	if (!(lAxisStatus & 0x200))
	{
		return false;
	}
	else
	{
		return true;
	}

}

bool GTNMotionControl::SetAxisEnable(Axis eAxis, bool bEnable)
{
	if (bEnable)
		return Enable(eAxis);
	else
		return Disable(eAxis);
}

bool GTNMotionControl::Jog(Axis eAxis, bool bDirection, double dVel)
{
	if (!requireInitializedConnection("Jog"))
		return false;
	if (!IsConnected())
		return false;
	short sRtn;
	TJogPrm tJogPrm{};
	sRtn = GTN_PrfJog(m_iCore,m_mapMotorValue[eAxis].AxisIndex);
	if (0 != sRtn)
		return LogError("Jog", "GTN_PrfJog", magic_enum::enum_name(eAxis).data(), sRtn),false;
	double dNewAcc, dNewDec;
	MillimeterToPulse(eAxis, m_mapMotorValue[eAxis].Acceleration / 1000000.00, dNewAcc);
	MillimeterToPulse(eAxis, m_mapMotorValue[eAxis].Deceleration / 1000000.00, dNewDec);
	tJogPrm.acc = dNewAcc;
	tJogPrm.dec = dNewDec;
	tJogPrm.smooth = m_mapMotorValue[eAxis].JogSmooth;
	sRtn = GTN_SetJogPrm(m_iCore, m_mapMotorValue[eAxis].AxisIndex, &tJogPrm);
	if (0 != sRtn)
		return LogError("Jog", "GTN_SetJogPrm", magic_enum::enum_name(eAxis).data(), sRtn), false;
	double dNewVel;
	MillimeterToPulse(eAxis, dVel / 1000.0, dNewVel);
	double directedVel = bDirection ? dNewVel : -dNewVel;
	sRtn = GTN_SetVel(m_iCore, m_mapMotorValue[eAxis].AxisIndex, directedVel);
	if (0 != sRtn)
		return LogError("Jog", "GTN_SetVel", magic_enum::enum_name(eAxis).data(), sRtn), false;
	long mask = AxisMask(eAxis);
	if (!mask)
		return LogError("Jog", "AxisMask", magic_enum::enum_name(eAxis).data(), -1), false;
	sRtn = GTN_Update(m_iCore, mask);
	if (0 != sRtn)
		return LogError("Jog", "GTN_Update", magic_enum::enum_name(eAxis).data(), sRtn), false;
	LCNC_INFO(lcnc::LogCode::Generic,
		"gtn.api: operation=Jog api=GTN_Update axis={} direction={} velocity={} acceleration={} deceleration={} jog_smooth={} mask=0x{:x} result=0",
		magic_enum::enum_name(eAxis), bDirection ? "positive" : "negative", dVel,
		m_mapMotorValue[eAxis].Acceleration, m_mapMotorValue[eAxis].Deceleration,
		m_mapMotorValue[eAxis].JogSmooth, mask);
	return true;
}

bool GTNMotionControl::MoveRelative(Axis eAxis, double dPos, double dVel)
{
	if (!requireInitializedConnection("MoveRelative"))
		return false;
	short sRtn;
	double RelativePos;
	double APos;
	double Velocity = m_mapMotorValue[eAxis].Velocity;
	int iAxis = m_mapMotorValue[eAxis].AxisIndex;
	sRtn = GTN_PrfTrap(m_iCore, iAxis);//设定点位运动
	if (0 != sRtn)
		return LogError("MoveRelative", "GTN_PrfTrap", magic_enum::enum_name(eAxis).data(), sRtn), false;

	double dNewVel, dNewAcc, dNewDec;
	
	if (!SetAxisVelAccDecJerk(eAxis, m_mapMotorValue[eAxis].Velocity, m_mapMotorValue[eAxis].Acceleration, m_mapMotorValue[eAxis].Deceleration, m_mapMotorValue[eAxis].SmoothTime))
		return false;

	if (!MillimeterToPulse(eAxis, dVel / 1000, dNewVel))
		return false;
	sRtn = GTN_SetVel(m_iCore, iAxis, dNewVel);
	if (0 != sRtn)
		return LogError("MoveRelative", "GTN_SetVel", magic_enum::enum_name(eAxis).data(), sRtn), false;
	sRtn = GTN_GetPrfPos(m_iCore, iAxis, &APos);//规划位置
	if (0 != sRtn)
		return LogError("MoveRelative", "GTN_GetPrfPos", magic_enum::enum_name(eAxis).data(), sRtn), false;
	MillimeterToPulse(eAxis, dPos, RelativePos);
	sRtn = GTN_SetPos(m_iCore, iAxis, APos + RelativePos);
	if (0 != sRtn)
		return LogError("MoveRelative", "GTN_SetPos", magic_enum::enum_name(eAxis).data(), sRtn), false;
	long mask = AxisMaskByIndex(iAxis);
	if (!mask)
		return LogError("MoveRelative", "AxisMaskByIndex", magic_enum::enum_name(eAxis).data(), -1), false;
	sRtn = GTN_Update(m_iCore, mask);
	if (0 != sRtn)
		return LogError("MoveRelative", "GTN_Update", magic_enum::enum_name(eAxis).data(), sRtn), false;
	LCNC_INFO(lcnc::LogCode::Generic,
		"gtn.api: operation=MoveRelative api=GTN_Update axis={} distance={} velocity={} target_pulse={} mask=0x{:x} result=0",
		magic_enum::enum_name(eAxis), dPos, dVel, APos + RelativePos, mask);
	return true;
	
}

bool GTNMotionControl::MoveAbsolute(Axis eAxis, double dPos, double dVel)
{
	if (!requireInitializedConnection("MoveAbsolute"))
		return false;
	int iAxis = m_mapMotorValue[eAxis].AxisIndex;
	short sRtn;
	sRtn = GTN_PrfTrap(m_iCore, iAxis);
	if (0 != sRtn)
		return LogError("MoveAbsolute", "GTN_PrfTrap", magic_enum::enum_name(eAxis).data(), sRtn), false;
	double cachedVel = m_mapMotorValue[eAxis].Velocity;
	double vel = (dVel > 0) ? dVel : cachedVel;
	if (!SetAxisVelAccDecJerk(eAxis, vel, m_mapMotorValue[eAxis].Acceleration, m_mapMotorValue[eAxis].Deceleration, m_mapMotorValue[eAxis].SmoothTime))
		return false;
	if (dVel > 0)
		m_mapMotorValue[eAxis].Velocity = cachedVel;	// 恢复缓存速度，避免临时速度污染轴配置
	double dNewPos;
	MillimeterToPulse(eAxis, dPos, dNewPos);
	sRtn = GTN_SetPos(m_iCore, iAxis, (long)dNewPos);
	if (0 != sRtn)
		return LogError("MoveAbsolute", "GTN_SetPos", magic_enum::enum_name(eAxis).data(), sRtn), false;
	long mask = AxisMaskByIndex(iAxis);
	if (!mask)
		return LogError("MoveAbsolute", "AxisMaskByIndex", magic_enum::enum_name(eAxis).data(), -1), false;
	sRtn = GTN_Update(m_iCore, mask);
	if (0 != sRtn)
		return LogError("MoveAbsolute", "GTN_Update", magic_enum::enum_name(eAxis).data(), sRtn), false;
	LCNC_INFO(lcnc::LogCode::Generic,
		"gtn.api: operation=MoveAbsolute api=GTN_Update axis={} position={} velocity={} target_pulse={} mask=0x{:x} result=0",
		magic_enum::enum_name(eAxis), dPos, vel, dNewPos, mask);
	return true;
}

bool GTNMotionControl::MoveMRelative(vector<Axis> vAxis, vector<double> dPos, double dVel)
{
	
	return true;
}

bool GTNMotionControl::MoveMAbsolute(vector<Axis> vAxis, vector<double> dPos, double dVel)
{
	
	return true;
}

bool GTNMotionControl::StopMotion()
{
	bool completed = false;
	const auto preserveStopFailure = qScopeGuard([this, &completed] {
		if (!completed) {
			m_stopFaultLatched = true;
			m_bErrorOccurred = true;
		}
	});
	const bool groupSuccess = StopFiveAxisGroupProgram();
	const long allMask = configuredAxisMask();
	const short sRtn = allMask != 0 ? GTN_Stop(m_iCore, allMask, 0x0) : 0;
	if (sRtn != 0)
	{
		LogError("StopMotion", "GTN_Stop","", sRtn);
		return false;
	}
	const bool axesStopped = waitForStoppedMotion(allMask, false, "StopMotion");
	if (!groupSuccess || !axesStopped)
		return false;
	if (!clearCoordinateBufferIfInitialized("StopMotion"))
		return false;
	if (!synchronizeAxisProfilesToEncoders(allMask, "StopMotion"))
		return false;
	m_bStop = false;
	LCNC_INFO(lcnc::LogCode::Generic,
		"gtn.api: operation=StopMotion api=GTN_Stop+conditional_GTN_CrdClear mask=0x{:x} fifo={} result=0",
		allMask, m_iWriteBuf);
	completed = true;
	return true;
}

bool GTNMotionControl::StopMotion(Axis eAxis)
{
	// A coordinated Group is one motion owner. Stopping one member must not
	// rebase it while its CommandList can still execute other group segments.
	if (m_groupReady)
		return StopMotion();
	bool completed = false;
	const auto preserveStopFailure = qScopeGuard([this, &completed] {
		if (!completed) {
			m_stopFaultLatched = true;
			m_bErrorOccurred = true;
		}
	});
	short sRtn;
	long mask = AxisMask(eAxis);
	if (!mask)
		return LogError("StopMotion", "AxisMask", magic_enum::enum_name(eAxis).data(), -1), false;
	sRtn = GTN_Stop(m_iCore, mask, mask);//急停
	if (sRtn != 0)
		return LogError("StopMotion", "GTN_Stop", magic_enum::enum_name(eAxis).data(), sRtn),false;
	if (!waitForStoppedMotion(mask, false, "StopMotionAxis"))
		return false;
	if (!clearCoordinateBufferIfInitialized("StopMotionAxis"))
		return false;
	if (!synchronizeAxisProfilesToEncoders(mask, "StopMotionAxis"))
		return false;
	completed = true;
	return true;
}

bool GTNMotionControl::IsAxisMoving()
{
	for (Axis axis : m_vecMotors)
	{
		if (IsAxisMoving(axis))
			return true;
	}
	return false;
}

bool GTNMotionControl::IsAxisMoving(Axis eAxis)
{
	long State;
	short sRtn;
	sRtn = GTN_GetSts(m_iCore, m_mapMotorValue[eAxis].AxisIndex, &State);

	if (sRtn != 0) {
		LogError("IsAxisMoving", "GTN_GetSts", magic_enum::enum_name(eAxis).data(), sRtn);
		m_bErrorOccurred = true;
		// Compatibility bool consumers must not interpret an unreadable status
		// as motion completed. Stop itself uses the explicit three-state read.
		return true;
	}
	if (State & 0x400)
		return true;
	else
		return false;
}

bool GTNMotionControl::GetActualPos(Axis eAxis, double& dAPos)
{
	if (!IsConnected())
	{
		dAPos = -1;
		return false;
	}
	short sRtn;
	double APos;
	int iAxis = m_mapMotorValue[eAxis].AxisIndex;
	sRtn = GTN_GetEncPos(m_iCore, iAxis, &APos);//编码器实际位置
	if (sRtn != 0)
	{
		dAPos = -1;
		//LogError("GetActualPos", "GTN_GetEncPos", magic_enum::enum_name(eAxis).data(), sRtn);
		return false;
	}
	PulseToMillimeter(eAxis, APos, dAPos);
	return true;
}

bool GTNMotionControl::GetFeedbackPos(Axis eAxis, double& dFPos)
{
	if (!IsConnected())
	{
		dFPos = -1;
		return false;
	}
	short sRtn;
	double APos;
	int iAxis = m_mapMotorValue[eAxis].AxisIndex;
	sRtn = GTN_GetEncPos(m_iCore, iAxis, &APos); // 编码器反馈位置
	if (sRtn != 0)
	{
		dFPos = -1;
		return false;
	}
	PulseToMillimeter(eAxis, APos, dFPos);
	return true;
}

bool GTNMotionControl::SetFPosition(Axis eAxis, double dPos)
{
	if (!requireInitializedConnection("SetFPosition"))
		return false;
	if (!IsConnected())
		return false;
	int iAxis = m_mapMotorValue[eAxis].AxisIndex;
	long mask = AxisMaskByIndex(iAxis);
	if (!mask)
		return LogError("SetFPosition", "AxisMaskByIndex", magic_enum::enum_name(eAxis).data(), -1), false;
	double dPulse = 0.0;
	if (!MillimeterToPulse(eAxis, dPos, dPulse))
		return false;
	// 对应 ACS setfpos：同步重写编码器反馈位置与规划位置，
	// 二者不一致会触发跟随误差报警，故必须同时设置并同步轴位置。
	short sRtn = GTN_SetEncPos(m_iCore, iAxis, (long)dPulse);
	if (0 != sRtn)
		return LogError("SetFPosition", "GTN_SetEncPos", magic_enum::enum_name(eAxis).data(), sRtn), false;
	sRtn = GTN_SetPrfPos(m_iCore, iAxis, (long)dPulse);
	if (0 != sRtn)
		return LogError("SetFPosition", "GTN_SetPrfPos", magic_enum::enum_name(eAxis).data(), sRtn), false;
	sRtn = GTN_SynchAxisPos(m_iCore, mask);
	if (0 != sRtn)
		return LogError("SetFPosition", "GTN_SynchAxisPos", magic_enum::enum_name(eAxis).data(), sRtn), false;
	LCNC_INFO(lcnc::LogCode::Generic,
		"gtn.api: operation=SetFPosition api=GTN_SetEncPos+GTN_SetPrfPos+GTN_SynchAxisPos axis={} position={} pulse={} mask=0x{:x} result=0",
		magic_enum::enum_name(eAxis), dPos, dPulse, mask);
	return true;
}

void GTNMotionControl::SetAxisHomePrm(Axis eAxis, const toml::table& tableHome)
{
	if (!m_mapMotorValue[eAxis].pTHomePrm)
		m_mapMotorValue[eAxis].pTHomePrm = new THomePrm();
	THomePrm* p = m_mapMotorValue[eAxis].pTHomePrm;
	memset(p, 0, sizeof(THomePrm));
	p->triggerIndex = -1;	// 默认使用本轴触发器

	if (tableHome.count("iMode"))
		p->mode				= (short)tableHome.at("iMode").as_integer();
	if (tableHome.count("iMoveDir"))
		p->moveDir			= (short)tableHome.at("iMoveDir").as_integer();
	if (tableHome.count("iIndexDir"))
		p->indexDir			= (short)tableHome.at("iIndexDir").as_integer();
	if (tableHome.count("iEdge"))
		p->edge				= (short)tableHome.at("iEdge").as_integer();
	if (tableHome.count("iTriggerIndex"))
		p->triggerIndex		= (short)tableHome.at("iTriggerIndex").as_integer();
	if (tableHome.count("fVelHigh"))
		p->velHigh			= tableHome.at("fVelHigh").as_floating();
	if (tableHome.count("fVelLow"))
		p->velLow			= tableHome.at("fVelLow").as_floating();
	if (tableHome.count("fAcc"))
		p->acc				= tableHome.at("fAcc").as_floating();
	if (tableHome.count("fDec"))
		p->dec				= tableHome.at("fDec").as_floating();
	if (tableHome.count("iSmoothTime"))
		p->smoothTime		= (short)tableHome.at("iSmoothTime").as_integer();
	if (tableHome.count("iHomeOffset"))
		p->homeOffset		= (long)tableHome.at("iHomeOffset").as_integer();
	if (tableHome.count("iSearchHomeDistance"))
		p->searchHomeDistance	= (long)tableHome.at("iSearchHomeDistance").as_integer();
	if (tableHome.count("iSearchIndexDistance"))
		p->searchIndexDistance	= (long)tableHome.at("iSearchIndexDistance").as_integer();
	if (tableHome.count("iEscapeStep"))
		p->escapeStep		= (long)tableHome.at("iEscapeStep").as_integer();
}

bool GTNMotionControl::GetAxisHomePrm(Axis eAxis, toml::table& tableHome)
{
	if (!m_mapMotorValue.count(eAxis))
		return false;

	THomePrm tHomePrm;
	memset(&tHomePrm, 0, sizeof(THomePrm));

	if (IsConnected())
	{
		short sRtn = GTN_GetHomePrm(m_iCore, m_mapMotorValue[eAxis].AxisIndex, &tHomePrm);
		if (sRtn != 0)
		{
			LogError("GetAxisHomePrm", "GTN_GetHomePrm", magic_enum::enum_name(eAxis).data(), sRtn);
			return false;
		}
		// 同步更新内存中存储的副本
		if (!m_mapMotorValue[eAxis].pTHomePrm)
			m_mapMotorValue[eAxis].pTHomePrm = new THomePrm();
		*m_mapMotorValue[eAxis].pTHomePrm = tHomePrm;
	}
	else if (m_mapMotorValue[eAxis].pTHomePrm)
	{
		tHomePrm = *m_mapMotorValue[eAxis].pTHomePrm;
	}
	else
	{
		return false;	// 未连接且无缓存参数
	}

	tableHome["iMode"]					= (int)tHomePrm.mode;
	tableHome["iMoveDir"]				= (int)tHomePrm.moveDir;
	tableHome["iIndexDir"]				= (int)tHomePrm.indexDir;
	tableHome["iEdge"]					= (int)tHomePrm.edge;
	tableHome["iTriggerIndex"]			= (int)tHomePrm.triggerIndex;
	tableHome["fVelHigh"]				= tHomePrm.velHigh;
	tableHome["fVelLow"]				= tHomePrm.velLow;
	tableHome["fAcc"]					= tHomePrm.acc;
	tableHome["fDec"]					= tHomePrm.dec;
	tableHome["iSmoothTime"]			= (int)tHomePrm.smoothTime;
	tableHome["iHomeOffset"]			= (int)tHomePrm.homeOffset;
	tableHome["iSearchHomeDistance"]	= (int)tHomePrm.searchHomeDistance;
	tableHome["iSearchIndexDistance"]	= (int)tHomePrm.searchIndexDistance;
	tableHome["iEscapeStep"]			= (int)tHomePrm.escapeStep;
	return true;
}

bool GTNMotionControl::SetAxisIndex(Axis eAxis, int iIndex)
{
	m_mapMotorValue[eAxis].AxisIndex = iIndex;
	return true;
}

bool GTNMotionControl::SetAxisIsRotary(Axis eAxis, bool bRotary)
{
	m_mapMotorValue[eAxis].Rotary = bRotary;
	return true;
}

bool GTNMotionControl::SetAxisResolution(Axis eAxis, int iResolution)
{
	m_mapMotorValue[eAxis].Resolution = iResolution;
	return true;
}

bool GTNMotionControl::SetAxisTubeDiamater(Axis eAxis, double dDiamater)
{
	if (dDiamater <= 0 || !m_mapMotorValue[eAxis].Rotary)
	{
		return false;
	}

	int iResolutionRatio = m_mapMotorValue[eAxis].Resolution;
	
	return false;
}

bool GTNMotionControl::SetAxisVel(Axis eAxis, double dVel)
{
	if (!IsConnected())
		return false;
	
	double dNewVel;
	MillimeterToPulse(eAxis, dVel/1000, dNewVel);
	short sRtn = GTN_SetVel(m_iCore, m_mapMotorValue[eAxis].AxisIndex, dNewVel);
	if (sRtn != 0)
	{
		return LogError("SetAxisVel", "GTN_SetVel", magic_enum::enum_name(eAxis).data(), sRtn),false;
	}
	m_mapMotorValue[eAxis].Velocity = dVel;
	LCNC_INFO(lcnc::LogCode::Generic,
		"gtn.api: operation=SetAxisVelocity api=GTN_SetVel axis={} velocity={} pulse_per_ms={} result=0",
		magic_enum::enum_name(eAxis), dVel, dNewVel);
	return true;
}

bool GTNMotionControl::SetAxisAcc(Axis eAxis, double dAcc)
{
	if (!IsConnected())
		return false;
	short sRtn;
	double dNewAcc, dNewDec;

	// 将 AXIS 轴设为点位模式
	sRtn = GTN_PrfTrap(m_iCore, m_mapMotorValue[eAxis].AxisIndex);
	if (0 != sRtn)
	{
		return LogError("SetAxisAcc", "GTN_PrfTrap", magic_enum::enum_name(eAxis).data(), sRtn), false;
	}
	TTrapPrm trap;
	MillimeterToPulse(eAxis, dAcc / 1000000.00, dNewAcc);
	sRtn = GTN_GetTrapPrm(m_iCore, m_mapMotorValue[eAxis].AxisIndex, &trap);
	if (0 != sRtn)
		return LogError("SetAxisAcc", "GTN_GetTrapPrm", magic_enum::enum_name(eAxis).data(), sRtn), false;
	trap.acc = dNewAcc;
	sRtn = GTN_SetTrapPrm(m_iCore, m_mapMotorValue[eAxis].AxisIndex, &trap);
	if (0!= sRtn)
	{
		return LogError("SetAxisAcc", "GTN_SetTrapPrm", magic_enum::enum_name(eAxis).data(), sRtn),false;
	}
	m_mapMotorValue[eAxis].Acceleration = dAcc;
	LCNC_INFO(lcnc::LogCode::Generic,
		"gtn.api: operation=SetAxisAcceleration api=GTN_SetTrapPrm axis={} acceleration={} pulse_per_ms2={} result=0",
		magic_enum::enum_name(eAxis), dAcc, dNewAcc);
	return true;
}

bool GTNMotionControl::SetAxisDec(Axis eAxis, double dDec)
{
	if (!IsConnected())
		return false;
	short sRtn;
	double dNewAcc,dNewDec;
	TTrapPrm trap;
	MillimeterToPulse(eAxis, dDec / 1000000.00, dNewDec);
	sRtn = GTN_GetTrapPrm(m_iCore, m_mapMotorValue[eAxis].AxisIndex, &trap);
	if (0 != sRtn)
		return LogError("SetAxisDec", "GTN_GetTrapPrm", magic_enum::enum_name(eAxis).data(), sRtn), false;
	trap.dec = dNewDec;
	sRtn = GTN_SetTrapPrm(m_iCore, m_mapMotorValue[eAxis].AxisIndex, &trap);
	if (0 != sRtn)
	{
		return LogError("SetAxisDec", "GTN_SetTrapPrm", magic_enum::enum_name(eAxis).data(), sRtn), false;
	}
	m_mapMotorValue[eAxis].Deceleration = dDec;
	LCNC_INFO(lcnc::LogCode::Generic,
		"gtn.api: operation=SetAxisDeceleration api=GTN_SetTrapPrm axis={} deceleration={} pulse_per_ms2={} result=0",
		magic_enum::enum_name(eAxis), dDec, dNewDec);
	return true;
}

bool GTNMotionControl::SetAxisJerk(Axis eAxis, double dSmoothTime)
{
	if (!IsConnected())
		return false;
	if (!validTrapSmoothTime(dSmoothTime))
		return LogError("SetAxisSmoothTime", "validate range [0,50] ms",
			magic_enum::enum_name(eAxis).data(), -1), false;
	short sRtn;
	TTrapPrm trap;
	sRtn = GTN_GetTrapPrm(m_iCore, m_mapMotorValue[eAxis].AxisIndex, &trap);
	if (0 != sRtn)
		return LogError("SetAxisSmoothTime", "GTN_GetTrapPrm", magic_enum::enum_name(eAxis).data(), sRtn), false;
	trap.smoothTime = static_cast<short>(dSmoothTime);
	sRtn = GTN_SetTrapPrm(m_iCore, m_mapMotorValue[eAxis].AxisIndex, &trap);
	if (0 != sRtn)
	{
		return LogError("SetAxisSmoothTime", "GTN_SetTrapPrm", magic_enum::enum_name(eAxis).data(), sRtn), false;
	}
	m_mapMotorValue[eAxis].SmoothTime = dSmoothTime;
	LCNC_INFO(lcnc::LogCode::Generic,
		"gtn.api: operation=SetAxisSmoothTime api=GTN_SetTrapPrm axis={} smooth_time_ms={} result=0",
		magic_enum::enum_name(eAxis), dSmoothTime);
	return true;
}

bool GTNMotionControl::SetAxisJogSmooth(Axis eAxis, double smooth)
{
	if (!validJogSmooth(smooth))
		return LogError("SetAxisJogSmooth", "validate range [0,1)",
			magic_enum::enum_name(eAxis).data(), -1), false;
	m_mapMotorValue[eAxis].JogSmooth = smooth;
	LCNC_INFO(lcnc::LogCode::Generic,
		"gtn.api: operation=SetAxisJogSmooth api=cache axis={} smooth={} result=0",
		magic_enum::enum_name(eAxis), smooth);
	return true;
}

bool GTNMotionControl::SetAxisNegLimit(Axis eAxis, double dNegLimit)
{
	if (!IsConnected()/* || !IsHomed(eAxis)*/)
	{
		return false;
	}
	short sRtn;
	double dNewNegLimit;
	MillimeterToPulse(eAxis, dNegLimit, dNewNegLimit);
	long lPosLimit, lNegLimit;
	sRtn = GTN_GetSoftLimit(m_iCore, m_mapMotorValue[eAxis].AxisIndex, &lPosLimit, &lNegLimit);
	if (0 != sRtn)
	{
		LogError("SetAxisNegLimit", "GTN_GetSoftLimit", magic_enum::enum_name(eAxis).data(), sRtn);
		return false;
	}
	lNegLimit = (long)dNewNegLimit;
	sRtn = GTN_SetSoftLimit(m_iCore, m_mapMotorValue[eAxis].AxisIndex, lPosLimit, lNegLimit);
	if (0 != sRtn)
	{
		LogError("SetAxisNegLimit", "GTN_SetSoftLimit", magic_enum::enum_name(eAxis).data(), sRtn);
		return false;
	}
	m_mapMotorValue[eAxis].NegLimit = dNegLimit;
	return true;
}

bool GTNMotionControl::SetAxisPosLimit(Axis eAxis, double dPosLimit)
{
	if (!IsConnected()/* || !IsHomed(eAxis)*/)
	{
		return false;
	}
	short sRtn;
	double dNewPosLimit;
	MillimeterToPulse(eAxis, dPosLimit, dNewPosLimit);
	long lPosLimit, lNegLimit;
	sRtn = GTN_GetSoftLimit(m_iCore, m_mapMotorValue[eAxis].AxisIndex, &lPosLimit, &lNegLimit);
	if (0 != sRtn)
	{
		LogError("SetAxisNegLimit", "GTN_GetSoftLimit", magic_enum::enum_name(eAxis).data(), sRtn);
		return false;
	}
	lPosLimit = (long)dNewPosLimit;
	sRtn = GTN_SetSoftLimit(m_iCore, m_mapMotorValue[eAxis].AxisIndex, lPosLimit, lNegLimit);
	if (0 != sRtn)
	{
		LogError("SetAxisNegLimit", "GTN_SetSoftLimit", magic_enum::enum_name(eAxis).data(), sRtn);
		return false;
	}
	m_mapMotorValue[eAxis].PosLimit = dPosLimit;
	return true;
}

bool GTNMotionControl::SetAxisVelAccDecJerk(Axis eAxis, double dVel, double dAcc, double dDec, double dJerk)
{
	if (!validTrapSmoothTime(dJerk))
		return LogError("SetAxisMotionParameters", "validate Trap smooth range [0,50] ms",
			magic_enum::enum_name(eAxis).data(), -1), false;
	if (!SetAxisVel(eAxis, dVel))
		return false;

	short sRtn = GTN_PrfTrap(m_iCore, m_mapMotorValue[eAxis].AxisIndex);
	if (0 != sRtn)
	{
		return LogError("SetAxisAcc", "GTN_PrfTrap", magic_enum::enum_name(eAxis).data(), sRtn), false;
	}

	TTrapPrm trap;
	sRtn = GTN_GetTrapPrm(m_iCore, m_mapMotorValue[eAxis].AxisIndex, &trap);
	if (0 != sRtn)
		return LogError("SetAxisAcc", "GTN_GetTrapPrm", magic_enum::enum_name(eAxis).data(), sRtn), false;
	MillimeterToPulse(eAxis, dAcc / 1000000.00, trap.acc);
	MillimeterToPulse(eAxis, dDec / 1000000.00, trap.dec);
	trap.smoothTime = static_cast<short>(dJerk);
	sRtn = GTN_SetTrapPrm(m_iCore, m_mapMotorValue[eAxis].AxisIndex, &trap);
	if (0 != sRtn)
	{
		return LogError("SetAxisAcc", "GTN_SetTrapPrm", magic_enum::enum_name(eAxis).data(), sRtn), false;
	}
	LCNC_INFO(lcnc::LogCode::Generic,
		"gtn.api: operation=SetAxisMotionParameters api=GTN_SetTrapPrm axis={} velocity={} acceleration={} deceleration={} smooth_time_ms={} result=0",
		magic_enum::enum_name(eAxis), dVel, dAcc, dDec, dJerk);
	return true;
}

bool GTNMotionControl::SetAxisSoftLimit(Axis eAxis, double dNegLimit, double dPosLimit)
{
	if (!IsConnected() || !std::isfinite(dNegLimit) || !std::isfinite(dPosLimit)
		|| dNegLimit > dPosLimit)
		return LogError("SetAxisSoftLimit", "validate limits",
			magic_enum::enum_name(eAxis).data(), -1), false;
	double dNegL, dPosL;
	MillimeterToPulse(eAxis, dNegLimit, dNegL);
	MillimeterToPulse(eAxis, dPosLimit, dPosL);

	short sRtn = GTN_SetSoftLimit(m_iCore, m_mapMotorValue[eAxis].AxisIndex, (long)dPosL, (long)dNegL);
	if (0 != sRtn)
		return LogError("SetAxisSoftLimit", "GTN_SetSoftLimit", magic_enum::enum_name(eAxis).data(), sRtn), false;

	m_mapMotorValue[eAxis].NegLimit = dNegLimit;
	m_mapMotorValue[eAxis].PosLimit = dPosLimit;
	LCNC_INFO(lcnc::LogCode::Generic,
		"gtn.api: operation=SetAxisSoftLimit api=GTN_SetSoftLimit axis={} limits=[{},{}] pulse_limits=[{},{}] result=0",
		magic_enum::enum_name(eAxis), dNegLimit, dPosLimit, dNegL, dPosL);
	return true;
}

bool GTNMotionControl::GetAxisIndex(Axis eAxis, int& iIndex)
{
	iIndex = m_mapMotorValue[eAxis].AxisIndex;
	return true;
}

bool GTNMotionControl::GetAxisIsRotary(Axis eAxis, bool& bRotary)
{
	bRotary = m_mapMotorValue[eAxis].Rotary;
	return true;
}

bool GTNMotionControl::GetAxisResolution(Axis eAxis, int& iResolution)
{
	iResolution = m_mapMotorValue[eAxis].Resolution;
	return true;
}

bool GTNMotionControl::GetAxisTubeDiamater(Axis eAxis, double& dTubeDiamater)
{
	int iEfac = 0;
// 	if (!ReadEFAC(eAxis, iEfac))
// 		return false;
// 
// 	int iResolutionRatio = m_mapMotorValue[eAxis].Resolution;
// 	dTubeDiamater = iResolutionRatio * 10000 / (iEfac * lcnc::process::kPi);
// 	m_dDiameter = dTubeDiamater;
	return true;
}

bool GTNMotionControl::GetAxisVel(Axis eAxis, double& dVel)
{
	double Vel;
	TrapPrm trap;
	short sRtn;
	int iAxis = m_mapMotorValue[eAxis].AxisIndex;
	sRtn = GTN_GetVel(m_iCore, iAxis, &Vel);
	if (0!= sRtn)
	{
		return LogError("GetAxisVel", "GTN_GetVel", magic_enum::enum_name(eAxis).data(), sRtn),false;
	}
	PulseToMillimeter(eAxis, Vel, dVel);
	dVel = dVel * 1000;  // pulse/ms → mm/ms → mm/s
	return true;
}

bool GTNMotionControl::GetAxisAcc(Axis eAxis, double& dAcc)
{
	TrapPrm trap;
	short sRtn;
	int iAxis = m_mapMotorValue[eAxis].AxisIndex;
	sRtn = GTN_GetTrapPrm(m_iCore, iAxis, &trap);
	if (0 != sRtn)
	{
		return LogError("GetAxisAcc", "GTN_GetTrapPrm", magic_enum::enum_name(eAxis).data(), sRtn), false;
	}
	PulseToMillimeter(eAxis, trap.acc, dAcc);
	dAcc = dAcc * 1000000;
	return true;
}

bool GTNMotionControl::GetAxisDec(Axis eAxis, double& dDec)
{
	TrapPrm trap;
	short sRtn;
	int iAxis = m_mapMotorValue[eAxis].AxisIndex;
	sRtn = GTN_GetTrapPrm(m_iCore, iAxis, &trap);
	if (0 != sRtn)
	{
		return LogError("GetAxisDec", "GTN_GetTrapPrm", magic_enum::enum_name(eAxis).data(), sRtn), false;
	}
	PulseToMillimeter(eAxis, trap.dec, dDec);
	dDec = dDec*1000000;
	return true;
}

bool GTNMotionControl::GetAxisJerk(Axis eAxis, double& dSmoothTime)
{
	double Vel;
	TrapPrm trap;
	short sRtn;
	int iAxis = m_mapMotorValue[eAxis].AxisIndex;
	sRtn = GTN_GetTrapPrm(m_iCore, iAxis, &trap);
	if (0 != sRtn)
	{
		return LogError("GetAxisJerk", "GTN_GetTrapPrm", magic_enum::enum_name(eAxis).data(), sRtn), false;
	}
	dSmoothTime = trap.smoothTime;
	return true;
}

bool GTNMotionControl::GetAxisNegLimit(Axis eAxis, double& dNegLimit)
{
	long LLimit;
	long Rlimit;
	short sRtn;
	int iAxis = m_mapMotorValue[eAxis].AxisIndex;
	sRtn = GTN_GetSoftLimit(m_iCore, iAxis, &Rlimit, &LLimit);
	if (0 != sRtn)
	{
		return LogError("GetAxisNegLimit", "GTN_GetSoftLimit", magic_enum::enum_name(eAxis).data(), sRtn), false;
	}
	PulseToMillimeter(eAxis, LLimit, dNegLimit);
	
	return true;
}

bool GTNMotionControl::GetAxisPosLimit(Axis eAxis, double& dPosLimit)
{
	long LLimit;
	long Rlimit;
	short sRtn;
	int iAxis = m_mapMotorValue[eAxis].AxisIndex;
	sRtn = GTN_GetSoftLimit(m_iCore, iAxis, &Rlimit, &LLimit);
	if (0 != sRtn)
	{
		return LogError("GetAxisPosLimit", "GTN_GetSoftLimit", magic_enum::enum_name(eAxis).data(), sRtn), false;
	}
	PulseToMillimeter(eAxis, Rlimit, dPosLimit);
	
	return true;
}

bool GTNMotionControl::GetAxisVelAccDecJerk(Axis eAxis, double& dVel, double& dAcc, double& dDec, double& dJerk)
{
	int iResult = 0;
	if (!GetAxisVel(eAxis, dVel))
		iResult++;
	if (!GetAxisAcc(eAxis, dAcc))
		iResult++;
	if (!GetAxisDec(eAxis, dDec))
		iResult++;
	if (!GetAxisJerk(eAxis, dJerk))
		iResult++;
	return (bool)!iResult;
}

bool GTNMotionControl::GetAxisSoftLimit(Axis eAxis, double& dNegLimit, double& dPosLimit)
{
	int iResult = 0;
	if (!GetAxisNegLimit(eAxis, dNegLimit))
		iResult++;
	if (!GetAxisPosLimit(eAxis, dPosLimit))
		iResult++;
	return (bool)!iResult;
}

void GTNMotionControl::ReadAxisSoftLimit(Axis eAxis, double& dNegLimit, double& dPosLimit)
{
	dNegLimit = m_mapMotorValue[eAxis].NegLimit;
	dPosLimit = m_mapMotorValue[eAxis].PosLimit;
}

bool GTNMotionControl::DigitalOutputSet(DigitalIOData& IOData, int iValue, bool bLogError)
{
	if (iValue != 0 && !requireInitializedConnection("DigitalOutputSet"))
		return false;
	//注意固高卡的ID从1开始，作为程序员痛恨此行为
	if (!IsConnected())
	{
		return false;
	}
	if (!IOData.iIO)
	{
		if (bLogError)
			LogError("DigitalOutputSet", "IOData.iIO = 0", IOData.qstrID.toStdString().data(), 0);
		return false;
	}
	short sRtn;
	int Value = iValue ? 0 : 1;	// 固高非扩展DO低电平有效，第一次取反做硬件校正
	
	if (IOData.bInversion)
		Value = Value ? 0 : 1;	// UI配置的逻辑翻转，在已校正值基础上再取反

	if (!IOData.bExpand)
	{
		if (IOData.iIO > 16 || IOData.iIO <= 0)
		{
			return true;
		}
		sRtn = GTN_SetDoBit(m_iCore, MC_GPO, IOData.iIO, Value);
		if (0 != sRtn)
			return LogError("DigitalOutputSet", "GTN_SetDoBit", IOData.qstrID.toStdString().data(), sRtn), false;
		
	}
	else
	{
		if (IOData.iIO > 12)
		{
			return true;
		}
// 		sRtn = GTN_ExtModuleInit(m_iCore);
// 		if (0 != sRtn)
// 			return LogError("DigitalOutputSet", "GTN_ExtModuleInit", IOData.qstrID.toStdString().data(), sRtn), false;
		Value = iValue;
		if (IOData.bInversion)
			Value = iValue ? 0 : 1;
		sRtn = GTN_SetExtDoBit(m_iCore, IOData.iIO, Value);		////按位设置核1的扩展模块第iIO路DO为iValue
		if (0 != sRtn)
			return LogError("DigitalOutputSet", "GTN_SetExtDoBit", IOData.qstrID.toStdString().data(), sRtn), false;
	}
	return true;
}

bool GTNMotionControl::DigitalOutputGet(DigitalIOData& IOData, int& iValue, bool bLogError)
{
	if (!IsConnected())
	{
		return false;
	}
	short sRtn;
	if (!IOData.iIO)
	{
		if (bLogError)
			LogError("DigitalOutputGet", "IOData.iIO = 0", IOData.qstrID.toStdString().data(), 0);
		return false; 
	}
	if (!IOData.bExpand)
	{
		short* pValue = new short();
		sRtn = GTN_ReadDigitalOutput(m_iCore, MC_GPO, IOData.iIO, pValue);
		if (0 != sRtn)
		{
			if (bLogError)
				LogError("DigitalOutputGet", "GTN_ReadDigitalOutput", IOData.qstrID.toStdString().data(), sRtn);
			
			delete pValue;
			pValue = nullptr;
			return false;
		}
		iValue = *pValue;
		delete pValue;
		pValue = nullptr;
		iValue = iValue ? 0 : 1;
		if (IOData.bInversion)
			iValue = iValue ? 0 : 1;
	}
	else
	{
		short* pValue = new short();
		sRtn = GTN_GetExtDoBit(m_iCore, IOData.iIO, pValue);
		
		if (0 != sRtn)
		{
			if (bLogError)
				LogError("DigitalOutputGet", "GTN_GetExtDoBit", IOData.qstrID.toStdString().data(), sRtn);
			
			delete pValue;
			pValue = nullptr;
			return false;
		}
		iValue = *pValue;
		if (IOData.bInversion)
			iValue = iValue ? 0 : 1;
	}
	
	return true;
}

bool GTNMotionControl::DigitalInputGet(DigitalIOData& IOData, int& iValue, bool bLogError)
{
	if (!IsConnected())
	{
		return false;
	}
	short sRtn;
	if (!IOData.bExpand)
	{
		if (!IOData.iIO)
		{
			if (bLogError)
				LogError("DigitalInputGet", "IOData.iIO = 0", IOData.qstrID.toStdString().data(), 0);
			return false;
		}
			
		short* pValue = new short();	
		sRtn = GTN_GetDiBit(m_iCore, MC_GPI, IOData.iIO, pValue);
		
		if (sRtn)
		{
			if (bLogError)
				LogError("DigitalInputGet", "GTN_GetDiBit", IOData.qstrID.toStdString().data(), sRtn);
			delete pValue;
			pValue = nullptr;
			return false;
		}
		iValue = *pValue;
		//搞个取反操作，固高的是低电平有效
		delete pValue;
		pValue = nullptr;
		iValue = iValue ? 0 : 1;
		if (IOData.bInversion)
			iValue = iValue ? 0 : 1;
	}
	else
	{
		if (!IOData.iIO && bLogError)
		{
			if (bLogError)
				LogError("DigitalInputGet", "IOData.iIO = 0", IOData.qstrID.toStdString().data(), 0);
			return  false;
		}
		short* pValue = new short();
		sRtn = GTN_GetExtDiBit(m_iCore, IOData.iIO, pValue);
		
		if (sRtn)
		{
			if (bLogError)
				LogError("DigitalInputGet", "GTN_GetExtDiBit", IOData.qstrID.toStdString().data(), sRtn);
			
			delete pValue;
			pValue = nullptr;
			return false;
		}
		iValue = *pValue;
		//拓展IO就不取反了，因为是高电平有效
		delete pValue;
		pValue = nullptr;
		if (IOData.bInversion)
			iValue = iValue ? 0 : 1;
	}
	return true;
}

bool GTNMotionControl::AnalogOutputSet(AnalogIOData& IOData, double dValue, bool bLogError)
{
	if (dValue != 0.0 && !requireInitializedConnection("AnalogOutputSet"))
		return false;
	//暂定编写，需了解如何使用拓展和非拓展
	if (!IsConnected())
	{
		return false;
	}
	short sRtn;
	if (!IOData.iPort)
	{
		if (bLogError)
			LogError("AnalogOutputSet", "IOData.iPort = 0", IOData.qstrID.toStdString().data(), 0);
		return false;
	}
		
	if (!IOData.bExpand)
	{
		short pValue = (short)dValue;
		sRtn = GTN_SetDac(m_iCore, IOData.iPort, &pValue);
		
		if (sRtn)
		{
			if (bLogError)
				LogError("AnalogOutputSet", "GTN_SetDac", IOData.qstrID.toStdString().data(), sRtn);
			return false;
		}
	}
	else
	{
		short pValue = (short)dValue;
		sRtn = GTN_SetAuDac(m_iCore, IOData.iPort, &pValue);
		if (sRtn)
		{
			if (bLogError)
				LogError("AnalogOutputSet", "GTN_SetAuDac", IOData.qstrID.toStdString().data(), sRtn);
			
			return false;
		}
		
	}
	return true;
}

bool GTNMotionControl::AnalogOutputGet(AnalogIOData& IOData, double& dValue, bool bLogError)
{
	//暂定编写，需了解如何使用拓展和非拓展
	if (!IsConnected())
	{
		return false;
	}
	short sRtn;
	if (!IOData.iPort )
	{
		if (bLogError)
			LogError("AnalogOutputGet", "IOData.iPort = 0", IOData.qstrID.toStdString().data(), 0);
		return  false; 
	}
	if (!IOData.bExpand)
	{
		short* pValue = new short();
		sRtn = GTN_GetDac(m_iCore, IOData.iPort, pValue);
		if (sRtn != 0)
		{
			if (bLogError)
				LogError("AnalogOutputGet", "GTN_GetDac", IOData.qstrID.toStdString().data(), sRtn);
			delete pValue;
			pValue = nullptr;
			return false;
		}
		dValue = *pValue;
		delete pValue;
		pValue = nullptr;
		return true;
	}
	else
	{
		short* pValue = new short();
		sRtn = GTN_GetAuDac(m_iCore, IOData.iPort, pValue, 1);
		if (sRtn)
		{
			if (bLogError)
				LogError("AnalogOutputGet", "GTN_SetAuDac", IOData.qstrID.toStdString().data(), sRtn);
			
			delete pValue;
			pValue = nullptr;
			return false;
		}
		dValue = *pValue;
		delete pValue;
		pValue = nullptr;
		return true;
	}
}

bool GTNMotionControl::AnalogInputGet(AnalogIOData& IOData, double& dValue, bool bLogError)
{
	//暂定编写，需了解如何使用拓展和非拓展
	if (!IsConnected())
	{
		return false;
	}
	short sRtn;
	if (!IOData.iPort)
	{
		if (bLogError)
			LogError("AnalogInputGet", "IOData.iPort = 0", IOData.qstrID.toStdString().data(), 0);
		return false;
	}
	if (!IOData.bExpand)
	{
		double* pValue = new double();
		sRtn = GTN_GetAdc(m_iCore, IOData.iPort, pValue);
		if (sRtn != 0)
		{
			if (bLogError)
				LogError("AnalogInputGet", "GTN_GetDac", IOData.qstrID.toStdString().data(), sRtn);
			delete pValue;
			pValue = nullptr;
			return false;
		}
		dValue = *pValue;
		delete pValue;
		pValue = nullptr;
		return true;
	}
	else
	{
		double* pValue = new double();
		sRtn = GTN_GetAuAdc(m_iCore, IOData.iPort, pValue);
		if (sRtn != 0)
		{
			if (bLogError)
				LogError("AnalogInputGet", "GTN_GetDac", IOData.qstrID.toStdString().data(), sRtn);
			delete pValue;
			pValue = nullptr;
			return false;
		}
		dValue = *pValue;
		delete pValue;
		pValue = nullptr;
		return true;
	}
}

bool GTNMotionControl::SetShutterOnOffWaitTime(double dBeforeOn, double dAfterOn, double dBeforeOff, double dAfterOff, double dBlowDelay)
{
	m_dLaserOnBWait = dBeforeOn;
	m_dLaserOnAWait = dAfterOn;
	m_dLaserOffBWait = dBeforeOff;
	m_dLaserOffAWait = dAfterOff;
	m_dBlowDelay = dBlowDelay;
	return true;
}

bool GTNMotionControl::StopAllBuffer()
{
	bool completed = false;
	const auto preserveStopFailure = qScopeGuard([this, &completed] {
		if (!completed) {
			m_stopFaultLatched = true;
			m_bErrorOccurred = true;
		}
	});
	const long allMask = configuredAxisMask();
	// The runtime follows StopMotion with StopAllBuffer in one safety attempt.
	// If that first step failed, this second attempt may stop remaining motion
	// but must preserve its Group/list evidence. A later explicit StopMotion or
	// Disconnect uses the normal confirmed-release path and can recover it.
	const bool preserveGroupEvidence = m_stopFaultLatched && m_groupReady;
	const bool groupSuccess = preserveGroupEvidence
		? stopAndReleaseFiveAxisGroup(allMask, "StopAllBufferAfterFailure", false)
		: StopFiveAxisGroupProgram();
	const short sRtn = allMask != 0 ? GTN_Stop(m_iCore, allMask, 0x0) : 0;
	if (sRtn)
		return LogError("StopAllBuffer", "GTN_Stop", "", sRtn), false;
	const bool axesStopped = waitForStoppedMotion(allMask, false, "StopAllBuffer");
	if (!groupSuccess || !axesStopped)
		return false;
	if (preserveGroupEvidence) {
		LCNC_WARN(lcnc::LogCode::Generic,
			"gtn.api: operation=StopAllBuffer phase=complete action=preserve_buffers reason=earlier_stop_failed stop_confirmed=true cleanup=false preserve_evidence=true result=0");
		completed = true;
		return true;
	}
	completed = clearCoordinateBufferIfInitialized("StopAllBuffer");
	return completed;
}

bool GTNMotionControl::clearCoordinateBufferIfInitialized(const char* operation)
{
	if (!m_coordinateBufferInitialized)
	{
		m_cuttingCoordinateReady = false;
		m_bCrdStarted = false;
		m_hasPreviousCuttingPose = false;
		LCNC_INFO(lcnc::LogCode::Generic,
			"gtn.api: operation={} api=GTN_CrdClear action=skip reason=coordinate_fifo_not_initialized result=0",
			operation);
		return true;
	}

	const short result = GTN_CrdClear(m_iCore, 1, m_iWriteBuf);
	if (result != 0)
		return LogError(operation, "GTN_CrdClear", "", result), false;

	m_coordinateBufferInitialized = false;
	m_cuttingCoordinateReady = false;
	m_bCrdStarted = false;
	m_hasPreviousCuttingPose = false;
	LCNC_INFO(lcnc::LogCode::Generic,
		"gtn.api: operation={} api=GTN_CrdClear fifo={} result=0",
		operation, m_iWriteBuf);
	return true;
}

// [P3 removed] GTNMotionControl::IsOffsetCutting

bool GTNMotionControl::IsBufferRunning(int iBufferIndex)
{
	short run, sRtn;
	long segment;
	long pSpace;
	sRtn = GTN_CrdStatus(
		m_iCore,
		1, // 坐标系是坐标系1 
		&run, // 读取插补运动状态 
		&segment, // 读取当前已经完成的插补段数
		m_iWriteBuf); // 查询坐标系1的FIFO0缓存区

	if (sRtn)
		return LogError("IsBufferRunning", "GTN_CrdStatus","", sRtn), false;
	return run;
}

bool GTNMotionControl::ConfigureCuttingAxes(const std::array<Axis, 5>& axes, int dimension)
{
	if (dimension < 3 || dimension > 5)
		return false;
	m_cuttingAxes = axes;
	m_cuttingAxisCount = dimension;
	m_cuttingCoordinateReady = false;
	return true;
}

bool GTNMotionControl::UsesGroupArchitecture() const
{
	return m_settings.rawValue(
		lcnc::process::ProcessConfigArea::Devices, QStringLiteral("GTN"),
		QStringLiteral("bUseGroupArchitecture"), false).toBool();
}

bool GTNMotionControl::IsGroupRtcpRequested() const
{
	return UsesGroupArchitecture() && m_settings.rawValue(
		lcnc::process::ProcessConfigArea::Devices, QStringLiteral("GTN"),
		QStringLiteral("bEnableRtcp"), false).toBool();
}

void GTNMotionControl::clearGroupRuntimeState()
{
	m_groupCompletionSettling = false;
	m_groupReady = false;
	m_groupRtcpActive = false;
	m_groupRtcpConfigurationDerived = false;
	m_groupListHasData = false;
	m_groupListHasMotion = false;
	m_groupListStarted = false;
	m_groupCommandPositionValid = false;
	m_groupCommandPosition.fill(0.0);
	m_groupSegmentNumber = 0;
	m_groupRtcpValidationCounter = 0;
}

bool GTNMotionControl::InitFiveAxisGroup(const Tool& tool)
{
	if (!requireInitializedConnection("InitFiveAxisGroup"))
		return false;
	// Stop/Reset retains the original profile and first-fault evidence.
	if (m_groupFeedbackFaultLatched) {
		m_bErrorOccurred = true;
		return LogError("InitFiveAxisGroup",
			"feedback fault latched; verify axis direction and reconnect", "", -1), false;
	}
	int currentFault = 0;
	if (!IsMachiningStatusNormal(currentFault) || currentFault != 0) {
		m_bErrorOccurred = true;
		return LogError("InitFiveAxisGroup", "current axis fault blocks machining",
			"", -1), false;
	}
	m_groupExecutionError.clear();
	if (!m_bConnectFlag)
		return LogError("InitFiveAxisGroup", "controller not connected", "", -1), false;
	if (!UsesGroupArchitecture())
		return LogError("InitFiveAxisGroup", "Group architecture disabled", "", -1), false;

	m_groupIndex = static_cast<short>(m_settings.rawValue(
		lcnc::process::ProcessConfigArea::Devices, QStringLiteral("GTN"),
		QStringLiteral("iFiveAxisGroup"), 1).toInt());
	m_commandListIndex = static_cast<short>(m_settings.rawValue(
		lcnc::process::ProcessConfigArea::Devices, QStringLiteral("GTN"),
		QStringLiteral("iFiveAxisCommandList"), 1).toInt());
	if (m_groupIndex < 1 || m_groupIndex > 2
		|| m_commandListIndex < 1 || m_commandListIndex > 4) {
		return LogError("InitFiveAxisGroup", "invalid Group/CommandList index", "", -1), false;
	}

	auto* kernel = lcnc::Kernel::tryCurrent();
	auto machine = kernel
		? kernel->services().getService<lcnc::MachineConfigurationService>() : nullptr;
	auto calibration = kernel
		? kernel->services().getService<lcnc::kinematics::MachineCalibrationService>() : nullptr;
	if (!machine)
		return LogError("InitFiveAxisGroup", "MachineConfigurationService", "", -1), false;

	const bool requestRtcp = IsGroupRtcpRequested();
	const bool allowConfigurationDerivedRtcp = requestRtcp && m_settings.rawValue(
		lcnc::process::ProcessConfigArea::Devices, QStringLiteral("GTN"),
		QStringLiteral("bAllowConfigurationDerivedRtcp"), false).toBool();
	lcnc::kinematics::ControllerKinematicsSnapshot snapshot;
	QString snapshotError;
	if (!lcnc::kinematics::buildControllerKinematicsSnapshot(
			*machine, calibration.get(),
			requestRtcp
				? (allowConfigurationDerivedRtcp
					? lcnc::kinematics::ControllerCalibrationRequirement::MachineVerifiedOrConfigurationDerived
					: lcnc::kinematics::ControllerCalibrationRequirement::MachineVerified)
				: lcnc::kinematics::ControllerCalibrationRequirement::None,
			&snapshot, &snapshotError)) {
		LCNC_ERR(lcnc::LogCode::Generic,
			"gtn.api: operation=InitFiveAxisGroup phase=preflight rtcp={} result=-1 reason={}",
			requestRtcp, snapshotError.toStdString());
		m_bErrorOccurred = true;
		return false;
	}
	const bool configurationDerived = requestRtcp
		&& snapshot.calibrationConfigurationDerived;
	for (int index = 0; index < 5; ++index) {
		const auto motor = m_mapMotorValue.find(m_cuttingAxes[index]);
		if (motor == m_mapMotorValue.end()
			|| motor->second.AxisIndex != snapshot.physicalAxisIndices[index]) {
			LCNC_ERR(lcnc::LogCode::Generic,
				"gtn.api: operation=InitFiveAxisGroup phase=axis_map slot={} "
				"machine_axis={} process_axis={} result=-1",
				index + 1, snapshot.physicalAxisIndices[index],
				motor == m_mapMotorValue.end() ? -1 : motor->second.AxisIndex);
			m_bErrorOccurred = true;
			return false;
		}
	}

	TListInfo immediate{};
	immediate.list = 0;
	immediate.modal = 0;
	const auto call = [this](const char* api, short result) {
		if (result != 0) {
			LogError("InitFiveAxisGroup", api, "", result);
			m_bErrorOccurred = true;
			return false;
		}
		LCNC_INFO(lcnc::LogCode::Generic,
			"gtn.api: operation=InitFiveAxisGroup api={} group={} list={} result=0",
			api, m_groupIndex, m_commandListIndex);
		return true;
	};
	// Start every group with safe outputs; machining IO is queued only later.
	// 中文翻译：所有 Group 初始化均先关闭输出，随后由正常加工时序控制激光和气体。
	if (!stopConfiguredOutputs("InitFiveAxisGroupOutputsOff"))
		return false;

	long groupAxisMask = 0;
	for (const short physicalAxis : snapshot.physicalAxisIndices)
		groupAxisMask |= AxisMaskByIndex(physicalAxis);
	// Retain possible ownership until all prior motion and release calls have
	// succeeded. This recovery is outside the rollback guard so a failed stop
	// cannot be retried implicitly by scope destruction in this same attempt.
	m_groupReady = true;
	if (!stopAndReleaseFiveAxisGroup(configuredAxisMask(), "InitFiveAxisGroupRecovery"))
		return false;
	clearGroupRuntimeState();
	bool initializationCommitted = false;
	bool hardwareInitializationBegun = false;
	const auto rollbackGroupInitialization = qScopeGuard(
		[this, &initializationCommitted, &hardwareInitializationBegun] {
		if (initializationCommitted || !hardwareInitializationBegun)
			return;
		const bool released = stopAndReleaseFiveAxisGroup(
			configuredAxisMask(), "InitFiveAxisGroupRollback");
		if (released)
			clearGroupRuntimeState();
		m_bErrorOccurred = true;
		LCNC_WARN(lcnc::LogCode::Generic,
			"gtn.api: operation=InitFiveAxisGroup phase=rollback group={} list={} released={} ownership_retained={} stop_fault_latched={} result={}",
			m_groupIndex, m_commandListIndex, released, m_groupReady,
			m_stopFaultLatched, released ? 0 : -1);
	});
	// A stopped Group can retain its unfinished target as the axis profile
	// position while APOS already reflects the physical stop point.  The next
	// absolute Group command is then judged against that stale target and can be
	// rejected as 17502/11802 (start equals end).  Rebase every stationary
	// member profile to its encoder before assigning the axes to a new Group.
	if (!synchronizeAxisProfilesToEncoders(groupAxisMask, "InitFiveAxisGroup", true))
		return false;
	// A failed AddAxis/Enable may have partially changed the controller. Stop
	// must continue checking this Group even when initialization never commits.
	hardwareInitializationBegun = true;
	m_groupReady = true;
	for (int index = 0; index < 5; ++index) {
		TProfileScale scale{};
		scale.count = snapshot.scales[index].count;
		scale.alpha[0] = snapshot.scales[index].alpha;
		scale.beta[0] = snapshot.scales[index].beta;
		if (!call("GTN_SetAxisScale", GTN_SetAxisScale(
				m_iCore, snapshot.physicalAxisIndices[index], &scale, &immediate)))
			return false;
	}
	for (int index = 0; index < 5; ++index) {
		if (!call("GTN_AddAxisToGroup", GTN_AddAxisToGroup(
				m_iCore, m_groupIndex, snapshot.physicalAxisIndices[index],
				static_cast<short>(index + 1), &immediate)))
			return false;
	}

	TKinematicTransform transform{};
	transform.type = KIN_TYPE_FIVE_AXIS;
	auto& fiveAxis = transform.kinPrm.fiveAxis;
	fiveAxis.type = snapshot.modelType;
	for (int component = 0; component < 3; ++component) {
		fiveAxis.primaryAxisPoint[component] = snapshot.primaryAxisPointMcs[component];
		fiveAxis.slaveAxisPoint[component] = snapshot.slaveAxisPointMcs[component];
		fiveAxis.toolLocationPoint[component] = snapshot.toolLocationPointMcs[component];
	}
	fiveAxis.dirMode = snapshot.directionMode;
	for (int axis = 0; axis < 5; ++axis) {
		fiveAxis.dir[axis] = snapshot.directions[axis];
		for (int component = 0; component < 3; ++component)
			fiveAxis.axisVector[axis][component] = snapshot.axisVectorsMcs[axis][component];
		LCNC_INFO(lcnc::LogCode::Generic,
			"gtn.api: operation=InitFiveAxisGroup phase=model_axis mcs=MachineWorldZUpV2 slot={} physical_axis={} dir_mode={} dir={} vector=[{},{},{}]",
			axis + 1, snapshot.physicalAxisIndices[axis], fiveAxis.dirMode,
			fiveAxis.dir[axis], fiveAxis.axisVector[axis][0],
			fiveAxis.axisVector[axis][1], fiveAxis.axisVector[axis][2]);
	}
	LCNC_INFO(lcnc::LogCode::Generic,
		"gtn.api: operation=InitFiveAxisGroup phase=model_points mcs=MachineWorldZUpV2 primary=[{},{},{}] slave=[{},{},{}] tool=[{},{},{}]",
		fiveAxis.primaryAxisPoint[0], fiveAxis.primaryAxisPoint[1], fiveAxis.primaryAxisPoint[2],
		fiveAxis.slaveAxisPoint[0], fiveAxis.slaveAxisPoint[1], fiveAxis.slaveAxisPoint[2],
		fiveAxis.toolLocationPoint[0], fiveAxis.toolLocationPoint[1], fiveAxis.toolLocationPoint[2]);
	if (!call("GTN_SetGroupKinematicTransform", GTN_SetGroupKinematicTransform(
			m_iCore, m_groupIndex, &transform, &immediate)))
		return false;

	TCartesianParameter identity{};
	if (!call("GTN_SetGroupCartesianTransform_PCS", GTN_SetGroupCartesianTransform(
			m_iCore, m_groupIndex, COORD_SYSTEM_PCS, 1, &identity, &immediate))
		|| !call("GTN_SetGroupCartesianTransform_TCS", GTN_SetGroupCartesianTransform(
			m_iCore, m_groupIndex, COORD_SYSTEM_TCS, 1, &identity, &immediate)))
		return false;
	const short commandCoordinate = requestRtcp ? COORD_SYSTEM_MCS : COORD_SYSTEM_ACS;
	const short orientationMode = requestRtcp ? ORI_MODE_ROTATE_AXIS_POS : ORI_MODE_NONE;
	const short profileCoordinate = requestRtcp ? COORD_SYSTEM_PCS : COORD_SYSTEM_ACS;
	if (!call("GTN_SetGroupCommandPosDefine", GTN_SetGroupCommandPosDefine(
			m_iCore, m_groupIndex, commandCoordinate, orientationMode, 0, &immediate))
		|| !call("GTN_SetGroupProfileCoordinateSystem", GTN_SetGroupProfileCoordinateSystem(
			m_iCore, m_groupIndex, profileCoordinate, &immediate)))
		return false;

	if (!call("GTN_SetCommandListLinkGroup", GTN_SetCommandListLinkGroup(
			m_iCore, m_commandListIndex, 1UL << (m_groupIndex - 1)))
		|| !call("GTN_SetGroupLinkCommandList", GTN_SetGroupLinkCommandList(
			m_iCore, m_groupIndex, 1UL << (m_commandListIndex - 1))))
		return false;

	TGroupMotionConstraint motion{};
	motion.velMax = std::max(tool.m_dLineVelocity, 1.0);
	motion.accMax = std::max(tool.m_dLineAcc, 1.0);
	motion.decMax = motion.accMax;
	motion.jerkMax = std::max(tool.m_dLineJerk, motion.accMax);
	TGroupOrientationConstraint orientation{};
	orientation.oriVelMax = 180.0;
	orientation.oriAccMax = 720.0;
	orientation.oriDecMax = 720.0;
	orientation.oriJerkMax = 7200.0;
	if (!call("GTN_SetGroupMotionConstraint", GTN_SetGroupMotionConstraint(
			m_iCore, m_groupIndex, &motion, &immediate))
		|| !call("GTN_SetGroupOrientationConstraint", GTN_SetGroupOrientationConstraint(
			m_iCore, m_groupIndex, &orientation, &immediate)))
		return false;
	const auto runtimeAxes = machine->axisConfigurations();
	for (int index = 0; index < static_cast<int>(snapshot.physicalAxisIndices.size()); ++index) {
		const short physicalAxis = snapshot.physicalAxisIndices[index];
		const auto configured = std::find_if(
			runtimeAxes.cbegin(), runtimeAxes.cend(), [physicalAxis](const auto& axis) {
				return axis.controllerIndex == physicalAxis;
			});
		if (configured == runtimeAxes.cend())
			return LogError("InitFiveAxisGroup", "axis motion constraint source", "", -1), false;
		TAxisMotionConstraint axisConstraint{};
		axisConstraint.velMax = std::max(configured->motionSpeed, 0.001);
		axisConstraint.accMax = std::max(configured->acceleration, 0.001);
		axisConstraint.decMax = axisConstraint.accMax;
		axisConstraint.jerkMax = std::max(configured->jerk, axisConstraint.accMax);
		axisConstraint.dvMax = std::max(axisConstraint.velMax * 0.05, 0.001);
		axisConstraint.reverseLimitMode = 1;
		axisConstraint.reserve1[AXIS_MOTION_CONSTRAINT_RESERVE1_DV_MAX_LIMIT] = 1;
		if (!call("GTN_SetAxisMotionConstraint", GTN_SetAxisMotionConstraint(
				m_iCore, physicalAxis, &axisConstraint, &immediate)))
			return false;
	}

	TVelProfileMode smooth{};
	smooth.mode = VEL_PROFILE_MODE_SMOOTH;
	smooth.parameter.smooth.accTime = m_settings.rawValue(
		lcnc::process::ProcessConfigArea::Devices, QStringLiteral("GTN"),
		QStringLiteral("fGroupSmoothTime"), 20.0).toDouble();
	smooth.parameter.smooth.k = m_settings.rawValue(
		lcnc::process::ProcessConfigArea::Devices, QStringLiteral("GTN"),
		QStringLiteral("fGroupSmoothK"), 15.0).toDouble();
	TGroupLookAheadParameter lookAhead{};
	lookAhead.lookAheadNum = m_settings.rawValue(
		lcnc::process::ProcessConfigArea::Devices, QStringLiteral("GTN"),
		QStringLiteral("iGroupLookAheadNum"), 200).toInt();
	lookAhead.time = m_settings.rawValue(
		lcnc::process::ProcessConfigArea::Devices, QStringLiteral("GTN"),
		QStringLiteral("fGroupLookAheadTime"), 0.01).toDouble();
	lookAhead.radiusRatio = m_settings.rawValue(
		lcnc::process::ProcessConfigArea::Devices, QStringLiteral("GTN"),
		QStringLiteral("fGroupLookAheadRadiusRatio"), 0.1).toDouble();
	// The GSN reference five-axis demo enables the Group before initializing
	// Group look-ahead.  Enabling look-ahead on a disabled Group can appear to
	// succeed here, but the first GTN_MoveLinearAbsolute then fails with 11703
	// (look-ahead initialization failed).
	if (!call("GTN_GroupEnable", GTN_GroupEnable(m_iCore, m_groupIndex, &immediate))
		|| !call("GTN_SetGroupVelProfileMode", GTN_SetGroupVelProfileMode(
			m_iCore, m_groupIndex, &smooth, &immediate))
		|| !call("GTN_GroupLookAheadEnable", GTN_GroupLookAheadEnable(
			m_iCore, m_groupIndex, &immediate))
		|| !call("GTN_SetGroupLookAheadParameter", GTN_SetGroupLookAheadParameter(
			m_iCore, m_groupIndex, &lookAhead, &immediate))
		|| !call("GTN_SetGroupCartesianCoordinateAxisLimit", GTN_SetGroupCartesianCoordinateAxisLimit(
			m_iCore, m_groupIndex, 2, &immediate)))
		return false;
	double rotaryRatios[2]{
		m_settings.rawValue(
			lcnc::process::ProcessConfigArea::Devices, QStringLiteral("GTN"),
			QStringLiteral("fGroupPrimaryRotaryVelRefRatio"), 20.0).toDouble(),
		m_settings.rawValue(
			lcnc::process::ProcessConfigArea::Devices, QStringLiteral("GTN"),
			QStringLiteral("fGroupSlaveRotaryVelRefRatio"), 20.0).toDouble()};
	if (!call("GTN_SetGroupCommandVelRefAxis", GTN_SetGroupCommandVelRefAxis(
			m_iCore, m_groupIndex, 0x1f, &immediate))
		|| !call("GTN_SetGroupCommandVelRefRatio", GTN_SetGroupCommandVelRefRatio(
			m_iCore, m_groupIndex, 4, rotaryRatios, 2, &immediate)))
		return false;

	short readCoordinate = -1;
	short readOrientation = -1;
	short readConfig = -1;
	short readProfile = -1;
	TKinematicTransform readTransform{};
	if (!call("GTN_GetGroupCommandPosDefine", GTN_GetGroupCommandPosDefine(
			m_iCore, m_groupIndex, &readCoordinate, &readOrientation, &readConfig))
		|| !call("GTN_GetGroupProfileCoordinateSystem", GTN_GetGroupProfileCoordinateSystem(
			m_iCore, m_groupIndex, &readProfile))
		|| !call("GTN_GetGroupKinematicTransform", GTN_GetGroupKinematicTransform(
			m_iCore, m_groupIndex, &readTransform)))
		return false;
	if (readCoordinate != commandCoordinate || readOrientation != orientationMode
		|| readProfile != profileCoordinate || readTransform.type != KIN_TYPE_FIVE_AXIS
		|| readTransform.kinPrm.fiveAxis.type != snapshot.modelType) {
		return LogError("InitFiveAxisGroup", "readback mismatch", "", -1), false;
	}

	m_groupReady = true;
	m_groupRtcpActive = requestRtcp;
	m_groupRtcpConfigurationDerived = configurationDerived;
	m_bErrorOccurred = false;
	const short commandPositionResult = GTN_GetGroupProfilePos(
		m_iCore, m_groupIndex, 1, m_groupCommandPosition.data(), 5,
		commandCoordinate, orientationMode);
	if (commandPositionResult != 0) {
		m_groupCommandPositionValid = false;
		return LogError("InitFiveAxisGroup", "GTN_GetGroupProfilePos(command-start)",
			"", commandPositionResult), false;
	}
	m_groupCommandPositionValid = true;
	LCNC_INFO(lcnc::LogCode::Generic,
		"gtn.api: operation=InitFiveAxisGroup phase=complete group={} list={} rtcp={} configuration_derived={} execution=normal motion_limits=configured command_coordinate={} profile_coordinate={} orientation_mode={} model={} axes=[{},{},{},{},{}] command_start=[{},{},{},{},{}] calibration={} result=0",
		m_groupIndex, m_commandListIndex, m_groupRtcpActive,
		m_groupRtcpConfigurationDerived, commandCoordinate,
		profileCoordinate, orientationMode, snapshot.modelType,
		snapshot.physicalAxisIndices[0], snapshot.physicalAxisIndices[1],
		snapshot.physicalAxisIndices[2], snapshot.physicalAxisIndices[3],
		snapshot.physicalAxisIndices[4], m_groupCommandPosition[0],
		m_groupCommandPosition[1], m_groupCommandPosition[2],
		m_groupCommandPosition[3], m_groupCommandPosition[4],
		snapshot.calibrationFingerprint.toStdString());
	if (!ResetFiveAxisGroupProgram())
		return false;
	initializationCommitted = true;
	return true;
}

bool GTNMotionControl::ResetFiveAxisGroupProgram()
{
	if (!m_groupReady)
		return false;
	TListInfo immediate{};
	const short result = GTN_ClearCommandListData(
		m_iCore, m_commandListIndex, &immediate);
	if (result != 0)
		return LogError("ResetFiveAxisGroupProgram", "GTN_ClearCommandListData", "", result), false;
	const short clearStatus = GTN_ClearCommandListStatus(
		m_iCore, m_commandListIndex, &immediate);
	if (clearStatus != 0)
		return LogError("ResetFiveAxisGroupProgram", "GTN_ClearCommandListStatus", "", clearStatus), false;
	m_groupListHasData = false;
	m_groupListStarted = false;
	m_groupListHasMotion = false;
	m_groupSegmentNumber = 0;
	m_groupRtcpValidationCounter = 0;
	LCNC_INFO(lcnc::LogCode::Generic,
		"gtn.api: operation=ResetFiveAxisGroupProgram api=GTN_ClearCommandListData+GTN_ClearCommandListStatus group={} list={} result=0",
		m_groupIndex, m_commandListIndex);
	return true;
}

bool GTNMotionControl::GroupLineTo(const std::array<double, 5>& position,
	                               const Tool& tool, long userTag)
{
	if (!requireInitializedConnection("GroupLineTo"))
		return false;
	if (m_bStop.load()) {
		LCNC_WARN(lcnc::LogCode::Generic,
			"gtn.api: operation=GroupLineTo action=reject reason=stop_requested group={} list={} next_segment={} user_tag={}",
			m_groupIndex, m_commandListIndex, m_groupSegmentNumber + 1, userTag);
		return false;
	}
	if (!m_groupReady || m_groupListStarted)
		return LogError("GroupLineTo", "Group list is not writable", "", -1), false;
	std::array<double, 5> resolutions{};
	for (int index = 0; index < 5; ++index) {
		const auto motor = m_mapMotorValue.find(m_cuttingAxes[index]);
		resolutions[index] = motor == m_mapMotorValue.end()
			? 0.0 : static_cast<double>(motor->second.Resolution);
	}
	std::array<double, 5> duplicateTolerance{};
	const auto decision = lcnc::process::classifyGroupMotion(
		position, m_groupCommandPosition, resolutions, m_groupRtcpActive,
		&duplicateTolerance);
	if (!m_groupCommandPositionValid
		|| decision == lcnc::process::GroupMotionDecision::Invalid) {
		m_bErrorOccurred = true;
		return LogError("GroupLineTo", "invalid command position or resolution", "", -1), false;
	}
	if (decision == lcnc::process::GroupMotionDecision::Skip) {
		LCNC_INFO(lcnc::LogCode::Generic,
			"gtn.api: operation=GroupLineTo action=skip reason=near_zero_target group={} list={} next_segment={} user_tag={} rtcp={} target=[{},{},{},{},{}] reference=[{},{},{},{},{}] tolerance=[{},{},{},{},{}] result=0",
			m_groupIndex, m_commandListIndex, m_groupSegmentNumber + 1,
			userTag, m_groupRtcpActive, position[0], position[1], position[2],
			position[3], position[4], m_groupCommandPosition[0],
			m_groupCommandPosition[1], m_groupCommandPosition[2],
			m_groupCommandPosition[3], m_groupCommandPosition[4],
			duplicateTolerance[0], duplicateTolerance[1],
			duplicateTolerance[2], duplicateTolerance[3],
			duplicateTolerance[4]);
		return true;
	}
	TListInfo list{};
	list.list = m_commandListIndex;
	list.modal = 0;
	list.segNum = ++m_groupSegmentNumber;
	list.reserve2[LISTINFO_RESERVE2_USERTAG] = userTag;
	TGroupMoveParameter move{};
	move.velocity = std::max(tool.m_dLineVelocity, 0.001);
	move.acceleration = std::max(tool.m_dLineAcc, 0.001);
	move.deceleration = move.acceleration;
	move.overrideSelect = 0;
	move.endVelocityMode = 0;
	move.orientationDir = 0;
	double target[8]{};
	short direction[8]{};
	std::copy(position.cbegin(), position.cend(), target);
	const short result = GTN_MoveLinearAbsolute(
		m_iCore, m_groupIndex, target, direction, &move, &list);
	if (result != 0) {
		m_bErrorOccurred = true;
		LCNC_ERR(lcnc::LogCode::Generic,
			"gtn.api: operation=GroupLineTo api=GTN_MoveLinearAbsolute group={} list={} segment={} user_tag={} rtcp={} target=[{},{},{},{},{}] result={}",
			m_groupIndex, m_commandListIndex, list.segNum, userTag,
			m_groupRtcpActive, position[0], position[1], position[2],
			position[3], position[4], result);

		// 11802 means identical start/end in the supplier's straight-line table.
		// Do not retry/drop a rejected command here: filtering belongs before
		// submission. LastCommandError may belong to a different earlier call.
		TLookAheadErrorInfo lookAheadError{};
		const short lookAheadResult = GTN_GetLookAheadErrorInfo(
			m_iCore, m_groupIndex, &lookAheadError);
		LCNC_ERR(lcnc::LogCode::Generic,
			"gtn.api: operation=GroupLineTo diagnostic=look_ahead api=GTN_GetLookAheadErrorInfo group={} list={} segment={} user_tag={} error_segment={} error_user_tag={} data=[{},{},{},{},{},{},{},{}] result={}",
			m_groupIndex, m_commandListIndex, list.segNum, userTag,
			lookAheadError.segNum, lookAheadError.userTag,
			lookAheadError.data[0], lookAheadError.data[1],
			lookAheadError.data[2], lookAheadError.data[3],
			lookAheadError.data[4], lookAheadError.data[5],
			lookAheadError.data[6], lookAheadError.data[7], lookAheadResult);

		TCommandInfoData commandError{};
		const short commandErrorResult = GTN_GetLastCommandError(
			m_iCore, &commandError);
		LCNC_ERR(lcnc::LogCode::Generic,
			"gtn.api: operation=GroupLineTo diagnostic=last_command api=GTN_GetLastCommandError group={} list={} segment={} user_tag={} command_code={} command_rtn={} error_code={} error_segment={} error_user_tag={} data_length={} data16=[{},{},{},{},{},{},{},{}] result={} expected_command_code=0x4680 same_command_code={}",
			m_groupIndex, m_commandListIndex, list.segNum, userTag,
			commandError.commandCode, commandError.commandRtn,
			commandError.errorCode, commandError.segmentNum,
			commandError.userTag, commandError.dataLength,
			commandError.data16[0], commandError.data16[1],
			commandError.data16[2], commandError.data16[3],
			commandError.data16[4], commandError.data16[5],
			commandError.data16[6], commandError.data16[7], commandErrorResult,
			commandErrorResult == 0 && commandError.commandCode == 0x4680);
		return false;
	}
	m_groupListHasData = true;
	m_groupListHasMotion = true;
	m_groupCommandPosition = position;
	m_groupCommandPositionValid = true;
	LCNC_DEBUG(lcnc::LogCode::Generic,
		"gtn.api: operation=GroupLineTo api=GTN_MoveLinearAbsolute group={} list={} segment={} user_tag={} rtcp={} target=[{},{},{},{},{}] result=0",
		m_groupIndex, m_commandListIndex, list.segNum, userTag,
		m_groupRtcpActive, position[0], position[1], position[2],
		position[3], position[4]);
	return true;
}

bool GTNMotionControl::ValidateGroupRtcpTarget(
	const std::array<double, 5>& tcpAndOrientation,
	const std::array<double, 5>& predictedAxes)
{
	if (!m_groupReady || !m_groupRtcpActive)
		return LogError("ValidateGroupRtcpTarget", "RTCP Group is not ready", "", -1), false;
	const double tolerance = m_settings.rawValue(
		lcnc::process::ProcessConfigArea::Devices, QStringLiteral("GTN"),
		QStringLiteral("fGroupRtcpAxisAgreementTolerance"), 0.05).toDouble();
	// Check every input, including points outside the configured sampling stride.
	if (!lcnc::process::finiteRtcpPose(tcpAndOrientation)
		|| !lcnc::process::finiteRtcpPose(predictedAxes)
		|| !std::isfinite(tolerance) || tolerance <= 0.0) {
		m_bErrorOccurred = true;
		return LogError("ValidateGroupRtcpTarget", "invalid RTCP reference, axes or tolerance", "", -1), false;
	}
	const int stride = std::max(1, m_settings.rawValue(
		lcnc::process::ProcessConfigArea::Devices, QStringLiteral("GTN"),
		QStringLiteral("iGroupRtcpValidationStride"), 100).toInt());
	const long sampleIndex = ++m_groupRtcpValidationCounter;
	if (sampleIndex != 1 && ((sampleIndex - 1) % stride) != 0)
		return true;
	TGroupPosTransformInput input{};
	input.coordSystem = COORD_SYSTEM_MCS;
	input.oriMode = ORI_MODE_ROTATE_AXIS_POS;
	input.configIndex = 0;
	input.targetOriMode = ORI_MODE_ROTATE_AXIS_POS;
	for (int index = 0; index < 5; ++index) {
		input.inputPos[index] = tcpAndOrientation[index];
		input.preAcsPos[index] = predictedAxes[index];
	}
	TGroupPosTransformOutput output{};
	const short result = GTN_GroupPosTransform(m_iCore, m_groupIndex, &input, &output);
	if (result != 0)
		return LogError("ValidateGroupRtcpTarget", "GTN_GroupPosTransform", "", result), false;
	double maximumError = 0.0;
	const std::array<double, 5> actualAxes{output.acsPos[0], output.acsPos[1],
		output.acsPos[2], output.acsPos[3], output.acsPos[4]};
	const bool agrees = lcnc::process::rtcpAxesAgree(
		predictedAxes, actualAxes, tolerance, &maximumError);
	if (output.singularity != 0 || !agrees) {
		m_bErrorOccurred = true;
		LCNC_ERR(lcnc::LogCode::Generic,
			"gtn.api: operation=ValidateGroupRtcpTarget api=GTN_GroupPosTransform group={} singularity={} config={} maximum_axis_error={} tolerance={} predicted=[{},{},{},{},{}] controller=[{},{},{},{},{}] input_mcs=[{},{},{},{},{}] reference=table_zero result=-1",
			m_groupIndex, output.singularity, output.configIndex, maximumError, tolerance,
			predictedAxes[0], predictedAxes[1], predictedAxes[2], predictedAxes[3], predictedAxes[4],
			output.acsPos[0], output.acsPos[1], output.acsPos[2], output.acsPos[3], output.acsPos[4],
			tcpAndOrientation[0], tcpAndOrientation[1], tcpAndOrientation[2],
			tcpAndOrientation[3], tcpAndOrientation[4]);
		return false;
	}
	if (sampleIndex == 1) {
		LCNC_INFO(lcnc::LogCode::Generic,
			"gtn.api: operation=ValidateGroupRtcpTarget api=GTN_GroupPosTransform group={} sample={} stride={} maximum_axis_error={} tolerance={} input_mcs=[{},{},{},{},{}] reference=table_zero result=0",
			m_groupIndex, sampleIndex, stride, maximumError, tolerance,
			tcpAndOrientation[0], tcpAndOrientation[1], tcpAndOrientation[2],
			tcpAndOrientation[3], tcpAndOrientation[4]);
	}
	return true;
}

bool GTNMotionControl::StartFiveAxisGroupProgram()
{
	if (!requireInitializedConnection("StartFiveAxisGroupProgram"))
		return false;
	const auto cancelled = [this] { return m_bStop.load(); };
	const auto logCancelled = [this](int attempts, short lastResult) {
		LCNC_WARN(lcnc::LogCode::Generic,
			"gtn.api: operation=StartFiveAxisGroupProgram action=reject reason=stop_requested group={} list={} attempts={} last_data_end_result={} start_called=false",
			m_groupIndex, m_commandListIndex, attempts, lastResult);
	};
	if (cancelled()) {
		logCancelled(0, 0);
		return false;
	}
	if (!m_groupReady || m_groupListStarted)
		return LogError("StartFiveAxisGroupProgram", "Group list state", "", -1), false;
	int currentFault = 0;
	if (!IsMachiningStatusNormal(currentFault) || currentFault != 0) {
		m_bErrorOccurred = true;
		return LogError("StartFiveAxisGroupProgram", "current axis fault blocks machining", "", -1), false;
	}
	if (m_groupListHasData && !m_groupListHasMotion) {
		// All motion was filtered. Never execute the remaining laser-on/delay
		// commands at a stationary point. This is a no-motion batch, not a cut.
		if (!stopConfiguredOutputs("StartFiveAxisGroupProgram(no-effective-motion)")
			|| !ResetFiveAxisGroupProgram()) {
			m_bErrorOccurred = true;
			return false;
		}
		LCNC_INFO(lcnc::LogCode::Generic,
			"gtn.api: operation=StartFiveAxisGroupProgram action=skip reason=no_effective_motion discarded_io=true start_called=false group={} list={} result=0",
			m_groupIndex, m_commandListIndex);
	}
	if (!m_groupListHasData) {
		LCNC_INFO(lcnc::LogCode::Generic,
			"gtn.api: operation=StartFiveAxisGroupProgram action=skip reason=no_effective_commands group={} list={} result=0",
			m_groupIndex, m_commandListIndex);
		return true;
	}
	const auto deadline = std::chrono::steady_clock::now() + kFifoFlushTimeout;
	const auto submission = lcnc::process::startCancellableCommandList(
		cancelled,
		[this] { return GTN_CommandListDataEnd(m_iCore, m_commandListIndex); },
		[this] {
			long startAxisMask = 0;
			for (const auto axis : m_cuttingAxes) startAxisMask |= AxisMask(axis);
			logAxisPositionEvidence(startAxisMask, "StartFiveAxisGroupProgram");
		},
		[this] {
			int fault = 0;
			if (!IsMachiningStatusNormal(fault) || fault != 0 || m_bStop.load()) {
				m_bErrorOccurred = true;
				LCNC_ERR(lcnc::LogCode::Generic,
					"gtn.api: operation=StartFiveAxisGroupProgram phase=final_admission fault={} stop_requested={} start_called=false result=-1",
					fault, m_bStop.load());
				return static_cast<short>(-1);
			}
			TListInfo immediate{};
			return GTN_StartCommandList(m_iCore, m_commandListIndex, &immediate);
		},
		[&deadline] { return std::chrono::steady_clock::now() < deadline; },
		[] { std::this_thread::sleep_for(kFifoFlushRetryInterval); },
		kCommandListDataPending);
	const short result = submission.apiResult;
	const int attempts = submission.finalizeAttempts;
	if (submission.state == lcnc::process::CommandListStartState::Cancelled) {
		logCancelled(attempts, result);
		return false;
	}
	if (submission.state == lcnc::process::CommandListStartState::FinalizeFailed) {
		TCommandInfoData commandError{};
		const short diagnosticResult = GTN_GetLastCommandError(
			m_iCore, &commandError, -1, 1);
		LCNC_ERR(lcnc::LogCode::Generic,
			"gtn.api: operation=StartFiveAxisGroupProgram diagnostic=last_command api=GTN_GetLastCommandError group={} list={} data_end_result={} attempts={} command_code=0x{:x} command_rtn={} error_code={} error_segment={} error_user_tag={} data_length={} data16=[{},{},{},{},{},{},{},{}] result={}",
			m_groupIndex, m_commandListIndex, result, attempts,
			commandError.commandCode, commandError.commandRtn,
			commandError.errorCode, commandError.segmentNum,
			commandError.userTag, commandError.dataLength,
			commandError.data16[0], commandError.data16[1],
			commandError.data16[2], commandError.data16[3],
			commandError.data16[4], commandError.data16[5],
			commandError.data16[6], commandError.data16[7], diagnosticResult);
		return LogError("StartFiveAxisGroupProgram", "GTN_CommandListDataEnd", "", result), false;
	}
	if (submission.state == lcnc::process::CommandListStartState::StartFailed)
		return LogError("StartFiveAxisGroupProgram", "GTN_StartCommandList", "", result), false;
	m_groupListStarted = true;
	m_groupCompletionSettling = false;
	if (cancelled()) {
		LCNC_WARN(lcnc::LogCode::Generic,
			"gtn.api: operation=StartFiveAxisGroupProgram phase=start_returned group={} list={} start_called=true stop_pending=true action=retain_stop_request result=0",
			m_groupIndex, m_commandListIndex);
	}
	LCNC_INFO(lcnc::LogCode::Generic,
		"gtn.api: operation=StartFiveAxisGroupProgram api=GTN_CommandListDataEnd+GTN_StartCommandList group={} list={} segments={} attempts={} rtcp={} result=0",
		m_groupIndex, m_commandListIndex, m_groupSegmentNumber, attempts, m_groupRtcpActive);
	return true;
}

bool GTNMotionControl::IsFiveAxisGroupProgramRunning()
{
	if (!m_groupReady || !m_groupListStarted)
		return false;
	TCommandListStatus listStatus{};
	TGroupStatus groupStatus{};
	short result = GTN_GetCommandListStatus(m_iCore, m_commandListIndex, &listStatus);
	if (result != 0) {
		m_bErrorOccurred = true;
		return LogError("IsFiveAxisGroupProgramRunning", "GTN_GetCommandListStatus", "", result), false;
	}
	result = GTN_GetGroupStatus(m_iCore, m_groupIndex, &groupStatus);
	if (result != 0) {
		m_bErrorOccurred = true;
		return LogError("IsFiveAxisGroupProgramRunning", "GTN_GetGroupStatus", "", result), false;
	}
	const bool completedByStreamExhaustion = isNormalCommandListCompletion(
		listStatus.stopInfo, listStatus.execute, groupStatus.run, groupStatus.state);
	const bool stoppedNormally = completedByStreamExhaustion
		|| (listStatus.stopInfo == kCommandListStopInfoNone
			&& listStatus.execute == 0 && groupStatus.run == 0
			&& groupStatus.state == GROUP_STATE_STANDBY);
	if (stoppedNormally) {
		long axisMask = 0;
		for (const auto axis : m_cuttingAxes)
			axisMask |= AxisMask(axis);
		if (!validateStationaryFeedback(axisMask, "CompleteFiveAxisGroupProgram", false)) {
			if (!m_groupCompletionSettling) {
				m_groupCompletionSettling = true;
				m_groupCompletionDeadline = std::chrono::steady_clock::now()
					+ std::chrono::milliseconds(500);
			}
			// Return to the queue between samples; settling never blocks monitors
			// or the Stop lane. Do not report completion before feedback is valid.
			if (std::chrono::steady_clock::now() < m_groupCompletionDeadline)
				return true;
			// A recovered deadline sample is not a fault.
			if (validateStationaryFeedback(axisMask, "CompleteFiveAxisGroupProgram", true))
				return true;
			LCNC_ERR(lcnc::LogCode::Generic,
				"gtn.api: operation=CompleteFiveAxisGroupProgram phase=fault_snapshot fault_source=software_feedback_validation group={} list={} last_submitted_segment={} list_stop_info={} execute={} group_run={} group_state={} group_stop_info={} execute_segment={} remaining={} rtcp={} last_command_valid={} last_submitted_target=[{},{},{},{},{}] action=reject_next_motion",
				m_groupIndex, m_commandListIndex, m_groupSegmentNumber, listStatus.stopInfo,
				listStatus.execute, groupStatus.run, groupStatus.state, groupStatus.stopInfo,
				listStatus.executeSegNum, listStatus.remainderSegCount, m_groupRtcpActive,
				m_groupCommandPositionValid, m_groupCommandPosition[0], m_groupCommandPosition[1],
				m_groupCommandPosition[2], m_groupCommandPosition[3], m_groupCommandPosition[4]);
			logAxisPositionEvidence(axisMask, "CompleteFiveAxisGroupProgram");
			m_bErrorOccurred = true;
			m_groupFeedbackFaultLatched = true;
			m_groupListStarted = false;
			return false;
		}
	}
	if (completedByStreamExhaustion) {
		m_groupListStarted = false;
		LCNC_INFO(lcnc::LogCode::Generic,
			"gtn.api: operation=IsFiveAxisGroupProgramRunning event=complete reason=command_stream_empty group={} list={} list_stop_info={} group_state={} group_stop_info={} execute_segment={} remaining={} result=0",
			m_groupIndex, m_commandListIndex, listStatus.stopInfo,
			groupStatus.state, groupStatus.stopInfo, listStatus.executeSegNum,
			listStatus.remainderSegCount);
		return false;
	}
	if (listStatus.stopInfo != kCommandListStopInfoNone
		|| groupStatus.state == GROUP_STATE_ERROR_STOP) {
		m_bErrorOccurred = true;
		// 中文翻译：GTN 控制器执行故障：Group %1，CommandList %2，列表停止码 %3，Group 状态 %4，Group 停止码 %5，执行段 %6
		m_groupExecutionError = QCoreApplication::translate("GTNMotionControl",
			"GTN controller execution fault: Group %1, CommandList %2, list stop %3, Group state %4, Group stop %5, executing segment %6")
			.arg(m_groupIndex).arg(m_commandListIndex).arg(listStatus.stopInfo)
			.arg(groupStatus.state).arg(groupStatus.stopInfo).arg(listStatus.executeSegNum);
		LCNC_ERR(lcnc::LogCode::Generic,
			"gtn.api: operation=IsFiveAxisGroupProgramRunning group={} list={} list_stop_info={} group_state={} group_stop_info={} execute_segment={} remaining={} result=-1",
			m_groupIndex, m_commandListIndex, listStatus.stopInfo, groupStatus.state,
			groupStatus.stopInfo, listStatus.executeSegNum, listStatus.remainderSegCount);
		// Capture before Stop/Reset replaces stop codes and clears the stream.
		// LastCommandError is historical evidence, not necessarily this segment.
		logAxisPositionEvidence(configuredAxisMask(), "GroupExecutionFault");
		int axisFault = 0;
		const bool axisStatusRead = IsMachiningStatusNormal(axisFault);
		TCommandInfoData commandError{};
		const short diagnosticResult = GTN_GetLastCommandError(m_iCore, &commandError);
		LCNC_ERR(lcnc::LogCode::Generic,
			"gtn.api: operation=GroupExecutionFault phase=before_cleanup axis_status_read={} axis_fault={} executing_segment={} last_command_code=0x{:x} last_command_clock={} last_command_rtn={} last_error_code={} last_error_segment={} last_error_tag={} diagnostic_result={} attribution=unconfirmed",
			axisStatusRead, axisFault, listStatus.executeSegNum, commandError.commandCode,
			commandError.clockTime, commandError.commandRtn, commandError.errorCode,
			commandError.segmentNum, commandError.userTag, diagnosticResult);
		return false;
	}
	const bool running = listStatus.execute != 0 || groupStatus.run != 0;
	if (!running)
		m_groupListStarted = false;
	return running;
}

bool GTNMotionControl::StopFiveAxisGroupProgram()
{
	if (!m_groupReady)
		return true;
	const bool success = stopAndReleaseFiveAxisGroup(configuredAxisMask(), "StopFiveAxisGroupProgram");
	if (success) {
		clearGroupRuntimeState();
		// Only controller/list lifecycle errors can be reset here. A feedback
		// direction fault and its first snapshot survive Stop and Reset.
		m_bErrorOccurred = m_groupFeedbackFaultLatched || m_stopFaultLatched || !m_connectionInitialized;
	}
	LCNC_INFO(lcnc::LogCode::Generic,
		"gtn.api: operation=StopFiveAxisGroupProgram phase=complete group={} list={} released={} success={}",
		m_groupIndex, m_commandListIndex, success, success);
	return success;
}

bool GTNMotionControl::appendGroupDigitalOutput(
	lcnc::process::DigitalOUT output, bool enabled, const char* operation)
{
	const auto found = m_mapDigitalOUT.find(output);
	if (found == m_mapDigitalOUT.end())
		return LogError(operation, "digital output is not configured",
			std::string(magic_enum::enum_name(output)), -1), false;
	const DigitalIOData& io = found->second;
	TDigitalOutputBit command{};
	command.mode = 0;
	command.doType = io.bExpand ? MC_EXT_DO : MC_GPO;
	command.doIndex = static_cast<short>(io.iIO);
	command.doValue = static_cast<short>(enabled
		? (io.bExpand ? (io.bInversion ? 0 : 1) : (io.bInversion ? 1 : 0))
		: (io.bExpand ? (io.bInversion ? 1 : 0) : (io.bInversion ? 0 : 1)));
	TListInfo list{};
	list.list = m_commandListIndex;
	list.segNum = ++m_groupSegmentNumber;
	const short result = GTN_WriteDigitalOutputBit(m_iCore, &command, &list);
	if (result != 0)
		return LogError(operation, "GTN_WriteDigitalOutputBit", io.qstrID.toStdString(), result), false;
	m_groupListHasData = true;
	LCNC_DEBUG(lcnc::LogCode::Generic,
		"gtn.api: operation={} api=GTN_WriteDigitalOutputBit group={} list={} segment={} output={} hardware_id={} enabled={} result=0",
		operation, m_groupIndex, m_commandListIndex, list.segNum,
		magic_enum::enum_name(output), io.qstrID.toStdString(), enabled);
	return true;
}

bool GTNMotionControl::appendGroupDelay(double milliseconds, const char* operation)
{
	if (milliseconds < 1.0)
		return true;
	TDelay delay{};
	delay.delayTime = milliseconds;
	TListInfo list{};
	list.list = m_commandListIndex;
	list.segNum = ++m_groupSegmentNumber;
	const short result = GTN_SetDelay(m_iCore, &delay, &list);
	if (result != 0)
		return LogError(operation, "GTN_SetDelay", "", result), false;
	m_groupListHasData = true;
	LCNC_DEBUG(lcnc::LogCode::Generic,
		"gtn.api: operation={} api=GTN_SetDelay group={} list={} segment={} delay_ms={} result=0",
		operation, m_groupIndex, m_commandListIndex, list.segNum, milliseconds);
	return true;
}

bool GTNMotionControl::GroupLaserControl(bool laserOn, const Tool& tool)
{
	if (!m_groupReady || m_groupListStarted)
		return LogError("GroupLaserControl", "Group list is not writable", "", -1), false;
	if (laserOn) {
		const DigitalOUT selectedGas = tool.m_bBlow2
			? DigitalOUT::Blow2 : DigitalOUT::Blow;
		const DigitalOUT inactiveGas = tool.m_bBlow2
			? DigitalOUT::Blow : DigitalOUT::Blow2;
		// A machine may physically have only one gas channel. The unselected
		// channel is optional; if it is configured, explicitly switch it off.
		// The selected channel remains mandatory so machining cannot silently
		// continue without its requested assist gas.
		// 中文翻译：机台可以只有一路气；未选中的气路可缺省，选中的气路仍必须配置。
		const auto inactive = m_mapDigitalOUT.find(inactiveGas);
		if (inactive != m_mapDigitalOUT.end()) {
			if (!appendGroupDigitalOutput(inactiveGas, false, "GroupLaserControl"))
				return false;
		} else {
			LCNC_INFO(lcnc::LogCode::Generic,
				"gtn.api: operation=GroupLaserControl action=skip reason=optional_inactive_gas_not_configured output={} laser_on=1 result=0",
				magic_enum::enum_name(inactiveGas));
		}
		if (!appendGroupDigitalOutput(selectedGas, true, "GroupLaserControl")
			|| !appendGroupDelay(m_dBlowDelay, "GroupLaserControl")
			|| !appendGroupDelay(m_dLaserOnBWait, "GroupLaserControl"))
			return false;
	} else if (!appendGroupDelay(m_dLaserOffBWait, "GroupLaserControl")) {
		return false;
	}
	// The application configuration owns the physical laser gate mapping. The
	// Pro laser channel requires a separate controller-side initialization that
	// this application does not perform; enqueueing GTN_SetLaserEnablePro could
	// therefore succeed initially and fail validation at DataEnd with 11700.
	// Use the configured digital laser gate in the same CommandList instead.
	// 中文翻译：使用已配置的激光数字输出，避免未初始化 Pro 通道在 DataEnd 报 11700。
	if (!appendGroupDigitalOutput(
			DigitalOUT::Laser, laserOn, "GroupLaserControl"))
		return false;
	if (!appendGroupDelay(laserOn ? m_dLaserOnAWait : m_dLaserOffAWait,
			"GroupLaserControl"))
		return false;
	LCNC_INFO(lcnc::LogCode::Generic,
		"gtn.api: operation=GroupLaserControl api=GTN_WriteDigitalOutputBit+GTN_SetDelay group={} list={} laser_on={} result=0",
		m_groupIndex, m_commandListIndex, laserOn);
	return true;
}

bool GTNMotionControl::MoveToPosition(Axis axis, double velocity, double position)
{
	return MovePostion(axis, velocity, position);
}

bool GTNMotionControl::OffsetLineTo(const std::array<double, 5>& target,
	                                 int dimension, const Tool& tool)
{
	if (!requireInitializedConnection("OffsetLineTo"))
		return false;
	if (!m_cuttingCoordinateReady || dimension != m_cuttingAxisCount) {
		return LogError("OffsetLineTo", "dynamic coordinate system is not ready", "", -1), false;
	}
	double dVelocity = tool.m_dLineVelocity;
	double LineAcc = tool.m_dLineAcc;

	if (m_hasPreviousCuttingPose
		&& (m_dPreX == target[0]) && (m_dPreY == target[1]) && (m_dPreZ == target[2])
		&& (dimension < 4 || m_dPreR1 == target[3])
		&& (dimension < 5 || m_dPreR2 == target[4]))
		return true;
	short sRtn;
	const int MAX_RETRY = 10;
	for (int retry = 0; retry < MAX_RETRY; retry++)
	{
		if (dimension == 3) {
			sRtn = GTN_LnXYZEx(m_iCore, 1, target[0], target[1], target[2],
			                     dVelocity, LineAcc, 0, 0, m_iWriteBuf);
		} else if (dimension == 4) {
			sRtn = GTN_LnXYZAEx(m_iCore, 1, target[0], target[1], target[2], target[3],
			                      dVelocity, LineAcc, 0, 0, m_iWriteBuf);
		} else {
			double position[5] = {target[0], target[1], target[2], target[3], target[4]};
			sRtn = GTN_LnXYZACEx(m_iCore, 1, position, 0x1f,
			                       dVelocity, LineAcc, 0, 0, m_iWriteBuf);
		}
		if (!sRtn) break;
		// A line-write error is not automatically a FIFO-full condition. Attempt
		// one bounded, cancellable drain; propagate its failure rather than
		// repeatedly occupying the device worker for every remaining retry.
		LCNC_WARN(lcnc::LogCode::Generic,
			"gtn.api: operation=OffsetLineTo phase=line_write_retry attempt={} dimension={} target=[{},{},{},{},{}] api_result={}",
			retry + 1, dimension, target[0], target[1], target[2], target[3], target[4], sRtn);
		if (!FlushToFifo())
			return false;
	}
	if (sRtn)
		return LogError("OffsetLineTo", "GTN dynamic line interpolation", "", sRtn), false;
	m_dPreX = target[0];
	m_dPreY = target[1];
	m_dPreZ = target[2];
	if (dimension >= 4) m_dPreR1 = target[3];
	if (dimension >= 5) m_dPreR2 = target[4];
	m_hasPreviousCuttingPose = true;
	return true;
}

// [P3 removed] GTNMotionControl::OffsetArcTo

// 改设置界面为旋转轴置位
// [P3 removed] GTNMotionControl::JumpToSetAFPos

void GTNMotionControl::JumpToIdleXYPosition(double dEndX, double dEndY, const Tool& curTool)
{
	if (!requireInitializedConnection("JumpToIdleXYPosition"))
		return;
	long sts;
	Axis eDirectionX = magic_enum::enum_cast<Axis>(curTool.m_strDirectionX).value();
	Axis eDirectionY = magic_enum::enum_cast<Axis>(curTool.m_strDirectionY).value();
	short sRtn;
	
	//X 定位轴
	if (curTool.m_bXIsMove && eDirectionX != Axis::X && m_runtimeConfiguration.isAxisEnabled(Axis::X))
	{
		MovePostion(Axis::X, curTool.m_dIdleXVelocity, curTool.m_dXPosition);
	}
	//A 定位轴
	if (curTool.m_bAIsMove && eDirectionY != Axis::A && m_runtimeConfiguration.isAxisEnabled(Axis::A))
	{
		MovePostion(Axis::A, curTool.m_dIdleAVelocity, curTool.m_dAPosition / 360 * lcnc::process::kPi * m_dDiameter);
		
	}
	//Y 定位轴
	if (curTool.m_bYIsMove && eDirectionY != Axis::Y && m_runtimeConfiguration.isAxisEnabled(Axis::Y))
	{
		MovePostion(Axis::Y, curTool.m_dIdleYVelocity, curTool.m_dYPosition);
		
	}
	
	//空程起点坐标
	MovePostion(eDirectionX, GetAxisIdleVel(eDirectionX, curTool), dEndX);
	MovePostion(eDirectionY, GetAxisIdleVel(eDirectionY, curTool), dEndY);

	m_dPreX = dEndX;
	m_dPreY = dEndY;
}

bool GTNMotionControl::SendCommand()
{
	if (!requireInitializedConnection("SendCommand"))
		return false;
	if (!FlushToFifo())
		return false;

	// 若尚未启动插补运动，现在启动（小图形/未触发提前启动时）
	if (!m_bCrdStarted)
	{
		const short sRtn = GTN_CrdStart(m_iCore, 1, m_iWriteBuf);
		if (sRtn)
			return LogError("SendCommand", "GTN_CrdStart", "", sRtn), false;
		m_bCrdStarted = true;
	}
	LCNC_INFO(lcnc::LogCode::Generic,
		"gtn.api: operation=SendCommand api=GTN_CrdDataEx+GTN_CrdStart crd=1 fifo={} started={} result=0",
		m_iWriteBuf, m_bCrdStarted);
	return true;
}

// 将软件前瞻缓冲区数据刷入硬件 FIFO。
// GTN_CrdDataEx 返回非零时，先确认坐标系是否已经运行；未运行则尝试启动插补，
// 再在有界时间内重试。不要把特定 SDK 错误码臆断为“FIFO 已满”。
// 用于 OffsetLineTo/OffsetArcTo 检测到软件缓冲区满时调用，实现流式压数据。
bool GTNMotionControl::FlushToFifo()
{
	if (!requireInitializedConnection("FlushToFifo"))
		return false;
	short sRtn = 0;
	int attempt = 0;
	const auto deadline = std::chrono::steady_clock::now() + kFifoFlushTimeout;
	while (std::chrono::steady_clock::now() < deadline)
	{
		if (m_bStop)
		{
			LCNC_INFO(lcnc::LogCode::Generic,
				"gtn.api: operation=FlushToFifo phase=cancelled fifo={} attempts={}",
				m_iWriteBuf, attempt);
			return true;
		}
		++attempt;
		sRtn = GTN_CrdDataEx(m_iCore, 1, NULL, m_iWriteBuf);
		if (!sRtn)
		{
			if (attempt > 1)
				LCNC_INFO(lcnc::LogCode::Generic,
					"gtn.api: operation=FlushToFifo phase=complete fifo={} attempts={} started={} result=0",
					m_iWriteBuf, attempt, m_bCrdStarted);
			return true;
		}

		// GTN_CrdDataEx may report that the transfer queue needs consumption.
		// Start interpolation once, but never conceal a status/start failure.
		if (!m_bCrdStarted)
		{
			short run = 0;
			long segment = 0;
			const short statusResult = GTN_CrdStatus(
				m_iCore, 1, &run, &segment, m_iWriteBuf);
			if (statusResult != 0)
				return LogError("FlushToFifo", "GTN_CrdStatus", "", statusResult), false;
			if (run)
			{
				m_bCrdStarted = true;
				LCNC_INFO(lcnc::LogCode::Generic,
					"gtn.api: operation=FlushToFifo phase=detected_running fifo={} segment={} data_result={}",
					m_iWriteBuf, segment, sRtn);
			}
			else
			{
				const short startResult = GTN_CrdStart(m_iCore, 1, m_iWriteBuf);
				if (startResult != 0)
					return LogError("FlushToFifo", "GTN_CrdStart", "", startResult), false;
				m_bCrdStarted = true;
				LCNC_INFO(lcnc::LogCode::Generic,
					"gtn.api: operation=FlushToFifo phase=start_interpolation fifo={} segment={} data_result={} result=0",
					m_iWriteBuf, segment, sRtn);
			}
		}
		std::this_thread::sleep_for(kFifoFlushRetryInterval);
	}
	LCNC_ERR(lcnc::LogCode::Generic,
		"gtn.api: operation=FlushToFifo phase=timeout fifo={} attempts={} last_data_result={} timeout_ms={}",
		m_iWriteBuf, attempt, sRtn,
		std::chrono::duration_cast<std::chrono::milliseconds>(kFifoFlushTimeout).count());
	LogError("FlushToFifo", "GTN_CrdDataEx", "", sRtn);
	return false;
}

bool GTNMotionControl::InitCrd(const Tool& curTool)
{
	if (!requireInitializedConnection("InitCrd"))
		return false;
	m_bCrdStarted = false; // 每次重新初始化前瞻时，重置启动状态
	m_hasPreviousCuttingPose = false;
	m_cuttingCoordinateReady = false;
	short sRtn;
	short crd = 1, fifo = 0;
	LCNC_INFO(lcnc::LogCode::Generic,
		"gtn.api: operation=InitCrd phase=begin crd={} fifo={} dimension={} cut_smooth_time_ms={} cut_smooth_coefficient={} axis_smooth_time_ms={} axis_smooth_coefficient={}",
		crd, fifo, m_cuttingAxisCount, curTool.m_dCutSmoothTime,
		curTool.m_dCutSmoothK, curTool.m_dAxisSmoothTime, curTool.m_dAxisSmoothK);

	int axisIndex[5] = {};
	for (int dimension = 0; dimension < m_cuttingAxisCount; ++dimension) {
		const Axis axis = m_cuttingAxes[dimension];
		auto it = m_mapMotorValue.find(axis);
		if (it == m_mapMotorValue.end() || it->second.AxisIndex < 1 || it->second.AxisIndex > 8)
			return LogError("InitCrd", "five-axis cutting axis not configured", magic_enum::enum_name(axis).data(), -1), false;
		axisIndex[dimension] = it->second.AxisIndex;
	}
	for (int lhs = 0; lhs < m_cuttingAxisCount; ++lhs) {
		for (int rhs = lhs + 1; rhs < m_cuttingAxisCount; ++rhs) {
			if (axisIndex[lhs] == axisIndex[rhs])
				return LogError("InitCrd", "duplicate five-axis cutting axis", "", -1), false;
		}
	}
	// 确保创建前瞻前没有轴系运动；设备异常时不得无限占用全局租约。
	constexpr auto kCoordinateIdleTimeout = std::chrono::seconds(30);
	const auto idleResult = lcnc::process::waitForDeviceCondition(
		[this] { return !IsAxisMoving(); },
		[this] { return m_bStop.load(); },
		kCoordinateIdleTimeout,
		std::chrono::milliseconds(50));
	if (idleResult != lcnc::process::DeviceWaitStatus::Completed)
	{
		LogError("InitCrd",
			idleResult == lcnc::process::DeviceWaitStatus::Cancelled
				? "cancelled waiting for axes to stop"
				: "timeout waiting for axes to stop",
			"",
			-2);
		return false;
	}
	if (!clearCoordinateBufferIfInitialized("InitCrd"))
		return false;
	// 建立号坐标系，设置坐标系参数
	TCrdPrm crdPrm;
	memset(&crdPrm, 0, sizeof(crdPrm));
	//sRtn = GTN_GetCrdPrm(core,crd,&crdPrm);
	crdPrm.dimension = m_cuttingAxisCount;
	crdPrm.synVelMax = 500; // 最大合成速度：pulse/ms
	crdPrm.synAccMax = 10; // 最大加速度：pulse/ms^2
	crdPrm.evenTime = 50; // 最小匀速时间：ms
	for (int dimension = 0; dimension < m_cuttingAxisCount; ++dimension)
		crdPrm.profile[axisIndex[dimension] - 1] = dimension + 1;
	crdPrm.setOriginFlag = 1; // 通过originPos指定坐标系原点

	for (int dimension = 0; dimension < m_cuttingAxisCount; ++dimension)
		crdPrm.originPos[axisIndex[dimension] - 1] = 0;
	sRtn = GTN_SetCrdPrm(m_iCore, crd, &crdPrm);
	if(sRtn)
	{
		LCNC_ERR(lcnc::LogCode::Generic, "{}", fmt::format(
			"GTN_SetCrdPrm failed({}): core={} crd={} dimension={} profile=[{},{},{},{},{}] "
			"synVelMax={} synAccMax={} evenTime={} setOriginFlag={} "
			"originPos=[{},{},{},{},{}] axisIndex=[{},{},{},{},{}]",
			sRtn, m_iCore, crd,
			crdPrm.dimension, crdPrm.profile[0], crdPrm.profile[1], crdPrm.profile[2],
			crdPrm.profile[3], crdPrm.profile[4],
			crdPrm.synVelMax, crdPrm.synAccMax, crdPrm.evenTime, crdPrm.setOriginFlag,
			crdPrm.originPos[0], crdPrm.originPos[1], crdPrm.originPos[2],
			crdPrm.originPos[3], crdPrm.originPos[4],
			axisIndex[0], axisIndex[1], axisIndex[2], axisIndex[3], axisIndex[4]));
		return LogError("InitCrd", "GTN_SetCrdPrm", "", sRtn), false;
	}

	// 对齐C#示例：在SetupLookAheadCrd之前先清除坐标系缓冲区
	sRtn = GTN_CrdClear(m_iCore, crd, fifo);
	if (sRtn) return LogError("InitCrd", "GTN_CrdClear", "", sRtn), false;

	// ---- 初始化多轴前瞻模块 ----
	// 五轴机床模式：CAM 已提供五个轴的连续机床坐标，RTCP 保持关闭。
	int axisLimitMode[8] = {0};

	TLookAheadParameter lookAheadPara;
	memset(&lookAheadPara, 0, sizeof(lookAheadPara));
	lookAheadPara.lookAheadNum = 200;
	lookAheadPara.time = 0.01;
	lookAheadPara.radiusRatio = 0.5;
	for (Axis axis : m_vecMotors)
	{
		int iIndex = m_mapMotorValue[axis].AxisIndex - 1; // 0-based
		axisLimitMode[iIndex] = AXIS_LIMIT_MAX_VEL | AXIS_LIMIT_MAX_ACC | AXIS_LIMIT_MAX_DV;
		lookAheadPara.vMax[iIndex]  = 500;
		lookAheadPara.aMax[iIndex]  = 5000;
		lookAheadPara.DVMax[iIndex] = 300;
	}

	// 构建完整的 axisRelation[0..7]：保证所有8个槽位均非零且不重复。
	// 任何槽位为0都会导致 GTN_InitLookAheadEx 返回错误107。
	// 对齐C#示例：[1,2,3,4,5,6,7,8]（C#固定填满全部8个槽位）
	{
		bool used[8] = {};
		int slotIdx = 0;
		// 前五槽位严格对应坐标系 XYZ/R1/R2，保证 GTN_LnXYZACEx 的 pPos 顺序一致。
		for (int dimension = 0; dimension < m_cuttingAxisCount; ++dimension) {
			const int index = axisIndex[dimension];
			lookAheadPara.axisRelation[slotIdx++] = static_cast<short>(index);
			used[index - 1] = true;
		}
		// 剩余槽位填写其它已配置轴。
		for (Axis axis : m_vecMotors)
		{
			if (slotIdx >= 8) break;
			int idx = m_mapMotorValue[axis].AxisIndex;
			if (used[idx - 1]) continue;
			lookAheadPara.axisRelation[slotIdx++] = (short)idx;
			used[idx - 1] = true;
		}
		// 剩余空位：用1..8中尚未出现的轴号按升序填充
		for (int n = 1; n <= 8 && slotIdx < 8; n++)
		{
			if (!used[n - 1])
			{
				lookAheadPara.axisRelation[slotIdx++] = (short)n;
				used[n - 1] = true;
			}
		}
	}

	// scale 与 axisRelation 使用相同槽位语义：每个物理轴必须使用自己的
	// 脉冲当量。直线轴单位为 pulse/mm，旋转轴单位为 pulse/degree；不能把
	// X 轴分辨率复制给 B/C，否则非零旋转角会被控制器按错误比例执行。
	for (int slot = 0; slot < 8; ++slot)
	{
		const int physicalAxisIndex = lookAheadPara.axisRelation[slot];
		double resolution = 0.0;
		for (const auto& entry : m_mapMotorValue)
		{
			if (entry.second.AxisIndex == physicalAxisIndex)
			{
				resolution = static_cast<double>(entry.second.Resolution);
				break;
			}
		}
		// 未配置的填充槽不会参与运动，但 SDK 仍要求非零 scale。
		lookAheadPara.scale[slot] = resolution > 0.0 ? resolution : 1.0;
	}

	for (Axis axis : m_vecMotors)
	{
		int iCurAxisIdx = m_mapMotorValue[axis].AxisIndex;
		sRtn = GTN_SetAxisMotionSmooth(m_iCore, iCurAxisIdx, curTool.m_dAxisSmoothTime, curTool.m_dAxisSmoothK);
		if (sRtn)
			return LogError("SetContiInterpolation", "GTN_SetAxisMotionSmooth", "", sRtn), false;
	}

	// 对齐C#示例：SetupLookAheadCrd → InitLookAheadEx 之间不插入任何其它调用
	const EMachineMode machineMode = m_cuttingAxisCount == 3 ? NORMAL_THREE_AXIS
		: m_cuttingAxisCount == 4 ? MULTI_AXES : FIVE_AXIS;
	sRtn = GTN_SetupLookAheadCrd(m_iCore, crd, machineMode);
	if (sRtn) return LogError("SetContiInterpolation", "GTN_SetupLookAheadCrd", "", sRtn), false;

	sRtn = GTN_InitLookAheadEx(m_iCore, crd, &lookAheadPara, fifo, 0);
	if (sRtn) return LogError("SetContiInterpolation", "GTN_InitLookAheadEx", "", sRtn), false;
	m_coordinateBufferInitialized = true;

	// InitLookAheadEx 成功后再配置轴限制和速度有效模式
	sRtn = GTN_SetAxisLimitModeLa(m_iCore, crd, axisLimitMode);
	if (sRtn) return LogError("SetContiInterpolation", "GTN_SetAxisLimitModeLa", "", sRtn), false;
	long velValidMask = 0;
	for (int dimension = 0; dimension < m_cuttingAxisCount; ++dimension)
		velValidMask |= (1L << (axisIndex[dimension] - 1));
	sRtn = GTN_SetAxisVelValidModeLa(m_iCore, crd, velValidMask);
	if (sRtn) return LogError("SetContiInterpolation", "GTN_SetAxisVelValidModeLa", "", sRtn), false;

	sRtn = GTN_SetCrdJerkTime(m_iCore, 1, curTool.m_dCutSmoothTime, curTool.m_dCutSmoothK);
	if (sRtn) return LogError("SetContiInterpolation", "GTN_SetCrdJerkTime", "", sRtn), false;
	sRtn = GTN_CrdHsOn(m_iCore, 1, 0, 1, 300, 0);
	if (sRtn) return LogError("SetContiInterpolation", "GTN_CrdHsOn", "", sRtn), false;
	m_cuttingCoordinateReady = true;
	LCNC_INFO(lcnc::LogCode::Generic,
		"gtn.api: operation=InitCrd phase=complete crd={} fifo={} dimension={} axis_indices=[{},{},{},{},{}] result=0",
		crd, fifo, m_cuttingAxisCount, axisIndex[0], axisIndex[1], axisIndex[2],
		axisIndex[3], axisIndex[4]);
	return true;
}

bool GTNMotionControl::PrfTrapAxis()
{
	short sRtn;
	for (Axis axis : m_vecMotors)
	{
		// 将 AXIS 轴设为点位模式
		sRtn = GTN_PrfTrap(m_iCore, m_mapMotorValue[axis].AxisIndex);
		if (sRtn)return LogError("SetJumpAccJerk", "GTN_PrfTrap", magic_enum::enum_name(axis).data(), sRtn), false;

	}
	return true;
}

bool GTNMotionControl::SetJumpAccJerk(const Tool& curTool)
{
	// GTN Trap has no jerk parameter. The tool supplies acceleration while
	// every axis retains its configured point-motion smooth time.
	short sRtn;
	for (Axis axis : m_vecMotors)
	{
		// 将 AXIS 轴设为点位模式
		sRtn = GTN_PrfTrap(m_iCore, m_mapMotorValue[axis].AxisIndex);
		if (sRtn)return LogError("SetJumpAccJerk", "GTN_PrfTrap", magic_enum::enum_name(axis).data(), sRtn), false;
		TTrapPrm trap;
		double dAccx ;
		double dDecx ;
		MillimeterToPulse(axis, curTool.m_dIdleXYAccDec / 1000000.00, dAccx);
		MillimeterToPulse(axis, curTool.m_dIdleXYAccDec / 1000000.00, dDecx);
		trap.acc = dAccx;
		trap.dec = dDecx;
		trap.velStart = 0;
		trap.smoothTime = static_cast<short>(m_mapMotorValue[axis].SmoothTime);
		sRtn = GTN_SetTrapPrm(m_iCore, m_mapMotorValue[axis].AxisIndex, &trap);
		if (sRtn)return LogError("SetJumpAccJerk", "GTN_SetTrapPrm", magic_enum::enum_name(axis).data(), sRtn), false;
		LCNC_INFO(lcnc::LogCode::Generic,
			"gtn.api: operation=SetJumpMotionParameters api=GTN_SetTrapPrm axis={} acceleration={} smooth_time_ms={} result=0",
			magic_enum::enum_name(axis), curTool.m_dIdleXYAccDec,
			m_mapMotorValue[axis].SmoothTime);
	}

	return true;
}

void GTNMotionControl::ProLaserControl(bool bLaser, bool bPso, const Tool& curTool, bool bAOUTFlag)
{
	if (bAOUTFlag) {
		m_bErrorOccurred = true;
		LogError("ProLaserControl", "native laser control is unsupported; configure digital output", "", -1);
		return;
	}
	if (!requireInitializedConnection("ProLaserControl"))
		return;
	short sRtn;
	string strLaserNum	= m_mapDigitalOUT[DigitalOUT::Laser].strIndex;
	string strAnalogNum = m_mapAnalogOUT[AnalogOUT::Laser].strIndex;
	string strEnd = "\n";

	if (bLaser)
	{
		string strLaserOnBWait = boost::lexical_cast<string>(m_dLaserOnBWait);
		string strLaserOnAWait = boost::lexical_cast<string>(m_dLaserOnAWait);
		string strBlowDelay = boost::lexical_cast<string>(m_dBlowDelay);

		if (curTool.m_bBlow2)
		{
			int ivalue;
			//一路气关气
			if (!m_mapDigitalOUT[DigitalOUT::Blow].bExpand)
			{
				ivalue = m_mapDigitalOUT[DigitalOUT::Blow].bInversion ? 0 : 1;
				sRtn = GTN_BufDoBitEx(m_iCore, 1, MC_GPO, m_mapDigitalOUT[DigitalOUT::Blow].iIO, ivalue, m_iWriteBuf);
				if (sRtn) LogError("ProLaserControl", "GTN_BufDoBitEx_Blow","", sRtn);
			}
			else
			{
				ivalue = m_mapDigitalOUT[DigitalOUT::Blow].bInversion ? 1 : 0;
				sRtn = GTN_BufExtDoBitEx(m_iCore, 1, m_mapDigitalOUT[DigitalOUT::Blow].iIO, ivalue, m_iWriteBuf);
				if (sRtn) LogError("ProLaserControl", "GTN_BufDoBitEx_Expand_Blow", "", sRtn);
			}
			//开气
			if (!m_mapDigitalOUT[DigitalOUT::Blow2].bExpand)
			{
				ivalue = m_mapDigitalOUT[DigitalOUT::Blow2].bInversion ? 1 : 0;
				sRtn = GTN_BufDoBitEx(m_iCore, 1, MC_GPO, m_mapDigitalOUT[DigitalOUT::Blow2].iIO, ivalue, m_iWriteBuf);
				if (sRtn) LogError("ProLaserControl", "GTN_BufDoBitEx_Blow2", "", sRtn);
			}
			else
			{
				ivalue = m_mapDigitalOUT[DigitalOUT::Blow2].bInversion ? 0 : 1;
				sRtn = GTN_BufExtDoBitEx(m_iCore, 1, m_mapDigitalOUT[DigitalOUT::Blow2].iIO, ivalue, m_iWriteBuf);
				if (sRtn) LogError("ProLaserControl", "GTN_BufDoBitEx_Expand_Blow2", "", sRtn);
			}
		}
		else
		{
			int ivalue;
			//二路气关气
			if (!m_mapDigitalOUT[DigitalOUT::Blow2].bExpand)
			{
				ivalue = m_mapDigitalOUT[DigitalOUT::Blow2].bInversion ? 0 : 1;
				sRtn = GTN_BufDoBitEx(m_iCore, 1, MC_GPO, m_mapDigitalOUT[DigitalOUT::Blow2].iIO, ivalue, m_iWriteBuf);
				if (sRtn) LogError("ProLaserControl", "GTN_BufDoBitEx_Blow2", "", sRtn);
			}
			else
			{
				ivalue = m_mapDigitalOUT[DigitalOUT::Blow2].bInversion ? 1 : 0;
				sRtn = GTN_BufExtDoBitEx(m_iCore, 1, m_mapDigitalOUT[DigitalOUT::Blow2].iIO, ivalue, m_iWriteBuf);
				if (sRtn) LogError("ProLaserControl", "GTN_BufDoBitEx_Expand_Blow2", "", sRtn);
			}
			//开气
			if (!m_mapDigitalOUT[DigitalOUT::Blow].bExpand)
			{
				ivalue = m_mapDigitalOUT[DigitalOUT::Blow].bInversion ? 1 : 0;
				sRtn = GTN_BufDoBitEx(m_iCore, 1, MC_GPO, m_mapDigitalOUT[DigitalOUT::Blow].iIO, ivalue, m_iWriteBuf);
				if (sRtn) LogError("ProLaserControl", "GTN_BufDoBitEx_Blow", "", sRtn);
			}
			else
			{
				ivalue = m_mapDigitalOUT[DigitalOUT::Blow].bInversion ? 0 : 1;
				sRtn = GTN_BufExtDoBitEx(m_iCore, 1, m_mapDigitalOUT[DigitalOUT::Blow].iIO, ivalue, m_iWriteBuf);
				if (sRtn) LogError("ProLaserControl", "GTN_BufDoBitEx_Expand_Blow", "", sRtn);
			}
		}
		if (m_dBlowDelay >= 1)
		{
			sRtn = GTN_BufDelayEx(m_iCore, 1, m_dBlowDelay, m_iWriteBuf);
			if (sRtn) LogError("ProLaserControl", "GTN_BufDelayEx", "", sRtn);
		}
		
		if (m_dLaserOffBWait >= 1)
		{
			sRtn = GTN_BufDelayEx(m_iCore, 1, m_dLaserOffBWait, m_iWriteBuf);
			if (sRtn) LogError("ProLaserControl", "GTN_BufDelayEx_OffBWait","", sRtn);
		}

		{
			int ivalue;
			if (!m_mapDigitalOUT[DigitalOUT::Laser].bExpand)
			{
				ivalue = m_mapDigitalOUT[DigitalOUT::Laser].bInversion ? 1 : 0;
				sRtn = GTN_BufDoBitEx(m_iCore, 1, MC_GPO, m_mapDigitalOUT[DigitalOUT::Laser].iIO, ivalue, m_iWriteBuf);
				if (sRtn) LogError("ProLaserControl", "GTN_BufDoBitEx_Laser", "", sRtn);
			}
			else
			{
				ivalue = m_mapDigitalOUT[DigitalOUT::Laser].bInversion ? 0 : 1;
				sRtn = GTN_BufExtDoBitEx(m_iCore, 1, m_mapDigitalOUT[DigitalOUT::Laser].iIO, ivalue, m_iWriteBuf);
				if (sRtn) LogError("ProLaserControl", "GTN_BufExtDoBitEx_Laser", "", sRtn);
			}

		}
		if (m_dLaserOnAWait >= 1)
		{
			sRtn = GTN_BufDelayEx(m_iCore, 1, m_dLaserOnAWait, m_iWriteBuf);
			if (sRtn) LogError("ProLaserControl", "GTN_BufDelayEx_OnAWait", "", sRtn);
		}
	}
	else
	{
		if (m_dLaserOffBWait >= 1)
		{
			sRtn = GTN_BufDelayEx(m_iCore, 1, m_dLaserOffBWait, m_iWriteBuf);
			if (sRtn) LogError("ProLaserControl", "GTN_BufDelayEx_OffBWait", "", sRtn);
		}

		{
			int iValue;
			if (!m_mapDigitalOUT[DigitalOUT::Laser].bExpand)
			{
				iValue = m_mapDigitalOUT[DigitalOUT::Laser].bInversion ? 0 : 1;
				sRtn = GTN_BufDoBitEx(m_iCore, 1, MC_GPO, m_mapDigitalOUT[DigitalOUT::Laser].iIO, iValue, m_iWriteBuf);
				if (sRtn) LogError("ProLaserControl", "GTN_BufDoBitEx_Laser", "", sRtn);
			}
			else
			{
				iValue = m_mapDigitalOUT[DigitalOUT::Laser].bInversion ? 1 : 0;
				sRtn = GTN_BufExtDoBitEx(m_iCore, 1, m_mapDigitalOUT[DigitalOUT::Laser].iIO, iValue, m_iWriteBuf);
				if (sRtn) LogError("ProLaserControl", "GTN_BufExtDoBitEx_Laser", "", sRtn);
			}
		}

		if (m_dLaserOffAWait >= 1)
		{
			sRtn = GTN_BufDelayEx(m_iCore, 1, m_dLaserOffAWait, m_iWriteBuf);
			if (sRtn) LogError("ProLaserControl", "GTN_BufDelayEx_OffAWait", "", sRtn);
		}

		//切割完是否关气没有编写
	}
}

bool GTNMotionControl::IsAxisStatusNormal(int& iFault)
{
	if (!m_connectionInitialized) {
		iFault = -1;
		return false;
	}
	iFault = 0;
	for (Axis axis : m_vecMotors)
	{
		const auto motor = m_mapMotorValue.find(axis);
		if (motor == m_mapMotorValue.end())
			return false;
		long status = 0;
		const short result = GTN_GetSts(m_iCore, motor->second.AxisIndex, &status);
		if (result != 0)
		{
			LogError("IsAxisStatusNormal", "GTN_GetSts",
				magic_enum::enum_name(axis).data(), result);
			return false;
		}
		TLimitInfo limitInfo{};
		if ((status & kGtnLimitMask) != 0)
		{
			const short limitResult = GTN_GetLimitInfo(
				m_iCore, motor->second.AxisIndex, &limitInfo);
			if (limitResult != 0)
			{
				LogError("IsAxisStatusNormal", "GTN_GetLimitInfo",
					magic_enum::enum_name(axis).data(), limitResult);
				return false;
			}
		}

		const long fault = classifyGtnAxisFault(
			status,
			limitInfo.hwLmtPositiveStatus != 0,
			limitInfo.hwLmtNegativeStatus != 0,
			limitInfo.swLmtPositiveStatus != 0,
			limitInfo.swLmtNegativeStatus != 0);
		const long softLimit = (status & kGtnLimitMask) & ~fault;
		if (softLimit != 0)
		{
			LCNC_WARN(lcnc::LogCode::Generic,
				"gtn.api: operation=IsAxisStatusNormal axis={} status=0x{:x} soft_limit_mask=0x{:x} hw_limit_positive={} hw_limit_negative={} sw_limit_positive={} sw_limit_negative={} result=soft_limit_active action=allow_escape_motion",
				magic_enum::enum_name(axis), status, softLimit,
				limitInfo.hwLmtPositiveStatus, limitInfo.hwLmtNegativeStatus,
				limitInfo.swLmtPositiveStatus, limitInfo.swLmtNegativeStatus);
		}
		if (fault != 0)
		{
			iFault |= static_cast<int>(fault);
			LCNC_ERR(lcnc::LogCode::Generic,
				"gtn.api: operation=IsAxisStatusNormal axis={} status=0x{:x} fault_mask=0x{:x} result=fault",
				magic_enum::enum_name(axis), status, fault);
		}
	}
	return true;
}

double GTNMotionControl::GetAxisIdleVel(Axis eAxis, const Tool& curTool)
{
	switch (eAxis)
	{
	case Axis::X:	return curTool.m_dIdleXVelocity;
	case Axis::Y:	return curTool.m_dIdleYVelocity;
	case Axis::Z:	return curTool.m_dIdleZVelocity;
	case Axis::A:	return curTool.m_dIdleAVelocity;	//临时
	default:	return curTool.m_dIdleX1Velocity;
	}
}

// [P3 removed] GTNMotionControl::SetFPos

// [P3 removed] GTNMotionControl::GetFPos


bool GTNMotionControl::MovePostion(Axis aAxis, double dVel, double dPos)
{
	if (!requireInitializedConnection("MovePostion"))
		return false;
	short sRtn;
	long sts;
	double dNewVel,dNewPos;
	MillimeterToPulse(aAxis, dVel / 1000.00, dNewVel);
	MillimeterToPulse(aAxis, dPos, dNewPos);
	sRtn = GTN_PrfTrap(m_iCore, m_mapMotorValue[aAxis].AxisIndex);
	if (sRtn) return LogError("MovePostion", "GTN_PrfTrap", magic_enum::enum_name(aAxis).data(), sRtn), false;
	sRtn = GTN_SetVel(m_iCore, m_mapMotorValue[aAxis].AxisIndex, dNewVel);
	if (sRtn) return LogError("MovePostion", "GTN_SetVel", magic_enum::enum_name(aAxis).data(), sRtn), false;

	sRtn = GTN_SetPos(m_iCore, m_mapMotorValue[aAxis].AxisIndex, (long)dNewPos);
	if (sRtn) return LogError("MovePostion", "GTN_SetPos", magic_enum::enum_name(aAxis).data(), sRtn), false;

	long mask = AxisMask(aAxis);
	if (!mask)
		return LogError("MovePostion", "AxisMask", magic_enum::enum_name(aAxis).data(), -1), false;
	sRtn = GTN_Update(m_iCore, mask);//启动轴运动
	if (sRtn) return LogError("MovePostion", "GTN_Update", magic_enum::enum_name(aAxis).data(), sRtn), false;

	bool statusReadFailed = false;
	constexpr auto kMoveTimeout = std::chrono::minutes(2);
	const auto moveResult = lcnc::process::waitForDeviceCondition(
		[this, aAxis, &sts, &statusReadFailed] {
			const short statusResult = GTN_GetSts(
				m_iCore, m_mapMotorValue[aAxis].AxisIndex, &sts);
			if (statusResult != 0)
			{
				statusReadFailed = true;
				LogError("MovePostion", "GTN_GetSts",
					magic_enum::enum_name(aAxis).data(), statusResult);
				return true;
			}
			return (sts & 0x400) == 0;
		},
		[this] { return m_bStop.load(); },
		kMoveTimeout,
		std::chrono::milliseconds(10));
	if (statusReadFailed || moveResult != lcnc::process::DeviceWaitStatus::Completed)
	{
		if (!statusReadFailed && moveResult == lcnc::process::DeviceWaitStatus::TimedOut)
			LogError("MovePostion", "axis motion timeout",
				magic_enum::enum_name(aAxis).data(), -2);
		return false;
	}
	return true;
}

bool GTNMotionControl::PulseToMillimeter(Axis aAxis, double dValue, double& dNewValue)
{
	const double resolution = static_cast<double>(m_mapMotorValue[aAxis].Resolution);
	if (!std::isfinite(resolution) || resolution <= 0.0)
		return LogError("PulseToMillimeter", "invalid axis resolution",
			magic_enum::enum_name(aAxis).data(), -1), false;
	// Process coordinates use native machine units: mm for linear axes and
	// degrees for rotary axes. Tube diameter is CAM geometry and must never
	// change the controller coordinate unit of a B/C/A axis.
	dNewValue = dValue / resolution;
	return true;
}

bool GTNMotionControl::MillimeterToPulse(Axis aAxis, double dValue, double& dNewValue)
{
	const double resolution = static_cast<double>(m_mapMotorValue[aAxis].Resolution);
	if (!std::isfinite(resolution) || resolution <= 0.0)
		return LogError("MillimeterToPulse", "invalid axis resolution",
			magic_enum::enum_name(aAxis).data(), -1), false;
	dNewValue = dValue * resolution;
	return true;
}

// [P3 removed] GTNMotionControl::GetCuttingCommand

// [P3 removed] GTNMotionControl::SetMFLAGSValue

// [P3 removed] GTNMotionControl::GSN_SetLaserParameterApplication

// [P3 removed] GTNMotionControl::GSN_SetLaserEnablePro

// [P3 removed] GTNMotionControl::GSN_LaserOnStatus

// [P3 removed] GTNMotionControl::CheckBuffer

// [P3 removed] GTNMotionControl::RunBuffer

bool GTNMotionControl::PauseBuffer(int iBufferIndex)
{
	short sRtn;
	sRtn = GTN_Stop(m_iCore, 0xff, 0x0);
	if (sRtn) return LogError("PauseBuffer", "GTN_Stop","", sRtn), false;
	return true;
}

// [P3 removed] GTNMotionControl::GetBufferState

// [P3 removed] GTNMotionControl::LoadCommandAndRunBuffer

bool GTNMotionControl::IsReachPos(Axis eAxis, bool bRelative, double dPos)
{
	short sRtn;
	if (bRelative)	//相对
	{
		double dNowPos,dPlusePos;
		sRtn = GTN_GetPrfPos(m_iCore, m_mapMotorValue[eAxis].AxisIndex, &dPlusePos);//规划位置
		PulseToMillimeter(eAxis, dPlusePos, dNowPos);
		if (sRtn != 0)
			return LogError("IsReachPos", "GTN_GetPrfPos","", sRtn), false;
		if (((dNowPos + dPos) >= m_mapMotorValue[eAxis].NegLimit) && ((dNowPos + dPos) <= m_mapMotorValue[eAxis].PosLimit))
			return true;
	}
	else			//绝对
	{
		if ((dPos >= m_mapMotorValue[eAxis].NegLimit) && (dPos <= m_mapMotorValue[eAxis].PosLimit))
			return true;
	}
	return false;
}

void GTNMotionControl::ClearAxisState()
{
	// Preserve the compatibility entry, but never issue unchecked axis-wide
	// clears (the former connection test also returned early when connected).
	(void)RecoverAfterStop();
}

bool GTNMotionControl::IsMachiningStatusNormal(int& fault, bool reportFaults)
{
	fault = 0;
	if (!m_bConnectFlag || !m_connectionInitialized || m_mapMotorValue.empty())
		return false;
	for (const auto& entry : m_mapMotorValue) {
		const auto& motor = entry.second;
		long status = 0;
		TLimitInfo limit{};
		short result = GTN_GetSts(m_iCore, motor.AxisIndex, &status);
		if (result == 0)
			result = GTN_GetLimitInfo(m_iCore, motor.AxisIndex, &limit);
		if (result != 0)
			return LogError("IsMachiningStatusNormal", "GTN_GetSts+GTN_GetLimitInfo",
				motor.Name, result), false;
		// A latched/unclassified limit status is not safe machining admission.
		// Only explicit Reset may clear it and then re-read the live inputs.
		const long axisFault = lcnc::process::gtnMachiningFault(status,
			limit.hwLmtPositiveStatus != 0, limit.hwLmtNegativeStatus != 0,
			limit.swLmtPositiveStatus != 0, limit.swLmtNegativeStatus != 0);
		fault |= static_cast<int>(axisFault);
		if (axisFault != 0 && reportFaults) {
			LCNC_ERR(lcnc::LogCode::Generic,
				"gtn.api: operation=IsMachiningStatusNormal axis={} index={} status=0x{:x} fault=0x{:x} hw_limit=[{},{}] sw_limit=[{},{}] action=reject_machining result=-1",
				motor.Name, motor.AxisIndex, status, axisFault,
				limit.hwLmtPositiveStatus, limit.hwLmtNegativeStatus,
				limit.swLmtPositiveStatus, limit.swLmtNegativeStatus);
		}
	}
	return true;
}

bool GTNMotionControl::RecoverAfterStop()
{
	LCNC_INFO(lcnc::LogCode::Generic,
		"gtn.api: operation=RecoverAfterStop phase=begin previous_error='{}' stop_fault_latched={} feedback_fault_latched={}",
		m_groupExecutionError.toStdString(), m_stopFaultLatched, m_groupFeedbackFaultLatched);
	// Keep history in the log and the dedicated first-feedback snapshot, but
	// report this recovery attempt's current failure rather than an old message.
	m_groupExecutionError.clear();
	// Do not use requireInitializedConnection here: a previous stop failure
	// may be recovered by a NEW confirmed stop, but never by clearing its flag.
	if (!m_bConnectFlag || !m_connectionInitialized) {
		m_bErrorOccurred = true;
		return LogError("RecoverAfterStop", "connection initialization incomplete", "", -1), false;
	}
	const long mask = configuredAxisMask();
	logAxisPositionEvidence(mask, "RecoverAfterStop(before-clear)");
	if (m_groupFeedbackFaultLatched) {
		m_bErrorOccurred = true;
		return LogError("RecoverAfterStop", "feedback fault requires verified repair and reconnect", "", -1), false;
	}
	const auto checked = [this](const char* api, short result) {
		LCNC_INFO(lcnc::LogCode::Generic,
			"gtn.api: operation=RecoverAfterStop api={} group={} list={} result={}",
			api, m_groupIndex, m_commandListIndex, result);
		if (result == 0) return true;
		LogError("RecoverAfterStop", api, "", result);
		return false;
	};
	TListInfo immediate{};
	const bool group = UsesGroupArchitecture();
	const auto recovered = lcnc::process::recoverStoppedController(
		[&] {
			if (!stopConfiguredOutputs("RecoverAfterStop")) return false;
			if (group) {
				if (!stopAndReleaseFiveAxisGroup(mask, "RecoverAfterStop", true)) return false;
				clearGroupRuntimeState();
			} else if (!waitForStoppedMotion(mask, false, "RecoverAfterStop")) {
				return false;
			}
			return clearCoordinateBufferIfInitialized("RecoverAfterStop");
		},
		[&] {
			for (const auto& entry : m_mapMotorValue) {
				if (!checked("GTN_ClrSts(axis)",
						GTN_ClrSts(m_iCore, entry.second.AxisIndex, 1))) return false;
			}
			return !group || checked("GTN_ClearGroupStatus",
				GTN_ClearGroupStatus(m_iCore, m_groupIndex, &immediate));
		},
		[&] {
			if (!waitForStoppedMotion(mask, group, "RecoverAfterStop(after-clear)")) return false;
			int fault = 0;
			const bool healthy = lcnc::process::verifyRecoveredControllerHealth([&] {
				if (!IsMachiningStatusNormal(fault, false))
					return lcnc::process::ControllerHealthSample::ReadFailed;
				return fault == 0 ? lcnc::process::ControllerHealthSample::Clean
					: lcnc::process::ControllerHealthSample::FaultActive;
			});
			if (!healthy) {
				LCNC_ERR(lcnc::LogCode::Generic,
					"gtn.api: operation=RecoverAfterStop phase=current_health fault={} stable_healthy=false result=-1", fault);
				LogError("RecoverAfterStop", "current controller health not verified", "",
					fault != 0 ? fault : -1);
				return false;
			}
			if (!validateStationaryFeedback(mask, "RecoverAfterStop", true)) {
				// Preserve actual mismatch evidence. An SDK read error does not
				// manufacture a direction fault, but still prevents this reset.
				m_groupFeedbackFaultLatched = m_feedbackFault.captured;
				return false;
			}
			if (group) {
				TCommandListStatus list{};
				TGroupStatus state{};
				if (!checked("GTN_GetCommandListStatus(readback)",
						GTN_GetCommandListStatus(m_iCore, m_commandListIndex, &list))
					|| !checked("GTN_GetGroupStatus(readback)",
						GTN_GetGroupStatus(m_iCore, m_groupIndex, &state))) return false;
				const bool clean = list.execute == 0 && list.stopInfo == 0
					&& list.remainderSegCount == 0 && state.run == 0 && state.stopInfo == 0
					&& (state.state == GROUP_STATE_DISABLED || state.state == GROUP_STATE_STANDBY);
				LCNC_INFO(lcnc::LogCode::Generic,
					"gtn.api: operation=RecoverAfterStop phase=readback list_execute={} list_stop={} remaining={} group_run={} group_state={} group_stop={} clean={}",
					list.execute, list.stopInfo, list.remainderSegCount,
					state.run, state.state, state.stopInfo, clean);
				if (!clean)
					return LogError("RecoverAfterStop", "Group/CommandList readback is not clean", "", -1), false;
			}
			return true;
		},
		[this] {
			m_stopFaultLatched = false;
			m_bErrorOccurred = false;
			m_groupExecutionError.clear();
		});
	const bool success = recovered == lcnc::process::ControllerRecoveryResult::Recovered;
	if (!success) m_bErrorOccurred = true;
	LCNC_INFO(lcnc::LogCode::Generic,
		"gtn.api: operation=RecoverAfterStop phase=complete stage={} stop_fault_latched={} feedback_fault_latched={} success={}",
		static_cast<int>(recovered), m_stopFaultLatched, m_groupFeedbackFaultLatched, success);
	return success;
}

void GTNMotionControl::StartCommand()
{
	if (!requireInitializedConnection("StartCommand"))
		return;
	short sRtn = GTN_CrdStart(m_iCore, 1, m_iWriteBuf);
	if (sRtn)
		LogError("StartCommand", "GTN_CrdStart", "", sRtn);
}

// Sink 入口：GTN 路径上 ResetProgramCommand 是 no-op，因为缓冲区的清空通过
// CrdClear/CrdData 在 InitCrd / SendCommand 中天然发生（详见 SendCommand 实现）。
void GTNMotionControl::ResetProgramCommand()
{
}

// Sink 入口：把工具的切割加速度/Jerk 推到坐标系前瞻 —— GTN 通过 GTN_SetCrdJerkTime
// 已在 InitCrd 里设置一次，每段刀路本身的 ACC/JERK 由 GTN_LnXYZACEx 内部按已建立的轨迹规划处理，
// 这里保留方法签名以匹配 IMotionCommandSink::applyToolMotionParams 调用入口，no-op 即可。
void GTNMotionControl::SetCuttingAccJerk(const Tool& tool)
{
	Q_UNUSED(tool);
}
