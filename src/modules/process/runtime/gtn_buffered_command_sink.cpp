#include "modules/process/runtime/gtn_buffered_command_sink.h"

#include <QCoreApplication>

#include "core/logging/logger.h"
#include "modules/process/tool/tool.h"
#include "modules/process/device/motion_control/gtn_motion_control.h"
#include "modules/process/runtime/process_interrupt_context.h"
#include "modules/process/runtime/gtn_exact_plan_lowering.h"

#include "magic_enum.hpp"

#include <QThread>

#include <utility>

namespace lcnc::process {

GtnBufferedCommandSink::GtnBufferedCommandSink(GTNMotionControl* gtn, AxisMap axisMap)
    : m_gtn(gtn)
    , m_axisMap(std::move(axisMap))
{
	if (!m_gtn)
		return;

	std::array<Axis, 5> configuredAxes{Axis::X, Axis::Y, Axis::Z, Axis::A, Axis::C};
	static constexpr AxisMap::SemanticAxis semanticAxes[5] = {
		AxisMap::X, AxisMap::Y, AxisMap::Z, AxisMap::R1, AxisMap::R2};
	for (int index = 0; index < m_axisMap.activeCount(); ++index) {
		const auto value = magic_enum::enum_cast<Axis>(m_axisMap.axisName(semanticAxes[index]).toStdString());
		if (!value) return;
		configuredAxes[index] = *value;
	}
	m_gtn->ConfigureCuttingAxes(configuredAxes, m_axisMap.activeCount());
}

void GtnBufferedCommandSink::resetProgram()
{
    m_exactProgram.reset();
    m_exactSection = -1;
    m_bufferCommandFailed = false;
    if (!m_gtn)
        return;
    if (m_groupProgramActive) {
        // resetProgram() is a batch boundary. A Group must be disabled and its
        // axes ungrouped before a later point move or a new Group is created.
        // 中文翻译：resetProgram 是批次边界，必须先完整释放 Group 轴所有权。
        releaseActiveGroup(nullptr, "reset_program");
    } else if (!m_gtn->UsesGroupArchitecture()) {
        m_gtn->ResetProgramCommand();
    }
}

bool GtnBufferedCommandSink::prepareExactSection(
    const PreparedDeviceProgram& program, int ordinal, QString* error)
{
    m_exactSection = -1;
    const auto fail = [&](const char* reason) {
        m_exactProgram.reset();
        if (error) *error = QString::fromLatin1(reason);
        return false;
    };
    if (!program.realMachine() || !m_gtn || ordinal < 0 || ordinal >= program.sections().size())
        return fail("GTN exact device preparation requires a real prepared program and valid section");
    if (m_token && m_token->isStopping())
        return fail("GTN exact preparation cancelled");
    const auto& profile = program.recipe().gtnLowering;
    // AxisMap currently stores layout-order slots, despite its legacy semantic
    // enum names. Compare through explicit physical indices; never reinterpret.
    if (m_axisMap.activeCount() != 5)
        return fail("GTN exact device axis mapping is unavailable");
    for (const auto& axis : profile.axes) {
        if (axis.physicalIndex < 0 || axis.physicalIndex >= 5)
            return fail("GTN exact physical axis index is invalid");
        const auto index = static_cast<AxisMap::SemanticAxis>(axis.physicalIndex);
        if (m_axisMap.axisName(index) != axis.name || m_axisMap.controllerIndex(index) != axis.controllerAxis)
            return fail("GTN exact device axis mapping differs from frozen qualification");
    }
    if (m_exactProgram) {
        const auto& first = *m_exactProgram->sections().front();
        if (first.planHash() != program.plan().planHash || first.contextHash() != program.plan().contextHash
            || first.recipeRevision() != program.recipe().revision || first.runEpoch() != program.runEpoch())
            return fail("GTN exact prepared program changed within the sink lifetime");
    } else {
        // No legacy GroupLineTo: it filters duplicate targets and reads Tool.
        // Compile the entire host list atomically before exposing any section.
        m_exactProgram = GtnEncodedProgram::lower(program,
            [this](const auto& target, const auto& physical, QString* validationError) {
                if (m_gtn->IsGroupRtcpActive() && m_gtn->ValidateGroupRtcpTarget(target, physical)) return true;
                if (validationError) *validationError = QStringLiteral("GTN RTCP Group target validation failed");
                return false;
            }, [this] { return m_token && m_token->isStopping(); }, error);
        if (!m_exactProgram) return false;
    }
    m_exactSection = ordinal;
    return true;
}

bool GtnBufferedCommandSink::startExactSection(const PreparedDeviceProgram&, int, QString* error)
{
    // S2 seals the immutable host encoding. S3 must bind it to qualified
    // Group/IO/profile admission and finite-list submission on this same queue.
    // Never adapt this to legacy startProgram(), whose buffer has another truth.
    if (error) *error = QStringLiteral("GTN exact Group submission lifecycle is unavailable (B2.S3)");
    return false;
}

bool GtnBufferedCommandSink::releaseActiveGroup(QString* errorMessage,
                                                 const char* reason)
{
    if (!m_gtn || !m_groupProgramActive)
        return true;

    LCNC_INFO(lcnc::LogCode::Generic,
              "gtn.api: operation=GtnBufferedCommandSink phase=release_begin reason={} result=pending",
              reason ? reason : "unspecified");
    const bool released = m_gtn->StopFiveAxisGroupProgram();
    if (released)
        m_groupProgramActive = false;
    else
        m_bufferCommandFailed = true;
    LCNC_INFO(lcnc::LogCode::Generic,
              "gtn.api: operation=GtnBufferedCommandSink phase=release_end reason={} released={} result={}",
              reason ? reason : "unspecified", released, released ? 0 : -1);
    if (!released && errorMessage) {
        // 中文翻译：GTN 在执行后释放五轴 Group 失败
        *errorMessage = QCoreApplication::translate(
            "GtnBufferedCommandSink",
            "GTN failed to release the five-axis Group after execution");
    }
    return released;
}

bool GtnBufferedCommandSink::flush(QString* errorMessage)
{
    if (!startProgram(errorMessage)) {
        releaseActiveGroup(nullptr, "start_failed");
        return false;
    }
    while (isProgramRunning(errorMessage)) {
        if (m_token) {
            while (m_token->isPaused())
                QThread::msleep(10);
            if (m_token->isStopping()) {
                // 中文翻译：切割已被中断
                if (errorMessage) *errorMessage = QStringLiteral("Cutting has been interrupted");
                releaseActiveGroup(nullptr, "interrupted");
                return false;
            }
        }
        QThread::msleep(10);
    }
    if (m_bufferCommandFailed || (errorMessage && !errorMessage->isEmpty()))
        return false;
    return true;
}

bool GtnBufferedCommandSink::startProgram(QString* errorMessage)
{
    if (!m_gtn) {
        if (errorMessage) *errorMessage = QStringLiteral("GtnBufferedCommandSink: motion control not bound");
        return false;
    }
    if (m_bufferCommandFailed) {
        // 中文翻译：GTN 缓冲指令在提交前失败
        if (errorMessage) *errorMessage = QCoreApplication::translate(
            "GtnBufferedCommandSink", "A GTN buffered command failed before submission");
        releaseActiveGroup(nullptr, "buffer_command_failed");
        return false;
    }
    const bool groupArchitectureConfigured = m_gtn->UsesGroupArchitecture();
    if (groupArchitectureConfigured && !m_groupProgramActive) {
        if (errorMessage) {
            // 中文翻译：GTN 五轴 Group 尚未初始化
            *errorMessage = QCoreApplication::translate(
                "GtnBufferedCommandSink", "GTN five-axis Group is not initialized");
        }
        return false;
    }
    const bool started = m_groupProgramActive
        ? m_gtn->StartFiveAxisGroupProgram() : m_gtn->SendCommand();
    if (!started) {
        // 中文翻译：GTN 批处理程序启动失败
        if (errorMessage) *errorMessage = !m_gtn->GroupExecutionError().isEmpty()
            ? m_gtn->GroupExecutionError() : QCoreApplication::translate(
            "GtnBufferedCommandSink", "GTN batch program start failed");
        LCNC_ERR(lcnc::LogCode::Generic,
                 "gtn.api: operation=GtnBufferedCommandSink phase=start result=-1");
        releaseActiveGroup(nullptr, "start_failed");
        return false;
    }
    // Legacy FIFO hands the profiles back to point mode after submission.
    // Group owns its profiles until the list has completed or been stopped.
    if (!m_groupProgramActive)
        m_gtn->PrfTrapAxis();
    return true;
}

bool GtnBufferedCommandSink::isProgramRunning(QString* errorMessage)
{
    if (!m_gtn) {
        if (errorMessage) *errorMessage = QStringLiteral("GtnBufferedCommandSink: motion control not bound");
        return false;
    }
    if (!m_groupProgramActive)
        return m_gtn->UsesGroupArchitecture() ? false : m_gtn->IsAxisMoving();

    const bool running = m_gtn->IsFiveAxisGroupProgramRunning();
    if (running)
        return true;

    // Capture the execution state before StopFiveAxisGroupProgram clears the
    // adapter's software latch. Release is mandatory even on an execution fault.
    // 中文翻译：先保存执行错误，再释放 Group；释放成功不能掩盖本批次故障。
    const bool executionOk = !m_gtn->ErrorOccurred();
    const QString executionError = m_gtn->GroupExecutionError();
    releaseActiveGroup(errorMessage, executionOk ? "completed" : "execution_fault");
    if (!executionOk) {
        m_bufferCommandFailed = true;
        if (errorMessage) {
            // 中文翻译：GTN 报告 Group 或 CommandList 执行故障
            *errorMessage = !executionError.isEmpty() ? executionError : QCoreApplication::translate(
                "GtnBufferedCommandSink",
                "GTN reported a Group/CommandList execution fault");
        }
    }
    return false;
}

bool GtnBufferedCommandSink::executeRapidSegment(const lcnc::cam::RapidMoveSegment& segment,
                                                  const Tool& tool,
                                                  QString* errorMessage)
{
    if (!m_gtn) {
        if (errorMessage) *errorMessage = QStringLiteral("GtnBufferedCommandSink: motion control not bound");
        return false;
    }
    if (segment.movingAxisMask == 0)
        return true;
    if (segment.synchronization != lcnc::cam::RapidSynchronization::Coordinated) {
        if (errorMessage) {
            *errorMessage = QStringLiteral(
                "A planned rapid segment requires an unsupported sequential axis order");
        }
        return false;
    }

    // GTN point-to-point calls schedule individual axes and therefore cannot
    // preserve a CAM-certified TCP path.  Execute each planned sample as its
    // own coordinated line and wait for it before the next sample is built.
    // This is intentionally conservative; a later FIFO batching optimization
    // must retain exactly the same coordinate-line semantics.
    Tool rapidTool = tool;
    rapidTool.m_dLineVelocity = tool.m_dIdleXVelocity > 0 ? tool.m_dIdleXVelocity : 10.0;
    rapidTool.m_dLineAcc = tool.m_dIdleXYAccDec > 0 ? tool.m_dIdleXYAccDec : tool.m_dLineAcc;
    rapidTool.m_dLineJerk = tool.m_dIdleXYJerk > 0 ? tool.m_dIdleXYJerk : tool.m_dLineJerk;
    if (m_gtn->UsesGroupArchitecture()) {
        if (m_axisMap.activeCount() != 5) {
            if (errorMessage) *errorMessage = QStringLiteral("GTN five-axis Group requires five active axes");
            return false;
        }
        if (!m_gtn->InitFiveAxisGroup(rapidTool)) {
            if (errorMessage) *errorMessage = !m_gtn->GroupExecutionError().isEmpty()
                ? m_gtn->GroupExecutionError() : QStringLiteral("GTN rapid Group initialization failed");
            return false;
        }
        m_groupProgramActive = true;
        std::array<double, 5> target{};
        if (m_gtn->IsGroupRtcpActive()) {
            if (!segment.target.tcpMcsValid) {
                if (errorMessage) *errorMessage = QStringLiteral("CAM rapid RTCP reference TCP is unavailable");
                releaseActiveGroup(nullptr, "rapid_rtcp_reference_invalid");
                return false;
            }
            target = {segment.target.tcpMcsX, segment.target.tcpMcsY, segment.target.tcpMcsZ,
                      segment.target.axes[static_cast<int>(AxisMap::R1)],
                      segment.target.axes[static_cast<int>(AxisMap::R2)]};
            const std::array<double, 5> predicted{
                segment.target.axes[static_cast<int>(AxisMap::X)],
                segment.target.axes[static_cast<int>(AxisMap::Y)],
                segment.target.axes[static_cast<int>(AxisMap::Z)],
                segment.target.axes[static_cast<int>(AxisMap::R1)],
                segment.target.axes[static_cast<int>(AxisMap::R2)]};
            if (!m_gtn->ValidateGroupRtcpTarget(target, predicted)) {
                if (errorMessage) *errorMessage = QStringLiteral("GTN RTCP rapid transform does not match the CAM-predicted axes");
                releaseActiveGroup(nullptr, "rapid_validation_failed");
                return false;
            }
        } else {
            target = {segment.target.axes[static_cast<int>(AxisMap::X)],
                      segment.target.axes[static_cast<int>(AxisMap::Y)],
                      segment.target.axes[static_cast<int>(AxisMap::Z)],
                      segment.target.axes[static_cast<int>(AxisMap::R1)],
                      segment.target.axes[static_cast<int>(AxisMap::R2)]};
        }
        if (!m_gtn->GroupLineTo(target, rapidTool)
            || !m_gtn->StartFiveAxisGroupProgram()) {
            if (errorMessage) *errorMessage = QStringLiteral("GTN coordinated Group rapid command failed");
            releaseActiveGroup(nullptr, "rapid_start_failed");
            return false;
        }
        while (m_gtn->IsFiveAxisGroupProgramRunning()) {
            if (m_token && m_token->isStopping()) {
                if (errorMessage) *errorMessage = QStringLiteral("Cutting has been interrupted");
                releaseActiveGroup(nullptr, "rapid_interrupted");
                return false;
            }
            QThread::msleep(10);
        }
        const bool executionOk = !m_gtn->ErrorOccurred();
        const QString executionError = m_gtn->GroupExecutionError();
        const bool releaseOk = releaseActiveGroup(errorMessage,
                                                   executionOk ? "rapid_completed" : "rapid_execution_fault");
        if (!executionOk && errorMessage)
            *errorMessage = !executionError.isEmpty() ? executionError
                : QStringLiteral("GTN reported a Group/CommandList rapid execution fault");
        return executionOk && releaseOk;
    }
    if (!m_gtn->InitCrd(rapidTool)) {
        if (errorMessage) *errorMessage = QStringLiteral("GTN rapid coordinate initialization failed");
        return false;
    }
    m_gtn->SetJumpAccJerk(rapidTool);
    const std::array<double, 5> target{
        segment.target.axes[static_cast<int>(AxisMap::X)],
        segment.target.axes[static_cast<int>(AxisMap::Y)],
        segment.target.axes[static_cast<int>(AxisMap::Z)],
        segment.target.axes[static_cast<int>(AxisMap::R1)],
        segment.target.axes[static_cast<int>(AxisMap::R2)]};
    if (!m_gtn->OffsetLineTo(target, m_axisMap.activeCount(), rapidTool)
        || !m_gtn->SendCommand()) {
        if (errorMessage) *errorMessage = QStringLiteral("GTN coordinated rapid command failed");
        return false;
    }
    while (m_gtn->IsAxisMoving()) {
        if (m_token && m_token->isStopping()) {
            if (errorMessage) *errorMessage = QStringLiteral("Cutting has been interrupted");
            return false;
        }
        QThread::msleep(10);
    }
    return true;
}

void GtnBufferedCommandSink::startCuttingHead(const Tool& tool)
{
    if (m_gtn) m_gtn->StartMovingCuttingHead(tool);
}

void GtnBufferedCommandSink::stopCuttingHead()
{
    if (m_gtn) m_gtn->StopMovingCuttingHead();
}

void GtnBufferedCommandSink::setShutterTimings(double beforeOn, double afterOn,
                                                double beforeOff, double afterOff,
                                                double blowDelay)
{
    if (m_gtn)
        m_gtn->SetShutterOnOffWaitTime(beforeOn, afterOn, beforeOff, afterOff, blowDelay);
}

void GtnBufferedCommandSink::applyToolMotionParams(const Tool& tool, bool jump)
{
    if (!m_gtn) return;
    if (jump) m_gtn->SetJumpAccJerk(tool);
    else      m_gtn->SetCuttingAccJerk(tool);
}

bool GtnBufferedCommandSink::beginSegment(const MachinePose5& /*startPose*/, const Tool& tool,
                                          QString* errorMessage)
{
    // GTN 在 InitCrd 阶段已经建立好坐标系（GTN_SetCrdPrm + GTN_InitLookAheadEx）。
    // 这里只需把当前工具的切割 ACC/JERK 推下去（写到 GTN_BufXxx FIFO，不触发执行）。
    if (!m_gtn) {
        // 中文翻译：GTN 控制器不可用
        if (errorMessage) *errorMessage = QCoreApplication::translate("GtnBufferedCommandSink", "GTN controller is unavailable");
        return false;
    }
    if (m_gtn->UsesGroupArchitecture()) {
        if (m_groupProgramActive) {
            if (errorMessage) *errorMessage = QCoreApplication::translate(
                "GtnBufferedCommandSink", "The previous GTN five-axis Group was not released");
            return false;
        }
        if (m_axisMap.activeCount() != 5) {
            if (errorMessage) *errorMessage = QCoreApplication::translate(
                "GtnBufferedCommandSink", "GTN five-axis Group requires five active axes");
            return false;
        }
        if (!m_gtn->InitFiveAxisGroup(tool)) {
            if (errorMessage) *errorMessage = !m_gtn->GroupExecutionError().isEmpty()
                ? m_gtn->GroupExecutionError() : QCoreApplication::translate(
                "GtnBufferedCommandSink", "GTN five-axis Group initialization failed");
            return false;
        }
        m_groupProgramActive = true;
        return true;
    }
    if (!m_gtn->InitCrd(tool)) {
        // 中文翻译：GTN 坐标系初始化失败
        if (errorMessage) *errorMessage = QCoreApplication::translate("GtnBufferedCommandSink", "GTN coordinate initialization failed");
        return false;
    }
    m_gtn->SetCuttingAccJerk(tool);
    return true;
}

bool GtnBufferedCommandSink::lineTo(const MachinePose5& target, const Tool& tool,
                                    QString* errorMessage)
{
	// 仅写入 GTN_LnXYZACEx 到 FIFO；CAM 已完成软件 IK，因此 RTCP 保持关闭。
	// CAM already provides final physical coordinates. Never add a tool Z
	// correction here or GTN would execute a path different from CAM's scan.
	if (!m_gtn) {
        // 中文翻译：GTN 控制器不可用
        if (errorMessage) *errorMessage = QCoreApplication::translate("GtnBufferedCommandSink", "GTN controller is unavailable");
        return false;
    }
	std::array<double, 5> position{
		target.x, target.y, target.z, target.r1, target.r2};
	bool success = false;
	if (m_groupProgramActive) {
		if (m_gtn->IsGroupRtcpActive()) {
			if (!target.tcpMcsValid) {
				if (errorMessage) *errorMessage = QCoreApplication::translate(
					"GtnBufferedCommandSink", "RTCP requires a valid CAM TCP in machine coordinates");
				releaseActiveGroup(nullptr, "rtcp_target_invalid");
				return false;
			}
			position = {target.tcpMcsX, target.tcpMcsY, target.tcpMcsZ,
			            target.r1, target.r2};
			const std::array<double, 5> predicted{
				target.x, target.y, target.z, target.r1, target.r2};
			if (!m_gtn->ValidateGroupRtcpTarget(position, predicted)) {
				if (errorMessage) *errorMessage = QCoreApplication::translate(
					"GtnBufferedCommandSink", "GTN RTCP transform does not match the CAM-predicted axes");
				releaseActiveGroup(nullptr, "rtcp_validation_failed");
				return false;
			}
		}
		success = m_gtn->GroupLineTo(position, tool);
	} else if (m_gtn->UsesGroupArchitecture()) {
		if (errorMessage) *errorMessage = QCoreApplication::translate(
			"GtnBufferedCommandSink", "GTN five-axis Group is not initialized");
		return false;
	} else {
		success = m_gtn->OffsetLineTo(position, m_axisMap.activeCount(), tool);
	}
	if (!success) {
        // 中文翻译：GTN 缓冲直线指令失败
        if (errorMessage) *errorMessage = QCoreApplication::translate("GtnBufferedCommandSink", "GTN buffered line command failed");
        releaseActiveGroup(nullptr, "line_command_failed");
        return false;
    }
    return true;
}

void GtnBufferedCommandSink::endSegment(const Tool& /*tool*/)
{
    // GTN 无显式 ENDS/SPLIT；flush() 时 GTN_CrdStart 会自然消费整段。
}

void GtnBufferedCommandSink::laserOn(const Tool& tool)
{
    if (!m_gtn) return;
    if (m_groupProgramActive) {
        m_bufferCommandFailed = !m_gtn->GroupLaserControl(true, tool) || m_bufferCommandFailed;
    } else if (m_gtn->UsesGroupArchitecture()) {
        m_bufferCommandFailed = true;
        LCNC_ERR(lcnc::LogCode::Generic,
                 "gtn.api: operation=GroupLaserControl action=reject reason=group_not_initialized laser_on=1 result=-1");
    } else {
        m_gtn->ProLaserControl(/*bLaser=*/true,  /*bPso=*/false, tool, /*bAOUTFlag=*/false);
    }
}

void GtnBufferedCommandSink::laserOff(const Tool& tool)
{
    if (!m_gtn) return;
    if (m_groupProgramActive) {
        m_bufferCommandFailed = !m_gtn->GroupLaserControl(false, tool) || m_bufferCommandFailed;
    } else if (m_gtn->UsesGroupArchitecture()) {
        m_bufferCommandFailed = true;
        LCNC_ERR(lcnc::LogCode::Generic,
                 "gtn.api: operation=GroupLaserControl action=reject reason=group_not_initialized laser_on=0 result=-1");
    } else {
        m_gtn->ProLaserControl(/*bLaser=*/false, /*bPso=*/false, tool, /*bAOUTFlag=*/false);
    }
}

void GtnBufferedCommandSink::endProgram(const Tool& /*tool*/)
{
    // GTN 端没有"结束程序文本"概念；停吹气等在 ProLaserControl(false,...) 中已写入 FIFO。
}

} // namespace lcnc::process
