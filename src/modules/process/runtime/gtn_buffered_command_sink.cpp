#include "modules/process/runtime/gtn_buffered_command_sink.h"

#include "core/logging/logger.h"
#include "modules/process/Tool/Tool.h"
#include "modules/process/device/MotionControl/GTNMotionControl.h"
#include "modules/process/runtime/process_interrupt_context.h"

#include <QThread>

#include <utility>

namespace lcnc::process {

GtnBufferedCommandSink::GtnBufferedCommandSink(GTNMotionControl* gtn, AxisMap axisMap)
    : m_gtn(gtn)
    , m_axisMap(std::move(axisMap))
{
	if (!m_gtn)
		return;

	auto configuredAxis = [this](AxisMap::SemanticAxis axis, Axis fallback) {
		if (!m_axisMap.isPresent(axis))
			return fallback;
		const auto value = enum_cast<Axis>(m_axisMap.axisName(axis).toStdString());
		return value ? *value : fallback;
	};
	m_gtn->ConfigureCuttingAxes(
		configuredAxis(AxisMap::X, Axis::X),
		configuredAxis(AxisMap::Y, Axis::Y),
		configuredAxis(AxisMap::Z, Axis::Z),
		configuredAxis(AxisMap::R1, Axis::A),
		configuredAxis(AxisMap::R2, Axis::C));
}

void GtnBufferedCommandSink::resetProgram()
{
    if (m_gtn)
        m_gtn->ResetProgramCommand();
}

bool GtnBufferedCommandSink::flush(QString* errorMessage)
{
    if (!startProgram(errorMessage))
        return false;
    while (isProgramRunning(errorMessage)) {
        if (m_token) {
            while (m_token->isPaused())
                QThread::msleep(10);
            if (m_token->isStopping()) {
                if (errorMessage) *errorMessage = QStringLiteral("切割已被中断");
                return false;
            }
        }
        QThread::msleep(10);
    }
    return true;
}

bool GtnBufferedCommandSink::startProgram(QString* errorMessage)
{
    if (!m_gtn) {
        if (errorMessage) *errorMessage = QStringLiteral("GtnBufferedCommandSink: motion control not bound");
        return false;
    }
    // 与 ACS 的 LoadBuffer+RunBuffer 对应：GTN 走 GTN_CrdDataEx + GTN_CrdStart。
    if (!m_gtn->SendCommand()) {
        if (errorMessage) *errorMessage = QStringLiteral("GTN SendCommand 失败");
        LCNC_ERR(lcnc::LogCode::Generic, "GtnBufferedCommandSink::flush SendCommand failed");
        return false;
    }
    // 切换回点位模式。
    m_gtn->PrfTrapAxis();
    return true;
}

bool GtnBufferedCommandSink::isProgramRunning(QString* errorMessage)
{
    if (!m_gtn) {
        if (errorMessage) *errorMessage = QStringLiteral("GtnBufferedCommandSink: motion control not bound");
        return false;
    }
    return m_gtn->IsAxisMoving();
}

void GtnBufferedCommandSink::jumpToIdleZ(const MachinePose5& pose, const Tool& tool)
{
    if (!m_gtn) return;
	m_gtn->MoveToPosition(Axis::Z,
                        tool.m_dIdleZVelocity > 0 ? tool.m_dIdleZVelocity : 10.0,
                        pose.z + tool.m_dIdleZHeight);
}

void GtnBufferedCommandSink::jumpToPose(const MachinePose5& pose, const Tool& tool)
{
    if (!m_gtn) return;
    auto move = [this](AxisMap::SemanticAxis axis, double position, double velocity) {
        const auto name = m_axisMap.axisName(axis).toStdString();
        const auto physicalAxis = enum_cast<Axis>(name);
        if (m_axisMap.isPresent(axis) && physicalAxis)
			m_gtn->MoveToPosition(*physicalAxis, velocity > 0 ? velocity : 10.0, position);
    };
    move(AxisMap::X, pose.x, tool.m_dIdleXVelocity);
    move(AxisMap::Y, pose.y, tool.m_dIdleYVelocity);
    move(AxisMap::R1, pose.r1, tool.m_dIdleAVelocity);
    move(AxisMap::R2, pose.r2, tool.m_dIdleA1Velocity);
}

void GtnBufferedCommandSink::jumpToCuttingZ(const MachinePose5& pose, const Tool& tool)
{
    if (!m_gtn) return;
	m_gtn->MoveToPosition(Axis::Z,
                        tool.m_dIdleZVelocity > 0 ? tool.m_dIdleZVelocity : 10.0,
                        pose.z + tool.m_dCuttingHeight + tool.m_dCuttingHeightCompensate);
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

void GtnBufferedCommandSink::beginSegment(const MachinePose5& /*startPose*/, const Tool& tool)
{
    // GTN 在 InitCrd 阶段已经建立好坐标系（GTN_SetCrdPrm + GTN_InitLookAheadEx）。
    // 这里只需把当前工具的切割 ACC/JERK 推下去（写到 GTN_BufXxx FIFO，不触发执行）。
    if (m_gtn) {
        m_gtn->InitCrd(tool);
        m_gtn->SetCuttingAccJerk(tool);
    }
}

void GtnBufferedCommandSink::lineTo(const MachinePose5& target, const Tool& tool)
{
	// 仅写入 GTN_LnXYZACEx 到 FIFO；CAM 已完成软件 IK，因此 RTCP 保持关闭。
	// Z 以实际刀路点为基准叠加切割高度，
	// 绝不在中途 SendCommand。
	if (m_gtn)
		m_gtn->OffsetLineTo(target.x, target.y,
		                    target.z + tool.m_dCuttingHeight + tool.m_dCuttingHeightCompensate,
		                    target.r1, target.r2,
		                    tool);
}

void GtnBufferedCommandSink::endSegment(const Tool& /*tool*/)
{
    // GTN 无显式 ENDS/SPLIT；flush() 时 GTN_CrdStart 会自然消费整段。
}

void GtnBufferedCommandSink::laserOn(const Tool& tool)
{
    if (m_gtn) m_gtn->ProLaserControl(/*bLaser=*/true,  /*bPso=*/false, tool, /*bAOUTFlag=*/true);
}

void GtnBufferedCommandSink::laserOff(const Tool& tool)
{
    if (m_gtn) m_gtn->ProLaserControl(/*bLaser=*/false, /*bPso=*/false, tool, /*bAOUTFlag=*/true);
}

void GtnBufferedCommandSink::endProgram(const Tool& /*tool*/)
{
    // GTN 端没有"结束程序文本"概念；停吹气等在 ProLaserControl(false,...) 中已写入 FIFO。
}

} // namespace lcnc::process
