#include "modules/process/runtime/gtn_buffered_command_sink.h"

#include <QCoreApplication>

#include "core/logging/logger.h"
#include "modules/process/tool/tool.h"
#include "modules/process/device/motion_control/gtn_motion_control.h"
#include "modules/process/runtime/process_interrupt_context.h"

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
                // 中文翻译：切割已被中断
                if (errorMessage) *errorMessage = QStringLiteral("Cutting has been interrupted");
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
        // 中文翻译：GTN SendCommand 失败
        if (errorMessage) *errorMessage = QStringLiteral("GTN SendCommand failed");
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
        const auto physicalAxis = magic_enum::enum_cast<Axis>(name);
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
	// Z 以实际刀路点为基准叠加切割高度，
	// 绝不在中途 SendCommand。
	if (!m_gtn) {
        // 中文翻译：GTN 控制器不可用
        if (errorMessage) *errorMessage = QCoreApplication::translate("GtnBufferedCommandSink", "GTN controller is unavailable");
        return false;
    }
	const std::array<double, 5> position{
		target.x, target.y,
		target.z + tool.m_dCuttingHeight + tool.m_dCuttingHeightCompensate,
		target.r1, target.r2};
	if (!m_gtn->OffsetLineTo(position, m_axisMap.activeCount(), tool)) {
        // 中文翻译：GTN 缓冲直线指令失败
        if (errorMessage) *errorMessage = QCoreApplication::translate("GtnBufferedCommandSink", "GTN buffered line command failed");
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
