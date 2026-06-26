#include "modules/process/runtime/gtn_buffered_command_sink.h"

#include "core/logging/logger.h"
#include "modules/process/Tool/Tool.h"
#include "modules/process/device/MotionControl/GTNMotionControl.h"

#include <QCoreApplication>
#include <QEventLoop>
#include <QThread>

#include <utility>

namespace lcnc::process {

GtnBufferedCommandSink::GtnBufferedCommandSink(GTNMotionControl* gtn, AxisMap axisMap)
    : m_gtn(gtn)
    , m_axisMap(std::move(axisMap))
{
}

void GtnBufferedCommandSink::resetProgram()
{
    if (m_gtn)
        m_gtn->ResetProgramCommand();
}

bool GtnBufferedCommandSink::flush(QString* errorMessage)
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

    // 轮询等待全部插补完成（与 ACS sink 的 IsBufferRunning 对应）。
    constexpr int kPollSliceMs = 20;
    while (m_gtn->IsAxisMoving()) {
        if (m_token) {
            while (m_token->isPaused()) {
                QCoreApplication::processEvents(QEventLoop::AllEvents, kPollSliceMs);
                QThread::msleep(2);
            }
            if (m_token->isStopping()) {
                if (errorMessage) *errorMessage = QStringLiteral("切割已被中断");
                return false;
            }
        }
        QCoreApplication::processEvents(QEventLoop::AllEvents, kPollSliceMs);
        QThread::msleep(2);
    }
    return true;
}

void GtnBufferedCommandSink::jumpToIdleZ(const Tool& tool)
{
    if (m_gtn) m_gtn->JumpToIdleHeight(tool);
}

void GtnBufferedCommandSink::jumpToXY(double x, double y, const Tool& tool)
{
    if (m_gtn) m_gtn->JumpToIdleXYPosition(x, y, tool);
}

void GtnBufferedCommandSink::jumpToPose(const MachinePose5& pose, const Tool& tool)
{
    if (!m_gtn) return;
    m_gtn->JumpToIdleXYPosition(pose.x, pose.y, tool);
    // GTN 的旋转轴首点定位由后续缓冲插补段完成；这里保持与旧链路一致。
}

void GtnBufferedCommandSink::jumpToCuttingZ(const Tool& tool)
{
    if (m_gtn) m_gtn->JumpToCuttingHeight(tool);
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
    // 仅写入 GTN_LnXYEx 到 FIFO；绝不在中途 SendCommand。
    if (m_gtn)
        m_gtn->OffsetLineTo(target.x, target.y, tool);
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
