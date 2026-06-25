#include "modules/process/runtime/acs_text_command_sink.h"

#include "core/logging/logger.h"
#include "modules/process/Tool/Tool.h"
#include "modules/process/device/MotionControl/ACSMotionControl.h"
#include "modules/process/runtime/process_interrupt_context.h"

#include <QCoreApplication>
#include <QEventLoop>
#include <QThread>

#include <boost/lexical_cast.hpp>
#include <cmath>
#include <utility>

namespace lcnc::process {

namespace {

constexpr double kPi = 3.14159265358979323846;

// 把 double 安全转字符串：非有限值 → "0"，避免在 ACSPL+ 程序里写出 0xCD…。
inline std::string D(double v)
{
    if (!std::isfinite(v))
        return "0";
    return boost::lexical_cast<std::string>(v);
}

inline std::string I(int v)
{
    return boost::lexical_cast<std::string>(v);
}

// 与 Fix #2 同源的回退：工具 acc/jerk 无效时使用 sink 自带 AxisMap 里 X 轴的默认。
struct ResolvedMotion
{
    double feed;
    double acc;
    double jerk;
    double junctionVel;
    double junctionAngle;
    double xsegEndVel;
};

ResolvedMotion resolveCuttingMotion(const Tool& tool, const AxisMap& axes)
{
    const auto& xDef = axes.axis(AxisMap::X);
    ResolvedMotion r{};
    r.feed         = (std::isfinite(tool.m_dLineVelocity) && tool.m_dLineVelocity > 0)
                         ? tool.m_dLineVelocity : xDef.velocity;
    r.acc          = (std::isfinite(tool.m_dLineAcc)      && tool.m_dLineAcc      > 0)
                         ? tool.m_dLineAcc      : xDef.acceleration;
    r.jerk         = (std::isfinite(tool.m_dLineJerk)     && tool.m_dLineJerk     > 0)
                         ? tool.m_dLineJerk     : xDef.jerk;
    r.junctionVel  = std::isfinite(tool.m_dJunctionVelocity)  ? std::max(0.0, tool.m_dJunctionVelocity) : 0.0;
    r.junctionAngle= std::isfinite(tool.m_dJunctionAngle)     ? std::max(0.0, tool.m_dJunctionAngle)    : 0.0;
    r.xsegEndVel   = std::isfinite(tool.m_dXsegEndVelocity)   ? std::max(0.0, tool.m_dXsegEndVelocity)  : r.feed;
    return r;
}

// 当前段是否需要写 Z / R1 / R2 —— 看 AxisMap 是否有该轴。
std::uint8_t segmentMaskFor(const AxisMap& axes)
{
    std::uint8_t m = MachinePose5::Bx | MachinePose5::By;
    if (axes.isPresent(AxisMap::Z))  m |= MachinePose5::Bz;
    if (axes.isPresent(AxisMap::R1)) m |= MachinePose5::Br1;
    if (axes.isPresent(AxisMap::R2)) m |= MachinePose5::Br2;
    return m;
}

// 按 pose.mask 取出每个语义轴对应的坐标值，组成 "x, y[, z, r1, r2]"。
std::string poseCoordsText(const MachinePose5& pose, std::uint8_t effectiveMask)
{
    std::string s;
    bool first = true;
    auto put = [&](double v) {
        if (!first) s += ", ";
        s += D(v);
        first = false;
    };
    if (effectiveMask & MachinePose5::Bx)  put(pose.x);
    if (effectiveMask & MachinePose5::By)  put(pose.y);
    if (effectiveMask & MachinePose5::Bz)  put(pose.z);
    if (effectiveMask & MachinePose5::Br1) put(pose.r1);
    if (effectiveMask & MachinePose5::Br2) put(pose.r2);
    return s;
}

// 按 axisMap + mask 取出 "APOSx, APOSy[, APOSz, APOSr1, APOSr2]"，
// 用作 XSEG 起点（控制器侧已有的实时位置）。
std::string aposListText(const AxisMap& axes, std::uint8_t effectiveMask)
{
    std::string s;
    bool first = true;
    static constexpr AxisMap::SemanticAxis order[5] = {
        AxisMap::X, AxisMap::Y, AxisMap::Z, AxisMap::R1, AxisMap::R2
    };
    static constexpr std::uint8_t bits[5] = {
        MachinePose5::Bx, MachinePose5::By, MachinePose5::Bz,
        MachinePose5::Br1, MachinePose5::Br2
    };
    for (int i = 0; i < 5; ++i) {
        if (!(effectiveMask & bits[i])) continue;
        const int idx = axes.controllerIndex(order[i]);
        if (idx < 0) continue;
        if (!first) s += ", ";
        s += "APOS";
        s += I(idx);
        first = false;
    }
    return s;
}

} // namespace

AcsTextCommandSink::AcsTextCommandSink(ACSMotionControl* acs, AxisMap axisMap)
    : m_acs(acs)
    , m_axisMap(std::move(axisMap))
{
}

QString AcsTextCommandSink::id() const
{
    return m_acs ? QString::fromStdString(m_acs->GetName()) : QStringLiteral("ACS");
}

void AcsTextCommandSink::appendText(const std::string& text)
{
    if (m_acs)
        m_acs->AppendRawProgramText(text);
}

void AcsTextCommandSink::resetProgram()
{
    if (m_acs)
        m_acs->ResetProgramCommand();
}

bool AcsTextCommandSink::flush(QString* errorMessage)
{
    if (!m_acs) {
        if (errorMessage) *errorMessage = QStringLiteral("AcsTextCommandSink: motion control not bound");
        return false;
    }
    if (!m_acs->SendCommand()) {
        if (errorMessage) *errorMessage = QStringLiteral("ACS SendCommand 失败");
        LCNC_ERR(lcnc::LogCode::Generic, "AcsTextCommandSink::flush SendCommand failed");
        return false;
    }

    // 轮询等待缓冲执行完成（对应遗留 waitForACSCompletion）。
    // ACSMotionControl 始终用 buffer #9 作为程序缓冲（参见 ACSMotionControl 构造函数）。
    constexpr int kAcsProgramBuffer = 9;
    constexpr int kPollSliceMs      = 20;
    while (m_acs->IsBufferRunning(kAcsProgramBuffer)) {
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

void AcsTextCommandSink::jumpToIdleZ(const Tool& tool)
{
    if (m_acs) m_acs->JumpToIdleHeight(tool);
}

void AcsTextCommandSink::jumpToXY(double x, double y, const Tool& tool)
{
    if (m_acs) m_acs->JumpToIdleXYPosition(x, y, tool);
}

void AcsTextCommandSink::jumpToCuttingZ(const Tool& tool)
{
    if (m_acs) m_acs->JumpToCuttingHeight(tool);
}

void AcsTextCommandSink::startCuttingHead(const Tool& tool)
{
    if (m_acs) m_acs->StartMovingCuttingHead(tool);
}

void AcsTextCommandSink::stopCuttingHead()
{
    if (m_acs) m_acs->StopMovingCuttingHead();
}

void AcsTextCommandSink::setShutterTimings(double beforeOn, double afterOn,
                                            double beforeOff, double afterOff,
                                            double blowDelay)
{
    if (m_acs)
        m_acs->SetShutterOnOffWaitTime(beforeOn, afterOn, beforeOff, afterOff, blowDelay);
}

void AcsTextCommandSink::applyToolMotionParams(const Tool& tool, bool jump)
{
    if (!m_acs) return;
    if (jump)
        m_acs->SetJumpAccJerk(tool);
    else
        m_acs->SetCuttingAccJerk(tool);
}

void AcsTextCommandSink::laserOn(const Tool& tool)
{
    if (m_acs) m_acs->ProLaserControl(/*bLaser=*/true, /*bPso=*/false, tool, /*bAOUTFlag=*/false);
}

void AcsTextCommandSink::laserOff(const Tool& tool)
{
    // bPso=false 走"只写关光等待+IO"分支，ENDS/SPLIT 由 endSegment 单独负责。
    if (m_acs) m_acs->ProLaserControl(/*bLaser=*/false, /*bPso=*/false, tool, /*bAOUTFlag=*/false);
}

void AcsTextCommandSink::endProgram(const Tool& tool)
{
    if (m_acs)
        m_acs->EndProgramCommand(tool);
}

void AcsTextCommandSink::endSegment(const Tool& /*tool*/)
{
    if (!m_acs) return;
    const std::uint8_t segMask = segmentMaskFor(m_axisMap);
    const std::string tuple = m_axisMap.axisTupleText(segMask).toStdString();

    // 结束一段协调插补：把整个 LINE/V 缓冲冲到主控；元组按构型展开，避免 (0,1) 硬编码。
    std::string text;
    text += "ENDS ";  text += tuple; text += "\n";
    text += "GO ";    text += tuple; text += "\n";
    text += "SPLIT "; text += tuple; text += "\n";
    appendText(text);
}

void AcsTextCommandSink::beginSegment(const MachinePose5& /*startPose*/, const Tool& tool)
{
    if (!m_acs) return;
    const auto motion = resolveCuttingMotion(tool, m_axisMap);
    const std::uint8_t segMask = segmentMaskFor(m_axisMap);

    // 把切割段的 ACC/DEC/JERK 推下去（与遗留 SetCuttingAccJerk 等价，但只发参与段的轴）。
    applyToolMotionParams(tool, /*jump=*/false);

    // XSEG/VFJA：构型驱动轴元组与起点 APOS 列表。
    //   5 轴：XSEG/VFJA (0, 1, 2, 3, 4), APOS0, APOS1, APOS2, APOS3, APOS4, feed, fxend, jvel, jang
    //   3 轴：XSEG/VFJA (0, 1, 2),       APOS0, APOS1, APOS2,            feed, fxend, jvel, jang
    //   2 轴：XSEG/VFJA (0, 1),          APOS0, APOS1,                   feed, fxend, jvel, jang
    std::string text;
    text += "XSEG/VFJA ";
    text += m_axisMap.axisTupleText(segMask).toStdString();
    text += ", ";
    text += aposListText(m_axisMap, segMask);
    text += ", ";
    text += D(motion.feed);
    text += ", ";
    text += D(motion.xsegEndVel);
    text += ", ";
    text += D(motion.junctionVel);
    text += ", ";
    text += D(motion.junctionAngle * kPi / 180.0);
    text += "\n";
    appendText(text);
}

void AcsTextCommandSink::lineTo(const MachinePose5& target, const Tool& tool)
{
    if (!m_acs) return;
    const auto motion = resolveCuttingMotion(tool, m_axisMap);
    // 段 mask = 当前调用的 pose mask ∩ 构型实际拥有的轴。
    const std::uint8_t segMask = static_cast<std::uint8_t>(
        target.mask & segmentMaskFor(m_axisMap));

    std::string text;
    text += "LINE/V ";
    text += m_axisMap.axisTupleText(segMask).toStdString();
    text += ", ";
    text += poseCoordsText(target, segMask);
    text += ", ";
    text += D(motion.feed);
    text += "\n";
    appendText(text);
}

} // namespace lcnc::process
