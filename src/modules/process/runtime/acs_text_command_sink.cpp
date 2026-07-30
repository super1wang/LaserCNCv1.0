#include "modules/process/runtime/acs_text_command_sink.h"

#include "core/logging/logger.h"
#include "modules/process/Tool/Tool.h"
#include "modules/process/device/MotionControl/ACSMotionControl.h"
#include "modules/process/runtime/process_interrupt_context.h"

#include <QThread>

#include <boost/lexical_cast.hpp>
#include <cmath>
#include <optional>
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

double normalizeSigned180(double value)
{
    while (value > 180.0) value -= 360.0;
    while (value < -180.0) value += 360.0;
    return std::abs(value) < 1e-10 ? 0.0 : value;
}

double normalizeRotaryForAxis(const AxisMap& axes, AxisMap::SemanticAxis axis, double value)
{
    return axes.axisName(axis).trimmed().toUpper() == QStringLiteral("C")
        ? value
        : normalizeSigned180(value);
}

QVector<AxisMap::SemanticAxis> rotaryJumpOrder(const AxisMap& axes)
{
    QVector<AxisMap::SemanticAxis> order;
    auto appendIfPresent = [&](AxisMap::SemanticAxis axis) {
        if (axes.isPresent(axis) && !order.contains(axis))
            order.append(axis);
    };

    if (axes.axisName(AxisMap::R1).trimmed().toUpper() == QStringLiteral("C"))
        appendIfPresent(AxisMap::R1);
    if (axes.axisName(AxisMap::R2).trimmed().toUpper() == QStringLiteral("C"))
        appendIfPresent(AxisMap::R2);

    appendIfPresent(AxisMap::R1);
    appendIfPresent(AxisMap::R2);
    return order;
}

double rotaryPoseValue(const MachinePose5& pose, AxisMap::SemanticAxis axis)
{
    return axis == AxisMap::R1 ? pose.r1 : pose.r2;
}

double rotaryIdleVelocity(const Tool& tool, const AxisMap& axes, AxisMap::SemanticAxis axis)
{
    const QString axisName = axes.axisName(axis).trimmed().toUpper();
    const double configured = axisName == QStringLiteral("C")
        ? tool.m_dIdleCVelocity
        : (axis == AxisMap::R1 ? tool.m_dIdleAVelocity : tool.m_dIdleA1Velocity);
    return configured > 0 ? configured : 10.0;
}

std::optional<Axis> deviceAxisFor(const AxisMap& axes, AxisMap::SemanticAxis semanticAxis)
{
    switch (semanticAxis) {
    case AxisMap::X: return Axis::X;
    case AxisMap::Y: return Axis::Y;
    case AxisMap::Z: return Axis::Z;
    case AxisMap::R1:
    case AxisMap::R2: {
        const QString name = axes.axisName(semanticAxis).trimmed().toUpper();
        if (name == QStringLiteral("A")) return Axis::A;
        if (name == QStringLiteral("B")) return Axis::B;
        if (name == QStringLiteral("C")) return Axis::C;
        return std::nullopt;
    }
    case AxisMap::Count:
        return std::nullopt;
    }
    return std::nullopt;
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

AcsTextCommandSink::AcsTextCommandSink(ACSMotionControl* acs, AxisMap axisMap,
                                       PositionObserver positionObserver)
    : m_acs(acs)
    , m_axisMap(std::move(axisMap))
    , m_positionObserver(std::move(positionObserver))
{
}

void AcsTextCommandSink::publishControllerPositions()
{
    if (!m_acs || !m_positionObserver)
        return;

    // Buffer-state polling can run every few milliseconds.  Publishing each
    // axis on every pass would flood the GUI event queue, so cap view updates
    // at 10 Hz while retaining controller-side completion polling frequency.
    constexpr qint64 kPositionPublishIntervalMs = 100;
    if (m_positionPublishTimer.isValid()
        && m_positionPublishTimer.elapsed() < kPositionPublishIntervalMs) {
        return;
    }
    m_positionPublishTimer.restart();

    static constexpr AxisMap::SemanticAxis kAxes[] = {
        AxisMap::X, AxisMap::Y, AxisMap::Z, AxisMap::R1, AxisMap::R2
    };
    for (const AxisMap::SemanticAxis semanticAxis : kAxes) {
        if (!m_axisMap.isPresent(semanticAxis))
            continue;
        const auto deviceAxis = deviceAxisFor(m_axisMap, semanticAxis);
        if (!deviceAxis)
            continue;
        double position = 0.0;
        if (m_acs->GetActualPos(*deviceAxis, position))
            m_positionObserver(m_axisMap.axisName(semanticAxis), position);
    }
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
    m_hasLastR1 = false;
    m_hasLastR2 = false;
    m_lastR1 = 0.0;
    m_lastR2 = 0.0;
    if (m_acs)
        m_acs->ResetProgramCommand();
}

bool AcsTextCommandSink::flush(QString* errorMessage)
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

bool AcsTextCommandSink::startProgram(QString* errorMessage)
{
    if (!m_acs) {
        if (errorMessage) *errorMessage = QStringLiteral("AcsTextCommandSink: motion control not bound");
        return false;
    }
    if (!m_acs->SendCommand()) {
        // 中文翻译：ACS SendCommand 失败
        if (errorMessage) *errorMessage = QStringLiteral("ACS SendCommand failed");
        LCNC_ERR(lcnc::LogCode::Generic, "AcsTextCommandSink::flush SendCommand failed");
        return false;
    }

    return true;
}

bool AcsTextCommandSink::isProgramRunning(QString* errorMessage)
{
    if (!m_acs) {
        if (errorMessage) *errorMessage = QStringLiteral("AcsTextCommandSink: motion control not bound");
        return false;
    }
    // ACSMotionControl 始终用 buffer #9 作为程序缓冲（参见 ACSMotionControl 构造函数）。
    constexpr int kAcsProgramBuffer = 9;
    publishControllerPositions();
    return m_acs->IsBufferRunning(kAcsProgramBuffer);
}

void AcsTextCommandSink::jumpToIdleZ(const MachinePose5& pose, const Tool& tool)
{
    const int index = m_axisMap.controllerIndex(AxisMap::Z);
    if (index < 0) return;
    appendText("PTP/EV " + I(index) + ", " + D(pose.z + tool.m_dIdleZHeight)
               + ", " + D(tool.m_dIdleZVelocity > 0 ? tool.m_dIdleZVelocity : 10.0) + "\n");
    appendText("TILL ^MST(" + I(index) + ").#MOVE\n");
}

void AcsTextCommandSink::jumpToPose(const MachinePose5& pose, const Tool& tool)
{
    if (!m_acs) return;
    auto emitPtp = [this](AxisMap::SemanticAxis axis, double pos, double vel) {
        const int idx = m_axisMap.controllerIndex(axis);
        if (idx < 0) return;
        std::string s;
        s += "PTP/EV ";
        s += I(idx);
        s += ", ";
        s += D(pos);
        s += ", ";
        s += D(vel);
        s += "\n";
        appendText(s);
        appendText("TILL ^MST(" + I(idx) + ").#MOVE\n");
    };

    emitPtp(AxisMap::X, pose.x, tool.m_dIdleXVelocity > 0 ? tool.m_dIdleXVelocity : 10.0);
    emitPtp(AxisMap::Y, pose.y, tool.m_dIdleYVelocity > 0 ? tool.m_dIdleYVelocity : 10.0);
    for (AxisMap::SemanticAxis axis : rotaryJumpOrder(m_axisMap)) {
        const double value = normalizeRotaryForAxis(m_axisMap, axis, rotaryPoseValue(pose, axis));
        emitPtp(axis, value, rotaryIdleVelocity(tool, m_axisMap, axis));
        if (axis == AxisMap::R1) {
            m_lastR1 = value;
            m_hasLastR1 = true;
        } else if (axis == AxisMap::R2) {
            m_lastR2 = value;
            m_hasLastR2 = true;
        }
    }
}

void AcsTextCommandSink::jumpToCuttingZ(const MachinePose5& pose, const Tool& tool)
{
    const int index = m_axisMap.controllerIndex(AxisMap::Z);
    if (index < 0) return;
    appendText("PTP/EV " + I(index) + ", "
               + D(pose.z + tool.m_dCuttingHeight + tool.m_dCuttingHeightCompensate)
               + ", " + D(tool.m_dIdleZVelocity > 0 ? tool.m_dIdleZVelocity : 10.0) + "\n");
    appendText("TILL ^MST(" + I(index) + ").#MOVE\n");
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

    MachinePose5 out = target;
    // 切割高度是相对轮廓 Z 的有符号增量；加工段中的每个目标点都必须
    // 使用同一偏移，不能仅在切入前 PTP 一次后又回到原始轮廓高度。
    out.z += tool.m_dCuttingHeight + tool.m_dCuttingHeightCompensate;
    if (segMask & MachinePose5::Br1) {
        out.r1 = normalizeRotaryForAxis(m_axisMap, AxisMap::R1, target.r1);
        m_lastR1 = out.r1;
        m_hasLastR1 = true;
    }
    if (segMask & MachinePose5::Br2) {
        out.r2 = normalizeRotaryForAxis(m_axisMap, AxisMap::R2, target.r2);
        m_lastR2 = out.r2;
        m_hasLastR2 = true;
    }

    std::string text;
    text += "LINE/V ";
    text += m_axisMap.axisTupleText(segMask).toStdString();
    text += ", ";
    text += poseCoordsText(out, segMask);
    text += ", ";
    text += D(motion.feed);
    text += "\n";
    appendText(text);
}

} // namespace lcnc::process
