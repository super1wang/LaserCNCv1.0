#include "modules/process/runtime/pure_simulation_sink.h"

#include <QCoreApplication>

#include "core/logging/logger.h"
#include "modules/process/tool/tool.h"
#include "modules/process/cutting/pure_simulation_toolpath_ticker.h"
#include "modules/process/process_module.h"
#include "modules/process/runtime/process_interrupt_context.h"

#include <QMetaObject>
#include <QThread>

#include <utility>

namespace lcnc::process {

namespace {

template <typename Fn>
void invokeOnObjectThread(QObject* object, Fn&& fn)
{
    if (!object)
        return;
    if (object->thread() == QThread::currentThread()) {
        fn();
        return;
    }
    QMetaObject::invokeMethod(
        object, std::forward<Fn>(fn), Qt::BlockingQueuedConnection);
}

} // namespace

PureSimulationSink::PureSimulationSink(PureSimulationToolpathTicker* ticker,
                                        ProcessModule* processModule,
                                        AxisMap axisMap)
    : m_ticker(ticker)
    , m_processModule(processModule)
    , m_axisMap(std::move(axisMap))
{
}

void PureSimulationSink::resetProgram()
{
    m_pending.clear();
    m_feedRate = 600.0;
    m_started = false;
}

bool PureSimulationSink::flush(QString* errorMessage)
{
    if (!startProgram(errorMessage))
        return false;
    while (isProgramRunning(errorMessage))
        QThread::msleep(10);
    return !(m_token && m_token->isStopping());
}

bool PureSimulationSink::startProgram(QString* errorMessage)
{
    if (!m_ticker || !m_processModule) {
        if (errorMessage) *errorMessage = QStringLiteral("PureSimulationSink: ticker/processModule unbound");
        return false;
    }
    if (m_pending.size() < 2)
        return true;  // 没有可回放的段

    double feedOverride = 1.0;
    invokeOnObjectThread(m_processModule, [this, &feedOverride] {
        feedOverride = m_processModule->feedOverride();
    });
    const auto points = m_pending;
    const double feedRate = m_feedRate;
    invokeOnObjectThread(m_ticker, [this, points, feedRate, feedOverride] {
        m_ticker->start(points, feedRate, feedOverride);
    });
    m_started = true;
    return true;
}

bool PureSimulationSink::isProgramRunning(QString* errorMessage)
{
    if (!m_ticker || !m_started)
        return false;
    if (m_token) {
        const bool paused = m_token->isPaused();
        invokeOnObjectThread(m_ticker, [this, paused] {
            if (paused)
                m_ticker->pause();
            else
                m_ticker->resume();
        });
        if (m_token->isStopping()) {
            invokeOnObjectThread(m_ticker, [this] { m_ticker->stop(); });
            // 中文翻译：仿真已被中断
            if (errorMessage) *errorMessage = QStringLiteral("Simulation has been interrupted");
            return false;
        }
    }
    bool done = true;
    invokeOnObjectThread(m_ticker, [this, &done] { done = m_ticker->isDone(); });
    if (!done)
        return true;
    m_started = false;
    m_pending.clear();
    return false;
}

void PureSimulationSink::jumpToIdleZ(const MachinePose5&, const Tool&) {}
void PureSimulationSink::jumpToPose(const MachinePose5&, const Tool&) {}
void PureSimulationSink::jumpToCuttingZ(const MachinePose5&, const Tool&) {}
void PureSimulationSink::startCuttingHead(const Tool& /*tool*/)   {}
void PureSimulationSink::stopCuttingHead()                         {}
void PureSimulationSink::setShutterTimings(double, double, double, double, double) {}
void PureSimulationSink::applyToolMotionParams(const Tool&, bool) {}
void PureSimulationSink::laserOn(const Tool& /*tool*/)             {}
void PureSimulationSink::laserOff(const Tool& /*tool*/)            {}
void PureSimulationSink::endProgram(const Tool& /*tool*/)          {}

bool PureSimulationSink::beginSegment(const MachinePose5& startPose, const Tool& tool,
                                      QString* /*errorMessage*/)
{
    m_feedRate = (tool.m_dLineVelocity > 0) ? tool.m_dLineVelocity : 600.0;
    // 把起点也压进去：PureSimulationToolpathTicker 需要至少 2 个点才能插值。
    if (m_pending.isEmpty()) {
        lcnc::cam::ToolpathExportPoint p{};
        p.machineX  = startPose.x;
        p.machineY  = startPose.y;
        p.machineZ  = startPose.z + tool.m_dCuttingHeight + tool.m_dCuttingHeightCompensate;
        p.machineR1 = startPose.r1;
        p.machineR2 = startPose.r2;
        p.rotaryAxis1Name = startPose.r1Name;
        p.rotaryAxis2Name = startPose.r2Name;
        m_pending.append(p);
    }
    return true;
}

bool PureSimulationSink::lineTo(const MachinePose5& target, const Tool& tool,
                                QString* errorMessage)
{
    const std::uint8_t expectedMask = static_cast<std::uint8_t>((1u << m_axisMap.activeCount()) - 1u);
    if ((target.mask & expectedMask) != expectedMask) {
        // 中文翻译：机床位姿轴掩码与当前轴布局不匹配
        if (errorMessage) *errorMessage = QCoreApplication::translate("PureSimulationSink", "Machine pose axis mask does not match the active layout");
        return false;
    }
    lcnc::cam::ToolpathExportPoint p{};
    p.machineX  = target.x;
    p.machineY  = target.y;
    p.machineZ  = target.z + tool.m_dCuttingHeight + tool.m_dCuttingHeightCompensate;
    p.machineR1 = target.r1;
    p.machineR2 = target.r2;
    p.rotaryAxis1Name = target.r1Name;
    p.rotaryAxis2Name = target.r2Name;
    m_pending.append(p);
    return true;
}

void PureSimulationSink::endSegment(const Tool& /*tool*/) {}

} // namespace lcnc::process
