#include "modules/process/cutting/pure_simulation_toolpath_ticker.h"

#include "modules/process/process_module.h"

#include <QDateTime>
#include <QTimer>

#include <algorithm>
#include <cmath>

namespace lcnc::process {

namespace {

double segmentLength(const lcnc::cam::ToolpathExportPoint& a,
                     const lcnc::cam::ToolpathExportPoint& b)
{
    const double dx = b.machineX - a.machineX;
    const double dy = b.machineY - a.machineY;
    const double dz = b.machineZ - a.machineZ;
    const double dr1 = b.machineR1 - a.machineR1;
    const double dr2 = b.machineR2 - a.machineR2;
    // Pure rotary cutting, such as a tube held on C, can have nearly fixed XYZ.
    // Give rotary-only segments an equivalent display length so playback remains continuous.
    return std::sqrt(dx * dx + dy * dy + dz * dz + dr1 * dr1 + dr2 * dr2);
}

lcnc::cam::ToolpathExportPoint interpolate(const lcnc::cam::ToolpathExportPoint& a,
                                           const lcnc::cam::ToolpathExportPoint& b,
                                           double t /* 0..1 */)
{
    t = std::clamp(t, 0.0, 1.0);
    lcnc::cam::ToolpathExportPoint p = a;
    p.machineX = a.machineX + (b.machineX - a.machineX) * t;
    p.machineY = a.machineY + (b.machineY - a.machineY) * t;
    p.machineZ = a.machineZ + (b.machineZ - a.machineZ) * t;
    p.machineR1 = a.machineR1 + (b.machineR1 - a.machineR1) * t;
    p.machineR2 = a.machineR2 + (b.machineR2 - a.machineR2) * t;
    p.rotaryAxis1Name = b.rotaryAxis1Name.isEmpty() ? a.rotaryAxis1Name : b.rotaryAxis1Name;
    p.rotaryAxis2Name = b.rotaryAxis2Name.isEmpty() ? a.rotaryAxis2Name : b.rotaryAxis2Name;
    return p;
}

} // namespace

PureSimulationToolpathTicker::PureSimulationToolpathTicker(ProcessModule* processModule,
                                                            QObject* parent)
    : QObject(parent)
    , m_processModule(processModule)
    , m_timer(new QTimer(this))
{
    m_timer->setInterval(10);
    connect(m_timer, &QTimer::timeout, this, &PureSimulationToolpathTicker::onTick);
}

PureSimulationToolpathTicker::~PureSimulationToolpathTicker() = default;

void PureSimulationToolpathTicker::start(const QVector<lcnc::cam::ToolpathExportPoint>& points,
                                         double feedRate,
                                         double feedOverride)
{
    m_points = points;
    m_currentSegment = 0;
    m_segmentProgress = 0.0;
    m_paused = false;
    m_done = (m_points.size() < 2);
    const double clampedFeed = std::max(0.1, feedRate);
    const double clampedOverride = std::max(0.1, feedOverride);
    // Tool::m_dLineVelocity 历来按 mm/min 录入；这里转 mm/s。
    m_speedMmPerSec = (clampedFeed * clampedOverride) / 60.0;
    m_lastTickMs = QDateTime::currentMSecsSinceEpoch();
    if (!m_points.isEmpty()) {
        // 立刻把模型挪到起点
        emitPosition(m_points.front());
    }
    if (!m_done)
        m_timer->start();
}

void PureSimulationToolpathTicker::pause()
{
    m_paused = true;
    m_lastTickMs = QDateTime::currentMSecsSinceEpoch();
}

void PureSimulationToolpathTicker::resume()
{
    if (m_paused) {
        m_paused = false;
        m_lastTickMs = QDateTime::currentMSecsSinceEpoch();
    }
}

void PureSimulationToolpathTicker::stop()
{
    m_timer->stop();
    m_done = true;
    m_paused = false;
}

bool PureSimulationToolpathTicker::isRunning() const
{
    return m_timer->isActive() && !m_done;
}

bool PureSimulationToolpathTicker::isDone() const
{
    return m_done;
}

void PureSimulationToolpathTicker::onTick()
{
    if (m_done || m_paused || m_points.size() < 2) {
        m_lastTickMs = QDateTime::currentMSecsSinceEpoch();
        return;
    }
    const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
    const double dtSec = std::max(0.0, (nowMs - m_lastTickMs) / 1000.0);
    m_lastTickMs = nowMs;

    double remainStep = m_speedMmPerSec * dtSec;
    while (remainStep > 1e-9 && m_currentSegment + 1 < m_points.size()) {
        const auto& a = m_points[m_currentSegment];
        const auto& b = m_points[m_currentSegment + 1];
        const double segLen = segmentLength(a, b);
        const double left = segLen - m_segmentProgress;
        if (remainStep < left) {
            m_segmentProgress += remainStep;
            remainStep = 0.0;
            const double t = (segLen > 1e-9) ? (m_segmentProgress / segLen) : 1.0;
            emitPosition(interpolate(a, b, t));
        } else {
            // 走到段末，跳进下一段
            remainStep -= left;
            m_segmentProgress = 0.0;
            ++m_currentSegment;
            emitPosition(m_points[m_currentSegment]);
        }
    }

    if (m_currentSegment + 1 >= m_points.size()) {
        // 末端
        if (!m_points.isEmpty())
            emitPosition(m_points.back());
        m_done = true;
        m_timer->stop();
    }
}

void PureSimulationToolpathTicker::emitPosition(const lcnc::cam::ToolpathExportPoint& p)
{
    if (!m_processModule)
        return;
    m_processModule->setAxisPosition(QStringLiteral("X"), p.machineX);
    m_processModule->setAxisPosition(QStringLiteral("Y"), p.machineY);
    m_processModule->setAxisPosition(QStringLiteral("Z"), p.machineZ);
    if (!p.rotaryAxis1Name.isEmpty())
        m_processModule->setAxisPosition(p.rotaryAxis1Name, p.machineR1);
    if (!p.rotaryAxis2Name.isEmpty())
        m_processModule->setAxisPosition(p.rotaryAxis2Name, p.machineR2);
}

} // namespace lcnc::process
