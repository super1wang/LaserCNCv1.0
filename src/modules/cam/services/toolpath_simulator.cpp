#include "modules/cam/services/toolpath_simulator.h"

#include "core/algorithms/cam/laser_toolpath.h"

#include <QTimer>

#include <algorithm>

namespace lcnc::cam {

ToolpathSimulator::ToolpathSimulator(QObject* parent)
    : QObject(parent)
    , m_timer(new QTimer(this))
{
    m_timer->setInterval(50);
    connect(m_timer, &QTimer::timeout, this, &ToolpathSimulator::onTick);
}

ToolpathSimulator::~ToolpathSimulator() = default;

int ToolpathSimulator::currentInterval() const
{
    return std::max(10, static_cast<int>(50.0 / m_speed));
}

void ToolpathSimulator::play()
{
    if (!m_toolpath || !m_applyAxis)
        return;

    if (m_paused) {
        m_paused = false;
        m_playing = true;
        m_timer->start();
        emit simulationStateChanged(true);
        return;
    }

    m_currentContour = 0;
    m_currentPoint = 0;
    m_totalPoints = 0;
    for (int i = 0; i < m_toolpath->contourCount(); ++i) {
        const LaserContour& c = m_toolpath->contour(i);
        if (c.enabled)
            m_totalPoints += static_cast<int>(c.points.size());
    }
    if (m_totalPoints == 0)
        return;

    while (m_currentContour < m_toolpath->contourCount()
           && !m_toolpath->contour(m_currentContour).enabled) {
        ++m_currentContour;
    }

    m_playing = true;
    m_paused = false;
    m_timer->setInterval(currentInterval());
    m_timer->start();
    emit simulationStateChanged(true);
}

void ToolpathSimulator::pause()
{
    m_timer->stop();
    m_paused = true;
    m_playing = false;
    emit simulationStateChanged(false);
}

void ToolpathSimulator::stop()
{
    m_timer->stop();
    m_playing = false;
    m_paused = false;
    m_currentContour = 0;
    m_currentPoint = 0;
    emit simulationStateChanged(false);
    emit simulationFinished();
}

void ToolpathSimulator::setSpeed(double factor)
{
    m_speed = (factor > 0.1) ? factor : 0.1;
    if (m_timer->isActive())
        m_timer->setInterval(currentInterval());
}

void ToolpathSimulator::onTick()
{
    if (!m_toolpath) {
        stop();
        return;
    }

    if (m_currentContour >= m_toolpath->contourCount()) {
        stop();
        return;
    }

    const LaserContour& c = m_toolpath->contour(m_currentContour);
    if (m_currentPoint >= static_cast<int>(c.points.size())) {
        ++m_currentContour;
        m_currentPoint = 0;
        while (m_currentContour < m_toolpath->contourCount()
               && !m_toolpath->contour(m_currentContour).enabled) {
            ++m_currentContour;
        }
        if (m_currentContour >= m_toolpath->contourCount()) {
            stop();
            return;
        }
        return;
    }

    const MachineCoord& mc = c.points[m_currentPoint].machineCoord;
    if (!mc.valid) {
        ++m_currentPoint;
        return;
    }

    m_applyAxis(QStringLiteral("X"), mc.x, false);
    m_applyAxis(QStringLiteral("Y"), mc.y, false);
    m_applyAxis(QStringLiteral("Z"), mc.z, false);
    if (!mc.r1Name.isEmpty())
        m_applyAxis(mc.r1Name, mc.r1, false);
    if (!mc.r2Name.isEmpty())
        m_applyAxis(mc.r2Name, mc.r2, false);
    if (m_refresh)
        m_refresh();

    emit simulationTick(m_currentContour, m_currentPoint, m_totalPoints);
    ++m_currentPoint;
}

} // namespace lcnc::cam
