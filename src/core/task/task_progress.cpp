#include "core/task/task_progress.h"

void TaskProgress::setRange(int min, int max)
{
    m_min = min;
    m_max = max;
    m_percent.store(0);
}

void TaskProgress::setValue(int value)
{
    int range = m_max - m_min;
    if (range <= 0) return;
    int pct = qBound(0, (value - m_min) * 100 / range, 100);
    m_percent.store(pct);
    if (m_callback)
        m_callback(pct, m_stepName);
}

void TaskProgress::setStepName(const QString& name)
{
    m_stepName = name;
    if (m_callback)
        m_callback(m_percent.load(), name);
}
