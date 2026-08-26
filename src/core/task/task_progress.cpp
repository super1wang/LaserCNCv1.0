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
    if (m_percent.exchange(pct) == pct)
        return;

    ProgressCallback callback;
    QString step;
    {
        std::lock_guard<std::mutex> lock(m_stateMutex);
        callback = m_callback;
        step = m_stepName;
    }
    if (callback)
        callback(pct, step);
}

void TaskProgress::setStepName(const QString& name)
{
    ProgressCallback callback;
    {
        std::lock_guard<std::mutex> lock(m_stateMutex);
        m_stepName = name;
        callback = m_callback;
    }
    if (callback)
        callback(m_percent.load(), name);
}

QString TaskProgress::stepName() const
{
    std::lock_guard<std::mutex> lock(m_stateMutex);
    return m_stepName;
}

void TaskProgress::setCallback(ProgressCallback cb)
{
    std::lock_guard<std::mutex> lock(m_stateMutex);
    m_callback = std::move(cb);
}
