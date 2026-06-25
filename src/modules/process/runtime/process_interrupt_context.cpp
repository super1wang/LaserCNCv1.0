#include "modules/process/runtime/process_interrupt_context.h"

#include <QCoreApplication>
#include <QEventLoop>
#include <QMutexLocker>
#include <QThread>

namespace lcnc::process {

namespace {

// 暂停期间每轮 processEvents 的时间片（ms）。
constexpr int kPauseSliceMs = 50;
// 每轮 processEvents 之后的让出 sleep，避免在 paused 下空转。
constexpr int kPauseSleepMs = 20;

} // namespace

ProcessInterruptContext::ProcessInterruptContext() = default;
ProcessInterruptContext::~ProcessInterruptContext() = default;

void ProcessInterruptContext::requestPause()
{
    paused.store(true);
}

void ProcessInterruptContext::requestResume()
{
    paused.store(false);
}

void ProcessInterruptContext::requestStop()
{
    stopRequested.store(true);
    // 同步翻 paused → 让正卡在 checkpoint 抽水循环里的步骤立刻醒来并退出。
    paused.store(false);
}

void ProcessInterruptContext::requestEmergencyStop()
{
    emergencyStop.store(true);
    stopRequested.store(true);
    paused.store(false);
}

void ProcessInterruptContext::reset()
{
    paused.store(false);
    stopRequested.store(false);
    emergencyStop.store(false);
    QMutexLocker lk(&m_resumeMutex);
    m_resumePoints.clear();
}

bool ProcessInterruptContext::checkpoint(const QString& nodeId,
                                          const QString& label,
                                          const QVariantMap& env)
{
    if (stopRequested.load() || emergencyStop.load())
        return false;

    // 1. 更新该节点最近一次到达的位置 / 环境（不论是否会阻塞）
    {
        QMutexLocker lk(&m_resumeMutex);
        ProcessResumePoint& p = m_resumePoints[nodeId];
        p.label = label;
        p.env = env;
        p.valid = true;
    }

    // 2. 暂停态下进入抽水等待循环
    while (paused.load()) {
        if (stopRequested.load() || emergencyStop.load())
            return false;
        // 抽水期间 GUI 仍可响应，再次 pause/resume/stop 会更新原子位。
        QCoreApplication::processEvents(QEventLoop::AllEvents, kPauseSliceMs);
        QThread::msleep(static_cast<unsigned long>(kPauseSleepMs));
    }

    // 3. 离开断点前再检一次（极少：刚解除 pause 又被 stop）
    return !(stopRequested.load() || emergencyStop.load());
}

bool ProcessInterruptContext::noteCheckpoint(const QString& nodeId,
                                              const QString& label,
                                              const QVariantMap& env)
{
    if (stopRequested.load() || emergencyStop.load())
        return false;
    QMutexLocker lk(&m_resumeMutex);
    ProcessResumePoint& p = m_resumePoints[nodeId];
    p.label = label;
    p.env = env;
    p.valid = true;
    return true;
}

bool ProcessInterruptContext::hasResumePoint(const QString& nodeId) const
{
    QMutexLocker lk(&m_resumeMutex);
    auto it = m_resumePoints.constFind(nodeId);
    return it != m_resumePoints.constEnd() && it->valid;
}

ProcessResumePoint ProcessInterruptContext::resumePoint(const QString& nodeId) const
{
    QMutexLocker lk(&m_resumeMutex);
    return m_resumePoints.value(nodeId, ProcessResumePoint{});
}

void ProcessInterruptContext::clearResumePoint(const QString& nodeId)
{
    QMutexLocker lk(&m_resumeMutex);
    m_resumePoints.remove(nodeId);
}

} // namespace lcnc::process
