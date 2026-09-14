#include "modules/process/runtime/process_interrupt_context.h"

#include <QMutexLocker>

namespace lcnc::process {

namespace {

constexpr unsigned long kPauseWaitSliceMs = 100;

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
    QMutexLocker lock(&m_pauseMutex);
    m_pauseChanged.wakeAll();
}

void ProcessInterruptContext::requestStop()
{
    stopRequested.store(true);
    // 同步翻 paused → 让正卡在 checkpoint 抽水循环里的步骤立刻醒来并退出。
    paused.store(false);
    QMutexLocker lock(&m_pauseMutex);
    m_pauseChanged.wakeAll();
}

void ProcessInterruptContext::reset()
{
    paused.store(false);
    stopRequested.store(false);
    {
        QMutexLocker lock(&m_pauseMutex);
        m_pauseChanged.wakeAll();
    }
    QMutexLocker lk(&m_resumeMutex);
    m_resumePoints.clear();
}

bool ProcessInterruptContext::checkpoint(const QString& nodeId,
                                          const QString& label,
                                          const QVariantMap& env)
{
    if (stopRequested.load())
        return false;

    // 1. 更新该节点最近一次到达的位置 / 环境（不论是否会阻塞）
    {
        QMutexLocker lk(&m_resumeMutex);
        ProcessResumePoint& p = m_resumePoints[nodeId];
        p.label = label;
        p.env = env;
        p.valid = true;
    }

    // 2. 暂停态下进入条件等待。不得在工作流线程调用 processEvents：
    // 那会把 GUI 事件重入到错误的线程，也会让轮询干扰流程执行。
    while (paused.load()) {
        if (stopRequested.load())
            return false;
        QMutexLocker lock(&m_pauseMutex);
        if (paused.load() && !stopRequested.load())
            m_pauseChanged.wait(&m_pauseMutex, kPauseWaitSliceMs);
    }

    // 3. 离开断点前再检一次（极少：刚解除 pause 又被 stop）
    return !stopRequested.load();
}

bool ProcessInterruptContext::noteCheckpoint(const QString& nodeId,
                                              const QString& label,
                                              const QVariantMap& env)
{
    if (stopRequested.load())
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
