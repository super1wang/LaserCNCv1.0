#pragma once

/**
 * @brief Lightweight progress reporter passed into async task jobs.
 *
 * A TaskProgress instance is created by TaskManager for each task and passed
 * to the job lambda. The job periodically calls setValue() / setStepName() to
 * update progress and checks isAbortRequested() to support cancellation.
 */
#include <atomic>
#include <functional>
#include <QString>

class TaskProgress
{
public:
    using ProgressCallback = std::function<void(int /*percent*/, const QString& /*step*/)>;

    TaskProgress() = default;

    // ── Called from worker thread ─────────────────────────────────────────────
    void setRange(int min, int max);
    void setValue(int value);       ///< absolute value within [min, max]
    void setStepName(const QString& name);
    bool isAbortRequested() const { return m_abortRequested.load(); }

    // ── Called from controller (TaskManager / UI) ─────────────────────────────
    void requestAbort() { m_abortRequested.store(true); }

    int     percent()  const { return m_percent.load(); }
    QString stepName() const { return m_stepName; }

    /// Set a callback invoked whenever progress or step changes (in worker thread)
    void setCallback(ProgressCallback cb) { m_callback = std::move(cb); }

private:
    std::atomic<bool> m_abortRequested{false};
    std::atomic<int>  m_percent{0};
    int               m_min{0};
    int               m_max{100};
    QString           m_stepName;
    ProgressCallback  m_callback;
};
