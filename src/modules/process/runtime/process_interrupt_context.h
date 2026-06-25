#pragma once

#include <QHash>
#include <QMutex>
#include <QString>
#include <QVariantMap>

#include <atomic>

namespace lcnc::process {

/**
 * @brief 单个 nodeId 的最近中断点（断点续跑用）。
 *
 * - label  : 由步骤代码起的语义标签，例如 "contour:7" / "gtn-segment" / "io-wait"。
 * - env    : 步骤自描述的环境变量（轮廓索引、当前段、累计进度、最后一次轴位等等）。
 * - valid  : false 表示尚未到达过任何中断点。
 */
struct ProcessResumePoint
{
    QString label;
    QVariantMap env;
    bool valid{false};
};

/**
 * @brief 全局统一的运行时中断上下文。
 *
 * 设计目标
 * --------
 *   1. 所有工作流步骤（含子流程节点）通过同一个 API —— `checkpoint()` —— 注册中断点。
 *   2. 暂停时主线程会自然挂在最近一个 checkpoint 上抽水等待；恢复时自然返回继续执行。
 *   3. 停止/急停时 checkpoint 返回 false，步骤代码沿正常 control flow 退出，线程自然收尾。
 *   4. 暂停期间记录"中断的位置和环境"，恢复时既可以"原地继续"（线程没退出），也可以
 *      在下一次进入同一节点时通过 `hasResumePoint / resumePoint` 取出 env，从断点继续。
 *
 * 线程模型
 * --------
 *   原子位（paused/stopRequested/emergencyStop）跨线程安全；恢复点容器以 QMutex 保护。
 *   实际工作流仍跑在主线程，GUI 可以无锁地 requestPause/requestStop。
 *
 * 与遗留命名的兼容
 * ----------------
 *   `using ProcessCancellationToken = ProcessInterruptContext;`（见
 *   process_cancellation_token.h），旧字段名 `paused / stopRequested / emergencyStop`
 *   仍以 atomic_bool 形式公开，可继续 `.load()`；新代码请用 API。
 */
class ProcessInterruptContext
{
public:
    ProcessInterruptContext();
    ~ProcessInterruptContext();

    ProcessInterruptContext(const ProcessInterruptContext&) = delete;
    ProcessInterruptContext& operator=(const ProcessInterruptContext&) = delete;

    // ── 状态控制（由 ProcessWorkflowExecutor / GUI 调用）─────────────────────────
    void requestPause();
    void requestResume();
    void requestStop();
    void requestEmergencyStop();
    /// 清空所有标志位 + 清空所有恢复点（每次 start 新流程时调用）
    void reset();

    bool isPaused() const     { return paused.load(); }
    bool isStopping() const   { return stopRequested.load() || emergencyStop.load(); }
    bool isEmergency() const  { return emergencyStop.load(); }

    // ── 中断点 API（由步骤插件代码调用）──────────────────────────────────────────
    /**
     * 注册一次中断点。
     * @return  true  应继续执行；
     *          false 收到停止/急停请求，请尽快沿正常 control flow 退出。
     * 暂停状态下函数会阻塞抽水（QCoreApplication::processEvents），直到收到 resume
     * 或 stop。期间会持续刷新本 nodeId 的 resumePoint。
     */
    bool checkpoint(const QString& nodeId,
                    const QString& label = QString(),
                    const QVariantMap& env = QVariantMap());

    /**
     * 仅刷新断点位置/环境而不阻塞，用于密集循环里需要持续记录"做到哪里"但不希望
     * 每次都进 processEvents 的场景。返回 isStopping() 的反值。
     */
    bool noteCheckpoint(const QString& nodeId,
                        const QString& label,
                        const QVariantMap& env);

    // ── 断点查询（步骤代码自决"从断点续跑"用）────────────────────────────────────
    bool hasResumePoint(const QString& nodeId) const;
    ProcessResumePoint resumePoint(const QString& nodeId) const;
    void clearResumePoint(const QString& nodeId);

public:
    // 公开字段：保留遗留命名以兼容 NormalCuttingManager 等代码直接 .load()。
    std::atomic_bool paused{false};
    std::atomic_bool stopRequested{false};
    std::atomic_bool emergencyStop{false};

private:
    mutable QMutex m_resumeMutex;
    QHash<QString, ProcessResumePoint> m_resumePoints;
};

} // namespace lcnc::process
