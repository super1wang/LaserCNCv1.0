#pragma once

#include "modules/cam/contracts/toolpath_export_dto.h"
#include "modules/process/tool/tool.h"
#include "modules/process/runtime/process_cancellation_token.h"
#include "modules/process/runtime/i_motion_command_sink.h"
#include "modules/process/toolpath/process_toolpath_service.h"

#include <QObject>
#include <QVariantMap>
#include <QVector>
#include <cstdint>
#include <functional>
#include <memory>

class ProcessDeviceRuntime;

namespace lcnc::cam {
class ICamToolpathProvider;
struct InitialApproachSnapshot;
}

namespace lcnc::process {

class PureSimulationToolpathTicker;
class ProcessCuttingPlanService;
class ProcessSettingsService;
class IMotionCommandSink;
class DeviceCommandQueue;

struct NormalCuttingCallbacks
{
    MotionSinkCallbacks motionSink;
    std::function<bool()> simulationModeProvider;
    std::function<void(bool)> normalCuttingActivityObserver;
};

/**
 * @brief 普通切割主管线 —— 把 CAM 顺序切割链表落地到统一的 IMotionCommandSink。
 *
 * 重构后只剩一条执行路径：MotionSinkFactory 根据"硬件是否连接 + mc->GetName() + 仿真模式"
 * 选出合适的 sink（ACS 文本 / GTN 缓存 / 纯仿真），上层只跟 sink 谈，不再 if-branch 控制器。
 *
 * 数据流：
 *   ICamToolpathProvider::exportCommittedExecutionSnapshot()
 *     → ProcessToolpathService::refreshSnapshot()
 *     → buildCuttingList() 按 startNumber/endNumber 切片，绑 Tool*、补偿
 *     → 每条 CuttingRow 喂给 sink：resetProgram → jumpTo* → setShutterTimings →
 *                                  laserOn → beginSegment → lineTo* → endSegment →
 *                                  laserOff → endProgram → flush
 *
 * 暂停/停止/急停经 ProcessCancellationToken 协同：
 *   - 工作流专属线程在每条轮廓边界做 ic.checkpoint()；pause 在此生效。
 *   - 每条轮廓仅在构建/启动期间持有设备租约；运行期改用短队列状态读取，
 *     使 Stop 优先级命令能在两次读取之间取得设备访问权。
 */
class NormalCuttingManager : public QObject
{
    Q_OBJECT
public:
    NormalCuttingManager(ProcessDeviceRuntime* service,
                         std::shared_ptr<lcnc::cam::ICamToolpathProvider> toolpathProvider,
                         NormalCuttingCallbacks callbacks,
                         DeviceCommandQueue* deviceQueue,
                         ProcessSettingsService* settings,
                         QObject* parent = nullptr);
    ~NormalCuttingManager() override;

    /// 注入项目级工艺数据服务；调用后切割链表的"图层→工具/顺序/补偿"映射由该服务提供。
    void setCuttingPlanService(ProcessCuttingPlanService* service);

    /**
     * 在工作流专属线程同步执行普通切割；不得抽取 GUI 事件。
     * @return true 全部完成；false 中途因停止/急停/控制器错误退出。
     */
    bool run(const QString& nodeId,
             const QVariantMap& parameters,
             ProcessInterruptContext* interrupt,
             QString* errorMessage);

signals:
    void contourStarted(int index, int total, const QString& description);
    void contourFinished(int index, int total);
    void logMessage(const QString& message);

private:
    struct CuttingRow
    {
        ProcessJobContour data;
        lcnc::cam::RapidTransition entryTransition;
        bool hasEntryTransition{false};
        Tool* tool{nullptr};
        double compensationOffsetX{0.0};
        double compensationOffsetY{0.0};
    };

    struct CuttingListCacheKey
    {
        std::uint64_t snapshotRevision{0};
        std::uint64_t planRevision{0};
        int startNumber{1};
        int endNumber{0};
        double offsetX{0.0};
        double offsetY{0.0};
    };

    QVector<CuttingRow> buildCuttingList(const lcnc::cam::ToolpathExportSnapshot& snapshot,
                                         int startNumber,
                                         int endNumber,
                                         double offsetX,
                                         double offsetY,
                                         QString* errorMessage);
    bool cacheKeyMatches(const CuttingListCacheKey& key) const;
    QVector<CuttingRow> cachedCuttingListCopy();
    void storeCuttingListCache(const CuttingListCacheKey& key, const QVector<CuttingRow>& rows);
    void clearCuttingListCache();

    Tool* resolveTool(const QString& toolName, const QString& layerName, QStringList* warnings);

    /// 把一条轮廓喂给 sink。控制器无关。
    bool executeContour(const std::shared_ptr<IMotionCommandSink>& sink,
                        bool pureSimulation,
                        const CuttingRow& row,
                        ProcessInterruptContext& interrupt,
                        const QString& nodeId,
                        int contourIndex,
                        int total,
                        QString* errorMessage);
    bool prepareInitialApproach(const CuttingRow& row,
                                ProcessInterruptContext& interrupt,
                                lcnc::cam::InitialApproachSnapshot* approach,
                                QString* errorMessage);

    ProcessDeviceRuntime* m_service{nullptr};
    std::shared_ptr<lcnc::cam::ICamToolpathProvider> m_toolpathProvider;
    NormalCuttingCallbacks m_callbacks;
    DeviceCommandQueue* m_deviceQueue{nullptr};
    ProcessSettingsService* m_settings{nullptr};
    std::unique_ptr<ProcessToolpathService> m_toolpathService;
    std::unique_ptr<PureSimulationToolpathTicker> m_simTicker;
    ProcessCuttingPlanService* m_planService{nullptr};
    Tool m_sanitizedDefaultTool;
    bool m_hasCuttingListCache{false};
    CuttingListCacheKey m_cuttingListCacheKey;
    QVector<CuttingRow> m_cuttingListCacheRows;
    QMetaObject::Connection m_planChangedConnection;
};

} // namespace lcnc::process
