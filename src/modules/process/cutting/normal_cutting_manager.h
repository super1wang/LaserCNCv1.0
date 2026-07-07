#pragma once

#include "modules/cam/contracts/toolpath_export_dto.h"
#include "modules/process/Tool/Tool.h"
#include "modules/process/runtime/process_cancellation_token.h"
#include "modules/process/toolpath/process_toolpath_service.h"

#include <QObject>
#include <QVariantMap>
#include <QVector>
#include <cstdint>
#include <memory>

class ProcessModule;
class Service;

namespace lcnc::cam { class ICamToolpathProvider; }

namespace lcnc::process {

class PureSimulationToolpathTicker;
class ProcessCuttingPlanService;
class IMotionCommandSink;

/**
 * @brief 普通切割主管线 —— 把 CAM 顺序切割链表落地到统一的 IMotionCommandSink。
 *
 * 重构后只剩一条执行路径：MotionSinkFactory 根据"硬件是否连接 + mc->GetName() + 仿真模式"
 * 选出合适的 sink（ACS 文本 / GTN 缓存 / 纯仿真），上层只跟 sink 谈，不再 if-branch 控制器。
 *
 * 数据流：
 *   ICamToolpathProvider::exportToolpathSnapshot()
 *     → ProcessToolpathService::refreshSnapshot()
 *     → buildCuttingList() 按 startNumber/endNumber 切片，绑 Tool*、补偿
 *     → 每条 CuttingRow 喂给 sink：resetProgram → jumpTo* → setShutterTimings →
 *                                  laserOn → beginSegment → lineTo* → endSegment →
 *                                  laserOff → endProgram → flush
 *
 * 暂停/停止/急停经 ProcessCancellationToken 协同：
 *   - 主线程同步执行，每条轮廓边界做 ic.checkpoint() —— pause 在此阻塞，stop 立刻返回 false；
 *   - sink 内部（PureSim 的 flush 循环、GTN/ACS 的 PrfTrapAxis/WaitProgramEnd 期间）也轮询 token。
 */
class NormalCuttingManager : public QObject
{
    Q_OBJECT
public:
    NormalCuttingManager(Service* service,
                         std::shared_ptr<lcnc::cam::ICamToolpathProvider> toolpathProvider,
                         ProcessModule* processModule,
                         QObject* parent = nullptr);
    ~NormalCuttingManager() override;

    /// 注入项目级工艺数据服务；调用后切割链表的"图层→工具/顺序/补偿"映射由该服务提供。
    void setCuttingPlanService(ProcessCuttingPlanService* service);

    /**
     * 同步执行普通切割。会反复 QCoreApplication::processEvents() 抽水。
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
    void unwrapCuttingListCAxis(QVector<CuttingRow>& rows) const;
    bool cacheKeyMatches(const CuttingListCacheKey& key) const;
    QVector<CuttingRow> cachedCuttingListCopy();
    void storeCuttingListCache(const CuttingListCacheKey& key, const QVector<CuttingRow>& rows);
    void clearCuttingListCache();

    Tool* resolveTool(const QString& toolName, const QString& layerName, QStringList* warnings);

    /// 把一条轮廓喂给 sink。控制器无关。
    bool executeContour(IMotionCommandSink& sink,
                        const CuttingRow& row,
                        ProcessInterruptContext& interrupt,
                        const QString& nodeId,
                        int contourIndex,
                        int total,
                        QString* errorMessage);

    Service* m_service{nullptr};
    std::shared_ptr<lcnc::cam::ICamToolpathProvider> m_toolpathProvider;
    ProcessModule* m_processModule{nullptr};
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
