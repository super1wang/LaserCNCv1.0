#include "modules/process/cutting/process_cutting_plan_service.h"

#include "core/algorithms/cam/contour_order_planner.h"
#include "core/kernel/event_bus.h"
#include "core/kernel/kernel.h"
#include "core/logging/logger.h"
#include "core/services/selection_service.h"
#include "modules/cam/i_cam_layer_provider.h"
#include "modules/cam/i_cam_toolpath_provider.h"
#include "modules/process/Tool/Tool.h"
#include "modules/process/Tool/ToolFactory.h"
#include "modules/process/runtime/process_events.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QObject>

#include <toml.hpp>

#include <algorithm>
#include <fstream>

namespace lcnc::process {

QString autoSortAxisToString(AutoSortAxis a)
{
    switch (a) {
    case AutoSortAxis::XPos: return QStringLiteral("X+");
    case AutoSortAxis::XNeg: return QStringLiteral("X-");
    case AutoSortAxis::YPos: return QStringLiteral("Y+");
    case AutoSortAxis::YNeg: return QStringLiteral("Y-");
    case AutoSortAxis::ZPos: return QStringLiteral("Z+");
    case AutoSortAxis::ZNeg: return QStringLiteral("Z-");
    }
    return QStringLiteral("X+");
}

AutoSortAxis autoSortAxisFromString(const QString& s, AutoSortAxis def)
{
    const QString t = s.trimmed();
    if (t.compare(QStringLiteral("X+"), Qt::CaseInsensitive) == 0) return AutoSortAxis::XPos;
    if (t.compare(QStringLiteral("X-"), Qt::CaseInsensitive) == 0) return AutoSortAxis::XNeg;
    if (t.compare(QStringLiteral("Y+"), Qt::CaseInsensitive) == 0) return AutoSortAxis::YPos;
    if (t.compare(QStringLiteral("Y-"), Qt::CaseInsensitive) == 0) return AutoSortAxis::YNeg;
    if (t.compare(QStringLiteral("Z+"), Qt::CaseInsensitive) == 0) return AutoSortAxis::ZPos;
    if (t.compare(QStringLiteral("Z-"), Qt::CaseInsensitive) == 0) return AutoSortAxis::ZNeg;
    return def;
}

QString sortStrategyToString(CuttingPlanSortStrategy s)
{
    switch (s) {
    case CuttingPlanSortStrategy::CamOrder:         return QStringLiteral("CamOrder");
    case CuttingPlanSortStrategy::LayerThenContour: return QStringLiteral("LayerThenContour");
    case CuttingPlanSortStrategy::ToolThenLayer:    return QStringLiteral("ToolThenLayer");
    case CuttingPlanSortStrategy::Manual:           return QStringLiteral("Manual");
    }
    return QStringLiteral("LayerThenContour");
}

CuttingPlanSortStrategy sortStrategyFromString(const QString& s, CuttingPlanSortStrategy def)
{
    if (s.compare(QStringLiteral("CamOrder"), Qt::CaseInsensitive) == 0)
        return CuttingPlanSortStrategy::CamOrder;
    if (s.compare(QStringLiteral("LayerThenContour"), Qt::CaseInsensitive) == 0)
        return CuttingPlanSortStrategy::LayerThenContour;
    if (s.compare(QStringLiteral("ToolThenLayer"), Qt::CaseInsensitive) == 0)
        return CuttingPlanSortStrategy::ToolThenLayer;
    if (s.compare(QStringLiteral("Manual"), Qt::CaseInsensitive) == 0)
        return CuttingPlanSortStrategy::Manual;
    return def;
}

ProcessCuttingPlanService::ProcessCuttingPlanService(QObject* parent)
    : QObject(parent)
{
}

ProcessCuttingPlanService::~ProcessCuttingPlanService() = default;

void ProcessCuttingPlanService::setToolpathProvider(std::shared_ptr<lcnc::cam::ICamToolpathProvider> provider)
{
    m_provider = std::move(provider);
}

void ProcessCuttingPlanService::setLayerProvider(std::shared_ptr<lcnc::cam::ICamLayerProvider> provider)
{
    m_layerProvider = std::move(provider);
    wireLayerProviderSignals();
}

void ProcessCuttingPlanService::wireLayerProviderSignals()
{
    // Phase B 收尾：调用方（ProcessModule）负责把 layerProvider->notifier() 的
    // 细粒度 Qt 信号桥接到本服务的 planChanged/manualOrderChanged。这里只在 provider
    // 接入完成时立即广播一次，确保首屏 UI 拉到正确值。
    if (m_layerProvider)
        bumpRevisionAndNotify();
}

QVector<ProcessLayerJob> ProcessCuttingPlanService::layerJobs() const
{
    QVector<ProcessLayerJob> out;
    if (!m_layerProvider)
        return out;
    const auto layers = m_layerProvider->layers();
    out.reserve(layers.size());
    for (const auto& s : layers) {
        ProcessLayerJob job;
        job.layerId           = s.layerId;
        job.layerName         = s.name;
        job.toolName          = s.toolName;
        job.enabled           = s.enabled;
        job.compensationIndex = s.compensationIndex;
        job.includedContours  = s.includedContours;
        out.append(job);
    }
    return out;
}

bool ProcessCuttingPlanService::layerJob(std::uint64_t layerId, ProcessLayerJob* out) const
{
    if (!m_layerProvider) return false;
    const auto layers = m_layerProvider->layers();
    for (const auto& s : layers) {
        if (s.layerId != layerId) continue;
        if (out) {
            out->layerId           = s.layerId;
            out->layerName         = s.name;
            out->toolName          = s.toolName;
            out->enabled           = s.enabled;
            out->compensationIndex = s.compensationIndex;
            out->includedContours  = s.includedContours;
        }
        return true;
    }
    return false;
}

void ProcessCuttingPlanService::setLayerJob(const ProcessLayerJob& job)
{
    if (!m_layerProvider || job.layerId == 0) return;
    m_layerProvider->setLayerToolName(job.layerId, job.toolName);
    m_layerProvider->setLayerEnabled(job.layerId, job.enabled);
    m_layerProvider->setLayerCompensationIndex(job.layerId, job.compensationIndex);
    m_layerProvider->setLayerIncludedContours(job.layerId, job.includedContours);
    bumpRevisionAndNotify();
}

void ProcessCuttingPlanService::setLayerJobs(const QVector<ProcessLayerJob>& jobs)
{
    if (!m_layerProvider) return;
    for (const auto& j : jobs) {
        if (j.layerId == 0) continue;
        m_layerProvider->setLayerToolName(j.layerId, j.toolName);
        m_layerProvider->setLayerEnabled(j.layerId, j.enabled);
        m_layerProvider->setLayerCompensationIndex(j.layerId, j.compensationIndex);
        m_layerProvider->setLayerIncludedContours(j.layerId, j.includedContours);
    }
    bumpRevisionAndNotify();
}

void ProcessCuttingPlanService::clearAll()
{
    // CAM 端 LayerContainer 的"清空"由 CamDataManager::clearToolpath 负责；
    // 这里只把策略与人工顺序复位即可（图层是 CAM 数据生命周期的）。
    if (m_layerProvider) {
        m_layerProvider->clearManualOrder();
        m_layerProvider->setSortStrategy(CuttingPlanSortStrategy::LayerThenContour);
    }
    bumpRevisionAndNotify();
}

CuttingPlanSortStrategy ProcessCuttingPlanService::sortStrategy() const
{
    return m_layerProvider ? m_layerProvider->sortStrategy()
                           : CuttingPlanSortStrategy::LayerThenContour;
}

void ProcessCuttingPlanService::setSortStrategy(CuttingPlanSortStrategy s)
{
    if (!m_layerProvider) return;
    m_layerProvider->setSortStrategy(s);
    bumpRevisionAndNotify();
}

void ProcessCuttingPlanService::syncFromCam()
{
    // Phase B：图层/人工顺序的"存量"已经在 CAM 内统一了；不再需要 jobs 增删表的同步。
    // 唯一仍可能需要做的是清掉手动顺序里已不存在的 contourId（同步 alive 集）。
    if (!m_provider || !m_layerProvider) {
        bumpRevisionAndNotify();
        return;
    }
    const auto snapshot = m_provider->exportToolpathSnapshot();
    QSet<lcnc::cam::ContourId> alive;
    for (const auto& c : snapshot.contours) alive.insert(c.contourId);

    const auto manual = m_layerProvider->manualContourOrder();
    QVector<lcnc::cam::ContourId> filtered;
    filtered.reserve(manual.size());
    for (auto id : manual)
        if (alive.contains(id)) filtered.append(id);
    if (filtered.size() != manual.size())
        m_layerProvider->setManualContourOrder(filtered);

    bumpRevisionAndNotify();
}

QStringList ProcessCuttingPlanService::availableToolNames() const
{
    QStringList list;
    list.append(QString()); // 第一项 = 空 = "未指定"
    for (const QString& name : ToolFactory::toolNames()) {
        if (!list.contains(name))
            list.append(name);
    }
    return list;
}

QVector<ProcessCuttingPlanService::CuttingListEntry>
ProcessCuttingPlanService::buildCuttingList(const CuttingListFilter& filter) const
{
    QVector<CuttingListEntry> out;
    if (!m_provider)
        return out;

    const auto snapshot = m_provider->exportToolpathSnapshot();
    if (snapshot.contours.isEmpty())
        return out;

    // 从 CAM 拉一份图层级映射 (layerId → snapshot)，按图层 layer-level 字段过滤/绑工具。
    QHash<std::uint64_t, lcnc::cam::LayerSnapshot> layerById;
    if (m_layerProvider) {
        const auto layers = m_layerProvider->layers();
        layerById.reserve(layers.size());
        for (const auto& l : layers) layerById.insert(l.layerId, l);
    }
    const CuttingPlanSortStrategy strategy = sortStrategy();
    const QVector<lcnc::cam::ContourId> manualOrder =
        m_layerProvider ? m_layerProvider->manualContourOrder() : QVector<lcnc::cam::ContourId>{};
    QSet<lcnc::cam::ContourId> manualSet(manualOrder.cbegin(), manualOrder.cend());

    struct Item
    {
        const lcnc::cam::ToolpathExportContour* contour;
        const lcnc::cam::LayerSnapshot*         layer;
    };
    QVector<Item> items;
    items.reserve(snapshot.contours.size());

    for (const auto& contour : snapshot.contours) {
        if (!contour.enabled || !contour.layerEnabled)
            continue;
        const lcnc::cam::LayerSnapshot* layer = nullptr;
        if (auto it = layerById.constFind(contour.layerId); it != layerById.constEnd())
            layer = &it.value();
        if (layer && !layer->enabled)
            continue;
        if (layer && !layer->includedContours.isEmpty()
            && !layer->includedContours.contains(contour.contourId)) {
            continue;
        }
        if (strategy == CuttingPlanSortStrategy::Manual
            && !manualSet.contains(contour.contourId)) {
            continue;
        }
        items.append({&contour, layer});
    }

    QHash<lcnc::cam::ContourId, int> manualRank;
    if (strategy == CuttingPlanSortStrategy::Manual) {
        manualRank.reserve(manualOrder.size());
        for (int i = 0; i < manualOrder.size(); ++i)
            manualRank.insert(manualOrder[i], i);
    }

    auto cmp = [strategy, &manualRank](const Item& a, const Item& b) -> bool {
        switch (strategy) {
        case CuttingPlanSortStrategy::CamOrder:
            return false;
        case CuttingPlanSortStrategy::LayerThenContour:
            if (a.contour->layerId != b.contour->layerId)
                return a.contour->layerId < b.contour->layerId;
            return a.contour->contourId < b.contour->contourId;
        case CuttingPlanSortStrategy::ToolThenLayer: {
            const QString ta = a.layer ? a.layer->toolName : QString();
            const QString tb = b.layer ? b.layer->toolName : QString();
            const int c = QString::localeAwareCompare(ta, tb);
            if (c != 0) return c < 0;
            if (a.contour->layerId != b.contour->layerId)
                return a.contour->layerId < b.contour->layerId;
            return a.contour->contourId < b.contour->contourId;
        }
        case CuttingPlanSortStrategy::Manual: {
            const int ra = manualRank.value(a.contour->contourId, INT_MAX);
            const int rb = manualRank.value(b.contour->contourId, INT_MAX);
            if (ra != rb) return ra < rb;
            return a.contour->contourId < b.contour->contourId;
        }
        }
        return false;
    };
    std::stable_sort(items.begin(), items.end(), cmp);

    const int start = std::max(1, filter.startSequence);
    const int end   = filter.endSequence > 0 ? filter.endSequence : items.size();

    int seq = 0;
    for (const Item& it : items) {
        ++seq;
        if (seq < start) continue;
        if (seq > end)   break;

        CuttingListEntry entry;
        entry.contourId         = it.contour->contourId;
        entry.layerId           = it.contour->layerId;
        entry.layerName         = it.contour->layerName;
        entry.contourName       = it.contour->contourName;
        entry.toolName          = it.layer ? it.layer->toolName : QString();
        entry.compensationIndex = it.layer ? it.layer->compensationIndex : QString();
        entry.sequence          = seq;
        out.append(entry);
    }
    return out;
}

QVector<ProcessCuttingPlanService::ContourBrief>
ProcessCuttingPlanService::contoursInLayer(std::uint64_t layerId) const
{
    QVector<ContourBrief> out;
    if (!m_provider)
        return out;
    const auto snapshot = m_provider->exportToolpathSnapshot();
    for (const auto& contour : snapshot.contours) {
        if (contour.layerId != layerId)
            continue;
        ContourBrief b;
        b.contourId    = contour.contourId;
        b.name         = contour.contourName;
        b.enabledInCam = contour.enabled;
        out.append(b);
    }
    return out;
}

// 项目持久化已下沉到 core（cam_toolpath_io）；本服务不再读写任何项目文件。

// ── 手动顺序 / 自动排序 / Provider 实现 ─────────────────────────────────────

void ProcessCuttingPlanService::bumpRevisionAndNotify(bool manualOnly)
{
    ++m_planRevision;
    if (m_provider) {
        const auto cuttingList = buildCuttingList();
        QVector<std::uint64_t> orderedContourIds;
        orderedContourIds.reserve(cuttingList.size());
        for (const CuttingListEntry& entry : cuttingList)
            orderedContourIds.append(entry.contourId);

        if (!m_provider->solveToolpathForOrder(orderedContourIds)) {
            LCNC_ERR(lcnc::LogCode::Generic,
                     "process.cuttingPlan: five-axis toolpath solve failed after cutting order changed");
        }
    }
    if (manualOnly)
        emit manualOrderChanged();
    emit planChanged();
    if (auto* k = lcnc::Kernel::tryCurrent())
        k->events().publish(lcnc::process::events::CuttingPlanChanged{m_planRevision});
}

void ProcessCuttingPlanService::notifyExternalPlanChanged()
{
    bumpRevisionAndNotify(false);
}

void ProcessCuttingPlanService::notifyExternalManualOrderChanged()
{
    bumpRevisionAndNotify(true);
}

QVector<lcnc::cam::ContourId> ProcessCuttingPlanService::manualContourOrder() const
{
    return m_layerProvider ? m_layerProvider->manualContourOrder()
                           : QVector<lcnc::cam::ContourId>{};
}

void ProcessCuttingPlanService::setManualContourOrder(const QVector<lcnc::cam::ContourId>& ids)
{
    if (!m_layerProvider) return;
    m_layerProvider->setManualContourOrder(ids);
    bumpRevisionAndNotify(true);
}

int ProcessCuttingPlanService::appendToManualOrder(const QVector<lcnc::cam::ContourId>& ids)
{
    if (!m_layerProvider) return 0;
    const int added = m_layerProvider->appendToManualOrder(ids);
    if (added > 0) bumpRevisionAndNotify(true);
    return added;
}

void ProcessCuttingPlanService::removeFromManualOrder(const QVector<lcnc::cam::ContourId>& ids)
{
    if (!m_layerProvider) return;
    m_layerProvider->removeFromManualOrder(ids);
    bumpRevisionAndNotify(true);
}

void ProcessCuttingPlanService::clearManualOrder()
{
    if (!m_layerProvider) return;
    m_layerProvider->clearManualOrder();
    bumpRevisionAndNotify(true);
}

AutoSortAxis ProcessCuttingPlanService::lastAutoSortAxis() const
{
    return m_layerProvider ? m_layerProvider->lastAutoSortAxis() : AutoSortAxis::XPos;
}

void ProcessCuttingPlanService::setLastAutoSortAxis(AutoSortAxis a)
{
    if (!m_layerProvider) return;
    m_layerProvider->setLastAutoSortAxis(a);
    // 仅 UI 状态变化，不动 plan revision。
}

bool ProcessCuttingPlanService::applyAutoSort(AutoSortAxis axis, QString* errorMessage)
{
    if (m_layerProvider)
        m_layerProvider->setLastAutoSortAxis(axis);

    if (!m_provider) {
        if (errorMessage)
            *errorMessage = QObject::tr("尚未关联 CAM 刀路数据源");
        return false;
    }
    const auto snapshot = m_provider->exportToolpathSnapshot();
    if (snapshot.contours.isEmpty()) {
        if (errorMessage)
            *errorMessage = QObject::tr("当前 CAM 中没有可排序的轮廓");
        return false;
    }

    QVector<lcnc::cam::ContourEndpoints> inputs;
    inputs.reserve(snapshot.contours.size());
    QHash<lcnc::cam::ContourId, lcnc::cam::ContourEndpoints> byId;
    byId.reserve(snapshot.contours.size());
    for (const auto& c : snapshot.contours) {
        if (!c.enabled || !c.layerEnabled) continue;
        if (!c.endpointsValid) continue;
        lcnc::cam::ContourEndpoints ep;
        ep.id = c.contourId;
        ep.sx = c.startX; ep.sy = c.startY; ep.sz = c.startZ;
        ep.ex = c.endX;   ep.ey = c.endY;   ep.ez = c.endZ;
        byId.insert(ep.id, ep);
        inputs.append(ep);
    }
    if (inputs.isEmpty()) {
        if (errorMessage)
            *errorMessage = QObject::tr("当前没有可参与排序的启用轮廓");
        return false;
    }

    QVector<lcnc::cam::ContourEndpoints> selectedInputs;
    selectedInputs.reserve(inputs.size());

    QSet<lcnc::cam::ContourId> selectedIds;
    QVector<lcnc::cam::ContourId> selectedOrder;
    if (auto* k = lcnc::Kernel::tryCurrent()) {
        if (auto selSvc = k->services().getService<lcnc::core::SelectionService>()) {
            const auto selected = selSvc->contoursInSelectionOrder();
            selectedOrder.reserve(selected.size());
            for (auto id : selected) {
                if (id == 0 || selectedIds.contains(id))
                    continue;
                selectedIds.insert(id);
                selectedOrder.append(id);
            }
        }
    }

    lcnc::cam::ContourOrderParams params;
    switch (axis) {
    case AutoSortAxis::XPos: params.axis = lcnc::cam::PrimaryAxis::XPos; break;
    case AutoSortAxis::XNeg: params.axis = lcnc::cam::PrimaryAxis::XNeg; break;
    case AutoSortAxis::YPos: params.axis = lcnc::cam::PrimaryAxis::YPos; break;
    case AutoSortAxis::YNeg: params.axis = lcnc::cam::PrimaryAxis::YNeg; break;
    case AutoSortAxis::ZPos: params.axis = lcnc::cam::PrimaryAxis::ZPos; break;
    case AutoSortAxis::ZNeg: params.axis = lcnc::cam::PrimaryAxis::ZNeg; break;
    }

    for (auto id : selectedOrder) {
        if (auto it = byId.constFind(id); it != byId.constEnd())
            selectedInputs.append(it.value());
    }

    const auto ordered = selectedInputs.isEmpty()
        ? lcnc::cam::planContourOrder(inputs, params)
        : lcnc::cam::planContourOrder(selectedInputs, params);
    if (ordered.isEmpty()) {
        if (errorMessage)
            *errorMessage = QObject::tr("自动排序未生成有效轮廓顺序");
        return false;
    }

    if (m_layerProvider) {
        m_layerProvider->setManualContourOrder(ordered);
        m_layerProvider->setSortStrategy(CuttingPlanSortStrategy::Manual);
    }

    LCNC_INFO(lcnc::LogCode::Generic,
              "process.cuttingPlan: applyAutoSort axis={} -> {} contours",
              autoSortAxisToString(axis).toStdString(), ordered.size());
    bumpRevisionAndNotify(true);
    return true;
}

QVector<lcnc::cam::ContourId> ProcessCuttingPlanService::orderedContourIds() const
{
    const auto list = buildCuttingList({});
    QVector<lcnc::cam::ContourId> out;
    out.reserve(list.size());
    for (const auto& e : list) out.append(e.contourId);
    return out;
}

} // namespace lcnc::process
