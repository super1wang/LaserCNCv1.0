#include "modules/process/cutting/process_cutting_plan_service.h"

#include "core/algorithms/cam/contour_order_planner.h"
#include "core/kernel/event_bus.h"
#include "core/kernel/kernel.h"
#include "core/logging/logger.h"
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

namespace {

constexpr int kPlanSchemaVersion = 2;

constexpr char kFieldLayerId[]          = "layerId";
constexpr char kFieldLayerName[]        = "layerName";
constexpr char kFieldToolName[]         = "toolName";
constexpr char kFieldEnabled[]          = "enabled";
constexpr char kFieldOrder[]            = "order";
constexpr char kFieldCompensation[]     = "compensationIndex";
constexpr char kFieldIncludedContours[] = "includedContours";
constexpr char kFieldManualOrder[]      = "manualContourOrder";
constexpr char kFieldLastAxis[]         = "lastAutoSortAxis";

} // namespace

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

QVector<ProcessLayerJob> ProcessCuttingPlanService::layerJobs() const
{
    QVector<ProcessLayerJob> out;
    out.reserve(m_layerOrder.size());
    for (std::uint64_t id : m_layerOrder) {
        auto it = m_jobs.constFind(id);
        if (it != m_jobs.constEnd())
            out.append(*it);
    }
    return out;
}

bool ProcessCuttingPlanService::layerJob(std::uint64_t layerId, ProcessLayerJob* out) const
{
    auto it = m_jobs.constFind(layerId);
    if (it == m_jobs.constEnd())
        return false;
    if (out) *out = *it;
    return true;
}

void ProcessCuttingPlanService::setLayerJob(const ProcessLayerJob& job)
{
    if (job.layerId == 0)
        return;
    const bool newEntry = !m_jobs.contains(job.layerId);
    m_jobs.insert(job.layerId, job);
    if (newEntry)
        m_layerOrder.append(job.layerId);
    bumpRevisionAndNotify();
}

void ProcessCuttingPlanService::setLayerJobs(const QVector<ProcessLayerJob>& jobs)
{
    m_jobs.clear();
    m_layerOrder.clear();
    for (const auto& j : jobs) {
        if (j.layerId == 0)
            continue;
        m_jobs.insert(j.layerId, j);
        m_layerOrder.append(j.layerId);
    }
    bumpRevisionAndNotify();
}

void ProcessCuttingPlanService::clearAll()
{
    m_jobs.clear();
    m_layerOrder.clear();
    m_sortStrategy = CuttingPlanSortStrategy::LayerThenContour;
    m_manualContourOrder.clear();
    m_manualOrderSet.clear();
    bumpRevisionAndNotify();
}

void ProcessCuttingPlanService::setSortStrategy(CuttingPlanSortStrategy s)
{
    if (m_sortStrategy == s)
        return;
    m_sortStrategy = s;
    bumpRevisionAndNotify();
}

void ProcessCuttingPlanService::syncFromCam()
{
    if (!m_provider) {
        bumpRevisionAndNotify();
        return;
    }

    const auto snapshot = m_provider->exportToolpathSnapshot();

    // 1. 从 snapshot 中提取唯一图层（按出现次序），保留 CAM 端 layerName 镜像。
    QHash<std::uint64_t, QString> camLayerNames;
    QVector<std::uint64_t> camLayerOrder;
    for (const auto& contour : snapshot.contours) {
        if (contour.layerId == 0)
            continue;
        if (!camLayerNames.contains(contour.layerId)) {
            camLayerNames.insert(contour.layerId, contour.layerName);
            camLayerOrder.append(contour.layerId);
        }
    }

    // 2. 删除已不存在的图层条目。
    QHash<std::uint64_t, ProcessLayerJob> nextJobs;
    QVector<std::uint64_t> nextOrder;
    nextJobs.reserve(camLayerOrder.size());
    nextOrder.reserve(camLayerOrder.size());

    int sequentialOrder = 1;
    for (std::uint64_t layerId : camLayerOrder) {
        ProcessLayerJob job;
        if (auto it = m_jobs.constFind(layerId); it != m_jobs.constEnd()) {
            job = *it; // 保留用户已设置的 toolName/enabled/order/compensation
        } else {
            job.layerId = layerId;
            job.enabled = true;
            job.order   = sequentialOrder; // 新图层放在末尾
        }
        job.layerName = camLayerNames.value(layerId, job.layerName);
        nextJobs.insert(layerId, job);
        nextOrder.append(layerId);
        ++sequentialOrder;
    }

    m_jobs = std::move(nextJobs);
    m_layerOrder = std::move(nextOrder);

    // 3. 清掉 manual 顺序里已不存在的 contourId。
    if (!m_manualContourOrder.isEmpty()) {
        QSet<lcnc::cam::ContourId> alive;
        for (const auto& c : snapshot.contours) alive.insert(c.contourId);
        QVector<lcnc::cam::ContourId> filtered;
        filtered.reserve(m_manualContourOrder.size());
        for (auto id : m_manualContourOrder)
            if (alive.contains(id)) filtered.append(id);
        if (filtered.size() != m_manualContourOrder.size()) {
            m_manualContourOrder = filtered;
            m_manualOrderSet = QSet<lcnc::cam::ContourId>(filtered.cbegin(), filtered.cend());
        }
    }

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

    // 1. 先把符合 enabled 条件的轮廓收集成 (contour, layerJob*) 列表。
    struct Item
    {
        const lcnc::cam::ToolpathExportContour* contour;
        const ProcessLayerJob* job;
    };
    QVector<Item> items;
    items.reserve(snapshot.contours.size());

    for (const auto& contour : snapshot.contours) {
        if (!contour.enabled || !contour.layerEnabled)
            continue;
        const ProcessLayerJob* job = nullptr;
        if (auto it = m_jobs.constFind(contour.layerId); it != m_jobs.constEnd())
            job = &it.value();
        if (job && !job->enabled)
            continue;
        // 轮廓级过滤：空 includedContours = 全选；非空时只接受集合内的轮廓。
        if (job && !job->includedContours.isEmpty()
            && !job->includedContours.contains(contour.contourId)) {
            continue;
        }
        // Manual 策略下，链表只包含用户显式排过序的轮廓；其它一律忽略。
        if (m_sortStrategy == CuttingPlanSortStrategy::Manual
            && !m_manualOrderSet.contains(contour.contourId)) {
            continue;
        }
        items.append({&contour, job});
    }

    // 2. 排序。
    // 预计算 manual 序列中 contourId -> 位置，加速查找。
    QHash<lcnc::cam::ContourId, int> manualRank;
    if (m_sortStrategy == CuttingPlanSortStrategy::Manual) {
        manualRank.reserve(m_manualContourOrder.size());
        for (int i = 0; i < m_manualContourOrder.size(); ++i)
            manualRank.insert(m_manualContourOrder[i], i);
    }

    auto cmp = [this, &manualRank](const Item& a, const Item& b) -> bool {
        switch (m_sortStrategy) {
        case CuttingPlanSortStrategy::CamOrder:
            // 已经是 CAM 顺序，stable_sort 等价
            return false;
        case CuttingPlanSortStrategy::LayerThenContour:
            if (a.contour->layerId != b.contour->layerId)
                return a.contour->layerId < b.contour->layerId;
            return a.contour->contourId < b.contour->contourId;
        case CuttingPlanSortStrategy::ToolThenLayer: {
            const QString ta = a.job ? a.job->toolName : QString();
            const QString tb = b.job ? b.job->toolName : QString();
            const int c = QString::localeAwareCompare(ta, tb);
            if (c != 0) return c < 0;
            if (a.contour->layerId != b.contour->layerId)
                return a.contour->layerId < b.contour->layerId;
            return a.contour->contourId < b.contour->contourId;
        }
        case CuttingPlanSortStrategy::Manual: {
            // 轮廓级 manual：以 m_manualContourOrder 中的索引为序；不在列表中的沉到末尾。
            const int ra = manualRank.value(a.contour->contourId, INT_MAX);
            const int rb = manualRank.value(b.contour->contourId, INT_MAX);
            if (ra != rb) return ra < rb;
            return a.contour->contourId < b.contour->contourId;
        }
        }
        return false;
    };
    std::stable_sort(items.begin(), items.end(), cmp);

    // 3. 应用 startSequence / endSequence 过滤（1-based）。
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
        entry.toolName          = it.job ? it.job->toolName : QString();
        entry.compensationIndex = it.job ? it.job->compensationIndex : QString();
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

QString ProcessCuttingPlanService::kPlanFileName()
{
    return QStringLiteral("process_cutting_plan.toml");
}

bool ProcessCuttingPlanService::saveToProjectDir(const QString& packageDir, QString* errorMessage) const
{
    QDir dir(packageDir);
    if (!dir.exists() && !QDir().mkpath(packageDir)) {
        if (errorMessage)
            *errorMessage = QObject::tr("无法创建项目目录: %1").arg(packageDir);
        return false;
    }

    toml::value root(toml::table{});
    root["schemaVersion"] = kPlanSchemaVersion;
    root["sortStrategy"]  = sortStrategyToString(m_sortStrategy).toStdString();

    toml::array layers;
    for (std::uint64_t id : m_layerOrder) {
        auto it = m_jobs.constFind(id);
        if (it == m_jobs.constEnd())
            continue;
        const ProcessLayerJob& j = *it;
        toml::value entry(toml::table{});
        entry[kFieldLayerId]      = static_cast<std::int64_t>(j.layerId);
        entry[kFieldLayerName]    = j.layerName.toStdString();
        entry[kFieldToolName]     = j.toolName.toStdString();
        entry[kFieldEnabled]      = j.enabled;
        entry[kFieldOrder]        = static_cast<std::int64_t>(j.order);
        entry[kFieldCompensation] = j.compensationIndex.toStdString();
        toml::array includedArr;
        // QSet 顺序不定，写入时排序确保 diff 稳定。
        QList<lcnc::cam::ContourId> sortedIncluded(j.includedContours.cbegin(),
                                                   j.includedContours.cend());
        std::sort(sortedIncluded.begin(), sortedIncluded.end());
        for (auto cid : sortedIncluded)
            includedArr.push_back(static_cast<std::int64_t>(cid));
        entry[kFieldIncludedContours] = includedArr;
        layers.push_back(entry);
    }
    root["layers"] = layers;

    // 手动轮廓顺序（轮廓级 Manual 策略）。
    toml::array manualArr;
    for (auto cid : m_manualContourOrder)
        manualArr.push_back(static_cast<std::int64_t>(cid));
    root[kFieldManualOrder] = manualArr;
    root[kFieldLastAxis] = autoSortAxisToString(m_lastAutoSortAxis).toStdString();

    const QString filePath = dir.filePath(kPlanFileName());
    std::ofstream out(filePath.toStdString(), std::ios::binary);
    if (!out.is_open()) {
        if (errorMessage)
            *errorMessage = QObject::tr("无法写入文件: %1").arg(filePath);
        return false;
    }
    out << toml::format(root);
    LCNC_INFO(lcnc::LogCode::Generic,
              "process.cuttingPlan: saved {} layers to '{}'",
              m_layerOrder.size(),
              filePath.toStdString());
    return true;
}

bool ProcessCuttingPlanService::loadFromProjectDir(const QString& packageDir, QString* errorMessage)
{
    const QString filePath = QDir(packageDir).filePath(kPlanFileName());
    if (!QFileInfo::exists(filePath)) {
        // 项目尚未保存过工艺数据，视为正常的"全新项目"。
        clearAll();
        return true;
    }

    toml::value root;
    try {
        root = toml::parse(filePath.toStdString());
    } catch (const std::exception& e) {
        if (errorMessage)
            *errorMessage = QObject::tr("解析 process_cutting_plan.toml 失败: %1")
                                .arg(QString::fromLocal8Bit(e.what()));
        return false;
    }

    if (!root.is_table()) {
        if (errorMessage)
            *errorMessage = QObject::tr("process_cutting_plan.toml 根节点不是 table");
        return false;
    }

    QHash<std::uint64_t, ProcessLayerJob> nextJobs;
    QVector<std::uint64_t> nextOrder;

    if (root.contains("sortStrategy") && root.at("sortStrategy").is_string()) {
        m_sortStrategy = sortStrategyFromString(
            QString::fromStdString(root.at("sortStrategy").as_string()),
            CuttingPlanSortStrategy::LayerThenContour);
    }

    if (root.contains("layers") && root.at("layers").is_array()) {
        for (const toml::value& entry : root.at("layers").as_array()) {
            if (!entry.is_table()) continue;
            ProcessLayerJob j;
            if (entry.contains(kFieldLayerId) && entry.at(kFieldLayerId).is_integer())
                j.layerId = static_cast<std::uint64_t>(entry.at(kFieldLayerId).as_integer());
            if (j.layerId == 0)
                continue;
            if (entry.contains(kFieldLayerName) && entry.at(kFieldLayerName).is_string())
                j.layerName = QString::fromStdString(entry.at(kFieldLayerName).as_string());
            if (entry.contains(kFieldToolName) && entry.at(kFieldToolName).is_string())
                j.toolName = QString::fromStdString(entry.at(kFieldToolName).as_string());
            if (entry.contains(kFieldEnabled) && entry.at(kFieldEnabled).is_boolean())
                j.enabled = entry.at(kFieldEnabled).as_boolean();
            if (entry.contains(kFieldOrder) && entry.at(kFieldOrder).is_integer())
                j.order = static_cast<int>(entry.at(kFieldOrder).as_integer());
            if (entry.contains(kFieldCompensation) && entry.at(kFieldCompensation).is_string())
                j.compensationIndex = QString::fromStdString(entry.at(kFieldCompensation).as_string());
            if (entry.contains(kFieldIncludedContours)
                && entry.at(kFieldIncludedContours).is_array()) {
                for (const toml::value& cid : entry.at(kFieldIncludedContours).as_array()) {
                    if (cid.is_integer())
                        j.includedContours.insert(
                            static_cast<lcnc::cam::ContourId>(cid.as_integer()));
                }
            }

            nextJobs.insert(j.layerId, j);
            nextOrder.append(j.layerId);
        }
    }

    m_jobs = std::move(nextJobs);
    m_layerOrder = std::move(nextOrder);

    // 读取手动轮廓顺序与 last axis（若存在）。
    m_manualContourOrder.clear();
    m_manualOrderSet.clear();
    if (root.contains(kFieldManualOrder) && root.at(kFieldManualOrder).is_array()) {
        for (const toml::value& v : root.at(kFieldManualOrder).as_array()) {
            if (!v.is_integer()) continue;
            const auto cid = static_cast<lcnc::cam::ContourId>(v.as_integer());
            if (cid == 0) continue;
            if (m_manualOrderSet.contains(cid)) continue;
            m_manualContourOrder.append(cid);
            m_manualOrderSet.insert(cid);
        }
    }
    if (root.contains(kFieldLastAxis) && root.at(kFieldLastAxis).is_string()) {
        m_lastAutoSortAxis = autoSortAxisFromString(
            QString::fromStdString(root.at(kFieldLastAxis).as_string()),
            AutoSortAxis::XPos);
    }

    LCNC_INFO(lcnc::LogCode::Generic,
              "process.cuttingPlan: loaded {} layers from '{}'",
              m_layerOrder.size(),
              filePath.toStdString());
    bumpRevisionAndNotify();
    return true;
}

// ── 手动顺序 / 自动排序 / Provider 实现 ─────────────────────────────────────

void ProcessCuttingPlanService::bumpRevisionAndNotify(bool manualOnly)
{
    ++m_planRevision;
    if (manualOnly) {
        emit manualOrderChanged();
        // manual 顺序变化必然会影响 Manual 策略下的链表，所以同时也广播一次 planChanged
        // 以便 UI / TravelPathRenderer 等订阅者统一刷新。
    }
    emit planChanged();
    // 跨模块通知（CAM 端 TravelPathRenderer 不能 connect 到我们的 Qt 信号——
    // 它只持有 IProcessCuttingPlanProvider 接口），统一走 EventBus。
    if (auto* k = lcnc::Kernel::tryCurrent())
        k->events().publish(lcnc::process::events::CuttingPlanChanged{m_planRevision});
}

void ProcessCuttingPlanService::setManualContourOrder(const QVector<lcnc::cam::ContourId>& ids)
{
    m_manualContourOrder.clear();
    m_manualOrderSet.clear();
    m_manualContourOrder.reserve(ids.size());
    for (auto id : ids) {
        if (id == 0) continue;
        if (m_manualOrderSet.contains(id)) continue;
        m_manualContourOrder.append(id);
        m_manualOrderSet.insert(id);
    }
    bumpRevisionAndNotify(true);
}

int ProcessCuttingPlanService::appendToManualOrder(const QVector<lcnc::cam::ContourId>& ids)
{
    int added = 0;
    for (auto id : ids) {
        if (id == 0) continue;
        if (m_manualOrderSet.contains(id)) continue;
        m_manualContourOrder.append(id);
        m_manualOrderSet.insert(id);
        ++added;
    }
    if (added > 0)
        bumpRevisionAndNotify(true);
    return added;
}

void ProcessCuttingPlanService::removeFromManualOrder(const QVector<lcnc::cam::ContourId>& ids)
{
    bool changed = false;
    for (auto id : ids) {
        if (m_manualOrderSet.remove(id)) {
            m_manualContourOrder.removeAll(id);
            changed = true;
        }
    }
    if (changed)
        bumpRevisionAndNotify(true);
}

void ProcessCuttingPlanService::clearManualOrder()
{
    if (m_manualContourOrder.isEmpty()) return;
    m_manualContourOrder.clear();
    m_manualOrderSet.clear();
    bumpRevisionAndNotify(true);
}

void ProcessCuttingPlanService::setLastAutoSortAxis(AutoSortAxis a)
{
    if (m_lastAutoSortAxis == a) return;
    m_lastAutoSortAxis = a;
    // 仅 UI 状态变化，不动 plan revision。
}

bool ProcessCuttingPlanService::applyAutoSort(AutoSortAxis axis, QString* errorMessage)
{
    m_lastAutoSortAxis = axis;

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
    for (const auto& c : snapshot.contours) {
        if (!c.enabled || !c.layerEnabled) continue;
        if (!c.endpointsValid) continue;
        lcnc::cam::ContourEndpoints ep;
        ep.id = c.contourId;
        ep.sx = c.startX; ep.sy = c.startY; ep.sz = c.startZ;
        ep.ex = c.endX;   ep.ey = c.endY;   ep.ez = c.endZ;
        inputs.append(ep);
    }
    if (inputs.isEmpty()) {
        if (errorMessage)
            *errorMessage = QObject::tr("当前没有可参与排序的启用轮廓");
        return false;
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
    const auto ordered = lcnc::cam::planContourOrder(inputs, params);

    m_manualContourOrder = ordered;
    m_manualOrderSet = QSet<lcnc::cam::ContourId>(ordered.cbegin(), ordered.cend());
    m_sortStrategy = CuttingPlanSortStrategy::Manual;

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
