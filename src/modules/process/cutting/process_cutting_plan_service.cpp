#include "modules/process/cutting/process_cutting_plan_service.h"

#include "core/logging/logger.h"
#include "modules/cam/contracts/i_cam_layer_provider.h"
#include "modules/cam/contracts/i_cam_contour_sequence_provider.h"
#include "modules/cam/contracts/i_cam_toolpath_provider.h"
#include "modules/process/tool/tool.h"
#include "modules/process/tool/tool_factory.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QObject>

#include <toml.hpp>

#include <algorithm>
#include <fstream>

namespace lcnc::process {

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

void ProcessCuttingPlanService::setContourSequenceProvider(
    std::shared_ptr<lcnc::cam::ICamContourSequenceProvider> provider)
{
    m_sequenceProvider = std::move(provider);
}

void ProcessCuttingPlanService::wireLayerProviderSignals()
{
    // 调用方（ProcessModule）负责把 layerProvider->notifier() 的
    // 细粒度 Qt 信号桥接到本服务的 planChanged。这里只在 provider
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

void ProcessCuttingPlanService::clearAll()
{
    // The CAM data lifecycle owns contour membership and sequencing.  Process
    // only drops its derived execution-list revision here.
    bumpRevisionAndNotify();
}

void ProcessCuttingPlanService::syncFromCam()
{
    // Never normalise, filter, or rewrite the CAM order from Process.
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

    const auto snapshot = m_provider->exportToolpathCatalogSnapshot();
    if (snapshot.contours.isEmpty())
        return out;

    // 从 CAM 拉一份图层级映射 (layerId → snapshot)，按图层 layer-level 字段过滤/绑工具。
    QHash<std::uint64_t, lcnc::cam::LayerSnapshot> layerById;
    if (m_layerProvider) {
        const auto layers = m_layerProvider->layers();
        layerById.reserve(layers.size());
        for (const auto& l : layers) layerById.insert(l.layerId, l);
    }
    const auto sequence = m_sequenceProvider ? m_sequenceProvider->contourSequence()
                                             : lcnc::cam::ContourSequenceSnapshot{};
    QHash<lcnc::cam::ContourId, const lcnc::cam::ToolpathExportContour*> contourById;
    contourById.reserve(snapshot.contours.size());
    for (const auto& contour : snapshot.contours)
        contourById.insert(contour.contourId, &contour);

    // The sequence already includes all CAM-level enable, inclusion, manual,
    // and automatic sorting rules.  Process must never supply a fallback sort.
    const int start = std::max(1, filter.startSequence);
    const int end = filter.endSequence > 0 ? filter.endSequence
                                            : sequence.orderedContourIds.size();

    int seq = 0;
    for (const lcnc::cam::ContourId contourId : sequence.orderedContourIds) {
        const auto contourIt = contourById.constFind(contourId);
        if (contourIt == contourById.constEnd())
            continue;
        const auto* contour = contourIt.value();
        if (!contour)
            continue;
        ++seq;
        if (seq < start) continue;
        if (seq > end)   break;

        const auto layerIt = layerById.constFind(contour->layerId);
        const auto* layer = layerIt == layerById.constEnd() ? nullptr : &layerIt.value();

        CuttingListEntry entry;
        entry.contourId         = contour->contourId;
        entry.layerId           = contour->layerId;
        entry.layerName         = contour->layerName;
        entry.contourName       = contour->contourName;
        entry.toolName          = layer ? layer->toolName : QString();
        entry.compensationIndex = layer ? layer->compensationIndex : QString();
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
    const auto snapshot = m_provider->exportToolpathCatalogSnapshot();
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

void ProcessCuttingPlanService::bumpRevisionAndNotify()
{
    ++m_planRevision;
    emit planChanged();
}

void ProcessCuttingPlanService::notifyExternalPlanChanged()
{
    bumpRevisionAndNotify();
}

} // namespace lcnc::process
