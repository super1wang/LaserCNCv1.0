#include "core/project/cam/cam_data_manager.h"

#include <QHash>
#include <QSet>
#include <QVector>

#include <algorithm>

namespace lcnc::cam {

namespace {

constexpr std::size_t stageIndex(CamPipelineStage stage)
{
    return static_cast<std::size_t>(stage);
}

} // namespace

CamDataManager::CamDataManager()
{
    m_layerContainer.attach(&m_toolpath);
    m_layerManager = std::make_unique<LayerManager>(&m_layerContainer);
}

CamDataManager::~CamDataManager() = default;

const CamPipelineStageState& CamDataManager::pipelineStageState(CamPipelineStage stage) const
{
    return m_pipelineStages.at(stageIndex(stage));
}

void CamDataManager::commitPipelineStage(CamPipelineStage stage, std::uint64_t inputRevision)
{
    CamPipelineStageState& state = m_pipelineStages.at(stageIndex(stage));
    state.available = true;
    state.dirty = false;
    state.inputRevision = inputRevision;
    ++state.revision;
    state.failureReason.clear();
    // 中文翻译：上游 CAM 阶段已更新
    invalidatePipelineAfter(stage, QStringLiteral("Upstream CAM stage updated"));
    m_dirty = true;
}

void CamDataManager::invalidatePipelineAfter(CamPipelineStage stage, const QString& reason)
{
    const std::size_t first = stageIndex(stage) + 1;
    for (std::size_t index = first; index < m_pipelineStages.size(); ++index) {
        CamPipelineStageState& state = m_pipelineStages[index];
        if (state.available)
            state.dirty = true;
        state.failureReason = reason;
    }
}

void CamDataManager::failPipelineStage(CamPipelineStage stage, const QString& reason)
{
    CamPipelineStageState& state = m_pipelineStages.at(stageIndex(stage));
    state.dirty = true;
    state.failureReason = reason;
    invalidatePipelineAfter(stage, reason);
    m_dirty = true;
}

void CamDataManager::clearPipelineStages()
{
    for (CamPipelineStageState& state : m_pipelineStages)
        state = {};
    m_dirty = true;
}

void CamDataManager::restorePipelineStageState(CamPipelineStage stage,
                                                const CamPipelineStageState& state)
{
    m_pipelineStages.at(stageIndex(stage)) = state;
}

bool CamDataManager::hasCompletePipelineChain() const
{
    const CamPipelineStageState& face = pipelineStageState(CamPipelineStage::FaceSeparation);
    if (!face.available || face.dirty)
        return false;
    CamPipelineStage previous = CamPipelineStage::FaceSeparation;
    for (int index = static_cast<int>(CamPipelineStage::ContourExtraction);
         index < static_cast<int>(CamPipelineStage::Count); ++index) {
        const CamPipelineStage current = static_cast<CamPipelineStage>(index);
        const CamPipelineStageState& state = pipelineStageState(current);
        const CamPipelineStageState& upstream = pipelineStageState(previous);
        if (!state.available || state.dirty || state.inputRevision != upstream.revision)
            return false;
        previous = current;
    }
    return true;
}

void CamDataManager::clearToolpath(bool resetIds)
{
    m_toolpath.clear();
    m_layerContainer.clearManualOrder();
    m_layerContainer.setSortStrategy(CuttingPlanSortStrategy::LayerThenContour);
    m_layerContainer.setLastAutoSortAxis(AutoSortAxis::XPos);
    clearPipelineStages();
    if (resetIds) {
        m_nextContourId = 1;
        m_nextLayerId = 1;
        m_signatureToContourId.clear();
        m_signatureToLayerId.clear();
    }
    m_dirty = true;
    if (m_layerManager)
        m_layerManager->emitLayersReset();
}

ContourId CamDataManager::contourIdAt(int contourIdx) const
{
    if (contourIdx < 0 || contourIdx >= m_toolpath.contourCount())
        return 0;

    return static_cast<ContourId>(m_toolpath.contour(contourIdx).contourId);
}

int CamDataManager::contourIndexById(ContourId contourId) const
{
    if (contourId == 0)
        return -1;

    for (int index = 0; index < m_toolpath.contourCount(); ++index) {
        if (m_toolpath.contour(index).contourId == contourId)
            return index;
    }
    return -1;
}

void CamDataManager::ensureContourIds()
{
    for (LaserContour& contour : m_toolpath.contours()) {
        // 优先：用 signature 复用历史 id（跨 generateToolpath / 跨会话稳定）。
        if (contour.contourId == 0 && contour.signature != 0) {
            auto it = m_signatureToContourId.constFind(contour.signature);
            if (it != m_signatureToContourId.constEnd())
                contour.contourId = it.value();
        }
        if (contour.contourId == 0)
            contour.contourId = nextContourId();
        if (contour.contourId >= m_nextContourId)
            m_nextContourId = static_cast<ContourId>(contour.contourId + 1);
        // 把当前映射写回缓存，使后续 commitToolpathStates() 拿到最新表。
        if (contour.signature != 0)
            m_signatureToContourId.insert(contour.signature, contour.contourId);
    }
}

void CamDataManager::ensureToolpathLayers()
{
    ensureContourIds();
    if (m_toolpath.contourCount() == 0) {
        m_toolpath.layers().clear();
        return;
    }

    static const QVector<QColor> palette = {
        QColor(80, 190, 150),
        QColor(235, 170, 70),
        QColor(95, 145, 230),
        QColor(215, 95, 115),
        QColor(135, 120, 210),
        QColor(90, 175, 210),
    };

    // 辅助：对分组 key 做确定性哈希 → layer 的 signature。
    auto layerSigFromKey = [](const QString& key) {
        std::uint64_t h = 1469598103934665603ull;
        for (QChar ch : key) {
            std::uint64_t v = static_cast<std::uint64_t>(ch.unicode());
            h ^= v + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
        }
        return h;
    };

    if (m_toolpath.layers().empty()) {
        QHash<QString, std::uint64_t> layerIdByKey;
        for (LaserContour& contour : m_toolpath.contours()) {
            QString key = contour.sourceInfo.trimmed();
            if (key.isEmpty())
                key = QStringLiteral("Type %1").arg(contour.contourType);
            if (key.contains(QStringLiteral("·")))
                key = key.section(QStringLiteral("·"), 0, 0).trimmed();
            if (key.isEmpty())
                key = QStringLiteral("Default");

            if (!layerIdByKey.contains(key)) {
                const std::uint64_t layerSig = layerSigFromKey(key);
                std::uint64_t lid = 0;
                auto it = m_signatureToLayerId.constFind(layerSig);
                if (it != m_signatureToLayerId.constEnd())
                    lid = it.value();
                if (lid == 0)
                    lid = nextLayerId();

                ToolpathLayer layer;
                layer.layerId   = lid;
                layer.signature = layerSig;
                layer.name      = (key == QStringLiteral("Default"))
                                    // 中文翻译：未分组
                                    ? QStringLiteral("Not grouped") : key;
                layer.color     = palette.at((layerIdByKey.size()) % palette.size());
                m_toolpath.layers().push_back(layer);
                layerIdByKey.insert(key, lid);
                m_signatureToLayerId.insert(layerSig, lid);
            }
            contour.layerId = layerIdByKey.value(key);
        }
    }

    if (!m_toolpath.layers().empty()) {
        const std::uint64_t fallbackLayerId = m_toolpath.layers().front().layerId;
        for (LaserContour& contour : m_toolpath.contours()) {
            if (!toolpathLayer(contour.layerId))
                contour.layerId = fallbackLayerId;
        }
    }

    syncLayerContourIds();
    if (m_layerManager)
        m_layerManager->emitLayersReset();
}

std::uint64_t CamDataManager::addLayer(const QString& name, const QColor& color)
{
    ToolpathLayer layer;
    layer.layerId = nextLayerId();
    const QString trimmed = name.trimmed();
    // 中文翻译：图层 %1
    layer.name = trimmed.isEmpty() ? QStringLiteral("Layer %1").arg(layer.layerId) : trimmed;
    if (color.isValid())
        layer.color = color;
    m_toolpath.layers().push_back(layer);
    m_dirty = true;
    if (m_layerManager)
        m_layerManager->emitLayerAdded(layer.layerId);
    return layer.layerId;
}

bool CamDataManager::removeLayer(std::uint64_t layerId, std::uint64_t reassignTo)
{
    std::vector<ToolpathLayer>& layers = m_toolpath.layers();
    auto it = std::find_if(layers.begin(), layers.end(),
                           [layerId](const ToolpathLayer& l) { return l.layerId == layerId; });
    if (it == layers.end())
        return false;
    if (layers.size() <= 1)
        return false; // 至少保留一个图层，避免轮廓变成"无主"。

    // 选择重挂目标：优先入参；非法/自身/0 时退回第一个其余图层。
    std::uint64_t target = reassignTo;
    if (target == layerId || target == 0 || !toolpathLayer(target)) {
        target = 0;
        for (const ToolpathLayer& l : layers) {
            if (l.layerId != layerId) { target = l.layerId; break; }
        }
    }
    for (LaserContour& c : m_toolpath.contours()) {
        if (c.layerId == layerId)
            c.layerId = target;
    }
    layers.erase(it);
    syncLayerContourIds();
    m_dirty = true;
    if (m_layerManager) {
        m_layerManager->emitLayerRemoved(layerId);
        m_layerManager->emitContourMembershipChanged();
    }
    return true;
}

ToolpathLayer* CamDataManager::toolpathLayer(std::uint64_t layerId)
{
    for (ToolpathLayer& layer : m_toolpath.layers()) {
        if (layer.layerId == layerId)
            return &layer;
    }
    return nullptr;
}

const ToolpathLayer* CamDataManager::toolpathLayer(std::uint64_t layerId) const
{
    for (const ToolpathLayer& layer : m_toolpath.layers()) {
        if (layer.layerId == layerId)
            return &layer;
    }
    return nullptr;
}

QList<int> CamDataManager::contourIndexesInLayer(std::uint64_t layerId) const
{
    QList<int> result;
    for (int index = 0; index < m_toolpath.contourCount(); ++index) {
        if (m_toolpath.contour(index).layerId == layerId)
            result.append(index);
    }
    return result;
}

bool CamDataManager::updateToolpathLayer(std::uint64_t layerId,
                                         const QString& name,
                                         const QColor& color,
                                         const QString& toolName)
{
    ToolpathLayer* layer = toolpathLayer(layerId);
    if (!layer)
        return false;
    bool anyChange = false;

    const QString trimmedName = name.trimmed();
    if (!trimmedName.isEmpty() && trimmedName != layer->name) {
        layer->name = trimmedName;
        anyChange = true;
        if (m_layerManager)
            m_layerManager->emitLayerPropertyChanged(layerId, LayerProperty::Name);
    }
    if (color.isValid() && layer->color != color) {
        layer->color = color;
        anyChange = true;
        if (m_layerManager)
            m_layerManager->emitLayerPropertyChanged(layerId, LayerProperty::Color);
    }
    const QString trimmedTool = toolName.trimmed();
    if (layer->toolName != trimmedTool) {
        layer->toolName = trimmedTool;
        anyChange = true;
        if (m_layerManager)
            m_layerManager->emitLayerPropertyChanged(layerId, LayerProperty::ToolName);
    }
    if (anyChange)
        m_dirty = true;
    // 保持与旧实现一致的"成功"语义：找到图层即返回 true，不要求字段必须变。
    return true;
}

bool CamDataManager::setToolpathLayerEnabled(std::uint64_t layerId, bool enabled)
{
    ToolpathLayer* layer = toolpathLayer(layerId);
    if (!layer)
        return false;
    if (layer->enabled == enabled)
        return false;
    layer->enabled = enabled;
    for (LaserContour& contour : m_toolpath.contours()) {
        if (contour.layerId == layerId)
            contour.enabled = enabled;
    }
    m_dirty = true;
    if (m_layerManager)
        m_layerManager->emitLayerPropertyChanged(layerId, LayerProperty::Enabled);
    return true;
}

bool CamDataManager::assignContourToLayer(ContourId contourId, std::uint64_t layerId)
{
    if (!toolpathLayer(layerId))
        return false;
    const int index = contourIndexById(contourId);
    if (index < 0)
        return false;
    if (m_toolpath.contour(index).layerId == layerId)
        return false;
    m_toolpath.contour(index).layerId = layerId;
    syncLayerContourIds();
    m_dirty = true;
    if (m_layerManager) {
        m_layerManager->emitContourMembershipChanged();
        m_layerManager->emitLayerPropertyChanged(layerId, LayerProperty::ContourMembership);
    }
    return true;
}

bool CamDataManager::reorderContours(const QList<int>& order)
{
    if (order.size() != m_toolpath.contourCount())
        return false;

    QSet<int> seen;
    std::vector<LaserContour> current = std::move(m_toolpath.contours());
    std::vector<LaserContour> reordered;
    reordered.reserve(current.size());

    for (int index : order) {
        if (index < 0 || index >= static_cast<int>(current.size()) || seen.contains(index)) {
            m_toolpath.contours() = std::move(current);
            return false;
        }

        seen.insert(index);
        reordered.push_back(std::move(current[index]));
    }

    m_toolpath.contours() = std::move(reordered);
    ensureContourIds();
    syncLayerContourIds();
    m_dirty = true;
    if (m_layerManager)
        m_layerManager->emitContourMembershipChanged();
    return true;
}

bool CamDataManager::reorderContoursById(const QList<ContourId>& order)
{
    if (order.size() != m_toolpath.contourCount())
        return false;

    QList<int> indexOrder;
    indexOrder.reserve(order.size());
    for (ContourId contourId : order) {
        const int index = contourIndexById(contourId);
        if (index < 0)
            return false;
        indexOrder.append(index);
    }

    return reorderContours(indexOrder);
}

ContourId CamDataManager::nextContourId()
{
    return m_nextContourId++;
}

std::uint64_t CamDataManager::nextLayerId()
{
    return m_nextLayerId++;
}

void CamDataManager::syncLayerContourIds()
{
    for (ToolpathLayer& layer : m_toolpath.layers()) {
        layer.contourIds.clear();
        if (layer.layerId >= m_nextLayerId)
            m_nextLayerId = layer.layerId + 1;
    }

    for (const LaserContour& contour : m_toolpath.contours()) {
        if (ToolpathLayer* layer = toolpathLayer(contour.layerId))
            layer->contourIds.push_back(contour.contourId);
    }
}

void CamDataManager::commitToolpathStates()
{
    // 把当前 toolpath 的 signature → id 写入持久映射表，给下次 generateToolpath
    // 或下次会话 restore 用。
    for (const LaserContour& contour : m_toolpath.contours()) {
        if (contour.signature != 0 && contour.contourId != 0)
            m_signatureToContourId.insert(contour.signature, contour.contourId);
    }
    for (const ToolpathLayer& layer : m_toolpath.layers()) {
        if (layer.signature != 0 && layer.layerId != 0)
            m_signatureToLayerId.insert(layer.signature, layer.layerId);
    }
}

void CamDataManager::restoreSignatureTables(const QHash<std::uint64_t, std::uint64_t>& sigToContour,
                                             const QHash<std::uint64_t, std::uint64_t>& sigToLayer,
                                             ContourId nextContour,
                                             std::uint64_t nextLayer)
{
    m_signatureToContourId = sigToContour;
    m_signatureToLayerId   = sigToLayer;
    if (nextContour > m_nextContourId) m_nextContourId = nextContour;
    if (nextLayer   > m_nextLayerId)   m_nextLayerId   = nextLayer;
}

void CamDataManager::replaceToolpath(LaserToolpath&& toolpath,
                                      ContourId nextContour,
                                      std::uint64_t nextLayer)
{
    m_toolpath = std::move(toolpath);
    m_layerContainer.clearManualOrder();
    m_layerContainer.setSortStrategy(CuttingPlanSortStrategy::LayerThenContour);
    m_layerContainer.setLastAutoSortAxis(AutoSortAxis::XPos);
    if (nextContour > m_nextContourId) m_nextContourId = nextContour;
    if (nextLayer   > m_nextLayerId)   m_nextLayerId   = nextLayer;
    // 把已加载的 signature 映射也填进缓存
    for (const LaserContour& c : m_toolpath.contours())
        if (c.signature != 0 && c.contourId != 0)
            m_signatureToContourId.insert(c.signature, c.contourId);
    for (const ToolpathLayer& l : m_toolpath.layers())
        if (l.signature != 0 && l.layerId != 0)
            m_signatureToLayerId.insert(l.signature, l.layerId);
    syncLayerContourIds();
    m_dirty = false;
    if (m_layerManager)
        m_layerManager->emitLayersReset();
}

} // namespace lcnc::cam
