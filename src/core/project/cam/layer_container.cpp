#include "core/project/cam/layer_container.h"

namespace lcnc::cam {

QVector<std::uint64_t> LayerContainer::orderedLayerIds() const
{
    QVector<std::uint64_t> result;
    if (!m_toolpath)
        return result;
    const auto& layers = m_toolpath->layers();
    result.reserve(static_cast<int>(layers.size()));
    for (const ToolpathLayer& layer : layers)
        result.push_back(layer.layerId);
    return result;
}

ToolpathLayer* LayerContainer::layer(std::uint64_t layerId)
{
    if (!m_toolpath)
        return nullptr;
    for (ToolpathLayer& layer : m_toolpath->layers()) {
        if (layer.layerId == layerId)
            return &layer;
    }
    return nullptr;
}

const ToolpathLayer* LayerContainer::layer(std::uint64_t layerId) const
{
    if (!m_toolpath)
        return nullptr;
    for (const ToolpathLayer& layer : m_toolpath->layers()) {
        if (layer.layerId == layerId)
            return &layer;
    }
    return nullptr;
}

QList<ContourId> LayerContainer::contoursInLayer(std::uint64_t layerId) const
{
    QList<ContourId> out;
    if (!m_toolpath)
        return out;
    for (const LaserContour& contour : m_toolpath->contours()) {
        if (contour.layerId == layerId)
            out.append(static_cast<ContourId>(contour.contourId));
    }
    return out;
}

bool LayerContainer::setLayerName(std::uint64_t layerId, const QString& name)
{
    ToolpathLayer* l = layer(layerId);
    if (!l)
        return false;
    const QString trimmed = name.trimmed();
    if (trimmed.isEmpty() || trimmed == l->name)
        return false;
    l->name = trimmed;
    return true;
}

bool LayerContainer::setLayerColor(std::uint64_t layerId, const QColor& color)
{
    ToolpathLayer* l = layer(layerId);
    if (!l || !color.isValid())
        return false;
    if (l->color == color)
        return false;
    l->color = color;
    return true;
}

bool LayerContainer::setLayerEnabled(std::uint64_t layerId, bool enabled)
{
    ToolpathLayer* l = layer(layerId);
    if (!l || l->enabled == enabled)
        return false;
    l->enabled = enabled;
    if (m_toolpath) {
        for (LaserContour& contour : m_toolpath->contours()) {
            if (contour.layerId == layerId)
                contour.enabled = enabled;
        }
    }
    return true;
}

bool LayerContainer::setLayerToolName(std::uint64_t layerId, const QString& toolName)
{
    ToolpathLayer* l = layer(layerId);
    if (!l)
        return false;
    const QString trimmed = toolName.trimmed();
    if (l->toolName == trimmed)
        return false;
    l->toolName = trimmed;
    return true;
}

bool LayerContainer::setLayerCompensationIndex(std::uint64_t layerId, const QString& compensationIndex)
{
    ToolpathLayer* l = layer(layerId);
    if (!l)
        return false;
    if (l->compensationIndex == compensationIndex)
        return false;
    l->compensationIndex = compensationIndex;
    return true;
}

bool LayerContainer::setLayerIncludedContours(std::uint64_t layerId, const QSet<ContourId>& included)
{
    ToolpathLayer* l = layer(layerId);
    if (!l)
        return false;
    if (l->includedContours == included)
        return false;
    l->includedContours = included;
    return true;
}

bool LayerContainer::assignContourToLayer(ContourId contourId, std::uint64_t layerId)
{
    if (!m_toolpath || !layer(layerId))
        return false;
    for (LaserContour& contour : m_toolpath->contours()) {
        if (contour.contourId == contourId) {
            if (contour.layerId == layerId)
                return false;
            contour.layerId = layerId;
            return true;
        }
    }
    return false;
}

bool LayerContainer::setManualContourOrder(const QVector<ContourId>& ids)
{
    QVector<ContourId> deduped;
    QSet<ContourId>    seen;
    deduped.reserve(ids.size());
    for (ContourId id : ids) {
        if (id == 0 || seen.contains(id))
            continue;
        seen.insert(id);
        deduped.push_back(id);
    }
    if (deduped == m_manualContourOrder)
        return false;
    m_manualContourOrder = std::move(deduped);
    m_manualOrderSet     = std::move(seen);
    return true;
}

int LayerContainer::appendToManualOrder(const QVector<ContourId>& ids)
{
    int added = 0;
    for (ContourId id : ids) {
        if (id == 0 || m_manualOrderSet.contains(id))
            continue;
        m_manualContourOrder.push_back(id);
        m_manualOrderSet.insert(id);
        ++added;
    }
    return added;
}

bool LayerContainer::removeFromManualOrder(const QVector<ContourId>& ids)
{
    bool changed = false;
    for (ContourId id : ids) {
        if (m_manualOrderSet.remove(id)) {
            // remove first match from vector
            const int idx = m_manualContourOrder.indexOf(id);
            if (idx >= 0)
                m_manualContourOrder.remove(idx);
            changed = true;
        }
    }
    return changed;
}

bool LayerContainer::clearManualOrder()
{
    if (m_manualContourOrder.isEmpty() && m_manualOrderSet.isEmpty())
        return false;
    m_manualContourOrder.clear();
    m_manualOrderSet.clear();
    return true;
}

bool LayerContainer::setSortStrategy(CuttingPlanSortStrategy s)
{
    if (m_sortStrategy == s)
        return false;
    m_sortStrategy = s;
    return true;
}

} // namespace lcnc::cam
