#include "core/project/cam/layer_manager.h"

namespace lcnc::cam {

bool LayerManager::setLayerName(std::uint64_t layerId, const QString& name)
{
    if (!m_container || !m_container->setLayerName(layerId, name))
        return false;
    emit layerPropertyChanged(layerId, LayerProperty::Name);
    return true;
}

bool LayerManager::setLayerColor(std::uint64_t layerId, const QColor& color)
{
    if (!m_container || !m_container->setLayerColor(layerId, color))
        return false;
    emit layerPropertyChanged(layerId, LayerProperty::Color);
    return true;
}

bool LayerManager::setLayerEnabled(std::uint64_t layerId, bool enabled)
{
    if (!m_container || !m_container->setLayerEnabled(layerId, enabled))
        return false;
    emit layerPropertyChanged(layerId, LayerProperty::Enabled);
    return true;
}

bool LayerManager::setLayerToolName(std::uint64_t layerId, const QString& toolName)
{
    if (!m_container || !m_container->setLayerToolName(layerId, toolName))
        return false;
    emit layerPropertyChanged(layerId, LayerProperty::ToolName);
    return true;
}

bool LayerManager::setLayerCompensationIndex(std::uint64_t layerId, const QString& compensationIndex)
{
    if (!m_container || !m_container->setLayerCompensationIndex(layerId, compensationIndex))
        return false;
    emit layerPropertyChanged(layerId, LayerProperty::CompensationIndex);
    return true;
}

bool LayerManager::setLayerIncludedContours(std::uint64_t layerId, const QSet<ContourId>& included)
{
    if (!m_container || !m_container->setLayerIncludedContours(layerId, included))
        return false;
    emit layerPropertyChanged(layerId, LayerProperty::IncludedContours);
    return true;
}

bool LayerManager::assignContourToLayer(ContourId contourId, std::uint64_t layerId)
{
    if (!m_container || !m_container->assignContourToLayer(contourId, layerId))
        return false;
    emit contourMembershipChanged();
    emit layerPropertyChanged(layerId, LayerProperty::ContourMembership);
    return true;
}

bool LayerManager::setManualContourOrder(const QVector<ContourId>& ids)
{
    if (!m_container || !m_container->setManualContourOrder(ids))
        return false;
    emit manualContourOrderChanged();
    return true;
}

int LayerManager::appendToManualOrder(const QVector<ContourId>& ids)
{
    if (!m_container)
        return 0;
    const int added = m_container->appendToManualOrder(ids);
    if (added > 0)
        emit manualContourOrderChanged();
    return added;
}

bool LayerManager::removeFromManualOrder(const QVector<ContourId>& ids)
{
    if (!m_container || !m_container->removeFromManualOrder(ids))
        return false;
    emit manualContourOrderChanged();
    return true;
}

bool LayerManager::clearManualOrder()
{
    if (!m_container || !m_container->clearManualOrder())
        return false;
    emit manualContourOrderChanged();
    return true;
}

bool LayerManager::setSortStrategy(CuttingPlanSortStrategy s)
{
    if (!m_container || !m_container->setSortStrategy(s))
        return false;
    emit sortStrategyChanged(s);
    return true;
}

void LayerManager::emitLayersReset()             { emit layersReset(); }
void LayerManager::emitLayersReordered()         { emit layersReordered(); }
void LayerManager::emitLayerPropertyChanged(std::uint64_t layerId, LayerProperty p)
{
    emit layerPropertyChanged(layerId, p);
}
void LayerManager::emitLayerAdded(std::uint64_t layerId)   { emit layerAdded(layerId); }
void LayerManager::emitLayerRemoved(std::uint64_t layerId) { emit layerRemoved(layerId); }
void LayerManager::emitContourMembershipChanged()          { emit contourMembershipChanged(); }

} // namespace lcnc::cam
