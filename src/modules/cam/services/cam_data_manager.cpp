#include "modules/cam/services/cam_data_manager.h"

#include <QHash>
#include <QSet>
#include <QVector>

namespace lcnc::cam {

void CamDataManager::clearToolpath()
{
    m_toolpath.clear();
    m_nextContourId = 1;
    m_nextLayerId = 1;
    m_dirty = true;
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
        if (contour.contourId == 0)
            contour.contourId = nextContourId();
        if (contour.contourId >= m_nextContourId)
            m_nextContourId = static_cast<ContourId>(contour.contourId + 1);
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

    if (m_toolpath.layers().empty()) {
        QHash<QString, std::uint64_t> layerByKey;
        for (LaserContour& contour : m_toolpath.contours()) {
            QString key = contour.sourceInfo.trimmed();
            if (key.isEmpty())
                key = QStringLiteral("Type %1").arg(contour.contourType);
            if (key.contains(QStringLiteral("·")))
                key = key.section(QStringLiteral("·"), 0, 0).trimmed();
            if (key.isEmpty())
                key = QStringLiteral("Default");

            if (!layerByKey.contains(key)) {
                ToolpathLayer layer;
                layer.layerId = nextLayerId();
                layer.name = key == QStringLiteral("Default")
                    ? QStringLiteral("未分组")
                    : key;
                layer.color = palette.at((layerByKey.size()) % palette.size());
                m_toolpath.layers().push_back(layer);
                layerByKey.insert(key, layer.layerId);
            }
            contour.layerId = layerByKey.value(key);
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
    layer->name = name.trimmed().isEmpty() ? layer->name : name.trimmed();
    if (color.isValid())
        layer->color = color;
    layer->toolName = toolName.trimmed();
    m_dirty = true;
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
    return true;
}

bool CamDataManager::assignContourToLayer(ContourId contourId, std::uint64_t layerId)
{
    if (!toolpathLayer(layerId))
        return false;
    const int index = contourIndexById(contourId);
    if (index < 0)
        return false;
    m_toolpath.contour(index).layerId = layerId;
    syncLayerContourIds();
    m_dirty = true;
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

} // namespace lcnc::cam