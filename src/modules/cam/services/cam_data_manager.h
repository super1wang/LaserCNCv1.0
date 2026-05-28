#pragma once

#include "core/algorithms/cam/laser_toolpath.h"
#include "modules/cam/contracts/cam_data_contracts.h"

#include <QList>
#include <QColor>

namespace lcnc::cam {

/**
 * @brief Project-scoped CAM runtime data owner.
 *
 * Owns dense/non-OCC CAM runtime data such as sampled points, lead-ins,
 * machine coordinates, and parameters. Sparse OCC geometry that should be
 * visible/selectable in the project tree is mirrored into the CAM document by
 * CamModule; dense point arrays intentionally stay here to avoid bloating OCAF.
 */
class CamDataManager
{
public:
    LaserToolpath& toolpath() { return m_toolpath; }
    const LaserToolpath& toolpath() const { return m_toolpath; }

    bool hasToolpath() const { return m_toolpath.contourCount() > 0; }
    void clearToolpath();

    ContourId contourIdAt(int contourIdx) const;
    int contourIndexById(ContourId contourId) const;
    void ensureContourIds();
    void ensureToolpathLayers();
    const std::vector<ToolpathLayer>& toolpathLayers() const { return m_toolpath.layers(); }
    ToolpathLayer* toolpathLayer(std::uint64_t layerId);
    const ToolpathLayer* toolpathLayer(std::uint64_t layerId) const;
    QList<int> contourIndexesInLayer(std::uint64_t layerId) const;
    bool updateToolpathLayer(std::uint64_t layerId,
                             const QString& name,
                             const QColor& color,
                             const QString& toolName);
    bool setToolpathLayerEnabled(std::uint64_t layerId, bool enabled);
    bool assignContourToLayer(ContourId contourId, std::uint64_t layerId);
    bool reorderContours(const QList<int>& order);
    bool reorderContoursById(const QList<ContourId>& order);

    void markDirty(bool dirty = true) { m_dirty = dirty; }
    bool isDirty() const { return m_dirty; }

private:
    ContourId nextContourId();
    std::uint64_t nextLayerId();
    void syncLayerContourIds();

    LaserToolpath m_toolpath;
    ContourId m_nextContourId{1};
    std::uint64_t m_nextLayerId{1};
    bool m_dirty{false};
};

} // namespace lcnc::cam