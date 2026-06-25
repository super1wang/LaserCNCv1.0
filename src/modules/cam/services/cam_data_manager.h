#pragma once

#include "core/algorithms/cam/laser_toolpath.h"
#include "modules/cam/contracts/cam_data_contracts.h"

#include <QHash>
#include <QList>
#include <QColor>

#include <cstdint>

namespace lcnc::cam {

/**
 * @brief Project-scoped CAM runtime data owner.
 *
 * Owns dense/non-OCC CAM runtime data such as sampled points, lead-ins,
 * machine coordinates, and parameters. Sparse OCC geometry that should be
 * visible/selectable in the project tree is mirrored into the CAM document by
 * CamModule; dense point arrays intentionally stay here to avoid bloating OCAF.
 *
 * ID stability
 * ------------
 * Contour and layer IDs are paired with a deterministic ``signature``
 * (see LaserContour::signature / ToolpathLayer::signature) so that
 * re-running generateToolpath() preserves the same IDs across regenerations
 * and across sessions when the toolpath state is persisted via
 * saveToolpathToDir / loadToolpathFromDir.
 *
 * The two ``m_signatureTo*`` maps are the "historical mapping" – they remember
 * which ID was assigned to each signature from the previous generation.
 * ensureContourIds() / ensureToolpathLayers() consult these maps first; if a
 * signature is found, the old ID is reused.
 */
class CamDataManager
{
public:
    LaserToolpath& toolpath() { return m_toolpath; }
    const LaserToolpath& toolpath() const { return m_toolpath; }

    bool hasToolpath() const { return m_toolpath.contourCount() > 0; }
    void clearToolpath(bool resetIds = true);

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

    /// --- ID stability helpers (see class doc) ---

    /// Flush the current signature→id mapping into persistent tables.
    /// Called by CamModule after a successful generateToolpath().
    void commitToolpathStates();

    /// Restore signature→id tables from a prior session (called on project load).
    void restoreSignatureTables(const QHash<std::uint64_t, std::uint64_t>& sigToContour,
                                const QHash<std::uint64_t, std::uint64_t>& sigToLayer,
                                ContourId nextContour,
                                std::uint64_t nextLayer);

    /// Replace the entire toolpath in one shot (used by loadToolpathFromDir).
    void replaceToolpath(LaserToolpath&& toolpath,
                         ContourId nextContour,
                         std::uint64_t nextLayer);

    /// Access the current signature mapping (for persistence).
    QHash<std::uint64_t, std::uint64_t> signatureToContourId() const { return m_signatureToContourId; }
    QHash<std::uint64_t, std::uint64_t> signatureToLayerId() const { return m_signatureToLayerId; }

private:
    ContourId nextContourId();
    std::uint64_t nextLayerId();
    void syncLayerContourIds();

    LaserToolpath m_toolpath;
    ContourId m_nextContourId{1};
    std::uint64_t m_nextLayerId{1};
    bool m_dirty{false};

    /// Deterministic signature → allocated id maps (see ID stability doc above).
    QHash<std::uint64_t, std::uint64_t> m_signatureToContourId;
    QHash<std::uint64_t, std::uint64_t> m_signatureToLayerId;
};

} // namespace lcnc::cam