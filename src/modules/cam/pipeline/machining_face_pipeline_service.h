#pragma once

#include "core/project/cam/cam_data_manager.h"

#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>

#include <QString>

#include <cstdint>
#include <vector>

namespace lcnc::cam {

/**
 * @brief Authoritative state for the selected machining-face set.
 *
 * The service owns stable face IDs and all mutable OCC face handles.  CAM
 * tasks receive copies of its entries; callers must compare revision() before
 * committing asynchronous results.
 */
class MachiningFacePipelineService final
{
public:
    struct Entry {
        std::uint64_t faceId{0};
        TopoDS_Face face;
        QString workpieceEntry;
        bool manual{false};
        MachiningFaceRole role{MachiningFaceRole::MachiningSurface};
    };

    struct Candidate {
        TopoDS_Face face;
        QString workpieceEntry;
        MachiningFaceRole role{MachiningFaceRole::MachiningSurface};
    };

    struct RebindSource {
        QString workpieceEntry;
        TopoDS_Shape shape;
    };

    struct RebindResult {
        int reboundCount{0};
        struct MissingFace {
            std::uint64_t faceId{0};
            std::uint64_t signature{0};
        };
        std::vector<MissingFace> missingFaces;
    };

    const std::vector<Entry>& entries() const noexcept { return m_entries; }

    void reset() noexcept;
    void clearEntries() noexcept;
    void replace(std::vector<Entry> entries);
    bool addManualFace(const TopoDS_Face& face, const QString& workpieceEntry);
    bool removeFace(std::uint64_t faceId);
    enum class RoleChangeResult { Changed, NotFoundOrUnchanged, InvalidCrossSection };
    RoleChangeResult setFaceRole(std::uint64_t faceId, MachiningFaceRole role);
    bool replaceAutomaticFaces(const std::vector<Candidate>& candidates);
    RebindResult rebindFromRecords(const std::vector<CamDataManager::MachiningFaceRecord>& records,
                                   const std::vector<RebindSource>& sources);
    std::uint64_t revision() const;
    std::vector<CamDataManager::MachiningFaceRecord> persistenceRecords() const;

private:
    std::uint64_t nextFaceId() noexcept { return m_nextFaceId++; }
    static bool facesShareBoundaryEdge(const TopoDS_Face& first, const TopoDS_Face& second);
    static bool containsEquivalent(const std::vector<Entry>& entries,
                                   const Candidate& candidate);
    std::vector<Entry> m_entries;
    std::uint64_t m_nextFaceId{1};
};

} // namespace lcnc::cam
