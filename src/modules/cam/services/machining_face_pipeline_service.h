#pragma once

#include "core/project/cam/cam_data_manager.h"

#include <TopoDS_Face.hxx>

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

    const std::vector<Entry>& entries() const noexcept { return m_entries; }
    std::vector<Entry>& entries() noexcept { return m_entries; }
    std::uint64_t nextFaceId() noexcept { return m_nextFaceId++; }

    void reset() noexcept;
    void replace(std::vector<Entry> entries);
    std::uint64_t revision() const;
    std::vector<CamDataManager::MachiningFaceRecord> persistenceRecords() const;

private:
    std::vector<Entry> m_entries;
    std::uint64_t m_nextFaceId{1};
};

} // namespace lcnc::cam
