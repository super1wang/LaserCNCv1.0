#include "modules/cam/services/machining_face_pipeline_service.h"

#include "core/algorithms/cam/laser_toolpath.h"

#include <algorithm>

namespace lcnc::cam {

void MachiningFacePipelineService::reset() noexcept
{
    m_entries.clear();
    m_nextFaceId = 1;
}

void MachiningFacePipelineService::replace(std::vector<Entry> entries)
{
    m_entries = std::move(entries);
    std::uint64_t maxId = 0;
    for (const Entry& entry : m_entries)
        maxId = std::max(maxId, entry.faceId);
    m_nextFaceId = std::max<std::uint64_t>(1, maxId + 1);
}

std::uint64_t MachiningFacePipelineService::revision() const
{
    std::uint64_t hash = 1469598103934665603ull;
    const auto mix = [&hash](std::uint64_t value) {
        hash ^= value + 0x9e3779b97f4a7c15ull + (hash << 6) + (hash >> 2);
    };
    for (const Entry& entry : m_entries) {
        mix(entry.faceId);
        mix(entry.face.IsNull() ? 0 : LaserToolpathBuilder::computeFaceSignature(entry.face));
        mix(static_cast<std::uint64_t>(entry.role));
        mix(entry.manual ? 1 : 0);
        for (const QChar character : entry.workpieceEntry)
            mix(character.unicode());
    }
    return hash;
}

std::vector<CamDataManager::MachiningFaceRecord>
MachiningFacePipelineService::persistenceRecords() const
{
    std::vector<CamDataManager::MachiningFaceRecord> records;
    records.reserve(m_entries.size());
    for (const Entry& entry : m_entries) {
        CamDataManager::MachiningFaceRecord record;
        record.faceId = entry.faceId;
        record.workpieceEntry = entry.workpieceEntry;
        record.manual = entry.manual;
        record.role = entry.role;
        record.signature = entry.face.IsNull()
            ? 0 : LaserToolpathBuilder::computeFaceSignature(entry.face);
        records.push_back(record);
    }
    return records;
}

} // namespace lcnc::cam
