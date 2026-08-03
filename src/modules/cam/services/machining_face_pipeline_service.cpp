#include "modules/cam/services/machining_face_pipeline_service.h"

#include "core/algorithms/cam/laser_toolpath.h"

#include <algorithm>

#include <TopAbs_ShapeEnum.hxx>
#include <TopoDS.hxx>
#include <TopExp_Explorer.hxx>

namespace lcnc::cam {

void MachiningFacePipelineService::reset() noexcept
{
    m_entries.clear();
    m_nextFaceId = 1;
}

void MachiningFacePipelineService::clearEntries() noexcept
{
    m_entries.clear();
}

void MachiningFacePipelineService::replace(std::vector<Entry> entries)
{
    m_entries = std::move(entries);
    std::uint64_t maxId = 0;
    for (const Entry& entry : m_entries)
        maxId = std::max(maxId, entry.faceId);
    m_nextFaceId = std::max<std::uint64_t>(1, maxId + 1);
}

bool MachiningFacePipelineService::facesShareBoundaryEdge(const TopoDS_Face& first,
                                                           const TopoDS_Face& second)
{
    if (first.IsNull() || second.IsNull())
        return false;
    for (TopExp_Explorer firstEdges(first, TopAbs_EDGE); firstEdges.More(); firstEdges.Next()) {
        const TopoDS_Shape edge = firstEdges.Current();
        for (TopExp_Explorer secondEdges(second, TopAbs_EDGE); secondEdges.More(); secondEdges.Next()) {
            if (edge.IsSame(secondEdges.Current()))
                return true;
        }
    }
    return false;
}

bool MachiningFacePipelineService::containsEquivalent(const std::vector<Entry>& entries,
                                                       const Candidate& candidate)
{
    return std::any_of(entries.cbegin(), entries.cend(), [&candidate](const Entry& entry) {
        return entry.workpieceEntry == candidate.workpieceEntry
            && entry.role == candidate.role
            && !entry.face.IsNull()
            && entry.face.IsSame(candidate.face);
    });
}

bool MachiningFacePipelineService::addManualFace(const TopoDS_Face& face,
                                                  const QString& workpieceEntry)
{
    if (face.IsNull()
        || std::any_of(m_entries.cbegin(), m_entries.cend(), [&face](const Entry& entry) {
            return !entry.face.IsNull() && entry.face.IsSame(face);
        })) {
        return false;
    }
    Entry entry;
    entry.faceId = nextFaceId();
    entry.face = face;
    entry.workpieceEntry = workpieceEntry;
    entry.manual = true;
    m_entries.push_back(std::move(entry));
    return true;
}

bool MachiningFacePipelineService::removeFace(std::uint64_t faceId)
{
    const auto found = std::find_if(m_entries.cbegin(), m_entries.cend(),
                                    [faceId](const Entry& entry) { return entry.faceId == faceId; });
    if (found == m_entries.cend())
        return false;
    m_entries.erase(found);
    return true;
}

MachiningFacePipelineService::RoleChangeResult
MachiningFacePipelineService::setFaceRole(std::uint64_t faceId, MachiningFaceRole role)
{
    const auto found = std::find_if(m_entries.begin(), m_entries.end(),
                                    [faceId](const Entry& entry) { return entry.faceId == faceId; });
    if (found == m_entries.end() || found->role == role)
        return RoleChangeResult::NotFoundOrUnchanged;
    if (role == MachiningFaceRole::CrossSection) {
        const bool intersectsMachiningFace = std::any_of(
            m_entries.cbegin(), m_entries.cend(), [&found](const Entry& other) {
                return other.faceId != found->faceId
                    && other.workpieceEntry == found->workpieceEntry
                    && other.role == MachiningFaceRole::MachiningSurface
                    && facesShareBoundaryEdge(other.face, found->face);
            });
        if (!intersectsMachiningFace)
            return RoleChangeResult::InvalidCrossSection;
    }
    found->role = role;
    found->manual = true;
    return RoleChangeResult::Changed;
}

MachiningFacePipelineService::RebindResult
MachiningFacePipelineService::rebindFromRecords(
    const std::vector<CamDataManager::MachiningFaceRecord>& records,
    const std::vector<RebindSource>& sources)
{
    struct CandidateFace {
        QString workpieceEntry;
        TopoDS_Face face;
        std::uint64_t signature{0};
    };
    std::vector<CandidateFace> candidates;
    for (const RebindSource& source : sources) {
        for (TopExp_Explorer explorer(source.shape, TopAbs_FACE); explorer.More(); explorer.Next()) {
            const TopoDS_Face face = TopoDS::Face(explorer.Current());
            if (!face.IsNull()) {
                candidates.push_back({source.workpieceEntry, face,
                    LaserToolpathBuilder::computeFaceSignature(face)});
            }
        }
    }

    RebindResult result;
    std::vector<Entry> rebound;
    rebound.reserve(records.size());
    for (const CamDataManager::MachiningFaceRecord& record : records) {
        if (record.signature == 0)
            continue;
        const auto found = std::find_if(candidates.cbegin(), candidates.cend(), [&record](const CandidateFace& candidate) {
            return candidate.signature == record.signature
                && (record.workpieceEntry.isEmpty()
                    || candidate.workpieceEntry == record.workpieceEntry);
        });
        if (found == candidates.cend()) {
            result.missingFaces.push_back({record.faceId, record.signature});
            continue;
        }
        Entry entry;
        entry.faceId = record.faceId;
        entry.face = found->face;
        entry.workpieceEntry = record.workpieceEntry;
        entry.manual = record.manual;
        entry.role = record.role;
        rebound.push_back(std::move(entry));
    }
    replace(std::move(rebound));
    result.reboundCount = static_cast<int>(m_entries.size());
    return result;
}

bool MachiningFacePipelineService::replaceAutomaticFaces(const std::vector<Candidate>& candidates)
{
    std::vector<Entry> merged;
    merged.reserve(m_entries.size() + candidates.size());
    for (const Entry& entry : m_entries) {
        if (entry.manual)
            merged.push_back(entry);
    }
    for (const Candidate& candidate : candidates) {
        if (candidate.face.IsNull() || containsEquivalent(merged, candidate))
            continue;
        Entry entry;
        entry.faceId = nextFaceId();
        entry.face = candidate.face;
        entry.workpieceEntry = candidate.workpieceEntry;
        entry.role = candidate.role;
        merged.push_back(std::move(entry));
    }
    if (merged.empty())
        return false;
    m_entries = std::move(merged);
    return true;
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
