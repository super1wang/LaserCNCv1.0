#include "modules/cad/services/sketch_manager.h"

#include "core/logging/logger.h"

namespace lcnc::cad {

int SketchManager::addSketch(SketchPlaneKind plane,
                             const std::vector<SketchElement>& elements,
                             const TopoDS_Face& profileFace)
{
    SketchRecord record;
    record.id = m_nextId++;
    // 中文翻译：草图 %1
    record.name = QStringLiteral("Sketch %1").arg(record.id);
    record.plane = plane;
    record.elements.reserve(static_cast<int>(elements.size()));
    for (const auto& element : elements)
        record.elements.append(element);
    record.profileFace = profileFace;
    record.visible = true;
    record.usage = SketchUsageState::Available;
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "SketchManager::addSketch id={} elements={}",
               record.id, elements.size());
    m_sketches.push_back(std::move(record));
    return m_sketches.back().id;
}

bool SketchManager::removeSketch(int sketchId)
{
    for (auto it = m_sketches.begin(); it != m_sketches.end(); ++it) {
        if (it->id == sketchId) {
            LCNC_DEBUG(lcnc::LogCode::Generic,
                       "SketchManager::removeSketch id={}", sketchId);
            m_sketches.erase(it);
            return true;
        }
    }
    return false;
}

bool SketchManager::setSketchVisible(int sketchId, bool visible)
{
    if (auto* record = sketch(sketchId)) {
        record->visible = visible;
        LCNC_DEBUG(lcnc::LogCode::Generic,
                   "SketchManager::setSketchVisible id={} visible={}",
                   sketchId, visible);
        return true;
    }
    return false;
}

bool SketchManager::markSketchUsedByFeature(int sketchId, const QString& featureEntry)
{
    if (auto* record = sketch(sketchId)) {
        record->usage = SketchUsageState::UsedByFeature;
        record->featureEntry = featureEntry;
        record->visible = false;
        LCNC_DEBUG(lcnc::LogCode::Generic,
                   "SketchManager::markSketchUsedByFeature id={} entry={}",
                   sketchId, featureEntry.toStdString());
        return true;
    }
    return false;
}

const SketchRecord* SketchManager::sketch(int sketchId) const
{
    for (const auto& record : m_sketches) {
        if (record.id == sketchId)
            return &record;
    }
    return nullptr;
}

SketchRecord* SketchManager::sketch(int sketchId)
{
    for (auto& record : m_sketches) {
        if (record.id == sketchId)
            return &record;
    }
    return nullptr;
}

SketchManager* SketchManagerRegistry::ensure(DocumentId docId)
{
    if (docId == kInvalidDocumentId)
        return nullptr;
    auto it = m_managers.find(docId);
    if (it != m_managers.end())
        return it->second.get();
    auto manager = std::make_unique<SketchManager>();
    SketchManager* raw = manager.get();
    m_managers.emplace(docId, std::move(manager));
    return raw;
}

SketchManager* SketchManagerRegistry::get(DocumentId docId)
{
    auto it = m_managers.find(docId);
    return it != m_managers.end() ? it->second.get() : nullptr;
}

const SketchManager* SketchManagerRegistry::get(DocumentId docId) const
{
    auto it = m_managers.find(docId);
    return it != m_managers.end() ? it->second.get() : nullptr;
}

void SketchManagerRegistry::erase(DocumentId docId)
{
    m_managers.erase(docId);
}

} // namespace lcnc::cad
