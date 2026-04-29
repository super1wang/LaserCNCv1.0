#include "modules/cad/document/cad_document_registry.h"

namespace lcnc::cad {

CadDocumentState* CadDocumentRegistry::ensure(DocumentId docId)
{
    if (docId == kInvalidDocumentId)
        return nullptr;
    auto it = m_states.find(docId);
    if (it != m_states.end())
        return it->second.get();
    auto state = std::make_unique<CadDocumentState>(docId);
    CadDocumentState* raw = state.get();
    m_states.emplace(docId, std::move(state));
    return raw;
}

CadDocumentState* CadDocumentRegistry::get(DocumentId docId)
{
    auto it = m_states.find(docId);
    return it != m_states.end() ? it->second.get() : nullptr;
}

const CadDocumentState* CadDocumentRegistry::get(DocumentId docId) const
{
    auto it = m_states.find(docId);
    return it != m_states.end() ? it->second.get() : nullptr;
}

SketchManager* CadDocumentRegistry::sketchManager(DocumentId docId)
{
    CadDocumentState* state = get(docId);
    return state ? &state->sketchManager() : nullptr;
}

const SketchManager* CadDocumentRegistry::sketchManager(DocumentId docId) const
{
    const CadDocumentState* state = get(docId);
    return state ? &state->sketchManager() : nullptr;
}

void CadDocumentRegistry::erase(DocumentId docId)
{
    m_states.erase(docId);
}

} // namespace lcnc::cad