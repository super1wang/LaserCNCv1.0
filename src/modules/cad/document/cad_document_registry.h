#pragma once

#include "core/project/project_types.h"
#include "modules/cad/document/cad_document_state.h"

#include <memory>
#include <unordered_map>

namespace lcnc::cad {

/// Owns per-document CAD state objects keyed by DocumentId.
class CadDocumentRegistry
{
public:
    /// Get-or-create state for a project workspace. Returns nullptr for invalid ids.
    CadDocumentState* ensure(DocumentId docId);

    /// Lookup state for a document. Returns nullptr if absent.
    CadDocumentState* get(DocumentId docId);
    /// Lookup state for a document. Returns nullptr if absent.
    const CadDocumentState* get(DocumentId docId) const;

    /// Convenience access to the document's sketch manager.
    SketchManager* sketchManager(DocumentId docId);
    /// Convenience access to the document's sketch manager.
    const SketchManager* sketchManager(DocumentId docId) const;

    /// Drop state for a closed document.
    void erase(DocumentId docId);

private:
    std::unordered_map<DocumentId, std::unique_ptr<CadDocumentState>> m_states;
};

} // namespace lcnc::cad