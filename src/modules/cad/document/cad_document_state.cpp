#include "modules/cad/document/cad_document_state.h"

namespace lcnc::cad {

CadDocumentState::CadDocumentState(DocumentId docId)
    : m_documentId(docId)
{
    m_selectionContext.docId = docId;
}

} // namespace lcnc::cad