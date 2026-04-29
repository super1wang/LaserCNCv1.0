#pragma once

#include "core/document/lcnc_application.h"
#include "modules/cad/selection/cad_selection.h"
#include "modules/cad/services/sketch_manager.h"

namespace lcnc::cad {

/// Presentation flags cached per CAD workpiece document.
struct CadPresentationState {
    int selectedSketchId{0};
    bool showSketchOverlay{true};
};

/// Aggregates all CAD-owned state for one workpiece document.
class CadDocumentState
{
public:
    /// Create state bound to the given document id.
    explicit CadDocumentState(DocumentId docId);

    /// Document id that owns this state.
    DocumentId documentId() const { return m_documentId; }

    /// Finished sketch manager for this document.
    SketchManager& sketchManager() { return m_sketchManager; }
    /// Finished sketch manager for this document.
    const SketchManager& sketchManager() const { return m_sketchManager; }

    /// Last normalized CAD selection context for this document.
    selection::CadSelectionContext& selectionContext() { return m_selectionContext; }
    /// Last normalized CAD selection context for this document.
    const selection::CadSelectionContext& selectionContext() const { return m_selectionContext; }

    /// Presentation/cache flags for model tree and sketch overlays.
    CadPresentationState& presentationState() { return m_presentationState; }
    /// Presentation/cache flags for model tree and sketch overlays.
    const CadPresentationState& presentationState() const { return m_presentationState; }

private:
    DocumentId m_documentId{kInvalidDocumentId};
    SketchManager m_sketchManager;
    selection::CadSelectionContext m_selectionContext;
    CadPresentationState m_presentationState;
};

} // namespace lcnc::cad