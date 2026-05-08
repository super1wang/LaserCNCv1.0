#pragma once

#include <QList>
#include <QString>
#include <memory>
#include <unordered_map>
#include <vector>

#include "core/project/project_types.h"
#include "core/document/lcnc_document.h"
#include "modules/cad/services/sketch_types.h"

namespace lcnc::cad {

/**
 * @brief Document-level manager for finished sketch records.
 *
 * One instance per project workspace. Machine sections do not own a manager.
 * Owns geometry data, visibility, and used-by-feature state. The active in-progress
 * sketch is still edited via `CadModelingSession`; on `finishSketch`, CadModule
 * pushes the resulting record here for persistence and selection.
 */
class SketchManager
{
public:
    /// Append a new sketch built from an editing session. Returns the assigned id.
    int addSketch(SketchPlaneKind plane,
                  const std::vector<SketchElement>& elements,
                  const TopoDS_Face& profileFace);

    /// Erase a sketch record by id. Returns true if removed.
    bool removeSketch(int sketchId);

    /// Toggle visibility flag for a sketch (overlay/tree only; data preserved).
    bool setSketchVisible(int sketchId, bool visible);

    /// Mark a sketch as consumed by the given feature; auto-hides it.
    bool markSketchUsedByFeature(int sketchId, const QString& featureEntry);

    /// Lookup a sketch by id; returns nullptr if missing.
    const SketchRecord* sketch(int sketchId) const;
    SketchRecord* sketch(int sketchId);

    /// Read-only list of all sketch records (insertion order).
    const std::vector<SketchRecord>& sketches() const { return m_sketches; }

    /// True when the manager owns at least one record.
    bool isEmpty() const { return m_sketches.empty(); }

private:
    int m_nextId{1};
    std::vector<SketchRecord> m_sketches;
};

/**
 * @brief Per-document registry of sketch managers, keyed by DocumentId.
 *
 * CadModule owns one of these and keys managers by project DocumentId.
 */
class SketchManagerRegistry
{
public:
    /// Get-or-create a manager for the document. Returns nullptr if docId is invalid.
    SketchManager* ensure(DocumentId docId);
    /// Lookup an existing manager; returns nullptr if absent.
    SketchManager* get(DocumentId docId);
    const SketchManager* get(DocumentId docId) const;
    /// Erase the manager bound to a document (called on close).
    void erase(DocumentId docId);

private:
    std::unordered_map<DocumentId, std::unique_ptr<SketchManager>> m_managers;
};

} // namespace lcnc::cad
