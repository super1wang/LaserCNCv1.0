#pragma once

#include <QString>
#include <QList>

// OCC
#include <Standard_Handle.hxx>
#include <Standard_Type.hxx>
#include <TDocStd_Document.hxx>
#include <TDF_Label.hxx>
#include <TDF_LabelSequence.hxx>
#include <TopoDS_Shape.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_DocumentTool.hxx>

/**
 * @brief Extends TDocStd_Document with LaserCNC-specific metadata.
 *
 * Holds the XCAF data tree, undo/redo state, and entity categorisation
 * (Workpiece, Machine, Auxiliary). All CAD data is stored via the
 * XCAFDoc_ShapeTool / ColorTool layers as TDF_Label entries.
 */
class LcncDocument : public TDocStd_Document
{
public:
    DEFINE_STANDARD_RTTI_INLINE(LcncDocument, TDocStd_Document)
    // Entity categories for the left-side tree panel
    enum class EntityKind : int {
        Workpiece  = 0,
        Machine    = 1,
        Auxiliary  = 2
    };

    // ── Identity ─────────────────────────────────────────────────────────────
    int     id()       const { return m_id; }
    QString name()     const { return m_name; }
    QString filePath() const { return m_filePath; }
    bool    isModified() const;

    void setName(const QString& name)         { m_name = name; }
    void setFilePath(const QString& filePath) { m_filePath = filePath; }

    // ── XCAF helpers ─────────────────────────────────────────────────────────
    Handle(XCAFDoc_ShapeTool) shapeTool() const;
    Handle(XCAFDoc_ColorTool) colorTool() const;

    // ── Entity management ────────────────────────────────────────────────────
    /// @return group root label for the given category
    TDF_Label entityGroup(EntityKind kind) const;

    /// Create a new named shape entity under the given category
    TDF_Label addShapeEntity(const TopoDS_Shape& shape,
                             const QString&      name,
                             EntityKind          kind = EntityKind::Workpiece);

    /// Enumerate all top-level entities (for tree building)
    TDF_LabelSequence entityLabels(EntityKind kind) const;

    // ── Undo/Redo ─────────────────────────────────────────────────────────────
    bool canUndo() const;
    bool canRedo() const;
    void undo();
    void redo();

    void openCommand(const QString& description = QString());
    void commitCommand();
    void abortCommand();

private:
    friend class LcncApplication;

    LcncDocument(int id, const QString& name);

    void initXcaf();

    int     m_id;
    QString m_name;
    QString m_filePath;

    // Category group labels (top-level organising nodes)
    TDF_Label m_workpieceGroup;
    TDF_Label m_machineGroup;
    TDF_Label m_auxiliaryGroup;
};

DEFINE_STANDARD_HANDLE(LcncDocument, TDocStd_Document)
