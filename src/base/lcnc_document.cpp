#include "base/lcnc_document.h"
#include "base/xcaf_utils.h"

// OCC
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <TDataStd_Name.hxx>
#include <TDataStd_Integer.hxx>
#include <TDF_Tool.hxx>
#include <TDF_LabelSequence.hxx>

// IMPLEMENT_STANDARD_RTTIEXT(LcncDocument, TDocStd_Document)

// Internal tag constants for category group labels
static constexpr int kTagWorkpiece  = 10;
static constexpr int kTagMachine    = 11;
static constexpr int kTagAuxiliary  = 12;

// ── Constructor ───────────────────────────────────────────────────────────────
LcncDocument::LcncDocument(int id, const QString& name)
    : TDocStd_Document("BinXCAF")  // default format
    , m_id(id)
    , m_name(name)
{
    SetUndoLimit(100);
    initXcaf();
}

void LcncDocument::initXcaf()
{
    // Initialise XDE tools at the document root
    XCAFDoc_DocumentTool::Set(this->Main());

    // Create persistent category group labels under the root
    TDF_Label root = this->Main();
    m_workpieceGroup  = XcafUtils::findOrCreateChild(root, kTagWorkpiece);
    m_machineGroup    = XcafUtils::findOrCreateChild(root, kTagMachine);
    m_auxiliaryGroup  = XcafUtils::findOrCreateChild(root, kTagAuxiliary);

    XcafUtils::setName(m_workpieceGroup,  QStringLiteral("工件模型"));
    XcafUtils::setName(m_machineGroup,    QStringLiteral("机台模型"));
    XcafUtils::setName(m_auxiliaryGroup,  QStringLiteral("辅助对象"));
}

// ── Identity helpers ───────────────────────────────────────────────────────────
bool LcncDocument::isModified() const
{
    return IsModified();
}

// ── XCAF helpers ───────────────────────────────────────────────────────────────
Handle(XCAFDoc_ShapeTool) LcncDocument::shapeTool() const
{
    return XCAFDoc_DocumentTool::ShapeTool(this->Main());
}

Handle(XCAFDoc_ColorTool) LcncDocument::colorTool() const
{
    return XCAFDoc_DocumentTool::ColorTool(this->Main());
}

// ── Entity management ─────────────────────────────────────────────────────────
TDF_Label LcncDocument::entityGroup(EntityKind kind) const
{
    switch (kind) {
    case EntityKind::Workpiece:  return m_workpieceGroup;
    case EntityKind::Machine:    return m_machineGroup;
    case EntityKind::Auxiliary:  return m_auxiliaryGroup;
    }
    return m_workpieceGroup;
}

TDF_Label LcncDocument::addShapeEntity(const TopoDS_Shape& shape,
                                       const QString&      name,
                                       EntityKind          kind)
{
    // Do NOT use OpenCommand/CommitCommand here: undo-tracking file imports
    // bloats the XCAF transaction stack and can cause crashes during document
    // destruction.  Undo is reserved for interactive CAD operations only.
    //
    // Do NOT call AddComponent(groupLabel, ...) with a raw TDF_Label that is
    // not a proper XDE shape — that corrupts the XCAF internal tree.
    // Register each shape as a top-level free shape and tag it with its
    // EntityKind via TDataStd_Integer so entityLabels() can filter by category.
    Handle(XCAFDoc_ShapeTool) st = shapeTool();
    TDF_Label shapeLabel = st->NewShape();
    st->SetShape(shapeLabel, shape);
    XcafUtils::setName(shapeLabel, name);
    TDataStd_Integer::Set(shapeLabel, static_cast<Standard_Integer>(kind));
    return shapeLabel;
}

TDF_LabelSequence LcncDocument::entityLabels(EntityKind kind) const
{
    // Shapes are stored as top-level XDE free shapes tagged with their
    // EntityKind via a TDataStd_Integer attribute.  Enumerate and filter.
    Handle(XCAFDoc_ShapeTool) st = shapeTool();
    TDF_LabelSequence freeShapes;
    st->GetFreeShapes(freeShapes);

    TDF_LabelSequence result;
    for (int i = 1; i <= freeShapes.Length(); ++i) {
        TDF_Label lbl = freeShapes.Value(i);
        Handle(TDataStd_Integer) attr;
        if (lbl.FindAttribute(TDataStd_Integer::GetID(), attr) &&
            static_cast<EntityKind>(attr->Get()) == kind) {
            result.Append(lbl);
        }
    }
    return result;
}

// ── Undo/Redo ──────────────────────────────────────────────────────────────────
bool LcncDocument::canUndo() const { return GetAvailableUndos() > 0; }
bool LcncDocument::canRedo() const { return GetAvailableRedos() > 0; }

void LcncDocument::undo() { if (canUndo()) Undo(); }
void LcncDocument::redo() { if (canRedo()) Redo(); }

void LcncDocument::openCommand(const QString& /*description*/)
{
    OpenCommand();
}

void LcncDocument::commitCommand() { CommitCommand(); }
void LcncDocument::abortCommand()  { AbortCommand();  }
