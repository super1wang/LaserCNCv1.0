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
#include <TDF_ChildIterator.hxx>
#include <BRepBuilderAPI_Copy.hxx>

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
    openCommand(QStringLiteral("添加形体: %1").arg(name));

    Handle(XCAFDoc_ShapeTool) st = shapeTool();
    TDF_Label shapeLabel = st->NewShape();
    st->SetShape(shapeLabel, shape);
    XcafUtils::setName(shapeLabel, name);

    // Reference the shape under the category group
    TDF_Label groupLabel = entityGroup(kind);
    TDF_Label ref;
    st->AddComponent(groupLabel, shapeLabel, TopLoc_Location());

    commitCommand();
    return shapeLabel;
}

TDF_LabelSequence LcncDocument::entityLabels(EntityKind kind) const
{
    TDF_LabelSequence seq;
    TDF_Label group = entityGroup(kind);
    TDF_ChildIterator it(group, false);
    while (it.More()) {
        seq.Append(it.Value());
        it.Next();
    }
    return seq;
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
