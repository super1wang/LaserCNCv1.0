#include "core/document/lcnc_document.h"
#include "core/document/xcaf_utils.h"
#include "core/logging/logger.h"


// OCC
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <TDataStd_Name.hxx>
#include <TDataStd_Integer.hxx>
#include <TDF_Tool.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDF_ChildIterator.hxx>
#include <TCollection_AsciiString.hxx>
#include <XCAFDoc_Location.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS_Iterator.hxx>

#include <QSignalBlocker>

#include <memory>

// IMPLEMENT_STANDARD_RTTIEXT(LcncDocument, TDocStd_Document)

// Internal tag constants for category group labels
static constexpr int kTagProject    = 20;
static constexpr int kTagWorkpiece  = 10;
static constexpr int kTagMachine    = 11;
static constexpr int kTagAuxiliary  = 12;
static constexpr int kTagCam        = 21;
static constexpr int kTagProcess    = 22;

// ── Constructor ───────────────────────────────────────────────────────────────
LcncDocument::LcncDocument(int id, const QString& name)
    : TDocStd_Document("BinXCAF")  // default format
    , m_id(id)
    , m_name(name)
{
    SetUndoLimit(100);
    initXcaf();
}

std::unique_ptr<LcncDocument> LcncDocument::createStandalone(int id, const QString& name)
{
    return std::unique_ptr<LcncDocument>(new LcncDocument(id, name));
}

void LcncDocument::initXcaf()
{
    // Initialise XDE tools at the document root
    XCAFDoc_DocumentTool::Set(this->Main());

    // Create persistent category group labels under the root
    TDF_Label root = this->Main();
    m_projectGroup    = XcafUtils::findOrCreateChild(root, kTagProject);
    m_workpieceGroup  = XcafUtils::findOrCreateChild(root, kTagWorkpiece);
    m_machineGroup    = XcafUtils::findOrCreateChild(root, kTagMachine);
    m_camGroup        = XcafUtils::findOrCreateChild(root, kTagCam);
    m_processGroup    = XcafUtils::findOrCreateChild(root, kTagProcess);
    m_auxiliaryGroup  = XcafUtils::findOrCreateChild(root, kTagAuxiliary);

    // 中文翻译：项目
    XcafUtils::setName(m_projectGroup,    QStringLiteral("Project"));
    // 中文翻译：工件模型
    XcafUtils::setName(m_workpieceGroup,  QStringLiteral("workpiece model"));
    // 中文翻译：机台模型
    XcafUtils::setName(m_machineGroup,    QStringLiteral("Machine model"));
    // 中文翻译：CAM 数据
    XcafUtils::setName(m_camGroup,        QStringLiteral("CAM data"));
    // 中文翻译：加工过程
    XcafUtils::setName(m_processGroup,    QStringLiteral("Process"));
    // 中文翻译：辅助对象
    XcafUtils::setName(m_auxiliaryGroup,  QStringLiteral("helper object"));
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
void LcncDocument::clearEntityKind(EntityKind kind)
{
    TDF_LabelSequence labels = entityLabels(kind);

    MachineKinematics* kin = m_kinematics;
    std::unique_ptr<QSignalBlocker> kinBlocker;
    if (kin)
        kinBlocker = std::make_unique<QSignalBlocker>(kin);

    for (int i = 1; i <= labels.Length(); ++i) {
        TDF_Label label = labels.Value(i);
        const QString entry = XcafUtils::entry(label);
        if (kin) {
            if (kind == EntityKind::Machine)
                kin->unassignShape(entry);
            else if (kind == EntityKind::Workpiece)
                kin->unmountWorkpiece(entry);
        }
        label.ForgetAllAttributes(Standard_True);
    }

    if (kind == EntityKind::Machine)
        m_machineTree.clear();
    else if (kind == EntityKind::Cam)
        m_camTree.clear();
    else if (kind == EntityKind::Workpiece)
        m_workpieceTree.clear();
}

void LcncDocument::clearProjectData()
{
    clearEntityKind(EntityKind::Workpiece);
    clearEntityKind(EntityKind::Machine);
    clearEntityKind(EntityKind::Cam);
    clearEntityKind(EntityKind::Auxiliary);
    m_treeUndoStack.clear();
    m_treeRedoStack.clear();
    if (m_kinematics)
        m_kinematics->clear();
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

void LcncDocument::removeShapeEntity(const QString& entry)
{
    Handle(XCAFDoc_ShapeTool) st = shapeTool();
    TDF_LabelSequence freeShapes;
    st->GetFreeShapes(freeShapes);

    for (int i = 1; i <= freeShapes.Length(); ++i) {
        TDF_Label lbl = freeShapes.Value(i);
        if (XcafUtils::entry(lbl) != entry) continue;

        // Forget all attributes: removes TNaming_NamedShape (so IsShape() returns
        // false and GetFreeShapes() will no longer return this label), the kind
        // tag (TDataStd_Integer), name and any colors.
        lbl.ForgetAllAttributes(Standard_True);

        // Remove from in-memory hierarchy trees so the "准备" tab rebuild is clean.
        // Empty virtual parent nodes are pruned automatically (request 2).
        struct TreeHelper {
            static bool remove(QList<ShapeTreeNode>& nodes, const QString& e) {
                for (int j = 0; j < nodes.size(); ++j) {
                    if (nodes[j].entry == e) { nodes.removeAt(j); return true; }
                    if (remove(nodes[j].children, e)) {
                        // Prune now-empty virtual group nodes as we unwind
                        if (nodes[j].entry.isEmpty() && nodes[j].children.isEmpty())
                            nodes.removeAt(j);
                        return true;
                    }
                }
                return false;
            }
        };
        TreeHelper::remove(m_machineTree,   entry);
        TreeHelper::remove(m_workpieceTree, entry);
        TreeHelper::remove(m_camTree,       entry);
        break;
    }
}

// ── Assembly import ───────────────────────────────────────────────────────────

/// Read TDataStd_Name from a label, preserving Unicode (Chinese, etc.).
static QString xcafLabelName(const TDF_Label& lbl)
{
    Handle(TDataStd_Name) attr;
    if (lbl.FindAttribute(TDataStd_Name::GetID(), attr)) {
        const TCollection_ExtendedString& ext = attr->Get();
        std::wstring ws;
        ws.reserve(static_cast<size_t>(ext.Length()));
        for (Standard_Integer i = 1; i <= ext.Length(); ++i)
            ws.push_back(static_cast<wchar_t>(ext.Value(i)));
        return QString::fromStdWString(ws);
    }
    return QString();
}

/// Recursively build a ShapeTreeNode from an XCAF design label.
///
/// - Leaf labels (no components): shape is registered via addShapeEntity;
///   node.entry is set to the created XCAF label entry.
/// - Assembly labels: node.entry is empty (virtual group); children are
///   populated by recursing into each component with its composed location.
static LcncDocument::ShapeTreeNode importBuildTreeR(
    LcncDocument*                    doc,
    const Handle(XCAFDoc_ShapeTool)& st,
    const TDF_Label&                 designLbl,
    const TopLoc_Location&           parentLoc,
    const QString&                   fallbackName,
    LcncDocument::EntityKind         kind)
{
    LcncDocument::ShapeTreeNode node;
    node.displayName = xcafLabelName(designLbl);
    if (node.displayName.isEmpty())
        node.displayName = fallbackName;

    // Try XDE GetComponents first; if it fails, fall back to scanning child
    // labels for any reference label (handles STEP files where IsAssembly
    // may not be set but NAUO component-references still exist as children).
    TDF_LabelSequence comps;
    st->GetComponents(designLbl, comps);
    if (comps.IsEmpty()) {
        TDF_ChildIterator childIt(designLbl);
        for (; childIt.More(); childIt.Next()) {
            TDF_Label child = childIt.Value();
            if (st->IsReference(child))
                comps.Append(child);
        }
    }
    if (comps.IsEmpty()) {
        // ── Leaf: register as entity ──────────────────────────────────────
        TopoDS_Shape sh = st->GetShape(designLbl);
        if (sh.IsNull()) return node;
        TopLoc_Location loc = parentLoc * sh.Location();
        if (!loc.IsIdentity())
            sh = sh.Located(loc);
        TDF_Label lbl = doc->addShapeEntity(sh, node.displayName, kind);
        node.entry = XcafUtils::entry(lbl);
        return node;
    }

    // ── Assembly: recurse into components ─────────────────────────────────
    for (int i = 1; i <= comps.Length(); ++i) {
        TDF_Label compRef = comps.Value(i);

        TopLoc_Location compLoc;
        Handle(XCAFDoc_Location) locAttr;
        if (compRef.FindAttribute(XCAFDoc_Location::GetID(), locAttr))
            compLoc = locAttr->Get();
        const TopLoc_Location childLoc = parentLoc * compLoc;

        // Prefer the component-reference (instance) name, then the design name
        QString nm = xcafLabelName(compRef);
        TDF_Label referred;
        if (!st->GetReferredShape(compRef, referred)) {
            // No referred design label — import the component shape directly
            TopoDS_Shape sh = st->GetShape(compRef);
            if (!sh.IsNull()) {
                if (!childLoc.IsIdentity())
                    sh = sh.Located(childLoc * sh.Location());
                if (nm.isEmpty()) nm = QStringLiteral("Part_%1").arg(i);
                TDF_Label lbl = doc->addShapeEntity(sh, nm, kind);
                LcncDocument::ShapeTreeNode leaf;
                leaf.entry       = XcafUtils::entry(lbl);
                leaf.displayName = nm;
                node.children.append(leaf);
            }
            continue;
        }
        if (nm.isEmpty()) nm = xcafLabelName(referred);
        if (nm.isEmpty()) nm = QStringLiteral("Part_%1").arg(i);

        LcncDocument::ShapeTreeNode child =
            importBuildTreeR(doc, st, referred, childLoc, nm, kind);
        if (!child.displayName.isEmpty() || !child.entry.isEmpty())
            node.children.append(child);
    }
    return node;
}

void LcncDocument::importFromXcaf(const Handle(TDocStd_Document)& xdeDoc,
                                   EntityKind kind)
{
    Handle(XCAFDoc_ShapeTool) st = XCAFDoc_DocumentTool::ShapeTool(xdeDoc->Main());
    TDF_LabelSequence freeShapes;
    st->GetFreeShapes(freeShapes);

    LCNC_DEBUG(lcnc::LogCode::Generic,
               "LcncDocument::importFromXcaf kind={} freeShapes={}",
               static_cast<int>(kind), freeShapes.Length());

    auto& tree = (kind == EntityKind::Machine)
        ? m_machineTree
        : (kind == EntityKind::Cam ? m_camTree : m_workpieceTree);

    for (int i = 1; i <= freeShapes.Length(); ++i) {
        const TDF_Label& root     = freeShapes.Value(i);
        const QString    rootName = xcafLabelName(root);
        ShapeTreeNode node =
            importBuildTreeR(this, st, root, TopLoc_Location(), rootName, kind);
        if (!node.displayName.isEmpty() || !node.entry.isEmpty())
            tree.append(node);
    }
}

const QList<LcncDocument::ShapeTreeNode>& LcncDocument::entityTree(EntityKind kind) const
{
    if (kind == EntityKind::Machine)
        return m_machineTree;
    if (kind == EntityKind::Cam)
        return m_camTree;
    return m_workpieceTree;
}

// ── Flat import (one-level, for machine models) ───────────────────────────────

void LcncDocument::importFromXcafFlat(const Handle(TDocStd_Document)& xdeDoc,
                                       EntityKind kind)
{
    Handle(XCAFDoc_ShapeTool) st = XCAFDoc_DocumentTool::ShapeTool(xdeDoc->Main());
    TDF_LabelSequence freeShapes;
    st->GetFreeShapes(freeShapes);

    auto& tree = (kind == EntityKind::Machine)
        ? m_machineTree
        : (kind == EntityKind::Cam ? m_camTree : m_workpieceTree);

    for (int ri = 1; ri <= freeShapes.Length(); ++ri) {
        const TDF_Label& root     = freeShapes.Value(ri);
        const QString    rootName = xcafLabelName(root);

        // Get direct components of this free shape (assembly top level)
        TDF_LabelSequence comps;
        st->GetComponents(root, comps);

        if (comps.IsEmpty()) {
            // Not an assembly — add as a single entity
            TopoDS_Shape sh = st->GetShape(root);
            if (sh.IsNull()) continue;
            QString nm = rootName.isEmpty() ? QStringLiteral("Part_%1").arg(ri) : rootName;
            TDF_Label lbl = addShapeEntity(sh, nm, kind);
            ShapeTreeNode node;
            node.entry       = XcafUtils::entry(lbl);
            node.displayName = nm;
            tree.append(node);
            continue;
        }

        // Assembly: add each direct component as one entity (no further recursion)
        for (int ci = 1; ci <= comps.Length(); ++ci) {
            TDF_Label compRef = comps.Value(ci);

            // Prefer the component-reference (instance) name, then the design name
            QString nm = xcafLabelName(compRef);

            // Get component placement (instance location)
            TopLoc_Location compLoc;
            Handle(XCAFDoc_Location) locAttr;
            if (compRef.FindAttribute(XCAFDoc_Location::GetID(), locAttr))
                compLoc = locAttr->Get();

            // Resolve to the design label to get the full assembled shape
            TDF_Label referred;
            TopoDS_Shape sh;
            if (st->GetReferredShape(compRef, referred)) {
                if (nm.isEmpty()) nm = xcafLabelName(referred);
                // GetShape on the design label returns the full assembled compound
                sh = st->GetShape(referred);
            } else {
                sh = st->GetShape(compRef);
            }

            if (sh.IsNull()) continue;

            // Apply the component instance location
            if (!compLoc.IsIdentity())
                sh = sh.Located(compLoc * sh.Location());

            if (nm.isEmpty())
                nm = QStringLiteral("Part_%1_%2").arg(ri).arg(ci);

            TDF_Label lbl = addShapeEntity(sh, nm, kind);
            ShapeTreeNode node;
            node.entry       = XcafUtils::entry(lbl);
            node.displayName = nm;
            tree.append(node);
        }
    }
}

// ── Root-only import (no decomposition) ───────────────────────────────────────

void LcncDocument::importFromXcafRoots(const Handle(TDocStd_Document)& xdeDoc,
                                        EntityKind kind)
{
    Handle(XCAFDoc_ShapeTool) st = XCAFDoc_DocumentTool::ShapeTool(xdeDoc->Main());
    TDF_LabelSequence freeShapes;
    st->GetFreeShapes(freeShapes);

    auto& tree = (kind == EntityKind::Machine)
        ? m_machineTree
        : (kind == EntityKind::Cam ? m_camTree : m_workpieceTree);

    for (int i = 1; i <= freeShapes.Length(); ++i) {
        const TDF_Label& root = freeShapes.Value(i);
        TopoDS_Shape sh = st->GetShape(root);
        if (sh.IsNull()) continue;
        QString nm = xcafLabelName(root);
        if (nm.isEmpty()) nm = QStringLiteral("Part_%1").arg(i);
        TDF_Label lbl = addShapeEntity(sh, nm, kind);
        ShapeTreeNode node;
        node.entry       = XcafUtils::entry(lbl);
        node.displayName = nm;
        tree.append(node);
    }
}

// ── Machine kinematics ─────────────────────────────────────────────────────────

MachineKinematics* LcncDocument::machineKinematics()
{
    if (!m_kinematics)
        m_kinematics = new MachineKinematics(nullptr);
    return m_kinematics;
}

// ── Undo/Redo ──────────────────────────────────────────────────────────────────
bool LcncDocument::canUndo() const { return GetAvailableUndos() > 0; }
bool LcncDocument::canRedo() const { return GetAvailableRedos() > 0; }

void LcncDocument::undo()
{
    if (canUndo()) {
        if (!m_treeUndoStack.isEmpty()) {
            // push current state onto redo stack before overwriting
            m_treeRedoStack.push_back({m_workpieceTree, m_machineTree, m_camTree});
            TreeSnapshot snap = m_treeUndoStack.takeLast();
            m_workpieceTree = snap.workpieceTree;
            m_machineTree   = snap.machineTree;
            m_camTree       = snap.camTree;
        }
        Undo();
    }
}

void LcncDocument::redo()
{
    if (canRedo()) {
        if (!m_treeRedoStack.isEmpty()) {
            // push current state onto undo stack before overwriting
            m_treeUndoStack.push_back({m_workpieceTree, m_machineTree, m_camTree});
            TreeSnapshot snap = m_treeRedoStack.takeLast();
            m_workpieceTree = snap.workpieceTree;
            m_machineTree   = snap.machineTree;
            m_camTree       = snap.camTree;
        }
        Redo();
    }
}

void LcncDocument::openCommand(const QString& /*description*/)
{
    // Snapshot the trees BEFORE the command modifies them so that undo can
    // restore the exact pre-command hierarchy (including virtual group nodes).
    m_treeUndoStack.push_back({m_workpieceTree, m_machineTree, m_camTree});
    m_treeRedoStack.clear();
    OpenCommand();
}

void LcncDocument::commitCommand() { CommitCommand(); }
void LcncDocument::abortCommand()
{
    // Restore the tree to the state it was in before the aborted command.
    if (!m_treeUndoStack.isEmpty()) {
        TreeSnapshot snap = m_treeUndoStack.takeLast();
        m_workpieceTree = snap.workpieceTree;
        m_machineTree   = snap.machineTree;
        m_camTree       = snap.camTree;
    }
    AbortCommand();
}

// syncEntityTreesFromXcaf() removed: tree hierarchy is now maintained via
// per-command TreeSnapshot stacks (m_treeUndoStack / m_treeRedoStack) pushed
// inside openCommand() and restored inside undo()/redo().
