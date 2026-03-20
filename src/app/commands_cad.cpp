#include "app/commands_cad.h"
#include "base/lcnc_application.h"
#include "base/lcnc_document.h"
#include "base/xcaf_utils.h"
#include "gui/gui_application.h"
#include "gui/gui_document.h"

// OCC — Primitives
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>

// OCC — Transforms
#include <BRepBuilderAPI_Transform.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>

// OCC — Boolean
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Common.hxx>

// OCC — Measurement
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <BRep_Tool.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <GeomLProp_SLProps.hxx>
#include <Geom_Surface.hxx>

// OCC — XCAF
#include <XCAFDoc_ShapeTool.hxx>
#include <TDF_LabelSequence.hxx>

// OCC — AIS (for viewport selection query)
#include <AIS_Shape.hxx>

// Qt
#include <QAction>
#include <QDialog>
#include <QFormLayout>
#include <QVBoxLayout>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QLabel>
#include <QMessageBox>
#include <QIcon>

#include <cmath>

// =============================================================================
// Internal helpers
// =============================================================================

namespace {

struct EntityInfo {
    TDF_Label    label;
    QString      name;
    TopoDS_Shape shape;
};

/// Collect all Workpiece-category entities from a document.
QList<EntityInfo> collectEntities(LcncDocument* doc)
{
    QList<EntityInfo> out;
    if (!doc) return out;
    TDF_LabelSequence labels = doc->entityLabels(LcncDocument::EntityKind::Workpiece);
    Handle(XCAFDoc_ShapeTool) st = doc->shapeTool();
    for (int i = 1; i <= labels.Length(); ++i) {
        TDF_Label lbl = labels.Value(i);
        EntityInfo info;
        info.label = lbl;
        info.name  = XcafUtils::name(lbl);
        if (info.name.isEmpty())
            info.name = QString("形体 %1").arg(i);
        info.shape = st->GetShape(lbl);
        out.append(info);
    }
    return out;
}

/// Check whether the active document has at least minCount workpiece entities.
bool hasEntities(IAppContext* ctx, int minCount = 1)
{
    LcncDocument* doc = ctx->activeDocument();
    if (!doc) return false;
    return doc->entityLabels(LcncDocument::EntityKind::Workpiece).Length() >= minCount;
}

/// Create or reuse the active document, add the shape, and refresh the display.
void commitShape(IAppContext* ctx, const TopoDS_Shape& shape, const QString& name)
{
    LcncDocument* doc = ctx->activeDocument();
    if (!doc)
        doc = ctx->app()->newDocument(name);

    doc->addShapeEntity(shape, name, LcncDocument::EntityKind::Workpiece);

    if (auto* gd = ctx->guiApp()->guiDocument(doc->id())) {
        gd->rebuildDisplay();
        gd->fitAll();
    }
    ctx->app()->notifyDocumentModified(doc->id());
    ctx->updateCommandStates();
}

/// Refresh display after an in-place shape update.
void refreshDocument(IAppContext* ctx)
{
    LcncDocument* doc = ctx->activeDocument();
    if (!doc) return;
    if (auto* gd = ctx->guiApp()->guiDocument(doc->id())) {
        gd->rebuildDisplay();
        gd->fitAll();
    }
    ctx->app()->notifyDocumentModified(doc->id());
    ctx->updateCommandStates();
}

/// Build a QDoubleSpinBox with common settings.
QDoubleSpinBox* makeSpin(double val, double lo, double hi,
                         int decimals = 3, const QString& suffix = " mm")
{
    auto* sp = new QDoubleSpinBox;
    sp->setRange(lo, hi);
    sp->setValue(val);
    sp->setDecimals(decimals);
    sp->setSuffix(suffix);
    return sp;
}

/// Get the outward normal of the first face found in a shape.
gp_Vec firstFaceNormal(const TopoDS_Shape& shape)
{
    for (TopExp_Explorer ex(shape, TopAbs_FACE); ex.More(); ex.Next()) {
        TopoDS_Face face = TopoDS::Face(ex.Current());
        Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
        if (surf.IsNull()) continue;
        Standard_Real u1, u2, v1, v2;
        surf->Bounds(u1, u2, v1, v2);
        GeomLProp_SLProps props(surf, (u1 + u2) * 0.5, (v1 + v2) * 0.5, 1, 1e-6);
        if (props.IsNormalDefined()) {
            gp_Dir d = props.Normal();
            return gp_Vec(d.X(), d.Y(), d.Z());
        }
    }
    return gp_Vec(0, 0, 1); // fallback
}

/// Return entities from 'all' that are currently selected in the viewport.
QList<EntityInfo> selectedEntities(IAppContext* ctx, const QList<EntityInfo>& all)
{
    GuiDocument* gd = ctx->activeGuiDocument();
    if (!gd) return {};
    const Handle(AIS_InteractiveContext)& aisCtx = gd->context();
    if (aisCtx.IsNull()) return {};
    QList<EntityInfo> result;
    for (const EntityInfo& e : all) {
        Handle(AIS_Shape) ais = gd->aisShape(XcafUtils::entry(e.label));
        if (!ais.IsNull() && aisCtx->IsSelected(ais))
            result.append(e);
    }
    return result;
}

/// Find the document-list index of 'target' by label entry.
int entityIndex(const QList<EntityInfo>& all, const EntityInfo& target)
{
    const QString entry = XcafUtils::entry(target.label);
    for (int i = 0; i < all.size(); ++i)
        if (XcafUtils::entry(all[i].label) == entry)
            return i;
    return 0;
}

/// Populate 'cb' with all entities.
/// If sel is non-empty a sentinel "[\u5f53\u524d\u9009\u4e2d] ..." is prepended and set as default.
/// Returns true when the sentinel was added.
bool setupEntityCombo(QComboBox* cb,
                      const QList<EntityInfo>& all,
                      const QList<EntityInfo>& sel)
{
    cb->clear();
    const bool hasSentinel = !sel.isEmpty();
    if (hasSentinel) {
        if (sel.size() == 1)
            cb->addItem(QObject::tr("[\u5f53\u524d\u9009\u4e2d] %1").arg(sel.front().name));
        else
            cb->addItem(QObject::tr("[\u5f53\u524d\u9009\u4e2d] %1 \u4e2a\u5f62\u4f53").arg(sel.size()));
    }
    for (const auto& e : all)
        cb->addItem(e.name);
    cb->setCurrentIndex(0);
    return hasSentinel;
}

} // namespace

// =============================================================================
// Primitive creation commands
// =============================================================================

// ── CmdCreateBox ──────────────────────────────────────────────────────────────
CmdCreateBox::CmdCreateBox(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/box.svg"), tr("长方体"), this);
    a->setStatusTip(tr("创建长方体基本体"));
    setAction(a);
}

void CmdCreateBox::execute()
{
    QDialog dlg;
    dlg.setWindowTitle(tr("创建长方体"));
    auto* form = new QFormLayout;
    auto* spX = makeSpin(100, 0.001, 10000);
    auto* spY = makeSpin(100, 0.001, 10000);
    auto* spZ = makeSpin(50,  0.001, 10000);
    form->addRow(tr("长 (X):"), spX);
    form->addRow(tr("宽 (Y):"), spY);
    form->addRow(tr("高 (Z):"), spZ);

    auto* btns = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(btns, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(btns, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
    auto* vl = new QVBoxLayout(&dlg);
    vl->addLayout(form);
    vl->addWidget(btns);

    if (dlg.exec() != QDialog::Accepted) return;

    BRepPrimAPI_MakeBox mk(spX->value(), spY->value(), spZ->value());
    commitShape(context(), mk.Shape(), tr("长方体"));
}

// ── CmdCreateCylinder ─────────────────────────────────────────────────────────
CmdCreateCylinder::CmdCreateCylinder(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/cylinder.svg"), tr("圆柱体"), this);
    a->setStatusTip(tr("创建圆柱体基本体"));
    setAction(a);
}

void CmdCreateCylinder::execute()
{
    QDialog dlg;
    dlg.setWindowTitle(tr("创建圆柱体"));
    auto* form = new QFormLayout;
    auto* spR = makeSpin(50,  0.001, 10000);
    auto* spH = makeSpin(100, 0.001, 10000);
    form->addRow(tr("半径:"), spR);
    form->addRow(tr("高度:"), spH);

    auto* btns = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(btns, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(btns, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
    auto* vl = new QVBoxLayout(&dlg);
    vl->addLayout(form);
    vl->addWidget(btns);

    if (dlg.exec() != QDialog::Accepted) return;

    BRepPrimAPI_MakeCylinder mk(spR->value(), spH->value());
    commitShape(context(), mk.Shape(), tr("圆柱体"));
}

// ── CmdCreateSphere ───────────────────────────────────────────────────────────
CmdCreateSphere::CmdCreateSphere(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/sphere.svg"), tr("球体"), this);
    a->setStatusTip(tr("创建球体基本体"));
    setAction(a);
}

void CmdCreateSphere::execute()
{
    QDialog dlg;
    dlg.setWindowTitle(tr("创建球体"));
    auto* form = new QFormLayout;
    auto* spR = makeSpin(50, 0.001, 10000);
    form->addRow(tr("半径:"), spR);

    auto* btns = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(btns, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(btns, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
    auto* vl = new QVBoxLayout(&dlg);
    vl->addLayout(form);
    vl->addWidget(btns);

    if (dlg.exec() != QDialog::Accepted) return;

    BRepPrimAPI_MakeSphere mk(spR->value());
    commitShape(context(), mk.Shape(), tr("球体"));
}

// ── CmdCreateCone ─────────────────────────────────────────────────────────────
CmdCreateCone::CmdCreateCone(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/cone.svg"), tr("圆锥体"), this);
    a->setStatusTip(tr("创建圆锥体（或截锥体）"));
    setAction(a);
}

void CmdCreateCone::execute()
{
    QDialog dlg;
    dlg.setWindowTitle(tr("创建圆锥体"));
    auto* form = new QFormLayout;
    auto* spR1 = makeSpin(50, 0,     10000); spR1->setSpecialValueText(tr("0 (尖顶)"));
    auto* spR2 = makeSpin(0,  0,     10000); spR2->setSpecialValueText(tr("0 (尖顶)"));
    auto* spH  = makeSpin(100, 0.001, 10000);
    form->addRow(tr("底部半径 R1:"), spR1);
    form->addRow(tr("顶部半径 R2:"), spR2);
    form->addRow(tr("高度:"),        spH);

    auto* note = new QLabel(tr("提示: R2=0 时为圆锥，R1=R2 时为圆柱"));
    note->setWordWrap(true);

    auto* btns = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(btns, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(btns, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
    auto* vl = new QVBoxLayout(&dlg);
    vl->addLayout(form);
    vl->addWidget(note);
    vl->addWidget(btns);

    if (dlg.exec() != QDialog::Accepted) return;

    const double r1 = spR1->value(), r2 = spR2->value(), h = spH->value();
    if (r1 <= 0.0 && r2 <= 0.0) {
        QMessageBox::warning(nullptr, tr("圆锥体"), tr("R1 和 R2 不能同时为零"));
        return;
    }

    BRepPrimAPI_MakeCone mk(r1, r2, h);
    commitShape(context(), mk.Shape(), tr("圆锥体"));
}

// ── CmdCreateTorus ────────────────────────────────────────────────────────────
CmdCreateTorus::CmdCreateTorus(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/torus.svg"), tr("圆环体"), this);
    a->setStatusTip(tr("创建圆环体基本体"));
    setAction(a);
}

void CmdCreateTorus::execute()
{
    QDialog dlg;
    dlg.setWindowTitle(tr("创建圆环体"));
    auto* form = new QFormLayout;
    auto* spR1 = makeSpin(60, 1.0, 10000);
    auto* spR2 = makeSpin(15, 0.001, 10000);
    form->addRow(tr("主半径 R1 (中心距离):"), spR1);
    form->addRow(tr("管半径 R2 (截面半径):"), spR2);

    auto* btns = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(btns, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(btns, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
    auto* vl = new QVBoxLayout(&dlg);
    vl->addLayout(form);
    vl->addWidget(btns);

    if (dlg.exec() != QDialog::Accepted) return;

    const double r1 = spR1->value(), r2 = spR2->value();
    if (r1 <= r2) {
        QMessageBox::warning(nullptr, tr("圆环体"), tr("主半径 R1 必须大于管半径 R2"));
        return;
    }

    BRepPrimAPI_MakeTorus mk(r1, r2);
    commitShape(context(), mk.Shape(), tr("圆环体"));
}

// =============================================================================
// Transform operations
// =============================================================================

// ── CmdMoveShape ──────────────────────────────────────────────────────────────
CmdMoveShape::CmdMoveShape(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/move.svg"), tr("移动"), this);
    a->setStatusTip(tr("沿 X/Y/Z 方向平移形体"));
    setAction(a);
}

bool CmdMoveShape::isEnabled() const
{
    return hasEntities(context(), 1);
}

void CmdMoveShape::execute()
{
    LcncDocument* doc = context()->activeDocument();
    if (!doc) return;
    auto entities = collectEntities(doc);
    if (entities.isEmpty()) {
        QMessageBox::information(nullptr, tr("移动"), tr("文档中没有工件"));
        return;
    }

    const auto sel = selectedEntities(context(), entities);

    QDialog dlg;
    dlg.setWindowTitle(tr("平移形体"));
    auto* form = new QFormLayout;

    auto* cb = new QComboBox;
    const bool hasSentinel = setupEntityCombo(cb, entities, sel);
    form->addRow(tr("形体:"), cb);

    auto* spX = makeSpin(0, -1e6, 1e6, 3, " mm");
    auto* spY = makeSpin(0, -1e6, 1e6, 3, " mm");
    auto* spZ = makeSpin(0, -1e6, 1e6, 3, " mm");
    form->addRow(tr("ΔX:"), spX);
    form->addRow(tr("ΔY:"), spY);
    form->addRow(tr("ΔZ:"), spZ);

    auto* btns = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(btns, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(btns, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
    auto* vl = new QVBoxLayout(&dlg);
    vl->addLayout(form);
    vl->addWidget(btns);

    if (dlg.exec() != QDialog::Accepted) return;

    gp_Trsf trsf;
    trsf.SetTranslation(gp_Vec(spX->value(), spY->value(), spZ->value()));

    // Resolve target entities from combobox selection.
    QList<EntityInfo> targets;
    if (hasSentinel && cb->currentIndex() == 0) {
        targets = sel;
    } else {
        const int idx = hasSentinel ? cb->currentIndex() - 1 : cb->currentIndex();
        if (idx >= 0 && idx < entities.size())
            targets = { entities[idx] };
    }

    for (const auto& ei : targets) {
        BRepBuilderAPI_Transform xform(ei.shape, trsf, Standard_True);
        if (xform.IsDone())
            doc->shapeTool()->SetShape(ei.label, xform.Shape());
    }
    refreshDocument(context());
}

// ── CmdRotateShape ────────────────────────────────────────────────────────────
CmdRotateShape::CmdRotateShape(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/rotate.svg"), tr("旋转"), this);
    a->setStatusTip(tr("绕轴旋转形体"));
    setAction(a);
}

bool CmdRotateShape::isEnabled() const
{
    return hasEntities(context(), 1);
}

void CmdRotateShape::execute()
{
    LcncDocument* doc = context()->activeDocument();
    if (!doc) return;
    auto entities = collectEntities(doc);
    if (entities.isEmpty()) {
        QMessageBox::information(nullptr, tr("旋转"), tr("文档中没有工件"));
        return;
    }

    const auto sel = selectedEntities(context(), entities);

    QDialog dlg;
    dlg.setWindowTitle(tr("旋转形体"));
    auto* form = new QFormLayout;

    auto* cb = new QComboBox;
    const bool hasSentinel = setupEntityCombo(cb, entities, sel);
    form->addRow(tr("形体:"), cb);

    auto* spAX = makeSpin(0, -1, 1, 4, "");
    auto* spAY = makeSpin(0, -1, 1, 4, "");
    auto* spAZ = makeSpin(1, -1, 1, 4, "");
    form->addRow(tr("轴向 X:"), spAX);
    form->addRow(tr("轴向 Y:"), spAY);
    form->addRow(tr("轴向 Z:"), spAZ);

    auto* spAngle = makeSpin(90, -360, 360, 2, " °");
    form->addRow(tr("旋转角度:"), spAngle);

    auto* btns = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(btns, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(btns, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
    auto* vl = new QVBoxLayout(&dlg);
    vl->addLayout(form);
    vl->addWidget(btns);

    if (dlg.exec() != QDialog::Accepted) return;

    const double ax = spAX->value(), ay = spAY->value(), az = spAZ->value();
    if (std::abs(ax) < 1e-9 && std::abs(ay) < 1e-9 && std::abs(az) < 1e-9) {
        QMessageBox::warning(nullptr, tr("旋转"), tr("轴向量不能为零向量"));
        return;
    }

    gp_Ax1 axis(gp_Pnt(0, 0, 0), gp_Dir(ax, ay, az));
    gp_Trsf trsf;
    trsf.SetRotation(axis, spAngle->value() * M_PI / 180.0);

    // Resolve target entities.
    QList<EntityInfo> targets;
    if (hasSentinel && cb->currentIndex() == 0) {
        targets = sel;
    } else {
        const int idx = hasSentinel ? cb->currentIndex() - 1 : cb->currentIndex();
        if (idx >= 0 && idx < entities.size())
            targets = { entities[idx] };
    }

    for (const auto& ei : targets) {
        BRepBuilderAPI_Transform xform(ei.shape, trsf, Standard_True);
        if (xform.IsDone())
            doc->shapeTool()->SetShape(ei.label, xform.Shape());
    }
    refreshDocument(context());
}

// =============================================================================
// Boolean operations
// =============================================================================

namespace {
/// Smart two-entity picker.
/// - sel.size() == 2: bypass dialog entirely, resolve idxA/idxB directly.
/// - sel.size() >  2: show dialog with A/B pre-set to sel[0]/sel[1].
/// - sel.size() <  2: show dialog with default indices 0/1.
/// Returns false if the user cancelled or inputs are invalid.
bool pickTwoEntities(const QString& title,
                     const QList<EntityInfo>& entities,
                     int& idxA, int& idxB,
                     const QList<EntityInfo>& sel = {})
{
    if (entities.size() < 2) {
        QMessageBox::information(nullptr, title, QObject::tr("需要至少两个工件"));
        return false;
    }

    // Exactly 2 selected — resolve immediately without dialog.
    if (sel.size() == 2) {
        idxA = entityIndex(entities, sel[0]);
        idxB = entityIndex(entities, sel[1]);
        if (idxA != idxB) return true;
        // Edge case: both map to same index (shouldn't happen), fall through.
    }

    QDialog dlg;
    dlg.setWindowTitle(title);
    auto* form = new QFormLayout;
    auto* cbA = new QComboBox;
    auto* cbB = new QComboBox;
    for (const auto& e : entities) { cbA->addItem(e.name); cbB->addItem(e.name); }

    // Pre-select from viewport selection when >2 shapes are selected.
    if (sel.size() >= 2) {
        cbA->setCurrentIndex(entityIndex(entities, sel[0]));
        cbB->setCurrentIndex(entityIndex(entities, sel[1]));
    } else {
        cbB->setCurrentIndex(1);
    }

    form->addRow(QObject::tr("形体 A:"), cbA);
    form->addRow(QObject::tr("形体 B:"), cbB);
    auto* btns = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    QObject::connect(btns, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    QObject::connect(btns, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
    auto* vl = new QVBoxLayout(&dlg);
    vl->addLayout(form);
    vl->addWidget(btns);
    if (dlg.exec() != QDialog::Accepted) return false;
    idxA = cbA->currentIndex();
    idxB = cbB->currentIndex();
    if (idxA == idxB) {
        QMessageBox::warning(nullptr, title, QObject::tr("请选择不同的两个形体"));
        return false;
    }
    return true;
}
} // namespace

// ── CmdBoolUnion ──────────────────────────────────────────────────────────────
CmdBoolUnion::CmdBoolUnion(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/bool_union.svg"), tr("布尔并"), this);
    a->setStatusTip(tr("布尔并运算 (A ∪ B)"));
    setAction(a);
}

bool CmdBoolUnion::isEnabled() const { return hasEntities(context(), 2); }

void CmdBoolUnion::execute()
{
    LcncDocument* doc = context()->activeDocument();
    if (!doc) return;
    auto entities = collectEntities(doc);
    const auto sel = selectedEntities(context(), entities);
    int idxA = 0, idxB = 1;
    if (!pickTwoEntities(tr("布尔并"), entities, idxA, idxB, sel)) return;

    BRepAlgoAPI_Fuse op(entities[idxA].shape, entities[idxB].shape);
    if (!op.IsDone()) {
        QMessageBox::critical(nullptr, tr("布尔并"), tr("布尔并运算失败"));
        return;
    }
    commitShape(context(), op.Shape(), tr("布尔并结果"));
}

// ── CmdBoolCut ────────────────────────────────────────────────────────────────
CmdBoolCut::CmdBoolCut(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/bool_cut.svg"), tr("布尔差"), this);
    a->setStatusTip(tr("布尔差运算 (A − B)"));
    setAction(a);
}

bool CmdBoolCut::isEnabled() const { return hasEntities(context(), 2); }

void CmdBoolCut::execute()
{
    LcncDocument* doc = context()->activeDocument();
    if (!doc) return;
    auto entities = collectEntities(doc);
    const auto sel = selectedEntities(context(), entities);
    int idxA = 0, idxB = 1;
    if (!pickTwoEntities(tr("布尔差 (A − B)"), entities, idxA, idxB, sel)) return;

    BRepAlgoAPI_Cut op(entities[idxA].shape, entities[idxB].shape);
    if (!op.IsDone()) {
        QMessageBox::critical(nullptr, tr("布尔差"), tr("布尔差运算失败"));
        return;
    }
    commitShape(context(), op.Shape(), tr("布尔差结果"));
}

// ── CmdBoolCommon ─────────────────────────────────────────────────────────────
CmdBoolCommon::CmdBoolCommon(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/bool_common.svg"), tr("布尔交"), this);
    a->setStatusTip(tr("布尔交运算 (A ∩ B)"));
    setAction(a);
}

bool CmdBoolCommon::isEnabled() const { return hasEntities(context(), 2); }

void CmdBoolCommon::execute()
{
    LcncDocument* doc = context()->activeDocument();
    if (!doc) return;
    auto entities = collectEntities(doc);
    const auto sel = selectedEntities(context(), entities);
    int idxA = 0, idxB = 1;
    if (!pickTwoEntities(tr("布尔交 (A ∩ B)"), entities, idxA, idxB, sel)) return;

    BRepAlgoAPI_Common op(entities[idxA].shape, entities[idxB].shape);
    if (!op.IsDone()) {
        QMessageBox::critical(nullptr, tr("布尔交"), tr("布尔交运算失败，可能形体不相交"));
        return;
    }
    commitShape(context(), op.Shape(), tr("布尔交结果"));
}

// =============================================================================
// Measurement commands
// =============================================================================

// ── CmdMeasureDistance ────────────────────────────────────────────────────────
CmdMeasureDistance::CmdMeasureDistance(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/measure_dist.svg"), tr("距离"), this);
    a->setStatusTip(tr("测量两形体间的最小距离"));
    setAction(a);
}

bool CmdMeasureDistance::isEnabled() const { return hasEntities(context(), 2); }

void CmdMeasureDistance::execute()
{
    LcncDocument* doc = context()->activeDocument();
    if (!doc) return;
    auto entities = collectEntities(doc);
    const auto sel = selectedEntities(context(), entities);
    int idxA = 0, idxB = 1;
    if (!pickTwoEntities(tr("距离测量"), entities, idxA, idxB, sel)) return;

    BRepExtrema_DistShapeShape dss(entities[idxA].shape, entities[idxB].shape);
    dss.Perform();
    if (!dss.IsDone()) {
        QMessageBox::critical(nullptr, tr("距离测量"), tr("距离计算失败"));
        return;
    }
    QMessageBox::information(nullptr, tr("距离测量"),
        tr("形体 \"%1\" 与 \"%2\" 之间的最小距离:\n\n%3 mm")
            .arg(entities[idxA].name)
            .arg(entities[idxB].name)
            .arg(dss.Value(), 0, 'f', 4));
}

// ── CmdMeasureAngle ───────────────────────────────────────────────────────────
CmdMeasureAngle::CmdMeasureAngle(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/measure_angle.svg"), tr("角度"), this);
    a->setStatusTip(tr("测量两形体第一个面的法向夹角"));
    setAction(a);
}

bool CmdMeasureAngle::isEnabled() const { return hasEntities(context(), 2); }

void CmdMeasureAngle::execute()
{
    LcncDocument* doc = context()->activeDocument();
    if (!doc) return;
    auto entities = collectEntities(doc);
    const auto sel = selectedEntities(context(), entities);
    int idxA = 0, idxB = 1;
    if (!pickTwoEntities(tr("角度测量"), entities, idxA, idxB, sel)) return;

    gp_Vec nA = firstFaceNormal(entities[idxA].shape);
    gp_Vec nB = firstFaceNormal(entities[idxB].shape);
    const double lenA = nA.Magnitude(), lenB = nB.Magnitude();
    if (lenA < 1e-9 || lenB < 1e-9) {
        QMessageBox::warning(nullptr, tr("角度测量"), tr("无法获取面法向量"));
        return;
    }
    double cosA = nA.Dot(nB) / (lenA * lenB);
    cosA = std::max(-1.0, std::min(1.0, cosA));
    const double angleDeg = std::acos(cosA) * 180.0 / M_PI;

    QMessageBox::information(nullptr, tr("角度测量"),
        tr("形体 \"%1\" 与 \"%2\" 首面法向夹角:\n\n%3°")
            .arg(entities[idxA].name)
            .arg(entities[idxB].name)
            .arg(angleDeg, 0, 'f', 2));
}

// ── CmdMeasureArea ────────────────────────────────────────────────────────────
CmdMeasureArea::CmdMeasureArea(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/measure_area.svg"), tr("面积"), this);
    a->setStatusTip(tr("计算形体的表面积"));
    setAction(a);
}

bool CmdMeasureArea::isEnabled() const { return hasEntities(context(), 1); }

void CmdMeasureArea::execute()
{
    LcncDocument* doc = context()->activeDocument();
    if (!doc) return;
    auto entities = collectEntities(doc);
    if (entities.isEmpty()) {
        QMessageBox::information(nullptr, tr("面积"), tr("文档中没有工件"));
        return;
    }

    const auto sel = selectedEntities(context(), entities);

    QList<EntityInfo> targets;
    if (!sel.isEmpty()) {
        // Use viewport selection directly — no dialog needed.
        targets = sel;
    } else {
        // Nothing selected: show single-entity picker.
        QDialog dlg;
        dlg.setWindowTitle(tr("面积测量"));
        auto* form = new QFormLayout;
        auto* cb   = new QComboBox;
        for (const auto& e : entities) cb->addItem(e.name);
        form->addRow(tr("形体:"), cb);
        auto* btns = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
        connect(btns, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
        connect(btns, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
        auto* vl = new QVBoxLayout(&dlg);
        vl->addLayout(form);
        vl->addWidget(btns);
        if (dlg.exec() != QDialog::Accepted) return;
        targets = { entities[cb->currentIndex()] };
    }

    if (targets.size() == 1) {
        GProp_GProps props;
        BRepGProp::SurfaceProperties(targets[0].shape, props);
        QMessageBox::information(nullptr, tr("面积测量"),
            tr("形体 \"%1\" 的总表面积:\n\n%2 mm²")
                .arg(targets[0].name)
                .arg(props.Mass(), 0, 'f', 3));
    } else {
        QString msg;
        for (const auto& ei : targets) {
            GProp_GProps props;
            BRepGProp::SurfaceProperties(ei.shape, props);
            msg += tr("%1: %2 mm²\n").arg(ei.name).arg(props.Mass(), 0, 'f', 3);
        }
        QMessageBox::information(nullptr,
            tr("面积测量 (已选中 %1 个)").arg(targets.size()),
            msg.trimmed());
    }
}
