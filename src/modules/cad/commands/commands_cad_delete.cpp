#include "modules/cad/commands/commands_cad.h"

#include "app/app_command_context.h"
#include "core/document/xcaf_utils.h"
#include "modules/cad/cad_module.h"
#include "modules/cad/commands/command_helpers.h"
#include "view/gui_document.h"

#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <TDataStd_Integer.hxx>
#include <TDF_LabelSequence.hxx>
#include <TopoDS_Iterator.hxx>
#include <XCAFDoc_ShapeTool.hxx>

#include <QAction>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QIcon>
#include <QKeySequence>
#include <QMessageBox>
#include <QVBoxLayout>

using namespace lcnc::cad::commands;

CmdDeleteShape::CmdDeleteShape(IAppContext* ctx) : CommandBase(ctx)
{
    // 中文翻译：删除
    auto* action = new QAction(QIcon("themeicons:delete.svg"), tr("Delete"), this);
    action->setShortcut(QKeySequence::Delete);
    // 中文翻译：删除选中的形体（同步移除三维视图和树节点）
    action->setStatusTip(tr("Delete the selected shape (remove the 3D view and tree nodes simultaneously)"));
    setAction(action);
}

bool CmdDeleteShape::isEnabled() const
{
    if (LcncDocument* doc = contextualDocument(context())) {
        Handle(XCAFDoc_ShapeTool) shapeTool = doc->shapeTool();
        TDF_LabelSequence freeShapes;
        shapeTool->GetFreeShapes(freeShapes);
        return freeShapes.Length() > 0;
    }
    return false;
}

void CmdDeleteShape::execute()
{
    LcncDocument* doc = contextualDocument(context());
    if (!doc)
        return;

    Handle(XCAFDoc_ShapeTool) shapeTool = doc->shapeTool();
    TDF_LabelSequence freeShapes;
    shapeTool->GetFreeShapes(freeShapes);

    QList<EntityInfo> all;
    for (int index = 1; index <= freeShapes.Length(); ++index) {
        TDF_Label label = freeShapes.Value(index);
        Handle(TDataStd_Integer) kindAttr;
        if (!label.FindAttribute(TDataStd_Integer::GetID(), kindAttr))
            continue;
        EntityInfo info;
        info.label = label;
        info.name = XcafUtils::name(label);
        if (info.name.isEmpty())
            info.name = XcafUtils::entry(label);
        info.shape = shapeTool->GetShape(label);
        all.append(info);
    }

    if (all.isEmpty()) {
        // 中文翻译：删除；文档中没有可删除的形体
        QMessageBox::information(nullptr, tr("Delete"), tr("There are no shapes in the document that can be deleted"));
        return;
    }

    const auto selected = selectedEntities(context(), all);

    QList<EntityInfo> targets;
    if (!selected.isEmpty()) {
        targets = selected;
    } else {
        QDialog dlg;
        // 中文翻译：删除形体
        dlg.setWindowTitle(tr("Delete shape"));
        auto* form = new QFormLayout;
        auto* combo = new QComboBox;
        for (const auto& entity : all)
            combo->addItem(entity.name);
        // 中文翻译：形体:
        form->addRow(tr("Shape:"), combo);
        auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
        connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
        connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
        auto* layout = new QVBoxLayout(&dlg);
        layout->addLayout(form);
        layout->addWidget(buttons);
        if (dlg.exec() != QDialog::Accepted)
            return;
        targets = {all[combo->currentIndex()]};
    }

    if (targets.size() > 1) {
        const int ret = QMessageBox::question(
            nullptr,
            // 中文翻译：删除
            tr("Delete"),
            // 中文翻译：将删除 %1 个形体，确认继续？
            tr("%1 shapes will be deleted, are you sure to continue?").arg(targets.size()),
            QMessageBox::Yes | QMessageBox::No);
        if (ret != QMessageBox::Yes)
            return;
    }

    QStringList entries;
    for (const auto& entity : targets)
        entries.append(XcafUtils::entry(entity.label));

    context()->cadModule()->deleteShapes(doc->id(), entries);
    context()->updateCommandStates();
}

CmdExplodeShape::CmdExplodeShape(IAppContext* ctx) : CommandBase(ctx)
{
    // 中文翻译：拆解
    auto* action = new QAction(QIcon("themeicons:explode.svg"), tr("Explode"), this);
    // 中文翻译：将选中的复合体拆解为下一层级子形体（一级拆解）
    action->setStatusTip(tr("Disassemble the selected complex into next-level sub-shapes (first-level disassembly)"));
    setAction(action);
}

bool CmdExplodeShape::isEnabled() const
{
    LcncDocument* activeDoc = context()->workpieceDocument();
    LcncDocument* machineDoc = context()->machineDocument();
    if (activeDoc && activeDoc->entityLabels(LcncDocument::EntityKind::Workpiece).Length() > 0)
        return true;
    if (machineDoc && machineDoc->entityLabels(LcncDocument::EntityKind::Machine).Length() > 0)
        return true;
    return false;
}

void CmdExplodeShape::execute()
{
    struct Hit {
        LcncDocument* doc{nullptr};
        LcncDocument::EntityKind kind{LcncDocument::EntityKind::Workpiece};
        TDF_Label label;
        TopoDS_Shape shape;
        QString name;
        QString entry;
    };

    auto tryFind = [](LcncDocument* doc, LcncDocument::EntityKind kind, GuiDocument* guiDocument) -> Hit {
        if (!doc || !guiDocument)
            return {};
        const Handle(AIS_InteractiveContext)& aisContext = guiDocument->context();
        if (aisContext.IsNull())
            return {};

        TDF_LabelSequence labels = doc->entityLabels(kind);
        Handle(XCAFDoc_ShapeTool) shapeTool = doc->shapeTool();
        for (int index = 1; index <= labels.Length(); ++index) {
            TDF_Label label = labels.Value(index);
            QString entry = XcafUtils::entry(label);
            Handle(AIS_Shape) ais = guiDocument->aisShape(doc->id(), entry);
            if (!ais.IsNull() && aisContext->IsSelected(ais)) {
                Hit hit;
                hit.doc = doc;
                hit.kind = kind;
                hit.label = label;
                hit.shape = shapeTool->GetShape(label);
                hit.name = XcafUtils::name(label);
                hit.entry = entry;
                return hit;
            }
        }
        return {};
    };

    Hit hit = tryFind(context()->machineDocument(),
                      LcncDocument::EntityKind::Machine,
                      context()->activeGuiDocument());
    if (!hit.doc) {
        hit = tryFind(context()->workpieceDocument(),
                      LcncDocument::EntityKind::Workpiece,
                      context()->activeGuiDocument());
    }

    if (!hit.doc) {
        LcncDocument* doc = context()->workpieceDocument();
        if (!doc) {
            doc = context()->machineDocument();
            if (!doc)
                return;
        }
        LcncDocument::EntityKind kind = doc == context()->machineDocument()
            ? LcncDocument::EntityKind::Machine
            : LcncDocument::EntityKind::Workpiece;
        TDF_LabelSequence labels = doc->entityLabels(kind);
        if (labels.IsEmpty())
            return;

        QDialog dlg;
        // 中文翻译：拆解形体
        dlg.setWindowTitle(tr("Explode Shape"));
        auto* form = new QFormLayout;
        auto* combo = new QComboBox;
        Handle(XCAFDoc_ShapeTool) shapeTool = doc->shapeTool();
        for (int index = 1; index <= labels.Length(); ++index)
            combo->addItem(XcafUtils::name(labels.Value(index)));
        // 中文翻译：选择形体:
        form->addRow(tr("Select shape:"), combo);
        auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
        connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
        connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
        auto* layout = new QVBoxLayout(&dlg);
        layout->addLayout(form);
        layout->addWidget(buttons);
        if (dlg.exec() != QDialog::Accepted)
            return;

        const int index = combo->currentIndex();
        hit.doc = doc;
        hit.kind = kind;
        hit.label = labels.Value(index + 1);
        hit.shape = shapeTool->GetShape(hit.label);
        hit.name = XcafUtils::name(hit.label);
        hit.entry = XcafUtils::entry(hit.label);
    }

    if (hit.shape.IsNull())
        return;

    int childCount = 0;
    for (TopoDS_Iterator iterator(hit.shape); iterator.More(); iterator.Next())
        ++childCount;

    if (childCount == 0) {
        // 中文翻译：拆解
        QMessageBox::information(nullptr, tr("Explode"),
            // 中文翻译：所选形体 "%1" 无法继续拆解（已是基本形体）。
            tr("The selected shape \"%1\" cannot be further disassembled (it is already a basic shape).").arg(hit.name));
        return;
    }

    // 中文翻译：拆解形体
    if (QMessageBox::question(nullptr, tr("Explode Shape"),
            // 中文翻译：将 "%1" 拆解为 %2 个子形体，原形体将被替换。\n继续？
            tr("Disassemble \"%1\" into %2 sub-shapes, and the original shape will be replaced.\ncontinue?")
                .arg(hit.name).arg(childCount),
            QMessageBox::Yes | QMessageBox::No) != QMessageBox::Yes) {
        return;
    }

    context()->cadModule()->explodeShape(hit.doc->id(), hit.label, static_cast<int>(hit.kind));
    context()->updateCommandStates();
}
