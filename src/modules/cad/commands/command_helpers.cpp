#include "modules/cad/commands/command_helpers.h"

#include "app/app_command_context.h"
#include "core/document/xcaf_utils.h"
#include "modules/cad/cad_module.h"
#include "view/gui_document.h"

#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <XCAFDoc_ShapeTool.hxx>

#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QMessageBox>
#include <QObject>
#include <QVBoxLayout>

namespace lcnc::cad::commands {

LcncDocument* contextualDocument(IAppContext* ctx)
{
    if (!ctx)
        return nullptr;
    if (ctx->isMachineViewActive())
        return ctx->machineDocument();
    return ctx->workpieceDocument();
}

GuiDocument* contextualGuiDocument(IAppContext* ctx)
{
    if (!ctx)
        return nullptr;
    return ctx->activeGuiDocument();
}

QList<EntityInfo> collectEntities(LcncDocument* doc, LcncDocument::EntityKind kind)
{
    QList<EntityInfo> out;
    if (!doc)
        return out;

    TDF_LabelSequence labels = doc->entityLabels(kind);
    Handle(XCAFDoc_ShapeTool) shapeTool = doc->shapeTool();
    for (int index = 1; index <= labels.Length(); ++index) {
        TDF_Label label = labels.Value(index);
        EntityInfo info;
        info.label = label;
        info.name = XcafUtils::name(label);
        if (info.name.isEmpty())
            info.name = QStringLiteral("形体 %1").arg(index);
        info.shape = shapeTool->GetShape(label);
        out.append(info);
    }
    return out;
}

QList<EntityInfo> collectContextualEntities(IAppContext* ctx)
{
    LcncDocument* doc = contextualDocument(ctx);
    if (!doc)
        return {};
    if (!ctx->isMachineViewActive())
        return collectEntities(doc, LcncDocument::EntityKind::Workpiece);

    QList<EntityInfo> machine = collectEntities(doc, LcncDocument::EntityKind::Machine);
    QList<EntityInfo> workpiece = collectEntities(doc, LcncDocument::EntityKind::Workpiece);
    for (auto& entity : machine)
        entity.name = QObject::tr("[机台] %1").arg(entity.name);
    for (auto& entity : workpiece)
        entity.name = QObject::tr("[工件] %1").arg(entity.name);

    QList<EntityInfo> all;
    all.reserve(workpiece.size() + machine.size());
    all.append(workpiece);
    all.append(machine);
    return all;
}

bool hasEntities(IAppContext* ctx, int minCount)
{
    LcncDocument* doc = contextualDocument(ctx);
    if (!doc)
        return false;
    if (ctx->isMachineViewActive()) {
        const int total = doc->entityLabels(LcncDocument::EntityKind::Workpiece).Length()
                        + doc->entityLabels(LcncDocument::EntityKind::Machine).Length();
        return total >= minCount;
    }
    return doc->entityLabels(LcncDocument::EntityKind::Workpiece).Length() >= minCount;
}

void commitShape(IAppContext* ctx, const TopoDS_Shape& shape, const QString& name)
{
    if (!ctx || !ctx->cadModule())
        return;
    DocumentId docId = ctx->workpieceDocumentId();
    if (docId == kInvalidDocumentId)
        docId = ctx->cadModule()->newDocument(name);
    ctx->cadModule()->createShape(
        docId,
        shape,
        name,
        static_cast<int>(LcncDocument::EntityKind::Workpiece));
    ctx->updateCommandStates();
}

QDoubleSpinBox* makeSpin(double val, double lo, double hi, int decimals, const QString& suffix)
{
    auto* spin = new QDoubleSpinBox;
    spin->setRange(lo, hi);
    spin->setValue(val);
    spin->setDecimals(decimals);
    spin->setSuffix(suffix);
    return spin;
}

QList<EntityInfo> selectedEntities(IAppContext* ctx, const QList<EntityInfo>& all)
{
    GuiDocument* guiDocument = contextualGuiDocument(ctx);
    if (!guiDocument)
        return {};

    const Handle(AIS_InteractiveContext)& aisContext = guiDocument->context();
    if (aisContext.IsNull())
        return {};

    LcncDocument* doc = contextualDocument(ctx);
    const DocumentId documentId = doc ? doc->id() : kInvalidDocumentId;
    QList<EntityInfo> result;
    for (const EntityInfo& entity : all) {
        Handle(AIS_Shape) ais = guiDocument->aisShape(documentId, XcafUtils::entry(entity.label));
        if (!ais.IsNull() && aisContext->IsSelected(ais))
            result.append(entity);
    }
    return result;
}

int entityIndex(const QList<EntityInfo>& all, const EntityInfo& target)
{
    const QString entry = XcafUtils::entry(target.label);
    for (int index = 0; index < all.size(); ++index) {
        if (XcafUtils::entry(all[index].label) == entry)
            return index;
    }
    return 0;
}

bool setupEntityCombo(QComboBox* cb,
                      const QList<EntityInfo>& all,
                      const QList<EntityInfo>& sel)
{
    cb->clear();
    const bool hasSentinel = !sel.isEmpty();
    if (hasSentinel) {
        if (sel.size() == 1)
            cb->addItem(QObject::tr("[当前选中] %1").arg(sel.front().name));
        else
            cb->addItem(QObject::tr("[当前选中] %1 个形体").arg(sel.size()));
    }
    for (const auto& entity : all)
        cb->addItem(entity.name);
    cb->setCurrentIndex(0);
    return hasSentinel;
}

bool pickTwoEntities(const QString& title,
                     const QList<EntityInfo>& entities,
                     int& idxA,
                     int& idxB,
                     const QList<EntityInfo>& sel)
{
    if (entities.size() < 2) {
        QMessageBox::information(nullptr, title, QObject::tr("需要至少两个工件"));
        return false;
    }

    if (sel.size() == 2) {
        idxA = entityIndex(entities, sel[0]);
        idxB = entityIndex(entities, sel[1]);
        if (idxA != idxB)
            return true;
    }

    QDialog dlg;
    dlg.setWindowTitle(title);
    auto* form = new QFormLayout;
    auto* cbA = new QComboBox;
    auto* cbB = new QComboBox;
    for (const auto& entity : entities) {
        cbA->addItem(entity.name);
        cbB->addItem(entity.name);
    }

    if (sel.size() >= 2) {
        cbA->setCurrentIndex(entityIndex(entities, sel[0]));
        cbB->setCurrentIndex(entityIndex(entities, sel[1]));
    } else {
        cbB->setCurrentIndex(1);
    }

    form->addRow(QObject::tr("形体 A:"), cbA);
    form->addRow(QObject::tr("形体 B:"), cbB);
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    QObject::connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    QObject::connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
    auto* layout = new QVBoxLayout(&dlg);
    layout->addLayout(form);
    layout->addWidget(buttons);
    if (dlg.exec() != QDialog::Accepted)
        return false;

    idxA = cbA->currentIndex();
    idxB = cbB->currentIndex();
    if (idxA == idxB) {
        QMessageBox::warning(nullptr, title, QObject::tr("请选择不同的两个形体"));
        return false;
    }
    return true;
}

} // namespace lcnc::cad::commands
