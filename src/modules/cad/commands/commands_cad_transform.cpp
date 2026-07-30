#include "modules/cad/commands/commands_cad.h"

#include "app/app_command_context.h"
#include "core/logging/logger.h"
#include "modules/cad/cad_module.h"
#include "modules/cad/commands/command_helpers.h"

#include <QAction>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QIcon>
#include <QMessageBox>
#include <QVBoxLayout>

#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Vec.hxx>

#include <cmath>

using namespace lcnc::cad::commands;

CmdMoveShape::CmdMoveShape(IAppContext* ctx) : CommandBase(ctx)
{
    // 中文翻译：移动
    auto* action = new QAction(QIcon(":/icons/move.svg"), tr("move"), this);
    // 中文翻译：沿 X/Y/Z 方向平移形体
    action->setStatusTip(tr("Translate the shape along the X/Y/Z direction"));
    setAction(action);
}

bool CmdMoveShape::isEnabled() const
{
    return hasEntities(context(), 1);
}

void CmdMoveShape::execute()
{
    LcncDocument* doc = contextualDocument(context());
    if (!doc)
        return;

    auto entities = collectContextualEntities(context());
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "CmdMoveShape::execute entities={} machineView={}",
               entities.size(), context()->isMachineViewActive());
    if (entities.isEmpty()) {
        // 中文翻译：移动；当前视图无可移动的形体
        QMessageBox::information(nullptr, tr("move"), tr("There are no movable shapes in the current view"));
        return;
    }

    const auto selected = selectedEntities(context(), entities);

    QDialog dlg;
    // 中文翻译：平移形体
    dlg.setWindowTitle(tr("Translate the shape"));
    auto* form = new QFormLayout;

    auto* combo = new QComboBox;
    const bool hasSentinel = setupEntityCombo(combo, entities, selected);
    // 中文翻译：形体:
    form->addRow(tr("Shape:"), combo);

    auto* spinX = makeSpin(0, -1e6, 1e6, 3, " mm");
    auto* spinY = makeSpin(0, -1e6, 1e6, 3, " mm");
    auto* spinZ = makeSpin(0, -1e6, 1e6, 3, " mm");
    form->addRow(tr("ΔX:"), spinX);
    form->addRow(tr("ΔY:"), spinY);
    form->addRow(tr("ΔZ:"), spinZ);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
    auto* layout = new QVBoxLayout(&dlg);
    layout->addLayout(form);
    layout->addWidget(buttons);

    if (dlg.exec() != QDialog::Accepted)
        return;

    QList<EntityInfo> targets;
    if (hasSentinel && combo->currentIndex() == 0) {
        targets = selected;
    } else {
        const int index = hasSentinel ? combo->currentIndex() - 1 : combo->currentIndex();
        if (index >= 0 && index < entities.size())
            targets = {entities[index]};
    }

    QList<TDF_Label> labels;
    for (const auto& entity : targets)
        labels.append(entity.label);

    context()->cadModule()->moveShapes(doc->id(), labels,
        gp_Vec(spinX->value(), spinY->value(), spinZ->value()));
    context()->updateCommandStates();
}

CmdRotateShape::CmdRotateShape(IAppContext* ctx) : CommandBase(ctx)
{
    // 中文翻译：旋转
    auto* action = new QAction(QIcon(":/icons/rotate.svg"), tr("rotate"), this);
    // 中文翻译：绕轴旋转形体
    action->setStatusTip(tr("Rotate a shape around an axis"));
    setAction(action);
}

bool CmdRotateShape::isEnabled() const
{
    return hasEntities(context(), 1);
}

void CmdRotateShape::execute()
{
    LcncDocument* doc = contextualDocument(context());
    if (!doc)
        return;

    auto entities = collectContextualEntities(context());
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "CmdRotateShape::execute entities={} machineView={}",
               entities.size(), context()->isMachineViewActive());
    if (entities.isEmpty()) {
        // 中文翻译：旋转；当前视图无可旋转的形体
        QMessageBox::information(nullptr, tr("rotate"), tr("There is no rotatable shape in the current view"));
        return;
    }

    const auto selected = selectedEntities(context(), entities);

    QDialog dlg;
    // 中文翻译：旋转形体
    dlg.setWindowTitle(tr("rotating shape"));
    auto* form = new QFormLayout;

    auto* combo = new QComboBox;
    const bool hasSentinel = setupEntityCombo(combo, entities, selected);
    // 中文翻译：形体:
    form->addRow(tr("Shape:"), combo);

    auto* spinAxisX = makeSpin(0, -1, 1, 4, "");
    auto* spinAxisY = makeSpin(0, -1, 1, 4, "");
    auto* spinAxisZ = makeSpin(1, -1, 1, 4, "");
    // 中文翻译：轴向 X:
    form->addRow(tr("Axial X:"), spinAxisX);
    // 中文翻译：轴向 Y:
    form->addRow(tr("Axial Y:"), spinAxisY);
    // 中文翻译：轴向 Z:
    form->addRow(tr("Axial Z:"), spinAxisZ);

    auto* spinAngle = makeSpin(90, -360, 360, 2, " °");
    // 中文翻译：旋转角度:
    form->addRow(tr("Rotation angle:"), spinAngle);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
    auto* layout = new QVBoxLayout(&dlg);
    layout->addLayout(form);
    layout->addWidget(buttons);

    if (dlg.exec() != QDialog::Accepted)
        return;

    const double axisX = spinAxisX->value();
    const double axisY = spinAxisY->value();
    const double axisZ = spinAxisZ->value();
    if (std::abs(axisX) < 1e-9 && std::abs(axisY) < 1e-9 && std::abs(axisZ) < 1e-9) {
        // 中文翻译：旋转；轴向量不能为零向量
        QMessageBox::warning(nullptr, tr("rotate"), tr("The axis vector cannot be a zero vector"));
        return;
    }

    gp_Ax1 axis(gp_Pnt(0, 0, 0), gp_Dir(axisX, axisY, axisZ));

    QList<EntityInfo> targets;
    if (hasSentinel && combo->currentIndex() == 0) {
        targets = selected;
    } else {
        const int index = hasSentinel ? combo->currentIndex() - 1 : combo->currentIndex();
        if (index >= 0 && index < entities.size())
            targets = {entities[index]};
    }

    QList<TDF_Label> labels;
    for (const auto& entity : targets)
        labels.append(entity.label);

    context()->cadModule()->rotateShapes(doc->id(), labels, axis, spinAngle->value());
    context()->updateCommandStates();
}
