#include "modules/cad/commands/commands_cad.h"

#include "core/algorithms/cad/measure.h"
#include "modules/cad/commands/command_helpers.h"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <gp_Vec.hxx>

#include <QAction>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QIcon>
#include <QMessageBox>
#include <QVBoxLayout>

using namespace lcnc::cad::commands;

CmdMeasureDistance::CmdMeasureDistance(IAppContext* ctx) : CommandBase(ctx)
{
    auto* action = new QAction(QIcon(":/icons/measure_dist.svg"), tr("距离"), this);
    action->setStatusTip(tr("测量两形体间的最小距离"));
    setAction(action);
}

bool CmdMeasureDistance::isEnabled() const
{
    return hasEntities(context(), 2);
}

void CmdMeasureDistance::execute()
{
    LcncDocument* doc = context()->activeDocument();
    if (!doc)
        return;

    auto entities = collectEntities(doc);
    const auto selected = selectedEntities(context(), entities);
    int indexA = 0;
    int indexB = 1;
    if (!pickTwoEntities(tr("距离测量"), entities, indexA, indexB, selected))
        return;

    const double distance = lcnc::cad_algo::minDistance(entities[indexA].shape, entities[indexB].shape);
    if (distance < 0.0) {
        QMessageBox::critical(nullptr, tr("距离测量"), tr("距离计算失败"));
        return;
    }
    QMessageBox::information(nullptr, tr("距离测量"),
        tr("形体 \"%1\" 与 \"%2\" 之间的最小距离:\n\n%3 mm")
            .arg(entities[indexA].name)
            .arg(entities[indexB].name)
            .arg(distance, 0, 'f', 4));
}

CmdMeasureAngle::CmdMeasureAngle(IAppContext* ctx) : CommandBase(ctx)
{
    auto* action = new QAction(QIcon(":/icons/measure_angle.svg"), tr("角度"), this);
    action->setStatusTip(tr("测量两形体第一个面的法向夹角"));
    setAction(action);
}

bool CmdMeasureAngle::isEnabled() const
{
    return hasEntities(context(), 2);
}

void CmdMeasureAngle::execute()
{
    LcncDocument* doc = context()->activeDocument();
    if (!doc)
        return;

    auto entities = collectEntities(doc);
    const auto selected = selectedEntities(context(), entities);
    int indexA = 0;
    int indexB = 1;
    if (!pickTwoEntities(tr("角度测量"), entities, indexA, indexB, selected))
        return;

    const gp_Vec normalA = lcnc::cad_algo::firstFaceNormal(entities[indexA].shape);
    const gp_Vec normalB = lcnc::cad_algo::firstFaceNormal(entities[indexB].shape);
    const double angleDeg = lcnc::cad_algo::angleBetween(normalA, normalB);
    if (angleDeg < 0.0) {
        QMessageBox::warning(nullptr, tr("角度测量"), tr("无法获取面法向量"));
        return;
    }

    QMessageBox::information(nullptr, tr("角度测量"),
        tr("形体 \"%1\" 与 \"%2\" 首面法向夹角:\n\n%3°")
            .arg(entities[indexA].name)
            .arg(entities[indexB].name)
            .arg(angleDeg, 0, 'f', 2));
}

CmdMeasureArea::CmdMeasureArea(IAppContext* ctx) : CommandBase(ctx)
{
    auto* action = new QAction(QIcon(":/icons/measure_area.svg"), tr("面积"), this);
    action->setStatusTip(tr("计算形体的表面积"));
    setAction(action);
}

bool CmdMeasureArea::isEnabled() const
{
    return hasEntities(context(), 1);
}

void CmdMeasureArea::execute()
{
    LcncDocument* doc = context()->activeDocument();
    if (!doc)
        return;

    auto entities = collectEntities(doc);
    if (entities.isEmpty()) {
        QMessageBox::information(nullptr, tr("面积"), tr("文档中没有工件"));
        return;
    }

    const auto selected = selectedEntities(context(), entities);

    QList<EntityInfo> targets;
    if (!selected.isEmpty()) {
        targets = selected;
    } else {
        QDialog dlg;
        dlg.setWindowTitle(tr("面积测量"));
        auto* form = new QFormLayout;
        auto* combo = new QComboBox;
        for (const auto& entity : entities)
            combo->addItem(entity.name);
        form->addRow(tr("形体:"), combo);
        auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
        connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
        connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
        auto* layout = new QVBoxLayout(&dlg);
        layout->addLayout(form);
        layout->addWidget(buttons);
        if (dlg.exec() != QDialog::Accepted)
            return;
        targets = {entities[combo->currentIndex()]};
    }

    if (targets.size() == 1) {
        GProp_GProps props;
        BRepGProp::SurfaceProperties(targets[0].shape, props);
        QMessageBox::information(nullptr, tr("面积测量"),
            tr("形体 \"%1\" 的总表面积:\n\n%2 mm²")
                .arg(targets[0].name)
                .arg(props.Mass(), 0, 'f', 3));
    } else {
        QString message;
        for (const auto& entity : targets) {
            GProp_GProps props;
            BRepGProp::SurfaceProperties(entity.shape, props);
            message += tr("%1: %2 mm²\n").arg(entity.name).arg(props.Mass(), 0, 'f', 3);
        }
        QMessageBox::information(nullptr,
            tr("面积测量 (已选中 %1 个)").arg(targets.size()),
            message.trimmed());
    }
}
