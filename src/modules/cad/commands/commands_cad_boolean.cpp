#include "modules/cad/commands/commands_cad.h"

#include "app/app_command_context.h"
#include "core/algorithms/cad/boolean_ops.h"
#include "modules/cad/commands/command_helpers.h"

#include <QAction>
#include <QIcon>
#include <QMessageBox>

using namespace lcnc::cad::commands;

CmdBoolUnion::CmdBoolUnion(IAppContext* ctx) : CommandBase(ctx)
{
    auto* action = new QAction(QIcon(":/icons/bool_union.svg"), tr("布尔并"), this);
    action->setStatusTip(tr("布尔并运算 (A ∪ B)"));
    setAction(action);
}

bool CmdBoolUnion::isEnabled() const
{
    return hasEntities(context(), 2);
}

void CmdBoolUnion::execute()
{
    LcncDocument* doc = context()->workpieceDocument();
    if (!doc)
        return;

    auto entities = collectEntities(doc);
    const auto selected = selectedEntities(context(), entities);
    int indexA = 0;
    int indexB = 1;
    if (!pickTwoEntities(tr("布尔并"), entities, indexA, indexB, selected))
        return;

    QString err;
    TopoDS_Shape shape = lcnc::cad_algo::fuseShapes(
        entities[indexA].shape, entities[indexB].shape, &err);
    if (shape.IsNull()) {
        QMessageBox::critical(nullptr, tr("布尔并"), err);
        return;
    }
    commitShape(context(), shape, tr("布尔并结果"));
}

CmdBoolCut::CmdBoolCut(IAppContext* ctx) : CommandBase(ctx)
{
    auto* action = new QAction(QIcon(":/icons/bool_cut.svg"), tr("布尔差"), this);
    action->setStatusTip(tr("布尔差运算 (A − B)"));
    setAction(action);
}

bool CmdBoolCut::isEnabled() const
{
    return hasEntities(context(), 2);
}

void CmdBoolCut::execute()
{
    LcncDocument* doc = context()->workpieceDocument();
    if (!doc)
        return;

    auto entities = collectEntities(doc);
    const auto selected = selectedEntities(context(), entities);
    int indexA = 0;
    int indexB = 1;
    if (!pickTwoEntities(tr("布尔差 (A − B)"), entities, indexA, indexB, selected))
        return;

    QString err;
    TopoDS_Shape shape = lcnc::cad_algo::cutShapes(
        entities[indexA].shape, entities[indexB].shape, &err);
    if (shape.IsNull()) {
        QMessageBox::critical(nullptr, tr("布尔差"), err);
        return;
    }
    commitShape(context(), shape, tr("布尔差结果"));
}

CmdBoolCommon::CmdBoolCommon(IAppContext* ctx) : CommandBase(ctx)
{
    auto* action = new QAction(QIcon(":/icons/bool_common.svg"), tr("布尔交"), this);
    action->setStatusTip(tr("布尔交运算 (A ∩ B)"));
    setAction(action);
}

bool CmdBoolCommon::isEnabled() const
{
    return hasEntities(context(), 2);
}

void CmdBoolCommon::execute()
{
    LcncDocument* doc = context()->workpieceDocument();
    if (!doc)
        return;

    auto entities = collectEntities(doc);
    const auto selected = selectedEntities(context(), entities);
    int indexA = 0;
    int indexB = 1;
    if (!pickTwoEntities(tr("布尔交 (A ∩ B)"), entities, indexA, indexB, selected))
        return;

    QString err;
    TopoDS_Shape shape = lcnc::cad_algo::commonShapes(
        entities[indexA].shape, entities[indexB].shape, &err);
    if (shape.IsNull()) {
        QMessageBox::critical(nullptr, tr("布尔交"), err);
        return;
    }
    commitShape(context(), shape, tr("布尔交结果"));
}
