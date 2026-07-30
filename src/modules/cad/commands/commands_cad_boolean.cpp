#include "modules/cad/commands/commands_cad.h"

#include "app/app_command_context.h"
#include "core/algorithms/cad/boolean_ops.h"
#include "modules/cad/commands/command_helpers.h"
#include "modules/cad/services/cad_algorithm_boundary.h"

#include <QAction>
#include <QIcon>
#include <QMessageBox>

using namespace lcnc::cad::commands;

CmdBoolUnion::CmdBoolUnion(IAppContext* ctx) : CommandBase(ctx)
{
    // 中文翻译：布尔并
    auto* action = new QAction(QIcon("themeicons:bool_union.svg"), tr("Boolean"), this);
    // 中文翻译：布尔并运算 (A ∪ B)
    action->setStatusTip(tr("Boolean union operation (A ∪ B)"));
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
    // 中文翻译：布尔并
    if (!pickTwoEntities(tr("Boolean"), entities, indexA, indexB, selected))
        return;

    QString err;
    TopoDS_Shape shape = lcnc::cad::invokeCadAlgorithm(
        [&] {
            return lcnc::cad_algo::fuseShapes(
                entities[indexA].shape, entities[indexB].shape);
        },
        &err);
    if (shape.IsNull()) {
        // 中文翻译：布尔并
        QMessageBox::critical(nullptr, tr("Boolean"), err);
        return;
    }
    // 中文翻译：布尔并结果
    commitShape(context(), shape, tr("boolean union result"));
}

CmdBoolCut::CmdBoolCut(IAppContext* ctx) : CommandBase(ctx)
{
    // 中文翻译：布尔差
    auto* action = new QAction(QIcon("themeicons:bool_cut.svg"), tr("Boolean difference"), this);
    // 中文翻译：布尔差运算 (A − B)
    action->setStatusTip(tr("Boolean difference operation (A − B)"));
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
    // 中文翻译：布尔差 (A − B)
    if (!pickTwoEntities(tr("Boolean difference (A − B)"), entities, indexA, indexB, selected))
        return;

    QString err;
    TopoDS_Shape shape = lcnc::cad::invokeCadAlgorithm(
        [&] {
            return lcnc::cad_algo::cutShapes(
                entities[indexA].shape, entities[indexB].shape);
        },
        &err);
    if (shape.IsNull()) {
        // 中文翻译：布尔差
        QMessageBox::critical(nullptr, tr("Boolean difference"), err);
        return;
    }
    // 中文翻译：布尔差结果
    commitShape(context(), shape, tr("Boolean difference result"));
}

CmdBoolCommon::CmdBoolCommon(IAppContext* ctx) : CommandBase(ctx)
{
    // 中文翻译：布尔交
    auto* action = new QAction(QIcon("themeicons:bool_common.svg"), tr("Bourgeois"), this);
    // 中文翻译：布尔交运算 (A ∩ B)
    action->setStatusTip(tr("Boolean intersection operation (A ∩ B)"));
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
    // 中文翻译：布尔交 (A ∩ B)
    if (!pickTwoEntities(tr("Boolean intersection (A ∩ B)"), entities, indexA, indexB, selected))
        return;

    QString err;
    TopoDS_Shape shape = lcnc::cad::invokeCadAlgorithm(
        [&] {
            return lcnc::cad_algo::commonShapes(
                entities[indexA].shape, entities[indexB].shape);
        },
        &err);
    if (shape.IsNull()) {
        // 中文翻译：布尔交
        QMessageBox::critical(nullptr, tr("Bourgeois"), err);
        return;
    }
    // 中文翻译：布尔交结果
    commitShape(context(), shape, tr("Boolean intersection result"));
}
