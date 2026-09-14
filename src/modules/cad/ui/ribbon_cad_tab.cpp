#include "modules/cad/ui/ribbon_cad_tab.h"

#include "core/command/commands_api.h"
#include "core/logging/logger.h"
#include "modules/cad/commands/commands_file.h"
#include "modules/cad/commands/commands_edit.h"
#include "modules/cad/commands/commands_cad.h"

#include <SARibbonCategory.h>
#include <SARibbonPanel.h>
#include <QAction>
#include <QIcon>
#include <QMenu>

#include <initializer_list>

namespace lcnc::cad {

namespace {

QMenu* makeCommandMenu(SARibbonCategory* parent,
                       const QString& title,
                       const QIcon& icon,
                       std::initializer_list<QAction*> actions)
{
    auto* menu = new QMenu(title, parent);
    menu->setIcon(icon);
    for (QAction* action : actions) {
        if (action)
            menu->addAction(action);
    }
    return menu;
}

} // namespace

void registerCommands(CommandContainer* container)
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "lcnc::cad::registerCommands begin");

    // File
    container->addCommand<CmdNewDocument>(CmdNewDocument::Name);
    container->addCommand<CmdOpenDocument>(CmdOpenDocument::Name);
    container->addCommand<CmdSaveDocument>(CmdSaveDocument::Name);
    container->addCommand<CmdSaveDocumentAs>(CmdSaveDocumentAs::Name);
    container->addCommand<CmdImportStep>(CmdImportStep::Name);
    container->addCommand<CmdImportStl>(CmdImportStl::Name);
    container->addCommand<CmdExportStep>(CmdExportStep::Name);
    container->addCommand<CmdCloseDocument>(CmdCloseDocument::Name);

    // Edit
    container->addCommand<CmdUndo>(CmdUndo::Name);
    container->addCommand<CmdRedo>(CmdRedo::Name);

    // CAD — Sketch
    container->addCommand<CmdNewSketch>(CmdNewSketch::Name);
    container->addCommand<CmdFinishSketch>(CmdFinishSketch::Name);
    container->addCommand<CmdCancelSketch>(CmdCancelSketch::Name);
    container->addCommand<CmdSketchPoint>(CmdSketchPoint::Name);
    container->addCommand<CmdSketchLine>(CmdSketchLine::Name);
    container->addCommand<CmdSketchArc>(CmdSketchArc::Name);
    container->addCommand<CmdSketchCircleTool>(CmdSketchCircleTool::Name);
    container->addCommand<CmdSketchRectangleTool>(CmdSketchRectangleTool::Name);
    container->addCommand<CmdSketchPolygon>(CmdSketchPolygon::Name);

    // CAD — View modeling aids
    container->addCommand<CmdToggleCadGrid>(CmdToggleCadGrid::Name);
    container->addCommand<CmdToggleGridSnap>(CmdToggleGridSnap::Name);
    container->addCommand<CmdSnapNone>(CmdSnapNone::Name);
    container->addCommand<CmdSnapVertex>(CmdSnapVertex::Name);
    container->addCommand<CmdSnapEdge>(CmdSnapEdge::Name);
    container->addCommand<CmdSnapFace>(CmdSnapFace::Name);

    // CAD — Primitives
    container->addCommand<CmdCreateBox>(CmdCreateBox::Name);
    container->addCommand<CmdCreateCylinder>(CmdCreateCylinder::Name);
    container->addCommand<CmdCreateSphere>(CmdCreateSphere::Name);
    container->addCommand<CmdCreateCone>(CmdCreateCone::Name);
    container->addCommand<CmdCreateTorus>(CmdCreateTorus::Name);

    // CAD — Transforms
    container->addCommand<CmdMoveShape>(CmdMoveShape::Name);
    container->addCommand<CmdRotateShape>(CmdRotateShape::Name);

    // CAD — Boolean
    container->addCommand<CmdBoolUnion>(CmdBoolUnion::Name);
    container->addCommand<CmdBoolCut>(CmdBoolCut::Name);
    container->addCommand<CmdBoolCommon>(CmdBoolCommon::Name);

    // CAD — Measurement
    container->addCommand<CmdMeasureDistance>(CmdMeasureDistance::Name);
    container->addCommand<CmdMeasureAngle>(CmdMeasureAngle::Name);
    container->addCommand<CmdMeasureArea>(CmdMeasureArea::Name);

    // CAD — Delete
    container->addCommand<CmdDeleteShape>(CmdDeleteShape::Name);
    container->addCommand<CmdExplodeShape>(CmdExplodeShape::Name);

    LCNC_DEBUG(lcnc::LogCode::Generic, "lcnc::cad::registerCommands end");
}

void buildRibbonTab(SARibbonCategory* cat,
                    CommandContainer* container,
                    QObject* /*parent*/)
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "lcnc::cad::buildRibbonTab begin");

    // ── 建模 ─────────────────────────────────────────────────────────────
    // 中文翻译：建模
    SARibbonPanel* panelModel = cat->addPanel(QObject::tr("Modeling"));
    auto* menuPrimitive = makeCommandMenu(
        cat,
        // 中文翻译：基本体
        QObject::tr("Basic body"),
        QIcon("themeicons:box.svg"),
        {container->findAction(CmdCreateBox::Name),
         container->findAction(CmdCreateCylinder::Name),
         container->findAction(CmdCreateSphere::Name),
         container->findAction(CmdCreateCone::Name),
         container->findAction(CmdCreateTorus::Name)});
    panelModel->addLargeMenu(menuPrimitive);

    auto* menuSketch = makeCommandMenu(
        cat,
        // 中文翻译：草图
        QObject::tr("Sketch"),
        QIcon("themeicons:sketch.svg"),
        {container->findAction(CmdNewSketch::Name),
         container->findAction(CmdFinishSketch::Name),
         container->findAction(CmdCancelSketch::Name)});
    menuSketch->addSeparator();
    if (auto* a = container->findAction(CmdSketchPoint::Name)) menuSketch->addAction(a);
    if (auto* a = container->findAction(CmdSketchLine::Name)) menuSketch->addAction(a);
    if (auto* a = container->findAction(CmdSketchArc::Name)) menuSketch->addAction(a);
    if (auto* a = container->findAction(CmdSketchCircleTool::Name)) menuSketch->addAction(a);
    if (auto* a = container->findAction(CmdSketchRectangleTool::Name)) menuSketch->addAction(a);
    if (auto* a = container->findAction(CmdSketchPolygon::Name)) menuSketch->addAction(a);
    panelModel->addLargeMenu(menuSketch);

    // ── 操作 ───────────────────────────────────────────────────────────────
    // 中文翻译：操作
    SARibbonPanel* panelOps = cat->addPanel(QObject::tr("Operation"));
    auto* menuHistory = makeCommandMenu(
        cat,
        // 中文翻译：历史
        QObject::tr("history"),
        QIcon("themeicons:undo.svg"),
        {container->findAction(CmdUndo::Name),
         container->findAction(CmdRedo::Name)});
    panelOps->addLargeMenu(menuHistory);

    auto* menuTransform = makeCommandMenu(
        cat,
        // 中文翻译：变换
        QObject::tr("transform"),
        QIcon("themeicons:move.svg"),
        {container->findAction(CmdMoveShape::Name),
         container->findAction(CmdRotateShape::Name)});
    panelOps->addLargeMenu(menuTransform);

    auto* menuBoolean = makeCommandMenu(
        cat,
        // 中文翻译：布尔
        QObject::tr("Boolean"),
        QIcon("themeicons:bool_union.svg"),
        {container->findAction(CmdBoolUnion::Name),
         container->findAction(CmdBoolCut::Name),
         container->findAction(CmdBoolCommon::Name)});
    panelOps->addLargeMenu(menuBoolean);

    auto* menuEdit = makeCommandMenu(
        cat,
        // 中文翻译：编辑
        QObject::tr("Edit"),
        QIcon("themeicons:close.svg"),
        {container->findAction(CmdDeleteShape::Name),
         container->findAction(CmdExplodeShape::Name)});
    panelOps->addLargeMenu(menuEdit);

    // ── 测量 ───────────────────────────────────────────────────────────────
    // 中文翻译：测量
    SARibbonPanel* panelMeas = cat->addPanel(QObject::tr("Measure"));
    auto* menuMeasure = makeCommandMenu(
        cat,
        // 中文翻译：测量
        QObject::tr("Measure"),
        QIcon("themeicons:measure_dist.svg"),
        {container->findAction(CmdMeasureDistance::Name),
         container->findAction(CmdMeasureAngle::Name),
         container->findAction(CmdMeasureArea::Name)});
    panelMeas->addLargeMenu(menuMeasure);

    // ── 辅助 ───────────────────────────────────────────────────────────────
    // 中文翻译：辅助
    SARibbonPanel* panelAssist = cat->addPanel(QObject::tr("Auxiliary"));
    auto* menuGrid = makeCommandMenu(
        cat,
        // 中文翻译：网格
        QObject::tr("grid"),
        QIcon("themeicons:grid.svg"),
        {container->findAction(CmdToggleCadGrid::Name),
         container->findAction(CmdToggleGridSnap::Name)});
    panelAssist->addLargeMenu(menuGrid);

    LCNC_DEBUG(lcnc::LogCode::Generic, "lcnc::cad::buildRibbonTab end");
}

} // namespace lcnc::cad
