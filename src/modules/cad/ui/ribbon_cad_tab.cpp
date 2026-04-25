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

namespace lcnc::cad {

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
                    QObject* parent)
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "lcnc::cad::buildRibbonTab begin");

    auto makeAct = [parent](const QString& label, const QString& iconPath) -> QAction* {
        auto* a = new QAction(QIcon(iconPath), label, parent);
        a->setStatusTip(QObject::tr("创建 ") + label);
        return a;
    };

    // ── 历史 ─────────────────────────────────────────────────────────────
    SARibbonPanel* panelHist = cat->addPanel(QObject::tr("历史"));
    panelHist->addLargeAction(container->findAction(CmdUndo::Name));
    panelHist->addLargeAction(container->findAction(CmdRedo::Name));

    // ── 基本体 ─────────────────────────────────────────────────────────────
    SARibbonPanel* panelPrim = cat->addPanel(QObject::tr("基本体"));
    panelPrim->addLargeAction(container->findAction(CmdCreateBox::Name));
    panelPrim->addLargeAction(container->findAction(CmdCreateCylinder::Name));
    panelPrim->addLargeAction(container->findAction(CmdCreateSphere::Name));
    panelPrim->addSmallAction(container->findAction(CmdCreateCone::Name));
    panelPrim->addSmallAction(container->findAction(CmdCreateTorus::Name));

    // ── 操作 ───────────────────────────────────────────────────────────────
    SARibbonPanel* panelOps = cat->addPanel(QObject::tr("操作"));
    panelOps->addLargeAction(container->findAction(CmdMoveShape::Name));
    panelOps->addLargeAction(container->findAction(CmdRotateShape::Name));
    panelOps->addSmallAction(makeAct(QObject::tr("缩放"), QStringLiteral(":/icons/scale.svg")));
    panelOps->addSmallAction(container->findAction(CmdBoolUnion::Name));
    panelOps->addSmallAction(container->findAction(CmdBoolCut::Name));
    panelOps->addSmallAction(container->findAction(CmdBoolCommon::Name));
    panelOps->addSmallAction(container->findAction(CmdDeleteShape::Name));
    panelOps->addSmallAction(container->findAction(CmdExplodeShape::Name));

    // ── 测量 ───────────────────────────────────────────────────────────────
    SARibbonPanel* panelMeas = cat->addPanel(QObject::tr("测量"));
    panelMeas->addLargeAction(container->findAction(CmdMeasureDistance::Name));
    panelMeas->addSmallAction(container->findAction(CmdMeasureAngle::Name));
    panelMeas->addSmallAction(container->findAction(CmdMeasureArea::Name));

    // ── 草图 (预留) ────────────────────────────────────────────────────────
    SARibbonPanel* panelSketch = cat->addPanel(QObject::tr("草图"));
    panelSketch->addLargeAction(makeAct(QObject::tr("新建草图"), QStringLiteral(":/icons/sketch.svg")));
    panelSketch->addSmallAction(makeAct(QObject::tr("直线"),     QStringLiteral(":/icons/line.svg")));
    panelSketch->addSmallAction(makeAct(QObject::tr("圆"),       QStringLiteral(":/icons/circle.svg")));
    panelSketch->addSmallAction(makeAct(QObject::tr("圆弧"),     QStringLiteral(":/icons/arc.svg")));
    panelSketch->addSmallAction(makeAct(QObject::tr("退出草图"), QStringLiteral(":/icons/exit_sketch.svg")));

    LCNC_DEBUG(lcnc::LogCode::Generic, "lcnc::cad::buildRibbonTab end");
}

} // namespace lcnc::cad
