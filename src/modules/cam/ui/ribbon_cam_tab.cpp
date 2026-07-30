#include "modules/cam/ui/ribbon_cam_tab.h"

#include "core/command/commands_api.h"
#include "core/logging/logger.h"
#include "modules/cam/commands/commands_machine.h"
#include "modules/cam/commands/commands_cam.h"

#include <SARibbonCategory.h>
#include <SARibbonPanel.h>
#include <QAction>
#include <QIcon>
#include <QWidget>

namespace lcnc::cam {

void registerCommands(CommandContainer* container)
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "lcnc::cam::registerCommands begin");

    // Machine
    container->addCommand<CmdLoadMachine>(CmdLoadMachine::Name);
    container->addCommand<CmdUnloadMachine>(CmdUnloadMachine::Name);
    container->addCommand<CmdExportMachine>(CmdExportMachine::Name);

    // CAM
    container->addCommand<CmdGenerateToolpath>(CmdGenerateToolpath::Name);
    container->addCommand<CmdSetLeadIn>(CmdSetLeadIn::Name);
    container->addCommand<CmdToolpathPreview>(CmdToolpathPreview::Name);
    container->addCommand<CmdRecalcToolpath>(CmdRecalcToolpath::Name);
    container->addCommand<CmdSelectMachiningFace>(CmdSelectMachiningFace::Name);
    container->addCommand<CmdClearMachiningFaces>(CmdClearMachiningFaces::Name);

    LCNC_DEBUG(lcnc::LogCode::Generic, "lcnc::cam::registerCommands end");
}

void buildRibbonTab(SARibbonCategory* cat,
                    CommandContainer* container,
                    QObject* parent)
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "lcnc::cam::buildRibbonTab begin");

    auto makeAct = [parent](const QString& label, const QString& iconPath) -> QAction* {
        return new QAction(QIcon(iconPath), label, parent);
    };

    // ── 机台 ───────────────────────────────────────────────────────────────
    // 中文翻译：机台
    SARibbonPanel* panelMach = cat->addPanel(QObject::tr("machine"));
    panelMach->addLargeAction(container->findAction(CmdLoadMachine::Name));
    panelMach->addLargeAction(container->findAction(CmdUnloadMachine::Name));
    panelMach->addLargeAction(container->findAction(CmdExportMachine::Name));

    // ── 刀路 ───────────────────────────────────────────────────────────────
    // 中文翻译：刀路
    SARibbonPanel* panelPath = cat->addPanel(QObject::tr("knife path"));
    panelPath->addLargeAction(container->findAction(CmdGenerateToolpath::Name));
    panelPath->addLargeAction(container->findAction(CmdSetLeadIn::Name));
    panelPath->addLargeAction(container->findAction(CmdRecalcToolpath::Name));
    panelPath->addLargeAction(container->findAction(CmdToolpathPreview::Name));

    // ── 加工面 ─────────────────────────────────────────────────────────────
    // 中文翻译：加工面
    SARibbonPanel* panelFace = cat->addPanel(QObject::tr("Processing surface"));
    panelFace->addLargeAction(container->findAction(CmdSelectMachiningFace::Name));
    panelFace->addLargeAction(container->findAction(CmdClearMachiningFaces::Name));

    // ── G代码（占位） ─────────────────────────────────────────────────────
    // 中文翻译：G代码
    SARibbonPanel* panelNC = cat->addPanel(QObject::tr("G code"));
    // 中文翻译：生成G代码
    panelNC->addLargeAction(makeAct(QObject::tr("Generate G-code"), QStringLiteral("themeicons:gcode.svg")));
    // 中文翻译：导入G代码
    panelNC->addLargeAction(makeAct(QObject::tr("Import G code"), QStringLiteral("themeicons:import.svg")));
    // 中文翻译：导出G代码
    panelNC->addLargeAction(makeAct(QObject::tr("Export G-code"), QStringLiteral("themeicons:export.svg")));
    // 中文翻译：代码查看
    panelNC->addLargeAction(makeAct(QObject::tr("code view"),  QStringLiteral("themeicons:code.svg")));

    LCNC_DEBUG(lcnc::LogCode::Generic, "lcnc::cam::buildRibbonTab end");
}

} // namespace lcnc::cam
