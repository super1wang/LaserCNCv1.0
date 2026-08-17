#include "modules/process/ui/ribbon_process_tab.h"

#include "core/command/commands_api.h"
#include "core/logging/logger.h"
#include "modules/process/commands/commands_process.h"

#include <SARibbonCategory.h>
#include <SARibbonPanel.h>
#include <QAction>
#include <QIcon>
#include <QKeySequence>

namespace lcnc::process {

void registerCommands(CommandContainer* container)
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "lcnc::process::registerCommands begin");

    container->addCommand<CmdNewProcess>(CmdNewProcess::Name);
    container->addCommand<CmdLoadProcess>(CmdLoadProcess::Name);
    container->addCommand<CmdSaveProcess>(CmdSaveProcess::Name);
    container->addCommand<CmdOpenProcessSettings>(CmdOpenProcessSettings::Name);

    container->addCommand<CmdConnectDevices>(CmdConnectDevices::Name);
    container->addCommand<CmdDisconnectDevices>(CmdDisconnectDevices::Name);

    container->addCommand<CmdRunStart>(CmdRunStart::Name);
    container->addCommand<CmdRunPause>(CmdRunPause::Name);
    container->addCommand<CmdRunStop>(CmdRunStop::Name);

    container->addCommand<CmdHome>(CmdHome::Name);
    container->addCommand<CmdMoveToLoadingPosition>(CmdMoveToLoadingPosition::Name);
    container->addCommand<CmdMoveToBlankingPosition>(CmdMoveToBlankingPosition::Name);
    container->addCommand<CmdResetStop>(CmdResetStop::Name);

    LCNC_DEBUG(lcnc::LogCode::Generic, "lcnc::process::registerCommands end");
}

void buildRibbonTab(SARibbonCategory* cat,
                    CommandContainer* container,
                    QObject* parent)
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "lcnc::process::buildRibbonTab begin");

    // ── 连接 ───────────────────────────────────────────────────────────────
    // 中文翻译：连接
    SARibbonPanel* panelConn = cat->addPanel(QObject::tr("connect"));
    panelConn->addLargeAction(container->findAction(CmdConnectDevices::Name));
    panelConn->addLargeAction(container->findAction(CmdHome::Name));
    panelConn->addLargeAction(container->findAction(CmdDisconnectDevices::Name));

    // ── 位置 ───────────────────────────────────────────────────────────────
    // 中文翻译：位置
    SARibbonPanel* panelPos = cat->addPanel(QObject::tr("position"));
    panelPos->addLargeAction(container->findAction(CmdMoveToLoadingPosition::Name));
    panelPos->addLargeAction(container->findAction(CmdMoveToBlankingPosition::Name));

    // ── 流程 ───────────────────────────────────────────────────────────────
    // 中文翻译：流程
    SARibbonPanel* panelProc = cat->addPanel(QObject::tr("process"));
    panelProc->addLargeAction(container->findAction(CmdNewProcess::Name));
    panelProc->addLargeAction(container->findAction(CmdLoadProcess::Name));
    panelProc->addLargeAction(container->findAction(CmdSaveProcess::Name));

    // ── 运行 ───────────────────────────────────────────────────────────────
    // 中文翻译：运行
    SARibbonPanel* panelRun = cat->addPanel(QObject::tr("run"));
    panelRun->addLargeAction(container->findAction(CmdRunStart::Name));
    panelRun->addLargeAction(container->findAction(CmdRunPause::Name));
    panelRun->addLargeAction(container->findAction(CmdRunStop::Name));
    panelRun->addLargeAction(container->findAction(CmdResetStop::Name));

    // ── 参数 (唯一设置按钮) ────────────────────────────────────────────────
    // 中文翻译：参数
    SARibbonPanel* panelParam = cat->addPanel(QObject::tr("parameters"));
    panelParam->addLargeAction(container->findAction(CmdOpenProcessSettings::Name));

    LCNC_DEBUG(lcnc::LogCode::Generic, "lcnc::process::buildRibbonTab end");
}

} // namespace lcnc::process
