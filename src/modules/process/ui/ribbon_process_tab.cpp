#include "modules/process/ui/ribbon_process_tab.h"

#include "core/command/commands_api.h"
#include "core/logging/logger.h"
#include "modules/process/commands/commands_process.h"

#include <SARibbonCategory.h>
#include <SARibbonPanel.h>
#include <QAction>
#include <QIcon>
#include <QKeySequence>
#include <QWidget>

namespace lcnc::process {

void registerCommands(CommandContainer* container)
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "lcnc::process::registerCommands begin");

    container->addCommand<CmdConnectController>(CmdConnectController::Name);
    container->addCommand<CmdDisconnectController>(CmdDisconnectController::Name);
    container->addCommand<CmdToggleSimulationMode>(CmdToggleSimulationMode::Name);

    container->addCommand<CmdRunStart>(CmdRunStart::Name);
    container->addCommand<CmdRunPause>(CmdRunPause::Name);
    container->addCommand<CmdRunStop>(CmdRunStop::Name);

    container->addCommand<CmdHome>(CmdHome::Name);
    container->addCommand<CmdEmergencyStop>(CmdEmergencyStop::Name);
    container->addCommand<CmdResetEmergencyStop>(CmdResetEmergencyStop::Name);

    LCNC_DEBUG(lcnc::LogCode::Generic, "lcnc::process::registerCommands end");
}

void buildRibbonTab(SARibbonCategory* cat,
                    CommandContainer* container,
                    QObject* parent)
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "lcnc::process::buildRibbonTab begin");

    auto makeAct = [parent](const QString& label, const QString& iconPath) -> QAction* {
        return new QAction(QIcon(iconPath), label, parent);
    };

    // ── 连接 ───────────────────────────────────────────────────────────────
    SARibbonPanel* panelConn = cat->addPanel(QObject::tr("连接"));
    panelConn->addLargeAction(container->findAction(CmdConnectController::Name));
    panelConn->addLargeAction(container->findAction(CmdToggleSimulationMode::Name));
    panelConn->addSmallAction(container->findAction(CmdDisconnectController::Name));

    // ── 流程（占位） ───────────────────────────────────────────────────────
    SARibbonPanel* panelProc = cat->addPanel(QObject::tr("流程"));
    panelProc->addLargeAction(makeAct(QObject::tr("新建流程"), QStringLiteral(":/icons/new_process.svg")));
    panelProc->addLargeAction(makeAct(QObject::tr("加载流程"), QStringLiteral(":/icons/open_process.svg")));
    panelProc->addSmallAction(makeAct(QObject::tr("保存流程"), QStringLiteral(":/icons/save_process.svg")));

    // ── 运行 ───────────────────────────────────────────────────────────────
    SARibbonPanel* panelRun = cat->addPanel(QObject::tr("运行"));
    panelRun->addLargeAction(container->findAction(CmdRunStart::Name));
    panelRun->addSmallAction(container->findAction(CmdRunPause::Name));
    panelRun->addSmallAction(container->findAction(CmdRunStop::Name));

    // ── 安全 ───────────────────────────────────────────────────────────────
    SARibbonPanel* panelSafe = cat->addPanel(QObject::tr("安全"));
    panelSafe->addLargeAction(container->findAction(CmdEmergencyStop::Name));
    panelSafe->addSmallAction(container->findAction(CmdResetEmergencyStop::Name));
    panelSafe->addSmallAction(container->findAction(CmdHome::Name));

    // ── 参数（占位） ───────────────────────────────────────────────────────
    SARibbonPanel* panelParam = cat->addPanel(QObject::tr("参数"));
    panelParam->addSmallAction(makeAct(QObject::tr("激光参数"), QStringLiteral(":/icons/laser_param.svg")));
    panelParam->addSmallAction(makeAct(QObject::tr("运动参数"), QStringLiteral(":/icons/motion_param.svg")));
    panelParam->addSmallAction(makeAct(QObject::tr("加工设置"), QStringLiteral(":/icons/process_param.svg")));

    LCNC_DEBUG(lcnc::LogCode::Generic, "lcnc::process::buildRibbonTab end");
}

} // namespace lcnc::process
