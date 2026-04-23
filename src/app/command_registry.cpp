#include "app/command_registry.h"

#include "app/commands/commands_display.h"
#include "core/command/commands_api.h"
#include "core/logging/logger.h"
#include "modules/cad/ui/ribbon_cad_tab.h"
#include "modules/cam/ui/ribbon_cam_tab.h"
#include "modules/process/ui/ribbon_process_tab.h"
#include "view/widget_occ_view.h"

#include <QAction>

namespace lcnc::app {

namespace {

/// 注册"显示"分组命令并把 QAction triggered 连接到 OccView。
void registerDisplayCommands(CommandContainer* container, WidgetOccView* occView)
{
    LCNC_DEBUG(lcnc::LogCode::InternalUnexpectedState,
               "lcnc::app::registerDisplayCommands begin");

    container->addCommand<CmdFitAll>(CmdFitAll::Name);
    container->addCommand<CmdToggleShaded>(CmdToggleShaded::Name);
    container->addCommand<CmdToggleWireframe>(CmdToggleWireframe::Name);
    container->addCommand<CmdToggleShadedWithEdges>(CmdToggleShadedWithEdges::Name);

    QObject::connect(container->findAction(CmdFitAll::Name),
                     &QAction::triggered, occView, &WidgetOccView::fitAll);
    QObject::connect(container->findAction(CmdToggleShaded::Name),
                     &QAction::triggered, occView, [occView] { occView->setDisplayMode(1); });
    QObject::connect(container->findAction(CmdToggleWireframe::Name),
                     &QAction::triggered, occView, [occView] { occView->setDisplayMode(0); });

    LCNC_DEBUG(lcnc::LogCode::InternalUnexpectedState,
               "lcnc::app::registerDisplayCommands end");
}

} // namespace

void registerAllCommands(CommandContainer* container,
                         AppContext* /*context*/,
                         WidgetOccView* occView,
                         QObject* /*parent*/)
{
    LCNC_DEBUG(lcnc::LogCode::InternalUnexpectedState,
               "lcnc::app::registerAllCommands begin");

    if (!container || !occView) {
        LCNC_ERR(lcnc::LogCode::InternalUnexpectedState,
                 "registerAllCommands: container or occView is null");
        return;
    }

    lcnc::cad::registerCommands(container);
    lcnc::cam::registerCommands(container);
    lcnc::process::registerCommands(container);
    registerDisplayCommands(container, occView);

    LCNC_DEBUG(lcnc::LogCode::InternalUnexpectedState,
               "lcnc::app::registerAllCommands end");
}

} // namespace lcnc::app
