#include "app/command_registry.h"

#include "app/commands/commands_display.h"
#include "core/command/commands_api.h"
#include "core/logging/logger.h"
#include "modules/cad/ui/ribbon_cad_tab.h"
#include "modules/cam/ui/ribbon_cam_tab.h"
#include "modules/process/ui/ribbon_process_tab.h"
#include "view/widget_occ_view.h"

#include <QAction>
#include <QActionGroup>

namespace lcnc::app {

namespace {

/// 注册"显示"分组命令；命令执行时通过 AppContext 动态取得当前 OccView。
void registerDisplayCommands(CommandContainer* container, QObject* parent)
{
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "lcnc::app::registerDisplayCommands begin");

    container->addCommand<CmdFitAll>(CmdFitAll::Name);
    container->addCommand<CmdToggleShaded>(CmdToggleShaded::Name);
    container->addCommand<CmdToggleWireframe>(CmdToggleWireframe::Name);
    container->addCommand<CmdToggleShadedWithEdges>(CmdToggleShadedWithEdges::Name);
    container->addCommand<CmdToggleWorldAxes>(CmdToggleWorldAxes::Name);
    container->addCommand<CmdShowOptions>(CmdShowOptions::Name);

    // 三个显示模式互斥：用 QActionGroup 自动维持 checked 状态唯一。
    QAction* aWire   = container->findAction(CmdToggleWireframe::Name);
    QAction* aShade  = container->findAction(CmdToggleShaded::Name);
    QAction* aEdges  = container->findAction(CmdToggleShadedWithEdges::Name);
    if (aWire && aShade && aEdges) {
        auto* group = new QActionGroup(parent);
        group->setExclusive(true);
        group->addAction(aWire);
        group->addAction(aShade);
        group->addAction(aEdges);
        // 默认“着色”：与 RenderingManager 启动时应用的 StartupDisplayMode::Shaded（mode=1，
        // FaceBoundaryDraw=false）保持一致；ribbon 高亮项必须反映视图实际生效的显示模式。
        aShade->setChecked(true);
    }

    LCNC_DEBUG(lcnc::LogCode::Generic,
               "lcnc::app::registerDisplayCommands end");
}

} // namespace

void registerAllCommands(CommandContainer* container,
                         AppContext* /*context*/,
                         WidgetOccView* occView,
                         QObject* parent)
{
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "lcnc::app::registerAllCommands begin");

    if (!container || !occView) {
        LCNC_ERR(lcnc::LogCode::InternalUnexpectedState,
                 "registerAllCommands: container or occView is null");
        return;
    }

    lcnc::cad::registerCommands(container);
    lcnc::cam::registerCommands(container);
    lcnc::process::registerCommands(container);
    registerDisplayCommands(container, parent ? parent : occView);

    LCNC_DEBUG(lcnc::LogCode::Generic,
               "lcnc::app::registerAllCommands end");
}

} // namespace lcnc::app
