#include "modules/process/ui/ribbon_process_tab.h"

#include "core/command/commands_api.h"
#include "core/kernel/kernel.h"
#include "core/logging/logger.h"
#include "modules/process/process_module.h"

#include <SARibbonCategory.h>
#include <SARibbonPanel.h>
#include <QAction>
#include <QIcon>
#include <QInputDialog>
#include <QKeySequence>
#include <QLineEdit>
#include <QWidget>

namespace lcnc::process {

void registerCommands(CommandContainer* /*container*/)
{
    LCNC_DEBUG(lcnc::LogCode::InternalUnexpectedState,
               "lcnc::process::registerCommands begin (currently no-op)");
    // Process 模块暂无独立命令对象。
    LCNC_DEBUG(lcnc::LogCode::InternalUnexpectedState, "lcnc::process::registerCommands end");
}

void buildRibbonTab(SARibbonCategory* cat,
                    CommandContainer* /*container*/,
                    QObject* parent)
{
    LCNC_DEBUG(lcnc::LogCode::InternalUnexpectedState, "lcnc::process::buildRibbonTab begin");

    auto* parentWidget = qobject_cast<QWidget*>(parent);
    auto* processModule = lcnc::Kernel::current().service<ProcessModule>();
    if (!processModule) {
        LCNC_WARN(lcnc::LogCode::InternalUnexpectedState,
                  "Process module unavailable while building ribbon");
    }

    auto makeAct = [parent](const QString& label, const QString& iconPath) -> QAction* {
        return new QAction(QIcon(iconPath), label, parent);
    };

    // ── 连接 ───────────────────────────────────────────────────────────────
    SARibbonPanel* panelConn = cat->addPanel(QObject::tr("连接"));
    QAction* actConnect    = makeAct(QObject::tr("连接控制器"), QStringLiteral(":/icons/connect.svg"));
    QAction* actSimulation = makeAct(QObject::tr("仿真模式"),   QStringLiteral(":/icons/simulate.svg"));
    QAction* actDisconnect = makeAct(QObject::tr("断开连接"),   QStringLiteral(":/icons/disconnect.svg"));
    actSimulation->setCheckable(true);
    if (processModule)
        actSimulation->setChecked(processModule->simulationMode());
    panelConn->addLargeAction(actConnect);
    panelConn->addLargeAction(actSimulation);
    panelConn->addSmallAction(actDisconnect);

    QObject::connect(actConnect, &QAction::triggered, parent, [parentWidget] {
        bool ok = false;
        const QString endpoint = QInputDialog::getText(
            parentWidget,
            QObject::tr("连接控制器"),
            QObject::tr("控制器地址:"),
            QLineEdit::Normal,
            QStringLiteral("tcp://127.0.0.1:5000"),
            &ok);
        if (!ok)
            return;
        if (auto* pm = lcnc::Kernel::current().service<ProcessModule>())
            pm->connectController(endpoint);
    });
    if (processModule) {
        QObject::connect(actDisconnect, &QAction::triggered,
                         processModule, &ProcessModule::disconnectController);
        QObject::connect(actSimulation, &QAction::toggled,
                         processModule, &ProcessModule::setSimulationMode);
        QObject::connect(processModule, &ProcessModule::simulationModeChanged,
                         actSimulation, &QAction::setChecked);
    }

    // ── 流程（占位） ───────────────────────────────────────────────────────
    SARibbonPanel* panelProc = cat->addPanel(QObject::tr("流程"));
    panelProc->addLargeAction(makeAct(QObject::tr("新建流程"), QStringLiteral(":/icons/new_process.svg")));
    panelProc->addLargeAction(makeAct(QObject::tr("加载流程"), QStringLiteral(":/icons/open_process.svg")));
    panelProc->addSmallAction(makeAct(QObject::tr("保存流程"), QStringLiteral(":/icons/save_process.svg")));

    // ── 运行 ───────────────────────────────────────────────────────────────
    SARibbonPanel* panelRun = cat->addPanel(QObject::tr("运行"));
    auto* actStart = makeAct(QObject::tr("运行"), QStringLiteral(":/icons/start.svg"));
    auto* actPause = makeAct(QObject::tr("暂停"), QStringLiteral(":/icons/pause.svg"));
    auto* actStop  = makeAct(QObject::tr("停止"), QStringLiteral(":/icons/stop.svg"));
    actStart->setShortcut(QKeySequence(Qt::Key_F5));
    panelRun->addLargeAction(actStart);
    panelRun->addSmallAction(actPause);
    panelRun->addSmallAction(actStop);

    if (processModule) {
        QObject::connect(actStart, &QAction::triggered, processModule, &ProcessModule::runStart);
        QObject::connect(actPause, &QAction::triggered, processModule, &ProcessModule::runPause);
        QObject::connect(actStop,  &QAction::triggered, processModule, &ProcessModule::runStop);
    }

    // ── 参数（占位） ───────────────────────────────────────────────────────
    SARibbonPanel* panelParam = cat->addPanel(QObject::tr("参数"));
    panelParam->addSmallAction(makeAct(QObject::tr("激光参数"), QStringLiteral(":/icons/laser_param.svg")));
    panelParam->addSmallAction(makeAct(QObject::tr("运动参数"), QStringLiteral(":/icons/motion_param.svg")));
    panelParam->addSmallAction(makeAct(QObject::tr("加工设置"), QStringLiteral(":/icons/process_param.svg")));

    LCNC_DEBUG(lcnc::LogCode::InternalUnexpectedState, "lcnc::process::buildRibbonTab end");
}

} // namespace lcnc::process
