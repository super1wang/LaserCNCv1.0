#include "modules/process/ui/ribbon_process_tab.h"

#include "core/command/commands_api.h"
#include "core/kernel/kernel.h"
#include "core/logging/logger.h"
#include "modules/process/commands/commands_process.h"
#include "modules/process/cutting/process_cutting_plan_service.h"
#include "modules/process/process_module.h"

#include <SARibbonCategory.h>
#include <SARibbonPanel.h>
#include <QAction>
#include <QComboBox>
#include <QIcon>
#include <QKeySequence>
#include <QToolButton>
#include <QVBoxLayout>
#include <QWidget>

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
    container->addCommand<CmdEmergencyStop>(CmdEmergencyStop::Name);
    container->addCommand<CmdResetEmergencyStop>(CmdResetEmergencyStop::Name);

    container->addCommand<CmdManualAppendSelectedToCuttingOrder>(CmdManualAppendSelectedToCuttingOrder::Name);
    container->addCommand<CmdAutoSortCuttingOrder>(CmdAutoSortCuttingOrder::Name);
    container->addCommand<CmdToggleTravelPath>(CmdToggleTravelPath::Name);

    LCNC_DEBUG(lcnc::LogCode::Generic, "lcnc::process::registerCommands end");
}

void buildRibbonTab(SARibbonCategory* cat,
                    CommandContainer* container,
                    QObject* parent)
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "lcnc::process::buildRibbonTab begin");

    // ── 连接 ───────────────────────────────────────────────────────────────
    SARibbonPanel* panelConn = cat->addPanel(QObject::tr("连接"));
    panelConn->addLargeAction(container->findAction(CmdConnectDevices::Name));
    panelConn->addLargeAction(container->findAction(CmdHome::Name));
    panelConn->addLargeAction(container->findAction(CmdDisconnectDevices::Name));

    // ── 流程 ───────────────────────────────────────────────────────────────
    SARibbonPanel* panelProc = cat->addPanel(QObject::tr("流程"));
    panelProc->addLargeAction(container->findAction(CmdNewProcess::Name));
    panelProc->addLargeAction(container->findAction(CmdLoadProcess::Name));
    panelProc->addLargeAction(container->findAction(CmdSaveProcess::Name));

    // ── 加工顺序 ──────────────────────────────────────────────────────────
    SARibbonPanel* panelOrder = cat->addPanel(QObject::tr("加工顺序"));
    panelOrder->addLargeAction(container->findAction(CmdManualAppendSelectedToCuttingOrder::Name));

    auto* axisCombo = new QComboBox();
    axisCombo->setObjectName("processAutoSortAxis");
    axisCombo->addItems({QStringLiteral("X+"), QStringLiteral("X-"),
                         QStringLiteral("Y+"), QStringLiteral("Y-"),
                         QStringLiteral("Z+"), QStringLiteral("Z-")});
    // 初始值从 ProcessModule 当前状态恢复（持久化由 cutting plan service 负责）
    auto restoreAxisCombo = [axisCombo]() {
        if (auto* m = lcnc::Kernel::current().service<ProcessModule>()) {
            axisCombo->setCurrentText(autoSortAxisToString(m->autoSortAxis()));
        }
    };
    restoreAxisCombo();
    QObject::connect(axisCombo, &QComboBox::currentTextChanged,
                     parent, [](const QString& text) {
                         if (auto* m = lcnc::Kernel::current().service<ProcessModule>()) {
                             m->setAutoSortAxisFromText(text);
                         }
                     });

    // Keep the sort direction physically attached to the action it controls.
    // This prevents the axis selector from being mistaken for the adjacent
    // manual-order command in the three-row ribbon layout.
    auto* autoSortControl = new QWidget(panelOrder);
    auto* autoSortLayout = new QVBoxLayout(autoSortControl);
    autoSortControl->setFixedSize(82, 56);
    autoSortLayout->setContentsMargins(1, 0, 1, 0);
    autoSortLayout->setSpacing(1);
    auto* autoSortButton = new QToolButton(autoSortControl);
    autoSortButton->setDefaultAction(container->findAction(CmdAutoSortCuttingOrder::Name));
    autoSortButton->setText(QObject::tr("自动排序"));
    autoSortButton->setToolTip(QObject::tr("自动设置加工顺序"));
    autoSortButton->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    autoSortButton->setIconSize(QSize(18, 18));
    autoSortButton->setFixedHeight(30);
    axisCombo->setParent(autoSortControl);
    axisCombo->setMinimumWidth(64);
    axisCombo->setFixedHeight(22);
    autoSortLayout->addWidget(autoSortButton);
    autoSortLayout->addWidget(axisCombo);
    panelOrder->addLargeWidget(autoSortControl);
    panelOrder->addLargeAction(container->findAction(CmdToggleTravelPath::Name));

    // ── 运行 ───────────────────────────────────────────────────────────────
    SARibbonPanel* panelRun = cat->addPanel(QObject::tr("运行"));
    panelRun->addLargeAction(container->findAction(CmdRunStart::Name));
    panelRun->addLargeAction(container->findAction(CmdRunPause::Name));
    panelRun->addLargeAction(container->findAction(CmdRunStop::Name));

    // ── 安全 ───────────────────────────────────────────────────────────────
    SARibbonPanel* panelSafe = cat->addPanel(QObject::tr("安全"));
    panelSafe->addLargeAction(container->findAction(CmdEmergencyStop::Name));
    panelSafe->addLargeAction(container->findAction(CmdResetEmergencyStop::Name));

    // ── 参数 (唯一设置按钮) ────────────────────────────────────────────────
    SARibbonPanel* panelParam = cat->addPanel(QObject::tr("参数"));
    panelParam->addLargeAction(container->findAction(CmdOpenProcessSettings::Name));

    LCNC_DEBUG(lcnc::LogCode::Generic, "lcnc::process::buildRibbonTab end");
}

} // namespace lcnc::process
