#include "modules/cam/ui/ribbon_cam_tab.h"

#include "core/command/commands_api.h"
#include "core/logging/logger.h"
#include "modules/cam/commands/commands_machine.h"
#include "modules/cam/commands/commands_cam.h"

#include <SARibbonCategory.h>
#include <SARibbonPanel.h>
#include <QAction>
#include <QComboBox>
#include <QIcon>
#include <QWidget>

namespace lcnc::cam {

void registerCommands(CommandContainer* container)
{
    LCNC_DEBUG(lcnc::LogCode::InternalUnexpectedState, "lcnc::cam::registerCommands begin");

    // Machine
    container->addCommand<CmdLoadMachine>(CmdLoadMachine::Name);
    container->addCommand<CmdCompressMachine>(CmdCompressMachine::Name);
    container->addCommand<CmdMountWorkpiece>(CmdMountWorkpiece::Name);
    container->addCommand<CmdUnloadMachine>(CmdUnloadMachine::Name);
    container->addCommand<CmdExportMachine>(CmdExportMachine::Name);

    // CAM
    container->addCommand<CmdGenerateToolpath>(CmdGenerateToolpath::Name);
    container->addCommand<CmdSetLeadIn>(CmdSetLeadIn::Name);
    container->addCommand<CmdToolpathPreview>(CmdToolpathPreview::Name);
    container->addCommand<CmdRecalcToolpath>(CmdRecalcToolpath::Name);
    container->addCommand<CmdSimulate>(CmdSimulate::Name);

    LCNC_DEBUG(lcnc::LogCode::InternalUnexpectedState, "lcnc::cam::registerCommands end");
}

void buildRibbonTab(SARibbonCategory* cat,
                    CommandContainer* container,
                    QObject* parent)
{
    LCNC_DEBUG(lcnc::LogCode::InternalUnexpectedState, "lcnc::cam::buildRibbonTab begin");

    auto makeAct = [parent](const QString& label, const QString& iconPath) -> QAction* {
        return new QAction(QIcon(iconPath), label, parent);
    };

    // ── 机台 ───────────────────────────────────────────────────────────────
    SARibbonPanel* panelMach = cat->addPanel(QObject::tr("机台"));
    panelMach->addLargeAction(container->findAction(CmdLoadMachine::Name));
    panelMach->addSmallAction(container->findAction(CmdCompressMachine::Name));
    panelMach->addSmallAction(container->findAction(CmdMountWorkpiece::Name));
    panelMach->addSmallAction(container->findAction(CmdUnloadMachine::Name));
    panelMach->addSmallAction(container->findAction(CmdExportMachine::Name));

    // ── 刀路 ───────────────────────────────────────────────────────────────
    SARibbonPanel* panelPath = cat->addPanel(QObject::tr("刀路"));
    panelPath->addLargeAction(container->findAction(CmdGenerateToolpath::Name));
    panelPath->addSmallAction(container->findAction(CmdSetLeadIn::Name));
    panelPath->addSmallAction(container->findAction(CmdRecalcToolpath::Name));
    panelPath->addSmallAction(container->findAction(CmdToolpathPreview::Name));

    // ── G代码（占位） ─────────────────────────────────────────────────────
    SARibbonPanel* panelNC = cat->addPanel(QObject::tr("G代码"));
    panelNC->addLargeAction(makeAct(QObject::tr("生成G代码"), QStringLiteral(":/icons/gcode.svg")));
    panelNC->addSmallAction(makeAct(QObject::tr("导入G代码"), QStringLiteral(":/icons/import.svg")));
    panelNC->addSmallAction(makeAct(QObject::tr("导出G代码"), QStringLiteral(":/icons/export.svg")));
    panelNC->addSmallAction(makeAct(QObject::tr("代码查看"),  QStringLiteral(":/icons/code.svg")));

    // ── 仿真 ───────────────────────────────────────────────────────────────
    SARibbonPanel* panelSim = cat->addPanel(QObject::tr("仿真"));
    panelSim->addLargeAction(container->findAction(CmdSimulate::Name));

    auto* actPause = makeAct(QObject::tr("暂停"), QStringLiteral(":/icons/pause.svg"));
    QObject::connect(actPause, &QAction::triggered, parent, [container] {
        if (auto* cmd = static_cast<CmdSimulate*>(container->findCommand(CmdSimulate::Name)))
            cmd->pause();
    });
    panelSim->addSmallAction(actPause);

    auto* actStop = makeAct(QObject::tr("停止"), QStringLiteral(":/icons/stop.svg"));
    QObject::connect(actStop, &QAction::triggered, parent, [container] {
        if (auto* cmd = static_cast<CmdSimulate*>(container->findCommand(CmdSimulate::Name)))
            cmd->stop();
    });
    panelSim->addSmallAction(actStop);

    // 速度倍率（0.5x / 1x / 2x / 5x / 10x）
    auto* parentWidget = qobject_cast<QWidget*>(parent);
    auto* speedCombo = new QComboBox(parentWidget);
    speedCombo->addItem(QObject::tr("0.5x"), 0.5);
    speedCombo->addItem(QObject::tr("1x"),   1.0);
    speedCombo->addItem(QObject::tr("2x"),   2.0);
    speedCombo->addItem(QObject::tr("5x"),   5.0);
    speedCombo->addItem(QObject::tr("10x"),  10.0);
    speedCombo->setCurrentIndex(1);
    QObject::connect(speedCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), parent,
                     [container, speedCombo](int idx) {
                         double factor = speedCombo->itemData(idx).toDouble();
                         if (auto* cmd = static_cast<CmdSimulate*>(container->findCommand(CmdSimulate::Name)))
                             cmd->setSpeed(factor);
                     });
    panelSim->addSmallWidget(speedCombo);

    LCNC_DEBUG(lcnc::LogCode::InternalUnexpectedState, "lcnc::cam::buildRibbonTab end");
}

} // namespace lcnc::cam
