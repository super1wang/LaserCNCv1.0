#include "modules/cam/ui/ribbon/ribbon_cam_tab.h"

#include "core/command/commands_api.h"
#include "core/kernel/kernel.h"
#include "core/logging/logger.h"
#include "modules/cam/commands/machine/machine_commands.h"
#include "modules/cam/commands/toolpath/toolpath_commands.h"
#include "modules/cam/cam_module.h"

#include <SARibbonCategory.h>
#include <SARibbonPanel.h>
#include <QAction>
#include <QComboBox>
#include <QIcon>
#include <QSignalBlocker>
#include <QToolButton>
#include <QVBoxLayout>
#include <QWidget>

namespace lcnc::cam {

void registerCommands(CommandContainer* container)
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "lcnc::cam::registerCommands begin");

    // Machine
    container->addCommand<CmdLoadMachine>(CmdLoadMachine::Name);
    container->addCommand<CmdMarkAxes>(CmdMarkAxes::Name);
    container->addCommand<CmdExportSimplifiedMachine>(CmdExportSimplifiedMachine::Name);
    container->addCommand<CmdBuildMachineSafetyPackage>(CmdBuildMachineSafetyPackage::Name);
    container->addCommand<CmdUnloadMachine>(CmdUnloadMachine::Name);
    container->addCommand<CmdExportMachine>(CmdExportMachine::Name);

    // CAM
    container->addCommand<CmdGenerateToolpath>(CmdGenerateToolpath::Name);
    container->addCommand<CmdSetLeadIn>(CmdSetLeadIn::Name);
    container->addCommand<CmdToolpathPreview>(CmdToolpathPreview::Name);
    container->addCommand<CmdRecalcToolpath>(CmdRecalcToolpath::Name);
    container->addCommand<CmdSelectMachiningFace>(CmdSelectMachiningFace::Name);
    container->addCommand<CmdClearMachiningFaces>(CmdClearMachiningFaces::Name);
    container->addCommand<CmdManualAppendSelectedToCamOrder>(CmdManualAppendSelectedToCamOrder::Name);
    container->addCommand<CmdAutoSortCamOrder>(CmdAutoSortCamOrder::Name);
    container->addCommand<CmdValidateCamCollisions>(CmdValidateCamCollisions::Name);
    container->addCommand<CmdToggleCamTravelPath>(CmdToggleCamTravelPath::Name);
    container->addCommand<CmdToggleCamContourOrderLabel>(CmdToggleCamContourOrderLabel::Name);

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
    panelMach->addLargeAction(container->findAction(CmdMarkAxes::Name));
    panelMach->addLargeAction(container->findAction(CmdExportSimplifiedMachine::Name));
    panelMach->addLargeAction(container->findAction(CmdBuildMachineSafetyPackage::Name));
    panelMach->addLargeAction(container->findAction(CmdUnloadMachine::Name));
    panelMach->addLargeAction(container->findAction(CmdExportMachine::Name));

    // ── 刀路 ───────────────────────────────────────────────────────────────
    // 中文翻译：刀路
    SARibbonPanel* panelPath = cat->addPanel(QObject::tr("knife path"));
    panelPath->addLargeAction(container->findAction(CmdGenerateToolpath::Name));
    panelPath->addLargeAction(container->findAction(CmdSetLeadIn::Name));
    panelPath->addLargeAction(container->findAction(CmdRecalcToolpath::Name));
    panelPath->addLargeAction(container->findAction(CmdToolpathPreview::Name));
    panelPath->addLargeAction(container->findAction(CmdValidateCamCollisions::Name));

    // ── 加工面 ─────────────────────────────────────────────────────────────
    // 中文翻译：加工面
    SARibbonPanel* panelFace = cat->addPanel(QObject::tr("Processing surface"));
    panelFace->addLargeAction(container->findAction(CmdSelectMachiningFace::Name));
    panelFace->addLargeAction(container->findAction(CmdClearMachiningFaces::Name));

    // ── 加工顺序 ──────────────────────────────────────────────────────────
    // 中文翻译：加工顺序
    SARibbonPanel* panelOrder = cat->addPanel(QObject::tr("Processing sequence"));
    panelOrder->addLargeAction(container->findAction(CmdManualAppendSelectedToCamOrder::Name));
    auto* axisCombo = new QComboBox(panelOrder);
    axisCombo->setObjectName("camAutoSortAxis");
    axisCombo->addItems({QStringLiteral("X+"), QStringLiteral("X-"),
                         QStringLiteral("Y+"), QStringLiteral("Y-"),
                         QStringLiteral("Z+"), QStringLiteral("Z-")});
    auto axisText = [](lcnc::cam::AutoSortAxis axis) {
        switch (axis) {
        case lcnc::cam::AutoSortAxis::XPos: return QStringLiteral("X+");
        case lcnc::cam::AutoSortAxis::XNeg: return QStringLiteral("X-");
        case lcnc::cam::AutoSortAxis::YPos: return QStringLiteral("Y+");
        case lcnc::cam::AutoSortAxis::YNeg: return QStringLiteral("Y-");
        case lcnc::cam::AutoSortAxis::ZPos: return QStringLiteral("Z+");
        case lcnc::cam::AutoSortAxis::ZNeg: return QStringLiteral("Z-");
        }
        return QStringLiteral("X+");
    };
    if (auto* cam = lcnc::Kernel::current().service<CamModule>()) {
        axisCombo->setCurrentText(axisText(cam->lastAutoContourSortAxis()));
        const auto syncAxis = [axisCombo, cam, axisText] {
            const QSignalBlocker blocker(axisCombo);
            axisCombo->setCurrentText(axisText(cam->lastAutoContourSortAxis()));
        };
        QObject::connect(cam, &CamModule::toolpathGenerated, axisCombo, syncAxis);
        QObject::connect(cam, &CamModule::toolpathCleared, axisCombo, syncAxis);
    }
    QObject::connect(axisCombo, &QComboBox::currentTextChanged, parent,
                     [](const QString& text) {
                         auto* cam = lcnc::Kernel::current().service<CamModule>();
                         if (!cam) return;
                         if (text == QStringLiteral("X-")) cam->setLastAutoContourSortAxis(lcnc::cam::AutoSortAxis::XNeg);
                         else if (text == QStringLiteral("Y+")) cam->setLastAutoContourSortAxis(lcnc::cam::AutoSortAxis::YPos);
                         else if (text == QStringLiteral("Y-")) cam->setLastAutoContourSortAxis(lcnc::cam::AutoSortAxis::YNeg);
                         else if (text == QStringLiteral("Z+")) cam->setLastAutoContourSortAxis(lcnc::cam::AutoSortAxis::ZPos);
                         else if (text == QStringLiteral("Z-")) cam->setLastAutoContourSortAxis(lcnc::cam::AutoSortAxis::ZNeg);
                         else cam->setLastAutoContourSortAxis(lcnc::cam::AutoSortAxis::XPos);
                     });
    auto* autoSortControl = new QWidget(panelOrder);
    auto* autoSortLayout = new QVBoxLayout(autoSortControl);
    autoSortControl->setFixedSize(82, 56);
    autoSortLayout->setContentsMargins(1, 0, 1, 0);
    autoSortLayout->setSpacing(1);
    auto* autoSortButton = new QToolButton(autoSortControl);
    autoSortButton->setDefaultAction(container->findAction(CmdAutoSortCamOrder::Name));
    autoSortButton->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    autoSortButton->setIconSize(QSize(18, 18));
    autoSortButton->setFixedHeight(30);
    axisCombo->setParent(autoSortControl);
    axisCombo->setMinimumWidth(64);
    axisCombo->setFixedHeight(22);
    autoSortLayout->addWidget(autoSortButton);
    autoSortLayout->addWidget(axisCombo);
    panelOrder->addLargeWidget(autoSortControl);

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
