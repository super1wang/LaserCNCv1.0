#include "modules/cam/ui/widget_machine_panel.h"
#include "core/kernel/kernel.h"

#include "modules/cam/cam_module.h"
#include "core/document/lcnc_document.h"
#include "core/kinematics/machine_kinematics.h"
#include "core/document/xcaf_utils.h"

#include <QDoubleSpinBox>
#include <QEvent>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QLayoutItem>
#include <QPushButton>
#include <QSignalBlocker>
#include <QVBoxLayout>
#include <QAbstractSpinBox>
#include <QCheckBox>

#include <QSet>

#include <TDF_LabelSequence.hxx>

namespace {

void clearLayout(QLayout* layout)
{
    if (!layout)
        return;

    while (QLayoutItem* item = layout->takeAt(0)) {
        if (QLayout* childLayout = item->layout())
            clearLayout(childLayout);

        if (QWidget* widget = item->widget())
            delete widget;

        delete item;
    }
}

bool supportsAcCalibration(const MachineKinematics* kin)
{
    return kin
        && kin->configType() == QStringLiteral("VERTICAL_AC_TABLE")
        && kin->findAxis(QStringLiteral("A"))
        && kin->findAxis(QStringLiteral("C"));
}

QString formatPointText(const gp_Pnt& point)
{
    return QObject::tr("X=%1  Y=%2  Z=%3")
        .arg(point.X(), 0, 'f', 3)
        .arg(point.Y(), 0, 'f', 3)
        .arg(point.Z(), 0, 'f', 3);
}

QDoubleSpinBox* createMillimeterSpin(QWidget* parent,
                                     double minValue = -99999.0,
                                     double maxValue = 99999.0)
{
    auto* spin = new QDoubleSpinBox(parent);
    spin->setRange(minValue, maxValue);
    spin->setDecimals(3);
    spin->setSingleStep(1.0);
    spin->setSuffix(QStringLiteral(" mm"));
    spin->setMinimumWidth(120);
    spin->setFocusPolicy(Qt::StrongFocus);
    return spin;
}

} // namespace

WidgetMachinePanel::WidgetMachinePanel(QWidget* parent)
    : QWidget(parent)
{
    buildUi();
}

bool WidgetMachinePanel::eventFilter(QObject* watched, QEvent* event)
{
    if (event && event->type() == QEvent::Wheel
        && qobject_cast<QAbstractSpinBox*>(watched)) {
        event->ignore();
        return true;
    }
    return QWidget::eventFilter(watched, event);
}

void WidgetMachinePanel::setDocument(LcncDocument* doc)
{
    m_doc = doc;

    const MachineKinematics* kin = doc ? doc->machineKinematics() : nullptr;
    Q_UNUSED(kin);

    refreshCalibrationSection();
    if (!doc)
        m_selectedEntries.clear();
    rebuildAssignmentSection();
    rebuildWpcSection();

}

void WidgetMachinePanel::buildUi()
{
    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(4, 4, 4, 4);
    mainLayout->setSpacing(6);

    buildConfigPage();
    const auto spins = findChildren<QAbstractSpinBox*>();
    for (QAbstractSpinBox* spin : spins) {
        spin->setFocusPolicy(Qt::StrongFocus);
        spin->installEventFilter(this);
    }
    mainLayout->addWidget(m_configPage);
}

void WidgetMachinePanel::buildConfigPage()
{
    m_configPage = new QWidget(this);
    auto* mainLayout = new QVBoxLayout(m_configPage);
    mainLayout->setContentsMargins(4, 4, 4, 4);
    mainLayout->setSpacing(8);

    // 中文翻译：标记所选部件
    m_assignGroup = new QGroupBox(tr("Mark selected parts"), m_configPage);
    auto* assignLayout = new QVBoxLayout(m_assignGroup);
    assignLayout->setContentsMargins(6, 6, 6, 6);
    assignLayout->setSpacing(6);
    // 中文翻译：请在机台视图或模型树中选择机台部件。
    m_lblAssignSelection = new QLabel(tr("Please select the machine part in the machine view or model tree."), m_assignGroup);
    m_lblAssignSelection->setWordWrap(true);
    m_lblAssignSelection->setStyleSheet("color: #888; font-size: 11px;");
    assignLayout->addWidget(m_lblAssignSelection);
    m_assignGrid = new QGridLayout;
    m_assignGrid->setHorizontalSpacing(8);
    m_assignGrid->setVerticalSpacing(6);
    m_assignGrid->setColumnStretch(0, 1);
    m_assignGrid->setColumnStretch(1, 1);
    assignLayout->addLayout(m_assignGrid);
    mainLayout->addWidget(m_assignGroup);

    // 中文翻译：坐标系转换
    auto* calibrationGroup = new QGroupBox(tr("Coordinate system conversion"), m_configPage);
    auto* calibrationLayout = new QVBoxLayout(calibrationGroup);
    calibrationLayout->setContentsMargins(6, 6, 6, 6);
    calibrationLayout->setSpacing(8);

    // 唯一入口：打开三段式标定向导（需求 3：移除旧的轴心/AC 中心/切割头独立控件）
    m_btnOpenCalibrationWizard = new QPushButton(
        // 中文翻译：打开标定向导...
        tr("Open the Calibration Wizard..."), calibrationGroup);
    m_btnOpenCalibrationWizard->setToolTip(
        // 中文翻译：依次拾取 A 轴、C 轴参考面与切割头下端面，填入物理 AC 中心与 A/C 角度，
        tr("Select the A-axis and C-axis reference planes and the lower end face of the cutting head in sequence, and fill in the physical AC center and A/C angle."
           // 中文翻译：一次性完成机台坐标系标定，并自动持久化到 cam.toml 与机台 STEP。
           "The machine coordinate system calibration is completed in one go and automatically persisted to cam.toml and machine STEP."));
    calibrationLayout->addWidget(m_btnOpenCalibrationWizard);
    connect(m_btnOpenCalibrationWizard, &QPushButton::clicked,
            this, &WidgetMachinePanel::axisCalibrationWizardRequested);

    mainLayout->addWidget(calibrationGroup, 1);

    buildWorkpiecePage();

    mainLayout->addStretch();
}

void WidgetMachinePanel::buildWorkpiecePage()
{
    auto* mainLayout = qobject_cast<QVBoxLayout*>(m_configPage ? m_configPage->layout() : nullptr);
    if (!mainLayout)
        return;

    // 中文翻译：工件安装姿态
    m_installGroup = new QGroupBox(tr("Workpiece setup"), m_configPage);
    auto* installLayout = new QFormLayout(m_installGroup);
    installLayout->setContentsMargins(6, 6, 6, 6);
    installLayout->setSpacing(6);
    // 中文翻译：自动挂载工件
    m_chkAutoInstallWorkpiece = new QCheckBox(tr("Automatically mount workpieces"), m_installGroup);
    const QStringList setupLabels = {
        tr("Setup X:"), tr("Setup Y:"), tr("Setup Z:"),
        tr("Rotation X:"), tr("Rotation Y:"), tr("Rotation Z:")};
    for (int index = 0; index < 6; ++index) {
        m_workpieceSetupEditors[index] = createMillimeterSpin(m_installGroup);
        if (index >= 3) {
            m_workpieceSetupEditors[index]->setRange(-360.0, 360.0);
            m_workpieceSetupEditors[index]->setSuffix(tr(" °"));
        }
    }
    // 中文翻译：将安装原点设为旋转中心
    m_btnAlignRotationCenter = new QPushButton(tr("Set setup origin to rotation center"), m_installGroup);
    installLayout->addRow(m_chkAutoInstallWorkpiece);
    for (int index = 0; index < 6; ++index)
        installLayout->addRow(setupLabels[index], m_workpieceSetupEditors[index]);
    installLayout->addRow(m_btnAlignRotationCenter);
    auto* installHint = new QLabel(
        // 中文翻译：这是 CAD 工件坐标到夹具零位的唯一刚体变换。它同时驱动模型显示和机床坐标，修改后需要重新计算刀路。
        tr("This is the only rigid transform from CAD workpiece coordinates to the fixture zero. It drives both model display and machine coordinates; recalculate the toolpath after a change."),
        m_installGroup);
    installHint->setWordWrap(true);
    installHint->setStyleSheet("color:#666;");
    installLayout->addRow(installHint);
    mainLayout->addWidget(m_installGroup);

    connect(m_chkAutoInstallWorkpiece, &QCheckBox::toggled,
        this, &WidgetMachinePanel::autoInstallWorkpieceChanged);
    connect(m_btnAlignRotationCenter, &QPushButton::clicked,
        this, &WidgetMachinePanel::alignWorkpieceSetupToRotationCenterRequested);
    for (QDoubleSpinBox* editor : m_workpieceSetupEditors) {
        connect(editor, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, &WidgetMachinePanel::onWorkpieceSetupChanged);
    }
}

void WidgetMachinePanel::refreshCalibrationSection()
{
    const MachineKinematics* kin = m_doc ? m_doc->machineKinematics() : nullptr;
    const bool supported = supportsAcCalibration(kin);
    const bool hasPreset = kin && !kin->configType().isEmpty();

    if (m_groupAcAxes)
        m_groupAcAxes->setVisible(supported);
    if (m_groupAcCenter)
        m_groupAcCenter->setVisible(supported);
    if (m_groupHeadAlignment)
        m_groupHeadAlignment->setVisible(hasPreset);
    if (m_lblCalibrationHint) {
        m_lblCalibrationHint->setText(
            supported
                // 中文翻译：适用于 AC 转台：A 轴参考面写入 Y/Z，C 轴参考面写入 X。整机对齐只做平移。切割头模型点与物理点可独立录入和对齐。
                ? tr("Applicable to AC rotary tables: Write Y/Z for the A-axis reference plane and X for the C-axis reference plane. The whole machine is aligned only for translation. The cutting head model points and physical points can be entered and aligned independently.")
                // 中文翻译：当前页用于轴心与切割头位置配置。AC 轴心快填和 AC 中心对齐仅在 AC 转台构型下显示。
                : tr("The current page is used for axis and cutting head position configuration. AC Pivot Quick Fill and AC Center Align are only shown in AC rotary configuration."));
    }

    if (m_axisAySpin && kin && kin->findAxis(QStringLiteral("A"))) {
        const gp_Pnt origin = kin->axisOrigin(QStringLiteral("A"));
        const QSignalBlocker blockY(m_axisAySpin);
        const QSignalBlocker blockZ(m_axisAzSpin);
        m_axisAySpin->setValue(origin.Y());
        m_axisAzSpin->setValue(origin.Z());
    }

    if (m_axisCxSpin && kin && kin->findAxis(QStringLiteral("C"))) {
        const gp_Pnt origin = kin->axisOrigin(QStringLiteral("C"));
        const QSignalBlocker blockX(m_axisCxSpin);
        m_axisCxSpin->setValue(origin.X());
    }

    gp_Pnt center;
    const bool hasCenter = supported && lcnc::Kernel::current().service<CamModule>()->currentAcRotationCenter(center);
    if (m_lblCurrentAcCenter) {
        if (hasCenter) {
            m_lblCurrentAcCenter->setText(formatPointText(center));
            m_lblCurrentAcCenter->setStyleSheet(QString());
        } else {
            // 中文翻译：当前构型暂不支持 AC 中心对齐。
            m_lblCurrentAcCenter->setText(tr("The current configuration does not support AC center alignment."));
            m_lblCurrentAcCenter->setStyleSheet("color: gray; font-size: 11px;");
        }
    }

    for (QDoubleSpinBox* spin : {m_targetCenterX, m_targetCenterY, m_targetCenterZ}) {
        if (spin)
            spin->setEnabled(supported);
    }

    if (m_btnPickAxisA)
        m_btnPickAxisA->setEnabled(supported);
    if (m_btnPickAxisC)
        m_btnPickAxisC->setEnabled(supported);
    if (m_axisAySpin)
        m_axisAySpin->setEnabled(supported);
    if (m_axisAzSpin)
        m_axisAzSpin->setEnabled(supported);
    if (m_axisCxSpin)
        m_axisCxSpin->setEnabled(supported);
    if (m_btnAlignToPhysical)
        m_btnAlignToPhysical->setEnabled(supported);
    if (m_btnOpenCalibrationWizard)
        m_btnOpenCalibrationWizard->setEnabled(supported);

    const gp_Pnt cutterHeadModel = lcnc::Kernel::current().service<CamModule>()->cutterHeadModelPosition();
    const gp_Pnt cutterHeadPhysical = lcnc::Kernel::current().service<CamModule>()->cutterHeadPhysicalPosition();
    for (const auto pair : {
             std::pair<QDoubleSpinBox*, double>(m_headModelX, cutterHeadModel.X()),
             std::pair<QDoubleSpinBox*, double>(m_headModelY, cutterHeadModel.Y()),
             std::pair<QDoubleSpinBox*, double>(m_headModelZ, cutterHeadModel.Z()),
             std::pair<QDoubleSpinBox*, double>(m_headPhysicalX, cutterHeadPhysical.X()),
             std::pair<QDoubleSpinBox*, double>(m_headPhysicalY, cutterHeadPhysical.Y()),
             std::pair<QDoubleSpinBox*, double>(m_headPhysicalZ, cutterHeadPhysical.Z()) }) {
        if (!pair.first)
            continue;
        const QSignalBlocker blocker(pair.first);
        pair.first->setValue(pair.second);
    }

    if (m_lblHeadModelPoint)
        // 中文翻译：当前切割头模型点: %1
        m_lblHeadModelPoint->setText(tr("Current cutting head model point: %1")
                                         .arg(formatPointText(cutterHeadModel)));

    if (!m_lblPickStatus)
        return;

    if (!m_pendingCalibrationAxis.isEmpty()) {
        const QString targetText = (m_pendingCalibrationAxis == QStringLiteral("CUTTER_HEAD"))
            // 中文翻译：切割头对齐
            ? tr("Cutting head alignment")
            // 中文翻译：%1 轴参考平面
            : tr("%1 axis reference plane").arg(m_pendingCalibrationAxis);
        m_lblPickStatus->setText(
            // 中文翻译：正在拾取 %1：左键确认，右键或 ESC 取消。
            tr("Picking %1: left click to confirm, right click or ESC to cancel.")
                .arg(targetText));
        m_lblPickStatus->setStyleSheet("color: #c98512; font-size: 11px; font-weight: bold;");
        return;
    }

    if (!supported) {
        // 中文翻译：左键选择参考平面，右键或 ESC 取消。切割头拾取始终可用，AC 轴心快填仅在 AC 转台构型下显示。
        m_lblPickStatus->setText(tr("Left click to select the reference plane, right click or ESC to cancel. Cutting head pick-up is always available, AC pivot fill is only shown in AC rotary configuration."));
        m_lblPickStatus->setStyleSheet("color: #888; font-size: 11px;");
        return;
    }

    // 中文翻译：左键选择参考平面，右键或 ESC 取消。
    m_lblPickStatus->setText(tr("Left click to select the reference plane, right click or ESC to cancel."));
    m_lblPickStatus->setStyleSheet("color: #888; font-size: 11px;");
}

void WidgetMachinePanel::rebuildAssignmentSection()
{
    if (!m_assignGroup || !m_lblAssignSelection || !m_assignGrid)
        return;

    clearLayout(m_assignGrid);

    const MachineKinematics* kin = m_doc ? m_doc->machineKinematics() : nullptr;
    const bool hasMachineEntities = m_doc
        && m_doc->entityLabels(LcncDocument::EntityKind::Machine).Length() > 0;
    if (!m_doc || !kin || !hasMachineEntities) {
        // 中文翻译：加载机台模型后，可为选中机台部件直接标记所属轴系。
        m_lblAssignSelection->setText(tr("After loading the machine model, you can directly mark the axis system to which the selected machine component belongs."));
        m_lblAssignSelection->setStyleSheet("color: #888; font-size: 11px;");
        return;
    }

    if (m_selectedEntries.isEmpty()) {
        // 中文翻译：请在机台视图或模型树中选择机台部件。
        m_lblAssignSelection->setText(tr("Please select the machine part in the machine view or model tree."));
        m_lblAssignSelection->setStyleSheet("color: #888; font-size: 11px;");
        return;
    }

    TDF_LabelSequence labels = m_doc->entityLabels(LcncDocument::EntityKind::Machine);
    QMap<QString, QString> entryNames;
    for (int i = 1; i <= labels.Length(); ++i) {
        const TDF_Label label = labels.Value(i);
        entryNames.insert(XcafUtils::entry(label), XcafUtils::name(label));
    }

    QStringList selectedNames;
    for (const QString& entry : m_selectedEntries)
        selectedNames << entryNames.value(entry, entry);

    QString summaryText;
    if (selectedNames.size() == 1) {
        // 中文翻译：当前选中: %1
        summaryText = tr("Currently selected: %1").arg(selectedNames.first());
    } else if (selectedNames.size() <= 3) {
        // 中文翻译：当前选中 %1 个部件: %2
        summaryText = tr("Currently %1 components are selected: %2")
            .arg(selectedNames.size())
            .arg(selectedNames.join(tr("、")));
    } else {
        // 中文翻译：当前选中 %1 个机台部件，可直接点击下方按钮归轴。
        summaryText = tr("%1 machine components are currently selected. You can directly click the button below to return to the axis.")
            .arg(selectedNames.size());
    }
    m_lblAssignSelection->setText(summaryText);
    m_lblAssignSelection->setStyleSheet("color: #2f5f9f; font-size: 11px;");

    int index = 0;
    for (const MachineAxisDef& axis : kin->axes()) {
        const QString axisName = axis.name;
        QString buttonText;
        if (axisName == QStringLiteral("BASE"))
            // 中文翻译：标记为 BASE
            buttonText = tr("Marked BASE");
        else
            // 中文翻译：标记为 %1
            buttonText = tr("Marked with %1").arg(axisName);

        auto* button = new QPushButton(buttonText, m_assignGroup);
        // 中文翻译：将当前选中的机台部件归到 %1 轴
        button->setToolTip(tr("Assign the currently selected machine parts to the %1 axis").arg(axisName));
        connect(button, &QPushButton::clicked, this, [this, axisName] {
            if (m_selectedEntries.isEmpty())
                return;
            lcnc::Kernel::current().service<CamModule>()->assignShapesToAxis(m_selectedEntries, axisName);
        });

        const int row = index / 2;
        const int column = index % 2;
        m_assignGrid->addWidget(button, row, column);
        ++index;
    }

    // 中文翻译：解除所选归轴
    auto* clearButton = new QPushButton(tr("Unselect homing"), m_assignGroup);
    // 中文翻译：清除当前选中机台部件已有的轴系归属
    clearButton->setToolTip(tr("Clear the existing axis system ownership of the currently selected machine parts"));
    connect(clearButton, &QPushButton::clicked, this, [this] {
        for (const QString& entry : m_selectedEntries)
            lcnc::Kernel::current().service<CamModule>()->unassignShape(entry);
    });
    m_assignGrid->addWidget(clearButton, index / 2, index % 2);
    ++index;

}

void WidgetMachinePanel::rebuildWpcSection()
{
    const MachineKinematics* kin = m_doc ? m_doc->machineKinematics() : nullptr;
    const bool hasPreset = kin && !kin->configType().isEmpty();
    const QString configType = kin ? kin->configType() : QString();
    const bool showRotationButton = configType == QStringLiteral("VERTICAL_AC_TABLE")
        || configType == QStringLiteral("VERTICAL_BC_TABLE")
        || configType == QStringLiteral("XYZA");

    if (m_chkAutoInstallWorkpiece) {
        const QSignalBlocker blocker(m_chkAutoInstallWorkpiece);
        const bool autoInstall = lcnc::Kernel::current().service<CamModule>()->autoInstallWorkpiece();
        m_chkAutoInstallWorkpiece->setChecked(autoInstall);
        m_chkAutoInstallWorkpiece->setEnabled(hasPreset);
    }

    if (m_workpieceSetupEditors[0]) {
        const lcnc::WorkpieceSetupTransform setup =
            lcnc::Kernel::current().service<CamModule>()->workpieceSetupTransform();
        const std::array<double, 6> values{setup.x, setup.y, setup.z,
            setup.rotationXDeg, setup.rotationYDeg, setup.rotationZDeg};
        for (int index = 0; index < 6; ++index) {
            const QSignalBlocker blocker(m_workpieceSetupEditors[index]);
            m_workpieceSetupEditors[index]->setValue(values[index]);
            m_workpieceSetupEditors[index]->setEnabled(hasPreset);
        }
    }

    if (m_installGroup)
        m_installGroup->setEnabled(hasPreset);

    if (m_btnAlignRotationCenter) {
        m_btnAlignRotationCenter->setVisible(showRotationButton);
        m_btnAlignRotationCenter->setEnabled(showRotationButton && hasPreset);
        if (configType == QStringLiteral("VERTICAL_AC_TABLE")) {
            // 中文翻译：将安装原点回填为 AC 旋转中心。
            m_btnAlignRotationCenter->setToolTip(tr("Set the setup origin to the AC center of rotation."));
        } else if (configType == QStringLiteral("VERTICAL_BC_TABLE")) {
            // 中文翻译：将安装原点回填为 BC 旋转中心。
            m_btnAlignRotationCenter->setToolTip(tr("Set the setup origin to the BC center of rotation."));
        } else if (configType == QStringLiteral("XYZA")) {
            // 中文翻译：将安装原点回填为 A 转台中心。
            m_btnAlignRotationCenter->setToolTip(tr("Set the setup origin to the A turntable center."));
        } else {
            m_btnAlignRotationCenter->setToolTip(QString());
        }
    }
}

void WidgetMachinePanel::setSelectedEntries(const QStringList& entries)
{
    m_selectedEntries.clear();

    if (!m_doc) {
        rebuildAssignmentSection();
        return;
    }

    TDF_LabelSequence labels = m_doc->entityLabels(LcncDocument::EntityKind::Machine);
    QSet<QString> machineEntries;
    for (int i = 1; i <= labels.Length(); ++i)
        machineEntries.insert(XcafUtils::entry(labels.Value(i)));

    for (const QString& entry : entries) {
        if (machineEntries.contains(entry) && !m_selectedEntries.contains(entry))
            m_selectedEntries.append(entry);
    }

    rebuildAssignmentSection();
}

void WidgetMachinePanel::setCalibrationPickAxis(const QString& axisName)
{
    if (m_pendingCalibrationAxis == axisName)
        return;

    m_pendingCalibrationAxis = axisName;
    refreshCalibrationSection();
}

void WidgetMachinePanel::onAxisOriginEditorChanged()
{
    if (!m_doc)
        return;

    const MachineKinematics* kin = m_doc->machineKinematics();
    if (!kin)
        return;

    const gp_Pnt aOrigin = kin->axisOrigin(QStringLiteral("A"));
    emit axisOriginChanged(QStringLiteral("A"), aOrigin.X(),
                           m_axisAySpin ? m_axisAySpin->value() : aOrigin.Y(),
                           m_axisAzSpin ? m_axisAzSpin->value() : aOrigin.Z());

    const gp_Pnt cOrigin = kin->axisOrigin(QStringLiteral("C"));
    emit axisOriginChanged(QStringLiteral("C"),
                           m_axisCxSpin ? m_axisCxSpin->value() : cOrigin.X(),
                           cOrigin.Y(), cOrigin.Z());
}

void WidgetMachinePanel::onCutterHeadModelEditorChanged()
{
    emit cutterHeadModelPositionChanged(
        m_headModelX ? m_headModelX->value() : 0.0,
        m_headModelY ? m_headModelY->value() : 0.0,
        m_headModelZ ? m_headModelZ->value() : 0.0);
}

void WidgetMachinePanel::onCutterHeadPhysicalEditorChanged()
{
    emit cutterHeadPhysicalPositionChanged(
        m_headPhysicalX ? m_headPhysicalX->value() : 0.0,
        m_headPhysicalY ? m_headPhysicalY->value() : 0.0,
        m_headPhysicalZ ? m_headPhysicalZ->value() : 0.0);
}

void WidgetMachinePanel::onWorkpieceSetupChanged()
{
    emit workpieceSetupChanged(
        m_workpieceSetupEditors[0]->value(), m_workpieceSetupEditors[1]->value(),
        m_workpieceSetupEditors[2]->value(), m_workpieceSetupEditors[3]->value(),
        m_workpieceSetupEditors[4]->value(), m_workpieceSetupEditors[5]->value());
}
