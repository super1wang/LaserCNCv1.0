#include "modules/cam/ui/machine/widget_machine_panel.h"

#include "core/document/lcnc_document.h"
#include "core/document/xcaf_utils.h"
#include "core/kernel/kernel.h"
#include "core/kinematics/machine_configuration_service.h"
#include "core/kinematics/machine_kinematics.h"
#include "modules/cam/cam_module.h"

#include <NCollection_Sequence.hxx>
#include <QAbstractSpinBox>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QEvent>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QLayoutItem>
#include <QPushButton>
#include <QSet>
#include <QSignalBlocker>
#include <QVBoxLayout>
#include <TDF_Label.hxx>

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
    if (!kin)
        return false;
    const MachineAxisDef* tiltAxis = nullptr;
    const MachineAxisDef* spinAxis = nullptr;
    for (const MachineAxisDef& axis : kin->axes()) {
        if (axis.role == lcnc::MachineAxisRole::TableTilt)
            tiltAxis = &axis;
        else if (axis.role == lcnc::MachineAxisRole::TableSpin)
            spinAxis = &axis;
    }
    return tiltAxis && spinAxis
        && kin->isAxisDescendantOf(spinAxis->name, tiltAxis->name);
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
        // 中文翻译：打开 STEP 几何对齐向导...
        tr("Open STEP geometry alignment wizard..."), calibrationGroup);
    m_btnOpenCalibrationWizard->setToolTip(
        // 中文翻译：依次拾取父旋转轴、子旋转轴参考面与切割头下端面；软件根据轴角色、方向和父链自动求取标定关系。
        tr("Select the parent rotary-axis, child rotary-axis, and lower cutter-head reference faces in sequence. Calibration is derived automatically from axis roles, directions, and parent links."
           // 中文翻译：一次性完成机台坐标系标定，并自动持久化到 cam.toml 与机台 STEP。
           "The machine coordinate system calibration is completed in one go and automatically persisted to cam.toml and machine STEP."));
    calibrationLayout->addWidget(m_btnOpenCalibrationWizard);
    connect(m_btnOpenCalibrationWizard, &QPushButton::clicked,
            this, &WidgetMachinePanel::axisCalibrationWizardRequested);

    m_btnOpenPhysicalCalibrationWizard = new QPushButton(
        // 中文翻译：打开物理五轴标定向导...
        tr("Open physical five-axis calibration wizard..."), calibrationGroup);
    m_btnOpenPhysicalCalibrationWizard->setToolTip(
        // 中文翻译：导入或填写多个旋转姿态的实测轴反馈与参考点坐标，自动拟合两条旋转轴线并生成带验证状态的不可变标定记录。
        tr("Import or enter measured axis feedback and reference-point coordinates at multiple rotary poses. Both rotary-axis lines are fitted automatically and saved as an immutable calibration record with an explicit verification state."));
    calibrationLayout->addWidget(m_btnOpenPhysicalCalibrationWizard);
    connect(m_btnOpenPhysicalCalibrationWizard, &QPushButton::clicked, this,
            &WidgetMachinePanel::physicalKinematicsCalibrationWizardRequested);

    m_btnGenerateConfigurationRtcp = new QPushButton(
        // 中文翻译：从当前构型生成 RTCP 参数...
        tr("Generate RTCP parameters from current configuration..."),
        calibrationGroup);
    m_btnGenerateConfigurationRtcp->setToolTip(
        // 中文翻译：使用当前旋转轴中心、方向、轴号和 TCP 生成 RTCP 记录，按正常刀具速度和激光/气体时序加工；这不是精密标定。
        tr("Generate an RTCP record from the current rotary centers, directions, axis mapping, and TCP. Machining uses configured tool speeds and laser/gas sequences. This is not a precision calibration."));
    calibrationLayout->addWidget(m_btnGenerateConfigurationRtcp);
    connect(m_btnGenerateConfigurationRtcp, &QPushButton::clicked, this,
            &WidgetMachinePanel::configurationDerivedRtcpRequested);

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
        // 中文翻译：安装 XYZ 直接使用轴系示教坐标；旋转仍为右手几何角。该变换同时驱动模型、碰撞与刀路，修改后需重新计算。
        tr("Setup XYZ uses taught controller-axis coordinates directly; rotations remain right-handed geometric angles. This transform drives model display, collision, and toolpath solving, so recalculate the toolpath after a change."),
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
                // 中文翻译：适用于配置了 TableTilt/TableSpin 角色的串联转台：软件根据轴方向和父链自动求旋转中心，并沿真实刀头承载链完成对齐。
                ? tr("Applicable to serial rotary tables with TableTilt/TableSpin roles: the rotation center is derived from axis directions and parent links, and cutter alignment follows the physical tool-carrier chain.")
                // 中文翻译：当前轴角色或父链不构成受支持的串联转台，标定入口已禁用。
                : tr("The configured axis roles or parent links do not form a supported serial rotary table, so calibration is disabled."));
    }

    auto* machineConfig = lcnc::Kernel::current().service<lcnc::MachineConfigurationService>();
    auto axisCoordinates = [machineConfig](const gp_Pnt& world) {
        gp_Pnt axis;
        return machineConfig && machineConfig->worldToAxisCoordinates(world, &axis)
            ? axis : world;
    };
    if (m_axisAzSpin && kin && kin->findAxis(QStringLiteral("A"))) {
        const gp_Pnt origin = axisCoordinates(kin->axisOrigin(QStringLiteral("A")));
        const QSignalBlocker blockZ(m_axisAzSpin);
        m_axisAzSpin->setValue(origin.Z());
    }

    if (m_axisCxSpin && m_axisCySpin && kin && kin->findAxis(QStringLiteral("C"))) {
        const gp_Pnt origin = axisCoordinates(kin->axisOrigin(QStringLiteral("C")));
        const QSignalBlocker blockX(m_axisCxSpin);
        const QSignalBlocker blockY(m_axisCySpin);
        m_axisCxSpin->setValue(origin.X());
        m_axisCySpin->setValue(origin.Y());
    }

    gp_Pnt center;
    const bool hasCenter = supported
        && lcnc::Kernel::current().service<CamModule>()
               ->currentAcRotationCenterAxisCoordinates(center);
    if (m_lblCurrentAcCenter) {
        if (hasCenter) {
            m_lblCurrentAcCenter->setText(formatPointText(center));
            m_lblCurrentAcCenter->setStyleSheet(QString());
        } else {
            // 中文翻译：当前构型暂不支持转台中心对齐。
            m_lblCurrentAcCenter->setText(tr("The current configuration does not support rotary-table center alignment."));
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
    if (m_axisAzSpin)
        m_axisAzSpin->setEnabled(supported);
    if (m_axisCxSpin)
        m_axisCxSpin->setEnabled(supported);
    if (m_axisCySpin)
        m_axisCySpin->setEnabled(supported);
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

    NCollection_Sequence<TDF_Label> labels = m_doc->entityLabels(LcncDocument::EntityKind::Machine);
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

    NCollection_Sequence<TDF_Label> labels = m_doc->entityLabels(LcncDocument::EntityKind::Machine);
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

    auto* machineConfig = lcnc::Kernel::current().service<lcnc::MachineConfigurationService>();
    auto editedWorldOrigin = [machineConfig](const gp_Pnt& currentWorld,
                                             const gp_Pnt& editedAxis) {
        gp_Pnt currentAxis;
        gp_Pnt world;
        if (!machineConfig
            || !machineConfig->worldToAxisCoordinates(currentWorld, &currentAxis)
            || !machineConfig->axisCoordinatesToWorld(editedAxis, &world)) {
            return editedAxis;
        }
        return world;
    };

    const gp_Pnt aCurrentWorld = kin->axisOrigin(QStringLiteral("A"));
    gp_Pnt aCurrentAxis = aCurrentWorld;
    if (machineConfig)
        (void)machineConfig->worldToAxisCoordinates(aCurrentWorld, &aCurrentAxis);
    const gp_Pnt aWorld = editedWorldOrigin(
        aCurrentWorld,
        gp_Pnt(aCurrentAxis.X(), aCurrentAxis.Y(),
               m_axisAzSpin ? m_axisAzSpin->value() : aCurrentAxis.Z()));
    emit axisOriginChanged(QStringLiteral("A"), aWorld.X(), aWorld.Y(), aWorld.Z());

    const gp_Pnt cCurrentWorld = kin->axisOrigin(QStringLiteral("C"));
    gp_Pnt cCurrentAxis = cCurrentWorld;
    if (machineConfig)
        (void)machineConfig->worldToAxisCoordinates(cCurrentWorld, &cCurrentAxis);
    const gp_Pnt cWorld = editedWorldOrigin(
        cCurrentWorld,
        gp_Pnt(m_axisCxSpin ? m_axisCxSpin->value() : cCurrentAxis.X(),
               m_axisCySpin ? m_axisCySpin->value() : cCurrentAxis.Y(),
               cCurrentAxis.Z()));
    emit axisOriginChanged(QStringLiteral("C"), cWorld.X(), cWorld.Y(), cWorld.Z());
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
