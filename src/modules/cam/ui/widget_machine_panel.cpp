#include "modules/cam/ui/widget_machine_panel.h"
#include "core/kernel/kernel.h"

#include "modules/cam/cam_module.h"
#include "core/document/lcnc_document.h"
#include "core/kinematics/machine_kinematics.h"
#include "core/document/xcaf_utils.h"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QEvent>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLayoutItem>
#include <QLineEdit>
#include <QPushButton>
#include <QSignalBlocker>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QAbstractSpinBox>

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

void clearFormLayout(QFormLayout* formLayout)
{
    if (!formLayout)
        return;

    while (formLayout->rowCount() > 0)
        formLayout->removeRow(0);
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

void addPresetOption(QComboBox* comboBox,
                     const QString& displayName,
                     const QString& presetName)
{
    if (!comboBox)
        return;
    comboBox->addItem(displayName, presetName);
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
    const QString configType = kin ? kin->configType() : QString();

    if (m_comboPreset) {
        const QSignalBlocker blocker(m_comboPreset);
        const int index = m_comboPreset->findData(configType);
        m_comboPreset->setCurrentIndex(index >= 0 ? index : 0);
    }

    refreshCalibrationSection();
    if (!doc)
        m_selectedEntries.clear();
    rebuildAssignmentSection();
    rebuildWpcSection();

    if (doc) {
        m_lblMachineName->setText(doc->name());
    } else {
        m_lblMachineName->setText(tr("—"));
    }
}

void WidgetMachinePanel::buildUi()
{
    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(4, 4, 4, 4);
    mainLayout->setSpacing(6);

    m_pages = new QTabWidget(this);
    m_pages->setDocumentMode(true);
    buildConfigPage();
    buildWorkpiecePage();
    const auto spins = findChildren<QAbstractSpinBox*>();
    for (QAbstractSpinBox* spin : spins) {
        spin->setFocusPolicy(Qt::StrongFocus);
        spin->installEventFilter(this);
    }
    mainLayout->addWidget(m_pages);
}

void WidgetMachinePanel::buildConfigPage()
{
    m_configPage = new QWidget(m_pages);
    auto* mainLayout = new QVBoxLayout(m_configPage);
    mainLayout->setContentsMargins(4, 4, 4, 4);
    mainLayout->setSpacing(8);

    auto* cfgGroup = new QGroupBox(tr("机台模型"), m_configPage);
    auto* cfgLayout = new QVBoxLayout(cfgGroup);
    cfgLayout->setSpacing(4);

    auto* infoRow = new QFormLayout;
    m_lblMachineName = new QLabel(tr("—"), cfgGroup);
    m_lblMachineName->setStyleSheet("color: #aaa;");
    m_comboPreset = new QComboBox(cfgGroup);
    addPresetOption(m_comboPreset, tr("— 请选择构型 —"), QString());
    addPresetOption(m_comboPreset, tr("XYZ 三轴（平面 / 圆管）"), QStringLiteral("XYZ"));
    addPresetOption(m_comboPreset, tr("XYZA 四轴（工件转台）"), QStringLiteral("XYZA"));
    addPresetOption(m_comboPreset, tr("AC 转台（垂直主轴）"), QStringLiteral("VERTICAL_AC_TABLE"));
    addPresetOption(m_comboPreset, tr("BC 转台（垂直主轴）"), QStringLiteral("VERTICAL_BC_TABLE"));
    addPresetOption(m_comboPreset, tr("AB 摆头"), QStringLiteral("AB_HEAD"));
    addPresetOption(m_comboPreset, tr("AC 摆头"), QStringLiteral("AC_HEAD"));
    m_editMachinePath = new QLineEdit(cfgGroup);
    m_editMachinePath->setClearButtonEnabled(true);
    m_editMachinePath->setPlaceholderText(tr("输入或粘贴机台模型路径"));
    infoRow->addRow(tr("文档:"), m_lblMachineName);
    infoRow->addRow(tr("构型:"), m_comboPreset);
    infoRow->addRow(tr("路径:"), m_editMachinePath);
    cfgLayout->addLayout(infoRow);

    auto* btnLoad = new QPushButton(QIcon(":/icons/machine.svg"), tr("加载机台模型..."), cfgGroup);
    auto* btnCompress = new QPushButton(QIcon(":/icons/machine.svg"), tr("压缩机台模型"), cfgGroup);
    auto* btnUnload = new QPushButton(QIcon(":/icons/machine.svg"), tr("卸载机台"), cfgGroup);
    auto* btnExport = new QPushButton(QIcon(":/icons/export.svg"), tr("导出机台模型..."), cfgGroup);

    cfgLayout->addWidget(btnLoad);
    cfgLayout->addWidget(btnCompress);
    cfgLayout->addWidget(btnUnload);
    cfgLayout->addWidget(btnExport);
    mainLayout->addWidget(cfgGroup);

    connect(m_comboPreset, &QComboBox::currentIndexChanged, this,
            [this](int index) {
                if (!m_comboPreset)
                    return;

                const QString presetName = m_comboPreset->itemData(index).toString();
                emit machinePresetChanged(presetName);
            });
    connect(btnLoad, &QPushButton::clicked, this, &WidgetMachinePanel::loadMachineRequested);
    connect(btnCompress, &QPushButton::clicked, this, &WidgetMachinePanel::compressMachineRequested);
    connect(btnUnload, &QPushButton::clicked, this, &WidgetMachinePanel::unloadMachineRequested);
    connect(btnExport, &QPushButton::clicked, this, &WidgetMachinePanel::exportMachineRequested);
    connect(m_editMachinePath, &QLineEdit::editingFinished, this, [this] {
        emit machineModelPathChanged(m_editMachinePath ? m_editMachinePath->text().trimmed() : QString());
    });
    connect(m_editMachinePath, &QLineEdit::returnPressed, this, [this] {
        emit machineModelPathChanged(m_editMachinePath ? m_editMachinePath->text().trimmed() : QString());
    });

    m_assignGroup = new QGroupBox(tr("标记所选部件"), m_configPage);
    auto* assignLayout = new QVBoxLayout(m_assignGroup);
    assignLayout->setContentsMargins(6, 6, 6, 6);
    assignLayout->setSpacing(6);
    m_lblAssignSelection = new QLabel(tr("请在机台视图或模型树中选择机台部件。"), m_assignGroup);
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

    auto* calibrationGroup = new QGroupBox(tr("坐标系转换"), m_configPage);
    auto* calibrationLayout = new QVBoxLayout(calibrationGroup);
    calibrationLayout->setContentsMargins(6, 6, 6, 6);
    calibrationLayout->setSpacing(8);

    // 唯一入口：打开三段式标定向导（需求 3：移除旧的轴心/AC 中心/切割头独立控件）
    m_btnOpenCalibrationWizard = new QPushButton(
        tr("打开标定向导..."), calibrationGroup);
    m_btnOpenCalibrationWizard->setToolTip(
        tr("依次拾取 A 轴、C 轴参考面与切割头下端面，填入物理 AC 中心与 A/C 角度，"
           "一次性完成机台坐标系标定，并自动持久化到 cam.toml 与机台 STEP。"));
    calibrationLayout->addWidget(m_btnOpenCalibrationWizard);
    connect(m_btnOpenCalibrationWizard, &QPushButton::clicked,
            this, &WidgetMachinePanel::axisCalibrationWizardRequested);

    mainLayout->addWidget(calibrationGroup, 1);

    mainLayout->addStretch();
    m_pages->addTab(m_configPage, tr("机台模型"));
}

void WidgetMachinePanel::buildWorkpiecePage()
{
    m_workpiecePage = new QWidget(m_pages);
    auto* mainLayout = new QVBoxLayout(m_workpiecePage);
    mainLayout->setContentsMargins(4, 4, 4, 4);
    mainLayout->setSpacing(8);

    auto* infoLabel = new QLabel(
        tr("挂载后的工件会按工件包围盒中心对齐到安装位置坐标。存在转台构型时，可直接把安装位置 X/Y 对齐到旋转中心。"),
        m_workpiecePage);
    infoLabel->setWordWrap(true);
    infoLabel->setStyleSheet("color: #888; font-size: 11px;");
    mainLayout->addWidget(infoLabel);

    m_wpcGroup = new QGroupBox(tr("工件挂载"), m_workpiecePage);
    auto* mountLayout = new QVBoxLayout(m_wpcGroup);
    mountLayout->setContentsMargins(6, 6, 6, 6);
    mountLayout->setSpacing(6);
    m_lblWorkpieceStatus = new QLabel(tr("暂无工件挂载"), m_wpcGroup);
    m_lblWorkpieceStatus->setWordWrap(true);
    m_lblWorkpieceStatus->setStyleSheet("color: gray; font-size: 11px;");
    m_btnMountWorkpiece = new QPushButton(QIcon(":/icons/workpiece.svg"), tr("挂载工件..."), m_wpcGroup);
    mountLayout->addWidget(m_lblWorkpieceStatus);
    mountLayout->addWidget(m_btnMountWorkpiece);
    mainLayout->addWidget(m_wpcGroup);

    m_installGroup = new QGroupBox(tr("工件安装位置"), m_workpiecePage);
    auto* installLayout = new QFormLayout(m_installGroup);
    installLayout->setContentsMargins(6, 6, 6, 6);
    installLayout->setSpacing(6);
    m_wpcInstallX = createMillimeterSpin(m_installGroup);
    m_wpcInstallY = createMillimeterSpin(m_installGroup);
    m_wpcInstallZ = createMillimeterSpin(m_installGroup);
    m_btnAlignRotationCenter = new QPushButton(tr("对齐旋转中心"), m_installGroup);
    installLayout->addRow(tr("安装 X:"), m_wpcInstallX);
    installLayout->addRow(tr("安装 Y:"), m_wpcInstallY);
    installLayout->addRow(tr("安装 Z:"), m_wpcInstallZ);
    installLayout->addRow(m_btnAlignRotationCenter);
    mainLayout->addWidget(m_installGroup);

    connect(m_btnMountWorkpiece, &QPushButton::clicked,
        this, &WidgetMachinePanel::mountWorkpieceRequested);
    connect(m_btnAlignRotationCenter, &QPushButton::clicked,
        this, &WidgetMachinePanel::alignWorkpieceRotationCenterRequested);
    connect(m_wpcInstallX, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
        this, &WidgetMachinePanel::onWorkpieceInstallPositionChanged);
    connect(m_wpcInstallY, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
        this, &WidgetMachinePanel::onWorkpieceInstallPositionChanged);
    connect(m_wpcInstallZ, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
        this, &WidgetMachinePanel::onWorkpieceInstallPositionChanged);

    mainLayout->addStretch();
    m_pages->addTab(m_workpiecePage, tr("工件配置"));
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
                ? tr("适用于 AC 转台：A 轴参考面写入 Y/Z，C 轴参考面写入 X。整机对齐只做平移。切割头模型点与物理点可独立录入和对齐。")
                : tr("当前页用于轴心与切割头位置配置。AC 轴心快填和 AC 中心对齐仅在 AC 转台构型下显示。"));
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
            m_lblCurrentAcCenter->setText(tr("当前构型暂不支持 AC 中心对齐。"));
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
        m_lblHeadModelPoint->setText(tr("当前切割头模型点: %1")
                                         .arg(formatPointText(cutterHeadModel)));

    if (!m_lblPickStatus)
        return;

    if (!m_pendingCalibrationAxis.isEmpty()) {
        const QString targetText = (m_pendingCalibrationAxis == QStringLiteral("CUTTER_HEAD"))
            ? tr("切割头对齐")
            : tr("%1 轴参考平面").arg(m_pendingCalibrationAxis);
        m_lblPickStatus->setText(
            tr("正在拾取 %1：左键确认，右键或 ESC 取消。")
                .arg(targetText));
        m_lblPickStatus->setStyleSheet("color: #c98512; font-size: 11px; font-weight: bold;");
        return;
    }

    if (!supported) {
        m_lblPickStatus->setText(tr("左键选择参考平面，右键或 ESC 取消。切割头拾取始终可用，AC 轴心快填仅在 AC 转台构型下显示。"));
        m_lblPickStatus->setStyleSheet("color: #888; font-size: 11px;");
        return;
    }

    m_lblPickStatus->setText(tr("左键选择参考平面，右键或 ESC 取消。"));
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
        m_lblAssignSelection->setText(tr("加载机台模型后，可为选中机台部件直接标记所属轴系。"));
        m_lblAssignSelection->setStyleSheet("color: #888; font-size: 11px;");
        return;
    }

    if (m_selectedEntries.isEmpty()) {
        m_lblAssignSelection->setText(tr("请在机台视图或模型树中选择机台部件。"));
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
        summaryText = tr("当前选中: %1").arg(selectedNames.first());
    } else if (selectedNames.size() <= 3) {
        summaryText = tr("当前选中 %1 个部件: %2")
            .arg(selectedNames.size())
            .arg(selectedNames.join(tr("、")));
    } else {
        summaryText = tr("当前选中 %1 个机台部件，可直接点击下方按钮归轴。")
            .arg(selectedNames.size());
    }
    m_lblAssignSelection->setText(summaryText);
    m_lblAssignSelection->setStyleSheet("color: #2f5f9f; font-size: 11px;");

    int index = 0;
    for (const MachineAxisDef& axis : kin->axes()) {
        const QString axisName = axis.name;
        QString buttonText;
        if (axisName == QStringLiteral("BASE"))
            buttonText = tr("标记为 BASE");
        else
            buttonText = tr("标记为 %1").arg(axisName);

        auto* button = new QPushButton(buttonText, m_assignGroup);
        button->setToolTip(tr("将当前选中的机台部件归到 %1 轴").arg(axisName));
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

    auto* clearButton = new QPushButton(tr("解除所选归轴"), m_assignGroup);
    clearButton->setToolTip(tr("清除当前选中机台部件已有的轴系归属"));
    connect(clearButton, &QPushButton::clicked, this, [this] {
        for (const QString& entry : m_selectedEntries)
            lcnc::Kernel::current().service<CamModule>()->unassignShape(entry);
    });
    m_assignGrid->addWidget(clearButton, index / 2, index % 2);
}

void WidgetMachinePanel::rebuildWpcSection()
{
    const MachineKinematics* kin = m_doc ? m_doc->machineKinematics() : nullptr;
    const bool hasMachineEntities = m_doc
        && m_doc->entityLabels(LcncDocument::EntityKind::Machine).Length() > 0;
    const bool hasPreset = kin && !kin->configType().isEmpty();
    const QString configType = kin ? kin->configType() : QString();
    const bool showRotationButton = configType == QStringLiteral("VERTICAL_AC_TABLE")
        || configType == QStringLiteral("VERTICAL_BC_TABLE")
        || configType == QStringLiteral("XYZA");

    if (m_btnMountWorkpiece)
        m_btnMountWorkpiece->setEnabled(hasMachineEntities && hasPreset);

    if (m_lblWorkpieceStatus && kin && !kin->wpcMounts().isEmpty()) {
        TDF_LabelSequence wpcLabels = m_doc->entityLabels(LcncDocument::EntityKind::Workpiece);
        QMap<QString, QString> workpieceNames;
        for (int i = 1; i <= wpcLabels.Length(); ++i) {
            const TDF_Label label = wpcLabels.Value(i);
            workpieceNames.insert(XcafUtils::entry(label), XcafUtils::name(label));
        }

        QStringList mountLines;
        for (auto it = kin->wpcMounts().cbegin(); it != kin->wpcMounts().cend(); ++it) {
            mountLines << tr("<b>%1</b> → %2 轴")
                              .arg(workpieceNames.value(it.key(), it.key()), it.value());
        }

        m_lblWorkpieceStatus->setText(mountLines.join(QStringLiteral("<br/>")));
        m_lblWorkpieceStatus->setStyleSheet("color: #3a8; font-size: 11px;");
    } else if (m_lblWorkpieceStatus) {
        if (!hasMachineEntities) {
            m_lblWorkpieceStatus->setText(tr("请先加载机台模型，再从这里挂载工件。"));
        } else if (!hasPreset) {
            m_lblWorkpieceStatus->setText(tr("请先在“机台模型”页选择机台构型。"));
        } else {
            m_lblWorkpieceStatus->setText(tr("暂无工件挂载"));
        }
        m_lblWorkpieceStatus->setStyleSheet("color: gray; font-size: 11px;");
    }

    if (m_wpcInstallX && m_wpcInstallY && m_wpcInstallZ) {
        const gp_Pnt installPosition = lcnc::Kernel::current().service<CamModule>()->workpieceInstallPosition();
        {
            const QSignalBlocker blockerX(m_wpcInstallX);
            m_wpcInstallX->setValue(installPosition.X());
        }
        {
            const QSignalBlocker blockerY(m_wpcInstallY);
            m_wpcInstallY->setValue(installPosition.Y());
        }
        {
            const QSignalBlocker blockerZ(m_wpcInstallZ);
            m_wpcInstallZ->setValue(installPosition.Z());
        }

        const bool enabled = hasPreset;
        m_wpcInstallX->setEnabled(enabled);
        m_wpcInstallY->setEnabled(enabled);
        m_wpcInstallZ->setEnabled(enabled);
    }

    if (m_installGroup)
        m_installGroup->setEnabled(hasPreset);

    if (m_btnAlignRotationCenter) {
        m_btnAlignRotationCenter->setVisible(showRotationButton);
        m_btnAlignRotationCenter->setEnabled(showRotationButton && hasPreset);
        if (configType == QStringLiteral("VERTICAL_AC_TABLE")) {
            m_btnAlignRotationCenter->setToolTip(tr("将安装位置 X/Y 回填为 AC 旋转中心。"));
        } else if (configType == QStringLiteral("VERTICAL_BC_TABLE")) {
            m_btnAlignRotationCenter->setToolTip(tr("将安装位置 X/Y 回填为 BC 旋转中心。"));
        } else if (configType == QStringLiteral("XYZA")) {
            m_btnAlignRotationCenter->setToolTip(tr("将安装位置 X/Y 回填为 A 转台中心。"));
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

void WidgetMachinePanel::setMachineModelPath(const QString& path)
{
    if (!m_editMachinePath)
        return;

    const QString normalized = path.trimmed();
    if (m_editMachinePath->text() == normalized)
        return;

    const QSignalBlocker blocker(m_editMachinePath);
    m_editMachinePath->setText(normalized);
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

void WidgetMachinePanel::onWorkpieceInstallPositionChanged()
{
    emit workpieceInstallPositionChanged(
        m_wpcInstallX ? m_wpcInstallX->value() : 0.0,
        m_wpcInstallY ? m_wpcInstallY->value() : 0.0,
        m_wpcInstallZ ? m_wpcInstallZ->value() : 0.0);
}