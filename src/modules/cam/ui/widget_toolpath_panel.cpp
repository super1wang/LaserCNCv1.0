#include "modules/cam/ui/widget_toolpath_panel.h"
#include "core/algorithms/cam/laser_toolpath.h"

#include <QVBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QPushButton>
#include <QTableWidget>
#include <QHeaderView>
#include <QLabel>
#include <QSignalBlocker>

WidgetToolpathPanel::WidgetToolpathPanel(QWidget* parent)
    : QWidget(parent)
{
    buildUi();
}

void WidgetToolpathPanel::buildUi()
{
    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(6, 6, 6, 6);
    mainLayout->setSpacing(8);

    // ── Title ──────────────────────────────────────────────────────────────
    auto* lblTitle = new QLabel(tr("<b>刀路参数</b>"), this);
    mainLayout->addWidget(lblTitle);

    // ── 参数 group ─────────────────────────────────────────────────────────
    auto* paramGroup = new QGroupBox(tr("参数"), this);
    auto* paramForm  = new QFormLayout(paramGroup);

    m_comboParameterScope = new QComboBox(paramGroup);
    m_comboParameterScope->addItem(tr("全局"), static_cast<int>(ParameterScope::Global));
    m_comboParameterScope->addItem(tr("当前轮廓"), static_cast<int>(ParameterScope::CurrentContour));
    paramForm->addRow(tr("作用域:"), m_comboParameterScope);

    m_labelCurrentContour = new QLabel(tr("未选择轮廓"), paramGroup);
    paramForm->addRow(tr("当前轮廓:"), m_labelCurrentContour);

    m_spinLeadInLength = new QDoubleSpinBox(paramGroup);
    m_spinLeadInLength->setRange(0.001, 100.0);
    m_spinLeadInLength->setValue(5.0);
    m_spinLeadInLength->setDecimals(3);
    m_spinLeadInLength->setSuffix(tr(" mm"));
    m_spinLeadInLength->setSingleStep(0.001);
    paramForm->addRow(tr("引刀长度:"), m_spinLeadInLength);

    m_spinDeflection = new QDoubleSpinBox(paramGroup);
    m_spinDeflection->setRange(0.001, 50.0);
    m_spinDeflection->setValue(0.1);
    m_spinDeflection->setDecimals(3);
    m_spinDeflection->setSuffix(tr(" mm"));
    m_spinDeflection->setSingleStep(0.001);
    m_spinDeflection->setToolTip(tr("轮廓离散采样间隔，越小越精细但计算越慢"));
    paramForm->addRow(tr("离散间隔:"), m_spinDeflection);

    mainLayout->addWidget(paramGroup);

    // ── 面分类 group ───────────────────────────────────────────────────────
    auto* classGroup = new QGroupBox(tr("全局面分类"), this);
    m_classificationGroup = classGroup;
    auto* classForm  = new QFormLayout(classGroup);

    m_spinSmoothAngle = new QDoubleSpinBox(classGroup);
    m_spinSmoothAngle->setRange(0.1, 45.0);
    m_spinSmoothAngle->setValue(5.0);
    m_spinSmoothAngle->setDecimals(1);
    m_spinSmoothAngle->setSuffix(tr(" °"));
    m_spinSmoothAngle->setSingleStep(0.5);
    m_spinSmoothAngle->setToolTip(tr("相邻面法线夹角小于此阈值视为光滑连接"));
    classForm->addRow(tr("光滑阈值:"), m_spinSmoothAngle);

    m_comboClassMode = new QComboBox(classGroup);
    m_comboClassMode->addItem(tr("自动识别"),   static_cast<int>(ExtractionStrategy::Auto));
    m_comboClassMode->addItem(tr("平面(取孔)"), static_cast<int>(ExtractionStrategy::PlanarFaceWires));
    m_comboClassMode->addItem(tr("管材(截面)"), static_cast<int>(ExtractionStrategy::TubeClassification));
    m_comboClassMode->addItem(tr("手动选面"),   static_cast<int>(ExtractionStrategy::ManualFaceSelection));
    m_comboClassMode->setToolTip(
        tr("自动识别: 按机台构型与装夹姿态自动选取加工面（平板取外环+孔，管材取截面）\n"
           "平面(取孔): 取加工面的全部 Wire（外轮廓 + 每个孔）\n"
           "管材(截面): 外表面 ∩ 截面交线（管端切割）\n"
           "手动选面: 在视图点选工件加工面"));
    classForm->addRow(tr("提取模式:"), m_comboClassMode);

    mainLayout->addWidget(classGroup);

    // ── 法线显示 group ───────────────────────────────────────────────
    auto* normalGroup = new QGroupBox(tr("法线显示"), this);
    auto* normalForm  = new QFormLayout(normalGroup);

    m_checkShowNormals = new QCheckBox(tr("显示法线"), normalGroup);
    m_checkShowNormals->setChecked(false);
    normalForm->addRow(m_checkShowNormals);

    m_spinNormalStep = new QDoubleSpinBox(normalGroup);
    m_spinNormalStep->setRange(0.1, 50.0);
    m_spinNormalStep->setValue(2.0);
    m_spinNormalStep->setDecimals(2);
    m_spinNormalStep->setSuffix(tr(" mm"));
    m_spinNormalStep->setSingleStep(0.5);
    m_spinNormalStep->setToolTip(tr("法线抽样步长，越小越密集，越大越稀疏"));
    normalForm->addRow(tr("抽样步长:"), m_spinNormalStep);

    mainLayout->addWidget(normalGroup);

    // ── 分阶段操作 group ───────────────────────────────────────────────────
    auto* opsGroup  = new QGroupBox(tr("加工流程"), this);
    auto* opsLayout = new QVBoxLayout(opsGroup);

    m_btnGenerate  = new QPushButton(tr("全自动执行全部阶段"), opsGroup);
    m_btnGenerate->setToolTip(tr("从工件开始，依次执行分离面、提取轮廓、离散点、构造刀路和机床求解。"));
    m_btnSeparateFaces = new QPushButton(tr("1. 自动分离加工面"), opsGroup);
    m_btnPickMachiningFaces = new QPushButton(tr("1. 手动选择加工面"), opsGroup);
    m_btnApplyMachiningFaces = new QPushButton(tr("应用加工面并继续"), opsGroup);
    m_btnExtractContours = new QPushButton(tr("2. 从当前加工面提取轮廓"), opsGroup);
    m_btnDiscretizePoints = new QPushButton(tr("3. 离散当前轮廓"), opsGroup);
    m_btnBuildToolpath = new QPushButton(tr("4. 构造下刀线与几何刀路"), opsGroup);
    m_btnSolveMachinePath = new QPushButton(tr("5. 求解机床坐标"), opsGroup);
    m_btnPickMachiningFaces->setToolTip(tr("连续点击工件面；右键或 Esc 结束拾取，然后点击“应用加工面并继续”。"));
    m_btnApplyMachiningFaces->setToolTip(tr("提交当前加工面集合，并使下游轮廓、点和刀路失效。"));
    opsLayout->addWidget(m_btnGenerate);
    opsLayout->addWidget(m_btnSeparateFaces);
    opsLayout->addWidget(m_btnPickMachiningFaces);
    opsLayout->addWidget(m_btnApplyMachiningFaces);
    opsLayout->addWidget(m_btnExtractContours);
    opsLayout->addWidget(m_btnDiscretizePoints);
    opsLayout->addWidget(m_btnBuildToolpath);
    opsLayout->addWidget(m_btnSolveMachinePath);

    mainLayout->addWidget(opsGroup);

    // ── 独立的机床坐标 tab ────────────────────────────────────────────────
    m_machineCoordinatesPage = new QWidget(this);
    auto* coordinatePageLayout = new QVBoxLayout(m_machineCoordinatesPage);
    coordinatePageLayout->setContentsMargins(6, 6, 6, 6);
    coordinatePageLayout->setSpacing(8);
    coordinatePageLayout->addWidget(new QLabel(tr("<b>机床坐标</b>"), m_machineCoordinatesPage));

    auto* coordGroup  = new QGroupBox(tr("当前轮廓"), m_machineCoordinatesPage);
    auto* coordLayout = new QVBoxLayout(coordGroup);

    m_coordTable = new QTableWidget(0, 6, coordGroup);
    m_coordTable->setHorizontalHeaderLabels({tr("#"), tr("X"), tr("Y"), tr("Z"), tr("R1"), tr("R2")});
    m_coordTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    m_coordTable->verticalHeader()->setVisible(false);
    m_coordTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_coordTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_coordTable->setSelectionMode(QAbstractItemView::SingleSelection);
    m_coordTable->setAlternatingRowColors(true);
    coordLayout->addWidget(m_coordTable);

    coordinatePageLayout->addWidget(coordGroup);
    coordinatePageLayout->addStretch(1);

    // Stretch at bottom
    mainLayout->addStretch(1);

    // ── Signal connections ─────────────────────────────────────────────────
    connect(m_btnGenerate,  &QPushButton::clicked,
            this, &WidgetToolpathPanel::generateRequested);
    connect(m_btnSeparateFaces, &QPushButton::clicked,
            this, &WidgetToolpathPanel::separateFacesRequested);
    connect(m_btnPickMachiningFaces, &QPushButton::clicked,
            this, &WidgetToolpathPanel::pickMachiningFacesRequested);
    connect(m_btnApplyMachiningFaces, &QPushButton::clicked,
            this, &WidgetToolpathPanel::applyMachiningFacesRequested);
    connect(m_btnExtractContours, &QPushButton::clicked,
            this, &WidgetToolpathPanel::extractContoursRequested);
    connect(m_btnDiscretizePoints, &QPushButton::clicked,
            this, &WidgetToolpathPanel::discretizePointsRequested);
    connect(m_btnBuildToolpath, &QPushButton::clicked,
            this, &WidgetToolpathPanel::buildToolpathRequested);
    connect(m_btnSolveMachinePath, &QPushButton::clicked,
            this, &WidgetToolpathPanel::solveMachinePathRequested);

    connect(m_spinLeadInLength, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &WidgetToolpathPanel::leadInLengthChanged);
    connect(m_spinDeflection,   QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &WidgetToolpathPanel::discretizationIntervalChanged);
    connect(m_comboParameterScope, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int) {
                refreshParameterEditors();
                emit parameterScopeChanged(parameterScope() == ParameterScope::CurrentContour);
            });

    connect(m_spinSmoothAngle,  QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &WidgetToolpathPanel::smoothAngleChanged);

    connect(m_comboClassMode, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int index) {
                int strategy = m_comboClassMode->itemData(index).toInt();
                emit extractionStrategyChanged(strategy);
            });
        connect(m_checkShowNormals, &QCheckBox::toggled,
            this, &WidgetToolpathPanel::showNormalsToggled);
        connect(m_spinNormalStep, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &WidgetToolpathPanel::normalSampleStepChanged);
}

void WidgetToolpathPanel::setToolpath(LaserToolpath* tp)
{
    m_toolpath = tp;
    m_activeContourIndex = -1;
    showContourCoordinates(-1);
    refreshParameterEditors();
}

WidgetToolpathPanel::ParameterScope WidgetToolpathPanel::parameterScope() const
{
    if (!m_comboParameterScope)
        return ParameterScope::Global;
    return static_cast<ParameterScope>(m_comboParameterScope->currentData().toInt());
}

void WidgetToolpathPanel::setActiveContour(int contourIndex)
{
    m_activeContourIndex = m_toolpath && contourIndex >= 0
        && contourIndex < m_toolpath->contourCount() ? contourIndex : -1;
    refreshParameterEditors();
}

void WidgetToolpathPanel::refreshParameterEditors()
{
    const bool currentScope = parameterScope() == ParameterScope::CurrentContour;
    const bool hasContour = m_toolpath && m_activeContourIndex >= 0
        && m_activeContourIndex < m_toolpath->contourCount();
    if (m_labelCurrentContour) {
        if (hasContour) {
            const LaserContour& contour = m_toolpath->contour(m_activeContourIndex);
            m_labelCurrentContour->setText(contour.needsRecalculation
                ? tr("%1（待重新计算）").arg(contour.name)
                : contour.name);
        } else {
            m_labelCurrentContour->setText(tr("未选择轮廓"));
        }
    }
    if (currentScope && hasContour) {
        const auto& params = m_toolpath->contour(m_activeContourIndex).pendingParams;
        setLeadInLength(params.leadInLength);
        setDiscretizationInterval(params.deflection);
    } else if (!currentScope && m_toolpath) {
        setLeadInLength(m_toolpath->globalLeadInLength());
    }
    if (m_spinLeadInLength) m_spinLeadInLength->setEnabled(!currentScope || hasContour);
    if (m_spinDeflection) m_spinDeflection->setEnabled(!currentScope || hasContour);
    if (m_classificationGroup) m_classificationGroup->setEnabled(!currentScope);
}

QWidget* WidgetToolpathPanel::machineCoordinatesPage() const
{
    return m_machineCoordinatesPage;
}

void WidgetToolpathPanel::setLeadInLength(double mm)
{
    if (!m_spinLeadInLength)
        return;

    if (qFuzzyCompare(m_spinLeadInLength->value() + 1.0, mm + 1.0))
        return;

    const QSignalBlocker blocker(m_spinLeadInLength);
    m_spinLeadInLength->setValue(mm);
}

void WidgetToolpathPanel::setDiscretizationInterval(double mm)
{
    if (!m_spinDeflection)
        return;

    if (qFuzzyCompare(m_spinDeflection->value() + 1.0, mm + 1.0))
        return;

    const QSignalBlocker blocker(m_spinDeflection);
    m_spinDeflection->setValue(mm);
}

void WidgetToolpathPanel::setSmoothAngle(double deg)
{
    if (!m_spinSmoothAngle)
        return;

    if (qFuzzyCompare(m_spinSmoothAngle->value() + 1.0, deg + 1.0))
        return;

    const QSignalBlocker blocker(m_spinSmoothAngle);
    m_spinSmoothAngle->setValue(deg);
}

void WidgetToolpathPanel::setExtractionStrategy(int strategy)
{
    if (!m_comboClassMode)
        return;

    const int index = m_comboClassMode->findData(strategy);
    if (index < 0 || m_comboClassMode->currentIndex() == index)
        return;

    const QSignalBlocker blocker(m_comboClassMode);
    m_comboClassMode->setCurrentIndex(index);
}

void WidgetToolpathPanel::setShowNormals(bool on)
{
    if (!m_checkShowNormals || m_checkShowNormals->isChecked() == on)
        return;

    const QSignalBlocker blocker(m_checkShowNormals);
    m_checkShowNormals->setChecked(on);
}

void WidgetToolpathPanel::setNormalSampleStep(double mm)
{
    if (!m_spinNormalStep)
        return;

    if (qFuzzyCompare(m_spinNormalStep->value() + 1.0, mm + 1.0))
        return;

    const QSignalBlocker blocker(m_spinNormalStep);
    m_spinNormalStep->setValue(mm);
}

double WidgetToolpathPanel::leadInLength() const
{
    return m_spinLeadInLength ? m_spinLeadInLength->value() : 5.0;
}

double WidgetToolpathPanel::discretizationInterval() const
{
    return m_spinDeflection ? m_spinDeflection->value() : 0.1;
}

double WidgetToolpathPanel::smoothAngle() const
{
    return m_spinSmoothAngle ? m_spinSmoothAngle->value() : 5.0;
}

int WidgetToolpathPanel::extractionStrategy() const
{
    if (!m_comboClassMode) return static_cast<int>(ExtractionStrategy::Auto);
    return m_comboClassMode->currentData().toInt();
}

void WidgetToolpathPanel::showContourCoordinates(int contourIndex)
{
    m_coordTable->setRowCount(0);
    setActiveContour(contourIndex);
    if (!m_toolpath) return;
    if (contourIndex < 0 || contourIndex >= m_toolpath->contourCount()) return;

    const LaserContour& c = m_toolpath->contour(contourIndex);
    const int n = static_cast<int>(c.points.size());
    m_coordTable->setRowCount(n);

    // Update R1/R2 header labels from first valid point
    if (n > 0 && c.points[0].machineCoord.valid) {
        const auto& mc = c.points[0].machineCoord;
        QStringList headers = {tr("#"), tr("X"), tr("Y"), tr("Z"),
                               mc.r1Name.isEmpty() ? tr("R1") : mc.r1Name,
                               mc.r2Name.isEmpty() ? tr("R2") : mc.r2Name};
        m_coordTable->setHorizontalHeaderLabels(headers);
    }

    for (int i = 0; i < n; ++i) {
        const auto& mc = c.points[i].machineCoord;
        m_coordTable->setItem(i, 0, new QTableWidgetItem(QString::number(i + 1)));
        if (mc.valid) {
            m_coordTable->setItem(i, 1, new QTableWidgetItem(QString::number(mc.x, 'f', 3)));
            m_coordTable->setItem(i, 2, new QTableWidgetItem(QString::number(mc.y, 'f', 3)));
            m_coordTable->setItem(i, 3, new QTableWidgetItem(QString::number(mc.z, 'f', 3)));
            m_coordTable->setItem(i, 4, new QTableWidgetItem(QString::number(mc.r1, 'f', 3)));
            m_coordTable->setItem(i, 5, new QTableWidgetItem(QString::number(mc.r2, 'f', 3)));
        } else {
            for (int col = 1; col <= 5; ++col)
                m_coordTable->setItem(i, col, new QTableWidgetItem(tr("--")));
        }
    }
}
