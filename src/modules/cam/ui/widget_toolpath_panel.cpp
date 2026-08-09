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
    // 中文翻译：<b>刀路参数</b>
    auto* lblTitle = new QLabel(tr("<b>Tool path parameters</b>"), this);
    mainLayout->addWidget(lblTitle);

    // ── 参数 group ─────────────────────────────────────────────────────────
    // 中文翻译：参数
    auto* paramGroup = new QGroupBox(tr("parameters"), this);
    auto* paramForm  = new QFormLayout(paramGroup);

    m_comboParameterScope = new QComboBox(paramGroup);
    // 中文翻译：全局
    m_comboParameterScope->addItem(tr("overall situation"), static_cast<int>(ParameterScope::Global));
    // 中文翻译：当前轮廓
    m_comboParameterScope->addItem(tr("current profile"), static_cast<int>(ParameterScope::CurrentContour));
    // 中文翻译：作用域:
    paramForm->addRow(tr("Scope:"), m_comboParameterScope);

    // 中文翻译：未选择轮廓
    m_labelCurrentContour = new QLabel(tr("No outline selected"), paramGroup);
    // 中文翻译：当前轮廓:
    paramForm->addRow(tr("Current profile:"), m_labelCurrentContour);

    m_spinLeadInLength = new QDoubleSpinBox(paramGroup);
    m_spinLeadInLength->setRange(0.001, 100.0);
    m_spinLeadInLength->setValue(5.0);
    m_spinLeadInLength->setDecimals(3);
    m_spinLeadInLength->setSuffix(tr(" mm"));
    m_spinLeadInLength->setSingleStep(0.001);
    // 中文翻译：引刀长度:
    paramForm->addRow(tr("Lead length:"), m_spinLeadInLength);

    m_spinDeflection = new QDoubleSpinBox(paramGroup);
    m_spinDeflection->setRange(0.001, 50.0);
    m_spinDeflection->setValue(0.1);
    m_spinDeflection->setDecimals(3);
    m_spinDeflection->setSuffix(tr(" mm"));
    m_spinDeflection->setSingleStep(0.001);
    // 中文翻译：轮廓离散采样间隔，越小越精细但计算越慢
    m_spinDeflection->setToolTip(tr("Contour discrete sampling interval, the smaller the more precise but the slower the calculation"));
    // 中文翻译：离散间隔:
    paramForm->addRow(tr("Discrete interval:"), m_spinDeflection);

    mainLayout->addWidget(paramGroup);

    // 中文翻译：工程加工模式
    auto* setupGroup = new QGroupBox(tr("Project machining mode"), this);
    auto* setupForm = new QFormLayout(setupGroup);
    m_comboMachiningMode = new QComboBox(setupGroup);
    // 中文翻译：加工模式:
    setupForm->addRow(tr("Machining mode:"), m_comboMachiningMode);
    mainLayout->addWidget(setupGroup);

    // ── 面分类 group ───────────────────────────────────────────────────────
    // 中文翻译：全局面分类
    auto* classGroup = new QGroupBox(tr("Overall situation classification"), this);
    m_classificationGroup = classGroup;
    auto* classForm  = new QFormLayout(classGroup);

    m_spinSmoothAngle = new QDoubleSpinBox(classGroup);
    m_spinSmoothAngle->setRange(0.1, 45.0);
    m_spinSmoothAngle->setValue(5.0);
    m_spinSmoothAngle->setDecimals(1);
    m_spinSmoothAngle->setSuffix(tr(" °"));
    m_spinSmoothAngle->setSingleStep(0.5);
    // 中文翻译：相邻面法线夹角小于此阈值视为光滑连接
    m_spinSmoothAngle->setToolTip(tr("The angle between the normals of adjacent surfaces is less than this threshold and is considered a smooth connection."));
    // 中文翻译：光滑阈值:
    classForm->addRow(tr("Smooth threshold:"), m_spinSmoothAngle);

    m_comboClassMode = new QComboBox(classGroup);
    // 中文翻译：自动识别
    m_comboClassMode->addItem(tr("automatic recognition"),   static_cast<int>(ExtractionStrategy::Auto));
    // 中文翻译：平面(取孔)
    m_comboClassMode->addItem(tr("Plane (hole)"), static_cast<int>(ExtractionStrategy::PlanarFaceWires));
    // 中文翻译：管材(截面)
    m_comboClassMode->addItem(tr("Pipe (section)"), static_cast<int>(ExtractionStrategy::TubeClassification));
    // 中文翻译：手动选面
    m_comboClassMode->addItem(tr("Manual face selection"),   static_cast<int>(ExtractionStrategy::ManualFaceSelection));
    m_comboClassMode->setToolTip(
        // 中文翻译：自动识别: 按机台构型与装夹姿态自动选取加工面（平板取外环+孔，管材取截面）\n
        tr("Automatic identification: Automatically select the processing surface according to the machine configuration and clamping posture (the outer ring + hole is selected for the flat plate, and the cross-section is selected for the pipe)"
           // 中文翻译：平面(取孔): 取加工面的全部 Wire（外轮廓 + 每个孔）\n
           "Plane (hole taking): Take all wires on the processing surface (outer contour + each hole)"
           // 中文翻译：管材(截面): 外表面 ∩ 截面交线（管端切割）\n
           "Pipe (section): Outer surface ∩ Section intersection (pipe end cutting)"
           // 中文翻译：手动选面: 在视图点选工件加工面
           "Manual face selection: Select the workpiece processing face in the view"));
    // 中文翻译：提取模式:
    classForm->addRow(tr("Extraction mode:"), m_comboClassMode);

    mainLayout->addWidget(classGroup);

    // ── 法线显示 group ───────────────────────────────────────────────
    // 中文翻译：法线显示
    auto* normalGroup = new QGroupBox(tr("normal display"), this);
    auto* normalForm  = new QFormLayout(normalGroup);

    // 中文翻译：显示法线
    m_checkShowNormals = new QCheckBox(tr("Show normals"), normalGroup);
    m_checkShowNormals->setChecked(false);
    normalForm->addRow(m_checkShowNormals);

    m_spinNormalStep = new QDoubleSpinBox(normalGroup);
    m_spinNormalStep->setRange(0.1, 50.0);
    m_spinNormalStep->setValue(2.0);
    m_spinNormalStep->setDecimals(2);
    m_spinNormalStep->setSuffix(tr(" mm"));
    m_spinNormalStep->setSingleStep(0.5);
    // 中文翻译：法线抽样步长，越小越密集，越大越稀疏
    m_spinNormalStep->setToolTip(tr("Normal sampling step size, the smaller it is, the denser it is, the larger it is, the sparser it is."));
    // 中文翻译：抽样步长:
    normalForm->addRow(tr("Sampling step size:"), m_spinNormalStep);

    mainLayout->addWidget(normalGroup);

    // ── 分阶段操作 group ───────────────────────────────────────────────────
    // 中文翻译：加工流程
    auto* opsGroup  = new QGroupBox(tr("Processing process"), this);
    auto* opsLayout = new QVBoxLayout(opsGroup);

    // 中文翻译：全自动执行全部阶段
    m_btnGenerate  = new QPushButton(tr("Fully automated execution of all stages"), opsGroup);
    // 中文翻译：从工件开始，依次执行分离面、提取轮廓、离散点、构造刀路和机床求解。
    m_btnGenerate->setToolTip(tr("Starting from the workpiece, separate surfaces, extracted contours, discrete points, constructed toolpaths, and machine solutions are performed in sequence."));
    // 中文翻译：1. 自动分离加工面
    m_btnSeparateFaces = new QPushButton(tr("1. Automatic separation of processing surfaces"), opsGroup);
    // 中文翻译：1. 手动选择加工面
    m_btnPickMachiningFaces = new QPushButton(tr("1. Manually select the processing surface"), opsGroup);
    // 中文翻译：应用加工面并继续
    m_btnApplyMachiningFaces = new QPushButton(tr("Apply work surface and continue"), opsGroup);
    // 中文翻译：2. 从当前加工面提取轮廓
    m_btnExtractContours = new QPushButton(tr("2. Extract the contour from the current processing surface"), opsGroup);
    // 中文翻译：3. 离散当前轮廓
    m_btnDiscretizePoints = new QPushButton(tr("3. Discretize the current contour"), opsGroup);
    // 中文翻译：4. 构造下刀线与几何刀路
    m_btnBuildToolpath = new QPushButton(tr("4. Construct the lower cutting line and geometric tool path"), opsGroup);
    // 中文翻译：5. 求解机床坐标
    m_btnSolveMachinePath = new QPushButton(tr("5. Solve the machine tool coordinates"), opsGroup);
    // 中文翻译：连续点击工件面；右键或 Esc 结束拾取，然后点击“应用加工面并继续”。
    m_btnPickMachiningFaces->setToolTip(tr("Continuously click on the workpiece surface; right-click or Esc to end picking, then click \"Apply Machining Surface and Continue\"."));
    // 中文翻译：提交当前加工面集合，并使下游轮廓、点和刀路失效。
    m_btnApplyMachiningFaces->setToolTip(tr("Commits the current set of machined surfaces and invalidates downstream contours, points, and toolpaths."));
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
    // 中文翻译：<b>机床坐标</b>
    coordinatePageLayout->addWidget(new QLabel(tr("<b>Machine coordinates</b>"), m_machineCoordinatesPage));

    // 中文翻译：当前轮廓
    auto* coordGroup  = new QGroupBox(tr("current profile"), m_machineCoordinatesPage);
    auto* coordLayout = new QVBoxLayout(coordGroup);

    // Axis columns are supplied by MachineAxisLayout after the active machine
    // mode is known; never expose a synthetic R1/R2 layout while initializing.
    // 中文翻译：轴列由当前模式的 MachineAxisLayout 动态提供，初始化时不显示虚构的 R1/R2。
    m_coordTable = new QTableWidget(0, 1, coordGroup);
    m_coordTable->setHorizontalHeaderLabels({tr("#")});
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
    connect(m_comboMachiningMode, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int index) {
                if (index >= 0) emit machiningModeChanged(
                    static_cast<lcnc::MachiningMode>(m_comboMachiningMode->itemData(index).toInt()));
            });
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
                // 中文翻译：%1（待重新计算）
                ? tr("%1 (to be recalculated)").arg(contour.name)
                : contour.name);
        } else {
            // 中文翻译：未选择轮廓
            m_labelCurrentContour->setText(tr("No outline selected"));
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

void WidgetToolpathPanel::setMachiningModes(const QList<lcnc::MachiningMode>& modes,
                                            lcnc::MachiningMode currentMode)
{
    const QSignalBlocker blocker(m_comboMachiningMode);
    m_comboMachiningMode->clear();
    for (lcnc::MachiningMode mode : modes) {
        QString name;
        switch (mode) {
        case lcnc::MachiningMode::Planar3Axis:
            // 中文翻译：三轴平面加工
            name = tr("Planar 3-axis"); break;
        case lcnc::MachiningMode::RotaryTube4Axis:
            // 中文翻译：四轴管材加工
            name = tr("Rotary tube 4-axis"); break;
        case lcnc::MachiningMode::SimultaneousTable5Axis:
            // 中文翻译：转台五轴联动
            name = tr("Simultaneous table 5-axis"); break;
        case lcnc::MachiningMode::SimultaneousHead5Axis:
            // 中文翻译：摆头五轴联动
            name = tr("Simultaneous head 5-axis"); break;
        }
        m_comboMachiningMode->addItem(name, static_cast<int>(mode));
    }
    const int index = m_comboMachiningMode->findData(static_cast<int>(currentMode));
    if (index >= 0) m_comboMachiningMode->setCurrentIndex(index);
}

void WidgetToolpathPanel::setMachineAxisLayout(const lcnc::MachineAxisLayout& layout)
{
    m_machineAxisLayout = layout;
    QStringList headers{tr("#")};
    headers.append(layout.axisNames());
    m_coordTable->setColumnCount(headers.size());
    m_coordTable->setHorizontalHeaderLabels(headers);
}

void WidgetToolpathPanel::setMachineSetupEditingEnabled(bool enabled)
{
    if (m_comboMachiningMode) m_comboMachiningMode->setEnabled(enabled);
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

    for (int i = 0; i < n; ++i) {
        const auto& mc = c.points[i].machineCoord;
        m_coordTable->setItem(i, 0, new QTableWidgetItem(QString::number(i + 1)));
        if (mc.valid) {
            for (int axisIndex = 0; axisIndex < m_machineAxisLayout.count; ++axisIndex)
                m_coordTable->setItem(i, axisIndex + 1,
                    new QTableWidgetItem(QString::number(mc.solvedPose.value(axisIndex), 'f', 3)));
        } else {
            for (int col = 1; col < m_coordTable->columnCount(); ++col)
                m_coordTable->setItem(i, col, new QTableWidgetItem(tr("--")));
        }
    }
}
