#include "app/dialog/dialog_options.h"

#include "core/kernel/kernel.h"
#include "core/logging/logger.h"
#include "core/project/lcnc_project_manager.h"
#include "modules/cam/cam_module.h"
#include "view/gui_application.h"
#include "view/rendering_manager.h"

#include <QApplication>
#include <QAbstractItemView>
#include <QCheckBox>
#include <QColorDialog>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QEvent>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>
#include <QHeaderView>
#include <QLineEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSplitter>
#include <QSpinBox>
#include <QStackedWidget>
#include <QTableWidget>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <QWidget>
#include <QMessageBox>
#include <QAbstractSpinBox>

namespace lcnc {

namespace {
constexpr const char* kAxes[] = {"BASE", "X", "Y", "Z", "A", "B", "C"};

QStringList machinePresetNames()
{
    return {
        QStringLiteral("XYZ"),
        QStringLiteral("XYZA"),
        QStringLiteral("VERTICAL_AC_TABLE"),
        QStringLiteral("VERTICAL_BC_TABLE"),
        QStringLiteral("AB_HEAD"),
        QStringLiteral("AC_HEAD")
    };
}

QString machinePresetText(const QString& preset)
{
    if (preset == QStringLiteral("XYZ"))
        return QStringLiteral("三轴 XYZ");
    if (preset == QStringLiteral("XYZA"))
        return QStringLiteral("四轴 XYZA");
    if (preset == QStringLiteral("VERTICAL_AC_TABLE"))
        return QStringLiteral("立式 AC 转台");
    if (preset == QStringLiteral("VERTICAL_BC_TABLE"))
        return QStringLiteral("立式 BC 转台");
    if (preset == QStringLiteral("AB_HEAD"))
        return QStringLiteral("AB 摆头");
    if (preset == QStringLiteral("AC_HEAD"))
        return QStringLiteral("AC 摆头");
    return preset;
}

gp_Dir defaultMachineAxisDirection(const QString& name, MachineAxisDef::MotionType type)
{
    const QString axisName = name.trimmed().toUpper();
    if (type == MachineAxisDef::Rotary) {
        if (axisName == QStringLiteral("A"))
            return gp_Dir(1, 0, 0);
        if (axisName == QStringLiteral("B"))
            return gp_Dir(0, 1, 0);
        return gp_Dir(0, 0, 1);
    }
    if (axisName == QStringLiteral("X"))
        return gp_Dir(1, 0, 0);
    if (axisName == QStringLiteral("Y"))
        return gp_Dir(0, 1, 0);
    return gp_Dir(0, 0, 1);
}

QVector<MachineAxisRuntimeConfig> machineConfigsForPreset(const QString& preset)
{
    MachineKinematics kinematics;
    kinematics.loadPreset(preset);
    QVector<MachineAxisRuntimeConfig> configs;
    int index = 0;
    for (const MachineAxisDef& axis : kinematics.axes()) {
        if (axis.name == QStringLiteral("BASE"))
            continue;
        MachineAxisRuntimeConfig config;
        config.axis = axis;
        config.controllerIndex = index;
        config.homeIndex = index;
        configs.append(config);
        ++index;
    }
    return configs;
}

QString algorithmTextForAxes(const QString& preset, const QList<MachineAxisDef>& axes)
{
    int rotaryCount = 0;
    for (const MachineAxisDef& axis : axes) {
        if (axis.motionType == MachineAxisDef::Rotary)
            ++rotaryCount;
    }
    if (rotaryCount <= 0)
        return machineToolpathAlgorithmName(MachineToolpathAlgorithm::ThreeAxis);
    if (preset.toUpper().contains(QStringLiteral("HEAD")))
        return machineToolpathAlgorithmName(MachineToolpathAlgorithm::FiveAxisHead);
    return machineToolpathAlgorithmName(MachineToolpathAlgorithm::FiveAxisTable);
}

QTableWidgetItem* machineAxisTableItem(const QString& text, bool editable = true)
{
    auto* item = new QTableWidgetItem(text);
    if (!editable)
        item->setFlags(item->flags() & ~Qt::ItemIsEditable);
    return item;
}

QDoubleSpinBox* machineCoordinateSpin(QWidget* parent)
{
    auto* spin = new QDoubleSpinBox(parent);
    spin->setFocusPolicy(Qt::StrongFocus);
    spin->setRange(-99999.0, 99999.0);
    spin->setDecimals(3);
    spin->setSingleStep(1.0);
    spin->setSuffix(QStringLiteral(" mm"));
    spin->setMinimumWidth(120);
    return spin;
}

QStringList rotaryAxisNames(const QList<MachineAxisDef>& axes)
{
    QStringList names;
    for (const MachineAxisDef& axis : axes) {
        if (axis.motionType == MachineAxisDef::Rotary
            && !axis.name.trimmed().isEmpty()
            && !names.contains(axis.name.trimmed().toUpper())) {
            names.append(axis.name.trimmed().toUpper());
        }
    }
    return names;
}

gp_Pnt rotationCenterFromAxes(const QList<MachineAxisDef>& axes, const QString& preset)
{
    auto originOf = [&axes](const QString& axisName, gp_Pnt* out) {
        for (const MachineAxisDef& axis : axes) {
            if (axis.name == axisName && axis.motionType == MachineAxisDef::Rotary) {
                if (out)
                    *out = axis.origin;
                return true;
            }
        }
        return false;
    };

    const QString normalizedPreset = preset.trimmed().toUpper();
    gp_Pnt first;
    for (const MachineAxisDef& axis : axes) {
        if (axis.motionType == MachineAxisDef::Rotary) {
            first = axis.origin;
            break;
        }
    }

    gp_Pnt a, b, c;
    if (normalizedPreset == QStringLiteral("VERTICAL_AC_TABLE")
        && originOf(QStringLiteral("A"), &a)
        && originOf(QStringLiteral("C"), &c)) {
        return gp_Pnt(c.X(), a.Y(), a.Z());
    }
    if (normalizedPreset == QStringLiteral("VERTICAL_BC_TABLE")
        && originOf(QStringLiteral("B"), &b)
        && originOf(QStringLiteral("C"), &c)) {
        return gp_Pnt(b.X(), c.Y(), b.Z());
    }
    return first;
}

bool sameMachineAxisDefinitions(const QVector<MachineAxisRuntimeConfig>& configs,
                                const QList<MachineAxisDef>& axes)
{
    if (configs.size() != axes.size())
        return false;
    for (int i = 0; i < configs.size(); ++i) {
        const MachineAxisDef& lhs = configs.at(i).axis;
        const MachineAxisDef& rhs = axes.at(i);
        if (lhs.name != rhs.name
            || lhs.motionType != rhs.motionType
            || lhs.parentAxis != rhs.parentAxis
            || !qFuzzyCompare(lhs.direction.X(), rhs.direction.X())
            || !qFuzzyCompare(lhs.direction.Y(), rhs.direction.Y())
            || !qFuzzyCompare(lhs.direction.Z(), rhs.direction.Z())
            || !qFuzzyCompare(lhs.origin.X(), rhs.origin.X())
            || !qFuzzyCompare(lhs.origin.Y(), rhs.origin.Y())
            || !qFuzzyCompare(lhs.origin.Z(), rhs.origin.Z())) {
            return false;
        }
    }
    return true;
}

void styleColorButton(QPushButton* btn, const QColor& color)
{
    if (!btn) return;
    btn->setText(color.name(QColor::HexRgb).toUpper());
    btn->setStyleSheet(QStringLiteral(
        "QPushButton{background-color:%1;color:%2;border:1px solid #888;padding:4px 10px;}")
        .arg(color.name(QColor::HexRgb))
        .arg(color.lightness() > 130 ? "black" : "white"));
}

template <typename SpinBox>
SpinBox* noWheel(SpinBox* spin)
{
    if (spin)
        spin->setFocusPolicy(Qt::StrongFocus);
    return spin;
}

bool profileRuntimeEqual(const RenderProfileSettings& a, const RenderProfileSettings& b)
{
    return a.qualityPreset == b.qualityPreset
        && a.renderMethod == b.renderMethod
        && a.material == b.material
        && a.antiAliasing == b.antiAliasing
        && a.msaaSamples == b.msaaSamples
        && a.shadows == b.shadows
        && a.reflections == b.reflections
        && a.adaptiveSampling == b.adaptiveSampling
        && a.frustumCulling == b.frustumCulling
        && a.backFaceCulling == b.backFaceCulling
        && a.geometryMerge == b.geometryMerge
        && a.proxyGeometry == b.proxyGeometry
        && a.lowLodWhileMoving == b.lowLodWhileMoving
        && a.disableHeavyEffectsDuringSimulation == b.disableHeavyEffectsDuringSimulation
        && qFuzzyCompare(a.ambientLight, b.ambientLight)
        && qFuzzyCompare(a.deviationCoefficient, b.deviationCoefficient)
        && qFuzzyCompare(a.deviationAngle, b.deviationAngle)
        && qFuzzyCompare(a.edgeWidth, b.edgeWidth)
        && qFuzzyCompare(a.renderResolutionScale, b.renderResolutionScale)
        && a.raytracingDepth == b.raytracingDepth
        && a.rayTracingTileSize == b.rayTracingTileSize
        && a.rayTracingTileCount == b.rayTracingTileCount
        && a.targetFps == b.targetFps;
}

bool colorModelEqual(const ColorSettings& a, const ColorSettings& b)
{
    return a.workpieceColor == b.workpieceColor
        && a.machineAxisColors == b.machineAxisColors;
}

bool highlightEqual(const ColorSettings& a, const ColorSettings& b)
{
    return a.selectionColor == b.selectionColor
        && a.hoverColor == b.hoverColor
        && a.highlightDisplayMode == b.highlightDisplayMode
        && qFuzzyCompare(a.highlightLineWidth, b.highlightLineWidth);
}

void applyPresetDefaultsForDialog(RenderProfileSettings& profile, bool cam)
{
    switch (profile.qualityPreset) {
    case RenderQualityPreset::Low:
        profile.renderMethod = RenderMethod::Rasterization;
        profile.antiAliasing = false;
        profile.msaaSamples = 0;
        profile.shadows = false;
        profile.reflections = false;
        profile.adaptiveSampling = false;
        profile.frustumCulling = true;
        profile.backFaceCulling = cam;
        profile.deviationCoefficient = cam ? 0.14 : 0.10;
        profile.deviationAngle = cam ? 0.75 : 0.65;
        profile.edgeWidth = 0.6;
        profile.targetFps = 60;
        break;
    case RenderQualityPreset::Medium:
        profile.renderMethod = RenderMethod::Rasterization;
        profile.antiAliasing = true;
        profile.msaaSamples = 2;
        profile.shadows = false;
        profile.reflections = false;
        profile.adaptiveSampling = false;
        profile.frustumCulling = true;
        profile.backFaceCulling = cam;
        profile.deviationCoefficient = cam ? 0.06 : 0.05;
        profile.deviationAngle = cam ? 0.40 : 0.35;
        profile.edgeWidth = 0.8;
        profile.targetFps = 60;
        break;
    case RenderQualityPreset::High:
        profile.antiAliasing = true;
        profile.msaaSamples = cam ? 4 : 8;
        profile.shadows = !cam;
        profile.reflections = !cam;
        profile.frustumCulling = true;
        profile.backFaceCulling = false;
        profile.deviationCoefficient = cam ? 0.025 : 0.01;
        profile.deviationAngle = cam ? 0.20 : 0.12;
        profile.edgeWidth = 1.1;
        profile.targetFps = cam ? 45 : 30;
        break;
    case RenderQualityPreset::Custom:
        break;
    }
}
} // namespace

DialogOptions::DialogOptions(QWidget* parent)
    : QDialog(parent)
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "DialogOptions ctor");
    setWindowTitle(tr("应用程序选项"));
    resize(980, 680);
    m_machineConfig = lcnc::Kernel::current().service<lcnc::MachineConfigurationService>();
    buildUi();
    loadFromSettings();
}

DialogOptions::~DialogOptions() = default;

bool DialogOptions::eventFilter(QObject* watched, QEvent* event)
{
    if (event && event->type() == QEvent::Wheel
        && qobject_cast<QAbstractSpinBox*>(watched)) {
        event->ignore();
        return true;
    }
    return QDialog::eventFilter(watched, event);
}

void DialogOptions::buildUi()
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "DialogOptions::buildUi begin");
    m_nav = new QTreeWidget(this);
    m_nav->setHeaderHidden(true);
    m_nav->setMinimumWidth(230);

    m_stack = new QStackedWidget(this);
    auto* itemRender = new QTreeWidgetItem(m_nav, QStringList(tr("视图渲染")));
    itemRender->setData(0, Qt::UserRole, 0);
    auto* itemColors = new QTreeWidgetItem(m_nav, QStringList(tr("颜色配置")));
    itemColors->setData(0, Qt::UserRole, 1);
    auto* itemApp = new QTreeWidgetItem(m_nav, QStringList(tr("应用程序")));
    itemApp->setData(0, Qt::UserRole, 2);
    auto* itemMachine = new QTreeWidgetItem(m_nav, QStringList(tr("机台构型")));
    itemMachine->setData(0, Qt::UserRole, 3);

    buildRenderPage(tr("视图渲染"), true, m_renderControls);
    buildColorPage();
    buildApplicationPage();
    buildMachineConfigurationPage();
    Q_UNUSED(itemApp);
    Q_UNUSED(itemMachine);

    connect(m_nav, &QTreeWidget::currentItemChanged, this,
            [this](QTreeWidgetItem* current, QTreeWidgetItem*) {
                if (current)
                    m_stack->setCurrentIndex(current->data(0, Qt::UserRole).toInt());
            });
    m_nav->setCurrentItem(itemRender);

    auto* splitter = new QSplitter(Qt::Horizontal, this);
    splitter->addWidget(m_nav);
    splitter->addWidget(m_stack);
    splitter->setStretchFactor(0, 0);
    splitter->setStretchFactor(1, 1);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok |
                                         QDialogButtonBox::Apply |
                                         QDialogButtonBox::Cancel,
                                         this);
    connect(buttons->button(QDialogButtonBox::Ok), &QPushButton::clicked, this,
            [this] { if (applyChanges()) accept(); });
    connect(buttons->button(QDialogButtonBox::Apply), &QPushButton::clicked, this,
            [this] { applyChanges(); });
    connect(buttons->button(QDialogButtonBox::Cancel), &QPushButton::clicked,
            this, &QDialog::reject);

    auto* layout = new QVBoxLayout(this);
    layout->addWidget(splitter, 1);
    layout->addWidget(buttons);
    disableSpinWheel(this);
    LCNC_DEBUG(lcnc::LogCode::Generic, "DialogOptions::buildUi end");
}

void DialogOptions::disableSpinWheel(QWidget* root)
{
    if (!root)
        return;

    const auto spins = root->findChildren<QAbstractSpinBox*>();
    for (QAbstractSpinBox* spin : spins) {
        spin->setFocusPolicy(Qt::StrongFocus);
        spin->installEventFilter(this);
    }
}

void DialogOptions::buildRenderPage(const QString& title, bool camView, RenderControls& c)
{
    auto* page = new QWidget(this);
    auto* pageLayout = new QVBoxLayout(page);
    auto* scroll = new QScrollArea(page);
    scroll->setWidgetResizable(true);
    auto* content = new QWidget(scroll);
    auto* root = new QVBoxLayout(content);

    auto* displayGroup = new QGroupBox(title, content);
    auto* displayForm = new QFormLayout(displayGroup);
    c.defaultDisplay = new QComboBox(displayGroup);
    c.defaultDisplay->addItem(tr("线框"), static_cast<int>(StartupDisplayMode::Wireframe));
    c.defaultDisplay->addItem(tr("着色"), static_cast<int>(StartupDisplayMode::Shaded));
    displayForm->addRow(tr("启动/新 View 默认显示模式:"), c.defaultDisplay);

    c.quality = new QComboBox(displayGroup);
    c.quality->addItem(tr("低 (Low)"), static_cast<int>(RenderQualityPreset::Low));
    c.quality->addItem(tr("中 (Medium)"), static_cast<int>(RenderQualityPreset::Medium));
    c.quality->addItem(tr("高 (High)"), static_cast<int>(RenderQualityPreset::High));
    c.quality->addItem(tr("自定义 (Custom)"), static_cast<int>(RenderQualityPreset::Custom));
    displayForm->addRow(tr("质量预设:"), c.quality);

    c.renderMethod = new QComboBox(displayGroup);
    c.renderMethod->addItem(tr("光栅化"), static_cast<int>(RenderMethod::Rasterization));
    c.renderMethod->addItem(tr("光线追踪"), static_cast<int>(RenderMethod::RayTracing));
    displayForm->addRow(tr("渲染方法:"), c.renderMethod);

    c.material = new QComboBox(displayGroup);
    c.material->addItem(tr("塑料"), QStringLiteral("plastic"));
    c.material->addItem(tr("亮塑料"), QStringLiteral("shiny_plastic"));
    c.material->addItem(tr("钢"), QStringLiteral("steel"));
    c.material->addItem(tr("铝"), QStringLiteral("aluminum"));
    c.material->addItem(tr("金属"), QStringLiteral("metal"));
    c.material->addItem(tr("铬"), QStringLiteral("chrome"));
    c.material->addItem(tr("缎面"), QStringLiteral("satin"));
    displayForm->addRow(tr("模型材质:"), c.material);
    root->addWidget(displayGroup);

    auto* performanceGroup = new QGroupBox(tr("性能 / 质量"), content);
    auto* performanceForm = new QFormLayout(performanceGroup);
    c.antiAliasing = new QCheckBox(tr("启用抗锯齿"), performanceGroup);
    performanceForm->addRow(QString(), c.antiAliasing);
    c.msaaSamples = new QComboBox(performanceGroup);
    c.msaaSamples->addItem(tr("关闭"), 0);
    c.msaaSamples->addItem(tr("2x"), 2);
    c.msaaSamples->addItem(tr("4x"), 4);
    c.msaaSamples->addItem(tr("8x"), 8);
    performanceForm->addRow(tr("MSAA:"), c.msaaSamples);
    c.renderResolutionScale = noWheel(new QDoubleSpinBox(performanceGroup));
    c.renderResolutionScale->setRange(0.25, 2.0);
    c.renderResolutionScale->setSingleStep(0.05);
    c.renderResolutionScale->setDecimals(2);
    performanceForm->addRow(tr("渲染分辨率比例:"), c.renderResolutionScale);
    c.deviationCoefficient = noWheel(new QDoubleSpinBox(performanceGroup));
    c.deviationCoefficient->setRange(0.001, 1.0);
    c.deviationCoefficient->setSingleStep(0.005);
    c.deviationCoefficient->setDecimals(3);
    performanceForm->addRow(tr("LOD 偏差系数:"), c.deviationCoefficient);
    c.deviationAngle = noWheel(new QDoubleSpinBox(performanceGroup));
    c.deviationAngle->setRange(0.01, 2.0);
    c.deviationAngle->setSingleStep(0.05);
    c.deviationAngle->setDecimals(3);
    performanceForm->addRow(tr("LOD 角度:"), c.deviationAngle);
    c.edgeWidth = noWheel(new QDoubleSpinBox(performanceGroup));
    c.edgeWidth->setRange(0.1, 5.0);
    c.edgeWidth->setSingleStep(0.1);
    c.edgeWidth->setDecimals(1);
    performanceForm->addRow(tr("边线宽度:"), c.edgeWidth);
    root->addWidget(performanceGroup);

    auto* renderGroup = new QGroupBox(tr("渲染特性"), content);
    auto* renderForm = new QFormLayout(renderGroup);
    c.shadows = new QCheckBox(tr("阴影"), renderGroup);
    c.reflections = new QCheckBox(tr("反射"), renderGroup);
    c.adaptiveSampling = new QCheckBox(tr("自适应采样"), renderGroup);
    c.frustumCulling = new QCheckBox(tr("视锥体裁剪"), renderGroup);
    c.backFaceCulling = new QCheckBox(tr("背面剔除"), renderGroup);
    c.geometryMerge = new QCheckBox(tr("几何合并（机台代理/压缩路径）"), renderGroup);
    c.proxyGeometry = new QCheckBox(tr("代理几何"), renderGroup);
    renderForm->addRow(QString(), c.shadows);
    renderForm->addRow(QString(), c.reflections);
    renderForm->addRow(QString(), c.adaptiveSampling);
    renderForm->addRow(QString(), c.frustumCulling);
    renderForm->addRow(QString(), c.backFaceCulling);
    renderForm->addRow(QString(), c.geometryMerge);
    renderForm->addRow(QString(), c.proxyGeometry);
    c.ambientLight = noWheel(new QDoubleSpinBox(renderGroup));
    c.ambientLight->setRange(0.0, 1.0);
    c.ambientLight->setSingleStep(0.05);
    c.ambientLight->setDecimals(2);
    renderForm->addRow(tr("环境光:"), c.ambientLight);
    root->addWidget(renderGroup);

    auto* rayGroup = new QGroupBox(tr("光线追踪 / 采样"), content);
    auto* rayForm = new QFormLayout(rayGroup);
    c.raytracingDepth = noWheel(new QSpinBox(rayGroup));
    c.raytracingDepth->setRange(1, 8);
    rayForm->addRow(tr("光追深度:"), c.raytracingDepth);
    c.rayTracingTileSize = noWheel(new QSpinBox(rayGroup));
    c.rayTracingTileSize->setRange(8, 128);
    c.rayTracingTileSize->setSingleStep(8);
    rayForm->addRow(tr("Tile 大小:"), c.rayTracingTileSize);
    c.rayTracingTileCount = noWheel(new QSpinBox(rayGroup));
    c.rayTracingTileCount->setRange(1, 1024);
    rayForm->addRow(tr("每帧 Tile 数:"), c.rayTracingTileCount);
    root->addWidget(rayGroup);

    auto* simulationGroup = new QGroupBox(tr("仿真效率"), content);
    auto* simulationForm = new QFormLayout(simulationGroup);
    c.targetFps = noWheel(new QSpinBox(simulationGroup));
    c.targetFps->setRange(15, 240);
    simulationForm->addRow(tr("目标帧率:"), c.targetFps);
    c.lowLodWhileMoving = new QCheckBox(tr("运动中使用低 LOD"), simulationGroup);
    c.disableHeavyEffectsDuringSimulation = new QCheckBox(tr("仿真时禁用阴影/反射等重效果"), simulationGroup);
    simulationForm->addRow(QString(), c.lowLodWhileMoving);
    simulationForm->addRow(QString(), c.disableHeavyEffectsDuringSimulation);
    simulationGroup->setVisible(camView);
    root->addWidget(simulationGroup);

    wireRenderPresetBehavior(c, camView);

    root->addStretch(1);
    scroll->setWidget(content);
    pageLayout->addWidget(scroll);
    m_stack->addWidget(page);
}

void DialogOptions::buildColorPage()
{
    auto* page = new QWidget(this);
    auto* root = new QVBoxLayout(page);

    auto* modelGroup = new QGroupBox(tr("模型与背景"), page);
    auto* modelForm = new QFormLayout(modelGroup);
    m_btnWorkpieceColor = makeColorButton(&m_colorDraft.workpieceColor);
    m_btnBackgroundColor = makeColorButton(&m_colorDraft.backgroundColor);
    modelForm->addRow(tr("工件颜色:"), m_btnWorkpieceColor);
    modelForm->addRow(tr("视图背景:"), m_btnBackgroundColor);
    root->addWidget(modelGroup);

    auto* axisGroup = new QGroupBox(tr("机台分轴颜色"), page);
    auto* axisForm = new QFormLayout(axisGroup);
    for (const char* axis : kAxes) {
        const QString name(axis);
        auto* btn = new QPushButton(page);
        btn->setMinimumWidth(150);
        connect(btn, &QPushButton::clicked, this, [this, btn, name] {
            const QColor current = m_colorDraft.machineAxisColors.value(name);
            const QColor picked = QColorDialog::getColor(current, this, tr("选择颜色"));
            if (!picked.isValid())
                return;
            m_colorDraft.machineAxisColors.insert(name, picked);
            styleColorButton(btn, picked);
        });
        m_axisColorButtons.insert(name, btn);
        axisForm->addRow(name, btn);
    }
    root->addWidget(axisGroup);

    auto* highlightGroup = new QGroupBox(tr("选择 / 悬停 / 树节点"), page);
    auto* highlightForm = new QFormLayout(highlightGroup);
    m_btnSelectionColor = makeColorButton(&m_colorDraft.selectionColor);
    m_btnHoverColor = makeColorButton(&m_colorDraft.hoverColor);
    m_btnTreeSelectionColor = makeColorButton(&m_colorDraft.treeSelectionColor);
    m_cbHighlightMode = new QComboBox(highlightGroup);
    m_cbHighlightMode->addItem(tr("沿用对象 displayMode"), -1);
    m_cbHighlightMode->addItem(tr("线框高亮"), 0);
    m_cbHighlightMode->addItem(tr("着色高亮"), 1);
    m_spHighlightLineWidth = noWheel(new QDoubleSpinBox(highlightGroup));
    m_spHighlightLineWidth->setRange(0.5, 10.0);
    m_spHighlightLineWidth->setSingleStep(0.5);
    m_spHighlightLineWidth->setDecimals(1);
    highlightForm->addRow(tr("选中高亮色:"), m_btnSelectionColor);
    highlightForm->addRow(tr("悬停高亮色:"), m_btnHoverColor);
    highlightForm->addRow(tr("树节点选中色:"), m_btnTreeSelectionColor);
    highlightForm->addRow(tr("高亮模式:"), m_cbHighlightMode);
    highlightForm->addRow(tr("高亮线宽:"), m_spHighlightLineWidth);
    root->addWidget(highlightGroup);

    root->addStretch(1);
    m_stack->addWidget(page);
}

void DialogOptions::buildApplicationPage()
{
    auto* page = new QWidget(this);
    auto* root = new QVBoxLayout(page);
    auto* group = new QGroupBox(tr("通用"), page);
    auto* form = new QFormLayout(group);

    m_cbLanguage = new QComboBox(group);
    m_cbLanguage->addItem(tr("简体中文"), QStringLiteral("zh_CN"));
    m_cbLanguage->addItem(QStringLiteral("English"), QStringLiteral("en"));
    form->addRow(tr("语言:"), m_cbLanguage);

    m_cbTheme = new QComboBox(group);
    m_cbTheme->addItem(tr("浅色"), QStringLiteral("light"));
    m_cbTheme->addItem(tr("深色（开发中）"), QStringLiteral("dark"));
    form->addRow(tr("主题:"), m_cbTheme);

    m_cbUnits = new QComboBox(group);
    m_cbUnits->addItem(tr("毫米 (mm)"), QStringLiteral("mm"));
    m_cbUnits->addItem(tr("英寸 (inch)"), QStringLiteral("inch"));
    form->addRow(tr("单位制:"), m_cbUnits);

    m_cbDocumentOpenMode = new QComboBox(group);
    m_cbDocumentOpenMode->addItem(tr("单文档（打开新文件时关闭当前文件）"),
                                  static_cast<int>(DocumentOpenMode::SingleDocument));
    m_cbDocumentOpenMode->addItem(tr("多文档（保留多个项目工作区）"),
                                  static_cast<int>(DocumentOpenMode::MultiDocument));
    form->addRow(tr("打开模式:"), m_cbDocumentOpenMode);

    m_spRecentLimit = noWheel(new QSpinBox(group));
    m_spRecentLimit->setRange(1, 50);
    form->addRow(tr("最近文件数:"), m_spRecentLimit);

    auto* hint = new QLabel(tr("提示：语言/主题修改后需要重启软件才会完全生效。"), group);
    hint->setStyleSheet("color:#888;");
    form->addRow(hint);

    root->addWidget(group);
    root->addStretch(1);
    m_stack->addWidget(page);
}

void DialogOptions::buildMachineConfigurationPage()
{
    auto* page = new QWidget(this);
    auto* root = new QVBoxLayout(page);

    auto* group = new QGroupBox(tr("机台构型"), page);
    auto* form = new QFormLayout(group);
    m_cbMachinePreset = new QComboBox(group);
    for (const QString& preset : machinePresetNames())
        m_cbMachinePreset->addItem(machinePresetText(preset), preset);
    form->addRow(tr("构型"), m_cbMachinePreset);

    auto* pathRow = new QWidget(group);
    auto* pathLayout = new QHBoxLayout(pathRow);
    pathLayout->setContentsMargins(0, 0, 0, 0);
    pathLayout->setSpacing(4);
    m_editMachineModelPath = new QLineEdit(pathRow);
    m_editMachineModelPath->setClearButtonEnabled(true);
    m_editMachineModelPath->setPlaceholderText(tr("选择或输入机台模型文件路径"));
    m_btnBrowseMachineModel = new QPushButton(tr("浏览..."), pathRow);
    pathLayout->addWidget(m_editMachineModelPath, 1);
    pathLayout->addWidget(m_btnBrowseMachineModel);
    form->addRow(tr("机台模型路径"), pathRow);

    m_chkAutoLoadMachineModel = new QCheckBox(tr("启动时自动加载机台模型"), group);
    form->addRow(QString(), m_chkAutoLoadMachineModel);

    m_lblMachineAlgorithm = new QLabel(group);
    m_lblMachineAlgorithm->setTextInteractionFlags(Qt::TextSelectableByMouse);
    form->addRow(tr("刀路算法"), m_lblMachineAlgorithm);
    root->addWidget(group);

    auto* centerGroup = new QGroupBox(tr("旋转中心"), page);
    auto* centerForm = new QFormLayout(centerGroup);
    auto* centerRow = new QWidget(centerGroup);
    auto* centerLayout = new QHBoxLayout(centerRow);
    centerLayout->setContentsMargins(0, 0, 0, 0);
    centerLayout->setSpacing(6);
    m_spRotationCenterX = machineCoordinateSpin(centerRow);
    m_spRotationCenterY = machineCoordinateSpin(centerRow);
    m_spRotationCenterZ = machineCoordinateSpin(centerRow);
    centerLayout->addWidget(new QLabel(QStringLiteral("X"), centerRow));
    centerLayout->addWidget(m_spRotationCenterX);
    centerLayout->addWidget(new QLabel(QStringLiteral("Y"), centerRow));
    centerLayout->addWidget(m_spRotationCenterY);
    centerLayout->addWidget(new QLabel(QStringLiteral("Z"), centerRow));
    centerLayout->addWidget(m_spRotationCenterZ);
    centerLayout->addStretch(1);
    centerForm->addRow(tr("中心坐标"), centerRow);
    m_lblRotationCenterHint = new QLabel(centerGroup);
    m_lblRotationCenterHint->setWordWrap(true);
    m_lblRotationCenterHint->setStyleSheet("color:#666;");
    centerForm->addRow(m_lblRotationCenterHint);
    root->addWidget(centerGroup);

    m_machineAxesTable = new QTableWidget(page);
    m_machineAxesTable->setColumnCount(9);
    m_machineAxesTable->setHorizontalHeaderLabels({
        tr("轴名"), tr("类型"), tr("父轴"), tr("方向X"), tr("方向Y"), tr("方向Z"),
        tr("原点X"), tr("原点Y"), tr("原点Z")
    });
    m_machineAxesTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    m_machineAxesTable->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
    m_machineAxesTable->verticalHeader()->setVisible(false);
    m_machineAxesTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_machineAxesTable->setSelectionMode(QAbstractItemView::SingleSelection);
    m_machineAxesTable->setAlternatingRowColors(true);
    root->addWidget(m_machineAxesTable, 1);

    connect(m_cbMachinePreset, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            [this] {
                if (m_loadingUi || !m_cbMachinePreset)
                    return;
                populateMachineAxisTable(machineConfigsForPreset(m_cbMachinePreset->currentData().toString()));
            });
    auto syncCenterToAxes = [this] {
        if (m_loadingUi)
            return;
        applyRotationCenterToMachineAxisTable();
        if (m_lblMachineAlgorithm && m_cbMachinePreset)
            m_lblMachineAlgorithm->setText(
                algorithmTextForAxes(m_cbMachinePreset->currentData().toString(),
                                     collectMachineAxisDefinitions()));
    };
    connect(m_spRotationCenterX, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, syncCenterToAxes);
    connect(m_spRotationCenterY, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, syncCenterToAxes);
    connect(m_spRotationCenterZ, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, syncCenterToAxes);
    connect(m_btnBrowseMachineModel, &QPushButton::clicked, this,
            [this] {
                const QString currentPath = m_editMachineModelPath
                    ? m_editMachineModelPath->text().trimmed()
                    : QString();
                const QString dir = currentPath.isEmpty() ? QString() : QFileInfo(currentPath).absolutePath();
                const QString path = QFileDialog::getOpenFileName(
                    this,
                    tr("选择机台模型文件"),
                    dir,
                    tr("三维模型文件 (*.stp *.step *.stl *.brep);;STEP (*.stp *.step);;STL (*.stl);;BREP (*.brep);;所有文件 (*)"));
                if (!path.isEmpty() && m_editMachineModelPath)
                    m_editMachineModelPath->setText(QFileInfo(path).absoluteFilePath());
            });

    m_stack->addWidget(page);
}

void DialogOptions::populateMachineAxisTable(const QVector<MachineAxisRuntimeConfig>& configs)
{
    if (!m_machineAxesTable)
        return;
    m_machineAxesTable->setRowCount(0);

    QStringList parentCandidates{QStringLiteral("BASE")};
    for (const MachineAxisRuntimeConfig& config : configs) {
        const QString name = config.axis.name.trimmed().toUpper();
        if (!name.isEmpty() && name != QStringLiteral("BASE") && !parentCandidates.contains(name))
            parentCandidates.append(name);
    }

    for (const MachineAxisRuntimeConfig& config : configs) {
        const QString name = config.axis.name.trimmed().toUpper();
        if (name.isEmpty() || name == QStringLiteral("BASE"))
            continue;
        const int row = m_machineAxesTable->rowCount();
        m_machineAxesTable->insertRow(row);
        auto* nameItem = machineAxisTableItem(name, false);
        nameItem->setData(Qt::UserRole, config.axis.minVal);
        nameItem->setData(Qt::UserRole + 1, config.axis.maxVal);
        m_machineAxesTable->setItem(row, 0, nameItem);

        auto* typeCombo = new QComboBox(m_machineAxesTable);
        typeCombo->addItem(tr("线性"), static_cast<int>(MachineAxisDef::Linear));
        typeCombo->addItem(tr("旋转"), static_cast<int>(MachineAxisDef::Rotary));
        typeCombo->setCurrentIndex(config.axis.motionType == MachineAxisDef::Rotary ? 1 : 0);
        connect(typeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
                [this] {
                    applyRotationCenterToMachineAxisTable();
                    if (m_lblMachineAlgorithm && m_cbMachinePreset)
                        m_lblMachineAlgorithm->setText(
                            algorithmTextForAxes(m_cbMachinePreset->currentData().toString(),
                                                 collectMachineAxisDefinitions()));
                });
        m_machineAxesTable->setCellWidget(row, 1, typeCombo);

        auto* parentCombo = new QComboBox(m_machineAxesTable);
        for (const QString& parent : parentCandidates) {
            if (parent != name)
                parentCombo->addItem(parent, parent);
        }
        QString parentAxis = config.axis.parentAxis.trimmed().toUpper();
        if (parentAxis.isEmpty())
            parentAxis = QStringLiteral("BASE");
        const int parentIndex = parentCombo->findData(parentAxis);
        parentCombo->setCurrentIndex(parentIndex >= 0 ? parentIndex : 0);
        m_machineAxesTable->setCellWidget(row, 2, parentCombo);

        m_machineAxesTable->setItem(row, 3, machineAxisTableItem(QString::number(config.axis.direction.X(), 'g', 15)));
        m_machineAxesTable->setItem(row, 4, machineAxisTableItem(QString::number(config.axis.direction.Y(), 'g', 15)));
        m_machineAxesTable->setItem(row, 5, machineAxisTableItem(QString::number(config.axis.direction.Z(), 'g', 15)));
        m_machineAxesTable->setItem(row, 6, machineAxisTableItem(QString::number(config.axis.origin.X(), 'g', 15)));
        m_machineAxesTable->setItem(row, 7, machineAxisTableItem(QString::number(config.axis.origin.Y(), 'g', 15)));
        m_machineAxesTable->setItem(row, 8, machineAxisTableItem(QString::number(config.axis.origin.Z(), 'g', 15)));
    }

    QList<MachineAxisDef> rawAxes;
    rawAxes.reserve(configs.size());
    for (const MachineAxisRuntimeConfig& config : configs)
        rawAxes.append(config.axis);
    setRotationCenterUiFromAxes(rawAxes);
    applyRotationCenterToMachineAxisTable();
    if (m_lblMachineAlgorithm && m_cbMachinePreset)
        m_lblMachineAlgorithm->setText(
            algorithmTextForAxes(m_cbMachinePreset->currentData().toString(), collectMachineAxisDefinitions()));
}

QList<MachineAxisDef> DialogOptions::collectMachineAxisDefinitions() const
{
    QList<MachineAxisDef> axes;
    if (!m_machineAxesTable)
        return axes;

    auto itemText = [this](int row, int column) {
        QTableWidgetItem* item = m_machineAxesTable->item(row, column);
        return item ? item->text().trimmed() : QString();
    };
    auto itemDouble = [&](int row, int column, double fallback) {
        bool ok = false;
        const double value = itemText(row, column).toDouble(&ok);
        return ok ? value : fallback;
    };

    for (int row = 0; row < m_machineAxesTable->rowCount(); ++row) {
        const QString name = itemText(row, 0).toUpper();
        if (name.isEmpty() || name == QStringLiteral("BASE"))
            continue;
        MachineAxisDef axis;
        axis.name = name;
        axis.motionType = MachineAxisDef::Linear;
        if (auto* combo = qobject_cast<QComboBox*>(m_machineAxesTable->cellWidget(row, 1)))
            axis.motionType = static_cast<MachineAxisDef::MotionType>(combo->currentData().toInt());
        if (auto* combo = qobject_cast<QComboBox*>(m_machineAxesTable->cellWidget(row, 2)))
            axis.parentAxis = combo->currentData().toString();
        if (axis.parentAxis.trimmed().isEmpty())
            axis.parentAxis = QStringLiteral("BASE");

        const gp_Dir fallbackDirection = defaultMachineAxisDirection(name, axis.motionType);
        const double dx = itemDouble(row, 3, fallbackDirection.X());
        const double dy = itemDouble(row, 4, fallbackDirection.Y());
        const double dz = itemDouble(row, 5, fallbackDirection.Z());
        const double norm2 = dx * dx + dy * dy + dz * dz;
        axis.direction = norm2 > 1e-12 ? gp_Dir(dx, dy, dz) : fallbackDirection;
        axis.origin = gp_Pnt(itemDouble(row, 6, 0.0),
                             itemDouble(row, 7, 0.0),
                             itemDouble(row, 8, 0.0));
        if (QTableWidgetItem* nameItem = m_machineAxesTable->item(row, 0)) {
            axis.minVal = nameItem->data(Qt::UserRole).toDouble();
            axis.maxVal = nameItem->data(Qt::UserRole + 1).toDouble();
        }
        axes.append(axis);
    }

    if (m_spRotationCenterX && m_spRotationCenterY && m_spRotationCenterZ && hasRotaryAxisInTable()) {
        const gp_Pnt center(m_spRotationCenterX->value(),
                            m_spRotationCenterY->value(),
                            m_spRotationCenterZ->value());
        for (MachineAxisDef& axis : axes) {
            if (axis.motionType == MachineAxisDef::Rotary)
                axis.origin = center;
        }
    }
    return axes;
}

bool DialogOptions::hasRotaryAxisInTable() const
{
    if (!m_machineAxesTable)
        return false;
    for (int row = 0; row < m_machineAxesTable->rowCount(); ++row) {
        if (auto* combo = qobject_cast<QComboBox*>(m_machineAxesTable->cellWidget(row, 1))) {
            if (static_cast<MachineAxisDef::MotionType>(combo->currentData().toInt())
                == MachineAxisDef::Rotary) {
                return true;
            }
        }
    }
    return false;
}

void DialogOptions::setRotationCenterUiFromAxes(const QList<MachineAxisDef>& axes)
{
    if (!m_spRotationCenterX || !m_spRotationCenterY || !m_spRotationCenterZ)
        return;

    const QStringList rotaryNames = rotaryAxisNames(axes);
    const bool enabled = !rotaryNames.isEmpty();
    const gp_Pnt center = enabled
        ? rotationCenterFromAxes(axes, m_cbMachinePreset ? m_cbMachinePreset->currentData().toString() : QString())
        : gp_Pnt(0.0, 0.0, 0.0);

    const QSignalBlocker blockX(m_spRotationCenterX);
    const QSignalBlocker blockY(m_spRotationCenterY);
    const QSignalBlocker blockZ(m_spRotationCenterZ);
    m_spRotationCenterX->setEnabled(enabled);
    m_spRotationCenterY->setEnabled(enabled);
    m_spRotationCenterZ->setEnabled(enabled);
    m_spRotationCenterX->setValue(center.X());
    m_spRotationCenterY->setValue(center.Y());
    m_spRotationCenterZ->setValue(center.Z());

    if (m_lblRotationCenterHint) {
        if (enabled) {
            m_lblRotationCenterHint->setText(
                tr("该坐标会写入旋转轴 %1 的原点；AC 转台请填写 A 轴与 C 轴的物理交点。")
                    .arg(rotaryNames.join(QStringLiteral("/"))));
        } else {
            m_lblRotationCenterHint->setText(tr("当前构型没有旋转轴，不需要填写旋转中心。"));
        }
    }
}

void DialogOptions::applyRotationCenterToMachineAxisTable()
{
    if (!m_machineAxesTable || !m_spRotationCenterX || !m_spRotationCenterY || !m_spRotationCenterZ)
        return;
    const bool hasRotary = hasRotaryAxisInTable();
    m_spRotationCenterX->setEnabled(hasRotary);
    m_spRotationCenterY->setEnabled(hasRotary);
    m_spRotationCenterZ->setEnabled(hasRotary);

    if (!hasRotary)
        return;

    const QString x = QString::number(m_spRotationCenterX->value(), 'g', 15);
    const QString y = QString::number(m_spRotationCenterY->value(), 'g', 15);
    const QString z = QString::number(m_spRotationCenterZ->value(), 'g', 15);
    for (int row = 0; row < m_machineAxesTable->rowCount(); ++row) {
        auto* combo = qobject_cast<QComboBox*>(m_machineAxesTable->cellWidget(row, 1));
        if (!combo || static_cast<MachineAxisDef::MotionType>(combo->currentData().toInt())
            != MachineAxisDef::Rotary) {
            continue;
        }
        if (QTableWidgetItem* item = m_machineAxesTable->item(row, 6)) item->setText(x);
        if (QTableWidgetItem* item = m_machineAxesTable->item(row, 7)) item->setText(y);
        if (QTableWidgetItem* item = m_machineAxesTable->item(row, 8)) item->setText(z);
    }
}

void DialogOptions::loadFromSettings()
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "DialogOptions::loadFromSettings begin");
    m_loadingUi = true;
    auto* settings = lcnc::Kernel::current().appSettings();
    if (!settings) {
        LCNC_ERR(lcnc::LogCode::InternalUnexpectedState,
                 "DialogOptions::loadFromSettings: AppSettings null");
        m_loadingUi = false;
        return;
    }

    m_originalCad = settings->cadViewRendering;
    m_originalCam = settings->camViewRendering;
    m_originalColors = settings->colors;
    m_originalLanguage = settings->language;
    m_originalTheme = settings->theme;
    m_originalUnitSystem = settings->unitSystem;
    m_originalDocumentOpenMode = settings->documentOpenMode;
    m_originalRecentLimit = settings->recentLimit;
    if (auto* cam = lcnc::Kernel::current().service<CamModule>()) {
        m_originalMachineModelPath = cam->machineModelPath();
        m_originalAutoLoadMachineModel = cam->config().autoLoadMachineModel();
    } else {
        m_originalMachineModelPath.clear();
        m_originalAutoLoadMachineModel = true;
    }

    m_renderDraft = m_originalCam;
    m_colorDraft = m_originalColors;
    m_colorDraft.cadBackgroundColor = m_colorDraft.backgroundColor;
    m_colorDraft.camBackgroundColor = m_colorDraft.backgroundColor;
    if (m_colorDraft.machineAxisColors.isEmpty())
        m_colorDraft.machineAxisColors = defaultMachineAxisColors();

    setProfileToUi(m_renderDraft, m_renderControls);

    styleColorButton(m_btnWorkpieceColor, m_colorDraft.workpieceColor);
    styleColorButton(m_btnBackgroundColor, m_colorDraft.backgroundColor);
    styleColorButton(m_btnSelectionColor, m_colorDraft.selectionColor);
    styleColorButton(m_btnHoverColor, m_colorDraft.hoverColor);
    styleColorButton(m_btnTreeSelectionColor, m_colorDraft.treeSelectionColor);
    for (const char* axis : kAxes) {
        const QString name(axis);
        if (!m_colorDraft.machineAxisColors.contains(name))
            m_colorDraft.machineAxisColors.insert(name, defaultMachineAxisColors().value(name));
        styleColorButton(m_axisColorButtons.value(name), m_colorDraft.machineAxisColors.value(name));
    }
    setComboByData(m_cbHighlightMode, m_colorDraft.highlightDisplayMode);
    m_spHighlightLineWidth->setValue(m_colorDraft.highlightLineWidth);

    setComboByData(m_cbLanguage, settings->language);
    setComboByData(m_cbTheme, settings->theme);
    setComboByData(m_cbUnits, settings->unitSystem);
    setComboByData(m_cbDocumentOpenMode, static_cast<int>(settings->documentOpenMode));
    m_spRecentLimit->setValue(settings->recentLimit);
    if (m_editMachineModelPath)
        m_editMachineModelPath->setText(m_originalMachineModelPath);
    if (m_chkAutoLoadMachineModel)
        m_chkAutoLoadMachineModel->setChecked(m_originalAutoLoadMachineModel);
    if (m_machineConfig) {
        m_originalMachinePreset = m_machineConfig->presetName();
        m_originalMachineConfigs = m_machineConfig->axisConfigurations();
        setComboByData(m_cbMachinePreset, m_originalMachinePreset);
        populateMachineAxisTable(m_originalMachineConfigs);
    } else {
        m_originalMachinePreset = QStringLiteral("VERTICAL_AC_TABLE");
        m_originalMachineConfigs = machineConfigsForPreset(m_originalMachinePreset);
        setComboByData(m_cbMachinePreset, m_originalMachinePreset);
        populateMachineAxisTable(m_originalMachineConfigs);
        if (m_machineAxesTable)
            m_machineAxesTable->setEnabled(false);
        if (m_cbMachinePreset)
            m_cbMachinePreset->setEnabled(false);
        if (m_spRotationCenterX) m_spRotationCenterX->setEnabled(false);
        if (m_spRotationCenterY) m_spRotationCenterY->setEnabled(false);
        if (m_spRotationCenterZ) m_spRotationCenterZ->setEnabled(false);
    }
    applyTreeSelectionColor(m_colorDraft.treeSelectionColor);
    m_loadingUi = false;
    LCNC_DEBUG(lcnc::LogCode::Generic, "DialogOptions::loadFromSettings end");
}

void DialogOptions::setProfileToUi(const RenderProfileSettings& p, const RenderControls& c)
{
    setComboByData(c.defaultDisplay, static_cast<int>(p.defaultDisplayMode));
    setComboByData(c.quality, static_cast<int>(p.qualityPreset));
    setComboByData(c.renderMethod, static_cast<int>(p.renderMethod));
    setComboByData(c.material, p.material);
    c.antiAliasing->setChecked(p.antiAliasing);
    setComboByData(c.msaaSamples, p.msaaSamples);
    c.shadows->setChecked(p.shadows);
    c.reflections->setChecked(p.reflections);
    c.adaptiveSampling->setChecked(p.adaptiveSampling);
    c.frustumCulling->setChecked(p.frustumCulling);
    c.backFaceCulling->setChecked(p.backFaceCulling);
    c.geometryMerge->setChecked(p.geometryMerge);
    c.proxyGeometry->setChecked(p.proxyGeometry);
    c.lowLodWhileMoving->setChecked(p.lowLodWhileMoving);
    c.disableHeavyEffectsDuringSimulation->setChecked(p.disableHeavyEffectsDuringSimulation);
    c.ambientLight->setValue(p.ambientLight);
    c.deviationCoefficient->setValue(p.deviationCoefficient);
    c.deviationAngle->setValue(p.deviationAngle);
    c.edgeWidth->setValue(p.edgeWidth);
    c.renderResolutionScale->setValue(p.renderResolutionScale);
    c.raytracingDepth->setValue(p.raytracingDepth);
    c.rayTracingTileSize->setValue(p.rayTracingTileSize);
    c.rayTracingTileCount->setValue(p.rayTracingTileCount);
    c.targetFps->setValue(p.targetFps);
}

RenderProfileSettings DialogOptions::collectProfileFromUi(const RenderControls& c) const
{
    RenderProfileSettings p;
    p.defaultDisplayMode = static_cast<StartupDisplayMode>(c.defaultDisplay->currentData().toInt());
    p.qualityPreset = static_cast<RenderQualityPreset>(c.quality->currentData().toInt());
    p.renderMethod = static_cast<RenderMethod>(c.renderMethod->currentData().toInt());
    p.material = c.material->currentData().toString();
    p.antiAliasing = c.antiAliasing->isChecked();
    p.msaaSamples = c.msaaSamples->currentData().toInt();
    p.shadows = c.shadows->isChecked();
    p.reflections = c.reflections->isChecked();
    p.adaptiveSampling = c.adaptiveSampling->isChecked();
    p.frustumCulling = c.frustumCulling->isChecked();
    p.backFaceCulling = c.backFaceCulling->isChecked();
    p.geometryMerge = c.geometryMerge->isChecked();
    p.proxyGeometry = c.proxyGeometry->isChecked();
    p.lowLodWhileMoving = c.lowLodWhileMoving->isChecked();
    p.disableHeavyEffectsDuringSimulation = c.disableHeavyEffectsDuringSimulation->isChecked();
    p.ambientLight = c.ambientLight->value();
    p.deviationCoefficient = c.deviationCoefficient->value();
    p.deviationAngle = c.deviationAngle->value();
    p.edgeWidth = c.edgeWidth->value();
    p.renderResolutionScale = c.renderResolutionScale->value();
    p.raytracingDepth = c.raytracingDepth->value();
    p.rayTracingTileSize = c.rayTracingTileSize->value();
    p.rayTracingTileCount = c.rayTracingTileCount->value();
    p.targetFps = c.targetFps->value();
    return p;
}

void DialogOptions::wireRenderPresetBehavior(RenderControls& c, bool camView)
{
    connect(c.quality, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            [this, &c, camView] {
                if (m_loadingUi)
                    return;
                auto preset = static_cast<RenderQualityPreset>(c.quality->currentData().toInt());
                if (preset == RenderQualityPreset::Custom)
                    return;
                RenderProfileSettings profile = collectProfileFromUi(c);
                profile.qualityPreset = preset;
                applyPresetDefaultsForDialog(profile, camView);
                m_loadingUi = true;
                setProfileToUi(profile, c);
                m_loadingUi = false;
            });

    auto markCustom = [this, &c] {
        if (m_loadingUi)
            return;
        setComboByData(c.quality, static_cast<int>(RenderQualityPreset::Custom));
    };
    connect(c.renderMethod, QOverload<int>::of(&QComboBox::currentIndexChanged), this, markCustom);
    connect(c.material, QOverload<int>::of(&QComboBox::currentIndexChanged), this, markCustom);
    connect(c.antiAliasing, &QCheckBox::toggled, this, markCustom);
    connect(c.msaaSamples, QOverload<int>::of(&QComboBox::currentIndexChanged), this, markCustom);
    connect(c.shadows, &QCheckBox::toggled, this, markCustom);
    connect(c.reflections, &QCheckBox::toggled, this, markCustom);
    connect(c.adaptiveSampling, &QCheckBox::toggled, this, markCustom);
    connect(c.frustumCulling, &QCheckBox::toggled, this, markCustom);
    connect(c.backFaceCulling, &QCheckBox::toggled, this, markCustom);
    connect(c.geometryMerge, &QCheckBox::toggled, this, markCustom);
    connect(c.proxyGeometry, &QCheckBox::toggled, this, markCustom);
    connect(c.lowLodWhileMoving, &QCheckBox::toggled, this, markCustom);
    connect(c.disableHeavyEffectsDuringSimulation, &QCheckBox::toggled, this, markCustom);
    connect(c.ambientLight, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, markCustom);
    connect(c.deviationCoefficient, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, markCustom);
    connect(c.deviationAngle, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, markCustom);
    connect(c.edgeWidth, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, markCustom);
    connect(c.renderResolutionScale, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, markCustom);
    connect(c.raytracingDepth, QOverload<int>::of(&QSpinBox::valueChanged), this, markCustom);
    connect(c.rayTracingTileSize, QOverload<int>::of(&QSpinBox::valueChanged), this, markCustom);
    connect(c.rayTracingTileCount, QOverload<int>::of(&QSpinBox::valueChanged), this, markCustom);
    connect(c.targetFps, QOverload<int>::of(&QSpinBox::valueChanged), this, markCustom);
}

void DialogOptions::setComboByData(QComboBox* combo, const QVariant& value)
{
    if (!combo) return;
    const int index = combo->findData(value);
    combo->setCurrentIndex(index >= 0 ? index : 0);
}

QPushButton* DialogOptions::makeColorButton(QColor* target)
{
    auto* button = new QPushButton(this);
    button->setMinimumWidth(150);
    connect(button, &QPushButton::clicked, this, [this, button, target] {
        const QColor picked = QColorDialog::getColor(*target, this, tr("选择颜色"));
        if (!picked.isValid())
            return;
        *target = picked;
        styleColorButton(button, picked);
    });
    return button;
}

void DialogOptions::applyTreeSelectionColor(const QColor& color)
{
    const QString qss = QStringLiteral(
        "QTreeWidget::item:selected { background-color: %1; color: white; }"
        "QTreeWidget::item:selected:!active { background-color: %1; color: white; }")
        .arg(color.name(QColor::HexRgb));
    const auto widgets = QApplication::allWidgets();
    for (QWidget* widget : widgets) {
        if (auto* tree = qobject_cast<QTreeWidget*>(widget))
            tree->setStyleSheet(qss);
    }
}

bool DialogOptions::applyChanges()
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "DialogOptions::applyChanges begin");
    auto* settings = lcnc::Kernel::current().appSettings();
    if (!settings) {
        LCNC_ERR(lcnc::LogCode::InternalUnexpectedState,
                 "DialogOptions::applyChanges: AppSettings null");
        return false;
    }

    m_renderDraft = collectProfileFromUi(m_renderControls);
    m_colorDraft.cadBackgroundColor = m_colorDraft.backgroundColor;
    m_colorDraft.camBackgroundColor = m_colorDraft.backgroundColor;
    m_colorDraft.highlightDisplayMode = m_cbHighlightMode->currentData().toInt();
    m_colorDraft.highlightLineWidth = m_spHighlightLineWidth->value();

    const QString newLanguage = m_cbLanguage->currentData().toString();
    const QString newTheme = m_cbTheme->currentData().toString();
    const QString newUnits = m_cbUnits->currentData().toString();
    const auto newDocumentOpenMode = static_cast<DocumentOpenMode>(
        m_cbDocumentOpenMode ? m_cbDocumentOpenMode->currentData().toInt()
                             : static_cast<int>(m_originalDocumentOpenMode));
    const int newRecentLimit = m_spRecentLimit->value();
    const QString newMachineModelPath = m_editMachineModelPath
        ? m_editMachineModelPath->text().trimmed()
        : m_originalMachineModelPath;
    const bool newAutoLoadMachineModel = m_chkAutoLoadMachineModel
        ? m_chkAutoLoadMachineModel->isChecked()
        : m_originalAutoLoadMachineModel;
    const QString newMachinePreset = m_cbMachinePreset
        ? m_cbMachinePreset->currentData().toString()
        : m_originalMachinePreset;
    const QList<MachineAxisDef> newMachineAxes = collectMachineAxisDefinitions();

    const bool cadRuntimeDirty = !profileRuntimeEqual(m_originalCad, m_renderDraft);
    const bool camRuntimeDirty = !profileRuntimeEqual(m_originalCam, m_renderDraft);
    const bool cadDefaultDirty = m_originalCad.defaultDisplayMode != m_renderDraft.defaultDisplayMode;
    const bool camDefaultDirty = m_originalCam.defaultDisplayMode != m_renderDraft.defaultDisplayMode;
    const bool backgroundDirty = m_originalColors.backgroundColor != m_colorDraft.backgroundColor;
    const bool modelColorDirty = !colorModelEqual(m_originalColors, m_colorDraft);
    const bool highlightDirty = !highlightEqual(m_originalColors, m_colorDraft);
    const bool treeDirty = m_originalColors.treeSelectionColor != m_colorDraft.treeSelectionColor;
    const bool applicationDirty = m_originalLanguage != newLanguage
        || m_originalTheme != newTheme
        || m_originalUnitSystem != newUnits
        || m_originalDocumentOpenMode != newDocumentOpenMode
        || m_originalRecentLimit != newRecentLimit;
    const bool machineModelPathDirty = m_originalMachineModelPath != newMachineModelPath;
    const bool autoLoadMachineDirty = m_originalAutoLoadMachineModel != newAutoLoadMachineModel;
    const bool machineDirty = m_machineConfig
        && (m_originalMachinePreset != newMachinePreset
            || !sameMachineAxisDefinitions(m_originalMachineConfigs, newMachineAxes));

    if (!cadRuntimeDirty && !camRuntimeDirty && !cadDefaultDirty && !camDefaultDirty
        && !backgroundDirty && !modelColorDirty
        && !highlightDirty && !treeDirty && !applicationDirty
        && !machineModelPathDirty && !autoLoadMachineDirty && !machineDirty) {
        LCNC_DEBUG(lcnc::LogCode::Generic, "DialogOptions::applyChanges no changes");
        return true;
    }

    settings->cadViewRendering = m_renderDraft;
    settings->camViewRendering = m_renderDraft;
    settings->colors = m_colorDraft;
    settings->language = newLanguage;
    settings->theme = newTheme;
    settings->unitSystem = newUnits;
    settings->documentOpenMode = newDocumentOpenMode;
    if (auto* project = lcnc::Kernel::current().projectManager())
        project->setDocumentOpenMode(newDocumentOpenMode);
    settings->recentLimit = newRecentLimit;
    if (!settings->saveDefault()) {
        LCNC_WARN(lcnc::LogCode::InternalUnexpectedState,
                  "DialogOptions: saveDefault() failed");
    }

    auto* guiApp = lcnc::Kernel::current().guiApp();
    if (guiApp) {
        lcnc::view::RenderDirtyFlags cadFlags(lcnc::view::RenderDirtyFlag::None);
        lcnc::view::RenderDirtyFlags camFlags(lcnc::view::RenderDirtyFlag::None);
        if (cadRuntimeDirty) cadFlags |= lcnc::view::RenderDirtyFlag::Profile;
        if (camRuntimeDirty) camFlags |= lcnc::view::RenderDirtyFlag::Profile;
        if (backgroundDirty) {
            cadFlags |= lcnc::view::RenderDirtyFlag::Background;
            camFlags |= lcnc::view::RenderDirtyFlag::Background;
        }
        if (modelColorDirty) {
            cadFlags |= lcnc::view::RenderDirtyFlag::Colors;
            camFlags |= lcnc::view::RenderDirtyFlag::Colors;
        }
        if (highlightDirty) {
            cadFlags |= lcnc::view::RenderDirtyFlag::Highlight;
            camFlags |= lcnc::view::RenderDirtyFlag::Highlight;
        }
    // 默认显示模式只作为启动/新 view 默认值保存，不刷新当前 view。
    Q_UNUSED(cadDefaultDirty);
    Q_UNUSED(camDefaultDirty);
        if (cadFlags != lcnc::view::RenderDirtyFlags(lcnc::view::RenderDirtyFlag::None)) {
            guiApp->requestApplyRenderingSettings(
                settings->cadViewRendering, settings->camViewRendering, settings->colors,
                cadFlags, true, false);
        }
        if (camFlags != lcnc::view::RenderDirtyFlags(lcnc::view::RenderDirtyFlag::None)) {
            guiApp->requestApplyRenderingSettings(
                settings->cadViewRendering, settings->camViewRendering, settings->colors,
                camFlags, false, true);
        }
    }

    if (treeDirty)
        applyTreeSelectionColor(m_colorDraft.treeSelectionColor);

    if (m_originalLanguage != newLanguage) {
        QMessageBox::information(this, tr("应用程序选项"),
            tr("语言修改将在重启软件后生效。"));
    }

    if (machineDirty) {
        m_machineConfig->setMachineAxisDefinitions(newMachinePreset, newMachineAxes);
        m_originalMachinePreset = m_machineConfig->presetName();
        m_originalMachineConfigs = m_machineConfig->axisConfigurations();
        populateMachineAxisTable(m_originalMachineConfigs);
    }
    if (auto* cam = lcnc::Kernel::current().service<CamModule>()) {
        if (machineModelPathDirty)
            cam->setMachineModelPath(newMachineModelPath);
        if (autoLoadMachineDirty)
            cam->config().setAutoLoadMachineModel(newAutoLoadMachineModel);
        m_originalMachineModelPath = cam->machineModelPath();
        m_originalAutoLoadMachineModel = cam->config().autoLoadMachineModel();
        if (m_editMachineModelPath && machineModelPathDirty)
            m_editMachineModelPath->setText(m_originalMachineModelPath);
    }

    m_originalCad = m_renderDraft;
    m_originalCam = m_renderDraft;
    m_originalColors = m_colorDraft;
    m_originalLanguage = newLanguage;
    m_originalTheme = newTheme;
    m_originalUnitSystem = newUnits;
    m_originalDocumentOpenMode = newDocumentOpenMode;
    m_originalRecentLimit = newRecentLimit;

    LCNC_INFO(lcnc::LogCode::Generic,
              "Application options applied: cadRuntime={} camRuntime={} colors={} highlight={} app={} machine={}",
              cadRuntimeDirty, camRuntimeDirty, modelColorDirty, highlightDirty, applicationDirty, machineDirty);
    return true;
}

} // namespace lcnc
