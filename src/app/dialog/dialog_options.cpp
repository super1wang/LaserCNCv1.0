#include "app/dialog/dialog_options.h"

#include "core/kernel/kernel.h"
#include "core/logging/logger.h"
#include "core/project/lcnc_project_manager.h"
#include "modules/cam/cam_module.h"
#include "modules/process/process_module.h"
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
#include <QObject>

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
        // 中文翻译：三轴 XYZ
        return QObject::tr("Three-axis XYZ");
    if (preset == QStringLiteral("XYZA"))
        // 中文翻译：四轴 XYZA
        return QObject::tr("Four-axis XYZA");
    if (preset == QStringLiteral("VERTICAL_AC_TABLE"))
        // 中文翻译：立式 AC 转台
        return QObject::tr("Vertical AC turntable");
    if (preset == QStringLiteral("VERTICAL_BC_TABLE"))
        // 中文翻译：立式 BC 转台
        return QObject::tr("Vertical BC turntable");
    if (preset == QStringLiteral("AB_HEAD"))
        // 中文翻译：AB 摆头
        return QObject::tr("AB head");
    if (preset == QStringLiteral("AC_HEAD"))
        // 中文翻译：AC 摆头
        return QObject::tr("AC head");
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

QString algorithmTextForAxes(const QString& /*preset*/, const QList<MachineAxisDef>& axes)
{
    bool workpieceRotary = false;
    bool tableTilt = false;
    bool tableSpin = false;
    bool headPrimary = false;
    bool headSecondary = false;
    for (const MachineAxisDef& axis : axes) {
        workpieceRotary |= axis.role == lcnc::MachineAxisRole::WorkpieceRotary;
        tableTilt |= axis.role == lcnc::MachineAxisRole::TableTilt;
        tableSpin |= axis.role == lcnc::MachineAxisRole::TableSpin;
        headPrimary |= axis.role == lcnc::MachineAxisRole::HeadTiltPrimary;
        headSecondary |= axis.role == lcnc::MachineAxisRole::HeadTiltSecondary;
    }
    QStringList modes{lcnc::machiningModeName(lcnc::MachiningMode::Planar3Axis)};
    if (workpieceRotary || (tableTilt && tableSpin))
        modes.append(lcnc::machiningModeName(lcnc::MachiningMode::RotaryTube4Axis));
    if (tableTilt && tableSpin)
        modes.append(lcnc::machiningModeName(lcnc::MachiningMode::SimultaneousTable5Axis));
    if (headPrimary && headSecondary)
        modes.append(lcnc::machiningModeName(lcnc::MachiningMode::SimultaneousHead5Axis));
    return modes.join(QStringLiteral(" / "));
}

QString machineAxisRoleDisplayName(lcnc::MachineAxisRole role)
{
    switch (role) {
    case lcnc::MachineAxisRole::LinearX: return QCoreApplication::translate("DialogOptions", "Linear X"); // 中文翻译：X 直线轴
    case lcnc::MachineAxisRole::LinearY: return QCoreApplication::translate("DialogOptions", "Linear Y"); // 中文翻译：Y 直线轴
    case lcnc::MachineAxisRole::LinearZ: return QCoreApplication::translate("DialogOptions", "Linear Z"); // 中文翻译：Z 直线轴
    case lcnc::MachineAxisRole::WorkpieceRotary: return QCoreApplication::translate("DialogOptions", "Workpiece rotary"); // 中文翻译：工件回转轴
    case lcnc::MachineAxisRole::TableTilt: return QCoreApplication::translate("DialogOptions", "Table tilt"); // 中文翻译：转台倾斜轴
    case lcnc::MachineAxisRole::TableSpin: return QCoreApplication::translate("DialogOptions", "Table spin"); // 中文翻译：转台回转轴
    case lcnc::MachineAxisRole::HeadTiltPrimary: return QCoreApplication::translate("DialogOptions", "Primary head tilt"); // 中文翻译：第一摆头轴
    case lcnc::MachineAxisRole::HeadTiltSecondary: return QCoreApplication::translate("DialogOptions", "Secondary head tilt"); // 中文翻译：第二摆头轴
    default: return QCoreApplication::translate("DialogOptions", "Unspecified"); // 中文翻译：未指定
    }
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
            || lhs.role != rhs.role
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
        && qFuzzyCompare(a.workpieceTransparency, b.workpieceTransparency)
        && qFuzzyCompare(a.machineTransparency, b.machineTransparency)
        && a.machineAxisColors == b.machineAxisColors
        && a.cutterHeadColor == b.cutterHeadColor
        && qFuzzyCompare(a.cutterHeadTransparency, b.cutterHeadTransparency)
        && qFuzzyCompare(a.cutterHeadScale, b.cutterHeadScale);
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
    // 中文翻译：应用程序选项
    setWindowTitle(tr("Application Options"));
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
    // 中文翻译：视图渲染
    auto* itemRender = new QTreeWidgetItem(m_nav, QStringList(tr("View rendering")));
    itemRender->setData(0, Qt::UserRole, 0);
    // 中文翻译：颜色配置
    auto* itemColors = new QTreeWidgetItem(m_nav, QStringList(tr("Color configuration")));
    itemColors->setData(0, Qt::UserRole, 1);
    // 中文翻译：应用程序
    auto* itemApp = new QTreeWidgetItem(m_nav, QStringList(tr("application")));
    itemApp->setData(0, Qt::UserRole, 2);
    // 中文翻译：机台构型
    auto* itemMachine = new QTreeWidgetItem(m_nav, QStringList(tr("Machine configuration")));
    itemMachine->setData(0, Qt::UserRole, 3);

    // 中文翻译：视图渲染
    buildRenderPage(tr("View rendering"), true, m_renderControls);
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
    // 中文翻译：线框
    c.defaultDisplay->addItem(tr("Wireframe"), static_cast<int>(StartupDisplayMode::Wireframe));
    // 中文翻译：着色
    c.defaultDisplay->addItem(tr("Coloring"), static_cast<int>(StartupDisplayMode::Shaded));
    // 中文翻译：带边着色
    c.defaultDisplay->addItem(tr("Shading with edges"), static_cast<int>(StartupDisplayMode::ShadedWithEdges));
    // 中文翻译：启动/新 View 默认显示模式:
    displayForm->addRow(tr("Start/new View default display mode:"), c.defaultDisplay);

    c.quality = new QComboBox(displayGroup);
    // 中文翻译：低 (Low)
    c.quality->addItem(tr("Low"), static_cast<int>(RenderQualityPreset::Low));
    // 中文翻译：中 (Medium)
    c.quality->addItem(tr("Medium"), static_cast<int>(RenderQualityPreset::Medium));
    // 中文翻译：高 (High)
    c.quality->addItem(tr("High"), static_cast<int>(RenderQualityPreset::High));
    // 中文翻译：自定义 (Custom)
    c.quality->addItem(tr("Custom"), static_cast<int>(RenderQualityPreset::Custom));
    // 中文翻译：质量预设:
    displayForm->addRow(tr("Quality preset:"), c.quality);

    c.renderMethod = new QComboBox(displayGroup);
    // 中文翻译：光栅化
    c.renderMethod->addItem(tr("rasterization"), static_cast<int>(RenderMethod::Rasterization));
    // 中文翻译：光线追踪
    c.renderMethod->addItem(tr("Ray tracing"), static_cast<int>(RenderMethod::RayTracing));
    // 中文翻译：渲染方法:
    displayForm->addRow(tr("Rendering method:"), c.renderMethod);

    c.material = new QComboBox(displayGroup);
    // 中文翻译：塑料
    c.material->addItem(tr("plastic"), QStringLiteral("plastic"));
    // 中文翻译：亮塑料
    c.material->addItem(tr("bright plastic"), QStringLiteral("shiny_plastic"));
    // 中文翻译：钢
    c.material->addItem(tr("steel"), QStringLiteral("steel"));
    // 中文翻译：铝
    c.material->addItem(tr("Aluminum"), QStringLiteral("aluminum"));
    // 中文翻译：金属
    c.material->addItem(tr("metal"), QStringLiteral("metal"));
    // 中文翻译：铬
    c.material->addItem(tr("Chromium"), QStringLiteral("chrome"));
    // 中文翻译：缎面
    c.material->addItem(tr("satin"), QStringLiteral("satin"));
    // 中文翻译：模型材质:
    displayForm->addRow(tr("Model material:"), c.material);
    root->addWidget(displayGroup);

    // 中文翻译：性能 / 质量
    auto* performanceGroup = new QGroupBox(tr("Performance/Quality"), content);
    auto* performanceForm = new QFormLayout(performanceGroup);
    // 中文翻译：启用抗锯齿
    c.antiAliasing = new QCheckBox(tr("Enable anti-aliasing"), performanceGroup);
    performanceForm->addRow(QString(), c.antiAliasing);
    c.msaaSamples = new QComboBox(performanceGroup);
    // 中文翻译：关闭
    c.msaaSamples->addItem(tr("Close"), 0);
    c.msaaSamples->addItem(tr("2x"), 2);
    c.msaaSamples->addItem(tr("4x"), 4);
    c.msaaSamples->addItem(tr("8x"), 8);
    performanceForm->addRow(tr("MSAA:"), c.msaaSamples);
    c.renderResolutionScale = noWheel(new QDoubleSpinBox(performanceGroup));
    c.renderResolutionScale->setRange(0.25, 2.0);
    c.renderResolutionScale->setSingleStep(0.05);
    c.renderResolutionScale->setDecimals(2);
    // 中文翻译：渲染分辨率比例:
    performanceForm->addRow(tr("Rendering resolution ratio:"), c.renderResolutionScale);
    c.deviationCoefficient = noWheel(new QDoubleSpinBox(performanceGroup));
    c.deviationCoefficient->setRange(0.001, 1.0);
    c.deviationCoefficient->setSingleStep(0.005);
    c.deviationCoefficient->setDecimals(3);
    // 中文翻译：LOD 偏差系数:
    performanceForm->addRow(tr("LOD deviation coefficient:"), c.deviationCoefficient);
    c.deviationAngle = noWheel(new QDoubleSpinBox(performanceGroup));
    c.deviationAngle->setRange(0.01, 2.0);
    c.deviationAngle->setSingleStep(0.05);
    c.deviationAngle->setDecimals(3);
    // 中文翻译：LOD 角度:
    performanceForm->addRow(tr("LOD angle:"), c.deviationAngle);
    c.edgeWidth = noWheel(new QDoubleSpinBox(performanceGroup));
    c.edgeWidth->setRange(0.1, 5.0);
    c.edgeWidth->setSingleStep(0.1);
    c.edgeWidth->setDecimals(1);
    // 中文翻译：边线宽度:
    performanceForm->addRow(tr("Edge width:"), c.edgeWidth);
    root->addWidget(performanceGroup);

    // 中文翻译：渲染特性
    auto* renderGroup = new QGroupBox(tr("Rendering properties"), content);
    auto* renderForm = new QFormLayout(renderGroup);
    // 中文翻译：阴影
    c.shadows = new QCheckBox(tr("shadow"), renderGroup);
    // 中文翻译：反射
    c.reflections = new QCheckBox(tr("reflection"), renderGroup);
    // 中文翻译：自适应采样
    c.adaptiveSampling = new QCheckBox(tr("adaptive sampling"), renderGroup);
    // 中文翻译：视锥体裁剪
    c.frustumCulling = new QCheckBox(tr("frustum clipping"), renderGroup);
    // 中文翻译：背面剔除
    c.backFaceCulling = new QCheckBox(tr("Backface culling"), renderGroup);
    // 中文翻译：几何合并（机台代理/压缩路径）
    c.geometryMerge = new QCheckBox(tr("Geometry merging (machine proxy/compression path)"), renderGroup);
    // 中文翻译：代理几何
    c.proxyGeometry = new QCheckBox(tr("proxy geometry"), renderGroup);
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
    // 中文翻译：环境光:
    renderForm->addRow(tr("Ambient light:"), c.ambientLight);
    root->addWidget(renderGroup);

    // 中文翻译：光线追踪 / 采样
    auto* rayGroup = new QGroupBox(tr("Ray tracing/sampling"), content);
    auto* rayForm = new QFormLayout(rayGroup);
    c.raytracingDepth = noWheel(new QSpinBox(rayGroup));
    c.raytracingDepth->setRange(1, 8);
    // 中文翻译：光追深度:
    rayForm->addRow(tr("Ray tracing depth:"), c.raytracingDepth);
    c.rayTracingTileSize = noWheel(new QSpinBox(rayGroup));
    c.rayTracingTileSize->setRange(8, 128);
    c.rayTracingTileSize->setSingleStep(8);
    // 中文翻译：Tile 大小:
    rayForm->addRow(tr("Tile size:"), c.rayTracingTileSize);
    c.rayTracingTileCount = noWheel(new QSpinBox(rayGroup));
    c.rayTracingTileCount->setRange(1, 1024);
    // 中文翻译：每帧 Tile 数:
    rayForm->addRow(tr("Number of Tiles per frame:"), c.rayTracingTileCount);
    root->addWidget(rayGroup);

    // 中文翻译：仿真效率
    auto* simulationGroup = new QGroupBox(tr("Simulation efficiency"), content);
    auto* simulationForm = new QFormLayout(simulationGroup);
    c.targetFps = noWheel(new QSpinBox(simulationGroup));
    c.targetFps->setRange(15, 240);
    // 中文翻译：目标帧率:
    simulationForm->addRow(tr("Target frame rate:"), c.targetFps);
    // 中文翻译：运动中使用低 LOD
    c.lowLodWhileMoving = new QCheckBox(tr("Use low LOD in motion"), simulationGroup);
    // 中文翻译：仿真时禁用阴影/反射等重效果
    c.disableHeavyEffectsDuringSimulation = new QCheckBox(tr("Disable shadow/reflection effects during simulation"), simulationGroup);
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

    // 中文翻译：模型与背景
    auto* modelGroup = new QGroupBox(tr("Models and backgrounds"), page);
    auto* modelForm = new QFormLayout(modelGroup);
    m_btnWorkpieceColor = makeColorButton(&m_colorDraft.workpieceColor);
    m_btnBackgroundColor = makeColorButton(&m_colorDraft.backgroundColor);
    m_spWorkpieceTransparency = noWheel(new QDoubleSpinBox(modelGroup));
    m_spWorkpieceTransparency->setRange(0.0, 100.0);
    m_spWorkpieceTransparency->setDecimals(0);
    m_spWorkpieceTransparency->setSingleStep(5.0);
    m_spWorkpieceTransparency->setSuffix(tr(" %"));
    m_spMachineTransparency = noWheel(new QDoubleSpinBox(modelGroup));
    m_spMachineTransparency->setRange(0.0, 100.0);
    m_spMachineTransparency->setDecimals(0);
    m_spMachineTransparency->setSingleStep(5.0);
    m_spMachineTransparency->setSuffix(tr(" %"));
    // 中文翻译：工件颜色:
    modelForm->addRow(tr("Work piece color:"), m_btnWorkpieceColor);
    // 中文翻译：工件模型透明度:
    modelForm->addRow(tr("Workpiece model transparency:"), m_spWorkpieceTransparency);
    // 中文翻译：机台模型透明度:
    modelForm->addRow(tr("Machine model transparency:"), m_spMachineTransparency);
    // 中文翻译：视图背景:
    modelForm->addRow(tr("View background:"), m_btnBackgroundColor);
    root->addWidget(modelGroup);

    // 中文翻译：机台分轴颜色
    auto* axisGroup = new QGroupBox(tr("Machine axis color"), page);
    auto* axisForm = new QFormLayout(axisGroup);
    for (const char* axis : kAxes) {
        const QString name(axis);
        auto* btn = new QPushButton(page);
        btn->setMinimumWidth(150);
        connect(btn, &QPushButton::clicked, this, [this, btn, name] {
            const QColor current = m_colorDraft.machineAxisColors.value(name);
            // 中文翻译：选择颜色
            const QColor picked = QColorDialog::getColor(current, this, tr("Choose color"));
            if (!picked.isValid())
                return;
            m_colorDraft.machineAxisColors.insert(name, picked);
            styleColorButton(btn, picked);
        });
        m_axisColorButtons.insert(name, btn);
        axisForm->addRow(name, btn);
    }
    root->addWidget(axisGroup);

    // 中文翻译：选择 / 悬停 / 树节点
    auto* highlightGroup = new QGroupBox(tr("Select / Hover / Tree Node"), page);
    auto* highlightForm = new QFormLayout(highlightGroup);
    m_btnSelectionColor = makeColorButton(&m_colorDraft.selectionColor);
    m_btnHoverColor = makeColorButton(&m_colorDraft.hoverColor);
    m_btnTreeSelectionColor = makeColorButton(&m_colorDraft.treeSelectionColor);
    m_cbHighlightMode = new QComboBox(highlightGroup);
    // 中文翻译：沿用对象 displayMode
    m_cbHighlightMode->addItem(tr("Inherit object displayMode"), -1);
    // 中文翻译：线框高亮
    m_cbHighlightMode->addItem(tr("Wireframe highlighting"), 0);
    // 中文翻译：着色高亮
    m_cbHighlightMode->addItem(tr("shading highlight"), 1);
    m_spHighlightLineWidth = noWheel(new QDoubleSpinBox(highlightGroup));
    m_spHighlightLineWidth->setRange(0.5, 10.0);
    m_spHighlightLineWidth->setSingleStep(0.5);
    m_spHighlightLineWidth->setDecimals(1);
    // 中文翻译：选中高亮色:
    highlightForm->addRow(tr("Select highlight color:"), m_btnSelectionColor);
    // 中文翻译：悬停高亮色:
    highlightForm->addRow(tr("Hover highlight color:"), m_btnHoverColor);
    // 中文翻译：树节点选中色:
    highlightForm->addRow(tr("Tree node selection color:"), m_btnTreeSelectionColor);
    // 中文翻译：高亮模式:
    highlightForm->addRow(tr("Highlight mode:"), m_cbHighlightMode);
    // 中文翻译：高亮线宽:
    highlightForm->addRow(tr("Highlight line width:"), m_spHighlightLineWidth);
    root->addWidget(highlightGroup);

    // 中文翻译：刀头
    auto* cutterHeadGroup = new QGroupBox(tr("Cutter head"), page);
    auto* cutterHeadForm = new QFormLayout(cutterHeadGroup);
    m_btnCutterHeadColor = makeColorButton(&m_colorDraft.cutterHeadColor);
    m_spCutterHeadTransparency = noWheel(new QDoubleSpinBox(cutterHeadGroup));
    m_spCutterHeadTransparency->setRange(0.0, 100.0);
    m_spCutterHeadTransparency->setDecimals(0);
    m_spCutterHeadTransparency->setSingleStep(5.0);
    m_spCutterHeadTransparency->setSuffix(tr(" %"));
    m_spCutterHeadScale = noWheel(new QDoubleSpinBox(cutterHeadGroup));
    m_spCutterHeadScale->setRange(0.2, 3.0);
    m_spCutterHeadScale->setDecimals(2);
    m_spCutterHeadScale->setSingleStep(0.1);
    // 中文翻译：刀头颜色:
    cutterHeadForm->addRow(tr("Cutter head color:"), m_btnCutterHeadColor);
    // 中文翻译：刀头透明度:
    cutterHeadForm->addRow(tr("Cutter head transparency:"), m_spCutterHeadTransparency);
    // 中文翻译：刀头大小(缩放):
    cutterHeadForm->addRow(tr("Cutter head size (scale):"), m_spCutterHeadScale);
    root->addWidget(cutterHeadGroup);

    root->addStretch(1);
    m_stack->addWidget(page);
}

void DialogOptions::buildApplicationPage()
{
    auto* page = new QWidget(this);
    auto* root = new QVBoxLayout(page);
    // 中文翻译：通用
    auto* group = new QGroupBox(tr("Universal"), page);
    auto* form = new QFormLayout(group);

    m_cbLanguage = new QComboBox(group);
    // 中文翻译：简体中文
    m_cbLanguage->addItem(tr("Chinese (Simplified)"), QStringLiteral("zh_CN"));
    // 中文翻译：English
    m_cbLanguage->addItem(tr("English"), QStringLiteral("en"));
    // 中文翻译：语言：
    form->addRow(tr("Language:"), m_cbLanguage);

    m_cbTheme = new QComboBox(group);
    // 中文翻译：浅色
    m_cbTheme->addItem(tr("Light"), QStringLiteral("light"));
    // 中文翻译：深色
    m_cbTheme->addItem(tr("Dark"), QStringLiteral("dark"));
    // 中文翻译：主题：
    form->addRow(tr("Theme:"), m_cbTheme);

    m_cbUnits = new QComboBox(group);
    // 中文翻译：毫米 (mm)
    m_cbUnits->addItem(tr("Millimetres (mm)"), QStringLiteral("mm"));
    // 中文翻译：英寸 (in)
    m_cbUnits->addItem(tr("Inches (in)"), QStringLiteral("inch"));
    // 中文翻译：单位制：
    form->addRow(tr("Units:"), m_cbUnits);

    m_cbDocumentOpenMode = new QComboBox(group);
    // 中文翻译：单文档（打开新文件时关闭当前文件）
    m_cbDocumentOpenMode->addItem(tr("Single document (close the current document when opening a file)"),
                                  static_cast<int>(DocumentOpenMode::SingleDocument));
    // 中文翻译：多文档（保留多个项目工作区）
    m_cbDocumentOpenMode->addItem(tr("Multiple documents (keep project workspaces open)"),
                                  static_cast<int>(DocumentOpenMode::MultiDocument));
    // 中文翻译：打开模式：
    form->addRow(tr("Open mode:"), m_cbDocumentOpenMode);

    m_spRecentLimit = noWheel(new QSpinBox(group));
    m_spRecentLimit->setRange(1, 50);
    // 中文翻译：最近文件数：
    form->addRow(tr("Recent files:"), m_spRecentLimit);

    // 中文翻译：语言和主题修改后，重启应用程序即可完全生效。
    // 语言切换在下一次启动时加载，避免在运行中重建含状态的加工 UI。
    auto* hint = new QLabel(tr("Language and theme changes take full effect after restarting the application."), group);
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

    // 中文翻译：机台构型
    auto* group = new QGroupBox(tr("Machine configuration"), page);
    auto* form = new QFormLayout(group);
    m_cbMachinePreset = new QComboBox(group);
    for (const QString& preset : machinePresetNames())
        m_cbMachinePreset->addItem(machinePresetText(preset), preset);
    // 中文翻译：构型
    form->addRow(tr("configuration"), m_cbMachinePreset);

    auto* pathRow = new QWidget(group);
    auto* pathLayout = new QHBoxLayout(pathRow);
    pathLayout->setContentsMargins(0, 0, 0, 0);
    pathLayout->setSpacing(4);
    m_editMachineModelPath = new QLineEdit(pathRow);
    m_editMachineModelPath->setClearButtonEnabled(true);
    // 中文翻译：选择或输入机台模型文件路径
    m_editMachineModelPath->setPlaceholderText(tr("Select or enter the machine model file path"));
    // 中文翻译：浏览...
    m_btnBrowseMachineModel = new QPushButton(tr("Browse..."), pathRow);
    pathLayout->addWidget(m_editMachineModelPath, 1);
    pathLayout->addWidget(m_btnBrowseMachineModel);
    // 中文翻译：机台模型路径
    form->addRow(tr("Machine model path"), pathRow);

    // 中文翻译：启动时自动加载机台模型
    m_chkAutoLoadMachineModel = new QCheckBox(tr("Automatically load the machine model at startup"), group);
    form->addRow(QString(), m_chkAutoLoadMachineModel);

    m_lblMachineAlgorithm = new QLabel(group);
    m_lblMachineAlgorithm->setTextInteractionFlags(Qt::TextSelectableByMouse);
    // 中文翻译：刀路算法
    form->addRow(tr("Tool path algorithm"), m_lblMachineAlgorithm);
    root->addWidget(group);

    // 中文翻译：切割嘴碰撞代理
    auto* nozzleGroup = new QGroupBox(tr("Cutting nozzle collision proxy"), page);
    auto* nozzleForm = new QFormLayout(nozzleGroup);
    m_cbCutterCollisionProxyMode = new QComboBox(nozzleGroup);
    // 中文翻译：模拟锥头
    m_cbCutterCollisionProxyMode->addItem(tr("Simulated cone"), 0);
    // 中文翻译：刀嘴模型文件
    m_cbCutterCollisionProxyMode->addItem(tr("Nozzle model file"), 1);
    // 中文翻译：代理类型
    nozzleForm->addRow(tr("Proxy type"), m_cbCutterCollisionProxyMode);

    auto* nozzlePathRow = new QWidget(nozzleGroup);
    auto* nozzlePathLayout = new QHBoxLayout(nozzlePathRow);
    nozzlePathLayout->setContentsMargins(0, 0, 0, 0);
    nozzlePathLayout->setSpacing(4);
    m_editCutterNozzleModelPath = new QLineEdit(nozzlePathRow);
    m_editCutterNozzleModelPath->setClearButtonEnabled(true);
    // 中文翻译：选择轻量化切割嘴模型文件
    m_editCutterNozzleModelPath->setPlaceholderText(tr("Select a lightweight cutting nozzle model"));
    // 中文翻译：浏览...
    m_btnBrowseCutterNozzleModel = new QPushButton(tr("Browse..."), nozzlePathRow);
    nozzlePathLayout->addWidget(m_editCutterNozzleModelPath, 1);
    nozzlePathLayout->addWidget(m_btnBrowseCutterNozzleModel);
    // 中文翻译：刀嘴模型路径
    nozzleForm->addRow(tr("Nozzle model path"), nozzlePathRow);

    const auto positiveDistanceSpin = [nozzleGroup](double minimum, double maximum) {
        auto* spin = noWheel(new QDoubleSpinBox(nozzleGroup));
        spin->setRange(minimum, maximum);
        spin->setDecimals(3);
        spin->setSuffix(QStringLiteral(" mm"));
        return spin;
    };
    m_spSimulatedConeLength = positiveDistanceSpin(0.1, 1000.0);
    m_spSimulatedConeTipRadius = positiveDistanceSpin(0.0, 100.0);
    m_spSimulatedConeBaseRadius = positiveDistanceSpin(0.0, 500.0);
    m_spCutterCollisionClearance = positiveDistanceSpin(0.0, 100.0);
    m_spMaximumRapidSafetyOffset = positiveDistanceSpin(0.1, 10000.0);
    // 中文翻译：锥头长度；尖端半径；底部半径；最小安全间隙；最大安全抬高量
    nozzleForm->addRow(tr("Cone length"), m_spSimulatedConeLength);
    nozzleForm->addRow(tr("Tip radius"), m_spSimulatedConeTipRadius);
    nozzleForm->addRow(tr("Base radius"), m_spSimulatedConeBaseRadius);
    nozzleForm->addRow(tr("Minimum clearance"), m_spCutterCollisionClearance);
    nozzleForm->addRow(tr("Maximum safety offset"), m_spMaximumRapidSafetyOffset);
    auto* nozzleHint = new QLabel(
        // 中文翻译：模型会自动将包围盒最低点作为刀嘴尖端，并以局部 +Z 作为远离加工面的方向。碰撞求高在生成刀路时完成。
        tr("The model bounding-box minimum is treated as the nozzle tip and local +Z points away from the machining surface. Collision height is solved while generating the tool path."),
        nozzleGroup);
    nozzleHint->setWordWrap(true);
    nozzleHint->setStyleSheet("color:#666;");
    nozzleForm->addRow(nozzleHint);
    root->addWidget(nozzleGroup);

    // 中文翻译：旋转中心
    auto* centerGroup = new QGroupBox(tr("center of rotation"), page);
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
    // 中文翻译：中心坐标
    centerForm->addRow(tr("Center coordinates"), centerRow);
    m_lblRotationCenterHint = new QLabel(centerGroup);
    m_lblRotationCenterHint->setWordWrap(true);
    m_lblRotationCenterHint->setStyleSheet("color:#666;");
    centerForm->addRow(m_lblRotationCenterHint);
    root->addWidget(centerGroup);

    // 中文翻译：摆头软件 TCP
    auto* headTcpGroup = new QGroupBox(tr("Head software TCP"), page);
    auto* headTcpForm = new QFormLayout(headTcpGroup);
    const QStringList tcpLabels = {
        tr("Zero beam X"), tr("Zero beam Y"), tr("Zero beam Z"), tr("Focus length"),
        tr("Installation offset X"), tr("Installation offset Y"), tr("Installation offset Z")};
    for (int index = 0; index < 7; ++index) {
        m_headTcpEditors[index] = machineCoordinateSpin(headTcpGroup);
        if (index == 3) m_headTcpEditors[index]->setRange(0.0, 10000.0);
        headTcpForm->addRow(tcpLabels[index], m_headTcpEditors[index]);
    }
    root->addWidget(headTcpGroup);

    m_machineAxesTable = new QTableWidget(page);
    m_machineAxesTable->setColumnCount(10);
    m_machineAxesTable->setHorizontalHeaderLabels({
        // 中文翻译：轴名；类型；父轴；方向X；方向Y；方向Z
        tr("Axis name"), tr("Type"), tr("Axis role"), tr("parent axis"), tr("DirectionX"), tr("Direction Y"), tr("Direction Z"),
        // 中文翻译：原点X；原点Y；原点Z
        tr("OriginX"), tr("Origin Y"), tr("Origin Z")
    });
    m_machineAxesTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    m_machineAxesTable->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
    m_machineAxesTable->verticalHeader()->setVisible(false);
    m_machineAxesTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_machineAxesTable->setSelectionMode(QAbstractItemView::SingleSelection);
    m_machineAxesTable->setAlternatingRowColors(true);
    root->addWidget(m_machineAxesTable, 1);
    auto* coordinateHint = new QLabel(
        // 中文翻译：线性 X/Y/Z 轴的方向同时用于机台模型运动和视图坐标提示。
        tr("The directions of the linear X/Y/Z axes are used for both machine model motion and view coordinate prompts."
           // 中文翻译：例如 Z 轴零点在上方且向下为正时，将 Z 方向设为 (0, 0, -1)。
           "For example, when the Z-axis zero point is above and downward is positive, set the Z direction to (0, 0, -1)."
           // 中文翻译：坐标三轴提示需要 X/Y/Z 构成正交右手系。
           "The coordinate three-axis prompt requires X/Y/Z to form an orthogonal right-handed system."), page);
    coordinateHint->setWordWrap(true);
    coordinateHint->setStyleSheet("color:#666;");
    root->addWidget(coordinateHint);

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
                    // 中文翻译：选择机台模型文件
                    tr("Select machine model file"),
                    dir,
                    // 中文翻译：三维模型文件 (*.stp *.step *.stl *.brep);;STEP (*.stp *.step);;STL (*.stl);;BREP (*.brep);;所有文件 (*)
                    tr("3D model files (*.stp *.step *.stl *.brep);;STEP (*.stp *.step);;STL (*.stl);;BREP (*.brep);;All files (*)"));
                if (!path.isEmpty() && m_editMachineModelPath)
                    m_editMachineModelPath->setText(QFileInfo(path).absoluteFilePath());
            });
    const auto updateNozzleMode = [this] {
        const bool modelMode = m_cbCutterCollisionProxyMode
            && m_cbCutterCollisionProxyMode->currentData().toInt() == 1;
        if (m_editCutterNozzleModelPath) m_editCutterNozzleModelPath->setEnabled(modelMode);
        if (m_btnBrowseCutterNozzleModel) m_btnBrowseCutterNozzleModel->setEnabled(modelMode);
        if (m_spSimulatedConeLength) m_spSimulatedConeLength->setEnabled(!modelMode);
        if (m_spSimulatedConeTipRadius) m_spSimulatedConeTipRadius->setEnabled(!modelMode);
        if (m_spSimulatedConeBaseRadius) m_spSimulatedConeBaseRadius->setEnabled(!modelMode);
    };
    connect(m_cbCutterCollisionProxyMode, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, updateNozzleMode);
    connect(m_btnBrowseCutterNozzleModel, &QPushButton::clicked, this,
            [this] {
                const QString currentPath = m_editCutterNozzleModelPath
                    ? m_editCutterNozzleModelPath->text().trimmed() : QString();
                const QString dir = currentPath.isEmpty()
                    ? QString() : QFileInfo(currentPath).absolutePath();
                const QString path = QFileDialog::getOpenFileName(
                    this,
                    // 中文翻译：选择切割嘴碰撞模型
                    tr("Select cutting nozzle collision model"), dir,
                    // 中文翻译：三维模型文件 (*.stp *.step *.stl *.brep);;所有文件 (*)
                    tr("3D model files (*.stp *.step *.stl *.brep);;All files (*)"));
                if (!path.isEmpty() && m_editCutterNozzleModelPath)
                    m_editCutterNozzleModelPath->setText(QFileInfo(path).absoluteFilePath());
            });
    updateNozzleMode();

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
        // 中文翻译：线性
        typeCombo->addItem(tr("Linear"), static_cast<int>(MachineAxisDef::Linear));
        // 中文翻译：旋转
        typeCombo->addItem(tr("rotate"), static_cast<int>(MachineAxisDef::Rotary));
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

        auto* roleCombo = new QComboBox(m_machineAxesTable);
        for (lcnc::MachineAxisRole role : {
                 lcnc::MachineAxisRole::LinearX, lcnc::MachineAxisRole::LinearY,
                 lcnc::MachineAxisRole::LinearZ, lcnc::MachineAxisRole::WorkpieceRotary,
                 lcnc::MachineAxisRole::TableTilt, lcnc::MachineAxisRole::TableSpin,
                 lcnc::MachineAxisRole::HeadTiltPrimary, lcnc::MachineAxisRole::HeadTiltSecondary}) {
            roleCombo->addItem(machineAxisRoleDisplayName(role), static_cast<int>(role));
        }
        const int roleIndex = roleCombo->findData(static_cast<int>(config.axis.role));
        roleCombo->setCurrentIndex(roleIndex >= 0 ? roleIndex : 0);
        m_machineAxesTable->setCellWidget(row, 2, roleCombo);

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
        m_machineAxesTable->setCellWidget(row, 3, parentCombo);

        m_machineAxesTable->setItem(row, 4, machineAxisTableItem(QString::number(config.axis.direction.X(), 'g', 15)));
        m_machineAxesTable->setItem(row, 5, machineAxisTableItem(QString::number(config.axis.direction.Y(), 'g', 15)));
        m_machineAxesTable->setItem(row, 6, machineAxisTableItem(QString::number(config.axis.direction.Z(), 'g', 15)));
        m_machineAxesTable->setItem(row, 7, machineAxisTableItem(QString::number(config.axis.origin.X(), 'g', 15)));
        m_machineAxesTable->setItem(row, 8, machineAxisTableItem(QString::number(config.axis.origin.Y(), 'g', 15)));
        m_machineAxesTable->setItem(row, 9, machineAxisTableItem(QString::number(config.axis.origin.Z(), 'g', 15)));
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
            axis.role = static_cast<lcnc::MachineAxisRole>(combo->currentData().toInt());
        if (auto* combo = qobject_cast<QComboBox*>(m_machineAxesTable->cellWidget(row, 3)))
            axis.parentAxis = combo->currentData().toString();
        if (axis.parentAxis.trimmed().isEmpty())
            axis.parentAxis = QStringLiteral("BASE");

        const gp_Dir fallbackDirection = defaultMachineAxisDirection(name, axis.motionType);
        const double dx = itemDouble(row, 4, fallbackDirection.X());
        const double dy = itemDouble(row, 5, fallbackDirection.Y());
        const double dz = itemDouble(row, 6, fallbackDirection.Z());
        const double norm2 = dx * dx + dy * dy + dz * dz;
        axis.direction = norm2 > 1e-12 ? gp_Dir(dx, dy, dz) : fallbackDirection;
        axis.origin = gp_Pnt(itemDouble(row, 7, 0.0),
                             itemDouble(row, 8, 0.0),
                             itemDouble(row, 9, 0.0));
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
                // 中文翻译：该坐标会写入旋转轴 %1 的原点；AC 转台请填写 A 轴与 C 轴的物理交点。
                tr("This coordinate will be written as the origin of the rotation axis %1; for AC turntable, please fill in the physical intersection point of the A-axis and C-axis.")
                    .arg(rotaryNames.join(QStringLiteral("/"))));
        } else {
            // 中文翻译：当前构型没有旋转轴，不需要填写旋转中心。
            m_lblRotationCenterHint->setText(tr("The current configuration does not have an axis of rotation, so there is no need to fill in the center of rotation."));
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
        if (QTableWidgetItem* item = m_machineAxesTable->item(row, 7)) item->setText(x);
        if (QTableWidgetItem* item = m_machineAxesTable->item(row, 8)) item->setText(y);
        if (QTableWidgetItem* item = m_machineAxesTable->item(row, 9)) item->setText(z);
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
        m_originalCutterCollisionProxyMode =
            static_cast<int>(cam->config().cutterCollisionProxyMode());
        m_originalCutterNozzleModelPath = cam->config().cutterNozzleModelPath();
        m_originalSimulatedConeLength = cam->config().simulatedConeLengthMm();
        m_originalSimulatedConeTipRadius = cam->config().simulatedConeTipRadiusMm();
        m_originalSimulatedConeBaseRadius = cam->config().simulatedConeBaseRadiusMm();
        m_originalCutterCollisionClearance = cam->config().cutterCollisionClearanceMm();
        m_originalMaximumRapidSafetyOffset = cam->config().maximumRapidSafetyOffsetMm();
    } else {
        m_originalMachineModelPath.clear();
        m_originalAutoLoadMachineModel = true;
    }

    m_renderDraft = m_originalCam;
    m_colorDraft = m_originalColors;
    if (m_colorDraft.machineAxisColors.isEmpty())
        m_colorDraft.machineAxisColors = defaultMachineAxisColors();

    setProfileToUi(m_renderDraft, m_renderControls);

    styleColorButton(m_btnWorkpieceColor, m_colorDraft.workpieceColor);
    styleColorButton(m_btnBackgroundColor, m_colorDraft.backgroundColor);
    m_spWorkpieceTransparency->setValue(m_colorDraft.workpieceTransparency * 100.0);
    m_spMachineTransparency->setValue(m_colorDraft.machineTransparency * 100.0);
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
    styleColorButton(m_btnCutterHeadColor, m_colorDraft.cutterHeadColor);
    m_spCutterHeadTransparency->setValue(m_colorDraft.cutterHeadTransparency * 100.0);
    m_spCutterHeadScale->setValue(m_colorDraft.cutterHeadScale);

    setComboByData(m_cbLanguage, settings->language);
    setComboByData(m_cbTheme, settings->theme);
    setComboByData(m_cbUnits, settings->unitSystem);
    setComboByData(m_cbDocumentOpenMode, static_cast<int>(settings->documentOpenMode));
    m_spRecentLimit->setValue(settings->recentLimit);
    if (m_editMachineModelPath)
        m_editMachineModelPath->setText(m_originalMachineModelPath);
    if (m_chkAutoLoadMachineModel)
        m_chkAutoLoadMachineModel->setChecked(m_originalAutoLoadMachineModel);
    setComboByData(m_cbCutterCollisionProxyMode, m_originalCutterCollisionProxyMode);
    if (m_editCutterNozzleModelPath)
        m_editCutterNozzleModelPath->setText(m_originalCutterNozzleModelPath);
    if (m_spSimulatedConeLength) m_spSimulatedConeLength->setValue(m_originalSimulatedConeLength);
    if (m_spSimulatedConeTipRadius) m_spSimulatedConeTipRadius->setValue(m_originalSimulatedConeTipRadius);
    if (m_spSimulatedConeBaseRadius) m_spSimulatedConeBaseRadius->setValue(m_originalSimulatedConeBaseRadius);
    if (m_spCutterCollisionClearance) m_spCutterCollisionClearance->setValue(m_originalCutterCollisionClearance);
    if (m_spMaximumRapidSafetyOffset) m_spMaximumRapidSafetyOffset->setValue(m_originalMaximumRapidSafetyOffset);
    if (m_machineConfig) {
        m_originalMachinePreset = m_machineConfig->presetName();
        m_originalMachineConfigs = m_machineConfig->axisConfigurations();
        setComboByData(m_cbMachinePreset, m_originalMachinePreset);
        populateMachineAxisTable(m_originalMachineConfigs);
        m_originalHeadToolGeometry = m_machineConfig->headToolGeometry();
        const std::array<double, 7> values{
            m_originalHeadToolGeometry.zeroBeamX, m_originalHeadToolGeometry.zeroBeamY,
            m_originalHeadToolGeometry.zeroBeamZ, m_originalHeadToolGeometry.focusLength,
            m_originalHeadToolGeometry.installationOffsetX,
            m_originalHeadToolGeometry.installationOffsetY,
            m_originalHeadToolGeometry.installationOffsetZ};
        for (int index = 0; index < 7; ++index) m_headTcpEditors[index]->setValue(values[index]);
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
        // 中文翻译：选择颜色
        const QColor picked = QColorDialog::getColor(*target, this, tr("Choose color"));
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
    m_colorDraft.workpieceTransparency = m_spWorkpieceTransparency->value() / 100.0;
    m_colorDraft.machineTransparency = m_spMachineTransparency->value() / 100.0;
    m_colorDraft.highlightDisplayMode = m_cbHighlightMode->currentData().toInt();
    m_colorDraft.highlightLineWidth = m_spHighlightLineWidth->value();
    m_colorDraft.cutterHeadTransparency = m_spCutterHeadTransparency->value() / 100.0;
    m_colorDraft.cutterHeadScale = m_spCutterHeadScale->value();

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
    const int newCutterCollisionProxyMode = m_cbCutterCollisionProxyMode
        ? m_cbCutterCollisionProxyMode->currentData().toInt()
        : m_originalCutterCollisionProxyMode;
    const QString newCutterNozzleModelPath = m_editCutterNozzleModelPath
        ? m_editCutterNozzleModelPath->text().trimmed()
        : m_originalCutterNozzleModelPath;
    const double newSimulatedConeLength = m_spSimulatedConeLength->value();
    const double newSimulatedConeTipRadius = m_spSimulatedConeTipRadius->value();
    const double newSimulatedConeBaseRadius = m_spSimulatedConeBaseRadius->value();
    const double newCutterCollisionClearance = m_spCutterCollisionClearance->value();
    const double newMaximumRapidSafetyOffset = m_spMaximumRapidSafetyOffset->value();
    const QString newMachinePreset = m_cbMachinePreset
        ? m_cbMachinePreset->currentData().toString()
        : m_originalMachinePreset;
    const QList<MachineAxisDef> newMachineAxes = collectMachineAxisDefinitions();
    HeadToolGeometry newHeadGeometry;
    newHeadGeometry.zeroBeamX = m_headTcpEditors[0]->value();
    newHeadGeometry.zeroBeamY = m_headTcpEditors[1]->value();
    newHeadGeometry.zeroBeamZ = m_headTcpEditors[2]->value();
    newHeadGeometry.focusLength = m_headTcpEditors[3]->value();
    newHeadGeometry.installationOffsetX = m_headTcpEditors[4]->value();
    newHeadGeometry.installationOffsetY = m_headTcpEditors[5]->value();
    newHeadGeometry.installationOffsetZ = m_headTcpEditors[6]->value();
    const bool headGeometryDirty =
        m_originalHeadToolGeometry.zeroBeamX != newHeadGeometry.zeroBeamX
        || m_originalHeadToolGeometry.zeroBeamY != newHeadGeometry.zeroBeamY
        || m_originalHeadToolGeometry.zeroBeamZ != newHeadGeometry.zeroBeamZ
        || m_originalHeadToolGeometry.focusLength != newHeadGeometry.focusLength
        || m_originalHeadToolGeometry.installationOffsetX != newHeadGeometry.installationOffsetX
        || m_originalHeadToolGeometry.installationOffsetY != newHeadGeometry.installationOffsetY
        || m_originalHeadToolGeometry.installationOffsetZ != newHeadGeometry.installationOffsetZ;

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
    const bool cutterCollisionDirty =
        m_originalCutterCollisionProxyMode != newCutterCollisionProxyMode
        || m_originalCutterNozzleModelPath != newCutterNozzleModelPath
        || m_originalSimulatedConeLength != newSimulatedConeLength
        || m_originalSimulatedConeTipRadius != newSimulatedConeTipRadius
        || m_originalSimulatedConeBaseRadius != newSimulatedConeBaseRadius
        || m_originalCutterCollisionClearance != newCutterCollisionClearance
        || m_originalMaximumRapidSafetyOffset != newMaximumRapidSafetyOffset;
    const bool machineDirty = m_machineConfig
        && (m_originalMachinePreset != newMachinePreset
            || !sameMachineAxisDefinitions(m_originalMachineConfigs, newMachineAxes)
            || headGeometryDirty);

    if (newSimulatedConeBaseRadius < newSimulatedConeTipRadius) {
        // 中文翻译：模拟锥头底部半径不能小于尖端半径。
        QMessageBox::warning(this, tr("Application Options"),
            tr("The simulated cone base radius cannot be smaller than its tip radius."));
        return false;
    }
    if (newCutterCollisionProxyMode == 1
        && (!QFileInfo::exists(newCutterNozzleModelPath)
            || !QFileInfo(newCutterNozzleModelPath).isFile())) {
        // 中文翻译：请选择有效的切割嘴碰撞模型文件。
        QMessageBox::warning(this, tr("Application Options"),
            tr("Select a valid cutting nozzle collision model file."));
        return false;
    }

    if (machineDirty) {
        if (auto* process = lcnc::Kernel::current().service<ProcessModule>();
            process && (process->isConnected()
                || (process->state() != ProcessModule::State::Idle
                    && process->state() != ProcessModule::State::Stopped
                    && process->state() != ProcessModule::State::Error))) {
            // 中文翻译：机床配置只能在设备断开且流程空闲时修改。
            QMessageBox::warning(this, tr("Application Options"),
                tr("Machine configuration can only be changed while devices are disconnected and the workflow is idle."));
            return false;
        }
        QString configurationError;
        if (!m_machineConfig->validateCandidateConfiguration(
                newMachinePreset, newMachineAxes, newHeadGeometry,
                &configurationError)) {
            // 中文翻译：机床配置无效：%1
            QMessageBox::warning(this, tr("Application Options"),
                tr("The machine configuration is invalid: %1").arg(configurationError));
            return false;
        }
    }

    if (!cadRuntimeDirty && !camRuntimeDirty && !cadDefaultDirty && !camDefaultDirty
        && !backgroundDirty && !modelColorDirty
        && !highlightDirty && !treeDirty && !applicationDirty
        && !machineModelPathDirty && !autoLoadMachineDirty
        && !cutterCollisionDirty && !machineDirty) {
        LCNC_DEBUG(lcnc::LogCode::Generic, "DialogOptions::applyChanges no changes");
        return true;
    }

    settings->cadViewRendering = m_renderDraft;
    settings->camViewRendering = m_renderDraft;
    if (cadDefaultDirty || camDefaultDirty) {
        settings->viewState.displayMode = m_renderDraft.defaultDisplayMode
            == StartupDisplayMode::Wireframe ? 0 : 1;
        settings->viewState.faceBoundary = m_renderDraft.defaultDisplayMode
            == StartupDisplayMode::ShadedWithEdges;
    }
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
        // 默认显示模式会在下次启动或新建 View 时生效，不刷新当前 View。
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

    // 刀头锥不在 RenderingManager 域形状内，Colors 路径触及不到，需显式让
    // CamModule 按新外观（颜色/透明度/缩放）重建刀头指示器。
    if (modelColorDirty) {
        if (auto* cam = lcnc::Kernel::current().service<CamModule>())
            cam->refreshCutterHeadAppearance();
    }

    if (treeDirty)
        applyTreeSelectionColor(m_colorDraft.treeSelectionColor);

    if (m_originalLanguage != newLanguage) {
        // 中文翻译：应用程序选项；语言修改将在重启应用程序后生效。
        QMessageBox::information(this, tr("Application Options"),
            tr("The language change will take effect after restarting the application."));
    }

    if (machineDirty) {
        m_machineConfig->setMachineAxisDefinitions(newMachinePreset, newMachineAxes);
        if (headGeometryDirty)
            m_machineConfig->setHeadToolGeometry(newHeadGeometry);
        m_originalMachinePreset = m_machineConfig->presetName();
        m_originalMachineConfigs = m_machineConfig->axisConfigurations();
        populateMachineAxisTable(m_originalMachineConfigs);
    }
    if (auto* cam = lcnc::Kernel::current().service<CamModule>()) {
        if (machineModelPathDirty)
            cam->setMachineModelPath(newMachineModelPath);
        if (autoLoadMachineDirty)
            cam->config().setAutoLoadMachineModel(newAutoLoadMachineModel);
        if (cutterCollisionDirty) {
            cam->config().setCutterCollisionProxyMode(
                newCutterCollisionProxyMode == 1
                    ? CutterCollisionProxyMode::ModelFile
                    : CutterCollisionProxyMode::SimulatedCone);
            cam->config().setCutterNozzleModelPath(newCutterNozzleModelPath);
            cam->config().setSimulatedConeLengthMm(newSimulatedConeLength);
            cam->config().setSimulatedConeTipRadiusMm(newSimulatedConeTipRadius);
            cam->config().setSimulatedConeBaseRadiusMm(newSimulatedConeBaseRadius);
            cam->config().setCutterCollisionClearanceMm(newCutterCollisionClearance);
            cam->config().setMaximumRapidSafetyOffsetMm(newMaximumRapidSafetyOffset);
            if (!cam->refreshCutterCollisionConfiguration()) {
                // Restore the last usable collision proxy configuration.  A
                // file that merely exists may still contain invalid STEP/STL
                // data; Apply must not persist a proxy that planning cannot load.
                // 中文翻译：模型文件即使存在也可能无法解析；应用失败时恢复上一份可用碰撞代理配置。
                cam->config().setCutterCollisionProxyMode(
                    m_originalCutterCollisionProxyMode == 1
                        ? CutterCollisionProxyMode::ModelFile
                        : CutterCollisionProxyMode::SimulatedCone);
                cam->config().setCutterNozzleModelPath(m_originalCutterNozzleModelPath);
                cam->config().setSimulatedConeLengthMm(m_originalSimulatedConeLength);
                cam->config().setSimulatedConeTipRadiusMm(m_originalSimulatedConeTipRadius);
                cam->config().setSimulatedConeBaseRadiusMm(m_originalSimulatedConeBaseRadius);
                cam->config().setCutterCollisionClearanceMm(m_originalCutterCollisionClearance);
                cam->config().setMaximumRapidSafetyOffsetMm(m_originalMaximumRapidSafetyOffset);
                cam->refreshCutterCollisionConfiguration();
                return false;
            }
        }
        m_originalMachineModelPath = cam->machineModelPath();
        m_originalAutoLoadMachineModel = cam->config().autoLoadMachineModel();
        m_originalCutterCollisionProxyMode =
            static_cast<int>(cam->config().cutterCollisionProxyMode());
        m_originalCutterNozzleModelPath = cam->config().cutterNozzleModelPath();
        m_originalSimulatedConeLength = cam->config().simulatedConeLengthMm();
        m_originalSimulatedConeTipRadius = cam->config().simulatedConeTipRadiusMm();
        m_originalSimulatedConeBaseRadius = cam->config().simulatedConeBaseRadiusMm();
        m_originalCutterCollisionClearance = cam->config().cutterCollisionClearanceMm();
        m_originalMaximumRapidSafetyOffset = cam->config().maximumRapidSafetyOffsetMm();
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
