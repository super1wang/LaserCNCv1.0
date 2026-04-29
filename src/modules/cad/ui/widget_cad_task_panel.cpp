#include "modules/cad/ui/widget_cad_task_panel.h"

#include "modules/cad/selection/cad_selection.h"
#include "modules/cad/task/cad_tool_filter.h"
#include "modules/cad/task/cad_tool_registry.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QSet>
#include <QSpinBox>
#include <QStackedWidget>
#include <QVBoxLayout>

namespace lcnc::cad::ui {

namespace {
constexpr const char* TOOL_SKETCH_BEGIN = "cad.sketch.begin";
constexpr const char* TOOL_FEATURE_EXTRUDE = "cad.feature.extrude";
constexpr const char* TOOL_FEATURE_REVOLVE = "cad.feature.revolve";

void setParameterVisible(QLabel* label, QWidget* editor, bool visible)
{
    if (label)
        label->setVisible(visible);
    if (editor)
        editor->setVisible(visible);
}
} // namespace

WidgetCadTaskPanel::WidgetCadTaskPanel(QWidget* parent)
    : QWidget(parent)
{
    buildUi();
}

double WidgetCadTaskPanel::featureLength() const
{
    return m_spinFeatureLength ? m_spinFeatureLength->value() : 10.0;
}

double WidgetCadTaskPanel::featureAngle() const
{
    return m_spinFeatureAngle ? m_spinFeatureAngle->value() : 360.0;
}

int WidgetCadTaskPanel::featureIndex() const
{
    return m_comboFeature ? m_comboFeature->currentData().toInt() : 0;
}

int WidgetCadTaskPanel::primitiveIndex() const
{
    return m_comboPrimitive ? m_comboPrimitive->currentData().toInt() : 0;
}

double WidgetCadTaskPanel::primitiveSizeX() const
{
    return m_spinPrimitiveSizeX ? m_spinPrimitiveSizeX->value() : 100.0;
}

double WidgetCadTaskPanel::primitiveSizeY() const
{
    return m_spinPrimitiveSizeY ? m_spinPrimitiveSizeY->value() : 100.0;
}

double WidgetCadTaskPanel::primitiveSizeZ() const
{
    return m_spinPrimitiveSizeZ ? m_spinPrimitiveSizeZ->value() : 50.0;
}

double WidgetCadTaskPanel::primitiveRadius1() const
{
    return m_spinPrimitiveRadius1 ? m_spinPrimitiveRadius1->value() : 50.0;
}

double WidgetCadTaskPanel::primitiveRadius2() const
{
    return m_spinPrimitiveRadius2 ? m_spinPrimitiveRadius2->value() : 15.0;
}

bool WidgetCadTaskPanel::isPreviewEnabled() const
{
    if (isPrimitivePageActive())
        return m_checkPrimitivePreview && m_checkPrimitivePreview->isChecked();
    if (isTransformPageActive())
        return m_checkTransformPreview && m_checkTransformPreview->isChecked();
    return m_checkPreview && m_checkPreview->isChecked();
}

bool WidgetCadTaskPanel::isFeaturePageActive() const
{
    return m_stack && m_stack->currentWidget() == m_pageFeature;
}

bool WidgetCadTaskPanel::isPrimitivePageActive() const
{
    return m_stack && m_stack->currentWidget() == m_pagePrimitive;
}

bool WidgetCadTaskPanel::isTransformPageActive() const
{
    return m_stack && m_stack->currentWidget() == m_pageTransform;
}

int WidgetCadTaskPanel::transformReferenceMode() const
{
    return m_comboTransformReference ? m_comboTransformReference->currentData().toInt() : 0;
}

double WidgetCadTaskPanel::transformTranslateX() const
{
    return m_spinTransformTranslateX ? m_spinTransformTranslateX->value() : 0.0;
}

double WidgetCadTaskPanel::transformTranslateY() const
{
    return m_spinTransformTranslateY ? m_spinTransformTranslateY->value() : 0.0;
}

double WidgetCadTaskPanel::transformTranslateZ() const
{
    return m_spinTransformTranslateZ ? m_spinTransformTranslateZ->value() : 0.0;
}

double WidgetCadTaskPanel::transformRotateX() const
{
    return m_spinTransformRotateX ? m_spinTransformRotateX->value() : 0.0;
}

double WidgetCadTaskPanel::transformRotateY() const
{
    return m_spinTransformRotateY ? m_spinTransformRotateY->value() : 0.0;
}

double WidgetCadTaskPanel::transformRotateZ() const
{
    return m_spinTransformRotateZ ? m_spinTransformRotateZ->value() : 0.0;
}

void WidgetCadTaskPanel::setContextState(bool hasDocument,
                                         int selectedCount,
                                         bool sketchEditing,
                                         bool hasSelectedSketch)
{
    lcnc::cad::selection::CadSelectionContext context;
    context.hasDocument = hasDocument;
    context.selectedShapeCount = selectedCount;
    context.sketchEditing = sketchEditing;
    context.hasSelectedSketch = hasSelectedSketch;
    setSelectionContext(context);
}

void WidgetCadTaskPanel::setSelectionContext(
    const lcnc::cad::selection::CadSelectionContext& context)
{
    m_selectionContext = context;
    m_hasDocument = context.hasDocument;
    m_selectedCount = context.selectedShapeCount;
    m_sketchEditing = context.sketchEditing;
    m_hasSelectedSketch = context.hasSelectedSketch;
    m_selectedSketchId = context.selectedSketchId;
    if (m_groupSketchTool)
        m_groupSketchTool->setEnabled(m_sketchEditing);
    if (m_btnExitSketch)
        m_btnExitSketch->setEnabled(m_sketchEditing);
    if (m_btnCancelSketch)
        m_btnCancelSketch->setEnabled(m_sketchEditing);
    if (m_btnStartSketch)
        m_btnStartSketch->setEnabled(!m_sketchEditing && m_hasDocument);
    updateHomeAvailability();
}

void WidgetCadTaskPanel::setActiveSketchTool(int toolKind)
{
    if (!m_comboSketchTool)
        return;
    const int idx = m_comboSketchTool->findData(toolKind);
    if (idx < 0)
        return;
    if (m_comboSketchTool->currentIndex() == idx) {
        rebuildSketchToolForm(toolKind);
        return;
    }
    QSignalBlocker blocker(m_comboSketchTool);
    m_comboSketchTool->setCurrentIndex(idx);
    rebuildSketchToolForm(toolKind);
}

void WidgetCadTaskPanel::setSketchElements(const QVector<SketchElementEntry>& elements)
{
    if (!m_listSketchElements)
        return;
    m_listSketchElements->clear();
    for (const auto& entry : elements) {
        auto* item = new QListWidgetItem(entry.label, m_listSketchElements);
        item->setData(Qt::UserRole, entry.id);
    }
}

void WidgetCadTaskPanel::setFinishedSketches(const QVector<FinishedSketchEntry>& entries,
                                             int selectedSketchId)
{
    if (!m_listFinishedSketches)
        return;
    QSignalBlocker blocker(m_listFinishedSketches);
    m_listFinishedSketches->clear();
    QListWidgetItem* itemToSelect = nullptr;
    for (const auto& entry : entries) {
        QString label = entry.label;
        if (entry.usedByFeature)
            label += tr("（已用）");
        auto* item = new QListWidgetItem(label, m_listFinishedSketches);
        item->setData(Qt::UserRole, entry.sketchId);
        item->setData(Qt::UserRole + 1, entry.visible);
        if (entry.usedByFeature)
            item->setForeground(Qt::gray);
        if (entry.sketchId == selectedSketchId)
            itemToSelect = item;
    }
    m_selectedSketchId = selectedSketchId;
    if (itemToSelect)
        m_listFinishedSketches->setCurrentItem(itemToSelect);
}

void WidgetCadTaskPanel::showHomePage()
{
    const bool leavingTransformPage = isTransformPageActive();
    if (m_stack && m_pageHome)
        m_stack->setCurrentWidget(m_pageHome);
    updateHomeAvailability();
    if (leavingTransformPage)
        emit transformCanceled();
}

void WidgetCadTaskPanel::showSketchPage()
{
    if (m_stack && m_pageSketch)
        m_stack->setCurrentWidget(m_pageSketch);
}

void WidgetCadTaskPanel::showPrimitivePage(int primitiveIndex)
{
    if (m_comboPrimitive) {
        const int comboIndex = m_comboPrimitive->findData(primitiveIndex);
        if (comboIndex >= 0)
            m_comboPrimitive->setCurrentIndex(comboIndex);
    }
    if (m_spinPrimitiveSizeX)
        m_spinPrimitiveSizeX->setValue(100.0);
    if (m_spinPrimitiveSizeY)
        m_spinPrimitiveSizeY->setValue(100.0);
    if (m_spinPrimitiveSizeZ)
        m_spinPrimitiveSizeZ->setValue((primitiveIndex == 0) ? 50.0 : 100.0);
    if (m_spinPrimitiveRadius1)
        m_spinPrimitiveRadius1->setValue(primitiveIndex == 4 ? 60.0 : 50.0);
    if (m_spinPrimitiveRadius2)
        m_spinPrimitiveRadius2->setValue(primitiveIndex == 3 ? 0.0 : 15.0);
    if (m_stack && m_pagePrimitive)
        m_stack->setCurrentWidget(m_pagePrimitive);
    updatePrimitiveParameterVisibility();
    emitPrimitiveParametersChanged();
}

void WidgetCadTaskPanel::showFeaturePage(int featureIndex)
{
    if (m_comboFeature) {
        const int comboIndex = m_comboFeature->findData(featureIndex);
        if (comboIndex >= 0)
            m_comboFeature->setCurrentIndex(comboIndex);
    }
    if (m_stack && m_pageFeature)
        m_stack->setCurrentWidget(m_pageFeature);
    emitFeatureParametersChanged();
}

void WidgetCadTaskPanel::showTransformPage()
{
    if (m_spinTransformTranslateX)
        m_spinTransformTranslateX->setValue(0.0);
    if (m_spinTransformTranslateY)
        m_spinTransformTranslateY->setValue(0.0);
    if (m_spinTransformTranslateZ)
        m_spinTransformTranslateZ->setValue(0.0);
    if (m_spinTransformRotateX)
        m_spinTransformRotateX->setValue(0.0);
    if (m_spinTransformRotateY)
        m_spinTransformRotateY->setValue(0.0);
    if (m_spinTransformRotateZ)
        m_spinTransformRotateZ->setValue(0.0);
    if (m_stack && m_pageTransform)
        m_stack->setCurrentWidget(m_pageTransform);
    emitTransformParametersChanged();
}

void WidgetCadTaskPanel::addTransformDragDelta(int operation, int axis, double delta)
{
    if (!isTransformPageActive())
        return;

    QDoubleSpinBox* target = nullptr;
    if (operation == 0) {
        if (axis == 0)
            target = m_spinTransformTranslateX;
        else if (axis == 1)
            target = m_spinTransformTranslateY;
        else if (axis == 2)
            target = m_spinTransformTranslateZ;
    } else {
        if (axis == 0)
            target = m_spinTransformRotateX;
        else if (axis == 1)
            target = m_spinTransformRotateY;
        else if (axis == 2)
            target = m_spinTransformRotateZ;
    }

    if (target) {
        const double clampedDelta = (operation == 1) ? qBound(-90.0, delta, 90.0) : delta;
        target->setValue(target->value() + clampedDelta);
    }
}

void WidgetCadTaskPanel::buildUi()
{
    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(6, 6, 6, 6);
    mainLayout->setSpacing(8);

    auto* title = new QLabel(tr("<b>CAD 建模</b>"), this);
    mainLayout->addWidget(title);

    m_stack = new QStackedWidget(this);
    mainLayout->addWidget(m_stack, 1);

    buildHomePage();
    buildSketchPage();
    buildPrimitivePage();
    buildFeaturePage();
    buildTransformPage();
    showHomePage();
}

QPushButton* WidgetCadTaskPanel::makeToolButton(
    const lcnc::cad::task::CadToolDescriptor& tool,
    QWidget* parent)
{
    auto* button = new QPushButton(tool.title, parent);
    button->setMinimumHeight(28);
    m_toolButtons.insert(tool.toolId, button);
    connect(button, &QPushButton::clicked, this, [this, tool]() {
        using lcnc::cad::task::CadToolActivation;
        switch (tool.activation) {
        case CadToolActivation::SketchPage:
            showSketchPage();
            break;
        case CadToolActivation::PrimitivePage:
            showPrimitivePage(tool.targetIndex);
            break;
        case CadToolActivation::FeaturePage:
            showFeaturePage(tool.targetIndex);
            break;
        case CadToolActivation::TransformPage:
            showTransformPage();
            break;
        case CadToolActivation::Command:
        default:
            emit commandRequested(tool.commandId);
            break;
        }
    });
    return button;
}

void WidgetCadTaskPanel::addToolButtons(
    QGridLayout* layout,
    const QVector<lcnc::cad::task::CadToolDescriptor>& tools,
    QWidget* parent,
    int columns)
{
    if (!layout || columns <= 0)
        return;

    int index = 0;
    for (const auto& tool : tools) {
        QPushButton* button = makeToolButton(tool, parent);
        if (tool.toolId == QString::fromLatin1(TOOL_SKETCH_BEGIN))
            m_btnOpenSketchPage = button;
        else if (tool.toolId == QString::fromLatin1(TOOL_FEATURE_EXTRUDE))
            m_btnExtrudeFeature = button;
        else if (tool.toolId == QString::fromLatin1(TOOL_FEATURE_REVOLVE))
            m_btnRevolveFeature = button;

        const int row = index / columns;
        const int column = index % columns;
        const int columnSpan = (columns == 2 && index == tools.size() - 1 && (tools.size() % 2) == 1) ? 2 : 1;
        layout->addWidget(button, row, column, 1, columnSpan);
        ++index;
    }
}

void WidgetCadTaskPanel::buildHomePage()
{
    const auto registry = lcnc::cad::task::CadToolRegistry::createDefault();

    m_pageHome = new QWidget(m_stack);
    auto* layout = new QVBoxLayout(m_pageHome);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(8);

    m_groupDocument = new QGroupBox(tr("文档"), m_pageHome);
    auto* docLayout = new QGridLayout(m_groupDocument);
    addToolButtons(docLayout,
                   registry.toolsByCategory(lcnc::cad::task::CadToolCategory::Document),
                   m_groupDocument);
    layout->addWidget(m_groupDocument);

    m_groupBase = new QGroupBox(tr("基础建模"), m_pageHome);
    auto* baseLayout = new QGridLayout(m_groupBase);
    addToolButtons(baseLayout,
                   registry.toolsByCategory(lcnc::cad::task::CadToolCategory::BaseModeling),
                   m_groupBase);
    layout->addWidget(m_groupBase);

    m_groupProfile = new QGroupBox(tr("草图特征"), m_pageHome);
    auto* profileLayout = new QGridLayout(m_groupProfile);
    addToolButtons(profileLayout,
                   registry.toolsByCategory(lcnc::cad::task::CadToolCategory::SketchFeature),
                   m_groupProfile);
    layout->addWidget(m_groupProfile);

    // 草图列表：显示当前文档已完成草图，选中后可进入拉伸/旋转。
    m_groupSketches = new QGroupBox(tr("草图列表"), m_pageHome);
    auto* sketchListLayout = new QVBoxLayout(m_groupSketches);
    m_listFinishedSketches = new QListWidget(m_groupSketches);
    m_listFinishedSketches->setMinimumHeight(80);
    sketchListLayout->addWidget(m_listFinishedSketches);
    auto* sketchBtnRow = new QHBoxLayout();
    m_btnToggleSketchVisible = new QPushButton(tr("显示/隐藏"), m_groupSketches);
    m_btnDeleteFinishedSketch = new QPushButton(tr("删除"), m_groupSketches);
    sketchBtnRow->addWidget(m_btnToggleSketchVisible);
    sketchBtnRow->addWidget(m_btnDeleteFinishedSketch);
    sketchListLayout->addLayout(sketchBtnRow);
    layout->addWidget(m_groupSketches);
    connect(m_listFinishedSketches, &QListWidget::currentItemChanged, this,
            [this](QListWidgetItem* current, QListWidgetItem*) {
        const int id = current ? current->data(Qt::UserRole).toInt() : 0;
        m_selectedSketchId = id;
        emit sketchSelectionChanged(id);
    });
    connect(m_btnDeleteFinishedSketch, &QPushButton::clicked, this, [this]() {
        if (m_selectedSketchId > 0)
            emit sketchDeleteRequested(m_selectedSketchId);
    });
    connect(m_btnToggleSketchVisible, &QPushButton::clicked, this, [this]() {
        QListWidgetItem* item = m_listFinishedSketches->currentItem();
        if (!item)
            return;
        const int id = item->data(Qt::UserRole).toInt();
        const bool currentlyVisible = item->data(Qt::UserRole + 1).toBool();
        emit sketchVisibilityToggled(id, !currentlyVisible);
    });

    m_groupSelection = new QGroupBox(tr("已选对象"), m_pageHome);
    auto* selectionLayout = new QGridLayout(m_groupSelection);
    addToolButtons(selectionLayout,
                   registry.toolsByCategory(lcnc::cad::task::CadToolCategory::Selection),
                   m_groupSelection);
    layout->addWidget(m_groupSelection);

    m_groupBoolean = new QGroupBox(tr("布尔运算"), m_pageHome);
    auto* booleanLayout = new QGridLayout(m_groupBoolean);
    addToolButtons(booleanLayout,
                   registry.toolsByCategory(lcnc::cad::task::CadToolCategory::Boolean),
                   m_groupBoolean);
    layout->addWidget(m_groupBoolean);

    m_groupMeasure = new QGroupBox(tr("测量"), m_pageHome);
    auto* measureLayout = new QGridLayout(m_groupMeasure);
    addToolButtons(measureLayout,
                   registry.toolsByCategory(lcnc::cad::task::CadToolCategory::Measure),
                   m_groupMeasure);
    layout->addWidget(m_groupMeasure);

    m_groupDelete = new QGroupBox(tr("编辑"), m_pageHome);
    auto* deleteLayout = new QGridLayout(m_groupDelete);
    addToolButtons(deleteLayout,
                   registry.toolsByCategory(lcnc::cad::task::CadToolCategory::Delete),
                   m_groupDelete);
    layout->addWidget(m_groupDelete);

    m_groupFaceTools = new QGroupBox(tr("面工具"), m_pageHome);
    auto* faceLayout = new QGridLayout(m_groupFaceTools);
    addToolButtons(faceLayout,
                   registry.toolsByCategory(lcnc::cad::task::CadToolCategory::FaceTool),
                   m_groupFaceTools);
    layout->addWidget(m_groupFaceTools);

    m_groupEdgeTools = new QGroupBox(tr("边工具"), m_pageHome);
    auto* edgeLayout = new QGridLayout(m_groupEdgeTools);
    addToolButtons(edgeLayout,
                   registry.toolsByCategory(lcnc::cad::task::CadToolCategory::EdgeTool),
                   m_groupEdgeTools);
    layout->addWidget(m_groupEdgeTools);

    layout->addStretch(1);
    m_stack->addWidget(m_pageHome);
}

void WidgetCadTaskPanel::buildSketchPage()
{
    m_pageSketch = new QWidget(m_stack);
    auto* layout = new QVBoxLayout(m_pageSketch);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(8);

    auto* group = new QGroupBox(tr("新建草图"), m_pageSketch);
    auto* form = new QFormLayout(group);

    m_comboPlane = new QComboBox(group);
    m_comboPlane->addItem(tr("XY 平面"), 0);
    m_comboPlane->addItem(tr("YZ 平面"), 1);
    m_comboPlane->addItem(tr("ZX 平面"), 2);
    form->addRow(tr("平面:"), m_comboPlane);

    layout->addWidget(group);

    // 草图工具调色板（开始草图后启用）。
    m_groupSketchTool = new QGroupBox(tr("草图工具"), m_pageSketch);
    auto* toolLayout = new QVBoxLayout(m_groupSketchTool);
    toolLayout->setContentsMargins(8, 8, 8, 8);
    toolLayout->setSpacing(6);

    m_comboSketchTool = new QComboBox(m_groupSketchTool);
    m_comboSketchTool->addItem(tr("未选择"), 0);
    m_comboSketchTool->addItem(tr("点"), 1);
    m_comboSketchTool->addItem(tr("直线"), 2);
    m_comboSketchTool->addItem(tr("圆弧"), 3);
    m_comboSketchTool->addItem(tr("圆"), 4);
    m_comboSketchTool->addItem(tr("矩形"), 5);
    m_comboSketchTool->addItem(tr("多边形"), 6);
    toolLayout->addWidget(m_comboSketchTool);

    m_sketchToolForm = new QWidget(m_groupSketchTool);
    auto* paramLayout = new QFormLayout(m_sketchToolForm);
    paramLayout->setContentsMargins(0, 0, 0, 0);
    toolLayout->addWidget(m_sketchToolForm);

    auto* toolBtnRow = new QHBoxLayout();
    m_btnAddSketchElement = new QPushButton(tr("添加"), m_groupSketchTool);
    m_btnRemoveSketchElement = new QPushButton(tr("移除选中"), m_groupSketchTool);
    toolBtnRow->addWidget(m_btnAddSketchElement);
    toolBtnRow->addWidget(m_btnRemoveSketchElement);
    toolLayout->addLayout(toolBtnRow);

    m_listSketchElements = new QListWidget(m_groupSketchTool);
    m_listSketchElements->setMinimumHeight(120);
    toolLayout->addWidget(m_listSketchElements);

    layout->addWidget(m_groupSketchTool);
    rebuildSketchToolForm(0);

    m_btnStartSketch = new QPushButton(tr("开始草图"), m_pageSketch);
    m_btnExitSketch = new QPushButton(tr("完成草图"), m_pageSketch);
    m_btnCancelSketch = new QPushButton(tr("取消"), m_pageSketch);
    layout->addWidget(m_btnStartSketch);
    layout->addWidget(m_btnExitSketch);
    layout->addWidget(m_btnCancelSketch);
    layout->addStretch(1);

    connect(m_btnStartSketch, &QPushButton::clicked, this, [this]() {
        emit sketchCreateRequested(m_comboPlane->currentData().toInt());
    });
    connect(m_btnExitSketch, &QPushButton::clicked, this, [this]() {
        emit sketchExitRequested();
    });
    connect(m_btnCancelSketch, &QPushButton::clicked, this, [this]() {
        emit sketchCanceled();
        showHomePage();
    });

    connect(m_comboSketchTool, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int) {
        const int kind = m_comboSketchTool->currentData().toInt();
        rebuildSketchToolForm(kind);
        emit sketchToolChanged(kind);
    });
    connect(m_btnAddSketchElement, &QPushButton::clicked, this, [this]() {
        const int kind = m_comboSketchTool->currentData().toInt();
        if (kind <= 0)
            return;
        emit sketchElementAddRequested(kind, currentSketchToolParams());
    });
    connect(m_btnRemoveSketchElement, &QPushButton::clicked, this, [this]() {
        QListWidgetItem* item = m_listSketchElements->currentItem();
        if (!item)
            return;
        const int id = item->data(Qt::UserRole).toInt();
        if (id > 0)
            emit sketchElementRemoveRequested(id);
    });

    m_stack->addWidget(m_pageSketch);
}

    void WidgetCadTaskPanel::buildPrimitivePage()
    {
        m_pagePrimitive = new QWidget(m_stack);
        auto* layout = new QVBoxLayout(m_pagePrimitive);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->setSpacing(8);

        auto* group = new QGroupBox(tr("基础体参数"), m_pagePrimitive);
        auto* form = new QFormLayout(group);

        m_comboPrimitive = new QComboBox(group);
        m_comboPrimitive->addItem(tr("长方体"), 0);
        m_comboPrimitive->addItem(tr("圆柱体"), 1);
        m_comboPrimitive->addItem(tr("球体"), 2);
        m_comboPrimitive->addItem(tr("圆锥体"), 3);
        m_comboPrimitive->addItem(tr("圆环体"), 4);
        form->addRow(tr("类型:"), m_comboPrimitive);

        m_spinPrimitiveSizeX = new QDoubleSpinBox(group);
        m_spinPrimitiveSizeX->setRange(0.001, 100000.0);
        m_spinPrimitiveSizeX->setDecimals(3);
        m_spinPrimitiveSizeX->setSingleStep(1.0);
        m_spinPrimitiveSizeX->setValue(100.0);
        m_spinPrimitiveSizeX->setSuffix(tr(" mm"));
        m_labelPrimitiveSizeX = new QLabel(tr("长度 X:"), group);
        form->addRow(m_labelPrimitiveSizeX, m_spinPrimitiveSizeX);

        m_spinPrimitiveSizeY = new QDoubleSpinBox(group);
        m_spinPrimitiveSizeY->setRange(0.001, 100000.0);
        m_spinPrimitiveSizeY->setDecimals(3);
        m_spinPrimitiveSizeY->setSingleStep(1.0);
        m_spinPrimitiveSizeY->setValue(100.0);
        m_spinPrimitiveSizeY->setSuffix(tr(" mm"));
        m_labelPrimitiveSizeY = new QLabel(tr("宽度 Y:"), group);
        form->addRow(m_labelPrimitiveSizeY, m_spinPrimitiveSizeY);

        m_spinPrimitiveSizeZ = new QDoubleSpinBox(group);
        m_spinPrimitiveSizeZ->setRange(0.001, 100000.0);
        m_spinPrimitiveSizeZ->setDecimals(3);
        m_spinPrimitiveSizeZ->setSingleStep(1.0);
        m_spinPrimitiveSizeZ->setValue(50.0);
        m_spinPrimitiveSizeZ->setSuffix(tr(" mm"));
        m_labelPrimitiveSizeZ = new QLabel(tr("高度 Z:"), group);
        form->addRow(m_labelPrimitiveSizeZ, m_spinPrimitiveSizeZ);

        m_spinPrimitiveRadius1 = new QDoubleSpinBox(group);
        m_spinPrimitiveRadius1->setRange(0.0, 100000.0);
        m_spinPrimitiveRadius1->setDecimals(3);
        m_spinPrimitiveRadius1->setSingleStep(1.0);
        m_spinPrimitiveRadius1->setValue(50.0);
        m_spinPrimitiveRadius1->setSuffix(tr(" mm"));
        m_labelPrimitiveRadius1 = new QLabel(tr("半径:"), group);
        form->addRow(m_labelPrimitiveRadius1, m_spinPrimitiveRadius1);

        m_spinPrimitiveRadius2 = new QDoubleSpinBox(group);
        m_spinPrimitiveRadius2->setRange(0.0, 100000.0);
        m_spinPrimitiveRadius2->setDecimals(3);
        m_spinPrimitiveRadius2->setSingleStep(1.0);
        m_spinPrimitiveRadius2->setValue(15.0);
        m_spinPrimitiveRadius2->setSuffix(tr(" mm"));
        m_labelPrimitiveRadius2 = new QLabel(tr("管半径:"), group);
        form->addRow(m_labelPrimitiveRadius2, m_spinPrimitiveRadius2);

        m_checkPrimitivePreview = new QCheckBox(tr("预览"), group);
        m_checkPrimitivePreview->setChecked(true);
        form->addRow(m_checkPrimitivePreview);
        layout->addWidget(group);

        m_btnApplyPrimitive = new QPushButton(tr("应用"), m_pagePrimitive);
        m_btnCancelPrimitive = new QPushButton(tr("取消"), m_pagePrimitive);
        layout->addWidget(m_btnApplyPrimitive);
        layout->addWidget(m_btnCancelPrimitive);
        layout->addStretch(1);

        connect(m_btnApplyPrimitive, &QPushButton::clicked, this, [this]() {
        emit primitiveApplyRequested(primitiveIndex(),
                         primitiveSizeX(),
                         primitiveSizeY(),
                         primitiveSizeZ(),
                         primitiveRadius1(),
                         primitiveRadius2());
        });
        connect(m_btnCancelPrimitive, &QPushButton::clicked, this, [this]() {
        emit primitiveCanceled();
        showHomePage();
        });
        connect(m_checkPrimitivePreview, &QCheckBox::toggled,
            this, &WidgetCadTaskPanel::previewToggled);
        connect(m_comboPrimitive, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int) {
        updatePrimitiveParameterVisibility();
        emitPrimitiveParametersChanged();
        });
        connect(m_spinPrimitiveSizeX, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, [this](double) { emitPrimitiveParametersChanged(); });
        connect(m_spinPrimitiveSizeY, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, [this](double) { emitPrimitiveParametersChanged(); });
        connect(m_spinPrimitiveSizeZ, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, [this](double) { emitPrimitiveParametersChanged(); });
        connect(m_spinPrimitiveRadius1, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, [this](double) { emitPrimitiveParametersChanged(); });
        connect(m_spinPrimitiveRadius2, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, [this](double) { emitPrimitiveParametersChanged(); });

        updatePrimitiveParameterVisibility();
        m_stack->addWidget(m_pagePrimitive);
    }

void WidgetCadTaskPanel::buildFeaturePage()
{
    m_pageFeature = new QWidget(m_stack);
    auto* layout = new QVBoxLayout(m_pageFeature);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(8);

    auto* group = new QGroupBox(tr("特征参数"), m_pageFeature);
    auto* form = new QFormLayout(group);

    m_comboFeature = new QComboBox(group);
    m_comboFeature->addItem(tr("拉伸凸台"), 0);
    m_comboFeature->addItem(tr("旋转凸台"), 1);
    form->addRow(tr("方式:"), m_comboFeature);

    m_spinFeatureLength = new QDoubleSpinBox(group);
    m_spinFeatureLength->setRange(-100000.0, 100000.0);
    m_spinFeatureLength->setDecimals(3);
    m_spinFeatureLength->setSingleStep(1.0);
    m_spinFeatureLength->setValue(10.0);
    m_spinFeatureLength->setSuffix(tr(" mm"));
    form->addRow(tr("长度:"), m_spinFeatureLength);

    m_spinFeatureAngle = new QDoubleSpinBox(group);
    m_spinFeatureAngle->setRange(-360.0, 360.0);
    m_spinFeatureAngle->setDecimals(3);
    m_spinFeatureAngle->setSingleStep(5.0);
    m_spinFeatureAngle->setValue(360.0);
    m_spinFeatureAngle->setSuffix(tr(" °"));
    form->addRow(tr("角度:"), m_spinFeatureAngle);

    m_checkPreview = new QCheckBox(tr("预览"), group);
    m_checkPreview->setChecked(true);
    form->addRow(m_checkPreview);
    layout->addWidget(group);

    m_btnApplyFeature = new QPushButton(tr("应用"), m_pageFeature);
    m_btnCancelFeature = new QPushButton(tr("取消"), m_pageFeature);
    layout->addWidget(m_btnApplyFeature);
    layout->addWidget(m_btnCancelFeature);
    layout->addStretch(1);

    connect(m_btnApplyFeature, &QPushButton::clicked, this, [this]() {
        emit featureApplyRequested(featureIndex(), featureLength(), featureAngle());
    });
    connect(m_btnCancelFeature, &QPushButton::clicked, this, [this]() {
        emit featureCanceled();
        showHomePage();
    });
    connect(m_checkPreview, &QCheckBox::toggled,
            this, &WidgetCadTaskPanel::previewToggled);
    connect(m_comboFeature, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int) { emitFeatureParametersChanged(); });
    connect(m_spinFeatureLength, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, [this](double) { emitFeatureParametersChanged(); });
    connect(m_spinFeatureAngle, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, [this](double) { emitFeatureParametersChanged(); });

    m_stack->addWidget(m_pageFeature);
}

void WidgetCadTaskPanel::buildTransformPage()
{
    m_pageTransform = new QWidget(m_stack);
    auto* layout = new QVBoxLayout(m_pageTransform);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(8);

    auto* group = new QGroupBox(tr("变换"), m_pageTransform);
    auto* form = new QFormLayout(group);

    m_comboTransformReference = new QComboBox(group);
    m_comboTransformReference->addItem(tr("模型中心"), 0);
    m_comboTransformReference->addItem(tr("世界原点"), 1);
    form->addRow(tr("参考:"), m_comboTransformReference);

    auto makeDistanceSpin = [group, this]() {
        auto* spin = new QDoubleSpinBox(group);
        spin->setRange(-1000000.0, 1000000.0);
        spin->setDecimals(3);
        spin->setSingleStep(1.0);
        spin->setSuffix(tr(" mm"));
        return spin;
    };
    auto makeAngleSpin = [group, this]() {
        auto* spin = new QDoubleSpinBox(group);
        spin->setRange(-36000.0, 36000.0);
        spin->setDecimals(3);
        spin->setSingleStep(1.0);
        spin->setSuffix(tr(" °"));
        return spin;
    };

    m_spinTransformTranslateX = makeDistanceSpin();
    m_spinTransformTranslateY = makeDistanceSpin();
    m_spinTransformTranslateZ = makeDistanceSpin();
    form->addRow(tr("平移 X:"), m_spinTransformTranslateX);
    form->addRow(tr("平移 Y:"), m_spinTransformTranslateY);
    form->addRow(tr("平移 Z:"), m_spinTransformTranslateZ);

    m_spinTransformRotateX = makeAngleSpin();
    m_spinTransformRotateY = makeAngleSpin();
    m_spinTransformRotateZ = makeAngleSpin();
    form->addRow(tr("旋转 X:"), m_spinTransformRotateX);
    form->addRow(tr("旋转 Y:"), m_spinTransformRotateY);
    form->addRow(tr("旋转 Z:"), m_spinTransformRotateZ);

    m_checkTransformPreview = new QCheckBox(tr("预览"), group);
    m_checkTransformPreview->setChecked(true);
    form->addRow(m_checkTransformPreview);
    layout->addWidget(group);

    m_btnApplyTransform = new QPushButton(tr("应用"), m_pageTransform);
    m_btnCancelTransform = new QPushButton(tr("取消"), m_pageTransform);
    layout->addWidget(m_btnApplyTransform);
    layout->addWidget(m_btnCancelTransform);
    layout->addStretch(1);

    auto connectTransformSpin = [this](QDoubleSpinBox* spin) {
        connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, [this](double) { emitTransformParametersChanged(); });
    };
    connectTransformSpin(m_spinTransformTranslateX);
    connectTransformSpin(m_spinTransformTranslateY);
    connectTransformSpin(m_spinTransformTranslateZ);
    connectTransformSpin(m_spinTransformRotateX);
    connectTransformSpin(m_spinTransformRotateY);
    connectTransformSpin(m_spinTransformRotateZ);
    connect(m_comboTransformReference, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int) { emitTransformParametersChanged(); });
    connect(m_checkTransformPreview, &QCheckBox::toggled,
            this, &WidgetCadTaskPanel::previewToggled);
    connect(m_btnApplyTransform, &QPushButton::clicked, this, [this]() {
        emit transformApplyRequested(transformTranslateX(),
                                     transformTranslateY(),
                                     transformTranslateZ(),
                                     transformRotateX(),
                                     transformRotateY(),
                                     transformRotateZ(),
                                     transformReferenceMode());
    });
    connect(m_btnCancelTransform, &QPushButton::clicked, this, [this]() {
        showHomePage();
    });

    m_stack->addWidget(m_pageTransform);
}

void WidgetCadTaskPanel::updateHomeAvailability()
{
    lcnc::cad::selection::CadSelectionContext context = m_selectionContext;
    context.hasDocument = m_hasDocument;
    context.sketchEditing = m_sketchEditing;
    context.hasSelectedSketch = m_hasSelectedSketch;
    context.selectedShapeCount = m_selectedCount;
    context.selectedSketchId = m_selectedSketchId;

    const auto registry = lcnc::cad::task::CadToolRegistry::createDefault();
    const auto availableTools = lcnc::cad::task::CadToolFilter::availableTools(registry, context);
    QSet<QString> visibleToolIds;
    for (const auto& tool : availableTools)
        visibleToolIds.insert(tool.toolId);
    auto hasVisibleCategory = [&](lcnc::cad::task::CadToolCategory category) {
        for (const auto& tool : registry.toolsByCategory(category)) {
            if (visibleToolIds.contains(tool.toolId))
                return true;
        }
        return false;
    };
    for (auto it = m_toolButtons.begin(); it != m_toolButtons.end(); ++it) {
        if (it.value())
            it.value()->setVisible(visibleToolIds.contains(it.key()));
    }

    if (m_groupDocument)
        m_groupDocument->setVisible(visibleToolIds.contains(QStringLiteral("cad.document.new"))
                                    || visibleToolIds.contains(QStringLiteral("cad.document.open")));
    if (m_groupBase)
        m_groupBase->setVisible(hasVisibleCategory(lcnc::cad::task::CadToolCategory::BaseModeling));
    if (m_groupProfile)
        m_groupProfile->setVisible(hasVisibleCategory(lcnc::cad::task::CadToolCategory::SketchFeature));
    if (m_groupSketches)
        m_groupSketches->setVisible(m_hasDocument && !m_sketchEditing);
    if (m_groupSelection)
        m_groupSelection->setVisible(hasVisibleCategory(lcnc::cad::task::CadToolCategory::Selection));
    if (m_groupBoolean)
        m_groupBoolean->setVisible(hasVisibleCategory(lcnc::cad::task::CadToolCategory::Boolean));
    if (m_groupMeasure)
        m_groupMeasure->setVisible(hasVisibleCategory(lcnc::cad::task::CadToolCategory::Measure));
    if (m_groupDelete)
        m_groupDelete->setVisible(hasVisibleCategory(lcnc::cad::task::CadToolCategory::Delete));
    if (m_groupFaceTools)
        m_groupFaceTools->setVisible(hasVisibleCategory(lcnc::cad::task::CadToolCategory::FaceTool));
    if (m_groupEdgeTools)
        m_groupEdgeTools->setVisible(hasVisibleCategory(lcnc::cad::task::CadToolCategory::EdgeTool));

    if (m_btnOpenSketchPage)
        m_btnOpenSketchPage->setText(m_sketchEditing ? tr("继续草图") : tr("新建草图"));
    if (m_btnStartSketch)
        m_btnStartSketch->setEnabled(!m_sketchEditing);
    if (m_btnExitSketch)
        m_btnExitSketch->setEnabled(m_sketchEditing);
    if (m_btnExtrudeFeature)
        m_btnExtrudeFeature->setEnabled(m_hasSelectedSketch);
    if (m_btnRevolveFeature)
        m_btnRevolveFeature->setEnabled(m_hasSelectedSketch);
}

void WidgetCadTaskPanel::updatePrimitiveParameterVisibility()
{
    const int index = primitiveIndex();
    const bool isBox = index == 0;
    const bool isCylinder = index == 1;
    const bool isSphere = index == 2;
    const bool isCone = index == 3;
    const bool isTorus = index == 4;

    setParameterVisible(m_labelPrimitiveSizeX, m_spinPrimitiveSizeX, isBox);
    setParameterVisible(m_labelPrimitiveSizeY, m_spinPrimitiveSizeY, isBox);
    setParameterVisible(m_labelPrimitiveSizeZ, m_spinPrimitiveSizeZ,
                        isBox || isCylinder || isCone);
    setParameterVisible(m_labelPrimitiveRadius1, m_spinPrimitiveRadius1,
                        isCylinder || isSphere || isCone || isTorus);
    setParameterVisible(m_labelPrimitiveRadius2, m_spinPrimitiveRadius2,
                        isCone || isTorus);

    if (m_labelPrimitiveSizeZ)
        m_labelPrimitiveSizeZ->setText(isBox ? tr("高度 Z:") : tr("高度:"));
    if (m_labelPrimitiveRadius1) {
        if (isCone)
            m_labelPrimitiveRadius1->setText(tr("底部半径:"));
        else if (isTorus)
            m_labelPrimitiveRadius1->setText(tr("主半径:"));
        else
            m_labelPrimitiveRadius1->setText(tr("半径:"));
    }
    if (m_labelPrimitiveRadius2)
        m_labelPrimitiveRadius2->setText(isCone ? tr("顶部半径:") : tr("管半径:"));
}

void WidgetCadTaskPanel::emitPrimitiveParametersChanged()
{
    emit primitiveParametersChanged(primitiveIndex(),
                                    primitiveSizeX(),
                                    primitiveSizeY(),
                                    primitiveSizeZ(),
                                    primitiveRadius1(),
                                    primitiveRadius2());
}

void WidgetCadTaskPanel::emitFeatureParametersChanged()
{
    emit featureParametersChanged(featureIndex(),
                                  featureLength(),
                                  featureAngle());
}

void WidgetCadTaskPanel::emitTransformParametersChanged()
{
    emit transformParametersChanged(transformTranslateX(),
                                    transformTranslateY(),
                                    transformTranslateZ(),
                                    transformRotateX(),
                                    transformRotateY(),
                                    transformRotateZ(),
                                    transformReferenceMode());
}

void WidgetCadTaskPanel::rebuildSketchToolForm(int toolKind)
{
    if (!m_sketchToolForm)
        return;
    auto* form = qobject_cast<QFormLayout*>(m_sketchToolForm->layout());
    if (!form)
        return;
    while (form->rowCount() > 0)
        form->removeRow(0);
    m_sketchToolSpins.clear();
    m_sketchToolIntSpins.clear();

    auto addDoubleField = [&](const QString& label, double value, double min, double max) {
        auto* spin = new QDoubleSpinBox(m_sketchToolForm);
        spin->setRange(min, max);
        spin->setDecimals(3);
        spin->setSingleStep(1.0);
        spin->setValue(value);
        spin->setSuffix(tr(" mm"));
        form->addRow(label, spin);
        m_sketchToolSpins.append(spin);
    };
    auto addIntField = [&](const QString& label, int value, int min, int max) {
        auto* spin = new QSpinBox(m_sketchToolForm);
        spin->setRange(min, max);
        spin->setValue(value);
        form->addRow(label, spin);
        m_sketchToolIntSpins.append(spin);
    };

    switch (toolKind) {
    case 1: // Point: x,y
        addDoubleField(tr("X:"), 0.0, -1e6, 1e6);
        addDoubleField(tr("Y:"), 0.0, -1e6, 1e6);
        break;
    case 2: // Line
        addDoubleField(tr("起点 X:"), 0.0, -1e6, 1e6);
        addDoubleField(tr("起点 Y:"), 0.0, -1e6, 1e6);
        addDoubleField(tr("终点 X:"), 50.0, -1e6, 1e6);
        addDoubleField(tr("终点 Y:"), 0.0, -1e6, 1e6);
        break;
    case 3: // Arc
        addDoubleField(tr("起点 X:"), -25.0, -1e6, 1e6);
        addDoubleField(tr("起点 Y:"), 0.0, -1e6, 1e6);
        addDoubleField(tr("中点 X:"), 0.0, -1e6, 1e6);
        addDoubleField(tr("中点 Y:"), 25.0, -1e6, 1e6);
        addDoubleField(tr("终点 X:"), 25.0, -1e6, 1e6);
        addDoubleField(tr("终点 Y:"), 0.0, -1e6, 1e6);
        break;
    case 4: // Circle
        addDoubleField(tr("圆心 X:"), 0.0, -1e6, 1e6);
        addDoubleField(tr("圆心 Y:"), 0.0, -1e6, 1e6);
        addDoubleField(tr("半径:"), 25.0, 0.001, 1e6);
        break;
    case 5: // Rectangle
        addDoubleField(tr("中心 X:"), 0.0, -1e6, 1e6);
        addDoubleField(tr("中心 Y:"), 0.0, -1e6, 1e6);
        addDoubleField(tr("宽度:"), 80.0, 0.001, 1e6);
        addDoubleField(tr("高度:"), 50.0, 0.001, 1e6);
        break;
    case 6: // Polygon
        addDoubleField(tr("中心 X:"), 0.0, -1e6, 1e6);
        addDoubleField(tr("中心 Y:"), 0.0, -1e6, 1e6);
        addDoubleField(tr("半径:"), 30.0, 0.001, 1e6);
        addIntField(tr("边数:"), 6, 3, 64);
        break;
    default:
        break;
    }
    if (m_btnAddSketchElement)
        m_btnAddSketchElement->setEnabled(toolKind > 0);
}

QVector<double> WidgetCadTaskPanel::currentSketchToolParams() const
{
    QVector<double> params;
    for (auto* spin : m_sketchToolSpins)
        params.append(spin->value());
    for (auto* spin : m_sketchToolIntSpins)
        params.append(static_cast<double>(spin->value()));
    return params;
}

} // namespace lcnc::cad::ui