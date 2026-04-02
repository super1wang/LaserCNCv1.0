#include "app/widget_machine_panel.h"
#include "modules/cam_module.h"
#include "base/lcnc_document.h"
#include "base/machine_kinematics.h"
#include "base/xcaf_utils.h"

#include <QVBoxLayout>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QPushButton>
#include <QLabel>
#include <QDoubleSpinBox>

#include <TDF_LabelSequence.hxx>

// ── Constructor ───────────────────────────────────────────────────────────────

WidgetMachinePanel::WidgetMachinePanel(QWidget* parent)
    : QWidget(parent)
{
    buildUi();
}

// ── setDocument ───────────────────────────────────────────────────────────────

void WidgetMachinePanel::setDocument(LcncDocument* doc)
{
    m_doc = doc;
    rebuildAxisRows();
    rebuildWpcSection();
    rebuildMarkButtons();

    if (doc) {
        MachineKinematics* kin = doc->machineKinematics();
        const QString cfg = kin->configType();
        m_lblConfigType->setText(cfg.isEmpty() ? tr("（未配置）") : cfg);
        m_lblMachineName->setText(doc->name());
    } else {
        m_lblConfigType->setText(tr("（无文档）"));
        m_lblMachineName->setText(tr("—"));
    }
}

// ── UI construction ───────────────────────────────────────────────────────────

void WidgetMachinePanel::buildUi()
{
    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(4, 4, 4, 4);
    mainLayout->setSpacing(6);

    // ── 机台配置 section ──────────────────────────────────────────────────
    auto* cfgGroup = new QGroupBox(tr("机台配置"), this);
    auto* cfgLayout = new QVBoxLayout(cfgGroup);
    cfgLayout->setSpacing(4);

    auto* infoRow = new QFormLayout;
    m_lblMachineName = new QLabel(tr("—"), this);
    m_lblConfigType  = new QLabel(tr("（未配置）"), this);
    m_lblMachineName->setStyleSheet("color: #aaa;");
    m_lblConfigType->setStyleSheet("color: #aaa; font-size: 11px;");
    infoRow->addRow(tr("文档:"),  m_lblMachineName);
    infoRow->addRow(tr("构型:"),  m_lblConfigType);
    cfgLayout->addLayout(infoRow);

    auto* btnLoad   = new QPushButton(QIcon(":/icons/machine.svg"),
                                      tr("加载机台模型..."), this);
    auto* btnMark   = new QPushButton(QIcon(":/icons/coordinate.svg"),
                                      tr("标记轴系..."), this);
    auto* btnUnload = new QPushButton(QIcon(":/icons/machine.svg"),
                                      tr("卸载机台"), this);
    auto* btnExport = new QPushButton(QIcon(":/icons/export.svg"),
                                      tr("导出机台模型..."), this);
    auto* btnMove   = new QPushButton(QIcon(":/icons/move.svg"),
                                      tr("移动部件..."), this);
    auto* btnRotate = new QPushButton(QIcon(":/icons/rotate.svg"),
                                      tr("旋转部件..."), this);
    auto* btnDelete = new QPushButton(QIcon(":/icons/delete.svg"),
                                      tr("删除部件"), this);
    cfgLayout->addWidget(btnLoad);
    cfgLayout->addWidget(btnMark);
    cfgLayout->addWidget(btnUnload);
    cfgLayout->addWidget(btnExport);
    cfgLayout->addWidget(btnMove);
    cfgLayout->addWidget(btnRotate);
    cfgLayout->addWidget(btnDelete);
    mainLayout->addWidget(cfgGroup);

    connect(btnLoad,   &QPushButton::clicked, this, &WidgetMachinePanel::loadMachineRequested);
    connect(btnMark,   &QPushButton::clicked, this, &WidgetMachinePanel::markAxesRequested);
    connect(btnUnload, &QPushButton::clicked, this, &WidgetMachinePanel::unloadMachineRequested);
    connect(btnExport, &QPushButton::clicked, this, &WidgetMachinePanel::exportMachineRequested);
    connect(btnMove,   &QPushButton::clicked, this, &WidgetMachinePanel::moveMachineShapeRequested);
    connect(btnRotate, &QPushButton::clicked, this, &WidgetMachinePanel::rotateMachineShapeRequested);
    connect(btnDelete, &QPushButton::clicked, this, &WidgetMachinePanel::deleteMachineShapeRequested);

    // ── 轴系位置 section ──────────────────────────────────────────────────
    m_axisGroup  = new QGroupBox(tr("轴系位置"), this);
    m_axisLayout = new QFormLayout(m_axisGroup);
    m_axisLayout->setSpacing(4);
    auto* axisPlaceholder = new QLabel(tr("加载机台后显示"), m_axisGroup);
    axisPlaceholder->setStyleSheet("color: gray; font-size: 11px;");
    m_axisLayout->addRow(axisPlaceholder);
    mainLayout->addWidget(m_axisGroup);

    // ── 工件挂载 section ──────────────────────────────────────────────────
    m_wpcGroup  = new QGroupBox(tr("工件挂载"), this);
    m_wpcLayout = new QFormLayout(m_wpcGroup);
    m_wpcLayout->setSpacing(4);
    auto* btnMount = new QPushButton(QIcon(":/icons/workpiece.svg"),
                                      tr("挂载工件..."), this);
    m_wpcLayout->addRow(btnMount);
    mainLayout->addWidget(m_wpcGroup);

    connect(btnMount, &QPushButton::clicked, this, &WidgetMachinePanel::mountWorkpieceRequested);

    // ── 标记所选形体 section ───────────────────────────────────────────────
    m_markGroup = new QGroupBox(tr("标记所选形体"), this);
    auto* markOuterLayout = new QVBoxLayout(m_markGroup);
    markOuterLayout->setContentsMargins(4, 2, 4, 2);
    markOuterLayout->setSpacing(2);
    m_markWidget = new QWidget(m_markGroup);
    markOuterLayout->addWidget(m_markWidget);
    {
        auto* l = new QVBoxLayout(m_markWidget);
        l->setContentsMargins(0, 0, 0, 0);
        auto* lbl = new QLabel(tr("加载机台后显示"), m_markWidget);
        lbl->setStyleSheet("color: gray; font-size: 11px;");
        l->addWidget(lbl);
    }
    mainLayout->addWidget(m_markGroup);

    mainLayout->addStretch();
}

// ── Axis spinbox rebuild ──────────────────────────────────────────────────────

static void clearFormLayout(QFormLayout* fl)
{
    while (fl->rowCount() > 0)
        fl->removeRow(0);
}

void WidgetMachinePanel::rebuildAxisRows()
{
    m_axisSpin.clear();
    clearFormLayout(m_axisLayout);

    if (!m_doc) {
        auto* lbl = new QLabel(tr("加载机台后显示"), m_axisGroup);
        lbl->setStyleSheet("color: gray; font-size: 11px;");
        m_axisLayout->addRow(lbl);
        return;
    }

    MachineKinematics* kin = m_doc->machineKinematics();
    if (kin->axes().isEmpty()) {
        auto* lbl = new QLabel(tr("未配置轴系"), m_axisGroup);
        lbl->setStyleSheet("color: gray; font-size: 11px;");
        m_axisLayout->addRow(lbl);
        return;
    }

    for (const auto& axis : kin->axes()) {
        if (axis.name == "BASE") continue;  // fixed base — no user control

        auto* sb = new QDoubleSpinBox(m_axisGroup);
        sb->setDecimals(axis.motionType == MachineAxisDef::Linear ? 3 : 2);
        sb->setSuffix(axis.motionType == MachineAxisDef::Linear
                      ? QStringLiteral(" mm") : QStringLiteral(" °"));
        const double viewMin = qMax(axis.minVal, -9999.0);
        const double viewMax = qMin(axis.maxVal,  9999.0);
        sb->setRange(viewMin, viewMax);
        sb->setValue(axis.currentPos);
        sb->setSingleStep(axis.motionType == MachineAxisDef::Linear ? 1.0 : 0.5);

        m_axisLayout->addRow(tr("%1 轴:").arg(axis.name), sb);
        m_axisSpin.insert(axis.name, sb);

        const QString axisName = axis.name;
        connect(sb, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, [this, axisName](double v) {
                    onAxisSpinChanged(axisName, v);
                });
    }
}

// ── Workpiece mount section rebuild ──────────────────────────────────────────

void WidgetMachinePanel::rebuildWpcSection()
{
    clearFormLayout(m_wpcLayout);

    auto* btnMount = new QPushButton(QIcon(":/icons/workpiece.svg"),
                                      tr("挂载工件..."), m_wpcGroup);
    connect(btnMount, &QPushButton::clicked, this, &WidgetMachinePanel::mountWorkpieceRequested);

    if (m_doc) {
        MachineKinematics* kin = m_doc->machineKinematics();
        const auto& mounts = kin->wpcMounts();

        if (!mounts.isEmpty()) {
            TDF_LabelSequence wpcLabels = m_doc->entityLabels(LcncDocument::EntityKind::Workpiece);
            QMap<QString,QString> wpcNames;
            for (int i = 1; i <= wpcLabels.Length(); ++i) {
                TDF_Label lbl = wpcLabels.Value(i);
                wpcNames.insert(XcafUtils::entry(lbl), XcafUtils::name(lbl));
            }
            for (auto it = mounts.cbegin(); it != mounts.cend(); ++it) {
                const QString wName = wpcNames.value(it.key(), it.key());
                auto* row = new QLabel(
                    tr("<b>%1</b>  →  %2 轴").arg(wName, it.value()),
                    m_wpcGroup);
                row->setStyleSheet("color: #3a8; font-size: 11px;");
                m_wpcLayout->addRow(row);
            }
        } else {
            auto* lbl = new QLabel(tr("暂无工件挂载"), m_wpcGroup);
            lbl->setStyleSheet("color: gray; font-size: 11px;");
            m_wpcLayout->addRow(lbl);
        }
    }
    m_wpcLayout->addRow(btnMount);
}

// ── setSelectedEntries / rebuildMarkButtons ─────────────────────────────────────

void WidgetMachinePanel::setSelectedEntries(const QStringList& entries)
{
    m_selectedEntries = entries;
}

void WidgetMachinePanel::rebuildMarkButtons()
{
    if (m_markWidget) {
        delete m_markWidget;
        m_markWidget = nullptr;
    }
    m_markWidget = new QWidget(m_markGroup);
    m_markGroup->layout()->addWidget(m_markWidget);

    if (!m_doc) {
        auto* l = new QVBoxLayout(m_markWidget);
        l->setContentsMargins(0, 0, 0, 0);
        auto* lbl = new QLabel(tr("加载机台后显示"), m_markWidget);
        lbl->setStyleSheet("color: gray; font-size: 11px;");
        l->addWidget(lbl);
        return;
    }

    MachineKinematics* kin = m_doc->machineKinematics();
    if (kin->axes().isEmpty()) {
        auto* l = new QVBoxLayout(m_markWidget);
        l->setContentsMargins(0, 0, 0, 0);
        auto* lbl = new QLabel(tr("未配置轴系"), m_markWidget);
        lbl->setStyleSheet("color: gray; font-size: 11px;");
        l->addWidget(lbl);
        return;
    }

    auto* grid = new QGridLayout(m_markWidget);
    grid->setSpacing(3);
    grid->setContentsMargins(0, 0, 0, 0);

    int idx = 0;
    for (const auto& axis : kin->axes()) {
        auto* btn = new QPushButton(tr("→%1").arg(axis.name), m_markWidget);
        btn->setToolTip(tr("将所选形体标记为 %1 轴").arg(axis.name));
        btn->setStyleSheet("font-size: 11px; padding: 2px;");
        grid->addWidget(btn, idx / 3, idx % 3);
        ++idx;
        const QString axisName = axis.name;
        connect(btn, &QPushButton::clicked, this, [this, axisName] {
            if (m_selectedEntries.isEmpty() || !m_doc) return;
            CamModule::instance()->assignShapesToAxis(m_selectedEntries, axisName);
        });
    }
}

// ── Axis spinbox changed ──────────────────────────────────────────────────────

void WidgetMachinePanel::onAxisSpinChanged(const QString& axisName, double value)
{
    emit axisPositionChanged(axisName, value);
}
