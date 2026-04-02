#include "app/widget_toolpath_panel.h"
#include "base/laser_toolpath.h"

#include <QVBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QPushButton>
#include <QListWidget>
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

    m_spinLeadInLength = new QDoubleSpinBox(paramGroup);
    m_spinLeadInLength->setRange(0.1, 100.0);
    m_spinLeadInLength->setValue(5.0);
    m_spinLeadInLength->setDecimals(2);
    m_spinLeadInLength->setSuffix(tr(" mm"));
    m_spinLeadInLength->setSingleStep(0.5);
    paramForm->addRow(tr("引刀长度:"), m_spinLeadInLength);

    m_spinNormalAngle = new QDoubleSpinBox(paramGroup);
    m_spinNormalAngle->setRange(-90.0, 90.0);
    m_spinNormalAngle->setValue(0.0);
    m_spinNormalAngle->setDecimals(1);
    m_spinNormalAngle->setSuffix(tr(" °"));
    m_spinNormalAngle->setSingleStep(1.0);
    paramForm->addRow(tr("法线角度:"), m_spinNormalAngle);

    mainLayout->addWidget(paramGroup);

    // ── 面分类 group ───────────────────────────────────────────────────────
    auto* classGroup = new QGroupBox(tr("面分类"), this);
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
    m_comboClassMode->addItem(tr("自动(面分类)"), 1);
    m_comboClassMode->addItem(tr("全部外轮廓(旧)"), 0);
    m_comboClassMode->setToolTip(
        tr("自动: 基于面连通域光滑度分类外表面/截面/内表面，提取交线轮廓\n"
           "全部外轮廓: 使用旧方法提取每个面的外轮廓线"));
    classForm->addRow(tr("提取模式:"), m_comboClassMode);

    mainLayout->addWidget(classGroup);

    // ── 操作 group ─────────────────────────────────────────────────────────
    auto* opsGroup  = new QGroupBox(tr("操作"), this);
    auto* opsLayout = new QVBoxLayout(opsGroup);

    m_btnGenerate  = new QPushButton(tr("生成刀路"), opsGroup);
    m_btnPickLeadIn = new QPushButton(tr("选择引刀位置"), opsGroup);
    m_btnRecalc    = new QPushButton(tr("重新计算"), opsGroup);
    m_btnPreview   = new QPushButton(tr("刀路预览"), opsGroup);
    m_btnPreview->setCheckable(true);
    m_btnPreview->setChecked(true);

    opsLayout->addWidget(m_btnGenerate);
    opsLayout->addWidget(m_btnPickLeadIn);
    opsLayout->addWidget(m_btnRecalc);
    opsLayout->addWidget(m_btnPreview);

    mainLayout->addWidget(opsGroup);

    // ── 轮廓列表 group ────────────────────────────────────────────────────
    auto* contourGroup = new QGroupBox(tr("轮廓列表"), this);
    auto* contourLayout = new QVBoxLayout(contourGroup);

    m_contourList = new QListWidget(contourGroup);
    contourLayout->addWidget(m_contourList);

    mainLayout->addWidget(contourGroup);

    // ── 坐标表 group ──────────────────────────────────────────────────────
    auto* coordGroup  = new QGroupBox(tr("机床坐标"), this);
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

    mainLayout->addWidget(coordGroup);

    // Stretch at bottom
    mainLayout->addStretch(1);

    // ── Signal connections ─────────────────────────────────────────────────
    connect(m_btnGenerate,  &QPushButton::clicked,
            this, &WidgetToolpathPanel::generateRequested);
    connect(m_btnPickLeadIn, &QPushButton::clicked,
            this, &WidgetToolpathPanel::pickLeadInRequested);
    connect(m_btnRecalc,    &QPushButton::clicked,
            this, &WidgetToolpathPanel::recalcRequested);
    connect(m_btnPreview,   &QPushButton::toggled,
            this, &WidgetToolpathPanel::previewToggled);

    connect(m_spinLeadInLength, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &WidgetToolpathPanel::leadInLengthChanged);
    connect(m_spinNormalAngle,  QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &WidgetToolpathPanel::normalAngleChanged);

    connect(m_spinSmoothAngle,  QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &WidgetToolpathPanel::smoothAngleChanged);

    connect(m_comboClassMode, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int index) {
                int mode = m_comboClassMode->itemData(index).toInt();
                emit classificationModeChanged(mode);
            });

    connect(m_contourList, &QListWidget::itemChanged, this,
            [this](QListWidgetItem* item) {
                int row = m_contourList->row(item);
                bool checked = (item->checkState() == Qt::Checked);
                emit contourToggled(row, checked);
            });
}

void WidgetToolpathPanel::setToolpath(LaserToolpath* tp)
{
    m_toolpath = tp;
    updateContourList();

    if (tp) {
        m_spinLeadInLength->setValue(tp->globalLeadInLength());
        m_spinNormalAngle->setValue(tp->globalNormalAngle());
    }
}

void WidgetToolpathPanel::updateContourList()
{
    QSignalBlocker blocker(m_contourList);
    m_contourList->clear();

    if (!m_toolpath) return;

    for (int i = 0; i < m_toolpath->contourCount(); ++i) {
        const LaserContour& c = m_toolpath->contour(i);
        auto* item = new QListWidgetItem(c.name, m_contourList);
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(c.enabled ? Qt::Checked : Qt::Unchecked);

        // Show lead-in status in tooltip
        QString tip;
        if (!c.sourceInfo.isEmpty())
            tip += c.sourceInfo + QStringLiteral("\n");
        if (c.leadIn.valid)
            tip += tr("引刀线已设置 (长度: %1 mm)").arg(c.leadIn.length);
        else
            tip += tr("未设置引刀线");
        item->setToolTip(tip);
    }

    emit contourListUpdated();
}

double WidgetToolpathPanel::leadInLength() const
{
    return m_spinLeadInLength ? m_spinLeadInLength->value() : 5.0;
}

double WidgetToolpathPanel::normalAngle() const
{
    return m_spinNormalAngle ? m_spinNormalAngle->value() : 0.0;
}

double WidgetToolpathPanel::smoothAngle() const
{
    return m_spinSmoothAngle ? m_spinSmoothAngle->value() : 5.0;
}

bool WidgetToolpathPanel::useFaceClassification() const
{
    if (!m_comboClassMode) return true;
    return m_comboClassMode->currentData().toInt() == 1;
}

void WidgetToolpathPanel::showContourCoordinates(int contourIndex)
{
    m_coordTable->setRowCount(0);
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
