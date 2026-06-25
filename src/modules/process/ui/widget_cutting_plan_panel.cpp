#include "modules/process/ui/widget_cutting_plan_panel.h"

#include "core/logging/logger.h"
#include "modules/process/Tool/ToolFactory.h"

#include <QComboBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSplitter>
#include <QStringList>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QVBoxLayout>

namespace lcnc::process {

namespace {

// 左栏「图层 → 工具」
constexpr int kLayerColName = 0;
constexpr int kLayerColTool = 1;

// 右栏「链表预览」
constexpr int kPlanColSeq     = 0;
constexpr int kPlanColContour = 1;

constexpr int kRoleLayerId = Qt::UserRole + 1;

} // namespace

WidgetCuttingPlanPanel::WidgetCuttingPlanPanel(ProcessCuttingPlanService* service, QWidget* parent)
    : QWidget(parent)
    , m_service(service)
{
    setObjectName(QStringLiteral("WidgetCuttingPlanPanel"));

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(8, 8, 8, 8);
    root->setSpacing(8);

    // ── 顶部按钮 ────────────────────────────────────────────────────────────
    auto* topRow = new QHBoxLayout();
    m_btnSyncCam = new QPushButton(tr("同步 CAM 图层"), this);
    m_btnApply   = new QPushButton(tr("应用更改"), this);
    m_btnReset   = new QPushButton(tr("重置"), this);
    topRow->addWidget(m_btnSyncCam);
    topRow->addWidget(m_btnApply);
    topRow->addWidget(m_btnReset);
    topRow->addStretch(1);
    m_statusLabel = new QLabel(this);
    m_statusLabel->setStyleSheet("color:#888;");
    topRow->addWidget(m_statusLabel);
    root->addLayout(topRow);

    // ── 左右两栏 ────────────────────────────────────────────────────────────
    auto* splitter = new QSplitter(Qt::Horizontal, this);

    auto* leftWrap = new QWidget(splitter);
    {
        auto* lay = new QVBoxLayout(leftWrap);
        lay->setContentsMargins(0, 0, 0, 0);
        lay->setSpacing(4);
        lay->addWidget(new QLabel(tr("图层 → 工具映射"), leftWrap));
        m_layerTable = new QTableWidget(0, 2, leftWrap);
        m_layerTable->setHorizontalHeaderLabels({tr("图层名"), tr("工具")});
        m_layerTable->verticalHeader()->setVisible(false);
        m_layerTable->setSelectionBehavior(QAbstractItemView::SelectRows);
        m_layerTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
        m_layerTable->horizontalHeader()->setSectionResizeMode(kLayerColName, QHeaderView::Stretch);
        m_layerTable->horizontalHeader()->setSectionResizeMode(kLayerColTool, QHeaderView::Stretch);
        lay->addWidget(m_layerTable);
    }

    auto* rightWrap = new QWidget(splitter);
    {
        auto* lay = new QVBoxLayout(rightWrap);
        lay->setContentsMargins(0, 0, 0, 0);
        lay->setSpacing(4);
        lay->addWidget(new QLabel(tr("当前切割链表（按加工顺序）"), rightWrap));
        m_preview = new QTableWidget(0, 2, rightWrap);
        m_preview->setHorizontalHeaderLabels({tr("#"), tr("轮廓名")});
        m_preview->verticalHeader()->setVisible(false);
        m_preview->setSelectionBehavior(QAbstractItemView::SelectRows);
        m_preview->setEditTriggers(QAbstractItemView::NoEditTriggers);
        m_preview->horizontalHeader()->setSectionResizeMode(kPlanColSeq, QHeaderView::ResizeToContents);
        m_preview->horizontalHeader()->setSectionResizeMode(kPlanColContour, QHeaderView::Stretch);
        lay->addWidget(m_preview);
    }

    splitter->addWidget(leftWrap);
    splitter->addWidget(rightWrap);
    splitter->setStretchFactor(0, 1);
    splitter->setStretchFactor(1, 1);
    root->addWidget(splitter, 1);

    connect(m_btnSyncCam, &QPushButton::clicked, this, &WidgetCuttingPlanPanel::onSyncFromCam);
    connect(m_btnApply,   &QPushButton::clicked, this, &WidgetCuttingPlanPanel::onApply);
    connect(m_btnReset,   &QPushButton::clicked, this, &WidgetCuttingPlanPanel::onResetFromService);

    if (m_service) {
        connect(m_service.data(), &ProcessCuttingPlanService::planChanged,
                this, &WidgetCuttingPlanPanel::refreshFromService);
    }
    refreshFromService();
}

WidgetCuttingPlanPanel::~WidgetCuttingPlanPanel() = default;

void WidgetCuttingPlanPanel::refreshFromService()
{
    if (!m_service) return;
    rebuildLayerTable();
    rebuildPreview();
}

void WidgetCuttingPlanPanel::rebuildLayerTable()
{
    if (!m_service || !m_layerTable) return;

    QSignalBlocker block(m_layerTable);
    m_layerTable->clearContents();
    const auto jobs = m_service->layerJobs();
    const QStringList tools = m_service->availableToolNames();

    m_layerTable->setRowCount(jobs.size());
    for (int row = 0; row < jobs.size(); ++row) {
        const ProcessLayerJob& job = jobs[row];

        auto* nameItem = new QTableWidgetItem(job.layerName);
        nameItem->setFlags(nameItem->flags() & ~Qt::ItemIsEditable);
        nameItem->setData(kRoleLayerId, static_cast<qulonglong>(job.layerId));
        m_layerTable->setItem(row, kLayerColName, nameItem);

        auto* combo = new QComboBox(m_layerTable);
        combo->addItems(tools);
        const int idx = combo->findText(job.toolName);
        if (idx >= 0) combo->setCurrentIndex(idx);
        else if (!job.toolName.isEmpty()) {
            // 当前持久化的 toolName 不在 ToolFactory 候选里，临时添加避免误擦。
            combo->addItem(job.toolName);
            combo->setCurrentText(job.toolName);
        }
        m_layerTable->setCellWidget(row, kLayerColTool, combo);
    }
    if (m_statusLabel) {
        m_statusLabel->setText(tr("共 %1 个图层").arg(jobs.size()));
    }
}

void WidgetCuttingPlanPanel::rebuildPreview()
{
    if (!m_service || !m_preview) return;
    const auto plan = m_service->buildCuttingList({});
    QSignalBlocker block(m_preview);
    m_preview->clearContents();
    m_preview->setRowCount(plan.size());
    for (int row = 0; row < plan.size(); ++row) {
        const auto& e = plan[row];
        auto* seqItem = new QTableWidgetItem(QString::number(row + 1));
        seqItem->setFlags(seqItem->flags() & ~Qt::ItemIsEditable);
        seqItem->setTextAlignment(Qt::AlignCenter);
        m_preview->setItem(row, kPlanColSeq, seqItem);

        const QString name = e.contourName.isEmpty()
                                 ? tr("轮廓 #%1").arg(e.contourId)
                                 : e.contourName;
        auto* nameItem = new QTableWidgetItem(name);
        nameItem->setFlags(nameItem->flags() & ~Qt::ItemIsEditable);
        m_preview->setItem(row, kPlanColContour, nameItem);
    }
}

void WidgetCuttingPlanPanel::onSyncFromCam()
{
    if (!m_service) return;
    m_service->syncFromCam();
    // refreshFromService 由 planChanged 信号驱动；这里立即调一次防止信号被屏蔽。
    refreshFromService();
}

void WidgetCuttingPlanPanel::onResetFromService()
{
    refreshFromService();
}

void WidgetCuttingPlanPanel::onApply()
{
    if (!m_service || !m_layerTable) return;

    auto jobs = m_service->layerJobs();
    QHash<std::uint64_t, ProcessLayerJob*> byId;
    byId.reserve(jobs.size());
    for (auto& j : jobs) byId.insert(j.layerId, &j);

    for (int row = 0; row < m_layerTable->rowCount(); ++row) {
        auto* nameItem = m_layerTable->item(row, kLayerColName);
        auto* combo    = qobject_cast<QComboBox*>(m_layerTable->cellWidget(row, kLayerColTool));
        if (!nameItem || !combo) continue;
        const auto layerId = static_cast<std::uint64_t>(
            nameItem->data(kRoleLayerId).toULongLong());
        auto it = byId.find(layerId);
        if (it == byId.end()) continue;
        it.value()->toolName = combo->currentText();
    }
    m_service->setLayerJobs(jobs);
    LCNC_INFO(lcnc::LogCode::Generic,
              "process.cuttingPlan: apply {} layer-tool mappings", jobs.size());
}

} // namespace lcnc::process
