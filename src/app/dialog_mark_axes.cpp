#include "app/dialog_mark_axes.h"
#include "modules/cam_module.h"
#include "base/lcnc_document.h"
#include "base/machine_kinematics.h"
#include "base/xcaf_utils.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QScrollArea>
#include <QComboBox>
#include <QLabel>
#include <QPushButton>
#include <QDialogButtonBox>
#include <QFrame>

#include <TDF_LabelSequence.hxx>

// ── Constructor ───────────────────────────────────────────────────────────────

DialogMarkAxes::DialogMarkAxes(LcncDocument*     doc,
                               MachineKinematics* kin,
                               QWidget*           parent)
    : QDialog(parent)
    , m_doc(doc)
    , m_kin(kin)
{
    setWindowTitle(tr("标记轴系零部件"));
    setMinimumSize(480, 460);
    buildUi();
    populateRows();
}

// ── UI construction ───────────────────────────────────────────────────────────

void DialogMarkAxes::buildUi()
{
    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setSpacing(8);
    mainLayout->setContentsMargins(8, 8, 8, 8);

    // ── Info banner ───────────────────────────────────────────────────────
    auto* infoLabel = new QLabel(
        tr("机台构型: %1 · 请为每个机台零件指定所属轴系").arg(m_kin->configType()),
        this);
    infoLabel->setStyleSheet("color: #888; font-size: 11px;");
    mainLayout->addWidget(infoLabel);

    // ── Separator ─────────────────────────────────────────────────────────
    auto* sep = new QFrame(this);
    sep->setFrameShape(QFrame::HLine);
    sep->setFrameShadow(QFrame::Sunken);
    mainLayout->addWidget(sep);

    // ── Scroll area with per-shape rows ───────────────────────────────────
    m_rowContainer = new QWidget;
    m_grid = new QGridLayout(m_rowContainer);
    m_grid->setColumnStretch(0, 1);
    m_grid->setColumnMinimumWidth(1, 170);
    m_grid->setSpacing(4);

    // Header
    auto* hdrName = new QLabel(tr("零件名称"), m_rowContainer);
    auto* hdrAxis = new QLabel(tr("所属轴系"), m_rowContainer);
    hdrName->setStyleSheet("font-weight: bold;");
    hdrAxis->setStyleSheet("font-weight: bold;");
    m_grid->addWidget(hdrName, 0, 0);
    m_grid->addWidget(hdrAxis, 0, 1);

    auto* scroll = new QScrollArea(this);
    scroll->setWidget(m_rowContainer);
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    mainLayout->addWidget(scroll, 1);

    // ── Bottom button row ─────────────────────────────────────────────────
    auto* autoBtn = new QPushButton(tr("🔍 自动识别"), this);
    autoBtn->setToolTip(tr("根据零件名称自动匹配轴系（可再手动调整）"));
    auto* btns = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    auto* btnRow = new QHBoxLayout;
    btnRow->addWidget(autoBtn);
    btnRow->addStretch();
    btnRow->addWidget(btns);
    mainLayout->addLayout(btnRow);

    connect(autoBtn, &QPushButton::clicked, this, &DialogMarkAxes::onAutoDetect);
    connect(btns, &QDialogButtonBox::accepted, this, &DialogMarkAxes::accept);
    connect(btns, &QDialogButtonBox::rejected, this, &QDialog::reject);
}

// ── Row population ────────────────────────────────────────────────────────────

void DialogMarkAxes::populateRows()
{
    // Clear existing rows (keep header at row 0)
    m_rows.clear();

    // Build axis name list for combo items
    QStringList axisNames;
    axisNames << tr("— 未分配 —");
    for (const auto& axis : m_kin->axes()) {
        QString label = axis.name;
        if (axis.name == "BASE") {
            label = tr("BASE（固定基座）");
        } else if (axis.motionType == MachineAxisDef::Linear) {
            label = tr("%1 轴（线性 ±%2 mm）").arg(axis.name).arg(axis.maxVal, 0, 'f', 0);
        } else {
            label = tr("%1 轴（旋转）").arg(axis.name);
        }
        axisNames << label;
    }

    // Enumerate all Machine entities
    TDF_LabelSequence labels = m_doc->entityLabels(LcncDocument::EntityKind::Machine);
    int row = 1;  // row 0 is header
    for (int i = 1; i <= labels.Length(); ++i) {
        TDF_Label lbl  = labels.Value(i);
        QString entry  = XcafUtils::entry(lbl);
        QString name   = XcafUtils::name(lbl);

        auto* nameLabel = new QLabel(name, m_rowContainer);
        auto* combo     = new QComboBox(m_rowContainer);
        combo->addItems(axisNames);

        // Pre-select current assignment
        const QString currentAxis = m_kin->axisForShape(entry);
        if (!currentAxis.isEmpty()) {
            for (int j = 0; j < m_kin->axes().size(); ++j) {
                if (m_kin->axes()[j].name == currentAxis) {
                    combo->setCurrentIndex(j + 1);  // +1 for "— 未分配 —"
                    break;
                }
            }
        }

        m_grid->addWidget(nameLabel, row, 0);
        m_grid->addWidget(combo,     row, 1);
        m_rows.append({entry, name, combo});
        ++row;
    }
}

// ── Auto-detect slot ──────────────────────────────────────────────────────────

void DialogMarkAxes::onAutoDetect()
{
    CamModule::instance()->autoDetectAxes();

    // Update combos
    for (auto& r : m_rows) {
        const QString axisName = m_kin->axisForShape(r.entry);
        int idx = 0;  // "— 未分配 —"
        if (!axisName.isEmpty()) {
            for (int j = 0; j < m_kin->axes().size(); ++j) {
                if (m_kin->axes()[j].name == axisName) {
                    idx = j + 1;
                    break;
                }
            }
        }
        r.combo->setCurrentIndex(idx);
    }
}

// ── Accept ────────────────────────────────────────────────────────────────────

void DialogMarkAxes::accept()
{
    QMap<QString, QString> entryToAxis;
    const auto& axes = m_kin->axes();
    for (const auto& r : m_rows) {
        const int idx = r.combo->currentIndex();
        if (idx == 0) {
            entryToAxis.insert(r.entry, QString());
        } else {
            const int axisIdx = idx - 1;
            if (axisIdx >= 0 && axisIdx < axes.size())
                entryToAxis.insert(r.entry, axes[axisIdx].name);
        }
    }

    CamModule::instance()->applyAxisAssignments(entryToAxis);
    QDialog::accept();
}
