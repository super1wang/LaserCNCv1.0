#include "modules/process/Setting/Setting_ProcessPlugins.h"

#include "modules/process/Setting/Settings.h"
#include "modules/process/steps/process_step_registry.h"

#include <QCheckBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QVBoxLayout>
#include <QWidget>

namespace lcnc::process {

namespace {

constexpr int kColName = 0;
constexpr int kColKey = 1;
constexpr int kColCategory = 2;
constexpr int kColRequired = 3;
constexpr int kColEnabled = 4;
constexpr int kColCount = 5;

QWidget* makeCenteredCheckHost(QCheckBox* check)
{
    auto* host = new QWidget();
    auto* lay = new QHBoxLayout(host);
    lay->setContentsMargins(0, 0, 0, 0);
    lay->addStretch();
    lay->addWidget(check);
    lay->addStretch();
    return host;
}

QCheckBox* checkInCell(QTableWidget* table, int row, int col)
{
    QWidget* host = table->cellWidget(row, col);
    return host ? host->findChild<QCheckBox*>() : nullptr;
}

} // namespace

Dialog_Setting_ProcessPlugins::Dialog_Setting_ProcessPlugins(QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle(tr("流程插件"));

    auto* layout = new QVBoxLayout(this);
    auto* hint = new QLabel(tr("管理工作流可用步骤插件。Required 项不可禁用。"), this);
    layout->addWidget(hint);

    m_table = new QTableWidget(this);
    m_table->setColumnCount(kColCount);
    m_table->setHorizontalHeaderLabels({
        tr("步骤"), tr("Key"), tr("分类"), tr("必需"), tr("启用")
    });
    m_table->horizontalHeader()->setSectionResizeMode(kColName, QHeaderView::Stretch);
    m_table->horizontalHeader()->setSectionResizeMode(kColKey, QHeaderView::ResizeToContents);
    m_table->horizontalHeader()->setSectionResizeMode(kColCategory, QHeaderView::ResizeToContents);
    m_table->horizontalHeader()->setSectionResizeMode(kColRequired, QHeaderView::ResizeToContents);
    m_table->horizontalHeader()->setSectionResizeMode(kColEnabled, QHeaderView::ResizeToContents);
    m_table->verticalHeader()->setVisible(false);
    layout->addWidget(m_table);
}

void Dialog_Setting_ProcessPlugins::InitSetting()
{
}

void Dialog_Setting_ProcessPlugins::SetPage()
{
    rebuildRows();
    m_dirty = false;
}

void Dialog_Setting_ProcessPlugins::GetPage()
{
    // 收集启用状态写入 settings 子表。
    table bucket;
    if (!m_table)
        return;
    for (int r = 0; r < m_table->rowCount(); ++r) {
        const auto* item = m_table->item(r, kColKey);
        if (!item)
            continue;
        const QString key = item->text();
        if (key.isEmpty())
            continue;
        bool enabled = true;
        if (auto* check = checkInCell(m_table, r, kColEnabled))
            enabled = check->isChecked();
        bucket[key.toStdString()] = enabled;
        ProcessStepRegistry::instance().setPluginEnabled(key, enabled);
    }
    table specialTable = SETTINGS->GetTable(SettingSection::Special);
    specialTable["ProcessPlugins"] = bucket;
    SETTINGS->SetTable(true, SettingSection::Special, specialTable);
}

bool Dialog_Setting_ProcessPlugins::GetChanged()
{
    if (!m_dirty)
        return false;
    GetPage();
    m_dirty = false;
    return true;
}

void Dialog_Setting_ProcessPlugins::rebuildRows()
{
    if (!m_table)
        return;
    m_table->setRowCount(0);
    const auto descriptors = ProcessStepRegistry::instance().descriptorsAll();
    for (const auto& d : descriptors) {
        const int r = m_table->rowCount();
        m_table->insertRow(r);
        m_table->setItem(r, kColName, new QTableWidgetItem(d.displayName));
        auto* keyItem = new QTableWidgetItem(d.executorKey);
        keyItem->setFlags(keyItem->flags() & ~Qt::ItemIsEditable);
        m_table->setItem(r, kColKey, keyItem);
        m_table->setItem(r, kColCategory, new QTableWidgetItem(d.category));
        m_table->setItem(r, kColRequired, new QTableWidgetItem(d.required ? tr("是") : tr("否")));

        auto* check = new QCheckBox();
        check->setChecked(d.pluginEnabled);
        check->setEnabled(!d.required);
        connect(check, &QCheckBox::toggled, this, [this](bool) { m_dirty = true; });
        m_table->setCellWidget(r, kColEnabled, makeCenteredCheckHost(check));
    }
}

} // namespace lcnc::process
