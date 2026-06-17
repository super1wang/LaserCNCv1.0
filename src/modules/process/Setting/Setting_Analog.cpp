#include "RegexPatterns.h"
#include "Setting_Analog.h"

#include <QCheckBox>
#include <QComboBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLineEdit>
#include <QPushButton>
#include <QRegularExpressionValidator>
#include <QSet>
#include <QSignalBlocker>
#include <QToolButton>
#include <QWidget>

namespace {

constexpr int kColName    = 0;
constexpr int kColIndex   = 1;
constexpr int kColType    = 2;
constexpr int kColEnabled = 3;
constexpr int kColOp      = 4;
constexpr int kColCount   = 5;

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
    if (!host)
        return nullptr;
    return host->findChild<QCheckBox*>();
}

}

Dialog_Setting_Analog::Dialog_Setting_Analog(QWidget* parent)
    : QDialog(parent)
{
    ui.setupUi(this);
    setWindowFlags(Qt::CustomizeWindowHint | Qt::FramelessWindowHint);
    setupTable();
    connect(ui.btnAddIN,  &QPushButton::clicked, this, &Dialog_Setting_Analog::onAddIN);
    connect(ui.btnAddOUT, &QPushButton::clicked, this, &Dialog_Setting_Analog::onAddOUT);
}

Dialog_Setting_Analog::~Dialog_Setting_Analog() = default;

void Dialog_Setting_Analog::setupTable()
{
    QTableWidget* t = ui.tableIO;
    t->setColumnCount(kColCount);
    t->setHorizontalHeaderLabels({
        tr("名称"), tr("索引"), tr("类型"), tr("启用"), tr("操作")
    });
    t->horizontalHeader()->setSectionResizeMode(kColName,    QHeaderView::Stretch);
    t->horizontalHeader()->setSectionResizeMode(kColIndex,   QHeaderView::ResizeToContents);
    t->horizontalHeader()->setSectionResizeMode(kColType,    QHeaderView::ResizeToContents);
    t->horizontalHeader()->setSectionResizeMode(kColEnabled, QHeaderView::ResizeToContents);
    t->horizontalHeader()->setSectionResizeMode(kColOp,      QHeaderView::ResizeToContents);
    t->verticalHeader()->setVisible(false);
    connect(t, &QTableWidget::itemChanged, this, &Dialog_Setting_Analog::markDirty);
}

void Dialog_Setting_Analog::InitSetting()
{
    // 预设种子由 ProcessModule::seedDefaultIOTables 写入；这里不再做静态注入。
}

void Dialog_Setting_Analog::SetPage(table table_Set)
{
    if (!table_Set.size())
        table_Set = SETTINGS->GetTable(SettingSection::Analog);

    auto readBucket = [](const table& bucket, bool isInput) {
        QList<RowDescriptor> rows;
        for (const auto& kv : bucket) {
            const std::string& key = kv.first.data();
            RowDescriptor r;
            r.tomlKey = QString::fromStdString(key);
            r.isInput = isInput;
            const value& v = kv.second;
            if (v.is_table()) {
                const auto& t = v.as_table();
                if (t.count("name"))    r.name    = QString::fromStdString(t.at("name").as_string());
                if (t.count("index"))   r.index   = QString::fromStdString(t.at("index").as_string());
                if (t.count("enabled")) r.enabled = t.at("enabled").as_boolean();
                if (t.count("builtin")) r.builtin = t.at("builtin").as_boolean();
            } else if (v.is_array()) {
                const auto& a = v.as_array();
                if (a.size() >= 1 && a.at(0).is_string()) r.name  = QString::fromStdString(a.at(0).as_string());
                if (a.size() >= 2 && a.at(1).is_string()) r.index = QString::fromStdString(a.at(1).as_string());
            }
            rows.push_back(r);
        }
        return rows;
    };

    QList<RowDescriptor> rows;
    if (table_Set.count("AnalogIN") && table_Set.at("AnalogIN").is_table())
        rows.append(readBucket(table_Set.at("AnalogIN").as_table(), true));
    if (table_Set.count("AnalogOUT") && table_Set.at("AnalogOUT").is_table())
        rows.append(readBucket(table_Set.at("AnalogOUT").as_table(), false));
    rebuildRows(rows);
    m_dirty = false;
}

void Dialog_Setting_Analog::rebuildRows(const QList<RowDescriptor>& rows)
{
    QTableWidget* t = ui.tableIO;
    QSignalBlocker blocker(t);
    t->setRowCount(0);
    for (const RowDescriptor& r : rows)
        appendRow(r);
}

void Dialog_Setting_Analog::appendRow(const RowDescriptor& row)
{
    QTableWidget* t = ui.tableIO;
    int r = t->rowCount();
    t->insertRow(r);

    auto* nameItem = new QTableWidgetItem(row.name);
    if (row.builtin)
        nameItem->setFlags(nameItem->flags() & ~Qt::ItemIsEditable);
    nameItem->setData(Qt::UserRole, row.tomlKey);
    nameItem->setData(Qt::UserRole + 1, row.builtin);
    t->setItem(r, kColName, nameItem);

    auto* indexEdit = new QLineEdit(row.index);
    indexEdit->setValidator(new QRegularExpressionValidator(Regex_Analog_Index(), indexEdit));
    connect(indexEdit, &QLineEdit::textEdited, this, [this](const QString&) { markDirty(); });
    t->setCellWidget(r, kColIndex, indexEdit);

    auto* typeBox = new QComboBox();
    typeBox->addItem(tr("输入"), QStringLiteral("IN"));
    typeBox->addItem(tr("输出"), QStringLiteral("OUT"));
    typeBox->setCurrentIndex(row.isInput ? 0 : 1);
    typeBox->setEnabled(!row.builtin);
    connect(typeBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int) { markDirty(); });
    t->setCellWidget(r, kColType, typeBox);

    auto* enabledCheck = new QCheckBox();
    enabledCheck->setChecked(row.enabled);
    connect(enabledCheck, &QCheckBox::toggled, this, [this](bool) { markDirty(); });
    t->setCellWidget(r, kColEnabled, makeCenteredCheckHost(enabledCheck));

    if (!row.builtin) {
        auto* delBtn = new QToolButton();
        delBtn->setText(tr("删除"));
        connect(delBtn, &QToolButton::clicked, this, [this, indexEdit]() {
            QTableWidget* tw = ui.tableIO;
            for (int i = 0; i < tw->rowCount(); ++i) {
                if (tw->cellWidget(i, kColIndex) == indexEdit) {
                    onDeleteRow(i);
                    break;
                }
            }
        });
        t->setCellWidget(r, kColOp, delBtn);
    } else {
        auto* lbl = new QTableWidgetItem(tr("预设"));
        lbl->setFlags(lbl->flags() & ~Qt::ItemIsEditable);
        lbl->setForeground(Qt::gray);
        t->setItem(r, kColOp, lbl);
    }
}

QList<Dialog_Setting_Analog::RowDescriptor> Dialog_Setting_Analog::readRowsFromUi() const
{
    QList<RowDescriptor> rows;
    QTableWidget* t = ui.tableIO;
    for (int i = 0; i < t->rowCount(); ++i) {
        RowDescriptor r;
        QTableWidgetItem* nameItem = t->item(i, kColName);
        if (!nameItem) continue;
        r.tomlKey = nameItem->data(Qt::UserRole).toString();
        r.builtin = nameItem->data(Qt::UserRole + 1).toBool();
        r.name    = nameItem->text();

        if (auto* le = qobject_cast<QLineEdit*>(t->cellWidget(i, kColIndex)))
            r.index = le->text();
        if (auto* cb = qobject_cast<QComboBox*>(t->cellWidget(i, kColType)))
            r.isInput = cb->currentData().toString() == QStringLiteral("IN");
        if (auto* ck = checkInCell(t, i, kColEnabled))
            r.enabled = ck->isChecked();
        rows.push_back(r);
    }
    return rows;
}

void Dialog_Setting_Analog::GetPage(table& table_Page)
{
    table aIN, aOUT;
    const auto rows = readRowsFromUi();
    for (const auto& r : rows) {
        table entry;
        entry["name"]    = r.name.toStdString();
        entry["index"]   = r.index.toStdString();
        entry["enabled"] = r.enabled;
        entry["builtin"] = r.builtin;
        const std::string key = r.tomlKey.toStdString();
        if (r.isInput)
            aIN[key] = entry;
        else
            aOUT[key] = entry;
    }
    table_Page["AnalogIN"]  = aIN;
    table_Page["AnalogOUT"] = aOUT;
}

bool Dialog_Setting_Analog::GetChanged(table table_Page, table& table_Changed)
{
    if (!m_dirty)
        return false;
    table_Changed["AnalogIN"]  = table_Page["AnalogIN"];
    table_Changed["AnalogOUT"] = table_Page["AnalogOUT"];
    SETTINGS->SetTable(true, SettingSection::Analog, table_Page);
    LOG_OPER_INFO(tr("Setting [Analog] full table rewritten").toUtf8().data());
    m_dirty = false;
    return true;
}

void Dialog_Setting_Analog::markDirty()
{
    m_dirty = true;
}

QString Dialog_Setting_Analog::allocateNewKey(bool isInput) const
{
    const QString prefix = isInput ? QStringLiteral("aIN") : QStringLiteral("aOUT");
    QSet<QString> used;
    for (int i = 0; i < ui.tableIO->rowCount(); ++i) {
        if (auto* item = ui.tableIO->item(i, kColName))
            used.insert(item->data(Qt::UserRole).toString());
    }
    for (int n = 1; n < 1000; ++n) {
        QString candidate = prefix + QString::number(n);
        if (!used.contains(candidate))
            return candidate;
    }
    return prefix + QStringLiteral("X");
}

void Dialog_Setting_Analog::onAddIN()
{
    RowDescriptor r;
    r.tomlKey = allocateNewKey(true);
    r.name    = tr("新输入 %1").arg(r.tomlKey.mid(3));
    r.isInput = true;
    appendRow(r);
    markDirty();
}

void Dialog_Setting_Analog::onAddOUT()
{
    RowDescriptor r;
    r.tomlKey = allocateNewKey(false);
    r.name    = tr("新输出 %1").arg(r.tomlKey.mid(4));
    r.isInput = false;
    appendRow(r);
    markDirty();
}

void Dialog_Setting_Analog::onDeleteRow(int row)
{
    if (row < 0 || row >= ui.tableIO->rowCount())
        return;
    auto* nameItem = ui.tableIO->item(row, kColName);
    if (nameItem && nameItem->data(Qt::UserRole + 1).toBool())
        return;
    ui.tableIO->removeRow(row);
    markDirty();
}

void Dialog_Setting_Analog::SetIDEnabled(bool bEnabled)
{
    QTableWidget* t = ui.tableIO;
    for (int i = 0; i < t->rowCount(); ++i) {
        if (auto* item = t->item(i, kColName)) {
            const bool isBuiltin = item->data(Qt::UserRole + 1).toBool();
            if (isBuiltin) continue;
            Qt::ItemFlags f = item->flags();
            if (bEnabled) f |= Qt::ItemIsEditable;
            else          f &= ~Qt::ItemIsEditable;
            item->setFlags(f);
        }
    }
}

void Dialog_Setting_Analog::SetIndexEnabled(bool bEnabled)
{
    QTableWidget* t = ui.tableIO;
    for (int i = 0; i < t->rowCount(); ++i) {
        if (auto* le = qobject_cast<QLineEdit*>(t->cellWidget(i, kColIndex)))
            le->setEnabled(bEnabled);
    }
}
