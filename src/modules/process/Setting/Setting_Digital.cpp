#include "RegexPatterns.h"
#include "Setting_Digital.h"

#include <QCheckBox>
#include <QComboBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLineEdit>
#include <QPushButton>
#include <QRegularExpression>
#include <QRegularExpressionValidator>
#include <QSet>
#include <QSignalBlocker>
#include <QStyledItemDelegate>
#include <QToolButton>
#include <QWidget>

namespace {

constexpr int kColName     = 0;
constexpr int kColIndex    = 1;
constexpr int kColType     = 2;
constexpr int kColActive   = 3;
constexpr int kColEnabled  = 4;
constexpr int kColShowMain = 5;
constexpr int kColOp       = 6;
constexpr int kColCount    = 7;

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

Dialog_Setting_Digital::Dialog_Setting_Digital(QWidget* parent)
    : QDialog(parent)
{
    ui.setupUi(this);
    setWindowFlags(Qt::CustomizeWindowHint | Qt::FramelessWindowHint);
    setupTable();

    connect(ui.btnAddIN,  &QPushButton::clicked, this, &Dialog_Setting_Digital::onAddIN);
    connect(ui.btnAddOUT, &QPushButton::clicked, this, &Dialog_Setting_Digital::onAddOUT);
}

Dialog_Setting_Digital::~Dialog_Setting_Digital() = default;

void Dialog_Setting_Digital::setupTable()
{
    QTableWidget* t = ui.tableIO;
    t->setColumnCount(kColCount);
    t->setHorizontalHeaderLabels({
        tr("名称"), tr("索引"), tr("类型"), tr("有效电平"),
        tr("启用"), tr("显示到主界面"), tr("操作")
    });
    t->horizontalHeader()->setSectionResizeMode(kColName,  QHeaderView::Stretch);
    t->horizontalHeader()->setSectionResizeMode(kColIndex, QHeaderView::ResizeToContents);
    t->horizontalHeader()->setSectionResizeMode(kColType,  QHeaderView::ResizeToContents);
    t->horizontalHeader()->setSectionResizeMode(kColActive,QHeaderView::ResizeToContents);
    t->horizontalHeader()->setSectionResizeMode(kColEnabled,QHeaderView::ResizeToContents);
    t->horizontalHeader()->setSectionResizeMode(kColShowMain,QHeaderView::ResizeToContents);
    t->horizontalHeader()->setSectionResizeMode(kColOp,    QHeaderView::ResizeToContents);
    t->verticalHeader()->setVisible(false);
    connect(t, &QTableWidget::itemChanged, this, &Dialog_Setting_Digital::markDirty);
}

void Dialog_Setting_Digital::InitSetting()
{
    // 预设种子由 ProcessModule::seedDefaultIOTables 写入；这里不再做静态注入。
}

void Dialog_Setting_Digital::SetPage(table table_Set)
{
    if (!table_Set.size())
        table_Set = SETTINGS->GetTable(SettingSection::Digital);

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
                if (t.count("name"))       r.name       = QString::fromStdString(t.at("name").as_string());
                if (t.count("index"))      r.index      = QString::fromStdString(t.at("index").as_string());
                if (t.count("active"))     r.activeHigh = t.at("active").as_boolean();
                if (t.count("enabled"))    r.enabled    = t.at("enabled").as_boolean();
                if (t.count("showInMain")) r.showInMain = t.at("showInMain").as_boolean();
                if (t.count("builtin"))    r.builtin    = t.at("builtin").as_boolean();
            } else if (v.is_array()) {
                const auto& a = v.as_array();
                if (a.size() >= 1 && a.at(0).is_string()) r.name  = QString::fromStdString(a.at(0).as_string());
                if (a.size() >= 2 && a.at(1).is_string()) r.index = QString::fromStdString(a.at(1).as_string());
                r.enabled = true;
                r.activeHigh = !r.index.startsWith('-');
                if (!r.activeHigh) r.index = r.index.mid(1);
            }
            rows.push_back(r);
        }
        return rows;
    };

    QList<RowDescriptor> rows;
    if (table_Set.count("DigitalIN") && table_Set.at("DigitalIN").is_table())
        rows.append(readBucket(table_Set.at("DigitalIN").as_table(), true));
    if (table_Set.count("DigitalOUT") && table_Set.at("DigitalOUT").is_table())
        rows.append(readBucket(table_Set.at("DigitalOUT").as_table(), false));
    rebuildRows(rows);
    m_dirty = false;
}

void Dialog_Setting_Digital::rebuildRows(const QList<RowDescriptor>& rows)
{
    QTableWidget* t = ui.tableIO;
    QSignalBlocker blocker(t);
    t->setRowCount(0);
    for (const RowDescriptor& r : rows)
        appendRow(r);
}

void Dialog_Setting_Digital::appendRow(const RowDescriptor& row)
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
    indexEdit->setValidator(new QRegularExpressionValidator(Regex_Digital_Index(), indexEdit));
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

    auto* activeBox = new QComboBox();
    activeBox->addItem(tr("高电平"), true);
    activeBox->addItem(tr("低电平"), false);
    activeBox->setCurrentIndex(row.activeHigh ? 0 : 1);
    connect(activeBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int) { markDirty(); });
    t->setCellWidget(r, kColActive, activeBox);

    auto* enabledCheck = new QCheckBox();
    enabledCheck->setChecked(row.enabled);
    connect(enabledCheck, &QCheckBox::toggled, this, [this](bool) { markDirty(); });
    t->setCellWidget(r, kColEnabled, makeCenteredCheckHost(enabledCheck));

    auto* showCheck = new QCheckBox();
    showCheck->setChecked(row.showInMain);
    showCheck->setEnabled(!row.isInput);  // 输入项不上主界面
    connect(showCheck, &QCheckBox::toggled, this, [this](bool) { markDirty(); });
    t->setCellWidget(r, kColShowMain, makeCenteredCheckHost(showCheck));

    if (!row.builtin) {
        auto* delBtn = new QToolButton();
        delBtn->setText(tr("删除"));
        connect(delBtn, &QToolButton::clicked, this, [this, indexEdit]() {
            // 通过 widget 反查行号，避免行索引被删除/插入打乱。
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

QList<Dialog_Setting_Digital::RowDescriptor> Dialog_Setting_Digital::readRowsFromUi() const
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
        if (auto* cb = qobject_cast<QComboBox*>(t->cellWidget(i, kColActive)))
            r.activeHigh = cb->currentData().toBool();
        if (auto* ck = checkInCell(t, i, kColEnabled))
            r.enabled = ck->isChecked();
        if (auto* ck = checkInCell(t, i, kColShowMain))
            r.showInMain = ck->isChecked();
        rows.push_back(r);
    }
    return rows;
}

void Dialog_Setting_Digital::GetPage(table& table_Page)
{
    table digIN, digOUT;
    const auto rows = readRowsFromUi();
    for (const auto& r : rows) {
        table entry;
        entry["name"]       = r.name.toStdString();
        entry["index"]      = r.index.toStdString();
        entry["active"]     = r.activeHigh;
        entry["enabled"]    = r.enabled;
        entry["showInMain"] = r.showInMain;
        entry["builtin"]    = r.builtin;
        const std::string key = r.tomlKey.toStdString();
        if (r.isInput)
            digIN[key] = entry;
        else
            digOUT[key] = entry;
    }
    table_Page["DigitalIN"]  = digIN;
    table_Page["DigitalOUT"] = digOUT;
}

bool Dialog_Setting_Digital::GetChanged(table table_Page, table& table_Changed)
{
    if (!m_dirty)
        return false;
    table_Changed["DigitalIN"]  = table_Page["DigitalIN"];
    table_Changed["DigitalOUT"] = table_Page["DigitalOUT"];
    SETTINGS->SetTable(true, SettingSection::Digital, table_Page);
    LOG_OPER_INFO(tr("Setting [Digital] full table rewritten").toUtf8().data());
    m_dirty = false;
    return true;
}

void Dialog_Setting_Digital::markDirty()
{
    m_dirty = true;
}

QString Dialog_Setting_Digital::allocateNewKey(bool isInput) const
{
    // 预设键以 "a"+词 命名；扩展键以 aIN<n>/aOUT<n> 走通 enum_cast 失败路径
    // （MotionControl 解析时会跳过未注册的扩展键，扩展 IO 暂不入硬件 map）。
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

void Dialog_Setting_Digital::onAddIN()
{
    RowDescriptor r;
    r.tomlKey = allocateNewKey(true);
    r.name    = tr("新输入 %1").arg(r.tomlKey.mid(3));
    r.isInput = true;
    appendRow(r);
    markDirty();
}

void Dialog_Setting_Digital::onAddOUT()
{
    RowDescriptor r;
    r.tomlKey = allocateNewKey(false);
    r.name    = tr("新输出 %1").arg(r.tomlKey.mid(4));
    r.isInput = false;
    appendRow(r);
    markDirty();
}

void Dialog_Setting_Digital::onDeleteRow(int row)
{
    if (row < 0 || row >= ui.tableIO->rowCount())
        return;
    auto* nameItem = ui.tableIO->item(row, kColName);
    if (nameItem && nameItem->data(Qt::UserRole + 1).toBool())
        return;  // 防呆：预设行不能删
    ui.tableIO->removeRow(row);
    markDirty();
}

void Dialog_Setting_Digital::SetIDEnabled(bool bEnabled)
{
    // 兼容旧接口：禁/启用所有名称单元格的可编辑性。
    QTableWidget* t = ui.tableIO;
    for (int i = 0; i < t->rowCount(); ++i) {
        if (auto* item = t->item(i, kColName)) {
            // 预设行始终只读，无视 bEnabled。
            const bool isBuiltin = item->data(Qt::UserRole + 1).toBool();
            if (isBuiltin) continue;
            Qt::ItemFlags f = item->flags();
            if (bEnabled)
                f |= Qt::ItemIsEditable;
            else
                f &= ~Qt::ItemIsEditable;
            item->setFlags(f);
        }
    }
}

void Dialog_Setting_Digital::SetIndexEnabled(bool bEnabled)
{
    QTableWidget* t = ui.tableIO;
    for (int i = 0; i < t->rowCount(); ++i) {
        if (auto* le = qobject_cast<QLineEdit*>(t->cellWidget(i, kColIndex)))
            le->setEnabled(bEnabled);
    }
}
