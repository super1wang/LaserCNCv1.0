#include "modules/process/settings/process_settings_dialog.h"

#include "modules/process/System/Service.h"
#include "modules/process/settings/process_property_model.h"
#include "modules/process/settings/process_io_table_model.h"
#include "modules/process/settings/process_settings_service.h"

#include <QDialogButtonBox>
#include <QHeaderView>
#include <QInputDialog>
#include <QLineEdit>
#include <QMap>
#include <QMessageBox>
#include <QPushButton>
#include <QSplitter>
#include <QStackedWidget>
#include <QTableView>
#include <QToolBar>
#include <QTreeView>
#include <QTreeWidget>
#include <QVBoxLayout>

namespace lcnc::process {

ProcessSettingsDialog::ProcessSettingsDialog(ProcessSettingsService* settings, Service* runtime, QWidget* parent)
    : QDialog(parent), m_settings(settings), m_runtime(runtime)
{
    setWindowTitle(tr("加工参数"));
    resize(1080, 700);
    auto* layout = new QVBoxLayout(this);
    auto* toolbar = new QToolBar(this);
    toolbar->addAction(tr("新建工具"), this, [this] { createTool(); });
    toolbar->addAction(tr("复制工具"), this, [this] { copyTool(); });
    toolbar->addAction(tr("重命名"), this, [this] { renameTool(); });
    toolbar->addAction(tr("删除工具"), this, [this] { deleteTool(); });
    toolbar->addSeparator();
    toolbar->addAction(tr("添加 I/O"), this, [this] {
        if (!m_ioTable || m_editorStack->currentWidget() != m_ioTable) return;
        QString error;
        if (!m_ioModel->addChannel(&error)) QMessageBox::warning(this, tr("添加 I/O"), error);
    });
    toolbar->addAction(tr("删除 I/O"), this, [this] {
        if (!m_ioTable || m_editorStack->currentWidget() != m_ioTable) return;
        const int row = m_ioTable->currentIndex().row();
        if (row < 0) return;
        QString error;
        if (!m_ioModel->removeChannel(row, &error)) QMessageBox::warning(this, tr("删除 I/O"), error);
    });
    layout->addWidget(toolbar);

    auto* split = new QSplitter(this);
    m_objects = new QTreeWidget(split);
    m_objects->setHeaderLabel(tr("设置对象"));
    auto* right = new QWidget(split);
    auto* rightLayout = new QVBoxLayout(right);
    m_search = new QLineEdit(right);
    m_search->setPlaceholderText(tr("筛选当前对象的参数"));
    rightLayout->addWidget(m_search);
    m_editorStack = new QStackedWidget(right);
    m_properties = new QTreeView(m_editorStack);
    m_model = new ProcessPropertyModel(m_settings, m_properties);
    m_properties->setModel(m_model);
    m_properties->setItemDelegate(new ProcessPropertyDelegate(m_properties));
    m_properties->setAlternatingRowColors(true);
    m_properties->setRootIsDecorated(true);
    m_properties->header()->setStretchLastSection(true);
    m_editorStack->addWidget(m_properties);
    m_ioTable = new QTableView(m_editorStack);
    m_ioModel = new ProcessIoTableModel(m_settings, m_ioTable);
    m_ioTable->setModel(m_ioModel);
    m_ioTable->setAlternatingRowColors(true);
    m_ioTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_ioTable->horizontalHeader()->setStretchLastSection(true);
    m_editorStack->addWidget(m_ioTable);
    rightLayout->addWidget(m_editorStack);
    split->addWidget(m_objects);
    split->addWidget(right);
    split->setStretchFactor(1, 1);
    layout->addWidget(split, 1);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Apply | QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttons->button(QDialogButtonBox::Apply), &QPushButton::clicked, this, [this] { apply(); });
    connect(buttons, &QDialogButtonBox::accepted, this, [this] { apply(); if (!m_settings->hasChanges()) accept(); });
    connect(buttons, &QDialogButtonBox::rejected, this, [this] { m_settings->cancelEdit(); reject(); });
    layout->addWidget(buttons);

    connect(m_objects, &QTreeWidget::currentItemChanged, this, [this] { showCurrentObject(); });
    connect(m_search, &QLineEdit::textChanged, this, [this] { showCurrentObject(); });
    m_settings->refreshDraft();
    rebuildObjectTree();
}

void ProcessSettingsDialog::rebuildObjectTree()
{
    m_objectDescriptors = m_settings->objects();
    m_objects->clear();
    QMap<QString, QTreeWidgetItem*> categories;
    for (int i = 0; i < m_objectDescriptors.size(); ++i) {
        const auto& object = m_objectDescriptors.at(i);
        auto* category = categories.value(object.category);
        if (!category) {
            category = new QTreeWidgetItem(m_objects, {object.category});
            category->setFlags(category->flags() & ~Qt::ItemIsSelectable);
            categories.insert(object.category, category);
        }
        auto* item = new QTreeWidgetItem(category, {object.title});
        item->setData(0, Qt::UserRole, i);
    }
    m_objects->expandAll();
    if (m_objects->topLevelItemCount() && m_objects->topLevelItem(0)->childCount())
        m_objects->setCurrentItem(m_objects->topLevelItem(0)->child(0));
}

void ProcessSettingsDialog::showCurrentObject()
{
    const auto* item = m_objects->currentItem();
    if (!item || !item->parent()) return;
    const int index = item->data(0, Qt::UserRole).toInt();
    if (index < 0 || index >= m_objectDescriptors.size()) return;
    const auto& object = m_objectDescriptors.at(index);
    const bool isIoTable = object.ioTableObject;
    m_search->setVisible(!isIoTable);
    if (isIoTable) {
        m_ioModel->setBucket(object.ioBucket);
        m_editorStack->setCurrentWidget(m_ioTable);
    } else {
        m_model->setObject(object, m_search->text());
        m_properties->expandAll();
        m_editorStack->setCurrentWidget(m_properties);
    }
}

QString ProcessSettingsDialog::selectedToolName() const
{
    const auto* item = m_objects->currentItem();
    if (!item || !item->parent()) return {};
    const int index = item->data(0, Qt::UserRole).toInt();
    if (index < 0 || index >= m_objectDescriptors.size()) return {};
    const QString id = m_objectDescriptors.at(index).id;
    return id.startsWith("tool:") ? id.section(':', 1) : QString();
}

void ProcessSettingsDialog::createTool()
{
    bool ok = false;
    const QString name = QInputDialog::getText(this, tr("新建工具"), tr("工具名称"), QLineEdit::Normal, {}, &ok);
    if (!ok) return;
    QString error;
    if (!m_settings->createTool(name, &error)) QMessageBox::warning(this, tr("新建工具"), error);
    else rebuildObjectTree();
}

void ProcessSettingsDialog::copyTool()
{
    const QString source = selectedToolName();
    if (source.isEmpty()) { QMessageBox::information(this, tr("复制工具"), tr("请先选择工具。")); return; }
    bool ok = false;
    const QString target = QInputDialog::getText(this, tr("复制工具"), tr("新工具名称"), QLineEdit::Normal, source + tr(" 副本"), &ok);
    if (!ok) return;
    QString error;
    if (!m_settings->copyTool(source, target, &error)) QMessageBox::warning(this, tr("复制工具"), error);
    else rebuildObjectTree();
}

void ProcessSettingsDialog::renameTool()
{
    const QString source = selectedToolName();
    if (source.isEmpty()) { QMessageBox::information(this, tr("重命名工具"), tr("请先选择工具。")); return; }
    bool ok = false;
    const QString target = QInputDialog::getText(this, tr("重命名工具"), tr("新名称"), QLineEdit::Normal, source, &ok);
    if (!ok) return;
    QString error;
    if (!m_settings->renameTool(source, target, &error)) QMessageBox::warning(this, tr("重命名工具"), error);
    else rebuildObjectTree();
}

void ProcessSettingsDialog::deleteTool()
{
    const QString name = selectedToolName();
    if (name.isEmpty()) { QMessageBox::information(this, tr("删除工具"), tr("请先选择工具。")); return; }
    if (QMessageBox::question(this, tr("删除工具"), tr("删除工具“%1”？").arg(name)) != QMessageBox::Yes) return;
    QString error;
    if (!m_settings->deleteTool(name, &error)) QMessageBox::warning(this, tr("删除工具"), error);
    else rebuildObjectTree();
}

void ProcessSettingsDialog::apply()
{
    const auto result = m_settings->commit();
    if (!result.success) { QMessageBox::critical(this, tr("应用参数"), result.error); return; }
    if (m_runtime) {
        if (result.changes.domains.contains("devices")) { m_runtime->SetMotionControlTable(); m_runtime->SetLaserTable(); }
        if (result.changes.domains.contains("io") && m_runtime->GetMotionControl()) { m_runtime->GetMotionControl()->SetDigitalTable(); m_runtime->GetMotionControl()->SetAnalogTable(); }
        if (result.changes.domains.contains("tools")) m_runtime->SetToolTable();
        if (result.changes.domains.contains("operations")) m_runtime->SetGasTable();
    }
    rebuildObjectTree();
}

} // namespace lcnc::process
