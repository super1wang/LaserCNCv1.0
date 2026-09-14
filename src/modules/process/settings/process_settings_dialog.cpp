#include "modules/process/settings/process_settings_dialog.h"

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

ProcessSettingsDialog::ProcessSettingsDialog(ProcessSettingsService* settings,
                                             SettingsAppliedHandler settingsApplied,
                                             QWidget* parent)
    : QDialog(parent), m_settings(settings), m_settingsApplied(std::move(settingsApplied))
{
    // 中文翻译：加工参数
    setWindowTitle(tr("Processing parameters"));
    resize(1080, 700);
    auto* layout = new QVBoxLayout(this);
    auto* toolbar = new QToolBar(this);
    // 中文翻译：新建工具
    toolbar->addAction(tr("New tool"), this, [this] { createTool(); });
    // 中文翻译：复制工具
    toolbar->addAction(tr("copy tool"), this, [this] { copyTool(); });
    // 中文翻译：重命名
    m_renameToolAction = toolbar->addAction(tr("Rename"), this, [this] { renameTool(); });
    // 中文翻译：删除工具
    m_deleteToolAction = toolbar->addAction(tr("removal tool"), this, [this] { deleteTool(); });
    toolbar->addSeparator();
    // 中文翻译：添加 I/O
    toolbar->addAction(tr("Add I/O"), this, [this] {
        if (!m_ioTable || m_editorStack->currentWidget() != m_ioTable) return;
        QString error;
        // 中文翻译：添加 I/O
        if (!m_ioModel->addChannel(&error)) QMessageBox::warning(this, tr("Add I/O"), error);
    });
    // 中文翻译：删除 I/O
    toolbar->addAction(tr("Delete I/O"), this, [this] {
        if (!m_ioTable || m_editorStack->currentWidget() != m_ioTable) return;
        const int row = m_ioTable->currentIndex().row();
        if (row < 0) return;
        QString error;
        // 中文翻译：删除 I/O
        if (!m_ioModel->removeChannel(row, &error)) QMessageBox::warning(this, tr("Delete I/O"), error);
    });
    layout->addWidget(toolbar);

    auto* split = new QSplitter(this);
    m_objects = new QTreeWidget(split);
    // 中文翻译：设置对象
    m_objects->setHeaderLabel(tr("Set object"));
    auto* right = new QWidget(split);
    auto* rightLayout = new QVBoxLayout(right);
    m_search = new QLineEdit(right);
    // 中文翻译：筛选当前对象的参数
    m_search->setPlaceholderText(tr("Filter parameters for the current object"));
    rightLayout->addWidget(m_search);
    m_editorStack = new QStackedWidget(right);
    m_properties = new QTreeView(m_editorStack);
    m_model = new ProcessPropertyModel(m_settings, m_properties);
    connect(m_model, &ProcessPropertyModel::fieldEdited, this,
            [this](const QString& objectId, const QString& fieldId) {
        if (objectId == QStringLiteral("controller")
            && fieldId == QStringLiteral("type")) {
            rebuildObjectTree(objectId);
        }
    });
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
    connect(buttons, &QDialogButtonBox::accepted, this, &ProcessSettingsDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &ProcessSettingsDialog::reject);
    layout->addWidget(buttons);

    connect(m_objects, &QTreeWidget::currentItemChanged, this, [this] { showCurrentObject(); });
    connect(m_search, &QLineEdit::textChanged, this, [this] { showCurrentObject(); });
    m_settings->refreshDraft();
    rebuildObjectTree();
}

void ProcessSettingsDialog::rebuildObjectTree(const QString& selectedObjectId)
{
    m_objectDescriptors = m_settings->objects();
    m_objects->clear();
    QMap<QString, QTreeWidgetItem*> categories;
    QMap<QString, QTreeWidgetItem*> objectItems;
    QTreeWidgetItem* restoredSelection = nullptr;
    for (int i = 0; i < m_objectDescriptors.size(); ++i) {
        const auto& object = m_objectDescriptors.at(i);
        QTreeWidgetItem* parent = objectItems.value(object.parentObjectId);
        if (!parent) {
            auto* category = categories.value(object.category);
            if (!category) {
                category = new QTreeWidgetItem(m_objects, {object.category});
                category->setFlags(category->flags() & ~Qt::ItemIsSelectable);
                categories.insert(object.category, category);
            }
            parent = category;
        }
        auto* item = new QTreeWidgetItem(parent, {object.title});
        item->setData(0, Qt::UserRole, i);
        objectItems.insert(object.id, item);
        if (!selectedObjectId.isEmpty() && object.id == selectedObjectId)
            restoredSelection = item;
    }
    m_objects->expandAll();
    if (restoredSelection)
        m_objects->setCurrentItem(restoredSelection);
    else if (m_objects->topLevelItemCount() && m_objects->topLevelItem(0)->childCount())
        m_objects->setCurrentItem(m_objects->topLevelItem(0)->child(0));
}

void ProcessSettingsDialog::showCurrentObject()
{
    const auto* item = m_objects->currentItem();
    const QString selectedTool = selectedToolName();
    const bool protectedDefault = ProcessSettingsService::isDefaultToolName(selectedTool);
    if (m_renameToolAction)
        m_renameToolAction->setEnabled(!selectedTool.isEmpty() && !protectedDefault);
    if (m_deleteToolAction)
        m_deleteToolAction->setEnabled(!selectedTool.isEmpty() && !protectedDefault);
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
    const QString id = selectedObjectId();
    return id.startsWith("tool:") ? id.section(':', 1) : QString();
}

QString ProcessSettingsDialog::selectedObjectId() const
{
    const auto* item = m_objects->currentItem();
    if (!item || !item->parent()) return {};
    const int index = item->data(0, Qt::UserRole).toInt();
    return index >= 0 && index < m_objectDescriptors.size()
        ? m_objectDescriptors.at(index).id : QString();
}

void ProcessSettingsDialog::createTool()
{
    bool ok = false;
    // 中文翻译：新建工具；工具名称
    const QString name = QInputDialog::getText(this, tr("New tool"), tr("Tool name"), QLineEdit::Normal, {}, &ok);
    if (!ok) return;
    QString error;
    // 中文翻译：新建工具
    if (!m_settings->createTool(name, &error)) QMessageBox::warning(this, tr("New tool"), error);
    else rebuildObjectTree();
}

void ProcessSettingsDialog::copyTool()
{
    const QString source = selectedToolName();
    // 中文翻译：复制工具；请先选择工具。
    if (source.isEmpty()) { QMessageBox::information(this, tr("copy tool"), tr("Please select a tool first.")); return; }
    bool ok = false;
    // 中文翻译：复制工具；新工具名称； 副本
    const QString target = QInputDialog::getText(this, tr("copy tool"), tr("New tool name"), QLineEdit::Normal, source + tr("copy"), &ok);
    if (!ok) return;
    QString error;
    // 中文翻译：复制工具
    if (!m_settings->copyTool(source, target, &error)) QMessageBox::warning(this, tr("copy tool"), error);
    else rebuildObjectTree();
}

void ProcessSettingsDialog::renameTool()
{
    const QString source = selectedToolName();
    // 中文翻译：重命名工具；请先选择工具。
    if (source.isEmpty()) { QMessageBox::information(this, tr("rename tool"), tr("Please select a tool first.")); return; }
    bool ok = false;
    // 中文翻译：重命名工具；新名称
    const QString target = QInputDialog::getText(this, tr("rename tool"), tr("new name"), QLineEdit::Normal, source, &ok);
    if (!ok) return;
    QString error;
    // 中文翻译：重命名工具
    if (!m_settings->renameTool(source, target, &error)) QMessageBox::warning(this, tr("rename tool"), error);
    else rebuildObjectTree();
}

void ProcessSettingsDialog::deleteTool()
{
    const QString name = selectedToolName();
    // 中文翻译：删除工具；请先选择工具。
    if (name.isEmpty()) { QMessageBox::information(this, tr("removal tool"), tr("Please select a tool first.")); return; }
    // 中文翻译：删除工具；删除工具“%1”？
    if (QMessageBox::question(this, tr("removal tool"), tr("Delete tool \"%1\"?").arg(name)) != QMessageBox::Yes) return;
    QString error;
    // 中文翻译：删除工具
    if (!m_settings->deleteTool(name, &error)) QMessageBox::warning(this, tr("removal tool"), error);
    else rebuildObjectTree();
}

bool ProcessSettingsDialog::apply()
{
    const QString currentObjectId = selectedObjectId();
    const auto result = m_settings->commit();
    // 中文翻译：应用参数
    if (!result.success) {
        QMessageBox::critical(this, tr("Application parameters"), result.error);
        return false;
    }
    if (m_settingsApplied)
        m_settingsApplied(result.changes);
    rebuildObjectTree(currentObjectId);
    return true;
}

void ProcessSettingsDialog::accept()
{
    if (apply())
        QDialog::accept();
}

void ProcessSettingsDialog::reject()
{
    if (m_settings)
        m_settings->cancelEdit();
    QDialog::reject();
}

} // namespace lcnc::process
