#include "modules/process/ui/process_node_edit_dialog.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QStackedWidget>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QHeaderView>

#include "modules/process/settings/process_settings_service.h"
#include "modules/process/steps/process_step_registry.h"
#include "modules/process/workflow/process_node_registry.h"

namespace lcnc::process {

namespace {

QStringList ioNames(ProcessIoBucket bucket)
{
    if (auto* settings = ProcessSettingsService::current()) return settings->ioDisplayNames(bucket);
    return {};
}

QString ioKeyFromDisplay(const QString& display)
{
    const int l = display.lastIndexOf('(');
    const int r = display.lastIndexOf(')');
    if (l >= 0 && r > l)
        return display.mid(l + 1, r - l - 1);
    return display;
}

} // namespace

ProcessNodeEditDialog::ProcessNodeEditDialog(ProcessNode node, QWidget* parent)
    : QDialog(parent)
    , m_node(std::move(node))
{
    setWindowTitle(tr("编辑流程节点"));
    resize(360, 240);

    auto* layout = new QVBoxLayout(this);
    layout->addWidget(buildGeneralPage());

    m_detailStack = new QStackedWidget(this);
    m_detailStack->addWidget(buildParameterPage());
    m_detailStack->addWidget(buildPluginParameterPage());
    m_detailStack->addWidget(buildWaitPage());
    m_detailStack->addWidget(buildAxisPage());
    m_detailStack->addWidget(buildTypedParameterPage());
    m_detailStack->addWidget(buildMultiAxisPage());
    layout->addWidget(m_detailStack);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttons, &QDialogButtonBox::accepted, this, [this] {
        applyNode();
        accept();
    });
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    layout->addWidget(buttons);

    loadNode();
}

ProcessNode ProcessNodeEditDialog::node() const
{
    return m_node;
}

QWidget* ProcessNodeEditDialog::buildGeneralPage()
{
    auto* page = new QWidget(this);
    auto* form = new QFormLayout(page);

    auto* typeLabel = new QLabel(processNodeTypeToString(m_node.type), page);
    form->addRow(tr("类型"), typeLabel);

    m_nameEdit = new QLineEdit(page);
    form->addRow(tr("名称"), m_nameEdit);

    m_enabledCheck = new QCheckBox(tr("启用"), page);
    form->addRow(tr("状态"), m_enabledCheck);
    return page;
}

QWidget* ProcessNodeEditDialog::buildParameterPage()
{
    auto* page = new QWidget(this);
    auto* layout = new QVBoxLayout(page);

    m_parameterTable = new QTableWidget(page);
    m_parameterTable->setColumnCount(2);
    m_parameterTable->setHorizontalHeaderLabels({ tr("参数"), tr("值") });
    m_parameterTable->horizontalHeader()->setStretchLastSection(true);
    layout->addWidget(m_parameterTable);

    auto* buttons = new QHBoxLayout();
    auto* addButton = new QPushButton(tr("添加"), page);
    auto* removeButton = new QPushButton(tr("删除"), page);
    buttons->addWidget(addButton);
    buttons->addWidget(removeButton);
    buttons->addStretch(1);
    layout->addLayout(buttons);

    connect(addButton, &QPushButton::clicked, this, [this] {
        const int row = m_parameterTable->rowCount();
        m_parameterTable->insertRow(row);
        m_parameterTable->setItem(row, 0, new QTableWidgetItem(QStringLiteral("key")));
        m_parameterTable->setItem(row, 1, new QTableWidgetItem(QString()));
    });
    connect(removeButton, &QPushButton::clicked, this, [this] {
        const auto ranges = m_parameterTable->selectedRanges();
        if (ranges.isEmpty())
            return;
        for (int row = ranges.first().bottomRow(); row >= ranges.first().topRow(); --row)
            m_parameterTable->removeRow(row);
    });
    return page;
}

QWidget* ProcessNodeEditDialog::buildPluginParameterPage()
{
    if (auto step = ProcessStepRegistry::instance().step(m_node.type)) {
        m_pluginEditor = step->createParameterEditor(m_node, this);
        if (m_pluginEditor)
            return m_pluginEditor;
    }
    auto* page = new QWidget(this);
    auto* layout = new QVBoxLayout(page);
    layout->addWidget(new QLabel(tr("该步骤暂无插件参数页，使用兼容参数表。"), page));
    return page;
}

QWidget* ProcessNodeEditDialog::buildWaitPage()
{
    auto* page = new QWidget(this);
    auto* form = new QFormLayout(page);

    m_waitDurationSpin = new QSpinBox(page);
    m_waitDurationSpin->setRange(0, 24 * 60 * 60 * 1000);
    m_waitDurationSpin->setSuffix(QStringLiteral(" ms"));
    form->addRow(tr("等待时间"), m_waitDurationSpin);
    return page;
}

QWidget* ProcessNodeEditDialog::buildTypedParameterPage()
{
    auto* page = new QWidget(this);
    auto* form = new QFormLayout(page);

    auto addText = [this, form, page](const QString& key, const QString& label, const QString& placeholder = QString()) {
        auto* editor = new QLineEdit(page);
        editor->setPlaceholderText(placeholder);
        form->addRow(label, editor);
        m_textEditors.insert(key, editor);
    };
    auto addDouble = [this, form, page](const QString& key, const QString& label, double minValue, double maxValue, const QString& suffix = QString()) {
        auto* editor = new QDoubleSpinBox(page);
        editor->setRange(minValue, maxValue);
        editor->setDecimals(3);
        editor->setSuffix(suffix);
        editor->setKeyboardTracking(false);
        form->addRow(label, editor);
        m_doubleEditors.insert(key, editor);
    };
    auto addInt = [this, form, page](const QString& key, const QString& label, int minValue, int maxValue, const QString& suffix = QString()) {
        auto* editor = new QSpinBox(page);
        editor->setRange(minValue, maxValue);
        editor->setSuffix(suffix);
        editor->setKeyboardTracking(false);
        form->addRow(label, editor);
        m_intEditors.insert(key, editor);
    };
    auto addBool = [this, form, page](const QString& key, const QString& label) {
        auto* editor = new QCheckBox(label, page);
        form->addRow(QString(), editor);
        m_boolEditors.insert(key, editor);
    };
    auto addCombo = [this, form, page](const QString& key, const QString& label, const QStringList& values) {
        auto* editor = new QComboBox(page);
        editor->addItems(values);
        form->addRow(label, editor);
        m_comboEditors.insert(key, editor);
        return editor;
    };

    switch (m_node.type) {
    case ProcessNodeType::Start:
        addText(QStringLiteral("variables"), tr("全局变量(JSON)"), QStringLiteral("[]"));
        break;
    case ProcessNodeType::Stop:
        addText(QStringLiteral("message"), tr("停止消息"), QStringLiteral("流程结束"));
        addBool(QStringLiteral("safeStopOutputs"), tr("停止时复位安全输出"));
        addBool(QStringLiteral("stopMotion"), tr("停止时停止运动"));
        break;
    case ProcessNodeType::Axis:
        addText(QStringLiteral("axis"), tr("轴"), QStringLiteral("X"));
        addText(QStringLiteral("mode"), tr("模式 absolute/relative"), QStringLiteral("absolute"));
        addDouble(QStringLiteral("target"), tr("目标位置"), -1000000.0, 1000000.0, QStringLiteral(" mm"));
        addDouble(QStringLiteral("velocity"), tr("速度"), 0.0, 1000000.0, QStringLiteral(" mm/s"));
        addInt(QStringLiteral("timeoutMs"), tr("超时"), 0, 24 * 60 * 60 * 1000, QStringLiteral(" ms"));
        break;
    case ProcessNodeType::AxesMove:
        // 多轴运动使用独立表格页 buildMultiAxisPage()。
        break;
    case ProcessNodeType::IO: {
        auto* typeCombo = addCombo(QStringLiteral("signalType"), tr("输出类型"), { QStringLiteral("digital"), QStringLiteral("analog") });
        auto* ioCombo = addCombo(QStringLiteral("ioName"), tr("IO 名"), ioNames(ProcessIoBucket::DigitalOutput));
        connect(typeCombo, &QComboBox::currentTextChanged, this, [ioCombo](const QString& text) {
            ioCombo->clear();
            ioCombo->addItems(text == QStringLiteral("analog")
                ? ioNames(ProcessIoBucket::AnalogOutput)
                : ioNames(ProcessIoBucket::DigitalOutput));
        });
        addText(QStringLiteral("value"), tr("值"), QStringLiteral("true"));
        break;
    }
    case ProcessNodeType::Monitor: {
        auto* typeCombo = addCombo(QStringLiteral("signalType"), tr("输入类型"), { QStringLiteral("digital"), QStringLiteral("analog") });
        auto* ioCombo = addCombo(QStringLiteral("ioName"), tr("输入 IO"), ioNames(ProcessIoBucket::DigitalInput));
        connect(typeCombo, &QComboBox::currentTextChanged, this, [ioCombo](const QString& text) {
            ioCombo->clear();
            ioCombo->addItems(text == QStringLiteral("analog")
                ? ioNames(ProcessIoBucket::AnalogInput)
                : ioNames(ProcessIoBucket::DigitalInput));
        });
        addBool(QStringLiteral("targetValue"), tr("目标为 true"));
        addInt(QStringLiteral("timeoutMs"), tr("超时"), 0, 24 * 60 * 60 * 1000, QStringLiteral(" ms"));
        addInt(QStringLiteral("pollIntervalMs"), tr("刷新间隔"), 10, 60000, QStringLiteral(" ms"));
        break;
    }
    case ProcessNodeType::Cutting:
        addText(QStringLiteral("selectionMode"), tr("选择模式"), QStringLiteral("allEnabled"));
        addInt(QStringLiteral("startNumber"), tr("起始序号"), 1, 1000000);
        addInt(QStringLiteral("endNumber"), tr("结束序号(0=不限)"), 0, 1000000);
        addText(QStringLiteral("compensationIndex"), tr("补偿索引"));
        break;
    default:
        break;
    }
    return page;
}

QWidget* ProcessNodeEditDialog::buildAxisPage()
{
    auto* page = new QWidget(this);
    auto* form = new QFormLayout(page);

    m_axisNameEdit = new QLineEdit(page);
    m_axisNameEdit->setPlaceholderText(QStringLiteral("X"));
    form->addRow(tr("轴名称"), m_axisNameEdit);

    m_axisPositionSpin = new QDoubleSpinBox(page);
    m_axisPositionSpin->setRange(-1000000.0, 1000000.0);
    m_axisPositionSpin->setDecimals(3);
    form->addRow(tr("目标位置"), m_axisPositionSpin);
    return page;
}

QWidget* ProcessNodeEditDialog::buildMultiAxisPage()
{
    auto* page = new QWidget(this);
    auto* layout = new QVBoxLayout(page);
    auto* form = new QFormLayout();
    m_multiModeCombo = new QComboBox(page);
    m_multiModeCombo->addItem(tr("顺序执行"), QStringLiteral("sequential"));
    m_multiModeCombo->addItem(tr("同步执行"), QStringLiteral("sync"));
    form->addRow(tr("多轴模式"), m_multiModeCombo);
    layout->addLayout(form);

    m_axesTable = new QTableWidget(page);
    m_axesTable->setColumnCount(5);
    m_axesTable->setHorizontalHeaderLabels({ tr("轴"), tr("模式"), tr("目标位置"), tr("速度"), tr("操作") });
    m_axesTable->horizontalHeader()->setStretchLastSection(true);
    layout->addWidget(m_axesTable);

    auto* row = new QHBoxLayout();
    auto* addButton = new QPushButton(tr("添加轴"), page);
    row->addWidget(addButton);
    row->addStretch();
    layout->addLayout(row);
    connect(addButton, &QPushButton::clicked, this, [this] { addAxisRow(); });
    return page;
}

void ProcessNodeEditDialog::addAxisRow(const QVariantMap& row)
{
    if (!m_axesTable)
        return;
    const int r = m_axesTable->rowCount();
    m_axesTable->insertRow(r);

    auto* axis = new QLineEdit(row.value(QStringLiteral("axis"), QStringLiteral("X")).toString(), m_axesTable);
    m_axesTable->setCellWidget(r, 0, axis);

    auto* mode = new QComboBox(m_axesTable);
    mode->addItem(tr("绝对"), QStringLiteral("absolute"));
    mode->addItem(tr("相对"), QStringLiteral("relative"));
    const int modeIndex = mode->findData(row.value(QStringLiteral("mode"), QStringLiteral("absolute")).toString());
    mode->setCurrentIndex(modeIndex < 0 ? 0 : modeIndex);
    m_axesTable->setCellWidget(r, 1, mode);

    auto* target = new QDoubleSpinBox(m_axesTable);
    target->setRange(-1000000.0, 1000000.0);
    target->setDecimals(3);
    target->setValue(row.value(QStringLiteral("target"), 0.0).toDouble());
    m_axesTable->setCellWidget(r, 2, target);

    auto* velocity = new QDoubleSpinBox(m_axesTable);
    velocity->setRange(0.0, 1000000.0);
    velocity->setDecimals(3);
    velocity->setValue(row.value(QStringLiteral("velocity"), 5.0).toDouble());
    m_axesTable->setCellWidget(r, 3, velocity);

    auto* del = new QPushButton(tr("删除"), m_axesTable);
    connect(del, &QPushButton::clicked, this, [this, del] {
        for (int i = 0; i < m_axesTable->rowCount(); ++i) {
            if (m_axesTable->cellWidget(i, 4) == del) {
                m_axesTable->removeRow(i);
                break;
            }
        }
    });
    m_axesTable->setCellWidget(r, 4, del);
}

QVariantList ProcessNodeEditDialog::axesFromTable() const
{
    QVariantList result;
    if (!m_axesTable)
        return result;
    for (int r = 0; r < m_axesTable->rowCount(); ++r) {
        QVariantMap item;
        if (auto* axis = qobject_cast<QLineEdit*>(m_axesTable->cellWidget(r, 0)))
            item.insert(QStringLiteral("axis"), axis->text().trimmed().toUpper());
        if (auto* mode = qobject_cast<QComboBox*>(m_axesTable->cellWidget(r, 1)))
            item.insert(QStringLiteral("mode"), mode->currentData().toString());
        if (auto* target = qobject_cast<QDoubleSpinBox*>(m_axesTable->cellWidget(r, 2)))
            item.insert(QStringLiteral("target"), target->value());
        if (auto* velocity = qobject_cast<QDoubleSpinBox*>(m_axesTable->cellWidget(r, 3)))
            item.insert(QStringLiteral("velocity"), velocity->value());
        if (!item.value(QStringLiteral("axis")).toString().isEmpty())
            result.append(item);
    }
    return result;
}

void ProcessNodeEditDialog::loadMultiAxisPage()
{
    if (!m_axesTable)
        return;
    m_axesTable->setRowCount(0);
    const QString mode = m_node.parameters.value(QStringLiteral("multiMode"), QStringLiteral("sequential")).toString();
    if (m_multiModeCombo) {
        const int index = m_multiModeCombo->findData(mode);
        m_multiModeCombo->setCurrentIndex(index < 0 ? 0 : index);
    }
    const QVariantList rows = m_node.parameters.value(QStringLiteral("axes"), QVariantList{}).toList();
    for (const QVariant& value : rows)
        addAxisRow(value.toMap());
}

void ProcessNodeEditDialog::applyMultiAxisPage()
{
    m_node.parameters.insert(QStringLiteral("multiMode"), m_multiModeCombo ? m_multiModeCombo->currentData().toString() : QStringLiteral("sequential"));
    m_node.parameters.insert(QStringLiteral("axes"), axesFromTable());
}

void ProcessNodeEditDialog::loadNode()
{
    m_nameEdit->setText(m_node.name);
    const bool canDisable = ProcessNodeRegistry::instance().isDisableable(m_node.type);
    m_enabledCheck->setChecked(canDisable ? m_node.enabled : true);
    m_enabledCheck->setEnabled(canDisable);
    loadParameterTable();

    if (m_pluginEditor && ProcessStepRegistry::instance().step(m_node.type)) {
        m_detailStack->setCurrentIndex(1);
    } else if (m_node.type == ProcessNodeType::AxesMove) {
        m_detailStack->setCurrentIndex(5);
        loadMultiAxisPage();
    } else if (!m_textEditors.isEmpty() || !m_doubleEditors.isEmpty() || !m_intEditors.isEmpty() || !m_boolEditors.isEmpty() || !m_comboEditors.isEmpty()) {
        m_detailStack->setCurrentIndex(4);
        loadTypedParameterEditors();
    } else {
        m_detailStack->setCurrentIndex(0);
    }
}

void ProcessNodeEditDialog::applyNode()
{
    m_node.name = m_nameEdit->text().trimmed();
    if (m_node.name.isEmpty())
        m_node.name = defaultProcessNodeName(m_node.type);

    m_node.enabled = m_enabledCheck->isChecked();
    m_node.state = m_node.enabled ? ProcessNodeState::Enabled : ProcessNodeState::Disabled;

    if (m_pluginEditor) {
        if (auto step = ProcessStepRegistry::instance().step(m_node.type)) {
            QString error;
            if (!step->applyParameterEditor(m_pluginEditor, m_node, &error))
                return;
        }
    } else if (m_node.type == ProcessNodeType::AxesMove) {
        applyMultiAxisPage();
    } else if (!m_textEditors.isEmpty() || !m_doubleEditors.isEmpty() || !m_intEditors.isEmpty() || !m_boolEditors.isEmpty() || !m_comboEditors.isEmpty()) {
        applyTypedParameterEditors();
    } else {
        applyParameterTable();
    }
}

void ProcessNodeEditDialog::loadTypedParameterEditors()
{
    for (auto it = m_textEditors.cbegin(); it != m_textEditors.cend(); ++it)
        it.value()->setText(m_node.parameters.value(it.key()).toString());
    if (auto* signalType = m_comboEditors.value(QStringLiteral("signalType"), nullptr)) {
        const QString value = m_node.parameters.value(QStringLiteral("signalType"), signalType->currentText()).toString();
        const int index = signalType->findText(value);
        if (index >= 0)
            signalType->setCurrentIndex(index);
    }
    for (auto it = m_comboEditors.cbegin(); it != m_comboEditors.cend(); ++it) {
        if (it.key() == QStringLiteral("signalType"))
            continue;
        const QString value = m_node.parameters.value(it.key()).toString();
        int index = it.value()->findText(value);
        if (it.key() == QStringLiteral("ioName") && index < 0) {
            for (int i = 0; i < it.value()->count(); ++i) {
                if (ioKeyFromDisplay(it.value()->itemText(i)) == value) {
                    index = i;
                    break;
                }
            }
        }
        if (index >= 0)
            it.value()->setCurrentIndex(index);
    }
    for (auto it = m_doubleEditors.cbegin(); it != m_doubleEditors.cend(); ++it)
        it.value()->setValue(m_node.parameters.value(it.key(), it.value()->minimum()).toDouble());
    for (auto it = m_intEditors.cbegin(); it != m_intEditors.cend(); ++it)
        it.value()->setValue(m_node.parameters.value(it.key(), it.value()->minimum()).toInt());
    for (auto it = m_boolEditors.cbegin(); it != m_boolEditors.cend(); ++it)
        it.value()->setChecked(m_node.parameters.value(it.key(), false).toBool());
}

void ProcessNodeEditDialog::applyTypedParameterEditors()
{
    QVariantMap parameters = m_node.parameters;
    if (m_node.type == ProcessNodeType::Cutting)
        parameters.remove(QStringLiteral("dryRun"));
    for (auto it = m_textEditors.cbegin(); it != m_textEditors.cend(); ++it)
        parameters.insert(it.key(), it.value()->text().trimmed());
    for (auto it = m_comboEditors.cbegin(); it != m_comboEditors.cend(); ++it) {
        const QString text = it.value()->currentText();
        parameters.insert(it.key(), it.key() == QStringLiteral("ioName") ? ioKeyFromDisplay(text) : text);
    }
    for (auto it = m_doubleEditors.cbegin(); it != m_doubleEditors.cend(); ++it)
        parameters.insert(it.key(), it.value()->value());
    for (auto it = m_intEditors.cbegin(); it != m_intEditors.cend(); ++it)
        parameters.insert(it.key(), it.value()->value());
    for (auto it = m_boolEditors.cbegin(); it != m_boolEditors.cend(); ++it)
        parameters.insert(it.key(), it.value()->isChecked());
    m_node.parameters = std::move(parameters);
}

void ProcessNodeEditDialog::loadParameterTable()
{
    m_parameterTable->setRowCount(0);
    for (auto it = m_node.parameters.cbegin(); it != m_node.parameters.cend(); ++it) {
        const int row = m_parameterTable->rowCount();
        m_parameterTable->insertRow(row);
        m_parameterTable->setItem(row, 0, new QTableWidgetItem(it.key()));
        m_parameterTable->setItem(row, 1, new QTableWidgetItem(it.value().toString()));
    }
}

void ProcessNodeEditDialog::applyParameterTable()
{
    QVariantMap parameters;
    for (int row = 0; row < m_parameterTable->rowCount(); ++row) {
        const auto* keyItem = m_parameterTable->item(row, 0);
        const auto* valueItem = m_parameterTable->item(row, 1);
        const QString key = keyItem ? keyItem->text().trimmed() : QString();
        if (key.isEmpty())
            continue;
        parameters.insert(key, valueItem ? valueItem->text().trimmed() : QString());
    }
    m_node.parameters = std::move(parameters);
}

} // namespace lcnc::process
