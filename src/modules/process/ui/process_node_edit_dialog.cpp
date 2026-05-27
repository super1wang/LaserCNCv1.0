#include "modules/process/ui/process_node_edit_dialog.h"

#include <QCheckBox>
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

namespace lcnc::process {

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
    m_detailStack->addWidget(buildWaitPage());
    m_detailStack->addWidget(buildAxisPage());
    m_detailStack->addWidget(buildTypedParameterPage());
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

    switch (m_node.type) {
    case ProcessNodeType::AxesMove:
        addDouble(QStringLiteral("x"), tr("X 位置"), -1000000.0, 1000000.0, QStringLiteral(" mm"));
        addDouble(QStringLiteral("y"), tr("Y 位置"), -1000000.0, 1000000.0, QStringLiteral(" mm"));
        addDouble(QStringLiteral("z"), tr("Z 位置"), -1000000.0, 1000000.0, QStringLiteral(" mm"));
        addDouble(QStringLiteral("feedRate"), tr("进给速度"), 0.0, 1000000.0, QStringLiteral(" mm/s"));
        break;
    case ProcessNodeType::Feeding:
        addDouble(QStringLiteral("feedRate"), tr("进给速度"), 0.0, 1000000.0, QStringLiteral(" mm/s"));
        break;
    case ProcessNodeType::AutoFocus:
        addDouble(QStringLiteral("range"), tr("搜索范围"), 0.0, 1000000.0, QStringLiteral(" mm"));
        addDouble(QStringLiteral("speed"), tr("速度"), 0.0, 1000000.0, QStringLiteral(" mm/s"));
        break;
    case ProcessNodeType::Cutting:
        addText(QStringLiteral("contourId"), tr("轮廓 ID"));
        addDouble(QStringLiteral("feedRate"), tr("进给速度"), 0.0, 1000000.0, QStringLiteral(" mm/s"));
        addDouble(QStringLiteral("laserEnergy"), tr("激光能量"), 0.0, 1000000.0, QStringLiteral(" uJ"));
        addBool(QStringLiteral("dryRun"), tr("Dry-run"));
        break;
    case ProcessNodeType::OverCutting:
        addDouble(QStringLiteral("length"), tr("过切长度"), 0.0, 1000000.0, QStringLiteral(" mm"));
        addDouble(QStringLiteral("feedRate"), tr("进给速度"), 0.0, 1000000.0, QStringLiteral(" mm/s"));
        break;
    case ProcessNodeType::EnergySwitch:
        addDouble(QStringLiteral("laserEnergy"), tr("激光能量"), 0.0, 1000000.0, QStringLiteral(" uJ"));
        break;
    case ProcessNodeType::IO:
        addText(QStringLiteral("channel"), tr("通道"), QStringLiteral("DO0"));
        addText(QStringLiteral("action"), tr("动作"), QStringLiteral("set"));
        addBool(QStringLiteral("value"), tr("输出高电平"));
        break;
    case ProcessNodeType::Commands:
        addText(QStringLiteral("command"), tr("命令"), QStringLiteral("noop"));
        break;
    case ProcessNodeType::Monitor:
        addText(QStringLiteral("signal"), tr("信号"), QStringLiteral("ready"));
        addBool(QStringLiteral("expected"), tr("期望为 true"));
        break;
    case ProcessNodeType::Camera:
        addText(QStringLiteral("cameraId"), tr("相机 ID"), QStringLiteral("default"));
        addInt(QStringLiteral("exposureMs"), tr("曝光"), 0, 3600000, QStringLiteral(" ms"));
        break;
    case ProcessNodeType::Measurement:
        addText(QStringLiteral("target"), tr("测量目标"), QStringLiteral("feature"));
        addDouble(QStringLiteral("tolerance"), tr("容差"), 0.0, 1000000.0, QStringLiteral(" mm"));
        break;
    case ProcessNodeType::MarkAcquire:
        addText(QStringLiteral("markId"), tr("标记 ID"), QStringLiteral("mark"));
        break;
    case ProcessNodeType::Alignment:
        addText(QStringLiteral("method"), tr("对位方法"), QStringLiteral("two-point"));
        break;
    case ProcessNodeType::Loop:
        addInt(QStringLiteral("count"), tr("循环次数"), 1, 1000000);
        break;
    case ProcessNodeType::RunGroup:
    case ProcessNodeType::RunGroupCheck:
        addText(QStringLiteral("groupName"), tr("组名"), QStringLiteral("default"));
        break;
    case ProcessNodeType::If:
    case ProcessNodeType::Compare:
        addText(QStringLiteral("expression"), tr("表达式"));
        break;
    case ProcessNodeType::Calculation:
        addText(QStringLiteral("expression"), tr("表达式"));
        addText(QStringLiteral("output"), tr("输出变量"), QStringLiteral("result"));
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

void ProcessNodeEditDialog::loadNode()
{
    m_nameEdit->setText(m_node.name);
    m_enabledCheck->setChecked(m_node.enabled);
    loadParameterTable();

    switch (m_node.type) {
    case ProcessNodeType::Wait:
        m_detailStack->setCurrentIndex(1);
        m_waitDurationSpin->setValue(m_node.parameters.value(QStringLiteral("durationMs"), 1000).toInt());
        break;
    case ProcessNodeType::Axis:
        m_detailStack->setCurrentIndex(2);
        m_axisNameEdit->setText(m_node.parameters.value(QStringLiteral("axis"), QStringLiteral("X")).toString());
        m_axisPositionSpin->setValue(m_node.parameters.value(QStringLiteral("position"), 0.0).toDouble());
        break;
    default:
        if (!m_textEditors.isEmpty() || !m_doubleEditors.isEmpty() || !m_intEditors.isEmpty() || !m_boolEditors.isEmpty()) {
            m_detailStack->setCurrentIndex(3);
            loadTypedParameterEditors();
        } else {
            m_detailStack->setCurrentIndex(0);
        }
        break;
    }
}

void ProcessNodeEditDialog::applyNode()
{
    m_node.name = m_nameEdit->text().trimmed();
    if (m_node.name.isEmpty())
        m_node.name = defaultProcessNodeName(m_node.type);

    m_node.enabled = m_enabledCheck->isChecked();
    m_node.state = m_node.enabled ? ProcessNodeState::Enabled : ProcessNodeState::Disabled;

    if (m_node.type == ProcessNodeType::Wait) {
        m_node.parameters.insert(QStringLiteral("durationMs"), m_waitDurationSpin->value());
    } else if (m_node.type == ProcessNodeType::Axis) {
        m_node.parameters.insert(QStringLiteral("axis"), m_axisNameEdit->text().trimmed());
        m_node.parameters.insert(QStringLiteral("position"), m_axisPositionSpin->value());
    } else if (!m_textEditors.isEmpty() || !m_doubleEditors.isEmpty() || !m_intEditors.isEmpty() || !m_boolEditors.isEmpty()) {
        applyTypedParameterEditors();
    } else {
        applyParameterTable();
    }
}

void ProcessNodeEditDialog::loadTypedParameterEditors()
{
    for (auto it = m_textEditors.cbegin(); it != m_textEditors.cend(); ++it)
        it.value()->setText(m_node.parameters.value(it.key()).toString());
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
    for (auto it = m_textEditors.cbegin(); it != m_textEditors.cend(); ++it)
        parameters.insert(it.key(), it.value()->text().trimmed());
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
