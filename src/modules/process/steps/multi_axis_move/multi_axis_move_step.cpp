#include "modules/process/steps/multi_axis_move/multi_axis_move_step.h"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHeaderView>
#include <QLineEdit>
#include <QObject>
#include <QPushButton>
#include <QSpinBox>
#include <QTableWidget>
#include <QVBoxLayout>
#include <QWidget>

namespace lcnc::process {

namespace {
constexpr char kMode[] = "multiMode";
constexpr char kAxes[] = "axes";
constexpr char kTimeout[] = "timeoutMs";
constexpr char kTable[] = "axesTable";
constexpr char kModeBox[] = "multiModeBox";
constexpr char kTimeoutSpin[] = "timeoutSpin";

void addAxisRow(QTableWidget* table, const QVariantMap& row = {})
{
    const int r = table->rowCount();
    table->insertRow(r);
    auto* axis = new QLineEdit(row.value("axis", "X").toString(), table);
    table->setCellWidget(r, 0, axis);
    auto* mode = new QComboBox(table);
    // 中文翻译：绝对
    mode->addItem(QObject::tr("Absolutely"), "absolute");
    // 中文翻译：相对
    mode->addItem(QObject::tr("relatively"), "relative");
    const int mi = mode->findData(row.value("mode", "absolute").toString());
    mode->setCurrentIndex(mi < 0 ? 0 : mi);
    table->setCellWidget(r, 1, mode);
    auto* target = new QDoubleSpinBox(table);
    target->setRange(-1000000, 1000000); target->setDecimals(3); target->setValue(row.value("target", 0.0).toDouble());
    table->setCellWidget(r, 2, target);
    auto* velocity = new QDoubleSpinBox(table);
    velocity->setRange(0, 1000000); velocity->setDecimals(3); velocity->setValue(row.value("velocity", 5.0).toDouble());
    table->setCellWidget(r, 3, velocity);
    // 中文翻译：删除
    auto* del = new QPushButton(QObject::tr("Delete"), table);
    QObject::connect(del, &QPushButton::clicked, table, [table, del] {
        for (int i = 0; i < table->rowCount(); ++i) {
            if (table->cellWidget(i, 4) == del) { table->removeRow(i); break; }
        }
    });
    table->setCellWidget(r, 4, del);
}

QVariantList rowsFromTable(QTableWidget* table)
{
    QVariantList rows;
    for (int r = 0; r < table->rowCount(); ++r) {
        QVariantMap row;
        if (auto* axis = qobject_cast<QLineEdit*>(table->cellWidget(r, 0))) row.insert(QStringLiteral("axis"), axis->text().trimmed().toUpper());
        if (auto* mode = qobject_cast<QComboBox*>(table->cellWidget(r, 1))) row.insert(QStringLiteral("mode"), mode->currentData().toString());
        if (auto* target = qobject_cast<QDoubleSpinBox*>(table->cellWidget(r, 2))) row.insert(QStringLiteral("target"), target->value());
        if (auto* velocity = qobject_cast<QDoubleSpinBox*>(table->cellWidget(r, 3))) row.insert(QStringLiteral("velocity"), velocity->value());
        if (!row.value("axis").toString().isEmpty()) rows.append(row);
    }
    return rows;
}
}

ProcessNodeDescriptor MultiAxisMoveStep::descriptor() const
{
    ProcessNodeDescriptor d;
    d.type = ProcessNodeType::MultiAxisMove;
    // 中文翻译：多轴运动
    d.displayName = QObject::tr("multi-axis motion");
    // 中文翻译：运动
    d.category = QObject::tr("sports");
    d.executorKey = QStringLiteral("multiAxisMove");
    d.defaultParameters.insert(QString::fromLatin1(kMode), QStringLiteral("sequential"));
    d.defaultParameters.insert(QString::fromLatin1(kAxes), QVariantList{});
    d.defaultParameters.insert(QString::fromLatin1(kTimeout), 30000);
    return d;
}

QString MultiAxisMoveStep::summary(const ProcessNode& node) const
{
    // 中文翻译：%1，多轴 %2 项
    return QObject::tr("%1, multi-axis %2 items").arg(node.parameters.value(kMode, "sequential").toString(), QString::number(node.parameters.value(kAxes, QVariantList{}).toList().size()));
}

QWidget* MultiAxisMoveStep::createParameterEditor(const ProcessNode& node, QWidget* parent) const
{
    auto* page = new QWidget(parent);
    auto* layout = new QVBoxLayout(page);
    auto* form = new QFormLayout();
    // 中文翻译：顺序执行；同步执行
    auto* mode = new QComboBox(page); mode->setObjectName(kModeBox); mode->addItem(QObject::tr("sequential execution"), "sequential"); mode->addItem(QObject::tr("Synchronous execution"), "sync");
    const int mi = mode->findData(node.parameters.value(kMode, "sequential").toString()); mode->setCurrentIndex(mi < 0 ? 0 : mi);
    // 中文翻译：多轴模式
    form->addRow(QObject::tr("multi-axis mode"), mode);
    auto* timeout = new QSpinBox(page); timeout->setObjectName(kTimeoutSpin); timeout->setRange(0, 24*60*60*1000); timeout->setSuffix(" ms"); timeout->setValue(node.parameters.value(kTimeout, 30000).toInt());
    // 中文翻译：超时
    form->addRow(QObject::tr("timeout"), timeout);
    layout->addLayout(form);
    // 中文翻译：轴；模式；目标位置；速度；操作
    auto* table = new QTableWidget(page); table->setObjectName(kTable); table->setColumnCount(5); table->setHorizontalHeaderLabels({QObject::tr("axis"), QObject::tr("mode"), QObject::tr("Target location"), QObject::tr("speed"), QObject::tr("Operation")}); table->horizontalHeader()->setStretchLastSection(true);
    layout->addWidget(table);
    for (const QVariant& v : node.parameters.value(kAxes, QVariantList{}).toList()) addAxisRow(table, v.toMap());
    // 中文翻译：添加轴
    auto* add = new QPushButton(QObject::tr("Add axis"), page); QObject::connect(add, &QPushButton::clicked, table, [table]{ addAxisRow(table); }); layout->addWidget(add);
    return page;
}

bool MultiAxisMoveStep::applyParameterEditor(QWidget* editor, ProcessNode& node, QString*) const
{
    node.parameters.insert(kMode, editor->findChild<QComboBox*>(kModeBox)->currentData().toString());
    node.parameters.insert(kTimeout, editor->findChild<QSpinBox*>(kTimeoutSpin)->value());
    node.parameters.insert(kAxes, rowsFromTable(editor->findChild<QTableWidget*>(kTable)));
    return true;
}

bool MultiAxisMoveStep::execute(const ProcessNodeExecutionRequest& request, ProcessStepContext& context, QString* errorMessage)
{
    // 中文翻译：运动服务不可用
    if (!context.motion) { if (errorMessage) *errorMessage = QObject::tr("Motion service unavailable"); return false; }
    const QVariantList rows = request.parameters.value(kAxes, QVariantList{}).toList();
    // 中文翻译：多轴运动轴表为空
    if (rows.isEmpty()) { if (errorMessage) *errorMessage = QObject::tr("Multi-axis motion axis table is empty"); return false; }
    return context.motion->moveAxes(rows, request.parameters.value(kMode, "sequential").toString(), request.parameters.value(kTimeout, 30000).toInt(), errorMessage);
}

int MultiAxisMoveStep::completionDelayMs(const ProcessNodeExecutionRequest&) const { return 200; }

} // namespace lcnc::process
