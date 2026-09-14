#include "modules/process/steps/set_axis_position/set_axis_position_step.h"

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
constexpr char kAxes[] = "axes";
constexpr char kTimeout[] = "timeoutMs";
constexpr char kTable[] = "axesTable";
constexpr char kTimeoutSpin[] = "timeoutSpin";

void addAxisRow(QTableWidget* table, const QVariantMap& row = {})
{
    const int r = table->rowCount();
    table->insertRow(r);
    auto* axis = new QLineEdit(row.value(QStringLiteral("axis"), QStringLiteral("X")).toString(), table);
    table->setCellWidget(r, 0, axis);
    auto* position = new QDoubleSpinBox(table);
    position->setRange(-1000000.0, 1000000.0);
    position->setDecimals(3);
    position->setValue(row.value(QStringLiteral("position"), 0.0).toDouble());
    table->setCellWidget(r, 1, position);
    // 中文翻译：删除
    auto* del = new QPushButton(QObject::tr("Delete"), table);
    QObject::connect(del, &QPushButton::clicked, table, [table, del] {
        for (int i = 0; i < table->rowCount(); ++i) {
            if (table->cellWidget(i, 2) == del) { table->removeRow(i); break; }
        }
    });
    table->setCellWidget(r, 2, del);
}

QVariantList rowsFromTable(QTableWidget* table)
{
    QVariantList rows;
    for (int r = 0; r < table->rowCount(); ++r) {
        QVariantMap row;
        if (auto* axis = qobject_cast<QLineEdit*>(table->cellWidget(r, 0)))
            row.insert(QStringLiteral("axis"), axis->text().trimmed().toUpper());
        if (auto* position = qobject_cast<QDoubleSpinBox*>(table->cellWidget(r, 1)))
            row.insert(QStringLiteral("position"), position->value());
        if (!row.value(QStringLiteral("axis")).toString().isEmpty())
            rows.append(row);
    }
    return rows;
}
} // namespace

ProcessNodeDescriptor SetAxisPositionStep::descriptor() const
{
    ProcessNodeDescriptor d;
    d.type = ProcessNodeType::SetAxisPosition;
    // 中文翻译：轴系置位
    d.displayName = QObject::tr("Axis position setting");
    // 中文翻译：运动
    d.category = QObject::tr("sports");
    d.executorKey = QStringLiteral("setAxisPosition");
    d.defaultParameters.insert(QString::fromLatin1(kAxes), QVariantList{});
    d.defaultParameters.insert(QString::fromLatin1(kTimeout), 5000);
    return d;
}

QString SetAxisPositionStep::summary(const ProcessNode& node) const
{
    const QVariantList rows = node.parameters.value(QString::fromLatin1(kAxes), QVariantList{}).toList();
    if (rows.isEmpty())
        // 中文翻译：置位（空）
        return QObject::tr("Set position (empty)");
    QStringList parts;
    for (const QVariant& v : rows) {
        const QVariantMap row = v.toMap();
        parts << QStringLiteral("%1=%2").arg(
            row.value(QStringLiteral("axis")).toString(),
            QString::number(row.value(QStringLiteral("position"), 0.0).toDouble(), 'f', 3));
    }
    // 中文翻译：置位 %1
    return QObject::tr("Set position: %1").arg(parts.join(QStringLiteral(", ")));
}

QWidget* SetAxisPositionStep::createParameterEditor(const ProcessNode& node, QWidget* parent) const
{
    auto* page = new QWidget(parent);
    auto* layout = new QVBoxLayout(page);
    auto* form = new QFormLayout();
    auto* timeout = new QSpinBox(page);
    timeout->setObjectName(QString::fromLatin1(kTimeoutSpin));
    timeout->setRange(0, 24 * 60 * 60 * 1000);
    timeout->setSuffix(QStringLiteral(" ms"));
    timeout->setValue(node.parameters.value(QString::fromLatin1(kTimeout), 5000).toInt());
    // 中文翻译：超时
    form->addRow(QObject::tr("timeout"), timeout);
    layout->addLayout(form);

    auto* table = new QTableWidget(page);
    table->setObjectName(QString::fromLatin1(kTable));
    table->setColumnCount(3);
    // 中文翻译：轴；置位坐标；操作
    table->setHorizontalHeaderLabels({QObject::tr("axis"), QObject::tr("Position coordinates"), QObject::tr("Operation")});
    table->horizontalHeader()->setStretchLastSection(true);
    layout->addWidget(table);
    for (const QVariant& v : node.parameters.value(QString::fromLatin1(kAxes), QVariantList{}).toList())
        addAxisRow(table, v.toMap());
    // 中文翻译：添加轴
    auto* add = new QPushButton(QObject::tr("Add axis"), page);
    QObject::connect(add, &QPushButton::clicked, table, [table] { addAxisRow(table); });
    layout->addWidget(add);
    return page;
}

bool SetAxisPositionStep::applyParameterEditor(QWidget* editor, ProcessNode& node, QString*) const
{
    node.parameters.insert(QString::fromLatin1(kTimeout),
                           editor->findChild<QSpinBox*>(QString::fromLatin1(kTimeoutSpin))->value());
    node.parameters.insert(QString::fromLatin1(kAxes),
                           rowsFromTable(editor->findChild<QTableWidget*>(QString::fromLatin1(kTable))));
    return true;
}

bool SetAxisPositionStep::execute(const ProcessNodeExecutionRequest& request,
                                  ProcessStepContext& context,
                                  QString* errorMessage)
{
    // 中文翻译：运动服务不可用
    if (!context.motion) { if (errorMessage) *errorMessage = QObject::tr("Motion service unavailable"); return false; }
    const QVariantList rows = request.parameters.value(QString::fromLatin1(kAxes), QVariantList{}).toList();
    // 中文翻译：置位轴表为空
    if (rows.isEmpty()) { if (errorMessage) *errorMessage = QObject::tr("Axis position table is empty"); return false; }
    return context.motion->setAxisPosition(rows,
                                           request.parameters.value(QString::fromLatin1(kTimeout), 5000).toInt(),
                                           errorMessage);
}

int SetAxisPositionStep::completionDelayMs(const ProcessNodeExecutionRequest&) const { return 200; }

} // namespace lcnc::process
