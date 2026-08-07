#include "modules/process/steps/loop/loop_step.h"

#include <QFormLayout>
#include <QObject>
#include <QSpinBox>
#include <QWidget>

namespace lcnc::process {
namespace {
constexpr char kLoopCount[] = "loopCount";
constexpr char kCountSpin[] = "countSpin";
} // namespace

ProcessNodeDescriptor LoopStep::descriptor() const
{
    ProcessNodeDescriptor d;
    d.type = ProcessNodeType::Loop;
    d.displayName = QObject::tr("Loop");
    d.category = QObject::tr("structure");
    d.canHaveChildren = true;
    d.executorKey = QStringLiteral("loop");
    d.defaultParameters.insert(QString::fromLatin1(kLoopCount), 1);
    return d;
}

QString LoopStep::summary(const ProcessNode& node) const
{
    return QObject::tr("Loop x%1").arg(node.parameters.value(kLoopCount, 1).toInt());
}

QWidget* LoopStep::createParameterEditor(const ProcessNode& node, QWidget* parent) const
{
    auto* page = new QWidget(parent);
    auto* form = new QFormLayout(page);
    auto* spin = new QSpinBox(page);
    spin->setObjectName(QString::fromLatin1(kCountSpin));
    spin->setRange(1, 99999);
    spin->setValue(node.parameters.value(kLoopCount, 1).toInt());
    form->addRow(QObject::tr("loop count"), spin);
    return page;
}

bool LoopStep::applyParameterEditor(QWidget* editor, ProcessNode& node, QString*) const
{
    if (auto* spin = editor ? editor->findChild<QSpinBox*>(QString::fromLatin1(kCountSpin)) : nullptr)
        node.parameters.insert(QString::fromLatin1(kLoopCount), spin->value());
    return true;
}

bool LoopStep::execute(const ProcessNodeExecutionRequest& request,
                       ProcessStepContext& context,
                       QString*)
{
    if (context.logMessage)
        // 中文翻译：循环开始：%1 次
        context.logMessage(QObject::tr("Loop start: %1 iterations")
                               .arg(request.parameters.value(kLoopCount, 1).toInt()));
    return true;
}

} // namespace lcnc::process
