#include "modules/process/steps/start/start_step.h"

#include <QObject>
#include <QPlainTextEdit>
#include <QVBoxLayout>
#include <QWidget>

namespace lcnc::process {

namespace {
constexpr char kVariablesEditorProperty[] = "lcnc.variables.editor";
}

ProcessNodeDescriptor StartStep::descriptor() const
{
    ProcessNodeDescriptor d;
    d.type = ProcessNodeType::Start;
    // 中文翻译：开始
    d.displayName = QObject::tr("start");
    // 中文翻译：结构
    d.category = QObject::tr("structure");
    d.topLevelOnly = true;
    d.required = true;
    d.addable = false;
    d.deletable = false;
    d.disableable = false;
    d.movable = false;
    d.executorKey = QStringLiteral("start");
    d.defaultParameters.insert(QStringLiteral("variables"), QStringLiteral("[]"));
    return d;
}

QString StartStep::summary(const ProcessNode& node) const
{
    const QString variables = node.parameters.value(QStringLiteral("variables"), QStringLiteral("[]")).toString();
    // 中文翻译：入口 / 变量声明 %1 字符
    return QObject::tr("Entry/variable declaration %1 characters").arg(variables.size());
}

QWidget* StartStep::createParameterEditor(const ProcessNode& node, QWidget* parent) const
{
    auto* page = new QWidget(parent);
    auto* layout = new QVBoxLayout(page);
    auto* editor = new QPlainTextEdit(page);
    editor->setObjectName(QString::fromLatin1(kVariablesEditorProperty));
    // 中文翻译：JSON 数组，例如：\n[\n  {"name":"speed","type":"number","default":5}\n]
    editor->setPlaceholderText(QObject::tr("JSON array, for example:\n[\n  {\"name\":\"speed\",\"type\":\"number\",\"default\":5}\n]"));
    editor->setPlainText(node.parameters.value(QStringLiteral("variables"), QStringLiteral("[]")).toString());
    layout->addWidget(editor);
    return page;
}

bool StartStep::applyParameterEditor(QWidget* editor, ProcessNode& node, QString* errorMessage) const
{
    Q_UNUSED(errorMessage);
    auto* text = editor ? editor->findChild<QPlainTextEdit*>(QString::fromLatin1(kVariablesEditorProperty)) : nullptr;
    node.parameters.insert(QStringLiteral("variables"), text ? text->toPlainText().trimmed() : QStringLiteral("[]"));
    node.enabled = true;
    return true;
}

bool StartStep::execute(const ProcessNodeExecutionRequest& request,
                        ProcessStepContext& context,
                        QString* errorMessage)
{
    Q_UNUSED(request);
    Q_UNUSED(errorMessage);
    if (context.logMessage)
        // 中文翻译：流程开始
        context.logMessage(QObject::tr("Process starts"));
    return true;
}

} // namespace lcnc::process
