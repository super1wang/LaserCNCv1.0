#include "modules/process/steps/stop/stop_step.h"

#include <QCheckBox>
#include <QFormLayout>
#include <QLineEdit>
#include <QObject>
#include <QWidget>

namespace lcnc::process {

namespace {
constexpr char kMessageName[] = "lcnc.stop.message";
constexpr char kSafeOutputsName[] = "lcnc.stop.safeOutputs";
constexpr char kStopMotionName[] = "lcnc.stop.stopMotion";
}

ProcessNodeDescriptor StopStep::descriptor() const
{
    ProcessNodeDescriptor d;
    d.type = ProcessNodeType::Stop;
    // 中文翻译：停止
    d.displayName = QObject::tr("stop");
    // 中文翻译：结构
    d.category = QObject::tr("structure");
    d.topLevelOnly = true;
    d.required = true;
    d.addable = false;
    d.deletable = false;
    d.disableable = false;
    d.movable = false;
    d.executorKey = QStringLiteral("stop");
    // 中文翻译：流程结束
    d.defaultParameters.insert(QStringLiteral("message"), QObject::tr("End of process"));
    d.defaultParameters.insert(QStringLiteral("safeStopOutputs"), true);
    d.defaultParameters.insert(QStringLiteral("stopMotion"), false);
    return d;
}

QString StopStep::summary(const ProcessNode& node) const
{
    // 中文翻译：流程结束
    return node.parameters.value(QStringLiteral("message"), QObject::tr("End of process")).toString();
}

QWidget* StopStep::createParameterEditor(const ProcessNode& node, QWidget* parent) const
{
    auto* page = new QWidget(parent);
    auto* form = new QFormLayout(page);

    auto* message = new QLineEdit(page);
    message->setObjectName(QString::fromLatin1(kMessageName));
    // 中文翻译：流程结束
    message->setText(node.parameters.value(QStringLiteral("message"), QObject::tr("End of process")).toString());
    // 中文翻译：停止消息
    form->addRow(QObject::tr("stop message"), message);

    // 中文翻译：停止时复位安全输出
    auto* safeOutputs = new QCheckBox(QObject::tr("Reset safety outputs when stopped"), page);
    safeOutputs->setObjectName(QString::fromLatin1(kSafeOutputsName));
    safeOutputs->setChecked(node.parameters.value(QStringLiteral("safeStopOutputs"), true).toBool());
    form->addRow(QString(), safeOutputs);

    // 中文翻译：停止时停止运动
    auto* stopMotion = new QCheckBox(QObject::tr("Stop movement when stopped"), page);
    stopMotion->setObjectName(QString::fromLatin1(kStopMotionName));
    stopMotion->setChecked(node.parameters.value(QStringLiteral("stopMotion"), false).toBool());
    form->addRow(QString(), stopMotion);
    return page;
}

bool StopStep::applyParameterEditor(QWidget* editor, ProcessNode& node, QString* errorMessage) const
{
    Q_UNUSED(errorMessage);
    auto* message = editor ? editor->findChild<QLineEdit*>(QString::fromLatin1(kMessageName)) : nullptr;
    auto* safeOutputs = editor ? editor->findChild<QCheckBox*>(QString::fromLatin1(kSafeOutputsName)) : nullptr;
    auto* stopMotion = editor ? editor->findChild<QCheckBox*>(QString::fromLatin1(kStopMotionName)) : nullptr;
    // 中文翻译：流程结束
    node.parameters.insert(QStringLiteral("message"), message ? message->text().trimmed() : QObject::tr("End of process"));
    node.parameters.insert(QStringLiteral("safeStopOutputs"), safeOutputs ? safeOutputs->isChecked() : true);
    node.parameters.insert(QStringLiteral("stopMotion"), stopMotion ? stopMotion->isChecked() : false);
    node.enabled = true;
    return true;
}

bool StopStep::execute(const ProcessNodeExecutionRequest& request,
                       ProcessStepContext& context,
                       QString* errorMessage)
{
    // 中文翻译：流程结束
    const QString message = request.parameters.value(QStringLiteral("message"), QObject::tr("End of process")).toString();
    if (context.logMessage)
        context.logMessage(message);
    if (request.parameters.value(QStringLiteral("stopMotion"), false).toBool() && context.motion) {
        if (!context.motion->stopMotion(errorMessage))
            return false;
    }
    // safeStopOutputs 由 ProcessModule::workflowFinished 当前统一处理；后续迁入 IO service。
    return true;
}

} // namespace lcnc::process
