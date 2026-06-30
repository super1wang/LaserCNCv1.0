#include "modules/process/steps/normal_cutting/normal_cutting_step.h"

#include <QFormLayout>
#include <QLineEdit>
#include <QObject>
#include <QSpinBox>
#include <QWidget>

namespace lcnc::process {

namespace {

constexpr char kSel[] = "selectionMode";
constexpr char kStart[] = "startNumber";
constexpr char kEnd[] = "endNumber";
constexpr char kComp[] = "compensationIndex";

} // namespace

ProcessNodeDescriptor NormalCuttingStep::descriptor() const
{
    ProcessNodeDescriptor d;
    d.type = ProcessNodeType::NormalCutting;
    d.displayName = QObject::tr("普通切割");
    d.category = QObject::tr("加工");
    d.executorKey = QStringLiteral("normalCutting");
    d.defaultParameters.insert(QString::fromLatin1(kSel), QStringLiteral("allEnabled"));
    d.defaultParameters.insert(QString::fromLatin1(kStart), 1);
    d.defaultParameters.insert(QString::fromLatin1(kEnd), 0);
    d.defaultParameters.insert(QString::fromLatin1(kComp), QString());
    return d;
}

QString NormalCuttingStep::summary(const ProcessNode& node) const
{
    Q_UNUSED(node);
    return QObject::tr("普通切割");
}

QWidget* NormalCuttingStep::createParameterEditor(const ProcessNode& node, QWidget* parent) const
{
    auto* page = new QWidget(parent);
    auto* form = new QFormLayout(page);

    auto* sel = new QLineEdit(node.parameters.value(kSel, "allEnabled").toString(), page);
    sel->setObjectName(kSel);
    form->addRow(QObject::tr("选择模式"), sel);

    auto* start = new QSpinBox(page);
    start->setObjectName(kStart);
    start->setRange(1, 1000000);
    start->setValue(node.parameters.value(kStart, 1).toInt());
    form->addRow(QObject::tr("起始序号"), start);

    auto* end = new QSpinBox(page);
    end->setObjectName(kEnd);
    end->setRange(0, 1000000);
    end->setValue(node.parameters.value(kEnd, 0).toInt());
    form->addRow(QObject::tr("结束序号(0=不限)"), end);

    auto* comp = new QLineEdit(node.parameters.value(kComp).toString(), page);
    comp->setObjectName(kComp);
    form->addRow(QObject::tr("补偿索引"), comp);
    return page;
}

bool NormalCuttingStep::applyParameterEditor(QWidget* editor, ProcessNode& node, QString*) const
{
    node.parameters.remove(QStringLiteral("dryRun"));
    node.parameters.insert(QString::fromLatin1(kSel), editor->findChild<QLineEdit*>(kSel)->text().trimmed());
    node.parameters.insert(QString::fromLatin1(kStart), editor->findChild<QSpinBox*>(kStart)->value());
    node.parameters.insert(QString::fromLatin1(kEnd), editor->findChild<QSpinBox*>(kEnd)->value());
    node.parameters.insert(QString::fromLatin1(kComp), editor->findChild<QLineEdit*>(kComp)->text().trimmed());
    return true;
}

bool NormalCuttingStep::execute(const ProcessNodeExecutionRequest& request,
                                ProcessStepContext& context,
                                QString* errorMessage)
{
    if (!context.cutting) {
        if (errorMessage)
            *errorMessage = QObject::tr("切割服务不可用");
        return false;
    }

    const ProcessToolpathSnapshot snapshot = context.cutting->toolpathSnapshot();
    if (!snapshot.available) {
        if (errorMessage)
            *errorMessage = QObject::tr("普通切割需要可用 CAM 刀路");
        return false;
    }

    if (!context.cutting->executeNormalCutting(
            request.nodeId,
            request.parameters,
            context.interrupt ? context.interrupt : context.cancellationToken,
            errorMessage)) {
        return false;
    }

    if (context.logMessage) {
        context.logMessage(QObject::tr("普通切割: %1，轮廓=%2，点数=%3")
                               .arg(snapshot.description,
                                    QString::number(snapshot.contourCount),
                                    QString::number(snapshot.totalPointCount)));
    }
    return true;
}

int NormalCuttingStep::completionDelayMs(const ProcessNodeExecutionRequest& request) const
{
    Q_UNUSED(request);
    return 0;
}

} // namespace lcnc::process
