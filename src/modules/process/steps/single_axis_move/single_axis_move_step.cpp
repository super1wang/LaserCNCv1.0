#include "modules/process/steps/single_axis_move/single_axis_move_step.h"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLineEdit>
#include <QObject>
#include <QSpinBox>
#include <QWidget>

namespace lcnc::process {
namespace {

constexpr char kAxis[] = "axis";
constexpr char kMode[] = "mode";
constexpr char kTarget[] = "target";
constexpr char kVelocity[] = "velocity";
constexpr char kTimeoutMs[] = "timeoutMs";

} // namespace

ProcessNodeDescriptor SingleAxisMoveStep::descriptor() const
{
    ProcessNodeDescriptor d;
    d.type = ProcessNodeType::SingleAxisMove;
    // 中文翻译：单轴运动
    d.displayName = QObject::tr("Single axis motion");
    // 中文翻译：运动
    d.category = QObject::tr("sports");
    d.executorKey = QStringLiteral("singleAxisMove");
    d.defaultParameters.insert(QString::fromLatin1(kAxis), QStringLiteral("X"));
    d.defaultParameters.insert(QString::fromLatin1(kMode), QStringLiteral("absolute"));
    d.defaultParameters.insert(QString::fromLatin1(kTarget), 0.0);
    d.defaultParameters.insert(QString::fromLatin1(kVelocity), 5.0);
    d.defaultParameters.insert(QString::fromLatin1(kTimeoutMs), 30000);
    return d;
}

QString SingleAxisMoveStep::summary(const ProcessNode& node) const
{
    const auto& p = node.parameters;
    return QObject::tr("%1 %2 %3 F%4")
        .arg(p.value(QString::fromLatin1(kAxis), QStringLiteral("X")).toString(),
             p.value(QString::fromLatin1(kMode), QStringLiteral("absolute")).toString(),
             p.value(QString::fromLatin1(kTarget), 0.0).toString(),
             p.value(QString::fromLatin1(kVelocity), 5.0).toString());
}

QWidget* SingleAxisMoveStep::createParameterEditor(const ProcessNode& node, QWidget* parent) const
{
    auto* page = new QWidget(parent);
    auto* form = new QFormLayout(page);

    auto* axisEdit = new QLineEdit(node.parameters.value(QString::fromLatin1(kAxis), QStringLiteral("X")).toString(), page);
    axisEdit->setObjectName(QString::fromLatin1(kAxis));
    // 中文翻译：轴
    form->addRow(QObject::tr("axis"), axisEdit);

    auto* modeCombo = new QComboBox(page);
    modeCombo->setObjectName(QString::fromLatin1(kMode));
    // 中文翻译：绝对
    modeCombo->addItem(QObject::tr("Absolutely"), QStringLiteral("absolute"));
    // 中文翻译：相对
    modeCombo->addItem(QObject::tr("relatively"), QStringLiteral("relative"));
    const int modeIndex = modeCombo->findData(node.parameters.value(QString::fromLatin1(kMode), QStringLiteral("absolute")).toString());
    modeCombo->setCurrentIndex(modeIndex < 0 ? 0 : modeIndex);
    // 中文翻译：模式
    form->addRow(QObject::tr("mode"), modeCombo);

    auto* targetSpin = new QDoubleSpinBox(page);
    targetSpin->setObjectName(QString::fromLatin1(kTarget));
    targetSpin->setRange(-1000000.0, 1000000.0);
    targetSpin->setDecimals(3);
    targetSpin->setValue(node.parameters.value(QString::fromLatin1(kTarget), 0.0).toDouble());
    // 中文翻译：目标位置
    form->addRow(QObject::tr("Target location"), targetSpin);

    auto* velocitySpin = new QDoubleSpinBox(page);
    velocitySpin->setObjectName(QString::fromLatin1(kVelocity));
    velocitySpin->setRange(0.0, 1000000.0);
    velocitySpin->setDecimals(3);
    velocitySpin->setValue(node.parameters.value(QString::fromLatin1(kVelocity), 5.0).toDouble());
    // 中文翻译：速度
    form->addRow(QObject::tr("speed"), velocitySpin);

    auto* timeoutSpin = new QSpinBox(page);
    timeoutSpin->setObjectName(QString::fromLatin1(kTimeoutMs));
    timeoutSpin->setRange(0, 24 * 60 * 60 * 1000);
    timeoutSpin->setSuffix(QStringLiteral(" ms"));
    timeoutSpin->setValue(node.parameters.value(QString::fromLatin1(kTimeoutMs), 30000).toInt());
    // 中文翻译：超时
    form->addRow(QObject::tr("timeout"), timeoutSpin);
    return page;
}

bool SingleAxisMoveStep::applyParameterEditor(QWidget* editor, ProcessNode& node, QString*) const
{
    node.parameters.insert(QString::fromLatin1(kAxis),
                           editor->findChild<QLineEdit*>(QString::fromLatin1(kAxis))->text().trimmed().toUpper());
    node.parameters.insert(QString::fromLatin1(kMode),
                           editor->findChild<QComboBox*>(QString::fromLatin1(kMode))->currentData().toString());
    node.parameters.insert(QString::fromLatin1(kTarget),
                           editor->findChild<QDoubleSpinBox*>(QString::fromLatin1(kTarget))->value());
    node.parameters.insert(QString::fromLatin1(kVelocity),
                           editor->findChild<QDoubleSpinBox*>(QString::fromLatin1(kVelocity))->value());
    node.parameters.insert(QString::fromLatin1(kTimeoutMs),
                           editor->findChild<QSpinBox*>(QString::fromLatin1(kTimeoutMs))->value());
    return true;
}

bool SingleAxisMoveStep::execute(const ProcessNodeExecutionRequest& request,
                                 ProcessStepContext& context,
                                 QString* errorMessage)
{
    if (!context.motion) {
        if (errorMessage)
            // 中文翻译：运动服务不可用
            *errorMessage = QObject::tr("Motion service unavailable");
        return false;
    }
    const auto& p = request.parameters;
    return context.motion->moveAxis(p.value(QString::fromLatin1(kAxis), QStringLiteral("X")).toString(),
                                    p.value(QString::fromLatin1(kMode), QStringLiteral("absolute")).toString(),
                                    p.value(QString::fromLatin1(kTarget), 0.0).toDouble(),
                                    p.value(QString::fromLatin1(kVelocity), 5.0).toDouble(),
                                    p.value(QString::fromLatin1(kTimeoutMs), 30000).toInt(),
                                    errorMessage);
}

int SingleAxisMoveStep::completionDelayMs(const ProcessNodeExecutionRequest&) const
{
    return 200;
}

} // namespace lcnc::process
