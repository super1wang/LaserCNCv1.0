#include "modules/process/steps/input_signal_wait/input_signal_wait_step.h"

#include "modules/process/settings/process_settings_service.h"
#include "modules/process/steps/process_step_registry.h"

#include <QCheckBox>
#include <QComboBox>
#include <QFormLayout>
#include <QObject>
#include <QSpinBox>
#include <QWidget>
#include <algorithm>

namespace lcnc::process {
namespace {

constexpr char kSignalType[] = "signalType";
constexpr char kIoName[] = "ioName";
constexpr char kTarget[] = "targetValue";
constexpr char kTimeout[] = "timeoutMs";
constexpr char kPoll[] = "pollIntervalMs";
constexpr char kTypeCombo[] = "signalTypeBox";
constexpr char kIoCombo[] = "ioNameBox";
constexpr char kTargetCheck[] = "targetCheck";
constexpr char kTimeoutSpin[] = "timeoutSpin";
constexpr char kPollSpin[] = "pollSpin";

QString ioKeyFromDisplay(const QString& display) {
    const int left = display.lastIndexOf('(');
    const int right = display.lastIndexOf(')');
    return left >= 0 && right > left ? display.mid(left + 1, right - left - 1) : display;
}

void refill(QComboBox* box, const QString& type) {
    if (!box)
        return;
    box->clear();
    if (const auto* settings = ProcessStepRegistry::instance().settingsService()) {
        box->addItems(settings->ioDisplayNames(type == QStringLiteral("analog")
                                                   ? ProcessIoBucket::AnalogInput
                                                   : ProcessIoBucket::DigitalInput));
    }
}

} // namespace

ProcessNodeDescriptor InputSignalWaitStep::descriptor() const {
    ProcessNodeDescriptor descriptor;
    descriptor.type = ProcessNodeType::InputSignalWait;
    // 中文翻译：输入信号
    descriptor.displayName = QObject::tr("input signal");
    descriptor.category = QObject::tr("IO");
    descriptor.executorKey = QStringLiteral("inputSignalWait");
    descriptor.defaultParameters.insert(kSignalType, QStringLiteral("digital"));
    descriptor.defaultParameters.insert(kIoName, QStringLiteral("aStart"));
    descriptor.defaultParameters.insert(kTarget, true);
    descriptor.defaultParameters.insert(kTimeout, 5000);
    descriptor.defaultParameters.insert(kPoll, 100);
    return descriptor;
}

QString InputSignalWaitStep::summary(const ProcessNode& node) const {
    // 中文翻译：等待 %1=%2 timeout=%3ms
    return QObject::tr("Wait %1=%2 timeout=%3ms")
        .arg(node.parameters.value(kIoName, "aStart").toString(),
             node.parameters.value(kTarget, true).toString(),
             node.parameters.value(kTimeout, 5000).toString());
}

QWidget* InputSignalWaitStep::createParameterEditor(const ProcessNode& node,
                                                    QWidget* parent) const {
    auto* page = new QWidget(parent);
    auto* form = new QFormLayout(page);
    auto* type = new QComboBox(page);
    type->setObjectName(kTypeCombo);
    type->addItems({QStringLiteral("digital"), QStringLiteral("analog")});
    type->setCurrentText(node.parameters.value(kSignalType, "digital").toString());
    // 中文翻译：输入类型
    form->addRow(QObject::tr("input type"), type);

    auto* io = new QComboBox(page);
    io->setObjectName(kIoCombo);
    refill(io, type->currentText());
    const QString current = node.parameters.value(kIoName).toString();
    for (int index = 0; index < io->count(); ++index) {
        if (ioKeyFromDisplay(io->itemText(index)) == current) {
            io->setCurrentIndex(index);
            break;
        }
    }
    QObject::connect(type, &QComboBox::currentTextChanged, io,
                     [io](const QString& value) { refill(io, value); });
    // 中文翻译：IO 名
    form->addRow(QObject::tr("IO name"), io);

    auto* target = new QCheckBox(QObject::tr("target is true"), page);
    target->setObjectName(kTargetCheck);
    target->setChecked(node.parameters.value(kTarget, true).toBool());
    form->addRow(QString{}, target);

    auto* timeout = new QSpinBox(page);
    timeout->setObjectName(kTimeoutSpin);
    timeout->setRange(0, 24 * 60 * 60 * 1000);
    timeout->setSuffix(QStringLiteral(" ms"));
    timeout->setValue(node.parameters.value(kTimeout, 5000).toInt());
    // 中文翻译：超时
    form->addRow(QObject::tr("timeout"), timeout);

    auto* poll = new QSpinBox(page);
    poll->setObjectName(kPollSpin);
    poll->setRange(10, 60000);
    poll->setSuffix(QStringLiteral(" ms"));
    poll->setValue(node.parameters.value(kPoll, 100).toInt());
    // 中文翻译：刷新间隔
    form->addRow(QObject::tr("refresh interval"), poll);
    return page;
}

bool InputSignalWaitStep::applyParameterEditor(QWidget* editor, ProcessNode& node,
                                               QString* errorMessage) const {
    auto* type = editor ? editor->findChild<QComboBox*>(kTypeCombo) : nullptr;
    auto* io = editor ? editor->findChild<QComboBox*>(kIoCombo) : nullptr;
    auto* target = editor ? editor->findChild<QCheckBox*>(kTargetCheck) : nullptr;
    auto* timeout = editor ? editor->findChild<QSpinBox*>(kTimeoutSpin) : nullptr;
    auto* poll = editor ? editor->findChild<QSpinBox*>(kPollSpin) : nullptr;
    if (!type || !io || !target || !timeout || !poll) {
        if (errorMessage)
            *errorMessage = QObject::tr("The input signal editor is incomplete.");
        return false;
    }
    node.parameters.insert(kSignalType, type->currentText());
    node.parameters.insert(kIoName, ioKeyFromDisplay(io->currentText()));
    node.parameters.insert(kTarget, target->isChecked());
    node.parameters.insert(kTimeout, timeout->value());
    node.parameters.insert(kPoll, poll->value());
    return true;
}

bool InputSignalWaitStep::execute(const ProcessNodeExecutionRequest& request,
                                  ProcessStepContext& context, QString* errorMessage) {
    if (!context.io) {
        if (errorMessage)
            *errorMessage = QObject::tr("IO service is not available");
        return false;
    }
    return context.io->waitInput(request.parameters.value(kSignalType, "digital").toString(),
                                 request.parameters.value(kIoName, "aStart").toString(),
                                 request.parameters.value(kTarget, true),
                                 request.parameters.value(kTimeout, 5000).toInt(),
                                 request.parameters.value(kPoll, 100).toInt(), errorMessage);
}

int InputSignalWaitStep::completionDelayMs(const ProcessNodeExecutionRequest& request) const {
    return std::clamp(request.parameters.value(kPoll, 100).toInt(), 10, 60000);
}

} // namespace lcnc::process
