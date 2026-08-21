#include "modules/process/steps/output_signal/output_signal_step.h"

#include "modules/process/settings/process_settings_service.h"
#include "modules/process/steps/process_step_registry.h"

#include <QComboBox>
#include <QFormLayout>
#include <QLineEdit>
#include <QObject>
#include <QWidget>

namespace lcnc::process {
namespace {

constexpr char kSignalType[] = "signalType";
constexpr char kIoName[] = "ioName";
constexpr char kValue[] = "value";
constexpr char kTypeCombo[] = "signalTypeBox";
constexpr char kIoCombo[] = "ioNameBox";
constexpr char kValueEdit[] = "valueEdit";

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
                                                   ? ProcessIoBucket::AnalogOutput
                                                   : ProcessIoBucket::DigitalOutput));
    }
}

} // namespace

ProcessNodeDescriptor OutputSignalStep::descriptor() const {
    ProcessNodeDescriptor descriptor;
    descriptor.type = ProcessNodeType::OutputSignal;
    // 中文翻译：输出信号
    descriptor.displayName = QObject::tr("Output signal");
    descriptor.category = QObject::tr("IO");
    descriptor.executorKey = QStringLiteral("outputSignal");
    descriptor.defaultParameters.insert(kSignalType, QStringLiteral("digital"));
    descriptor.defaultParameters.insert(kIoName, QStringLiteral("aLaser"));
    descriptor.defaultParameters.insert(kValue, true);
    return descriptor;
}

QString OutputSignalStep::summary(const ProcessNode& node) const {
    return QObject::tr("%1 %2=%3")
        .arg(node.parameters.value(kSignalType, "digital").toString(),
             node.parameters.value(kIoName, "aLaser").toString(),
             node.parameters.value(kValue, true).toString());
}

QWidget* OutputSignalStep::createParameterEditor(const ProcessNode& node, QWidget* parent) const {
    auto* page = new QWidget(parent);
    auto* form = new QFormLayout(page);
    auto* type = new QComboBox(page);
    type->setObjectName(kTypeCombo);
    type->addItems({QStringLiteral("digital"), QStringLiteral("analog")});
    type->setCurrentText(node.parameters.value(kSignalType, "digital").toString());
    // 中文翻译：输出类型
    form->addRow(QObject::tr("Output type"), type);

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

    auto* value = new QLineEdit(node.parameters.value(kValue, true).toString(), page);
    value->setObjectName(kValueEdit);
    // 中文翻译：值
    form->addRow(QObject::tr("value"), value);
    return page;
}

bool OutputSignalStep::applyParameterEditor(QWidget* editor, ProcessNode& node,
                                            QString* errorMessage) const {
    auto* type = editor ? editor->findChild<QComboBox*>(kTypeCombo) : nullptr;
    auto* io = editor ? editor->findChild<QComboBox*>(kIoCombo) : nullptr;
    auto* value = editor ? editor->findChild<QLineEdit*>(kValueEdit) : nullptr;
    if (!type || !io || !value) {
        if (errorMessage)
            *errorMessage = QObject::tr("The output signal editor is incomplete.");
        return false;
    }
    node.parameters.insert(kSignalType, type->currentText());
    node.parameters.insert(kIoName, ioKeyFromDisplay(io->currentText()));
    node.parameters.insert(kValue, value->text().trimmed());
    return true;
}

bool OutputSignalStep::execute(const ProcessNodeExecutionRequest& request,
                               ProcessStepContext& context, QString* errorMessage) {
    if (!context.io) {
        if (errorMessage)
            *errorMessage = QObject::tr("IO service is not available");
        return false;
    }
    return context.io->setOutput(request.parameters.value(kSignalType, "digital").toString(),
                                 request.parameters.value(kIoName, "aLaser").toString(),
                                 request.parameters.value(kValue, true), errorMessage);
}

} // namespace lcnc::process
