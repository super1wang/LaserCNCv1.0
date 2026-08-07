#include "modules/process/steps/if/if_step.h"

#include "modules/process/settings/process_settings_service.h"
#include "modules/process/steps/process_step_registry.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QObject>
#include <QRegularExpression>
#include <QStackedWidget>
#include <QWidget>

#include <cmath>

namespace lcnc::process {
namespace {
constexpr char kMode[] = "conditionMode";
constexpr char kVarName[] = "variableName";
constexpr char kRegex[] = "regex";
constexpr char kSignalType[] = "signalType";
constexpr char kIoName[] = "ioName";
constexpr char kOperator[] = "operator";
constexpr char kTarget[] = "targetValue";

constexpr char kModeCombo[] = "modeCombo";
constexpr char kStack[] = "stack";
constexpr char kVarEdit[] = "varEdit";
constexpr char kRegexEdit[] = "regexEdit";
constexpr char kTypeCombo[] = "typeCombo";
constexpr char kIoCombo[] = "ioCombo";
constexpr char kOpCombo[] = "opCombo";
constexpr char kTargetCheck[] = "targetCheck";
constexpr char kTargetSpin[] = "targetSpin";

const QStringList kDigitalOps = { QStringLiteral("=="), QStringLiteral("!=") };
const QStringList kAnalogOps = { QStringLiteral("=="), QStringLiteral("!="),
                                 QStringLiteral(">"), QStringLiteral(">="),
                                 QStringLiteral("<"), QStringLiteral("<=") };

QString ioKeyFromDisplay(const QString& display)
{
    const int l = display.lastIndexOf('(');
    const int r = display.lastIndexOf(')');
    if (l >= 0 && r > l)
        return display.mid(l + 1, r - l - 1);
    return display;
}

void refillIo(QComboBox* box, const QString& type)
{
    box->clear();
    if (const auto* settings = ProcessStepRegistry::instance().settingsService())
        box->addItems(settings->ioDisplayNames(type == QStringLiteral("analog")
                                                   ? ProcessIoBucket::AnalogInput
                                                   : ProcessIoBucket::DigitalInput));
}

void selectByKey(QComboBox* box, const QString& key)
{
    for (int i = 0; i < box->count(); ++i) {
        if (ioKeyFromDisplay(box->itemText(i)) == key) {
            box->setCurrentIndex(i);
            return;
        }
    }
}

bool compareAnalog(double lhs, const QString& op, double rhs)
{
    if (op == QStringLiteral("!=")) return std::abs(lhs - rhs) >= 1e-9;
    if (op == QStringLiteral(">"))  return lhs > rhs;
    if (op == QStringLiteral(">=")) return lhs >= rhs;
    if (op == QStringLiteral("<"))  return lhs < rhs;
    if (op == QStringLiteral("<=")) return lhs <= rhs;
    return std::abs(lhs - rhs) < 1e-9; // "==" 默认
}
} // namespace

ProcessNodeDescriptor IfStep::descriptor() const
{
    ProcessNodeDescriptor d;
    d.type = ProcessNodeType::If;
    d.displayName = QObject::tr("If");
    d.category = QObject::tr("structure");
    d.canHaveChildren = true;
    d.executorKey = QStringLiteral("if");
    d.defaultParameters.insert(QString::fromLatin1(kMode), QStringLiteral("variable"));
    d.defaultParameters.insert(QString::fromLatin1(kVarName), QString());
    d.defaultParameters.insert(QString::fromLatin1(kRegex), QString());
    d.defaultParameters.insert(QString::fromLatin1(kSignalType), QStringLiteral("digital"));
    d.defaultParameters.insert(QString::fromLatin1(kIoName), QStringLiteral("aStart"));
    d.defaultParameters.insert(QString::fromLatin1(kOperator), QStringLiteral("=="));
    d.defaultParameters.insert(QString::fromLatin1(kTarget), true);
    return d;
}

QString IfStep::summary(const ProcessNode& node) const
{
    if (node.parameters.value(kMode, QStringLiteral("variable")).toString() == QStringLiteral("io")) {
        return QObject::tr("If %1 %2 %3").arg(
            node.parameters.value(kIoName).toString(),
            node.parameters.value(kOperator, QStringLiteral("==")).toString(),
            node.parameters.value(kTarget, true).toString());
    }
    return QObject::tr("If %1 ~= /%2/").arg(
        node.parameters.value(kVarName).toString(),
        node.parameters.value(kRegex).toString());
}

QWidget* IfStep::createParameterEditor(const ProcessNode& node, QWidget* parent) const
{
    auto* page = new QWidget(parent);
    auto* form = new QFormLayout(page);

    auto* modeCombo = new QComboBox(page);
    modeCombo->setObjectName(QString::fromLatin1(kModeCombo));
    modeCombo->addItem(QObject::tr("variable"), QStringLiteral("variable"));
    modeCombo->addItem(QObject::tr("IO"), QStringLiteral("io"));
    const QString mode = node.parameters.value(kMode, QStringLiteral("variable")).toString();
    modeCombo->setCurrentIndex(mode == QStringLiteral("io") ? 1 : 0);
    form->addRow(QObject::tr("condition type"), modeCombo);

    auto* stack = new QStackedWidget(page);
    stack->setObjectName(QString::fromLatin1(kStack));
    form->addRow(stack);

    // ── 变量条件页 ───────────────────────────────────────────────────────
    auto* varPage = new QWidget(stack);
    auto* varForm = new QFormLayout(varPage);
    auto* varEdit = new QLineEdit(varPage);
    varEdit->setObjectName(QString::fromLatin1(kVarEdit));
    varEdit->setText(node.parameters.value(kVarName).toString());
    varEdit->setPlaceholderText(QObject::tr("variable name declared in Start"));
    varForm->addRow(QObject::tr("variable name"), varEdit);
    auto* regexEdit = new QLineEdit(varPage);
    regexEdit->setObjectName(QString::fromLatin1(kRegexEdit));
    regexEdit->setText(node.parameters.value(kRegex).toString());
    regexEdit->setPlaceholderText(QObject::tr("regular expression"));
    varForm->addRow(QObject::tr("regex"), regexEdit);
    stack->addWidget(varPage);

    // ── IO 条件页 ────────────────────────────────────────────────────────
    auto* ioPage = new QWidget(stack);
    auto* ioForm = new QFormLayout(ioPage);
    auto* typeCombo = new QComboBox(ioPage);
    typeCombo->setObjectName(QString::fromLatin1(kTypeCombo));
    typeCombo->addItem(QObject::tr("digital"), QStringLiteral("digital"));
    typeCombo->addItem(QObject::tr("analog"), QStringLiteral("analog"));
    ioForm->addRow(QObject::tr("input type"), typeCombo);
    auto* ioCombo = new QComboBox(ioPage);
    ioCombo->setObjectName(QString::fromLatin1(kIoCombo));
    ioForm->addRow(QObject::tr("IO name"), ioCombo);
    auto* opCombo = new QComboBox(ioPage);
    opCombo->setObjectName(QString::fromLatin1(kOpCombo));
    ioForm->addRow(QObject::tr("operator"), opCombo);

    auto* targetContainer = new QWidget(ioPage);
    auto* targetLayout = new QHBoxLayout(targetContainer);
    targetLayout->setContentsMargins(0, 0, 0, 0);
    auto* targetCheck = new QCheckBox(QObject::tr("target is true"), targetContainer);
    targetCheck->setObjectName(QString::fromLatin1(kTargetCheck));
    auto* targetSpin = new QDoubleSpinBox(targetContainer);
    targetSpin->setObjectName(QString::fromLatin1(kTargetSpin));
    targetSpin->setRange(-1e9, 1e9);
    targetSpin->setDecimals(3);
    targetLayout->addWidget(targetCheck);
    targetLayout->addWidget(targetSpin);
    ioForm->addRow(QObject::tr("target value"), targetContainer);
    stack->addWidget(ioPage);

    // 根据 signalType 刷新 IO 名/运算符/目标值控件。
    const QString signalType = node.parameters.value(kSignalType, QStringLiteral("digital")).toString();
    const bool analog = signalType == QStringLiteral("analog");
    typeCombo->setCurrentIndex(analog ? 1 : 0);
    refillIo(ioCombo, signalType);
    selectByKey(ioCombo, node.parameters.value(kIoName).toString());
    opCombo->clear();
    opCombo->addItems(analog ? kAnalogOps : kDigitalOps);
    {
        const int opIndex = opCombo->findText(node.parameters.value(kOperator, QStringLiteral("==")).toString());
        opCombo->setCurrentIndex(opIndex < 0 ? 0 : opIndex);
    }
    targetCheck->setVisible(!analog);
    targetSpin->setVisible(analog);
    targetCheck->setChecked(node.parameters.value(kTarget, true).toBool());
    targetSpin->setValue(node.parameters.value(kTarget, 0.0).toDouble());

    QObject::connect(typeCombo, &QComboBox::currentTextChanged, ioPage,
        [ioCombo, opCombo, targetCheck, targetSpin](const QString& text) {
            const bool isAnalog = text == QStringLiteral("analog");
            refillIo(ioCombo, text);
            opCombo->clear();
            opCombo->addItems(isAnalog ? kAnalogOps : kDigitalOps);
            targetCheck->setVisible(!isAnalog);
            targetSpin->setVisible(isAnalog);
        });

    stack->setCurrentIndex(mode == QStringLiteral("io") ? 1 : 0);
    QObject::connect(modeCombo, &QComboBox::currentIndexChanged, stack,
        [stack](int index) { stack->setCurrentIndex(index); });

    return page;
}

bool IfStep::applyParameterEditor(QWidget* editor, ProcessNode& node, QString*) const
{
    auto* modeCombo = editor ? editor->findChild<QComboBox*>(QString::fromLatin1(kModeCombo)) : nullptr;
    const QString mode = modeCombo ? modeCombo->currentData().toString() : QStringLiteral("variable");
    node.parameters.insert(QString::fromLatin1(kMode), mode);

    if (mode == QStringLiteral("io")) {
        auto* typeCombo = editor->findChild<QComboBox*>(QString::fromLatin1(kTypeCombo));
        const QString signalType = typeCombo ? typeCombo->currentData().toString() : QStringLiteral("digital");
        node.parameters.insert(QString::fromLatin1(kSignalType), signalType);
        auto* ioCombo = editor->findChild<QComboBox*>(QString::fromLatin1(kIoCombo));
        node.parameters.insert(QString::fromLatin1(kIoName),
                               ioCombo ? ioKeyFromDisplay(ioCombo->currentText()) : QString());
        auto* opCombo = editor->findChild<QComboBox*>(QString::fromLatin1(kOpCombo));
        node.parameters.insert(QString::fromLatin1(kOperator),
                               opCombo ? opCombo->currentText() : QStringLiteral("=="));
        if (signalType == QStringLiteral("analog")) {
            auto* spin = editor->findChild<QDoubleSpinBox*>(QString::fromLatin1(kTargetSpin));
            node.parameters.insert(QString::fromLatin1(kTarget), spin ? spin->value() : 0.0);
        } else {
            auto* check = editor->findChild<QCheckBox*>(QString::fromLatin1(kTargetCheck));
            node.parameters.insert(QString::fromLatin1(kTarget), check ? check->isChecked() : true);
        }
    } else {
        auto* varEdit = editor->findChild<QLineEdit*>(QString::fromLatin1(kVarEdit));
        node.parameters.insert(QString::fromLatin1(kVarName), varEdit ? varEdit->text().trimmed() : QString());
        auto* regexEdit = editor->findChild<QLineEdit*>(QString::fromLatin1(kRegexEdit));
        node.parameters.insert(QString::fromLatin1(kRegex), regexEdit ? regexEdit->text() : QString());
    }
    return true;
}

bool IfStep::execute(const ProcessNodeExecutionRequest& request,
                     ProcessStepContext& context,
                     QString* errorMessage)
{
    const QString mode = request.parameters.value(kMode, QStringLiteral("variable")).toString();
    bool result = false;

    if (mode == QStringLiteral("io")) {
        if (!context.io) {
            if (errorMessage)
                // 中文翻译：IO 服务不可用
                *errorMessage = QObject::tr("IO service is not available");
            return false;
        }
        const QString signalType = request.parameters.value(kSignalType, QStringLiteral("digital")).toString();
        QVariant value;
        if (!context.io->readInput(signalType,
                                   request.parameters.value(kIoName).toString(),
                                   &value,
                                   errorMessage)) {
            return false;
        }
        const QString op = request.parameters.value(kOperator, QStringLiteral("==")).toString();
        if (signalType == QStringLiteral("analog")) {
            result = compareAnalog(value.toDouble(), op,
                                   request.parameters.value(kTarget, 0.0).toDouble());
        } else {
            result = compareAnalog(value.toBool() ? 1.0 : 0.0, op,
                                   request.parameters.value(kTarget, true).toBool() ? 1.0 : 0.0);
        }
    } else {
        const QString varName = request.parameters.value(kVarName).toString();
        const QString varValue = context.variables
            ? context.variables->value(varName).toString()
            : QString();
        const QString pattern = request.parameters.value(kRegex).toString();
        const QRegularExpression re(pattern);
        if (!re.isValid()) {
            if (context.logMessage)
                // 中文翻译：If：正则表达式无效 %1
                context.logMessage(QObject::tr("If: invalid regex '%1'").arg(pattern));
            result = false;
        } else {
            result = re.match(varValue).hasMatch();
        }
    }

    if (context.ifResults)
        (*context.ifResults)[request.nodeId] = result;
    if (context.logMessage)
        // 中文翻译：If 条件：%1
        context.logMessage(QObject::tr("If condition: %1")
                               .arg(result ? QObject::tr("true") : QObject::tr("false")));
    return true;
}

} // namespace lcnc::process
