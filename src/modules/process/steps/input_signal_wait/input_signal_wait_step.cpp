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
constexpr char kSignalType[]="signalType"; constexpr char kIoName[]="ioName"; constexpr char kTarget[]="targetValue"; constexpr char kTimeout[]="timeoutMs"; constexpr char kPoll[]="pollIntervalMs"; constexpr char kTypeCombo[]="signalTypeBox"; constexpr char kIoCombo[]="ioNameBox"; constexpr char kTargetCheck[]="targetCheck"; constexpr char kTimeoutSpin[]="timeoutSpin"; constexpr char kPollSpin[]="pollSpin";
QString ioKeyFromDisplay(const QString& display){ const int l=display.lastIndexOf('('), r=display.lastIndexOf(')'); return (l>=0&&r>l)?display.mid(l+1,r-l-1):display; }
void refill(QComboBox* box, const QString& type){ box->clear(); if (const auto* settings=ProcessStepRegistry::instance().settingsService()) box->addItems(settings->ioDisplayNames(type==QStringLiteral("analog")?ProcessIoBucket::AnalogInput:ProcessIoBucket::DigitalInput)); }
}

ProcessNodeDescriptor InputSignalWaitStep::descriptor() const{ ProcessNodeDescriptor d; d.type=ProcessNodeType::InputSignalWait; d.displayName=QObject::tr("输入信号"); d.category=QObject::tr("IO"); d.executorKey=QStringLiteral("inputSignalWait"); d.defaultParameters.insert(QString::fromLatin1(kSignalType), QStringLiteral("digital")); d.defaultParameters.insert(QString::fromLatin1(kIoName), QStringLiteral("aStart")); d.defaultParameters.insert(QString::fromLatin1(kTarget), true); d.defaultParameters.insert(QString::fromLatin1(kTimeout), 5000); d.defaultParameters.insert(QString::fromLatin1(kPoll), 100); return d; }
QString InputSignalWaitStep::summary(const ProcessNode& node) const{ return QObject::tr("等待 %1=%2 timeout=%3ms").arg(node.parameters.value(kIoName,"aStart").toString(), node.parameters.value(kTarget,true).toString(), node.parameters.value(kTimeout,5000).toString()); }
QWidget* InputSignalWaitStep::createParameterEditor(const ProcessNode& node, QWidget* parent) const{ auto* page=new QWidget(parent); auto* form=new QFormLayout(page); auto* type=new QComboBox(page); type->setObjectName(kTypeCombo); type->addItems({"digital","analog"}); type->setCurrentText(node.parameters.value(kSignalType,"digital").toString()); form->addRow(QObject::tr("输入类型"), type); auto* io=new QComboBox(page); io->setObjectName(kIoCombo); refill(io,type->currentText()); const QString current=node.parameters.value(kIoName).toString(); for(int i=0;i<io->count();++i){ if(ioKeyFromDisplay(io->itemText(i))==current){ io->setCurrentIndex(i); break; }} QObject::connect(type,&QComboBox::currentTextChanged,io,[io](const QString& t){ refill(io,t); }); form->addRow(QObject::tr("IO 名"), io); auto* target=new QCheckBox(QObject::tr("目标为 true"),page); target->setObjectName(kTargetCheck); target->setChecked(node.parameters.value(kTarget,true).toBool()); form->addRow(QString(),target); auto* timeout=new QSpinBox(page); timeout->setObjectName(kTimeoutSpin); timeout->setRange(0,24*60*60*1000); timeout->setSuffix(" ms"); timeout->setValue(node.parameters.value(kTimeout,5000).toInt()); form->addRow(QObject::tr("超时"),timeout); auto* poll=new QSpinBox(page); poll->setObjectName(kPollSpin); poll->setRange(10,60000); poll->setSuffix(" ms"); poll->setValue(node.parameters.value(kPoll,100).toInt()); form->addRow(QObject::tr("刷新间隔"),poll); return page; }
bool InputSignalWaitStep::applyParameterEditor(QWidget* editor, ProcessNode& node, QString*) const{ node.parameters.insert(QString::fromLatin1(kSignalType), editor->findChild<QComboBox*>(kTypeCombo)->currentText()); node.parameters.insert(QString::fromLatin1(kIoName), ioKeyFromDisplay(editor->findChild<QComboBox*>(kIoCombo)->currentText())); node.parameters.insert(QString::fromLatin1(kTarget), editor->findChild<QCheckBox*>(kTargetCheck)->isChecked()); node.parameters.insert(QString::fromLatin1(kTimeout), editor->findChild<QSpinBox*>(kTimeoutSpin)->value()); node.parameters.insert(QString::fromLatin1(kPoll), editor->findChild<QSpinBox*>(kPollSpin)->value()); return true; }
bool InputSignalWaitStep::execute(const ProcessNodeExecutionRequest& request, ProcessStepContext& context, QString* errorMessage){ if(!context.io){ if(errorMessage)*errorMessage=QObject::tr("IO 服务不可用"); return false;} return context.io->waitInput(request.parameters.value(kSignalType,"digital").toString(), request.parameters.value(kIoName,"aStart").toString(), request.parameters.value(kTarget,true), request.parameters.value(kTimeout,5000).toInt(), request.parameters.value(kPoll,100).toInt(), errorMessage); }
int InputSignalWaitStep::completionDelayMs(const ProcessNodeExecutionRequest& request) const { return std::clamp(request.parameters.value(kPoll,100).toInt(),10,60000); }

} // namespace lcnc::process
