#include "modules/process/steps/output_signal/output_signal_step.h"

#include "modules/process/settings/process_settings_service.h"

#include <QComboBox>
#include <QFormLayout>
#include <QLineEdit>
#include <QObject>
#include <QWidget>

namespace lcnc::process {
namespace {
constexpr char kSignalType[]="signalType"; constexpr char kIoName[]="ioName"; constexpr char kValue[]="value"; constexpr char kTypeCombo[]="signalTypeBox"; constexpr char kIoCombo[]="ioNameBox"; constexpr char kValueEdit[]="valueEdit";
QString ioKeyFromDisplay(const QString& display){ const int l=display.lastIndexOf('('), r=display.lastIndexOf(')'); return (l>=0&&r>l)?display.mid(l+1,r-l-1):display; }
void refill(QComboBox* box, const QString& type){ box->clear(); if (auto* settings=ProcessSettingsService::current()) box->addItems(settings->ioDisplayNames(type==QStringLiteral("analog")?ProcessIoBucket::AnalogOutput:ProcessIoBucket::DigitalOutput)); }
}

ProcessNodeDescriptor OutputSignalStep::descriptor() const{ ProcessNodeDescriptor d; d.type=ProcessNodeType::OutputSignal; d.displayName=QObject::tr("输出信号"); d.category=QObject::tr("IO"); d.executorKey=QStringLiteral("outputSignal"); d.defaultParameters.insert(QString::fromLatin1(kSignalType), QStringLiteral("digital")); d.defaultParameters.insert(QString::fromLatin1(kIoName), QStringLiteral("aLaser")); d.defaultParameters.insert(QString::fromLatin1(kValue), true); return d; }
QString OutputSignalStep::summary(const ProcessNode& node) const{ return QObject::tr("%1 %2=%3").arg(node.parameters.value(kSignalType,"digital").toString(), node.parameters.value(kIoName,"aLaser").toString(), node.parameters.value(kValue,true).toString()); }
QWidget* OutputSignalStep::createParameterEditor(const ProcessNode& node, QWidget* parent) const{ auto* page=new QWidget(parent); auto* form=new QFormLayout(page); auto* type=new QComboBox(page); type->setObjectName(kTypeCombo); type->addItems({"digital","analog"}); type->setCurrentText(node.parameters.value(kSignalType,"digital").toString()); form->addRow(QObject::tr("输出类型"), type); auto* io=new QComboBox(page); io->setObjectName(kIoCombo); refill(io,type->currentText()); const QString current=node.parameters.value(kIoName).toString(); for(int i=0;i<io->count();++i){ if(ioKeyFromDisplay(io->itemText(i))==current){ io->setCurrentIndex(i); break; }} QObject::connect(type,&QComboBox::currentTextChanged,io,[io](const QString& t){ refill(io,t); }); form->addRow(QObject::tr("IO 名"), io); auto* value=new QLineEdit(node.parameters.value(kValue,true).toString(),page); value->setObjectName(kValueEdit); form->addRow(QObject::tr("值"), value); return page; }
bool OutputSignalStep::applyParameterEditor(QWidget* editor, ProcessNode& node, QString*) const{ node.parameters.insert(QString::fromLatin1(kSignalType), editor->findChild<QComboBox*>(kTypeCombo)->currentText()); node.parameters.insert(QString::fromLatin1(kIoName), ioKeyFromDisplay(editor->findChild<QComboBox*>(kIoCombo)->currentText())); node.parameters.insert(QString::fromLatin1(kValue), editor->findChild<QLineEdit*>(kValueEdit)->text().trimmed()); return true; }
bool OutputSignalStep::execute(const ProcessNodeExecutionRequest& request, ProcessStepContext& context, QString* errorMessage){ if(!context.io){ if(errorMessage)*errorMessage=QObject::tr("IO 服务不可用"); return false;} return context.io->setOutput(request.parameters.value(kSignalType,"digital").toString(), request.parameters.value(kIoName,"aLaser").toString(), request.parameters.value(kValue,true), errorMessage); }

} // namespace lcnc::process
