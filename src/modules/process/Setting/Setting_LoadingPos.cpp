#include "RegexPatterns.h"
#include "Setting_LoadingPos.h"

Dialog_Setting_LoadingPos::Dialog_Setting_LoadingPos(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
	setupLineEditValidators(this);
	setupCheckBoxValidators(this);
}

Dialog_Setting_LoadingPos::~Dialog_Setting_LoadingPos()
{
}

void Dialog_Setting_LoadingPos::InitSetting()
{
	table t_Init;

	t_Init["LoadingPos"]["bLoadingPosX"]		= false;
	t_Init["LoadingPos"]["bLoadingPosA"]		= false;
	t_Init["LoadingPos"]["bLoadingPosY"]		= false;
	t_Init["LoadingPos"]["bLoadingPosZ"]		= false;
	t_Init["LoadingPos"]["bLoadingPosZIdle"]	= false;
	t_Init["LoadingPos"]["bLoadingPosX1"]		= false;
	t_Init["LoadingPos"]["bLoadingPosA1"]		= false;
	t_Init["LoadingPos"]["bLoadingPosY1"]		= false;
	t_Init["LoadingPos"]["bLoadingPosZ1"]		= false;
	t_Init["LoadingPos"]["bLoadingPosZ1Idle"]	= false;

	t_Init["LoadingPos"]["fLoadingPosX"]		= 0.0;
	t_Init["LoadingPos"]["fLoadingPosA"]		= 0.0;
	t_Init["LoadingPos"]["fLoadingPosY"]		= 0.0;
	t_Init["LoadingPos"]["fLoadingPosZ"]		= 0.0;
	t_Init["LoadingPos"]["fLoadingPosZIdle"]	= 0.0;
	t_Init["LoadingPos"]["fLoadingPosX1"]		= 0.0;
	t_Init["LoadingPos"]["fLoadingPosA1"]		= 0.0;
	t_Init["LoadingPos"]["fLoadingPosY1"]		= 0.0;
	t_Init["LoadingPos"]["fLoadingPosZ1"]		= 0.0;
	t_Init["LoadingPos"]["fLoadingPosZ1Idle"]	= 0.0;

	t_Init["BlankingPos"]["bBlankingPosX"]		= false;
	t_Init["BlankingPos"]["bBlankingPosA"]		= false;
	t_Init["BlankingPos"]["bBlankingPosY"]		= false;
	t_Init["BlankingPos"]["bBlankingPosZ"]		= false;
	t_Init["BlankingPos"]["bBlankingPosZIdle"]	= false;
	t_Init["BlankingPos"]["bBlankingPosX1"]		= false;
	t_Init["BlankingPos"]["bBlankingPosA1"]		= false;
	t_Init["BlankingPos"]["bBlankingPosY1"]		= false;
	t_Init["BlankingPos"]["bBlankingPosZ1"]		= false;
	t_Init["BlankingPos"]["bBlankingPosZ1Idle"] = false;

	t_Init["BlankingPos"]["fBlankingPosX"]		= 0.0;
	t_Init["BlankingPos"]["fBlankingPosA"]		= 0.0;
	t_Init["BlankingPos"]["fBlankingPosY"]		= 0.0;
	t_Init["BlankingPos"]["fBlankingPosZ"]		= 0.0;
	t_Init["BlankingPos"]["fBlankingPosZIdle"]	= 0.0;
	t_Init["BlankingPos"]["fBlankingPosX1"]		= 0.0;
	t_Init["BlankingPos"]["fBlankingPosA1"]		= 0.0;
	t_Init["BlankingPos"]["fBlankingPosY1"]		= 0.0;
	t_Init["BlankingPos"]["fBlankingPosZ1"]		= 0.0;
	t_Init["BlankingPos"]["fBlankingPosZ1Idle"] = 0.0;


	if (!DT::IsAxisUse(Axis::X))
	{
		ui.checkBox_LoadingPos_bLoadingPosX->setHidden(true);
		ui.lineEdit_LoadingPos_fLoadingPosX->setHidden(true);
		ui.checkBox_BlankingPos_bBlankingPosX->setHidden(true);
		ui.lineEdit_BlankingPos_fBlankingPosX->setHidden(true);
	}
	if (!DT::IsAxisUse(Axis::A))
	{
		ui.checkBox_LoadingPos_bLoadingPosA->setHidden(true);
		ui.lineEdit_LoadingPos_fLoadingPosA->setHidden(true);
		ui.checkBox_BlankingPos_bBlankingPosA->setHidden(true);
		ui.lineEdit_BlankingPos_fBlankingPosA->setHidden(true);
	}
	if (!DT::IsAxisUse(Axis::Y))
	{
		ui.checkBox_LoadingPos_bLoadingPosY->setHidden(true);
		ui.lineEdit_LoadingPos_fLoadingPosY->setHidden(true);
		ui.checkBox_BlankingPos_bBlankingPosY->setHidden(true);
		ui.lineEdit_BlankingPos_fBlankingPosY->setHidden(true);
	}
	if (!DT::IsAxisUse(Axis::Z))
	{
		ui.checkBox_LoadingPos_bLoadingPosZ->setHidden(true);
		ui.lineEdit_LoadingPos_fLoadingPosZ->setHidden(true);
		ui.checkBox_LoadingPos_bLoadingPosZIdle->setHidden(true);
		ui.lineEdit_LoadingPos_fLoadingPosZIdle->setHidden(true);
		ui.checkBox_BlankingPos_bBlankingPosZ->setHidden(true);
		ui.lineEdit_BlankingPos_fBlankingPosZ->setHidden(true);
		ui.checkBox_BlankingPos_bBlankingPosZIdle->setHidden(true);
		ui.lineEdit_BlankingPos_fBlankingPosZIdle->setHidden(true);
	}
	if (!DT::IsAxisUse(Axis::X1))
	{
		ui.checkBox_LoadingPos_bLoadingPosX1->setHidden(true);
		ui.lineEdit_LoadingPos_fLoadingPosX1->setHidden(true);
		ui.checkBox_BlankingPos_bBlankingPosX1->setHidden(true);
		ui.lineEdit_BlankingPos_fBlankingPosX1->setHidden(true);
	}
	if (!DT::IsAxisUse(Axis::A1))
	{
		ui.checkBox_LoadingPos_bLoadingPosA1->setHidden(true);
		ui.lineEdit_LoadingPos_fLoadingPosA1->setHidden(true);
		ui.checkBox_BlankingPos_bBlankingPosA1->setHidden(true);
		ui.lineEdit_BlankingPos_fBlankingPosA1->setHidden(true);
	}
	if (!DT::IsAxisUse(Axis::Y1))
	{
		ui.checkBox_LoadingPos_bLoadingPosY1->setHidden(true);
		ui.lineEdit_LoadingPos_fLoadingPosY1->setHidden(true);
		ui.checkBox_BlankingPos_bBlankingPosY1->setHidden(true);
		ui.lineEdit_BlankingPos_fBlankingPosY1->setHidden(true);
	}

	if (!DT::IsAxisUse(Axis::Z1))
	{
		ui.checkBox_LoadingPos_bLoadingPosZ1->setHidden(true);
		ui.lineEdit_LoadingPos_fLoadingPosZ1->setHidden(true);
		ui.checkBox_LoadingPos_bLoadingPosZ1Idle->setHidden(true);
		ui.lineEdit_LoadingPos_fLoadingPosZ1Idle->setHidden(true);
		ui.checkBox_BlankingPos_bBlankingPosZ1->setHidden(true);
		ui.lineEdit_BlankingPos_fBlankingPosZ1->setHidden(true);
		ui.checkBox_BlankingPos_bBlankingPosZ1Idle->setHidden(true);
		ui.lineEdit_BlankingPos_fBlankingPosZ1Idle->setHidden(true);
	}
	SETTINGS->SetTable(true, SettingSection::LoadingPos, t_Init);
}

void Dialog_Setting_LoadingPos::SetPage(table table_Set)
{
	if (!table_Set.size())
		table_Set = SETTINGS->GetTable(SettingSection::LoadingPos);
	
	QStringList parts;
	for (QCheckBox* checkBox : m_qlCheckBoxB)
	{
		parts = checkBox->objectName().split('_');
		checkBox->setChecked(table_Set[parts[1].toStdString()][parts[2].toStdString()].as_boolean());
	}

	for (QLineEdit* lineEdit : m_qlLineEditF)
	{
		parts = lineEdit->objectName().split('_');
		lineEdit->setText(QString::number(table_Set[parts[1].toStdString()][parts[2].toStdString()].as_floating(), 'g', 16));
	}
}

void Dialog_Setting_LoadingPos::GetPage(table& table_Page)
{
	QStringList parts;
	for (QCheckBox* checkBox : m_qlCheckBoxB)
	{
		parts = checkBox->objectName().split('_');
		table_Page[parts[1].toStdString()][parts[2].toStdString()] = checkBox->isChecked();
	}

	for (QLineEdit* lineEdit : m_qlLineEditF)
	{
		parts = lineEdit->objectName().split('_');
		table_Page[parts[1].toStdString()][parts[2].toStdString()] = lineEdit->text().toDouble();
	}
}

bool Dialog_Setting_LoadingPos::GetChanged(table table_Page, table& table_Changed)
{
	if (set_Changed.empty())
		return false;

	for (auto it = set_Changed.begin(); it != set_Changed.end(); ++it)
	{
		string	strTable = it->first;
		string	strKey = it->second;
		value	Value = table_Page[strTable][strKey];
		table_Changed[strTable][strKey] = Value;
		SETTINGS->SetKeyValue(strKey, Value, SettingSection::LoadingPos, strTable);
		LOG_OPER_INFO(tr("Setting [LoadingPos][%1][%2] %3").arg(tr(strTable.c_str()))
			.arg(tr(strKey.c_str())).arg(toml::format(Value).c_str()).toUtf8().data());
	}
	set<pair<string, string>> set_null;
	set_Changed.swap(set_null);
	return true;
}

void Dialog_Setting_LoadingPos::lineEditChanged()
{
	QStringList parts	 = qobject_cast<QLineEdit*>(sender())->objectName().split("_");
	string		strTable = parts.at(parts.size() - 2).toStdString();
	string		strKey	 = parts.at(parts.size() - 1).toStdString();
	set_Changed.insert(make_pair(strTable, strKey));
}

void Dialog_Setting_LoadingPos::checkBoxChanged()
{
	QStringList parts	 = qobject_cast<QCheckBox*>(sender())->objectName().split("_");
	string		strTable = parts.at(parts.size() - 2).toStdString();
	string		strKey	 = parts.at(parts.size() - 1).toStdString();
	set_Changed.insert(make_pair(strTable, strKey));
}

void Dialog_Setting_LoadingPos::setupLineEditValidators(QWidget* dialog)
{
	const QList<QLineEdit* > lineEdits = dialog->findChildren<QLineEdit*>();
	for (QLineEdit* lineEdit : lineEdits)
	{
		QStringList parts = lineEdit->objectName().split('_');
		if (parts.size() >= 3)
		{
			if (parts[2].left(1) == "f")
			{
				lineEdit->setValidator(new QRegularExpressionValidator(Regex_Pos_Double(), nullptr));
				m_qlLineEditF.append(lineEdit);
			}
			connect(lineEdit, SIGNAL(editingFinished()), this, SLOT(lineEditChanged()));
		}
	}
}

void Dialog_Setting_LoadingPos::setupCheckBoxValidators(QWidget* dialog)
{
	const QList<QCheckBox* > checkBoxs = dialog->findChildren<QCheckBox*>();
	for (QCheckBox* checkBox : checkBoxs)
	{
		QStringList parts = checkBox->objectName().split('_');
		if (parts.size() >= 3)
		{
			connect(checkBox, SIGNAL(clicked()), this, SLOT(checkBoxChanged()));
			m_qlCheckBoxB.append(checkBox);
		}
	}
}
