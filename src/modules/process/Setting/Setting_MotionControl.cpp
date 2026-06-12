#include "RegexPatterns.h"
#include "Setting_MotionControl.h"

#include <QHeaderView>
#include <QColor>

Dialog_Setting_MotionControl::Dialog_Setting_MotionControl(QWidget* parent)
	: QDialog(parent)
	, set_Changed()
{
	ui.setupUi(this);
	setWindowFlags(Qt::CustomizeWindowHint | Qt::FramelessWindowHint);

	// Setup table headers
	QTableWidget* tw = ui.tableWidget_AxisConfig;
	tw->setColumnCount(Col_Count);
	QStringList headers;
	headers << tr("Axis") << tr("Index") << tr("HomeIndex") << tr("Resolution")
			<< tr("LowSpeed\n(mm/s)") << tr("MediumSpeed\n(mm/s)") << tr("HighSpeed\n(mm/s)")
			<< tr("Acc\n(mm/s^2)") << tr("Jerk\n(mm/s^3)")
			<< tr("LeftLimit\n(mm)") << tr("RightLimit\n(mm)");
	tw->setHorizontalHeaderLabels(headers);
	tw->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);

	connect(ui.comboBox_MotionControl_sType, SIGNAL(activated(int)), this, SLOT(TypeChanged()));
	connect(tw, &QTableWidget::cellChanged, this, &Dialog_Setting_MotionControl::onTableCellChanged);
	connect(ui.pushButton_AddAxis, &QPushButton::clicked, this, &Dialog_Setting_MotionControl::onAddAxis);
	connect(ui.pushButton_DeleteAxis, &QPushButton::clicked, this, &Dialog_Setting_MotionControl::onDeleteAxis);
}

Dialog_Setting_MotionControl::~Dialog_Setting_MotionControl()
{
}

void Dialog_Setting_MotionControl::setUI()
{
	TypeChanged();
}

void Dialog_Setting_MotionControl::ClearChange()
{
	set_Changed.clear();
	table_Temp.clear();
}

void Dialog_Setting_MotionControl::rebuildAxisNames()
{
	m_machineAxisNames.clear();
	for (const auto& eAxis : magic_enum::enum_values<Axis>())
	{
		if (DT::IsAxisUse(eAxis))
			m_machineAxisNames.append(QString::fromStdString(enum_name(eAxis).data()));
	}

	// Load extension axes from settings
	m_extensionAxisNames.clear();
	string extStr;
	SETTINGS->GetKeyValue("ExtensionAxes", extStr, SettingSection::MotionControl, "MotionControl");
	if (!extStr.empty())
	{
		for (const QString& p : QString::fromStdString(extStr).split(',', Qt::SkipEmptyParts))
		{
			QString name = p.trimmed().toUpper();
			if (!name.isEmpty() && !m_machineAxisNames.contains(name))
				m_extensionAxisNames.append(name);
		}
	}

	m_axisNames = m_machineAxisNames + m_extensionAxisNames;

	// Also sync to DT
	DT::setExtensionAxes(m_extensionAxisNames);
}

bool Dialog_Setting_MotionControl::isMachineAxis(const QString& name) const
{
	return m_machineAxisNames.contains(name);
}

void Dialog_Setting_MotionControl::InitSetting()
{
	rebuildAxisNames();

	// Init MotionControl section defaults
	table t_MC_Init;
	t_MC_Init["MotionControl"]["sType"] = "SimulatorCMHP";
	t_MC_Init["MotionControl"]["sAxis"] = m_axisNames.isEmpty() ? "X" : m_axisNames.first().toStdString();
	t_MC_Init["MotionControl"]["ExtensionAxes"] = "";

	for (const QString& axisName : m_axisNames)
	{
		string strAxis = axisName.toStdString();
		t_MC_Init[strAxis]["iIndex"]      = static_cast<int>(m_axisNames.indexOf(axisName));
		t_MC_Init[strAxis]["iHomeIndex"]  = static_cast<int>(m_axisNames.indexOf(axisName));
		t_MC_Init[strAxis]["bRotation"]   = false;
		t_MC_Init[strAxis]["fResolution"] = 2000.0;
		t_MC_Init[strAxis]["fVel"]        = 10.0;
		t_MC_Init[strAxis]["fAcc"]        = 1000.0;
		t_MC_Init[strAxis]["fJerk"]       = 10000.0;
		t_MC_Init[strAxis]["fLeftLimit"]  = 0.0;
		t_MC_Init[strAxis]["fRightLimit"] = 50.0;

		// Home parameters (not editable via UI, stored for controller use)
		t_MC_Init[strAxis]["Home"]["iMode"]               = 10;
		t_MC_Init[strAxis]["Home"]["iMoveDir"]            = -1;
		t_MC_Init[strAxis]["Home"]["iIndexDir"]           = -1;
		t_MC_Init[strAxis]["Home"]["iEdge"]               = 0;
		t_MC_Init[strAxis]["Home"]["iTriggerIndex"]       = -1;
		t_MC_Init[strAxis]["Home"]["fVelHigh"]            = 5.0;
		t_MC_Init[strAxis]["Home"]["fVelLow"]             = 1.0;
		t_MC_Init[strAxis]["Home"]["fAcc"]                = 50.0;
		t_MC_Init[strAxis]["Home"]["fDec"]                = 50.0;
		t_MC_Init[strAxis]["Home"]["iSmoothTime"]         = 0;
		t_MC_Init[strAxis]["Home"]["iHomeOffset"]         = 0;
		t_MC_Init[strAxis]["Home"]["iSearchHomeDistance"] = 0;
		t_MC_Init[strAxis]["Home"]["iSearchIndexDistance"]= 0;
		t_MC_Init[strAxis]["Home"]["iEscapeStep"]         = 20000;
	}
	SETTINGS->SetTable(true, SettingSection::MotionControl, t_MC_Init);

	// Init Axis section defaults (speed tiers, kept in Axis section for backward compat)
	table t_Axis_Init;
	t_Axis_Init["Axis"]["sAxis"] = m_axisNames.isEmpty() ? "X" : m_axisNames.first().toStdString();
	for (const QString& axisName : m_axisNames)
	{
		string strAxis = axisName.toStdString();
		t_Axis_Init[strAxis]["fLowSpeed"]     = 3.0;
		t_Axis_Init[strAxis]["fMediumSpeed"]  = 5.0;
		t_Axis_Init[strAxis]["fHighSpeed"]    = 10.0;
		t_Axis_Init[strAxis]["fPipeDiameter"] = 1.6;
	}
	SETTINGS->SetTable(true, SettingSection::Axis, t_Axis_Init);
}

void Dialog_Setting_MotionControl::SetPage(table table_Set)
{
	if (!table_Set.size())
		table_Set = SETTINGS->GetTable(SettingSection::MotionControl);

	// Refresh axis names (extension axes may have been loaded from disk)
	rebuildAxisNames();

	// Update controller type combo
	string sType;
	SETTINGS->GetKeyValue("sType", sType, SettingSection::MotionControl, "MotionControl");
	ui.comboBox_MotionControl_sType->blockSignals(true);
	ui.comboBox_MotionControl_sType->setCurrentText(QString::fromStdString(sType));
	ui.comboBox_MotionControl_sType->blockSignals(false);

	populateAxisTable(table_Set);
}

void Dialog_Setting_MotionControl::populateAxisTable(const table& table_Set)
{
	table table_Axis = SETTINGS->GetTable(SettingSection::Axis);

	QTableWidget* tw = ui.tableWidget_AxisConfig;
	tw->blockSignals(true);
	tw->setRowCount(0);
	tw->setRowCount(m_axisNames.size());

	for (int row = 0; row < m_axisNames.size(); ++row)
	{
		string strAxis = m_axisNames[row].toStdString();
		bool bIsMachine = isMachineAxis(m_axisNames[row]);

		// Axis name (read-only for machine axes)
		QTableWidgetItem* nameItem = new QTableWidgetItem(m_axisNames[row]);
		if (bIsMachine)
			nameItem->setFlags(nameItem->flags() & ~Qt::ItemIsEditable);
		tw->setItem(row, Col_AxisName, nameItem);

		// Highlight extension axis rows
		if (!bIsMachine)
		{
			nameItem->setBackground(QColor(220, 240, 255));
		}

		// Helper: get value from table
		auto getDouble = [&](const char* key, double def) -> double {
			if (table_Set.count(strAxis) && table_Set.at(strAxis).contains(key))
				return table_Set.at(strAxis).at(key).as_floating();
			return def;
		};
		auto getInt = [&](const char* key, int def) -> int {
			if (table_Set.count(strAxis) && table_Set.at(strAxis).contains(key))
				return table_Set.at(strAxis).at(key).as_integer();
			return def;
		};

		// Index (int)
		tw->setItem(row, Col_Index, new QTableWidgetItem(QString::number(getInt("iIndex", row))));

		// HomeIndex (int)
		tw->setItem(row, Col_HomeIndex, new QTableWidgetItem(QString::number(getInt("iHomeIndex", row))));

		// Resolution (double)
		tw->setItem(row, Col_Resolution, new QTableWidgetItem(QString::number(getDouble("fResolution", 2000.0), 'g', 16)));

		// LowSpeed (from Axis section)
		double fLowSpeed = 3.0;
		if (table_Axis.count(strAxis) && table_Axis.at(strAxis).contains("fLowSpeed"))
			fLowSpeed = table_Axis.at(strAxis).at("fLowSpeed").as_floating();
		tw->setItem(row, Col_LowSpeed, new QTableWidgetItem(QString::number(fLowSpeed, 'g', 16)));

		// MediumSpeed (from Axis section)
		double fMediumSpeed = 5.0;
		if (table_Axis.count(strAxis) && table_Axis.at(strAxis).contains("fMediumSpeed"))
			fMediumSpeed = table_Axis.at(strAxis).at("fMediumSpeed").as_floating();
		tw->setItem(row, Col_MediumSpeed, new QTableWidgetItem(QString::number(fMediumSpeed, 'g', 16)));

		// HighSpeed (from Axis section)
		double fHighSpeed = 10.0;
		if (table_Axis.count(strAxis) && table_Axis.at(strAxis).contains("fHighSpeed"))
			fHighSpeed = table_Axis.at(strAxis).at("fHighSpeed").as_floating();
		tw->setItem(row, Col_HighSpeed, new QTableWidgetItem(QString::number(fHighSpeed, 'g', 16)));

		// Acceleration (from MotionControl section)
		tw->setItem(row, Col_Acceleration, new QTableWidgetItem(QString::number(getDouble("fAcc", 1000.0), 'g', 16)));

		// Jerk (from MotionControl section)
		tw->setItem(row, Col_Jerk, new QTableWidgetItem(QString::number(getDouble("fJerk", 10000.0), 'g', 16)));

		// LeftLimit (from MotionControl section)
		tw->setItem(row, Col_LeftLimit, new QTableWidgetItem(QString::number(getDouble("fLeftLimit", 0.0), 'g', 16)));

		// RightLimit (from MotionControl section)
		tw->setItem(row, Col_RightLimit, new QTableWidgetItem(QString::number(getDouble("fRightLimit", 50.0), 'g', 16)));
	}

	tw->blockSignals(false);
	TypeChanged();
}

void Dialog_Setting_MotionControl::GetPage(table& table_Page)
{
	QTableWidget* tw = ui.tableWidget_AxisConfig;

	// Save controller type
	table_Temp["MotionControl"]["sType"] = ui.comboBox_MotionControl_sType->currentText().toStdString();
	table_Temp["MotionControl"]["sAxis"] = m_axisNames.isEmpty() ? "X" : m_axisNames.first().toStdString();

	// Save extension axis list
	QStringList extNames;
	for (const QString& name : m_axisNames)
	{
		if (!isMachineAxis(name))
			extNames.append(name);
	}
	table_Temp["MotionControl"]["ExtensionAxes"] = extNames.join(",").toStdString();

	for (int row = 0; row < m_axisNames.size(); ++row)
	{
		string strAxis = m_axisNames[row].toStdString();

		auto cellText = [&](int col) -> QString {
			QTableWidgetItem* item = tw->item(row, col);
			return item ? item->text() : QString();
		};

		// MotionControl section params
		table_Temp[strAxis]["iIndex"]      = cellText(Col_Index).toInt();
		table_Temp[strAxis]["iHomeIndex"]  = cellText(Col_HomeIndex).toInt();
		table_Temp[strAxis]["fResolution"] = cellText(Col_Resolution).toDouble();
		table_Temp[strAxis]["fAcc"]        = cellText(Col_Acceleration).toDouble();
		table_Temp[strAxis]["fJerk"]       = cellText(Col_Jerk).toDouble();
		table_Temp[strAxis]["fLeftLimit"]  = cellText(Col_LeftLimit).toDouble();
		table_Temp[strAxis]["fRightLimit"] = cellText(Col_RightLimit).toDouble();

		// Axis section params (speed tiers)
		table_Temp["Axis_Speed"][strAxis]["fLowSpeed"]    = cellText(Col_LowSpeed).toDouble();
		table_Temp["Axis_Speed"][strAxis]["fMediumSpeed"]  = cellText(Col_MediumSpeed).toDouble();
		table_Temp["Axis_Speed"][strAxis]["fHighSpeed"]    = cellText(Col_HighSpeed).toDouble();
	}

	table_Page = table_Temp;
}

bool Dialog_Setting_MotionControl::GetChanged(table table_Page, table& table_Changed)
{
	if (set_Changed.empty())
		return false;

	for (auto it = set_Changed.begin(); it != set_Changed.end(); ++it)
	{
		string	strTable = it->first;
		string	strKey   = it->second;

		bool bAxisSection = (strTable.size() > 5 && strTable.compare(0, 5, "Axis:") == 0);
		if (bAxisSection)
		{
			string axisName = strTable.substr(5);
			value Value = table_Page["Axis_Speed"][axisName][strKey];
			table_Changed["Axis"][axisName][strKey] = Value;
			SETTINGS->SetKeyValue(strKey, Value, SettingSection::Axis, axisName);
			LOG_OPER_INFO(tr("Setting [Axis][%1][%2] %3").arg(tr(axisName.c_str()))
				.arg(tr(strKey.c_str())).arg(toml::format(Value).c_str()).toUtf8().data());
		}
		else
		{
			value Value;
			if (table_Page.count(strTable) && table_Page.at(strTable).contains(strKey))
				Value = table_Page[strTable][strKey];
			else if (strTable == "MotionControl" && table_Page.count("MotionControl") && table_Page.at("MotionControl").contains(strKey))
				Value = table_Page["MotionControl"][strKey];

			if (!Value.is_empty())
			{
				table_Changed[strTable][strKey] = Value;
				SETTINGS->SetKeyValue(strKey, Value, SettingSection::MotionControl, strTable);
				LOG_OPER_INFO(tr("Setting [MotionControl][%1][%2] %3").arg(tr(strTable.c_str()))
					.arg(tr(strKey.c_str())).arg(toml::format(Value).c_str()).toUtf8().data());
			}
		}
	}

	set<pair<string, string>> set_null;
	set_Changed.swap(set_null);
	return true;
}

void Dialog_Setting_MotionControl::onTableCellChanged(int row, int column)
{
	if (row < 0 || row >= m_axisNames.size())
		return;
	if (column <= Col_AxisName)
		return;

	string strAxis = m_axisNames[row].toStdString();

	static const char* mcKeys[] = {
		nullptr,        // Col_AxisName
		"iIndex",       // Col_Index
		"iHomeIndex",   // Col_HomeIndex
		"fResolution",  // Col_Resolution
		nullptr,        // Col_LowSpeed → Axis section
		nullptr,        // Col_MediumSpeed → Axis section
		nullptr,        // Col_HighSpeed → Axis section
		"fAcc",         // Col_Acceleration
		"fJerk",        // Col_Jerk
		"fLeftLimit",   // Col_LeftLimit
		"fRightLimit",  // Col_RightLimit
	};

	static const char* axisKeys[] = {
		nullptr, nullptr, nullptr, nullptr,
		"fLowSpeed", "fMediumSpeed", "fHighSpeed",
		nullptr, nullptr, nullptr, nullptr,
	};

	if (column >= 0 && column < Col_Count)
	{
		if (axisKeys[column])
			set_Changed.insert(make_pair("Axis:" + strAxis, string(axisKeys[column])));
		else if (mcKeys[column])
			set_Changed.insert(make_pair(strAxis, string(mcKeys[column])));
	}
}

void Dialog_Setting_MotionControl::onAddAxis()
{
	bool ok = false;
	QString name = QInputDialog::getText(this, tr("Add Extension Axis"),
		tr("Axis name (single letter A-Z):"), QLineEdit::Normal, QString(), &ok);

	if (!ok || name.isEmpty())
		return;

	name = name.trimmed().toUpper();

	// Validate: single uppercase letter
	if (name.size() != 1 || name[0] < 'A' || name[0] > 'Z')
	{
		QMessageBox::warning(this, tr("Invalid Name"),
			tr("Axis name must be a single uppercase letter (A-Z)."));
		return;
	}

	// Validate: not "BASE"
	if (name == QStringLiteral("BASE"))
	{
		QMessageBox::warning(this, tr("Invalid Name"),
			tr("\"BASE\" is reserved and cannot be used as an axis name."));
		return;
	}

	// Validate: not already a machine axis
	if (isMachineAxis(name))
	{
		QMessageBox::warning(this, tr("Already Exists"),
			tr("Axis \"%1\" is already defined in the machine configuration.").arg(name));
		return;
	}

	// Validate: not already an extension axis
	if (m_extensionAxisNames.contains(name))
	{
		QMessageBox::warning(this, tr("Already Exists"),
			tr("Extension axis \"%1\" already exists.").arg(name));
		return;
	}

	// Add the extension axis
	m_extensionAxisNames.append(name);
	m_axisNames = m_machineAxisNames + m_extensionAxisNames;

	// Initialize defaults in settings
	string strAxis = name.toStdString();
	int idx = m_axisNames.size() - 1;

	SETTINGS->SetKeyValue("iIndex", idx, SettingSection::MotionControl, strAxis);
	SETTINGS->SetKeyValue("iHomeIndex", idx, SettingSection::MotionControl, strAxis);
	SETTINGS->SetKeyValue("bRotation", false, SettingSection::MotionControl, strAxis);
	SETTINGS->SetKeyValue("fResolution", 2000.0, SettingSection::MotionControl, strAxis);
	SETTINGS->SetKeyValue("fVel", 10.0, SettingSection::MotionControl, strAxis);
	SETTINGS->SetKeyValue("fAcc", 1000.0, SettingSection::MotionControl, strAxis);
	SETTINGS->SetKeyValue("fJerk", 10000.0, SettingSection::MotionControl, strAxis);
	SETTINGS->SetKeyValue("fLeftLimit", 0.0, SettingSection::MotionControl, strAxis);
	SETTINGS->SetKeyValue("fRightLimit", 50.0, SettingSection::MotionControl, strAxis);
	SETTINGS->SetKeyValue("fLowSpeed", 3.0, SettingSection::Axis, strAxis);
	SETTINGS->SetKeyValue("fMediumSpeed", 5.0, SettingSection::Axis, strAxis);
	SETTINGS->SetKeyValue("fHighSpeed", 10.0, SettingSection::Axis, strAxis);
	SETTINGS->SetKeyValue("fPipeDiameter", 1.6, SettingSection::Axis, strAxis);

	DT::setExtensionAxes(m_extensionAxisNames);

	// Record the extension axis list change
	set_Changed.insert(make_pair("MotionControl", "ExtensionAxes"));

	// Rebuild table
	table table_Set = SETTINGS->GetTable(SettingSection::MotionControl);
	populateAxisTable(table_Set);

	LOG_OPER_INFO(tr("Added extension axis: %1").arg(name).toUtf8().data());
}

void Dialog_Setting_MotionControl::onDeleteAxis()
{
	QTableWidget* tw = ui.tableWidget_AxisConfig;
	int row = tw->currentRow();
	if (row < 0)
	{
		QMessageBox::information(this, tr("No Selection"),
			tr("Please select an axis row to delete."));
		return;
	}

	QString axisName = m_axisNames[row];

	if (isMachineAxis(axisName))
	{
		QMessageBox::warning(this, tr("Cannot Delete"),
			tr("Machine axis \"%1\" cannot be deleted. Only extension axes can be removed.").arg(axisName));
		return;
	}

	int ret = QMessageBox::question(this, tr("Confirm Delete"),
		tr("Delete extension axis \"%1\"? This will remove all its configuration data.").arg(axisName),
		QMessageBox::Yes | QMessageBox::No, QMessageBox::No);

	if (ret != QMessageBox::Yes)
		return;

	// Remove from extension list
	m_extensionAxisNames.removeAll(axisName);
	m_axisNames = m_machineAxisNames + m_extensionAxisNames;

	// Remove settings data
	string strAxis = axisName.toStdString();
	SETTINGS->DelTable(SettingSection::MotionControl, strAxis);
	SETTINGS->DelTable(SettingSection::Axis, strAxis);

	DT::setExtensionAxes(m_extensionAxisNames);

	// Record the extension axis list change
	set_Changed.insert(make_pair("MotionControl", "ExtensionAxes"));

	// Rebuild table
	table table_Set = SETTINGS->GetTable(SettingSection::MotionControl);
	populateAxisTable(table_Set);

	LOG_OPER_INFO(tr("Deleted extension axis: %1").arg(axisName).toUtf8().data());
}

void Dialog_Setting_MotionControl::TypeChanged()
{
	string strType = ui.comboBox_MotionControl_sType->currentText().toStdString();
	m_bGTN = (strType == "GTN");

	QTableWidget* tw = ui.tableWidget_AxisConfig;
	if (!tw || tw->columnCount() < Col_Count)
		return;

	tw->setColumnHidden(Col_HomeIndex, m_bGTN);

	if (tw->horizontalHeaderItem(Col_Jerk))
	{
		if (m_bGTN)
			tw->horizontalHeaderItem(Col_Jerk)->setText(tr("smoothTime\n(ms)"));
		else
			tw->horizontalHeaderItem(Col_Jerk)->setText(tr("Jerk\n(mm/s^3)"));
	}

	if (tw->horizontalHeaderItem(Col_Resolution))
	{
		if (m_bGTN)
			tw->horizontalHeaderItem(Col_Resolution)->setText(tr("Conversion\n(pulse)"));
		else
			tw->horizontalHeaderItem(Col_Resolution)->setText(tr("Resolution"));
	}

	// Record the type change
	set_Changed.insert(make_pair("MotionControl", "sType"));
}
