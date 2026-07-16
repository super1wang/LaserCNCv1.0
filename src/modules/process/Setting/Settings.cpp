#include "Settings.h"
#include "MessageModule.h"
#include "modules/process/Setting/BuiltinIODefs.h"

#include <filesystem>
#include <fstream>

namespace {

void mergeMissingTable(table& target, const table& defaults)
{
	for (const auto& [key, defValue] : defaults)
	{
		if (!target.count(key))
		{
			target[key] = defValue;
			continue;
		}
		if (defValue.is_table())
		{
			if (!target[key].is_table())
				target[key] = table{};
			mergeMissingTable(target[key].as_table(), defValue.as_table());
		}
	}
}

void addIODefaults(table& section, const lcnc::process::BuiltinIODefList& list)
{
	for (int i = 0; i < list.count; ++i)
	{
		const auto& def = list.items[i];
		if (!section.count(def.sectionKey))
			section[def.sectionKey] = table{};
		table& bucket = section[def.sectionKey].as_table();
		if (bucket.count(def.tomlKey))
			continue;
		table entry;
		entry["name"] = string(def.nameZh);
		entry["index"] = string(def.defaultIndex);
		entry["active"] = def.defaultActive;
		entry["enabled"] = def.defaultEnabled;
		entry["showInMain"] = def.defaultShowInMain;
		entry["builtin"] = true;
		bucket[def.tomlKey] = entry;
	}
}

table defaultPeripheralSettings()
{
	table setting;

	table motionControl;
	motionControl["MotionControl"]["sType"] = "SimulatorCMHP";
	motionControl["MotionControl"]["sAxis"] = "X";
	motionControl["MotionControl"]["ExtensionAxes"] = "";
	motionControl["X"]["iIndex"] = 0;
	motionControl["X"]["iHomeIndex"] = 0;
	motionControl["X"]["bRotation"] = false;
	motionControl["X"]["fResolution"] = 2000.0;
	motionControl["X"]["fVel"] = 10.0;
	motionControl["X"]["fAcc"] = 1000.0;
	motionControl["X"]["fJerk"] = 10000.0;
	motionControl["X"]["fLeftLimit"] = 0.0;
	motionControl["X"]["fRightLimit"] = 50.0;
	motionControl["X"]["Home"]["iMode"] = 10;
	motionControl["X"]["Home"]["iMoveDir"] = -1;
	motionControl["X"]["Home"]["iIndexDir"] = -1;
	motionControl["X"]["Home"]["iEdge"] = 0;
	motionControl["X"]["Home"]["iTriggerIndex"] = -1;
	motionControl["X"]["Home"]["fVelHigh"] = 5.0;
	motionControl["X"]["Home"]["fVelLow"] = 1.0;
	motionControl["X"]["Home"]["fAcc"] = 50.0;
	motionControl["X"]["Home"]["fDec"] = 50.0;
	motionControl["X"]["Home"]["iSmoothTime"] = 0;
	motionControl["X"]["Home"]["iHomeOffset"] = 0;
	motionControl["X"]["Home"]["iSearchHomeDistance"] = 0;
	motionControl["X"]["Home"]["iSearchIndexDistance"] = 0;
	motionControl["X"]["Home"]["iEscapeStep"] = 20000;
	setting["MotionControl"] = motionControl;

	table digital;
	addIODefaults(digital, lcnc::process::builtinDigitalOUT());
	addIODefaults(digital, lcnc::process::builtinDigitalIN());
	setting["Digital"] = digital;

	table analog;
	addIODefaults(analog, lcnc::process::builtinAnalogOUT());
	addIODefaults(analog, lcnc::process::builtinAnalogIN());
	setting["Analog"] = analog;

	table laser;
	laser["Laser"]["sType"] = "Simulator";
	laser["Laser"]["fEnergy"] = 20.0;
	laser["Laser"]["fFrequency"] = 20.0;
	laser["Laser"]["fPulseWidth"] = 20.0;
	laser["Laser"]["fResolution"] = 0.0;
	laser["Laser"]["fAttenuatorPercentage"] = 30.0;
	laser["Laser"]["fPpDivider"] = 2.0;
	laser["Laser"]["iDelay"] = 0;
	laser["ComSetting"]["sPort"] = "COM1";
	laser["ComSetting"]["sBaudRate"] = "9600";
	laser["ComSetting"]["sDataBits"] = "8";
	laser["ComSetting"]["sParity"] = "NONE";
	laser["ComSetting"]["sStopBits"] = "1";
	laser["SignalSource"]["bSignal"] = false;
	laser["SignalSource"]["sPort"] = "COM1";
	laser["SignalSource"]["sBaudRate"] = "9600";
	laser["SignalSource"]["sDataBits"] = "8";
	laser["SignalSource"]["sParity"] = "NONE";
	laser["SignalSource"]["sStopBits"] = "1";
	laser["HTTP"]["sHost"] = "127.0.0.1";
	laser["HTTP"]["iPort"] = 20020;
	laser["HTTP"]["sPath"] = "v1/Basic";
	laser["HTTP"]["iTimeOut"] = 1000;
	setting["Laser"] = laser;

	table internet;
	internet["Internet"]["sProtocol"] = "TCP";
	internet["Internet"]["sIP"] = "127.0.0.1";
	internet["Internet"]["sPort"] = "22";
	internet["Internet"]["sCommand"] = "NULL";
	setting["Internet"] = internet;

	setting["Special"] = table{};

	table root;
	root["Setting"] = setting;
	return root;
}

table defaultToolSettings()
{
	table tool;
	tool["ToolIndex"]["sToolIndex"] = "Default";
	tool["ToolIndex"]["sTool_0"] = "Default";

	table def;
	def["fLineVel"] = 10.0;
	def["fArcVel"] = 10.0;
	def["fCutAcc"] = 100.0;
	def["fCutJerk"] = 1000.0;
	def["fXVel"] = 20.0;
	def["fYVel"] = 20.0;
	def["fAVel"] = 10.0;
	def["fA1Vel"] = 10.0;
	def["fX1Vel"] = 20.0;
	def["fY1Vel"] = 20.0;
	def["fZVel"] = 10.0;
	def["fIdelAcc"] = 100.0;
	def["fIdelJerk"] = 1000.0;
	def["fEnergy"] = 20.0;
	def["fPluse"] = 20.0;
	def["fFrequency"] = 30.0;
	def["fAttenuatorPercentage"] = 30.0;
	def["fPpDivider"] = 2.0;
	def["iDelay"] = 0;
	def["fPulsePickerDivider"] = 30.0;
	def["fBeforeOpenLaser"] = 0.0;
	def["fAfterOpenLaser"] = 0.0;
	def["fAfterCloseLaser"] = 0.0;
	def["fBeforeCloseLaser"] = 0.0;
	def["fCornerVelocity"] = 1.0;
	def["fCornerAngle"] = 1.0;
	def["fXSEGVelocity"] = 1.0;
	def["fCutSmoothTime"] = 0.0;
	def["fCutSmoothK"] = 0.0;
	def["fAxisSmoothTime"] = 0.0;
	def["fAxisSmoothK"] = 0.0;
	def["fCuttingHeight"] = 0.0;
	def["fIdleHeight"] = 0.0;
	def["sDirectionsX"] = "X";
	def["sDirectionsY"] = "A";
	def["bStopBlow"] = false;
	def["bPunch"] = false;
	def["bSetPosA"] = false;
	def["fSetPosA"] = 0.0;
	def["bSetPosA1"] = false;
	def["fSetPosA1"] = 0.0;
	def["bMovePosX"] = false;
	def["fMovePosX"] = 0.0;
	def["bMovePosX1"] = false;
	def["fMovePosX1"] = 0.0;
	def["bMovePosA"] = false;
	def["fMovePosA"] = 0.0;
	def["bMovePosA1"] = false;
	def["fMovePosA1"] = 0.0;
	def["bMovePosY"] = false;
	def["fMovePosY"] = 0.0;
	def["bMovePosY1"] = false;
	def["fMovePosY1"] = 0.0;
	def["bCuttingHead"] = false;
	def["bCrossBridge"] = false;
	def["fServoCuttingHeight"] = 0.0;
	def["bAxisZLinkage"] = false;
	def["fLinkedDelay"] = 50.0;
	def["sLinkedDirection"] = "X";
	def["iLinkedMode"] = 0;
	def["fLinkageParameterA"] = 0.0;
	def["fLinkageParameterB"] = 0.0;
	def["sLinkedFormula"] = "";
	def["bTrough"] = false;
	def["iRunBuffer"] = 0;
	def["fCHCompensate"] = 0.0;
	def["fExtendSctart"] = 0.0;
	def["fExtendEnd"] = 0.0;
	def["fAccTime"] = 0.0;
	def["fDelay"] = 0.0;
	def["bFlightCutting"] = false;
	def["fMotorDelay"] = 0.0;
	def["bEnergySwitch"] = false;
	tool["Default"] = def;
	return tool;
}

table defaultTechnologySettings()
{
	table setting;
	setting["Tool"] = defaultToolSettings();

	table axis;
	axis["Axis"]["sAxis"] = "X";
	axis["X"]["fLowSpeed"] = 3.0;
	axis["X"]["fMediumSpeed"] = 5.0;
	axis["X"]["fHighSpeed"] = 10.0;
	axis["X"]["fPipeDiameter"] = 1.6;
	setting["Axis"] = axis;

	table gas;
	gas["Gas"]["bBlow"] = true;
	gas["Gas"]["fBlowDelay"] = 0.0;
	gas["Gas"]["fPressure"] = 500.0;
	gas["GasSetting"]["iConversions"] = 65535;
	setting["Gas"] = gas;

	table water;
	water["Water"]["bWater"] = false;
	water["Water"]["fWaterDelay"] = 0.0;
	water["Pump"]["bPump"] = false;
	water["Pump"]["fPumpOpenTime"] = 10.0;
	water["Pump"]["fPumpCloseTime"] = 2.0;
	setting["Water"] = water;

	table monitor;
	monitor["Cutting"]["bInterLock"] = false;
	monitor["Cutting"]["bSafetyLightCurtain"] = false;
	monitor["Gas"]["bPressureMonitor"] = false;
	monitor["Water"]["bWaterLeakageMonitor"] = false;
	monitor["Water"]["bWaterTankMonitor"] = false;
	monitor["Water"]["bWaterPressureMonitor"] = false;
	monitor["Water"]["fWaterPressureLimit"] = 1.0;
	monitor["Water"]["bWaterLevelMonitor"] = false;
	monitor["Water"]["fWaterLevelLimit"] = 50.0;
	monitor["WaterSetting"]["sWaterPressureConversions"] = "Y=X*4096";
	monitor["WaterSetting"]["sWaterLevelConversions"] = "Y=X*4096";
	setting["Monitor"] = monitor;

	table loading;
	const char* axes[] = { "X", "A", "Y", "Z", "ZIdle", "X1", "A1", "Y1", "Z1", "Z1Idle" };
	for (const char* axis : axes)
	{
		loading["LoadingPos"][string("bLoadingPos") + axis] = false;
		loading["LoadingPos"][string("fLoadingPos") + axis] = 0.0;
		loading["BlankingPos"][string("bBlankingPos") + axis] = false;
		loading["BlankingPos"][string("fBlankingPos") + axis] = 0.0;
	}
	setting["LoadingPos"] = loading;

	table camera;
	camera["Calibration"]["fPixelAccuracy"] = 0.025;
	camera["Calibration"]["iXStandard"] = 0;
	camera["Calibration"]["iYStandard"] = 0;
	camera["Commands"]["sCommand1Name"] = "指令1";
	camera["Commands"]["sCommand2Name"] = "指令2";
	camera["Commands"]["sCommand3Name"] = "指令3";
	camera["Commands"]["sCommand4Name"] = "指令4";
	camera["Commands"]["sCommand1"] = "T1";
	camera["Commands"]["sCommand2"] = "T2";
	camera["Commands"]["sCommand3"] = "T3";
	camera["Commands"]["sCommand4"] = "T4";
	camera["Connect"]["sHost"] = "127.0.0.1";
	camera["Connect"]["iPort"] = 6800;
	camera["Connect"]["iTimeOut"] = 1000;
	setting["Camera"] = camera;

	table root;
	root["Setting"] = setting;
	return root;
}

bool writeTomlFile(const string& filePath, const table& root)
{
	const std::filesystem::path path(filePath);
	if (path.has_parent_path())
		std::filesystem::create_directories(path.parent_path());

	std::ofstream outFile(filePath, std::ios::binary);
	if (!outFile.is_open())
		return false;
	outFile << toml::format(value(root));
	return true;
}

bool mergeDefaultsIntoFile(const string& filePath, const table& defaults, bool* created)
{
	if (created)
		*created = false;

	if (!std::filesystem::exists(filePath))
	{
		if (!writeTomlFile(filePath, defaults))
			return false;
		if (created)
			*created = true;
		return true;
	}

	try
	{
		value input = toml::parse(filePath);
		table merged = input.as_table();
		const table before = merged;
		mergeMissingTable(merged, defaults);
		if (toml::format(value(merged)) != toml::format(value(before)))
			return writeTomlFile(filePath, merged);
	}
	catch (const std::exception&)
	{
		return false;
	}

	return true;
}

} // namespace

Settings* Settings::uniqueInstance = nullptr;

Settings::Settings()
{
}

Settings::~Settings()
{
}

Settings* Settings::instance()
{
	if (!uniqueInstance)
		uniqueInstance = new Settings();
	return uniqueInstance;
}

bool Settings::EnsureDefaultFiles(const string& peripheralPath, const string& technologyPath)
{
	try
	{
		bool peripheralCreated = false;
		bool technologyCreated = false;
		const bool peripheralOk = mergeDefaultsIntoFile(peripheralPath, defaultPeripheralSettings(), &peripheralCreated);
		const bool technologyOk = mergeDefaultsIntoFile(technologyPath, defaultTechnologySettings(), &technologyCreated);

		if (peripheralCreated)
			SHOW_OPER_WARN(WarnCode::WARN_FILE_OPENFAILD,
				QStringLiteral("Peripheral.toml 不存在，已生成默认外设配置并加载。"));
		if (technologyCreated)
			SHOW_OPER_WARN(WarnCode::WARN_FILE_OPENFAILD,
				QStringLiteral("config.toml 不存在，已生成默认工艺配置并加载。"));

		return peripheralOk && technologyOk;
	}
	catch (const std::exception& e)
	{
		SHOW_OPER_WARN(WarnCode::WARN_FILE_OPENFAILD, QString::fromLocal8Bit(e.what()));
		return false;
	}
}

// 向已有table修改
void Settings::LoadTable(bool bOverride, table& Value_Original, const table& Value_Input)//覆写则添加key，反之忽略
{
	auto&		t_Original	= Value_Original;
	const auto& t_Input		= Value_Input;

	for (const auto& [key, v_Input] : t_Input)
	{
		if (v_Input.is_table())
		{
			// 值是table，递归处理
			if (!t_Original.count(key) || !t_Original[key].is_table())
				t_Original[key] = table{};
			LoadTable(bOverride, t_Original[key].as_table(), v_Input.as_table());
		}
		else
		{
			// 值不是table，判断是否存在设置项，进行修改
			if (bOverride)
			{
				t_Original[key] = v_Input;
			}
			else
			{
				if (t_Original.count(key))
					t_Original[key] = v_Input;
			}
		}
	}
}

// 存读全部参数
bool Settings::SaveSettings(string FilePath)
{
	try
	{
		//QByteArray _path = FilePath.toLocal8Bit();
		string _path = FilePath;
		std::ofstream outFile(_path);
		if (!outFile.is_open())
		{
			return false;
		}
		value v_out = UserSetting;
		string str_out = toml::format(v_out);
		outFile << str_out;
		// 关闭文件
		outFile.close();
	}
	catch (const std::exception& e)
	{
		SHOW_OPER_WARN(WarnCode::WARN_FILE_OPENFAILD, QString::fromLocal8Bit(e.what()));
		return false;
	}
	return true;
}

bool Settings::LoadSettings(string FilePath)
{
	try
	{
		//QByteArray _path = FilePath.toLocal8Bit();
		string _path = FilePath;
		value v_input = toml::parse(string(_path));
		LoadTable(true, UserSetting, v_input.as_table());
	}
	catch (const std::exception& e)
	{
		SHOW_OPER_WARN(WarnCode::WARN_FILE_OPENFAILD, QString::fromLocal8Bit(e.what()));
		return false;
	}
	return true;
}

bool Settings::LoadSettingsValue(const value& valueSetting)
{
	if (!valueSetting.size())
	{
		SHOW_OPER_INFO(InfoCode::INFO_FILE_NONESETTINGS);
		return false;
	}

	LoadTable(true, UserSetting, valueSetting.as_table());
	return true;
}

value Settings::GetPeripheralSetting()
{
	table t_out;
	for (Peripheral ePeripheral : magic_enum::enum_values<Peripheral>())
	{
		string str_Section = magic_enum::enum_name(ePeripheral).data();
		t_out["Setting"][str_Section] = UserSetting["Setting"][str_Section];
	}
	value v_out = t_out;
	return v_out;
}

value Settings::GetTechnologySetting()
{
	table t_out;
	for (Technology eTechnology : magic_enum::enum_values<Technology>())
	{
		string str_Section = magic_enum::enum_name(eTechnology).data();
		t_out["Setting"][str_Section] = UserSetting["Setting"][str_Section];
	}
	value v_out = t_out;
	return v_out;
}

bool Settings::SavePeripheralSetting(string FilePath)
{
	try
	{
		string _path = FilePath;
		std::ofstream outFile(_path, std::ios::binary);
		if (!outFile.is_open())
		{
			return false;
		}

		value v_out = GetPeripheralSetting();
		string str_out = toml::format(v_out);
		outFile << str_out;
		// 关闭文件
		outFile.close();
	}
	catch (const std::exception& e)
	{
		SHOW_OPER_WARN(WarnCode::WARN_FILE_OPENFAILD, QString::fromLocal8Bit(e.what()));
		return false;
	}
	return true;
}

bool Settings::SaveTechnologySetting(string FilePath)
{
	try
	{
		string _path = FilePath;
		std::ofstream outFile(_path, std::ios::binary);
		if (!outFile.is_open())
		{
			return false;
		}

		value v_out = GetTechnologySetting();
		string str_out = toml::format(v_out);
		outFile << str_out;
		// 关闭文件
		outFile.close();
	}
	catch (const std::exception& e)
	{
		SHOW_OPER_WARN(WarnCode::WARN_FILE_OPENFAILD, QString::fromLocal8Bit(e.what()));
		return false;
	}
	return true;
}

bool Settings::SaveSectionSetting(string FilePath, SettingSection Section, string TableName)
{
	try
	{
		string str_Section = enum_name(Section).data();
		string _path = FilePath;
		std::ofstream outFile(_path, std::ios::binary);
		if (!outFile.is_open())
		{
			return false;
		}
		value v_out;
		if (TableName == "")
			v_out["Setting"][str_Section] = UserSetting["Setting"][str_Section];
		else
			v_out["Setting"][str_Section][TableName] = UserSetting["Setting"][str_Section][TableName];
		string str_out = toml::format(v_out);
		outFile << str_out;
		// 关闭文件
		outFile.close();
	}
	catch (const std::exception& e)
	{
		SHOW_OPER_WARN(WarnCode::WARN_FILE_OPENFAILD, QString::fromLocal8Bit(e.what()));
		return false;
	}
	return true;
}

bool Settings::LoadSectionSetting(string FilePath, SettingSection Section, string TableName)
{
	try
	{
		string _path = FilePath;
		toml::value v_input = toml::parse(string(_path));
		toml::table t_input = std::move(v_input.as_table());
		return LoadSectionSetting(t_input, Section, TableName);
	}
	catch (const std::exception& e)
	{
		SHOW_OPER_WARN(WarnCode::WARN_FILE_OPENFAILD, QString::fromLocal8Bit(e.what()));
		return false;
	}
	return true;
}

bool Settings::LoadSectionSetting(table Table, SettingSection Section, string TableName)
{
	try
	{
		string str_Section = enum_name(Section).data();
		if (!(Table.count("Setting") && Table["Setting"].count(str_Section)))
			return false;

		if (TableName == "")
		{
			LoadTable(true, UserSetting["Setting"][str_Section].as_table(), Table["Setting"][str_Section].as_table());
		}
		else
		{
			if (Table["Setting"][str_Section].count(TableName))
			{
				// 覆写Tool.ToolIndex整个节点, 同时清空内部的其他Tool中的节点
				if (Section == SettingSection::Tool && TableName == "ToolIndex")
				{
					UserSetting["Setting"]["Tool"].as_table().clear();
					UserSetting["Setting"][str_Section][TableName] = Table["Setting"][str_Section][TableName].as_table();
				}
				else
					LoadTable(true, UserSetting["Setting"][str_Section][TableName].as_table(), Table["Setting"][str_Section][TableName].as_table());
			}
		}
	}
	catch (const std::exception& e)
	{
		SHOW_OPER_WARN(WarnCode::WARN_FILE_OPENFAILD, QString::fromLocal8Bit(e.what()));
		return false;
	}
	return true;
}

// 读写无表头的表，缺省值用于嵌套表
table Settings::GetTable(SettingSection Section, string TableName)
{
	try
	{
		string str_Section = enum_name(Section).data();
		table t_table;
		if (TableName == "")
			t_table = UserSetting["Setting"][str_Section].as_table();
		else
			t_table = UserSetting["Setting"][str_Section][TableName].as_table();
		return t_table;
	}
	catch (const std::exception& e)
	{
		return {};
	}
}

// Override True：重写，false：修改
void Settings::SetTable(bool bOverride, SettingSection Section, table Table, string TableName)
{
	try
	{
		string str_Section = enum_name(Section).data();
		if (bOverride)
		{
			if (TableName == "")
				UserSetting["Setting"][str_Section] = Table;
			else
				UserSetting["Setting"][str_Section][TableName] = Table;
		}
		else
		{
			if (TableName == "")
				LoadTable(false, UserSetting["Setting"][str_Section].as_table(), Table);
			else
				LoadTable(false, UserSetting["Setting"][str_Section][TableName].as_table(), Table);
		}
	}
	catch (const std::exception& e)
	{
		return;
	}
}

void Settings::DelTable(SettingSection Section, string TableName)
{
	try
	{
		string str_Section = enum_name(Section).data();
		//UserSetting["Setting"][str_Section].as_table().erase(TableName);//删不干净

		table table_save;
		for (const auto& pair : UserSetting["Setting"][str_Section].as_table())
		{
			if (pair.first.data() != TableName)
			{
				table_save[pair.first.data()] = UserSetting["Setting"][str_Section][pair.first.data()];
			}
		}
		UserSetting["Setting"][str_Section].as_table().clear();
		UserSetting["Setting"][str_Section] = table_save;
	}
	catch (const std::exception& e)
	{
		return;
	}
}

void Settings::DelKey(string Key, SettingSection Section, string TableName)
{
	try
	{
		const string section = enum_name(Section).data();
		table& target = TableName.empty()
			? UserSetting["Setting"][section].as_table()
			: UserSetting["Setting"][section][TableName].as_table();
		target.erase(Key);
	}
	catch (const std::exception&)
	{
	}
}

// 读写指定参数，返回值用于判定Key值正确性
void Settings::GetKeyValue(string Key, bool& Value, SettingSection Section, string TableName)
{
	string str_Section = enum_name(Section).data();
	try
	{
		if (TableName == "")
			Value = UserSetting["Setting"][str_Section][Key].as_boolean();
		else
			Value = UserSetting["Setting"][str_Section][TableName][Key].as_boolean();
	}
	catch (const std::exception& e)
	{
		return;
	}
}

void Settings::GetKeyValue(string Key, int& Value, SettingSection Section, string TableName)
{
	string str_Section = enum_name(Section).data();
	try
	{
		if (TableName == "")
			Value = UserSetting["Setting"][str_Section][Key].as_integer();
		else
			Value = UserSetting["Setting"][str_Section][TableName][Key].as_integer();
	}
	catch (const std::exception& e)
	{
		return;
	}
}

void Settings::GetKeyValue(string Key, double& Value, SettingSection Section, string TableName)
{
	string str_Section = enum_name(Section).data();
	try
	{
		if (TableName == "")
			Value = UserSetting["Setting"][str_Section][Key].as_floating();
		else
			Value = UserSetting["Setting"][str_Section][TableName][Key].as_floating();
	}
	catch (const std::exception& e)
	{
		return;
	}
}

void Settings::GetKeyValue(string Key, string& Value, SettingSection Section, string TableName)
{
	string str_Section = enum_name(Section).data();
	try
	{
		if (TableName == "")
			Value = UserSetting["Setting"][str_Section][Key].as_string();
		else
			Value = UserSetting["Setting"][str_Section][TableName][Key].as_string();
	}
	catch (const std::exception& e)
	{
		return;
	}
}

void Settings::GetKeyValue(string Key, value& Value, SettingSection Section, string TableName)
{
	string str_Section = enum_name(Section).data();
	try
	{
		if (TableName == "")
			Value = UserSetting["Setting"][str_Section][Key];
		else
			Value = UserSetting["Setting"][str_Section][TableName][Key];
	}
	catch (const std::exception& e)
	{
		return;
	}
}

void Settings::SetKeyValue(string Key, bool Value, SettingSection Section, string TableName)
{
	try
	{
		string str_Section = enum_name(Section).data();
		if (TableName == "")
			UserSetting["Setting"][str_Section][Key] = Value;
		else
			UserSetting["Setting"][str_Section][TableName][Key] = Value;
	}
	catch (const std::exception& e)
	{
		return;
	}
}

void Settings::SetKeyValue(string Key, int Value, SettingSection Section, string TableName)
{
	try
	{
		string str_Section = enum_name(Section).data();
		if (TableName == "")
			UserSetting["Setting"][str_Section][Key] = Value;
		else
			UserSetting["Setting"][str_Section][TableName][Key] = Value;
	}
	catch (const std::exception& e)
	{
		return;
	}
}

void Settings::SetKeyValue(string Key, double Value, SettingSection Section, string TableName)
{
	try
	{
		string str_Section = enum_name(Section).data();
		if (TableName == "")
			UserSetting["Setting"][str_Section][Key] = Value;
		else
			UserSetting["Setting"][str_Section][TableName][Key] = Value;
	}
	catch (const std::exception& e)
	{
		return;
	}
}

void Settings::SetKeyValue(string Key, string Value, SettingSection Section, string TableName)
{
	try
	{
		string str_Section = enum_name(Section).data();
		if (TableName == "")
			UserSetting["Setting"][str_Section][Key] = Value;
		else
			UserSetting["Setting"][str_Section][TableName][Key] = Value;
	}
	catch (const std::exception& e)
	{
		return;
	}
}

void Settings::SetKeyValue(string Key, value Value, SettingSection Section, string TableName)
{
	try
	{
		string str_Section = enum_name(Section).data();
		if (TableName == "")
			UserSetting["Setting"][str_Section][Key] = Value;
		else
			UserSetting["Setting"][str_Section][TableName][Key] = Value;
	}
	catch (const std::exception& e)
	{
		return;
	}
}
