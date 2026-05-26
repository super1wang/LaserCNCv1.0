#include "Settings.h"
#include "MessageModule.h"

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
			if (!t_Original.count(key)) 
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