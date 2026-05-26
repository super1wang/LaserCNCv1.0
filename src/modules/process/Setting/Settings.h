#pragma once

#include <set>
#include <string>
#include "toml.hpp"
#include "DataType.h"

using std::string;
using toml::table;
using toml::value;

#define SETTINGS Settings::instance()

class Settings
{
public:
	Settings();
	~Settings();

	// 函数宏指针
	static Settings*	instance();

	// 存读全部参数
	bool				SaveSettings(string FilePath);  //处理并集中非初始参数部分，通过与初始参数比对，删除对应的Key&Value
	bool				LoadSettings(string FilePath);  //先读取初始参数，再读取用户参数，得出并集Settings 
	bool				LoadSettingsValue(const value& valueSetting);

	// 存读节参数
	value				GetPeripheralSetting(); //获取工艺参数
	value				GetTechnologySetting(); //获取工艺参数
	bool				SavePeripheralSetting(string FilePath);	//保存外设参数
	bool				SaveTechnologySetting(string FilePath);	//保存工艺参数
	bool				SaveSectionSetting(string FilePath, SettingSection Section, string TableName = "");	//单独保存某一节参数至某一文件, 不可重复保存至同一文件
	bool				LoadSectionSetting(string FilePath, SettingSection Section, string TableName = "");
	bool				LoadSectionSetting(table Table, SettingSection Section, string TableName = "");

	// 读写表，缺省值用于嵌套表
	table				GetTable(SettingSection Section, string TableName = "");
	void				SetTable(bool bOverride, SettingSection Section, table Table, string TableName = ""); //Override True:重写，false:修改
	void				DelTable(SettingSection Section, string TableName);

	// 读指定参数
	void				GetKeyValue(string Key, bool&    Value, SettingSection Section, string TableName = "");
	void				GetKeyValue(string Key, int&     Value, SettingSection Section, string TableName = "");
	void				GetKeyValue(string Key, double&  Value, SettingSection Section, string TableName = "");
	void				GetKeyValue(string Key, string&  Value, SettingSection Section, string TableName = "");
	void				GetKeyValue(string Key, value&	 Value, SettingSection Section, string TableName = "");

	// 写指定参数
	void				SetKeyValue(string Key, bool    Value, SettingSection Section, string TableName = "");
	void				SetKeyValue(string Key, int     Value, SettingSection Section, string TableName = "");
	void				SetKeyValue(string Key, double  Value, SettingSection Section, string TableName = "");
	void				SetKeyValue(string Key, string  Value, SettingSection Section, string TableName = "");
	void				SetKeyValue(string Key, value	Value, SettingSection Section, string TableName = "");

private:
	void				LoadTable(bool bOverride, table& Value_Original, const table& Value_Input);
	
	static Settings*	uniqueInstance;
	table				UserSetting;		//用户参数
};