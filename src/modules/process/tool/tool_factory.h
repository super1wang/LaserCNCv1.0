#pragma once
#include <map>
#include <string>
#include <vector>
#include <QString>
#include <QStringList>
#include "tool.h"
using std::map;
using std::vector;
using std::string;

#define TOOL_NUM 40

class ToolFactory
{
private:
	static map<int,Tool> m_mapTools;			// 工具名和工具的键值对

public:
	ToolFactory();

	static Tool* GetTool(const QString&);
	static Tool* GetTool(const string&);

	void  SetTool(int,Tool);

	void ToolClear();

	/// 枚举当前注册过的工具名（按 map 索引升序），用于 UI 下拉。
	static QStringList toolNames();
	/// Stable snapshot keyed by factory index for project-local persistence.
	static ::toml::table snapshot();
	static bool restoreSnapshot(const ::toml::table& snapshot);
};
