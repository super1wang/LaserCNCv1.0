#pragma once
#include <map>
#include <string>
#include <vector>
#include <QString>
#include "Tool.h"
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


};