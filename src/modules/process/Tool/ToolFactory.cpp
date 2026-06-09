#include "ToolFactory.h"
#include <boost/lexical_cast.hpp>

map<int,Tool> ToolFactory::m_mapTools;
Tool toolNew;
ToolFactory::ToolFactory()
{
	m_mapTools.clear();
}

Tool* ToolFactory::GetTool(const QString& qstrToolName)
{
	return GetTool(qstrToolName.toStdString());
}

Tool* ToolFactory::GetTool(const string& strToolName)
{
	for (map<int, Tool>::iterator itr = m_mapTools.begin();
		itr != m_mapTools.end(); itr++)
	{
		if (itr->second.m_strName == strToolName)
			return &(itr->second);
	}
	return &m_mapTools[0];
}

void ToolFactory::SetTool( int i,Tool tool )
{
	toolNew = tool;
	m_mapTools.insert(map<int, Tool>::value_type(i, toolNew));
}
	
void ToolFactory::ToolClear()
{
	m_mapTools.clear();
}

