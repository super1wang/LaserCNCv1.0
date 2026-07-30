#include "ToolFactory.h"

#include "core/logging/logger.h"

#include <boost/lexical_cast.hpp>

map<int,Tool> ToolFactory::m_mapTools;
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
	// Never materialize an implicit default entry on a failed lookup.  That
	// hid missing tool configuration and left stale state in the global map.
	return m_mapTools.empty() ? nullptr : &(m_mapTools.begin()->second);
}

void ToolFactory::SetTool( int i,Tool tool )
{
	m_mapTools.insert_or_assign(i, std::move(tool));
}
	
void ToolFactory::ToolClear()
{
	m_mapTools.clear();
}

QStringList ToolFactory::toolNames()
{
	QStringList names;
	for (const auto& kv : m_mapTools) {
		const QString name = QString::fromStdString(kv.second.m_strName);
		if (!name.trimmed().isEmpty() && !names.contains(name))
			names.append(name);
	}
	return names;
}

::toml::table ToolFactory::snapshot()
{
	::toml::table root;
	for (const auto& [index, tool] : m_mapTools)
		root[std::to_string(index)] = tool.toTable();
	return root;
}

bool ToolFactory::restoreSnapshot(const ::toml::table& snapshot)
{
	map<int, Tool> restored;
	for (const auto& [key, value] : snapshot) {
		if (!value.is_table())
			return false;
		try {
			Tool tool;
			tool.SetFromTable(value.as_table());
			restored.emplace(std::stoi(key), std::move(tool));
		} catch (const std::exception& exception) {
			LCNC_ERR(lcnc::LogCode::Generic,
					 "process.tool: failed to restore tool '{}': {}",
					 key,
					 exception.what());
			return false;
		} catch (...) {
			LCNC_ERR(lcnc::LogCode::Generic,
					 "process.tool: failed to restore tool '{}' due to an unknown exception",
					 key);
			return false;
		}
	}
	if (restored.empty())
		return false;
	m_mapTools = std::move(restored);
	return true;
}

