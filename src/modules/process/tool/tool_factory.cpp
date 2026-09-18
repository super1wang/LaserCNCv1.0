#include "tool_factory.h"

#include "core/logging/logger.h"

#include <boost/lexical_cast.hpp>
#include <mutex>
#include <set>

namespace {
std::mutex toolSnapshotMutex;
std::set<int> explicitRecipeIndices;
}

map<int,Tool> ToolFactory::m_mapTools;
ToolFactory::ToolFactory()
{
	std::lock_guard<std::mutex> lock(toolSnapshotMutex);
	m_mapTools.clear();
	explicitRecipeIndices.clear();
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

void ToolFactory::SetTool(int i, Tool tool, bool explicitRecipe)
{
	std::lock_guard<std::mutex> lock(toolSnapshotMutex);
	m_mapTools.insert_or_assign(i, std::move(tool));
	if (explicitRecipe) explicitRecipeIndices.insert(i);
	else explicitRecipeIndices.erase(i);
}

QHash<QString, Tool> ToolFactory::executionRecipes()
{
	std::lock_guard<std::mutex> lock(toolSnapshotMutex);
	QHash<QString, Tool> result;
	for (const int index : explicitRecipeIndices) {
		const auto& tool = m_mapTools.at(index);
		const QString name = QString::fromStdString(tool.m_strName);
		// Ambiguous names cannot be resolved by arbitrary map order.
		if (result.contains(name)) return {};
		result.insert(name, tool);
	}
	return result;
}
	
void ToolFactory::ToolClear()
{
	std::lock_guard<std::mutex> lock(toolSnapshotMutex);
	m_mapTools.clear();
	explicitRecipeIndices.clear();
}

QStringList ToolFactory::toolNames()
{
	std::lock_guard<std::mutex> lock(toolSnapshotMutex);
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
	std::lock_guard<std::mutex> lock(toolSnapshotMutex);
	::toml::table root;
	for (const auto& [index, tool] : m_mapTools) {
		auto table = tool.toTable();
		table["_lcncExplicitRecipe"] = explicitRecipeIndices.count(index) != 0;
		root[std::to_string(index)] = std::move(table);
	}
	return root;
}

bool ToolFactory::restoreSnapshot(const ::toml::table& snapshot)
{
	map<int, Tool> restored;
	std::set<int> restoredExplicit;
	for (const auto& [key, value] : snapshot) {
		if (!value.is_table())
			return false;
		try {
			Tool tool;
			tool.SetFromTable(value.as_table());
			const int index = std::stoi(key);
			restored.emplace(index, std::move(tool));
			// Historical embedded tools are explicit project snapshots. New
			// compatibility defaults retain their non-executable provenance.
			if (!value.contains("_lcncExplicitRecipe") || value.at("_lcncExplicitRecipe").as_boolean())
				restoredExplicit.insert(index);
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
	std::lock_guard<std::mutex> lock(toolSnapshotMutex);
	m_mapTools = std::move(restored);
	explicitRecipeIndices = std::move(restoredExplicit);
	return true;
}

