#pragma once 

#include "CompDevice.h"

void CompDevice::SetCompValue(const QString& qstrName, const CompValue& value)
{
	map_Comps[qstrName] = value;
}

bool CompDevice::GetCompValue(const QString& qstrName, CompValue& value)
{
	if (!map_Comps.count(qstrName))
		return false;

	value = map_Comps[qstrName];
	return true;
}

bool CompDevice::IsCompIndex(const QString& qstrName)
{
	return map_Comps.count(qstrName);
}

QStringList CompDevice::GetCompList()
{
	QStringList qlist;
	for (const auto& pair : map_Comps)
	{
		qlist.append(pair.first);
	}
	return qlist;
}

map<QString, CompValue>	CompDevice::GetComps()
{
	return map_Comps;
}

void CompDevice::CleanComps()
{
	map_Comps.clear();
}