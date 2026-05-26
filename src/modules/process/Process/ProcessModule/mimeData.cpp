#include "mimeData.h"

ProcessMimeData::ProcessMimeData(TreeItem* item)
{
	m_treeItem = item;
}

ProcessMimeData::~ProcessMimeData(void)
{
}

ProcessMimeData* ProcessMimeData::clone(TreeItem* item) const
{
	if (!item)
	{
		return nullptr;
	}

	TreeItem* cloneItem = item->clone();
	if (!cloneItem)
	{
		return nullptr;
	}

	ProcessMimeData* mimeData = new ProcessMimeData(cloneItem);
	return mimeData;
}
