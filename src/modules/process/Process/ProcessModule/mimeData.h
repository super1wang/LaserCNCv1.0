#pragma once
#include "qmimedata.h"
#include "treeitem.h"

class ProcessMimeData :
	public QMimeData
{
	Q_OBJECT
public:
	ProcessMimeData(TreeItem* item);
	~ProcessMimeData(void);

public:
	TreeItem* treeItem() const { return m_treeItem; }
	ProcessMimeData* clone(TreeItem* item) const;


private:
	TreeItem* m_treeItem;  
};

