#include <QStringList>
#include <QtWidgets>

#include "treemodel.h"
#include "treeitem.h"
#include "Process_TreeView.h"

TreeModel::TreeModel(const QStringList& headers, QObject* parent)
	: QAbstractItemModel(parent)
{
	QVector<QVariant> rootData;
	foreach(QString header, headers)
		rootData << header;

	rootItem = new TreeItem(rootData);
}

TreeModel::~TreeModel()
{
    delete rootItem;
}

QVariant TreeModel::headerData(int section, Qt::Orientation orientation, int role) const
{
	if (orientation == Qt::Horizontal && role == Qt::DisplayRole)
	{
		if (rootItem && section >= 0 && section < rootItem->columnCount())
		{
			return rootItem->data(section);
		}
	}
	return QVariant();
}

QVariant TreeModel::data(const QModelIndex& index, int role) const
{
	if (!index.isValid())
		return QVariant();

	if (role != Qt::DisplayRole && role != Qt::EditRole)
		return QVariant();

	TreeItem* item = itemFromIndex(index);

	return item->data(index.column());
}

QModelIndex TreeModel::index(int row, int column, const QModelIndex& parent) const
{
	if (parent.isValid() && parent.column() != 0)
		return QModelIndex();

	TreeItem* parentItem = itemFromIndex(parent);
	if (!parentItem)
		return QModelIndex();

	TreeItem* childItem = parentItem->child(row);
	if (childItem)
		return createIndex(row, column, childItem);
	return QModelIndex();
}

QModelIndex TreeModel::parent(const QModelIndex& index) const
{
	if (!index.isValid())
		return QModelIndex();

	TreeItem* childItem = static_cast<TreeItem*>(index.internalPointer());
	TreeItem* parentItem = childItem->parentItem();

	if (parentItem == rootItem)
		return QModelIndex();

	return createIndex(parentItem->row(), 0, parentItem);
}

int TreeModel::rowCount(const QModelIndex& parent) const
{
	if (parent.isValid() && parent.column() > 0)
		return 0;

	const TreeItem* parentItem = itemFromIndex(parent);

	return parentItem ? parentItem->childCount() : 0;
}

int TreeModel::columnCount(const QModelIndex& parent) const
{
	Q_UNUSED(parent);
	return rootItem->columnCount();
}

bool TreeModel::insertRow(int row, TreeItem* item, const QModelIndex& parent)
{
	TreeItem* parentItem = parent.isValid() ? itemFromIndex(parent) : rootItem;

	const int childCount = parentItem->childCount();
	row = qBound(0, row, childCount);

	beginInsertRows(parent, row, row);
	const bool success = parentItem->insertChild(row, item);
	endInsertRows();
	return success;
}

bool TreeModel::appendRow(TreeItem* item, const QModelIndex& parent)
{
	TreeItem* parentItem = parent.isValid() ? itemFromIndex(parent) : rootItem;
	if (!parentItem || !item)
		return false;

	return insertRow(parentItem->childCount(), item, parent);
}

bool TreeModel::removeRow(const QModelIndex& index)
{
	return removeRows(index.row(), 1, index.parent());
}

bool TreeModel::removeRows(int position, int rows, const QModelIndex& parent)
{
	TreeItem* parentItem = itemFromIndex(parent);
	if (!parentItem)
		return false;
	if (rows <= 0 || position < 0 || position + rows > parentItem->childCount())
		return false;

	beginRemoveRows(parent, position, position + rows - 1);
	const bool success = parentItem->removeChildren(position, rows);
	endRemoveRows();

	return success;
}

TreeItem* TreeModel::itemFromIndex(const QModelIndex& index) const
{
	if (index.isValid())
	{
		TreeItem* item = static_cast<TreeItem*>(index.internalPointer());
		if (item)
			return item;
	}
	return rootItem;
}

QModelIndex TreeModel::indexFromItem(const TreeItem* item) const
{
	if (item && item->parent())
		return createIndex(item->row(), 0, const_cast<TreeItem*>(item));
    return QModelIndex();
}
