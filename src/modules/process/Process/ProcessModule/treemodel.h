#ifndef TREEMODEL_H
#define TREEMODEL_H

#include <QAbstractItemModel>
#include <QModelIndex>
#include <QVariant>
#include <QMutex>

class TreeItem;

class TreeModel : public QAbstractItemModel
{
    Q_OBJECT

public:
    TreeModel(const QStringList& headers, QObject* parent);
    ~TreeModel();

    QVariant        headerData(int section, Qt::Orientation orientation, int role) const;
	QVariant        data(const QModelIndex& index, int role) const override;
    QModelIndex     index(int row, int column, const QModelIndex &parent = QModelIndex()) const override;
    QModelIndex     parent(const QModelIndex &index) const override;
    int             rowCount(const QModelIndex &parent = QModelIndex()) const override;
	int             columnCount(const QModelIndex& parent = QModelIndex()) const override;

    bool            insertRow(int row, TreeItem* item, const QModelIndex& parent = QModelIndex());
    bool            appendRow(TreeItem* item, const QModelIndex& parent = QModelIndex());
    bool            removeRow(const QModelIndex& index);
    bool            removeRows(int position, int rows, const QModelIndex& parent = QModelIndex());
    //bool            moveRow(TreeItem* item, int torow);

    TreeItem*       itemFromIndex(const QModelIndex& index) const;
    QModelIndex     indexFromItem(const TreeItem* item) const;

    TreeItem*       root() { return rootItem; }

private:
    TreeItem        *rootItem;
};
//! [0]

#endif // TREEMODEL_H
