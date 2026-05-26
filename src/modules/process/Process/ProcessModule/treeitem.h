#ifndef TREEITEM_H
#define TREEITEM_H

#include <QList>
#include <QVariant>
#include <QVector>
#include <QTreeView>
#include "DataType.h"

class TreeItem : public QObject
{
    Q_OBJECT
        Q_ENUMS(ItemState)
        Q_PROPERTY(ItemState state READ GetState WRITE SetState)

public:
    explicit                TreeItem(TreeItem* parent = 0);
    explicit                TreeItem(const QString& text, TreeItem* parent = 0);
    explicit                TreeItem(const QVector<QVariant>& data, TreeItem* parentItem = nullptr);
    ~TreeItem();

    TreeItem* parentItem();
    TreeItem* parent() const;
    TreeItem* child(int row);

    bool                    appendChild(TreeItem* child);
    bool                    insertChild(int row, TreeItem* child);
    bool                    removeChild(TreeItem* child);

    int                     childCount() const;
    int                     columnCount() const;
    bool                    insertChildren(int position, int count, int columns);
    bool                    insertColumns(int position, int columns);
    bool                    removeChildren(int position, int count);
    bool                    removeColumns(int position, int columns);

    QVariant                data(int column) const;
    bool                    setData(int column, const QVariant& value);

    int                     row() const;
    void                    setRow(int row);

    virtual TreeItem*                   clone() const;
    virtual void                        Edit();
    virtual void						UpdateInfo();
    virtual void						SwitchState(ItemState state = ItemState::StateSave);
    virtual void                        SetState(ItemState state);
    virtual void                        SetMaps(map<QString, QString> maps);
    virtual ItemType                    GetType();
    virtual ItemState                   GetState();
    virtual map<QString, QString>       GetMaps();

    ItemType                            type()  { return m_type; }
    ItemState                           state() { return m_state; }
    map<QString, QString>               maps()  { return m_maps; }

	QList<TreeItem*>                    m_childItems;

 public:
    QVector<QVariant>                   m_itemData;
    TreeItem*                           m_parentItem;
    ItemType                            m_type;             //类型
    ItemState                           m_state;            //状态
    ItemState				            m_stateSave;        //状态备份
	map<QString, QString>               m_maps;             //参数
    
    // 语言状态管理
    static bool                         m_bChinese;
    static void                         SetChinese(bool bChinese) { m_bChinese = bChinese; }
    static bool                         IsChinese() { return m_bChinese; }
};

#endif // TREEITEM_H