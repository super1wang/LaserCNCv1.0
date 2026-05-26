#pragma once

#include "treemodel.h"
#include "treeitem.h"
#include "toml.hpp"

#include <QTreeView>

#include <vector>

class QAction;
class QMenu;
class QPainter;
class QMainWindow;

class ProcessTreeView : public QTreeView
{
    Q_OBJECT
public:
    explicit ProcessTreeView(QWidget* parent = nullptr);
    ~ProcessTreeView() override = default;

    void setParent(QMainWindow* parentWindow);
    void CreateNewFileData();

    QModelIndex* GetCurrentModelIndex() { return &m_currentModelIndex; }
    TreeModel* GetModel() { return m_model; }

    QString GetIndexRelation(const QModelIndex& index);
    void Save(QString fileName = QString());
    bool SaveValue(toml::value& valueProcess);
    void Load(QString fileName = QString());
    bool LoadValue(const toml::value& valueProcess);
    void ViewportUpdate();

    QModelIndex ModelIndex(int parentIndex, int childIndex = -1);
    TreeItem* GetItem(int parentIndex, int childIndex = -1);
    std::vector<Item> GetTreeItemVector();
    int Count(int parentIndex = -1);

public slots:
    void OnCustomContextMenuRequested(QPoint pos);
    void OnTriggeredActionDelete();
    void OnTriggeredActionEnable();
    void OnTriggeredActionDisable();
    void OnTriggeredActionClear();
    void OnTriggeredActionStart();
    void OnTriggeredActionStop();
    void OnTriggeredActionWait();
    void OnTriggeredActionAxis();
    void OnTriggeredActionGroup();
    void OnTriggeredActionIf();
    void OnTriggeredActionLoop();
    void OnClickTreeView(const QModelIndex& index);

protected:
    void drawBranches(QPainter* painter, const QRect& rect, const QModelIndex& index) const override;
    void mouseDoubleClickEvent(QMouseEvent* event) override;

private:
    TreeItem* createItem(ItemType type, const QString& label) const;
    void insertDataItem(const QModelIndex& insertIndex, TreeItem* item);
    void addItem(ItemType type, const QString& label);

    QMenu* m_menuRight{nullptr};
    QMenu* m_menuAdd{nullptr};
    QAction* m_actionAdd{nullptr};
    QAction* m_actionDelete{nullptr};
    QAction* m_actionEnable{nullptr};
    QAction* m_actionDisable{nullptr};
    QAction* m_actionClear{nullptr};
    QAction* m_actionSave{nullptr};
    QAction* m_actionLoad{nullptr};
    QAction* m_actionStart{nullptr};
    QAction* m_actionStop{nullptr};
    QAction* m_actionWait{nullptr};
    QAction* m_actionAxis{nullptr};
    QAction* m_actionGroup{nullptr};
    QAction* m_actionIf{nullptr};
    QAction* m_actionLoop{nullptr};

    TreeModel* m_model{nullptr};
    QModelIndex m_currentModelIndex;
    QMainWindow* m_parentWindow{nullptr};
};
