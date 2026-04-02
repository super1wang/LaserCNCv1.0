#pragma once

#include <QWidget>
#include <QTreeWidget>
#include <QStringList>
#include "base/lcnc_application.h"
#include "base/lcnc_document.h"
#include "base/machine_kinematics.h"

class WidgetModelTree : public QWidget
{
    Q_OBJECT
public:
    explicit WidgetModelTree(QWidget* parent = nullptr);

    void rebuildForDocument(LcncDocument* doc);
    void clear();
    LcncDocument* currentDocument() const { return m_doc; }

signals:
    void entitySelected(const QString& labelEntry);
    void selectionChanged(const QStringList& entries);
    /// Emitted after an axis node assignment is removed via context menu.
    void axisNodeUnassigned();
    /// Emitted when user toggles a node's checkbox.
    void visibilityChanged(const QString& entry, bool visible);

public slots:
    void highlightEntries(const QStringList& entries);

private slots:
    void onItemSelectionChanged();
    void onItemDoubleClicked(QTreeWidgetItem* item, int column);
    void onContextMenuRequested(const QPoint& pos);
    void onItemChanged(QTreeWidgetItem* item, int column);

private:
    void populateGroup(QTreeWidgetItem*   groupItem,
                       LcncDocument*      doc,
                       LcncDocument::EntityKind kind,
                       MachineKinematics* kin = nullptr);
    void populateMachineGroup(QTreeWidgetItem*   groupItem,
                               LcncDocument*      doc,
                               MachineKinematics* kin);
    void addTreeNodes(QTreeWidgetItem*                             parent,
                      const QList<LcncDocument::ShapeTreeNode>&    nodes,
                      LcncDocument::EntityKind                     kind,
                      MachineKinematics*                           kin);

    QTreeWidget*  m_tree{nullptr};
    LcncDocument* m_doc{nullptr};   ///< current document (set in rebuildForDocument)
    bool          m_blockItemChanged{false};
};
