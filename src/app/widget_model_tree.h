#pragma once

#include <QWidget>
#include <QTreeWidget>
#include "base/lcnc_application.h"
#include "base/lcnc_document.h"

/**
 * @brief Left-panel "准备" tab: displays the document entity tree.
 *
 * Shows two top-level groups — Machine Model and Workpiece Model — driven
 * by the LcncDocument's XDE label tree.  Selecting an item in the tree
 * highlights the corresponding shape in the 3D view.
 */
class WidgetModelTree : public QWidget
{
    Q_OBJECT
public:
    explicit WidgetModelTree(QWidget* parent = nullptr);

    void rebuildForDocument(LcncDocument* doc);
    void clear();

signals:
    void entitySelected(const QString& labelEntry);

private slots:
    void onItemSelectionChanged();
    void onItemDoubleClicked(QTreeWidgetItem* item, int column);

private:
    void populateGroup(QTreeWidgetItem* groupItem,
                       LcncDocument*   doc,
                       LcncDocument::EntityKind kind);

    QTreeWidget* m_tree{nullptr};
};
