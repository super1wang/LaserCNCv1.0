#include "app/controllers/project_explorer_controller.h"
#include "app/controllers/view_state_controller.h"
#include "app/controllers/workspace_presenter.h"
#include "app/project_explorer_tree_utils.h"

#include <QApplication>
#include <QByteArray>
#include <QTreeWidget>
#include <QStackedWidget>
#include <QWidget>

#include <cassert>

int main(int argc, char** argv)
{
    qputenv("QT_QPA_PLATFORM", QByteArrayLiteral("offscreen"));
    QApplication application(argc, argv);

    QTreeWidget tree;
    lcnc::app::ProjectExplorerController controller(&tree);

    lcnc::app::ProjectExplorerNode child;
    child.kind = lcnc::app::ProjectExplorerNodeKind::CadShape;
    child.nodeKey = QStringLiteral("shape:1");
    child.displayName = QStringLiteral("Shape");

    lcnc::app::ProjectExplorerNode root;
    root.kind = lcnc::app::ProjectExplorerNodeKind::WorkpieceRoot;
    root.nodeKey = QStringLiteral("root");
    root.displayName = QStringLiteral("Root");
    root.children.append(child);

    lcnc::app::ProjectExplorerSnapshot snapshot;
    snapshot.roots.append(root);
    controller.rebuild(snapshot);
    assert(tree.topLevelItemCount() == 1);
    QTreeWidgetItem* rootItem = tree.topLevelItem(0);
    rootItem->setExpanded(true);
    tree.setCurrentItem(rootItem->child(0));

    snapshot.roots[0].displayName = QStringLiteral("Renamed Root");
    snapshot.roots[0].children[0].displayName = QStringLiteral("Renamed Shape");
    controller.rebuild(snapshot);

    rootItem = tree.topLevelItem(0);
    assert(rootItem->isExpanded());
    assert(tree.currentItem());
    assert(tree.currentItem()->data(
               0, lcnc::app::ProjectExplorerRoles::NodeKey).toString()
           == QStringLiteral("shape:1"));
    assert(tree.currentItem()->text(0) == QStringLiteral("Renamed Shape"));

    lcnc::app::ProjectExplorerNode contourA;
    contourA.kind = lcnc::app::ProjectExplorerNodeKind::ToolpathContour;
    contourA.nodeKey = QStringLiteral("contour:11");
    contourA.displayName = QStringLiteral("Contour A");
    contourA.contourIndex = 0;
    contourA.contourId = 11;
    lcnc::app::ProjectExplorerNode contourB = contourA;
    contourB.nodeKey = QStringLiteral("contour:12");
    contourB.displayName = QStringLiteral("Contour B");
    contourB.contourIndex = 1;
    contourB.contourId = 12;
    lcnc::app::ProjectExplorerNode toolpathRoot;
    toolpathRoot.kind = lcnc::app::ProjectExplorerNodeKind::ToolpathRoot;
    toolpathRoot.nodeKey = QStringLiteral("toolpath");
    toolpathRoot.children = {contourA, contourB};
    snapshot.roots = {toolpathRoot};
    controller.rebuild(snapshot);

    const auto selectedContour = controller.selectContour(12, -1);
    assert(selectedContour && selectedContour->contourIndex == 1);
    const auto selectedContours = controller.selectContours({11, 12}, {});
    assert(selectedContours && selectedContours->contourId == 12);
    const auto order = controller.contourOrder();
    assert(order && order->indexes == QList<int>({0, 1}) && order->hasStableIds);
    controller.selectEntries(kInvalidDocumentId, {});
    assert(tree.selectedItems().isEmpty());

    lcnc::AppSettings settings;
    int persistCount = 0;
    lcnc::app::ViewStateController viewState(
        settings, [&persistCount] {
            ++persistCount;
            return true;
        });
    assert(viewState.persistDisplayMode(2, true));
    assert(viewState.persistToggles(true, false, true));
    const auto persisted = viewState.state();
    assert(persisted.displayMode == 2);
    assert(persisted.faceBoundary);
    assert(persisted.worldAxesVisible);
    assert(!persisted.rotaryAxisGuidesVisible);
    assert(persisted.cutterHeadGuideVisible);
    assert(!persisted.machineModelVisible);
    assert(persistCount == 2);

    QStackedWidget stack;
    QWidget defaultView;
    QWidget firstWorkspace;
    QWidget secondWorkspace;
    stack.addWidget(&defaultView);
    lcnc::app::WorkspacePresenter presenter(&stack, &defaultView);
    presenter.registerView(1, &firstWorkspace);
    presenter.registerView(2, &secondWorkspace);
    assert(presenter.activate(2));
    assert(stack.currentWidget() == &secondWorkspace);
    assert(presenter.take(2) == &secondWorkspace);
    assert(!presenter.activate(2));
    presenter.showDefault();
    assert(stack.currentWidget() == &defaultView);
    return 0;
}
