#pragma once

#include "app/project_explorer_model.h"

#include <Qt>

class QTreeWidget;
class QTreeWidgetItem;

namespace lcnc::app {

namespace ProjectExplorerRoles {
inline constexpr int DocId = Qt::UserRole + 1;
inline constexpr int Entry = Qt::UserRole + 2;
inline constexpr int NodeKey = Qt::UserRole + 3;
inline constexpr int LeafEntries = Qt::UserRole + 4;
inline constexpr int NodeKind = Qt::UserRole + 5;
inline constexpr int ContourIndex = Qt::UserRole + 6;
inline constexpr int ContourId = Qt::UserRole + 7;
inline constexpr int LayerId = Qt::UserRole + 8;
inline constexpr int MachiningFaceId = Qt::UserRole + 9;
} // namespace ProjectExplorerRoles

ProjectExplorerNodeKind projectNodeKind(const QTreeWidgetItem* item);
void populateProjectExplorerTree(QTreeWidget* tree, const ProjectExplorerSnapshot& snapshot);

} // namespace lcnc::app
