#pragma once

#include "app/project_explorer_model.h"

class QTreeWidget;

namespace lcnc::app {

class ProjectExplorerController
{
public:
    explicit ProjectExplorerController(QTreeWidget* tree);

    void rebuild(const ProjectExplorerSnapshot& snapshot);

private:
    QTreeWidget* m_tree{nullptr};
};

} // namespace lcnc::app
