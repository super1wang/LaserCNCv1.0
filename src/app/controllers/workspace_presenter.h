#pragma once

#include "core/project/project_types.h"

#include <QHash>
#include <QList>

class QStackedWidget;
class QWidget;

namespace lcnc::app {

class WorkspacePresenter
{
public:
    WorkspacePresenter(QStackedWidget* stack, QWidget* defaultView);

    [[nodiscard]] QWidget* view(ProjectWorkspaceId id) const;
    [[nodiscard]] QList<QWidget*> views() const;
    void registerView(ProjectWorkspaceId id, QWidget* view);
    bool activate(ProjectWorkspaceId id);
    [[nodiscard]] QWidget* take(ProjectWorkspaceId id);
    void showDefault();

private:
    QStackedWidget* m_stack{nullptr};
    QWidget* m_defaultView{nullptr};
    QHash<ProjectWorkspaceId, QWidget*> m_views;
};

} // namespace lcnc::app
