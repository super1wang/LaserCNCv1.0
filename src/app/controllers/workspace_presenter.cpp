#include "app/controllers/workspace_presenter.h"

#include <QStackedWidget>
#include <QWidget>

namespace lcnc::app {

WorkspacePresenter::WorkspacePresenter(QStackedWidget* stack, QWidget* defaultView)
    : m_stack(stack)
    , m_defaultView(defaultView)
{
}

QWidget* WorkspacePresenter::view(ProjectWorkspaceId id) const
{
    return m_views.value(id, nullptr);
}

QList<QWidget*> WorkspacePresenter::views() const
{
    return m_views.values();
}

void WorkspacePresenter::registerView(ProjectWorkspaceId id, QWidget* view)
{
    if (!m_stack || !view || id == kInvalidProjectWorkspaceId)
        return;
    if (QWidget* previous = m_views.value(id, nullptr); previous && previous != view)
        m_stack->removeWidget(previous);
    m_views.insert(id, view);
    if (m_stack->indexOf(view) < 0)
        m_stack->addWidget(view);
}

bool WorkspacePresenter::activate(ProjectWorkspaceId id)
{
    QWidget* workspaceView = view(id);
    if (!m_stack || !workspaceView)
        return false;
    m_stack->setCurrentWidget(workspaceView);
    return true;
}

QWidget* WorkspacePresenter::take(ProjectWorkspaceId id)
{
    QWidget* workspaceView = m_views.take(id);
    if (workspaceView && m_stack)
        m_stack->removeWidget(workspaceView);
    return workspaceView;
}

void WorkspacePresenter::showDefault()
{
    if (m_stack && m_defaultView)
        m_stack->setCurrentWidget(m_defaultView);
}

} // namespace lcnc::app
