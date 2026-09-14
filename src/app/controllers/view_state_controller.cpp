#include "app/controllers/view_state_controller.h"

#include <utility>

namespace lcnc::app {

ViewStateController::ViewStateController(lcnc::AppSettings& settings,
                                         PersistCallback persist)
    : m_settings(settings)
    , m_persist(std::move(persist))
{
}

lcnc::ViewStateSettings ViewStateController::state() const
{
    return m_settings.viewState;
}

bool ViewStateController::persistDisplayMode(int displayMode, bool faceBoundary)
{
    m_settings.viewState.displayMode = displayMode;
    m_settings.viewState.faceBoundary = faceBoundary;
    return !m_persist || m_persist();
}

bool ViewStateController::persistToggles(bool worldAxesVisible,
                                         bool rotaryAxisGuidesVisible,
                                         bool cutterHeadGuideVisible)
{
    m_settings.viewState.worldAxesVisible = worldAxesVisible;
    m_settings.viewState.rotaryAxisGuidesVisible = rotaryAxisGuidesVisible;
    m_settings.viewState.cutterHeadGuideVisible = cutterHeadGuideVisible;
    // The machine model is a reference workspace and never persists as active.
    m_settings.viewState.machineModelVisible = false;
    return !m_persist || m_persist();
}

} // namespace lcnc::app
