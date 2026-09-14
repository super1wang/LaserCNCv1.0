#pragma once

#include "core/settings/app_settings.h"

#include <functional>

namespace lcnc::app {

class ViewStateController
{
public:
    using PersistCallback = std::function<bool()>;

    ViewStateController(lcnc::AppSettings& settings,
                        PersistCallback persist);

    [[nodiscard]] lcnc::ViewStateSettings state() const;
    bool persistDisplayMode(int displayMode, bool faceBoundary);
    bool persistToggles(bool worldAxesVisible,
                        bool rotaryAxisGuidesVisible,
                        bool cutterHeadGuideVisible);

private:
    lcnc::AppSettings& m_settings;
    PersistCallback m_persist;
};

} // namespace lcnc::app
