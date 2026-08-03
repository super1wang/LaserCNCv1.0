#pragma once

#include <QString>

#include <functional>

class CadModule;
class WidgetOccView;

namespace lcnc::cad::ui {
class WidgetCadTaskPanel;
}

namespace lcnc::app {

/**
 * @brief Owns CAD task-panel projections and viewport-only interactions.
 *
 * The controller deliberately keeps command execution in the command layer.
 * It translates the panel's immutable CAD snapshots into widget/overlay state
 * and routes preview and sketch-overlay gestures back to the CAD facade.
 */
class CadTaskPanelController
{
public:
    using ViewProvider = std::function<WidgetOccView*()>;
    using ActivePredicate = std::function<bool()>;

    CadTaskPanelController(lcnc::cad::ui::WidgetCadTaskPanel* panel,
                           CadModule* cad,
                           ViewProvider viewProvider,
                           ActivePredicate cadContextActive);

    void updatePrimitivePreview();
    void updateFeaturePreview();
    void updateTransformPreview();
    void refreshPanelState();
    void refreshSketchElements();
    void refreshFinishedSketches();
    void updateSketchOverlay();
    void handleSketchOverlayPicked(const QString& key);
    void handleSketchOverlayDrag(const QString& key, double deltaX, double deltaY);

private:
    [[nodiscard]] WidgetOccView* view() const;
    [[nodiscard]] bool isCadContextActive() const;

    lcnc::cad::ui::WidgetCadTaskPanel* m_panel{nullptr};
    CadModule* m_cad{nullptr};
    ViewProvider m_viewProvider;
    ActivePredicate m_cadContextActive;
};

} // namespace lcnc::app
