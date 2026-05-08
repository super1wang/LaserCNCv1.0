#pragma once

#include <QObject>

#include "core/settings/app_settings.h"
#include "view/rendering_manager.h"

class GuiDocument;

/**
 * @brief GUI workspace manager.
 *
 * Owns the single visible GuiDocument used by all project domains. Project data
 * remains in LcncProjectManager; this object only manages view state and render
 * settings for the shared workspace.
 */
class GuiApplication : public QObject
{
    Q_OBJECT
public:
    explicit GuiApplication(QObject* parent = nullptr);
    ~GuiApplication() override;

    GuiDocument* workspaceGuiDocument() const { return m_workspaceGuiDocument; }

    int  currentDisplayMode() const { return m_currentDisplayMode; }
    bool currentFaceBoundaryDraw() const { return m_currentFaceBoundary; }
    void setCurrentDisplayMode(int displayMode, bool faceBoundary);

    void requestApplyRenderingSettings(const lcnc::RenderProfileSettings& cadProfile,
                                       const lcnc::RenderProfileSettings& camProfile,
                                       const lcnc::ColorSettings& colors,
                                       lcnc::view::RenderDirtyFlags dirtyFlags,
                                       bool applyCadViews,
                                       bool applyCamView);

signals:
    void workspaceGuiDocumentReady();

private:
    static GuiApplication* s_instance;

    GuiDocument* m_workspaceGuiDocument{nullptr};
    int  m_currentDisplayMode{1};
    bool m_currentFaceBoundary{false};
};
