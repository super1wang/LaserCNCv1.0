#pragma once

#include <QObject>

#include "core/settings/app_settings.h"
#include "view/rendering_manager.h"

class GuiDocument;

/**
 * @brief GUI workspace manager.
 *
 * Owns the active GuiDocument used by the current project workspace. Project
 * data remains in LcncProjectManager; this object manages view/context
 * lifecycle and render settings for the active workspace.
 */
class GuiApplication : public QObject
{
    Q_OBJECT
public:
    explicit GuiApplication(QObject* parent = nullptr);
    ~GuiApplication() override;

    GuiDocument* workspaceGuiDocument() const { return m_workspaceGuiDocument; }
    void resetWorkspaceGuiDocument();

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
    void workspaceGuiDocumentAboutToClose(GuiDocument* document);
    void workspaceGuiDocumentReady();
    void workspaceGuiDocumentChanged(GuiDocument* document);

private:
    static GuiApplication* s_instance;

    GuiDocument* createWorkspaceGuiDocument();

    GuiDocument* m_workspaceGuiDocument{nullptr};
    int  m_currentDisplayMode{1};
    bool m_currentFaceBoundary{false};
};
