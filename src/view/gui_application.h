#pragma once

#include <QObject>
#include <QHash>

#include "core/project/project_types.h"
#include "core/kinematics/machine_kinematics.h"
#include "core/settings/app_settings.h"
#include "view/rendering_manager.h"

class GuiDocument;

/**
 * @brief GUI workspace manager.
 *
 * Owns the GuiDocument map for open project workspaces. Project data remains
 * in LcncProjectManager; this object manages display document/context
 * lifecycle and render settings per workspace.
 */
class GuiApplication : public QObject
{
    Q_OBJECT
public:
    explicit GuiApplication(QObject* parent = nullptr);
    ~GuiApplication() override;

    GuiDocument* activeGuiDocument() const;
    GuiDocument* guiDocument(ProjectWorkspaceId id) const;
    GuiDocument* ensureGuiDocument(ProjectWorkspaceId id);
    void closeGuiDocument(ProjectWorkspaceId id);
    void resetWorkspaceGuiDocument();

    int  currentDisplayMode() const { return m_currentDisplayMode; }
    bool currentFaceBoundaryDraw() const { return m_currentFaceBoundary; }
    void setCurrentDisplayMode(int displayMode, bool faceBoundary);
    /// Push the machine-coordinate frame to every workspace view.
    void setMachineCoordinateFrame(const QList<MachineAxisDef>& axes);

    void requestApplyRenderingSettings(const lcnc::RenderProfileSettings& cadProfile,
                                       const lcnc::RenderProfileSettings& camProfile,
                                       const lcnc::ColorSettings& colors,
                                       lcnc::view::RenderDirtyFlags dirtyFlags,
                                       bool applyCadViews,
                                       bool applyCamView);

signals:
    void guiDocumentAboutToClose(ProjectWorkspaceId id, GuiDocument* document);
    void guiDocumentReady(ProjectWorkspaceId id, GuiDocument* document);
    void activeWorkspaceDocumentChanged(ProjectWorkspaceId id, GuiDocument* document);
    void activeGuiDocumentAboutToClose(GuiDocument* document);
    void activeGuiDocumentReady();
    void activeGuiDocumentChanged(GuiDocument* document);

private:
    static GuiApplication* s_instance;

    GuiDocument* createWorkspaceGuiDocument();
    void setActiveWorkspace(ProjectWorkspaceId id);
    void applyDisplayModeToDocument(GuiDocument* document) const;

    QHash<ProjectWorkspaceId, GuiDocument*> m_guiDocuments;
    ProjectWorkspaceId m_activeWorkspaceId{kInvalidProjectWorkspaceId};
    int  m_currentDisplayMode{1};
    bool m_currentFaceBoundary{false};
    QList<MachineAxisDef> m_machineCoordinateAxes;
};
