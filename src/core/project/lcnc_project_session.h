#pragma once

#include "core/project/lcnc_project_manifest.h"
#include "core/project/project_save_options.h"
#include "core/project/project_types.h"

#include <QString>

class LcncDocument;

namespace lcnc {

/**
 * @brief Project-level identity and source data for the workpiece domain.
 */
struct WorkpieceProjectState {
    QString displayName;
    QString sourceFilePath;

    void clear();
};

/**
 * @brief Project-level state for CAM runtime and sparse OCC data.
 */
struct CamProjectState {
    bool hasRuntimeData{false};

    void clear();
};

/**
 * @brief Runtime aggregate for one LaserCNC project.
 *
 * The session owns no OCC documents. It binds the domain stores owned by
 * LcncProjectManager and keeps project-level metadata outside those stores.
 */
class LcncProjectSession
{
public:
    void bindDomainDocuments(LcncDocument* workpieceDocument,
                             LcncDocument* machineDocument,
                             LcncDocument* camDocument);

    LcncDocument* document(ProjectDomain domain) const;
    LcncDocument* workpieceDocument() const { return m_workpieceDocument; }
    LcncDocument* machineDocument() const { return m_machineDocument; }
    LcncDocument* camDocument() const { return m_camDocument; }

    QString projectName() const { return m_projectName; }
    void setProjectName(const QString& name) { m_projectName = name; }

    QString projectPath() const { return m_projectPath; }
    void setProjectPath(const QString& path) { m_projectPath = path; }

    LcncProjectManifest& manifest() { return m_manifest; }
    const LcncProjectManifest& manifest() const { return m_manifest; }
    void setManifest(const LcncProjectManifest& manifest) { m_manifest = manifest; }

    /// A project opened on a different machine may be inspected or simulated,
    /// but real machining must be gated until its configuration is reviewed.
    bool machineConfigurationCompatible() const { return m_machineConfigurationCompatible; }
    void setMachineConfigurationCompatible(bool compatible) { m_machineConfigurationCompatible = compatible; }

    ProjectSaveOptions& saveOptions() { return m_saveOptions; }
    const ProjectSaveOptions& saveOptions() const { return m_saveOptions; }
    void setSaveOptions(const ProjectSaveOptions& options) { m_saveOptions = options; }

    WorkpieceProjectState& workpiece() { return m_workpiece; }
    const WorkpieceProjectState& workpiece() const { return m_workpiece; }
    CamProjectState& cam() { return m_cam; }
    const CamProjectState& cam() const { return m_cam; }

    ProjectDirtyFlags dirtyFlags() const { return m_dirtyFlags; }
    bool isDirty() const { return m_dirtyFlags != ProjectDirtyFlags{}; }
    void markDirty(ProjectDomain domain);
    void clearDirty();

    void resetProjectState();

private:
    LcncDocument* m_workpieceDocument{nullptr};
    LcncDocument* m_machineDocument{nullptr};
    LcncDocument* m_camDocument{nullptr};

    QString m_projectName;
    QString m_projectPath;
    LcncProjectManifest m_manifest;
    ProjectSaveOptions m_saveOptions;
    bool m_machineConfigurationCompatible{true};

    WorkpieceProjectState m_workpiece;
    CamProjectState m_cam;
    ProjectDirtyFlags m_dirtyFlags;
};

ProjectDirtyFlag dirtyFlagForDomain(ProjectDomain domain);

} // namespace lcnc
