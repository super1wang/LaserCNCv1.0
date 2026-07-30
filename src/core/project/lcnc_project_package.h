#pragma once

#include "core/project/lcnc_project_manifest.h"
#include "core/project/project_package_extension.h"
#include "core/project/project_save_options.h"

#include <QString>

class LcncDocument;

namespace lcnc::cam { class CamDataManager; }

namespace lcnc {

/**
 * @brief Result metadata returned after loading a .lcnc package.
 */
struct ProjectLoadResult {
    LcncProjectManifest manifest;
    QString packagePath;
    QString documentName;
};

/**
 * @brief Reader/writer for the LaserCNC .lcnc zip package.
 */
class LcncProjectPackage
{
public:
    /// Register module-owned project data hooks. The hooks run inside the core
    /// staging transaction and must not retain staging paths after returning.
    static void setExtension(ProjectPackageExtension extension);
     /// Returns true for .lcnc package files, package directories, and project.toml paths.
    static bool isProjectPath(const QString& path);

     /// Normalize either foo.lcnc or foo.lcnc/project.toml to the logical package path.
    static QString packageDirectory(const QString& path);

    static QString manifestPath(const QString& path);
    static QString workpieceXcafPath(const QString& path,
                                     const LcncProjectManifest& manifest = LcncProjectManifest{});

    static bool save(const LcncDocument& document,
                     const QString& path,
                     const ProjectSaveOptions& options,
                     QString* errorMsg = nullptr);

    static bool save(const LcncDocument& workpieceDocument,
                     const LcncDocument* machineDocument,
                     const LcncDocument* camDocument,
                     const QString& path,
                     const ProjectSaveOptions& options,
                     QString* errorMsg = nullptr);

    /// Save using project/session metadata supplied by LcncProjectManager.
    /// When @p camData is non-null, the project-core CAM data is written **inside**
    /// the same staging dir (so it ends up inside the .lcnc archive) — one transaction.
    static bool save(const LcncDocument& workpieceDocument,
                     const LcncDocument* machineDocument,
                     const LcncDocument* camDocument,
                     const QString& path,
                     const LcncProjectManifest& manifestTemplate,
                     const ProjectSaveOptions& options,
                     LcncProjectManifest* savedManifest = nullptr,
                     QString* errorMsg = nullptr,
                     lcnc::cam::CamDataManager* camData = nullptr);

    static bool load(LcncDocument& document,
                     const QString& path,
                     ProjectLoadResult* result = nullptr,
                     QString* errorMsg = nullptr);

    /// When @p camData is non-null, the project-core CAM data is read from the same
    /// extraction dir (inside the .lcnc archive) before it is torn down — one transaction.
    static bool load(LcncDocument& workpieceDocument,
                     LcncDocument* machineDocument,
                     LcncDocument* camDocument,
                     const QString& path,
                     ProjectLoadResult* result = nullptr,
                     QString* errorMsg = nullptr,
                     lcnc::cam::CamDataManager* camData = nullptr);

private:
    static bool loadInternal(LcncDocument& workpieceDocument,
                             LcncDocument* machineDocument,
                             LcncDocument* camDocument,
                             const QString& path,
                             ProjectLoadResult* result,
                             QString* errorMsg,
                             lcnc::cam::CamDataManager* camData);
};

} // namespace lcnc
