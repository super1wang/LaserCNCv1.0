#pragma once

#include <QString>

namespace lcnc::cam {

class CamDataManager;

/**
 * @brief Persistence for the project's CAM runtime data (core-owned).
 *
 * These free functions are the single reader/writer for the dense CAM toolpath
 * data (contours + layers + lead-ins + process parameters + sampled points +
 * signature tables). They operate purely on CamDataManager and perform no view
 * refresh — display updates remain a CAM-module concern. Invoked transactionally
 * from LcncProjectManager::saveProject / openProject so that the workpiece
 * geometry and CAM data are written/read together as one project.
 *
 * On-disk layout inside the package directory:
 *   - cam_toolpath.toml         (metadata: layers, contours, signature tables)
 *   - cam_toolpath_points.bin   (dense per-contour sampled points)
 */

/// True when both CAM toolpath cache files exist in @p packageDir.
bool hasCamToolpathCache(const QString& packageDir);

/// Write the CAM toolpath data to @p packageDir. No-op (returns true) when the
/// manager holds no toolpath.
bool saveCamToolpath(const CamDataManager& cam, const QString& packageDir, QString* errorMsg = nullptr);

/// Load CAM toolpath data from @p packageDir into @p cam. Returns false when the
/// cache files are absent (treated by callers as "no persisted toolpath").
bool loadCamToolpath(CamDataManager& cam, const QString& packageDir, QString* errorMsg = nullptr);

} // namespace lcnc::cam
