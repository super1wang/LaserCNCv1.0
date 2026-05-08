#pragma once

#include <QFlags>
#include <QMetaType>

// Stable identifier for OCC-backed domain documents inside one project session.
using DocumentId = int;
constexpr DocumentId kInvalidDocumentId = -1;

namespace lcnc {

/**
 * @brief Stable project data domains owned by one LaserCNC project session.
 */
enum class ProjectDomain {
    Project = 0,
    Workpiece,
    Machine,
    Cam
};

/**
 * @brief Dirty regions tracked by the project manager.
 */
enum class ProjectDirtyFlag {
    None      = 0x0,
    Project   = 0x1,
    Workpiece = 0x2,
    Machine   = 0x4,
    Cam       = 0x8
};

Q_DECLARE_FLAGS(ProjectDirtyFlags, ProjectDirtyFlag)

} // namespace lcnc

Q_DECLARE_OPERATORS_FOR_FLAGS(lcnc::ProjectDirtyFlags)
Q_DECLARE_METATYPE(lcnc::ProjectDomain)
