#pragma once

#include <QString>
#include <QVector>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>

#include "modules/cad/services/cad_modeling_session.h"

namespace lcnc::cad {

/// Visibility / usage state for a finished sketch record.
enum class SketchUsageState {
    Available = 0,   ///< Free to be used by features.
    UsedByFeature = 1 ///< Already consumed by an applied feature; auto-hidden.
};

/// One persisted sketch attached to a workpiece document.
struct SketchRecord {
    int id{0};                           ///< Manager-assigned positive id.
    QString name;                        ///< Display name (\"草图 N\").
    SketchPlaneKind plane{SketchPlaneKind::XY};
    QVector<SketchElement> elements;     ///< Geometry elements composing the sketch.
    TopoDS_Face profileFace;             ///< Cached face for feature builders.
    bool visible{true};                  ///< User-controlled visibility flag.
    SketchUsageState usage{SketchUsageState::Available};
    QString featureEntry;                ///< Entry of the feature that consumed it (if any).
};

} // namespace lcnc::cad
