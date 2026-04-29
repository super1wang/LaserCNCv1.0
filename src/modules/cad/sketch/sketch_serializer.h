#pragma once

#include "modules/cad/services/sketch_types.h"

#include <QVariantMap>

namespace lcnc::cad {

/// Serializes sketch records to lightweight metadata snapshots for future persistence/undo.
class SketchSerializer
{
public:
    /// Convert a sketch record to a QVariantMap without embedding the cached OCC face.
    static QVariantMap toVariantMap(const SketchRecord& record);

    /// Restore scalar sketch metadata and elements from a QVariantMap.
    static SketchRecord fromVariantMap(const QVariantMap& map);
};

} // namespace lcnc::cad