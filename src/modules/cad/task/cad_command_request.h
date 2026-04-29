#pragma once

#include "modules/cad/selection/cad_selection.h"

#include <QVariantMap>

namespace lcnc::cad::task {

/// Parameterized CAD command invocation used by TaskPanel preview/apply.
struct CadCommandRequest {
    selection::CadSelectionContext selection;
    QVariantMap params;
    bool preview{false};
};

} // namespace lcnc::cad::task