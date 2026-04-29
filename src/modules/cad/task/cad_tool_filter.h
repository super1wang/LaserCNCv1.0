#pragma once

#include "modules/cad/selection/cad_selection.h"
#include "modules/cad/task/cad_tool_descriptor.h"
#include "modules/cad/task/cad_tool_registry.h"

#include <QVector>

namespace lcnc::cad::task {

/// Filters registered CAD tools for a TaskPanel selection context.
class CadToolFilter
{
public:
    /// Return true if a single tool can be shown for the current context.
    static bool isToolAvailable(const CadToolDescriptor& tool,
                                const lcnc::cad::selection::CadSelectionContext& context);

    /// Return all visible tools for the current context.
    static QVector<CadToolDescriptor> availableTools(
        const CadToolRegistry& registry,
        const lcnc::cad::selection::CadSelectionContext& context);
};

} // namespace lcnc::cad::task