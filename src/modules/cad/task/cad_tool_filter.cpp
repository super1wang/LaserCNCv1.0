#include "modules/cad/task/cad_tool_filter.h"

namespace lcnc::cad::task {

bool CadToolFilter::isToolAvailable(
    const CadToolDescriptor& tool,
    const lcnc::cad::selection::CadSelectionContext& context)
{
    if (!context.hasDocument && !tool.showWhenNoDocument)
        return false;
    if (context.hasDocument && !tool.showWhenHasDocument)
        return false;
    if (context.sketchEditing
        && tool.toolId != QStringLiteral("cad.sketch.begin")
        && !tool.requiresSketchEditing) {
        return false;
    }
    if (tool.requiresSketchEditing && !context.sketchEditing)
        return false;
    if (tool.requiresSelectedSketch && !context.hasSelectedSketch)
        return false;
    if (context.selectedShapeCount < tool.minSelectedShapes)
        return false;
    if (tool.maxSelectedShapes >= 0 && context.selectedShapeCount > tool.maxSelectedShapes)
        return false;
    if (!tool.acceptedDomains.isEmpty()) {
        bool hasAcceptedDomain = false;
        for (const auto& item : context.items) {
            if (tool.acceptedDomains.contains(item.domain)) {
                hasAcceptedDomain = true;
                break;
            }
        }
        if (!hasAcceptedDomain)
            return false;
    }
    return true;
}

QVector<CadToolDescriptor> CadToolFilter::availableTools(
    const CadToolRegistry& registry,
    const lcnc::cad::selection::CadSelectionContext& context)
{
    QVector<CadToolDescriptor> result;
    for (const auto& tool : registry.tools()) {
        if (isToolAvailable(tool, context))
            result.append(tool);
    }
    return result;
}

} // namespace lcnc::cad::task