#pragma once

#include "modules/cad/task/cad_command_request.h"
#include "modules/cad/task/cad_tool_descriptor.h"

#include <QString>

class TopoDS_Shape;

namespace lcnc::cad::task {

/// Parameterized CAD tool command with optional preview support.
class ICadToolCommand
{
public:
    virtual ~ICadToolCommand() = default;

    /// Descriptor that exposes this tool to UI registries.
    virtual CadToolDescriptor descriptor() const = 0;

    /// Build a preview shape. Returns false when the tool has no preview or validation fails.
    virtual bool preview(const CadCommandRequest& request,
                         TopoDS_Shape* outShape,
                         QString* errMsg) = 0;

    /// Execute the tool and commit document mutations.
    virtual bool execute(const CadCommandRequest& request, QString* errMsg) = 0;
};

} // namespace lcnc::cad::task