#pragma once

#include "modules/cad/task/icad_tool_command.h"

#include <QHash>
#include <memory>

class CadModule;

namespace lcnc::cad::task {

/// Dispatches TaskPanel parameter requests to registered CAD tool commands.
class CadCommandDispatcher
{
public:
    /// Create a dispatcher bound to the CAD facade that owns document mutation.
    explicit CadCommandDispatcher(CadModule* cadModule);

    /// Register the built-in primitive and sketch-feature commands.
    void registerDefaultTools();

    /// Register or replace one tool command by descriptor tool id.
    void registerCommand(std::unique_ptr<ICadToolCommand> command);

    /// Build a transient preview shape for a tool id.
    bool preview(const QString& toolId,
                 const CadCommandRequest& request,
                 TopoDS_Shape* outShape,
                 QString* errMsg);

    /// Execute a tool id and commit its effects.
    bool execute(const QString& toolId,
                 const CadCommandRequest& request,
                 QString* errMsg);

private:
    CadModule* m_cadModule{nullptr};
    QHash<QString, std::shared_ptr<ICadToolCommand>> m_commands;
};

} // namespace lcnc::cad::task