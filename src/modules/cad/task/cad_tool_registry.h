#pragma once

#include "modules/cad/task/cad_tool_descriptor.h"

#include <QVector>

namespace lcnc::cad::task {

/// Registry of CAD TaskPanel tool descriptors.
class CadToolRegistry
{
public:
    /// Register a tool descriptor. Existing toolId entries are replaced.
    void registerTool(const CadToolDescriptor& descriptor);

    /// All registered tools sorted by category and order.
    QVector<CadToolDescriptor> tools() const;

    /// Tools belonging to one category, sorted by order.
    QVector<CadToolDescriptor> toolsByCategory(CadToolCategory category) const;

    /// Build the default CAD TaskPanel registry for existing commands.
    static CadToolRegistry createDefault();

private:
    QVector<CadToolDescriptor> m_tools;
};

} // namespace lcnc::cad::task