#pragma once

#include <QByteArray>
#include <QString>
#include <optional>

class Tool;

namespace lcnc::process {

struct FrozenToolExecutionRecipe {
    QString toolName;
    QString sourceId;
    QString profileSchema{QStringLiteral("legacy-configured-units-v1")};
#define LCNC_EXECUTION_FIELD(type, name, legacy) type name{};
#include "modules/process/runtime/frozen_tool_execution_fields.inc"
#undef LCNC_EXECUTION_FIELD
};

// Only the owner-thread capture boundary may interpret legacy Tool fields.
// Trajectory-changing/native special modes must be neutral, including inactive
// numeric values. S2 consumes this DTO and must qualify units/profile support.
std::optional<FrozenToolExecutionRecipe> freezeToolExecutionRecipe(
    const Tool& tool, const QString& sourceId, QString* error);
QByteArray frozenToolExecutionRecipeHash(const FrozenToolExecutionRecipe& recipe);
bool validateToolExecutionRecipe(const FrozenToolExecutionRecipe& recipe,
                                 bool realMachine, QString* error);

} // namespace lcnc::process
