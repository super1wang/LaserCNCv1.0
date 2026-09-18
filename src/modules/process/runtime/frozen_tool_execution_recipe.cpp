#include "modules/process/runtime/frozen_tool_execution_recipe.h"

#include "modules/process/tool/tool.h"

#include <QCryptographicHash>
#include <QDataStream>
#include <QIODevice>
#include <cmath>

namespace lcnc::process {

QByteArray frozenToolExecutionRecipeHash(const FrozenToolExecutionRecipe& recipe)
{
    QByteArray bytes;
    QDataStream stream(&bytes, QIODevice::WriteOnly);
    stream.setVersion(QDataStream::Qt_6_0);
    stream << QByteArray("frozen-tool-execution-v1") << recipe.toolName
           << recipe.sourceId << recipe.profileSchema;
#define LCNC_EXECUTION_FIELD(type, name, legacy) stream << recipe.name;
#include "modules/process/runtime/frozen_tool_execution_fields.inc"
#undef LCNC_EXECUTION_FIELD
    return QCryptographicHash::hash(bytes, QCryptographicHash::Sha256);
}

bool validateToolExecutionRecipe(const FrozenToolExecutionRecipe& recipe,
                                 bool realMachine, QString* error)
{
    const auto reject = [&](const char* reason) {
        if (error) *error = QString::fromLatin1(reason);
        return false;
    };
    if (recipe.toolName.trimmed().isEmpty() || recipe.toolName == QStringLiteral("__fallback__")
        || recipe.sourceId.trimmed().isEmpty()
        || recipe.profileSchema != QStringLiteral("legacy-configured-units-v1"))
        return reject("Missing explicit tool/source or unsupported execution profile schema");
#define LCNC_EXECUTION_FIELD(type, name, legacy) \
    if (!std::isfinite(static_cast<double>(recipe.name))) \
        return reject("Non-finite tool execution field: " #name);
#include "modules/process/runtime/frozen_tool_execution_fields.inc"
#undef LCNC_EXECUTION_FIELD
    if (realMachine && (recipe.lineVelocity <= 0 || recipe.lineAcceleration <= 0 || recipe.lineJerk <= 0))
        return reject("Required tool feed/dynamics must be positive");
    if (recipe.beforeOn < 0 || recipe.afterOn < 0 || recipe.beforeOff < 0
        || recipe.afterOff < 0 || recipe.blowDelay < 0 || recipe.laserDelay < 0)
        return reject("Tool IO delays must be nonnegative");
    return true;
}

std::optional<FrozenToolExecutionRecipe> freezeToolExecutionRecipe(
    const Tool& tool, const QString& sourceId, QString* error)
{
    const auto reject = [&](const char* field) -> std::optional<FrozenToolExecutionRecipe> {
        if (error) *error = QStringLiteral("Unsupported legacy tool semantics must be neutral: %1")
            .arg(QString::fromLatin1(field));
        return std::nullopt;
    };
#define REQUIRE_ZERO(field) if (tool.field != 0) return reject(#field)
    REQUIRE_ZERO(m_bAZero); REQUIRE_ZERO(m_dAPos);
    REQUIRE_ZERO(m_bA1Zero); REQUIRE_ZERO(m_dA1Pos);
    REQUIRE_ZERO(m_bXIsMove); REQUIRE_ZERO(m_dXPosition);
    REQUIRE_ZERO(m_bX1IsMove); REQUIRE_ZERO(m_dX1Position);
    REQUIRE_ZERO(m_bYIsMove); REQUIRE_ZERO(m_dYPosition);
    REQUIRE_ZERO(m_bY1IsMove); REQUIRE_ZERO(m_dY1Position);
    REQUIRE_ZERO(m_bAIsMove); REQUIRE_ZERO(m_dAPosition);
    REQUIRE_ZERO(m_bA1IsMove); REQUIRE_ZERO(m_dA1Position);
    REQUIRE_ZERO(m_dExtend); REQUIRE_ZERO(m_dExtend_End);
    REQUIRE_ZERO(m_dRadius); REQUIRE_ZERO(m_dOffsetDiameter);
    REQUIRE_ZERO(m_dOffsetDistance); REQUIRE_ZERO(m_dOffsetIgnoreLength);
    REQUIRE_ZERO(m_dR0); REQUIRE_ZERO(m_dZ0); REQUIRE_ZERO(m_dFocusOffset);
    REQUIRE_ZERO(m_bCuttingHead); REQUIRE_ZERO(m_bCrossBridge); REQUIRE_ZERO(m_dServoCuttingHeight);
    REQUIRE_ZERO(m_bAxisZLinkage); REQUIRE_ZERO(m_dLinkedDelay); REQUIRE_ZERO(m_iLinkedMode);
    REQUIRE_ZERO(m_dLinkageParameterA); REQUIRE_ZERO(m_dLinkageParameterB);
    REQUIRE_ZERO(m_bFlightCutting); REQUIRE_ZERO(m_dFlightCutting_MotorDelay);
    REQUIRE_ZERO(m_bTroughFlag); REQUIRE_ZERO(m_iTroughBuffer); REQUIRE_ZERO(m_dTroughDelay);
    REQUIRE_ZERO(m_bPunch); REQUIRE_ZERO(m_dWaitFirst); REQUIRE_ZERO(m_dWaitSecond);
    REQUIRE_ZERO(m_dArcVelocity); REQUIRE_ZERO(m_dArcAcc); REQUIRE_ZERO(m_dArcJerk);
    REQUIRE_ZERO(m_iPDMode); REQUIRE_ZERO(m_dPDScaleFactor); REQUIRE_ZERO(m_dPDWidth);
    REQUIRE_ZERO(m_dPDPosOffset); REQUIRE_ZERO(m_dPDLowVelMax); REQUIRE_ZERO(m_dPDPosLowVelMax);
    REQUIRE_ZERO(m_dPDLowVelMax_1); REQUIRE_ZERO(m_dPDPosLowVelMax_1);
#undef REQUIRE_ZERO
    if (!tool.m_strOffsetType.empty() || !tool.m_sLinkedDirection.empty() || !tool.m_sLinkedFormula.empty())
        return reject("offset/linkage expression");
    // Legacy axis labels are accepted only in their inert default form; axis
    // mapping is owned by the frozen MachineAxisLayout, never by these strings.
    if ((!tool.m_strDirectionX.empty() && tool.m_strDirectionX != "X")
        || (!tool.m_strDirectionY.empty() && tool.m_strDirectionY != "Y"))
        return reject("axis direction remapping");
    FrozenToolExecutionRecipe result;
    result.toolName = QString::fromStdString(tool.m_strName);
    result.sourceId = sourceId;
#define LCNC_EXECUTION_FIELD(type, name, legacy) result.name = tool.legacy;
#include "modules/process/runtime/frozen_tool_execution_fields.inc"
#undef LCNC_EXECUTION_FIELD
    if (!validateToolExecutionRecipe(result, false, error)) return std::nullopt;
    return result;
}
} // namespace lcnc::process
