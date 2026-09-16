#include "modules/cam/toolpath/toolpath_generation_service.h"

#include <QCryptographicHash>

#include <algorithm>
#include <cmath>

namespace lcnc::cam {
namespace {

bool sameSources(const std::vector<ToolpathGenerationSource>& first,
                 const std::vector<ToolpathGenerationSource>& second)
{
    return first.size() == second.size()
        && std::equal(first.cbegin(), first.cend(), second.cbegin(),
            [](const ToolpathGenerationSource& left,
               const ToolpathGenerationSource& right) {
                return left.entry == right.entry
                    && left.componentIndex == right.componentIndex
                    && left.shape.IsSame(right.shape);
            });
}

} // namespace

bool ToolpathGenerationService::acceptsResult(
    const ToolpathGenerationStamp& captured,
    const ToolpathGenerationStamp& current,
    bool taskSucceeded,
    bool cancellationRequested) noexcept
{
    constexpr double kTolerance = 1.0e-12;
    return taskSucceeded
        && !cancellationRequested
        && captured.toolpathRevision == current.toolpathRevision
        && captured.machiningFaceRevision == current.machiningFaceRevision
        && captured.machineSetupRevision == current.machineSetupRevision
        && std::abs(captured.leadInLength - current.leadInLength) <= kTolerance
        && std::abs(captured.smoothAngle - current.smoothAngle) <= kTolerance
        && std::abs(captured.deflection - current.deflection) <= kTolerance
        && std::abs(captured.cuttingOffsetMm - current.cuttingOffsetMm) <= kTolerance
        && std::abs(captured.rapidOffsetMm - current.rapidOffsetMm) <= kTolerance
        && captured.useFaceClassification == current.useFaceClassification
        && captured.extractionStrategy == current.extractionStrategy
        && captured.contourIds == current.contourIds
        && sameSources(captured.sources, current.sources);
}

MotionCompilationInput ToolpathGenerationService::captureMotionInput(
    const ToolpathGenerationStamp& generation,
    const MotionCompilationContext& context,
    const QVector<MotionCompilationInput::Parameter>& parameters)
{
    MotionCompilationInput input;
    input.generation = generation;
    input.parameters = parameters;
    input.context = context;
    QByteArray policy("lcnc.motion-policy.v3.2.1;");
    const auto append = [&policy](const QByteArray& value) {
        policy += QByteArray::number(value.size()) + ':' + value + ';';
    };
    for (const auto& parameter : input.parameters) {
        append(parameter.key.toUtf8());
        append(parameter.requested.toUtf8());
        append(parameter.effective.toUtf8());
        append(parameter.sourceId.toUtf8());
        append(parameter.unit.toUtf8());
        append(QByteArray::number(parameter.revision));
        append(parameter.available ? QByteArrayLiteral("available")
                                   : QByteArrayLiteral("unavailable"));
    }
    input.context.optimizationPolicyHash =
        QCryptographicHash::hash(policy, QCryptographicHash::Sha256);
    input.capturedContextHash = motionCompilationContextHash(input.context);
    return input;
}

bool ToolpathGenerationService::acceptsMotionResult(
    const MotionCompilationInput& captured,
    const ToolpathGenerationStamp& currentGeneration,
    const MotionCompilationContext& currentContext,
    bool taskSucceeded,
    bool cancellationRequested)
{
    return acceptsResult(captured.generation, currentGeneration,
                         taskSucceeded, cancellationRequested)
        && captured.capturedContextHash == motionCompilationContextHash(captured.context)
        && captured.capturedContextHash == motionCompilationContextHash(currentContext);
}

} // namespace lcnc::cam
