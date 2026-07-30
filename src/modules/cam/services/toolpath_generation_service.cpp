#include "modules/cam/services/toolpath_generation_service.h"

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
        && captured.useFaceClassification == current.useFaceClassification
        && captured.extractionStrategy == current.extractionStrategy
        && captured.contourIds == current.contourIds
        && sameSources(captured.sources, current.sources);
}

} // namespace lcnc::cam
