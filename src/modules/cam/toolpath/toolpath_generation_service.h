#pragma once

#include <QString>
#include <TopoDS_Shape.hxx>

#include <cstdint>
#include <vector>

namespace lcnc::cam {

struct ToolpathGenerationSource
{
    QString entry;
    int componentIndex{-1};
    TopoDS_Shape shape;
};

struct ToolpathGenerationStamp
{
    std::uint64_t toolpathRevision{0};
    std::uint64_t machiningFaceRevision{0};
    std::uint64_t machineSetupRevision{0};
    double leadInLength{0.0};
    double smoothAngle{0.0};
    double deflection{0.0};
    double cuttingOffsetMm{1.0};
    double rapidOffsetMm{5.0};
    bool useFaceClassification{false};
    int extractionStrategy{0};
    std::vector<std::uint64_t> contourIds;
    std::vector<ToolpathGenerationSource> sources;
};

class ToolpathGenerationService
{
public:
    [[nodiscard]] static bool acceptsResult(
        const ToolpathGenerationStamp& captured,
        const ToolpathGenerationStamp& current,
        bool taskSucceeded,
        bool cancellationRequested) noexcept;
};

} // namespace lcnc::cam
