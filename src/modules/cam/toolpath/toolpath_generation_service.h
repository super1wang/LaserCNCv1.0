#pragma once

#include "core/project/cam/collision_validation_contracts.h"

#include <QByteArray>
#include <QString>
#include <QVector>
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

/// Detached motion compilation input. The generation part reuses the CAM
/// owner-thread stamp; worker code may only read this captured value.
struct MotionCompilationInput
{
    ToolpathGenerationStamp generation;
    struct Parameter {
        QString key;
        QString requested;
        QString effective;
        QString sourceId;
        QString unit;
        std::uint64_t revision{0};
        bool available{true};
    };
    QVector<Parameter> parameters;
    MotionCompilationContext context;
    QByteArray capturedContextHash;
};

class ToolpathGenerationService
{
public:
    [[nodiscard]] static bool acceptsResult(
        const ToolpathGenerationStamp& captured,
        const ToolpathGenerationStamp& current,
        bool taskSucceeded,
        bool cancellationRequested) noexcept;

    [[nodiscard]] static MotionCompilationInput captureMotionInput(
        const ToolpathGenerationStamp& generation,
        const MotionCompilationContext& context,
        const QVector<MotionCompilationInput::Parameter>& parameters);
    [[nodiscard]] static bool acceptsMotionResult(
        const MotionCompilationInput& captured,
        const ToolpathGenerationStamp& currentGeneration,
        const MotionCompilationContext& currentContext,
        bool taskSucceeded,
        bool cancellationRequested);
};

} // namespace lcnc::cam
