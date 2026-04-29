#pragma once

#include <QString>

namespace lcnc::cad_algo {

/// Coarse constraint state for a sketch constraint graph.
enum class SketchConstraintState {
    UnderDefined = 0,
    FullyDefined,
    OverDefined
};

/// Minimal scalar input used by the current placeholder constraint analyzer.
struct SketchConstraintAnalysisInput {
    int geometryCount{0};
    int constraintCount{0};
    int dimensionCount{0};
};

/// Estimate sketch constraint state until a full solver is introduced.
SketchConstraintState analyzeSketchConstraints(const SketchConstraintAnalysisInput& input,
                                               QString* errMsg = nullptr);

} // namespace lcnc::cad_algo