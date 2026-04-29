#include "core/algorithms/cad/sketch_constraints.h"

namespace lcnc::cad_algo {

SketchConstraintState analyzeSketchConstraints(const SketchConstraintAnalysisInput& input,
                                               QString* errMsg)
{
    if (input.geometryCount < 0 || input.constraintCount < 0 || input.dimensionCount < 0) {
        if (errMsg)
            *errMsg = QStringLiteral("草图约束输入数量不能为负数");
        return SketchConstraintState::UnderDefined;
    }

    const int constraints = input.constraintCount + input.dimensionCount;
    const int roughDof = input.geometryCount * 2;
    if (constraints > roughDof)
        return SketchConstraintState::OverDefined;
    if (constraints == roughDof)
        return SketchConstraintState::FullyDefined;
    return SketchConstraintState::UnderDefined;
}

} // namespace lcnc::cad_algo