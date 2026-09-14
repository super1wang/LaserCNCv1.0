#pragma once

#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

namespace lcnc::cad_algo {

/// Parameters for a CAD shape transform around a reference point.
struct TransformParams {
    gp_Vec translation{0.0, 0.0, 0.0};
    gp_Pnt referencePoint{0.0, 0.0, 0.0};
    double rotateXDeg{0.0};
    double rotateYDeg{0.0};
    double rotateZDeg{0.0};
};

/// Build a transformed copy of a shape without mutating document state.
TopoDS_Shape transformShape(const TopoDS_Shape& shape,
                            const TransformParams& params);

} // namespace lcnc::cad_algo
