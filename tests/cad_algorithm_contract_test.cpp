#include "core/algorithms/cad/boolean_ops.h"
#include "core/algorithms/cad/primitives.h"
#include "core/algorithms/cad/sketch.h"
#include "core/algorithms/cad/transform_ops.h"
#include "modules/cad/services/cad_algorithm_boundary.h"

#include <Standard_Failure.hxx>
#include <TopoDS_Shape.hxx>

#include <cassert>
#include <stdexcept>

int main()
{
    const TopoDS_Shape box = lcnc::cad_algo::makeBox(10.0, 20.0, 30.0);
    assert(!box.IsNull());

    bool invalidArgumentPropagated = false;
    try {
        (void)lcnc::cad_algo::makeBox(0.0, 20.0, 30.0);
    } catch (const std::invalid_argument&) {
        invalidArgumentPropagated = true;
    }
    assert(invalidArgumentPropagated);

    invalidArgumentPropagated = false;
    try {
        (void)lcnc::cad_algo::makeLineWire(
            lcnc::cad_algo::SketchPlane::xy(), {1.0, 1.0}, {1.0, 1.0});
    } catch (const std::invalid_argument&) {
        invalidArgumentPropagated = true;
    }
    assert(invalidArgumentPropagated);

    invalidArgumentPropagated = false;
    try {
        (void)lcnc::cad_algo::fuseShapes({}, box);
    } catch (const std::invalid_argument&) {
        invalidArgumentPropagated = true;
    }
    assert(invalidArgumentPropagated);

    QString boundaryError;
    const TopoDS_Shape rejected = lcnc::cad::invokeCadAlgorithm(
        [] { return lcnc::cad_algo::makeSphere(-1.0); },
        &boundaryError);
    assert(rejected.IsNull());
    assert(!boundaryError.isEmpty());

    boundaryError.clear();
    const TopoDS_Shape occFailure = lcnc::cad::invokeCadAlgorithm(
        []() -> TopoDS_Shape { throw Standard_Failure("test OCC failure"); },
        &boundaryError);
    assert(occFailure.IsNull());
    assert(boundaryError.contains(QStringLiteral("test OCC failure")));

    return 0;
}
