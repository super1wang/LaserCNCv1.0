#include "core/algorithms/cad/transform_ops.h"

#include <BRepBuilderAPI_Transform.hxx>
#include <Standard_Failure.hxx>
#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Trsf.hxx>

#include <cmath>
#include <stdexcept>

namespace lcnc::cad_algo {
namespace {

constexpr double kPi = 3.14159265358979323846;

void applyTrsf(TopoDS_Shape* shape, const gp_Trsf& trsf)
{
    if (!shape || shape->IsNull())
        throw std::invalid_argument("Transform input shape is null");

    BRepBuilderAPI_Transform transform(*shape, trsf, true);
    if (!transform.IsDone())
        throw Standard_Failure("Body transformation failed");
    *shape = transform.Shape();
    if (shape->IsNull())
        throw Standard_Failure("Body transformation produced a null shape");
}

void applyRotation(TopoDS_Shape* shape,
                   const gp_Pnt& referencePoint,
                   const gp_Dir& axisDir,
                   double angleDeg)
{
    if (std::abs(angleDeg) < 1.0e-9)
        return;

    gp_Trsf trsf;
    trsf.SetRotation(gp_Ax1(referencePoint, axisDir), angleDeg * kPi / 180.0);
    applyTrsf(shape, trsf);
}

} // namespace

TopoDS_Shape transformShape(const TopoDS_Shape& shape,
                            const TransformParams& params)
{
    if (shape.IsNull())
        throw std::invalid_argument("Transform input shape is null");

    TopoDS_Shape result = shape;
    applyRotation(&result, params.referencePoint, gp_Dir(1.0, 0.0, 0.0), params.rotateXDeg);
    applyRotation(&result, params.referencePoint, gp_Dir(0.0, 1.0, 0.0), params.rotateYDeg);
    applyRotation(&result, params.referencePoint, gp_Dir(0.0, 0.0, 1.0), params.rotateZDeg);

    if (params.translation.SquareMagnitude() > 1.0e-18) {
        gp_Trsf trsf;
        trsf.SetTranslation(params.translation);
        applyTrsf(&result, trsf);
    }

    return result;
}

} // namespace lcnc::cad_algo
