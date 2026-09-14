#include "core/algorithms/cad/features.h"

#include <BRepOffsetAPI_MakePipe.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <Standard_Failure.hxx>
#include <gp_Vec.hxx>

#include <cmath>
#include <stdexcept>

namespace lcnc::cad_algo {

TopoDS_Shape extrudeShape(const TopoDS_Shape& profile,
                          const gp_Dir& direction,
                          double length)
{
    if (profile.IsNull())
        throw std::invalid_argument("Extrude profile is null");
    if (std::abs(length) <= 1.0e-12)
        throw std::invalid_argument("Extrude length must be non-zero");

    gp_Vec prismVector(direction);
    prismVector.Multiply(length);
    BRepPrimAPI_MakePrism prism(profile, prismVector, false, true);
    if (!prism.IsDone())
        throw Standard_Failure("Extrude operation failed");
    return prism.Shape();
}

TopoDS_Shape revolveShape(const TopoDS_Shape& profile,
                          const gp_Ax1& axis,
                          double angleDeg)
{
    if (profile.IsNull())
        throw std::invalid_argument("Revolve profile is null");
    if (std::abs(angleDeg) <= 1.0e-12)
        throw std::invalid_argument("Revolve angle must be non-zero");

    constexpr double kPi = 3.14159265358979323846;
    BRepPrimAPI_MakeRevol revol(profile, axis, angleDeg * kPi / 180.0, false);
    if (!revol.IsDone())
        throw Standard_Failure("Revolve operation failed");
    return revol.Shape();
}

TopoDS_Shape sweepShape(const TopoDS_Wire& profile,
                        const TopoDS_Wire& spine)
{
    if (profile.IsNull() || spine.IsNull())
        throw std::invalid_argument("Sweep profile or spine is null");

    BRepOffsetAPI_MakePipe pipe(spine, profile);
    if (!pipe.IsDone())
        throw Standard_Failure("Sweep operation failed");
    return pipe.Shape();
}

} // namespace lcnc::cad_algo
