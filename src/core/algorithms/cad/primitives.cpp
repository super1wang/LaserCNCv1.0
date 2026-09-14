#include "core/algorithms/cad/primitives.h"

#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <stdexcept>

namespace lcnc::cad_algo {

TopoDS_Shape makeBox(double dx, double dy, double dz)
{
    if (dx <= 0.0 || dy <= 0.0 || dz <= 0.0)
        throw std::invalid_argument("Box dimensions must be > 0");
    return BRepPrimAPI_MakeBox(dx, dy, dz).Shape();
}

TopoDS_Shape makeCylinder(double radius, double height)
{
    if (radius <= 0.0 || height <= 0.0)
        throw std::invalid_argument("Cylinder radius/height must be > 0");
    return BRepPrimAPI_MakeCylinder(radius, height).Shape();
}

TopoDS_Shape makeSphere(double radius)
{
    if (radius <= 0.0)
        throw std::invalid_argument("Sphere radius must be > 0");
    return BRepPrimAPI_MakeSphere(radius).Shape();
}

TopoDS_Shape makeCone(double r1, double r2, double height)
{
    if (height <= 0.0 || (r1 <= 0.0 && r2 <= 0.0) || r1 < 0.0 || r2 < 0.0)
        throw std::invalid_argument(
            "Cone parameters invalid (height>0, r1>=0, r2>=0, not both zero)");
    return BRepPrimAPI_MakeCone(r1, r2, height).Shape();
}

TopoDS_Shape makeTorus(double r1, double r2)
{
    if (r1 <= r2 || r2 <= 0.0)
        throw std::invalid_argument("Torus requires r1 > r2 > 0");
    return BRepPrimAPI_MakeTorus(r1, r2).Shape();
}

} // namespace lcnc::cad_algo
