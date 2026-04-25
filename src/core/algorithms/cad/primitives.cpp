#include "core/algorithms/cad/primitives.h"

#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <Standard_Failure.hxx>

#include <QString>

namespace lcnc::cad_algo {

namespace {
inline void setErr(QString* errMsg, const char* what)
{
    if (errMsg) *errMsg = QString::fromLatin1(what);
}
} // namespace

TopoDS_Shape makeBox(double dx, double dy, double dz, QString* errMsg)
{
    if (dx <= 0.0 || dy <= 0.0 || dz <= 0.0) {
        setErr(errMsg, "Box dimensions must be > 0");
        return {};
    }
    try {
        BRepPrimAPI_MakeBox mk(dx, dy, dz);
        return mk.Shape();
    } catch (const Standard_Failure& f) {
        if (errMsg) *errMsg = QString::fromUtf8(f.GetMessageString());
        return {};
    }
}

TopoDS_Shape makeCylinder(double radius, double height, QString* errMsg)
{
    if (radius <= 0.0 || height <= 0.0) {
        setErr(errMsg, "Cylinder radius/height must be > 0");
        return {};
    }
    try {
        BRepPrimAPI_MakeCylinder mk(radius, height);
        return mk.Shape();
    } catch (const Standard_Failure& f) {
        if (errMsg) *errMsg = QString::fromUtf8(f.GetMessageString());
        return {};
    }
}

TopoDS_Shape makeSphere(double radius, QString* errMsg)
{
    if (radius <= 0.0) {
        setErr(errMsg, "Sphere radius must be > 0");
        return {};
    }
    try {
        BRepPrimAPI_MakeSphere mk(radius);
        return mk.Shape();
    } catch (const Standard_Failure& f) {
        if (errMsg) *errMsg = QString::fromUtf8(f.GetMessageString());
        return {};
    }
}

TopoDS_Shape makeCone(double r1, double r2, double height, QString* errMsg)
{
    if (height <= 0.0 || (r1 <= 0.0 && r2 <= 0.0) || r1 < 0.0 || r2 < 0.0) {
        setErr(errMsg, "Cone parameters invalid (height>0, r1>=0, r2>=0, not both zero)");
        return {};
    }
    try {
        BRepPrimAPI_MakeCone mk(r1, r2, height);
        return mk.Shape();
    } catch (const Standard_Failure& f) {
        if (errMsg) *errMsg = QString::fromUtf8(f.GetMessageString());
        return {};
    }
}

TopoDS_Shape makeTorus(double r1, double r2, QString* errMsg)
{
    if (r1 <= r2 || r2 <= 0.0) {
        setErr(errMsg, "Torus requires r1 > r2 > 0");
        return {};
    }
    try {
        BRepPrimAPI_MakeTorus mk(r1, r2);
        return mk.Shape();
    } catch (const Standard_Failure& f) {
        if (errMsg) *errMsg = QString::fromUtf8(f.GetMessageString());
        return {};
    }
}

} // namespace lcnc::cad_algo
