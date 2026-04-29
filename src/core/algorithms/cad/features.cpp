#include "core/algorithms/cad/features.h"

#include <BRepOffsetAPI_MakePipe.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <Standard_Failure.hxx>
#include <gp_Vec.hxx>

#include <QString>
#include <QtMath>

namespace lcnc::cad_algo {

namespace {
void setErr(QString* errMsg, const QString& message)
{
    if (errMsg)
        *errMsg = message;
}
} // namespace

TopoDS_Shape extrudeShape(const TopoDS_Shape& profile,
                          const gp_Dir& direction,
                          double length,
                          QString* errMsg)
{
    if (profile.IsNull()) {
        setErr(errMsg, QStringLiteral("Extrude profile is null"));
        return {};
    }
    if (qFuzzyIsNull(length)) {
        setErr(errMsg, QStringLiteral("Extrude length must be non-zero"));
        return {};
    }

    try {
        gp_Vec prismVector(direction);
        prismVector.Multiply(length);
        BRepPrimAPI_MakePrism prism(profile, prismVector, false, true);
        if (!prism.IsDone()) {
            setErr(errMsg, QStringLiteral("Extrude operation failed"));
            return {};
        }
        return prism.Shape();
    } catch (const Standard_Failure& f) {
        setErr(errMsg, QString::fromUtf8(f.GetMessageString()));
        return {};
    }
}

TopoDS_Shape revolveShape(const TopoDS_Shape& profile,
                          const gp_Ax1& axis,
                          double angleDeg,
                          QString* errMsg)
{
    if (profile.IsNull()) {
        setErr(errMsg, QStringLiteral("Revolve profile is null"));
        return {};
    }
    if (qFuzzyIsNull(angleDeg)) {
        setErr(errMsg, QStringLiteral("Revolve angle must be non-zero"));
        return {};
    }

    try {
        BRepPrimAPI_MakeRevol revol(profile, axis, qDegreesToRadians(angleDeg), false);
        if (!revol.IsDone()) {
            setErr(errMsg, QStringLiteral("Revolve operation failed"));
            return {};
        }
        return revol.Shape();
    } catch (const Standard_Failure& f) {
        setErr(errMsg, QString::fromUtf8(f.GetMessageString()));
        return {};
    }
}

TopoDS_Shape sweepShape(const TopoDS_Wire& profile,
                        const TopoDS_Wire& spine,
                        QString* errMsg)
{
    if (profile.IsNull() || spine.IsNull()) {
        setErr(errMsg, QStringLiteral("Sweep profile or spine is null"));
        return {};
    }

    try {
        BRepOffsetAPI_MakePipe pipe(spine, profile);
        if (!pipe.IsDone()) {
            setErr(errMsg, QStringLiteral("Sweep operation failed"));
            return {};
        }
        return pipe.Shape();
    } catch (const Standard_Failure& f) {
        setErr(errMsg, QString::fromUtf8(f.GetMessageString()));
        return {};
    }
}

} // namespace lcnc::cad_algo