#include "core/algorithms/cad/transform_ops.h"

#include <BRepBuilderAPI_Transform.hxx>
#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Trsf.hxx>

#include <cmath>

namespace lcnc::cad_algo {
namespace {

constexpr double kPi = 3.14159265358979323846;

void setErr(QString* errMsg, const QString& message)
{
    if (errMsg)
        *errMsg = message;
}

bool applyTrsf(TopoDS_Shape* shape, const gp_Trsf& trsf, QString* errMsg)
{
    if (!shape || shape->IsNull()) {
        // 中文翻译：变换输入形体为空
        setErr(errMsg, QStringLiteral("Transform input shape to empty"));
        return false;
    }

    BRepBuilderAPI_Transform transform(*shape, trsf, Standard_True);
    if (!transform.IsDone()) {
        // 中文翻译：形体变换失败
        setErr(errMsg, QStringLiteral("Body transformation failed"));
        return false;
    }
    *shape = transform.Shape();
    return !shape->IsNull();
}

bool applyRotation(TopoDS_Shape* shape,
                   const gp_Pnt& referencePoint,
                   const gp_Dir& axisDir,
                   double angleDeg,
                   QString* errMsg)
{
    if (std::abs(angleDeg) < 1.0e-9)
        return true;

    gp_Trsf trsf;
    trsf.SetRotation(gp_Ax1(referencePoint, axisDir), angleDeg * kPi / 180.0);
    return applyTrsf(shape, trsf, errMsg);
}

} // namespace

TopoDS_Shape transformShape(const TopoDS_Shape& shape,
                            const TransformParams& params,
                            QString* errMsg)
{
    if (shape.IsNull()) {
        // 中文翻译：变换输入形体为空
        setErr(errMsg, QStringLiteral("Transform input shape to empty"));
        return {};
    }

    TopoDS_Shape result = shape;
    if (!applyRotation(&result, params.referencePoint, gp_Dir(1.0, 0.0, 0.0), params.rotateXDeg, errMsg))
        return {};
    if (!applyRotation(&result, params.referencePoint, gp_Dir(0.0, 1.0, 0.0), params.rotateYDeg, errMsg))
        return {};
    if (!applyRotation(&result, params.referencePoint, gp_Dir(0.0, 0.0, 1.0), params.rotateZDeg, errMsg))
        return {};

    if (params.translation.SquareMagnitude() > 1.0e-18) {
        gp_Trsf trsf;
        trsf.SetTranslation(params.translation);
        if (!applyTrsf(&result, trsf, errMsg))
            return {};
    }

    return result;
}

} // namespace lcnc::cad_algo
