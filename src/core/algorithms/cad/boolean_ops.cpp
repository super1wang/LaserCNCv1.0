#include "core/algorithms/cad/boolean_ops.h"

#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <Standard_Failure.hxx>

#include <QString>

namespace lcnc::cad_algo {

namespace {
inline bool checkInputs(const TopoDS_Shape& a, const TopoDS_Shape& b, QString* errMsg)
{
    if (a.IsNull() || b.IsNull()) {
        if (errMsg) *errMsg = QStringLiteral("Boolean: input shape is null");
        return false;
    }
    return true;
}
} // namespace

TopoDS_Shape fuseShapes(const TopoDS_Shape& a, const TopoDS_Shape& b, QString* errMsg)
{
    if (!checkInputs(a, b, errMsg)) return {};
    try {
        BRepAlgoAPI_Fuse op(a, b);
        if (!op.IsDone()) {
            if (errMsg) *errMsg = QStringLiteral("Fuse failed");
            return {};
        }
        return op.Shape();
    } catch (const Standard_Failure& f) {
        if (errMsg) *errMsg = QString::fromUtf8(f.GetMessageString());
        return {};
    }
}

TopoDS_Shape cutShapes(const TopoDS_Shape& a, const TopoDS_Shape& b, QString* errMsg)
{
    if (!checkInputs(a, b, errMsg)) return {};
    try {
        BRepAlgoAPI_Cut op(a, b);
        if (!op.IsDone()) {
            if (errMsg) *errMsg = QStringLiteral("Cut failed");
            return {};
        }
        return op.Shape();
    } catch (const Standard_Failure& f) {
        if (errMsg) *errMsg = QString::fromUtf8(f.GetMessageString());
        return {};
    }
}

TopoDS_Shape commonShapes(const TopoDS_Shape& a, const TopoDS_Shape& b, QString* errMsg)
{
    if (!checkInputs(a, b, errMsg)) return {};
    try {
        BRepAlgoAPI_Common op(a, b);
        if (!op.IsDone()) {
            if (errMsg) *errMsg = QStringLiteral("Common failed (shapes may not intersect)");
            return {};
        }
        return op.Shape();
    } catch (const Standard_Failure& f) {
        if (errMsg) *errMsg = QString::fromUtf8(f.GetMessageString());
        return {};
    }
}

} // namespace lcnc::cad_algo
