#include "core/algorithms/cad/boolean_ops.h"
#include "core/algorithms/occt_exact_operation_lock.h"

#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <Standard_Failure.hxx>
#include <stdexcept>

namespace lcnc::cad_algo {

namespace {
void checkInputs(const TopoDS_Shape& a, const TopoDS_Shape& b)
{
    if (a.IsNull() || b.IsNull())
        throw std::invalid_argument("Boolean input shape is null");
}
} // namespace

TopoDS_Shape fuseShapes(const TopoDS_Shape& a, const TopoDS_Shape& b)
{
    checkInputs(a, b);
    lcnc::OcctExactOperationLock exactOperationLock;
    BRepAlgoAPI_Fuse op(a, b);
    if (!op.IsDone())
        throw Standard_Failure("Fuse failed");
    return op.Shape();
}

TopoDS_Shape cutShapes(const TopoDS_Shape& a, const TopoDS_Shape& b)
{
    checkInputs(a, b);
    lcnc::OcctExactOperationLock exactOperationLock;
    BRepAlgoAPI_Cut op(a, b);
    if (!op.IsDone())
        throw Standard_Failure("Cut failed");
    return op.Shape();
}

TopoDS_Shape commonShapes(const TopoDS_Shape& a, const TopoDS_Shape& b)
{
    checkInputs(a, b);
    lcnc::OcctExactOperationLock exactOperationLock;
    BRepAlgoAPI_Common op(a, b);
    if (!op.IsDone())
        throw Standard_Failure("Common failed");
    return op.Shape();
}

} // namespace lcnc::cad_algo
