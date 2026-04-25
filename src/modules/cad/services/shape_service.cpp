#include "modules/cad/services/shape_service.h"

#include "core/document/lcnc_document.h"
#include "core/document/xcaf_utils.h"
#include "core/kinematics/machine_kinematics.h"

#include <BRepBuilderAPI_Transform.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <TDF_LabelSequence.hxx>
#include <TopoDS_Iterator.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <gp_Ax1.hxx>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ShapeService {

bool moveShape(LcncDocument* doc, const TDF_Label& label, const gp_Vec& translation)
{
    if (!doc || label.IsNull()) return false;

    Handle(XCAFDoc_ShapeTool) st = doc->shapeTool();
    TopoDS_Shape shape = st->GetShape(label);
    if (shape.IsNull()) return false;

    gp_Trsf trsf;
    trsf.SetTranslation(translation);
    BRepBuilderAPI_Transform xform(shape, trsf, Standard_True);
    if (!xform.IsDone()) return false;

    st->SetShape(label, xform.Shape());
    return true;
}

bool rotateShape(LcncDocument* doc, const TDF_Label& label,
                 const gp_Ax1& axis, double angleDeg)
{
    if (!doc || label.IsNull()) return false;

    Handle(XCAFDoc_ShapeTool) st = doc->shapeTool();
    TopoDS_Shape shape = st->GetShape(label);
    if (shape.IsNull()) return false;

    gp_Trsf trsf;
    trsf.SetRotation(axis, angleDeg * M_PI / 180.0);
    BRepBuilderAPI_Transform xform(shape, trsf, Standard_True);
    if (!xform.IsDone()) return false;

    st->SetShape(label, xform.Shape());
    return true;
}

void deleteShape(LcncDocument* doc, const QString& entry)
{
    if (!doc || entry.isEmpty()) return;

    MachineKinematics* kin = doc->machineKinematics();
    if (kin) {
        kin->unassignShape(entry);
        kin->unmountWorkpiece(entry);
    }

    doc->removeShapeEntity(entry);
}

int explodeShape(LcncDocument* doc, const TDF_Label& label, int entityKind)
{
    if (!doc || label.IsNull()) return 0;

    Handle(XCAFDoc_ShapeTool) st = doc->shapeTool();
    TopoDS_Shape shape = st->GetShape(label);
    if (shape.IsNull()) return 0;

    // Count direct sub-shapes
    int childCount = 0;
    for (TopoDS_Iterator it(shape); it.More(); it.Next())
        ++childCount;
    if (childCount == 0) return 0;

    const QString parentName = XcafUtils::name(label);
    const QString entry = XcafUtils::entry(label);
    auto kind = static_cast<LcncDocument::EntityKind>(entityKind);

    // Clean up kinematics references
    MachineKinematics* kin = doc->machineKinematics();
    if (kin) {
        kin->unassignShape(entry);
        kin->unmountWorkpiece(entry);
    }

    // Remove the original entity
    doc->removeShapeEntity(entry);

    // Add each direct child as a new entity
    int idx = 1;
    for (TopoDS_Iterator it(shape); it.More(); it.Next(), ++idx) {
        const TopoDS_Shape& sub = it.Value();
        if (sub.IsNull()) continue;
        const QString childName = QString("%1_%2").arg(parentName).arg(idx);
        doc->addShapeEntity(sub, childName, kind);
    }

    return childCount;
}

TDF_Label addShape(LcncDocument* doc, const TopoDS_Shape& shape,
                   const QString& name, int entityKind)
{
    if (!doc || shape.IsNull()) return TDF_Label();
    auto kind = static_cast<LcncDocument::EntityKind>(entityKind);
    return doc->addShapeEntity(shape, name, kind);
}

} // namespace ShapeService
