#pragma once

#include <TDF_Label.hxx>
#include <TopoDS_Shape.hxx>

#include <QString>

class LcncDocument;
class gp_Vec;
class gp_Ax1;

namespace ShapeService {

/// Translate a shape entity in place within the XCAF document.
/// Returns true if the transform was applied successfully.
bool moveShape(LcncDocument* doc, const TDF_Label& label, const gp_Vec& translation);

/// Rotate a shape entity in place within the XCAF document.
/// @p angleDeg is in degrees. Returns true if the transform was applied successfully.
bool rotateShape(LcncDocument* doc, const TDF_Label& label,
                 const gp_Ax1& axis, double angleDeg);

/// Delete a shape entity from the XCAF document and clean up kinematics references.
/// Does NOT erase the AIS object — caller must handle display removal.
void deleteShape(LcncDocument* doc, const QString& entry);

/// Decompose a compound/assembly into direct child shapes.
/// The original shape is removed, and child shapes are added as new entities.
/// Returns the number of child shapes created, or 0 if decomposition was not possible.
int explodeShape(LcncDocument* doc, const TDF_Label& label,
                 int entityKind);

/// Add a shape to a document as a new entity, returning its label.
TDF_Label addShape(LcncDocument* doc, const TopoDS_Shape& shape,
                   const QString& name, int entityKind);

} // namespace ShapeService
