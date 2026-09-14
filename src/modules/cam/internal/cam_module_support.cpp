#include "modules/cam/internal/cam_module_support.h"

#include "core/document/xcaf_utils.h"
#include "core/kernel/kernel.h"
#include "core/task/task_manager.h"

#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Trsf.hxx>

#include <QMetaObject>

#include <memory>

namespace lcnc::cam::detail {

void watchTask(QObject* owner, TaskId taskId, std::function<void(bool)> onFinished)
{
    auto connection = std::make_shared<QMetaObject::Connection>();
    *connection = QObject::connect(
        lcnc::Kernel::current().taskManager(),
        &TaskManager::taskFinished,
        owner,
        [taskId, onFinished = std::move(onFinished), connection](TaskId finishedId,
                                                                 bool success) mutable {
            if (finishedId != taskId)
                return;
            QObject::disconnect(*connection);
            onFinished(success);
        });
}

QStringList entityEntries(LcncDocument* document, LcncDocument::EntityKind kind)
{
    QStringList result;
    if (!document)
        return result;
    const NCollection_Sequence<TDF_Label> labels = document->entityLabels(kind);
    for (int index = 1; index <= labels.Length(); ++index)
        result << XcafUtils::entry(labels.Value(index));
    return result;
}

bool shapeContainsFace(const TopoDS_Shape& shape, const TopoDS_Face& face)
{
    if (shape.IsNull() || face.IsNull())
        return false;
    for (TopExp_Explorer explorer(shape, TopAbs_FACE); explorer.More(); explorer.Next()) {
        if (TopoDS::Face(explorer.Current()).IsSame(face))
            return true;
    }
    return false;
}

TopoDS_Shape translatedShapeCopy(const TopoDS_Shape& shape, const gp_Vec& translation)
{
    if (shape.IsNull() || translation.SquareMagnitude() < 1e-12)
        return shape;
    gp_Trsf transform;
    transform.SetTranslation(translation);
    BRepBuilderAPI_Transform moved(shape, transform, true);
    return moved.IsDone() ? moved.Shape() : shape;
}

gp_Pnt bboxCenter(const Bnd_Box& box)
{
    if (box.IsVoid())
        return gp_Pnt(0.0, 0.0, 0.0);
    double xmin = 0.0, ymin = 0.0, zmin = 0.0;
    double xmax = 0.0, ymax = 0.0, zmax = 0.0;
    box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    return gp_Pnt(0.5 * (xmin + xmax),
                  0.5 * (ymin + ymax),
                  0.5 * (zmin + zmax));
}

gp_Pnt shapeCenter(const TopoDS_Shape& shape)
{
    if (shape.IsNull())
        return gp_Pnt(0.0, 0.0, 0.0);
    Bnd_Box box;
    BRepBndLib::Add(shape, box);
    return bboxCenter(box);
}

} // namespace lcnc::cam::detail
