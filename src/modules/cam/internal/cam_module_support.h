#pragma once

#include "core/document/lcnc_document.h"
#include "core/task/task_manager.h"

#include <Bnd_Box.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <QList>
#include <QStringList>

#include <algorithm>
#include <functional>

class QObject;

namespace lcnc::cam::detail {

void watchTask(QObject* owner, TaskId taskId, std::function<void(bool)> onFinished);
QStringList entityEntries(LcncDocument* document, LcncDocument::EntityKind kind);
bool shapeContainsFace(const TopoDS_Shape& shape, const TopoDS_Face& face);

template <typename Source>
bool faceBelongsToSource(const TopoDS_Face& face,
                         const Source& source,
                         const QList<Source>& sources)
{
    if (shapeContainsFace(source.shape, face))
        return true;
    if (std::any_of(sources.cbegin(), sources.cend(), [&face](const auto& candidate) {
            return shapeContainsFace(candidate.shape, face);
        })) {
        return false;
    }
    return std::count_if(sources.cbegin(), sources.cend(), [&source](const auto& candidate) {
               return candidate.workpieceEntry == source.workpieceEntry;
           }) == 1;
}

TopoDS_Shape translatedShapeCopy(const TopoDS_Shape& shape, const gp_Vec& translation);
gp_Pnt bboxCenter(const Bnd_Box& box);
gp_Pnt shapeCenter(const TopoDS_Shape& shape);

} // namespace lcnc::cam::detail
