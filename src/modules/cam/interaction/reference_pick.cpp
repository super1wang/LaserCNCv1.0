#include "modules/cam/interaction/reference_pick.h"

#include "core/algorithms/cam/laser_toolpath.h"
#include "view/widget_occ_view.h"

#include <AIS_InteractiveContext.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <SelectMgr_EntityOwner.hxx>
#include <StdSelect_BRepOwner.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <V3d_View.hxx>

#include <QCoreApplication>
#include <QPoint>
#include <QString>

#include <cmath>

namespace lcnc::cam::reference_pick {

namespace {
QString tr(const char* key)
{
    return QCoreApplication::translate("CamModule", key);
}
} // namespace

bool resolveReferencePlaneCenter(WidgetOccView* occView,
                                 const QPoint& screenPos,
                                 gp_Pnt& center,
                                 QString* errorMessage)
{
    if (!occView || occView->view().IsNull() || occView->context().IsNull()) {
        if (errorMessage)
            // 中文翻译：当前没有可用的机台视图用于参考面拾取。
            *errorMessage = tr("There are currently no machine views available for reference surface picking.");
        return false;
    }

    const Handle(AIS_InteractiveContext)& context = occView->context();
    context->MoveTo(screenPos.x(), screenPos.y(), occView->view(), Standard_False);

    const Handle(SelectMgr_EntityOwner) owner = context->DetectedOwner();
    Handle(StdSelect_BRepOwner) brepOwner = Handle(StdSelect_BRepOwner)::DownCast(owner);
    if (brepOwner.IsNull() || !brepOwner->HasShape()) {
        if (errorMessage)
            // 中文翻译：请将光标放在机台模型或挂载工件的平面上。\n当前未检测到可用参考面。
            *errorMessage = tr("Please place the cursor on the machine model or the plane where the workpiece is mounted.\nNo available reference surfaces are currently detected.");
        return false;
    }

    const TopoDS_Shape pickedShape = brepOwner->Shape();
    if (pickedShape.IsNull() || pickedShape.ShapeType() != TopAbs_FACE) {
        if (errorMessage)
            // 中文翻译：当前拾取的不是平面面片，请重新选择参考平面。
            *errorMessage = tr("What is currently picked is not a plane patch, please reselect the reference plane.");
        return false;
    }

    const TopoDS_Face face = TopoDS::Face(pickedShape);
    const BRepAdaptor_Surface surface(face);
    if (surface.GetType() != GeomAbs_Plane) {
        if (errorMessage)
            // 中文翻译：当前拾取的面不是平面，请选择平面参考面。
            *errorMessage = tr("The currently picked face is not a plane, please select a planar reference face.");
        return false;
    }

    GProp_GProps props;
    BRepGProp::SurfaceProperties(face, props);
    center = props.CentreOfMass();

    if (owner->HasSelectable() && !owner->Selectable().IsNull())
        center.Transform(owner->Selectable()->LocalTransformation());

    return true;
}

bool resolveLeadInHit(WidgetOccView* occView,
                      const QPoint& screenPos,
                      const LaserToolpath& toolpath,
                      int& contourIdx,
                      int& pointIdx,
                      gp_Pnt& entryPoint,
                      double& entryParam)
{
    if (!occView || occView->view().IsNull())
        return false;

    const Handle(V3d_View)& view = occView->view();
    constexpr double kMaxScreenDistanceSq = 24.0 * 24.0;
    double bestDistanceSq = kMaxScreenDistanceSq;
    int bestContour = -1;
    int bestPoint = -1;

    for (int contourIndex = 0; contourIndex < toolpath.contourCount(); ++contourIndex) {
        const LaserContour& contour = toolpath.contour(contourIndex);
        if (!contour.enabled)
            continue;

        for (int pointIndex = 0; pointIndex < static_cast<int>(contour.points.size()); ++pointIndex) {
            const ToolpathPoint& point = contour.points[pointIndex];
            Standard_Integer px = 0;
            Standard_Integer py = 0;
            view->Convert(point.position.X(), point.position.Y(), point.position.Z(), px, py);

            const double dx = static_cast<double>(px - screenPos.x());
            const double dy = static_cast<double>(py - screenPos.y());
            const double distanceSq = dx * dx + dy * dy;
            if (distanceSq > bestDistanceSq)
                continue;

            bestDistanceSq = distanceSq;
            bestContour = contourIndex;
            bestPoint = pointIndex;
        }
    }

    if (bestContour < 0 || bestPoint < 0)
        return false;

    const ToolpathPoint& hitPoint = toolpath.contour(bestContour).points[bestPoint];
    contourIdx = bestContour;
    pointIdx = bestPoint;
    entryPoint = hitPoint.position;
    entryParam = hitPoint.param;
    return true;
}

} // namespace lcnc::cam::reference_pick
