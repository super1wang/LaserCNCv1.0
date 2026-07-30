#include "view/sketch_overlay_renderer.h"

#include "core/algorithms/cad/sketch.h"
#include "core/logging/logger.h"

#include <AIS_InteractiveContext.hxx>
#include <AIS_InteractiveObject.hxx>
#include <AIS_Shape.hxx>
#include <BRep_Builder.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <Quantity_Color.hxx>
#include <SelectMgr_EntityOwner.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Wire.hxx>

#include <Standard_Failure.hxx>
#include <stdexcept>

namespace lcnc::view {
namespace {

constexpr int kSketchToolPoint = 1;
constexpr int kSketchToolLine = 2;
constexpr int kSketchToolArc = 3;
constexpr int kSketchToolCircle = 4;
constexpr int kSketchToolRectangle = 5;
constexpr int kSketchToolPolygon = 6;

lcnc::cad_algo::SketchPlane sketchPlane(int planeKind)
{
    switch (planeKind) {
    case 1:
        return lcnc::cad_algo::SketchPlane::yz();
    case 2:
        return lcnc::cad_algo::SketchPlane::zx();
    case 0:
    default:
        return lcnc::cad_algo::SketchPlane::xy();
    }
}

gp_Pnt toWorld(const lcnc::cad_algo::SketchPlane& plane, double x, double y)
{
    gp_Pnt point = plane.axes.Location();
    point.Translate(gp_Vec(plane.axes.XDirection()).Multiplied(x));
    point.Translate(gp_Vec(plane.axes.YDirection()).Multiplied(y));
    return point;
}

Quantity_Color toQuantityColor(const QColor& color)
{
    return Quantity_Color(color.redF(), color.greenF(), color.blueF(), Quantity_TOC_RGB);
}

TopoDS_Shape buildOverlayShape(const SketchOverlayItem& item)
{
    const auto plane = sketchPlane(item.plane);
    const QVector<double>& p = item.params;
    switch (item.kind) {
    case kSketchToolPoint:
        if (p.size() < 2)
            return {};
        {
            constexpr double kHalfMarkerSize = 2.0;
            TopoDS_Compound marker;
            BRep_Builder builder;
            builder.MakeCompound(marker);

            const gp_Pnt left = toWorld(plane, p[0] - kHalfMarkerSize, p[1]);
            const gp_Pnt right = toWorld(plane, p[0] + kHalfMarkerSize, p[1]);
            const gp_Pnt bottom = toWorld(plane, p[0], p[1] - kHalfMarkerSize);
            const gp_Pnt top = toWorld(plane, p[0], p[1] + kHalfMarkerSize);
            BRepBuilderAPI_MakeEdge horizontal(left, right);
            BRepBuilderAPI_MakeEdge vertical(bottom, top);
            if (horizontal.IsDone())
                builder.Add(marker, horizontal.Shape());
            if (vertical.IsDone())
                builder.Add(marker, vertical.Shape());
            if (marker.IsNull())
                return BRepBuilderAPI_MakeVertex(toWorld(plane, p[0], p[1])).Shape();
            return marker;
        }
    case kSketchToolLine:
        if (p.size() < 4)
            return {};
        return lcnc::cad_algo::makeLineWire(plane, {p[0], p[1]}, {p[2], p[3]});
    case kSketchToolArc:
        if (p.size() < 6)
            return {};
        return lcnc::cad_algo::makeArcWire(plane, {p[0], p[1]}, {p[2], p[3]}, {p[4], p[5]});
    case kSketchToolCircle:
        if (p.size() < 3)
            return {};
        return lcnc::cad_algo::makeCircleWire(plane, p[2], {p[0], p[1]});
    case kSketchToolRectangle:
        if (p.size() < 4)
            return {};
        return lcnc::cad_algo::makeRectangleWire(plane, p[2], p[3], {p[0], p[1]});
    case kSketchToolPolygon:
        if (p.size() < 4)
            return {};
        return lcnc::cad_algo::makePolygonWire(plane, static_cast<int>(p[3]), p[2], {p[0], p[1]});
    default:
        return {};
    }
}

} // namespace

void SketchOverlayRenderer::setItems(const QVector<SketchOverlayItem>& items)
{
    m_items = items;
}

void SketchOverlayRenderer::render(const Handle(AIS_InteractiveContext)& context, bool updateViewer)
{
    if (context.IsNull())
        return;

    clearObjects(context, false);

    for (const auto& item : m_items) {
        if (item.key.isEmpty() || !item.visible)
            continue;

        TopoDS_Shape shape;
        try {
            shape = buildOverlayShape(item);
        } catch (const Standard_Failure& exception) {
            LCNC_ERR(lcnc::LogCode::Generic,
                     "Sketch overlay OCC projection failed: {}",
                     exception.GetMessageString());
            continue;
        } catch (const std::exception& exception) {
            LCNC_ERR(lcnc::LogCode::Generic,
                     "Sketch overlay projection failed: {}",
                     exception.what());
            continue;
        } catch (...) {
            LCNC_ERR(lcnc::LogCode::Generic,
                     "Sketch overlay projection failed with an unknown exception");
            continue;
        }
        if (shape.IsNull())
            continue;

        Handle(AIS_Shape) ais = new AIS_Shape(shape);
        context->Display(ais, AIS_WireFrame, 0, false);
        context->SetColor(ais, toQuantityColor(item.color), false);
        context->SetWidth(ais, 2.0, false);
        m_objectsByKey.insert(item.key, ais);
        m_keysByObject.insert(ais.get(), item.key);
    }

    if (updateViewer)
        context->UpdateCurrentViewer();
}

void SketchOverlayRenderer::clearObjects(const Handle(AIS_InteractiveContext)& context, bool updateViewer)
{
    if (!context.IsNull()) {
        for (auto it = m_objectsByKey.cbegin(); it != m_objectsByKey.cend(); ++it) {
            if (!it.value().IsNull())
                context->Erase(it.value(), false);
        }
        if (updateViewer)
            context->UpdateCurrentViewer();
    }
    m_objectsByKey.clear();
    m_keysByObject.clear();
}

void SketchOverlayRenderer::clear()
{
    m_items.clear();
    m_objectsByKey.clear();
    m_keysByObject.clear();
}

QString SketchOverlayRenderer::detectedKey(const Handle(AIS_InteractiveContext)& context) const
{
    if (context.IsNull() || !context->HasDetected())
        return {};

    const Handle(SelectMgr_EntityOwner)& owner = context->DetectedOwner();
    if (owner.IsNull())
        return {};

    const Handle(AIS_InteractiveObject) object =
        Handle(AIS_InteractiveObject)::DownCast(owner->Selectable());
    if (object.IsNull())
        return {};
    return m_keysByObject.value(object.get());
}

int SketchOverlayRenderer::planeForKey(const QString& key) const
{
    for (const auto& item : m_items) {
        if (item.key == key)
            return item.plane;
    }
    return 0;
}

bool SketchOverlayRenderer::isDraggable(const QString& key) const
{
    for (const auto& item : m_items) {
        if (item.key == key)
            return item.draggable;
    }
    return false;
}

} // namespace lcnc::view
