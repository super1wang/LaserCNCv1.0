#include "view/transform_gizmo_renderer.h"

#include <QStringList>

#include <AIS_InteractiveContext.hxx>
#include <AIS_InteractiveObject.hxx>
#include <AIS_Shape.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <GC_MakeCircle.hxx>
#include <Quantity_Color.hxx>
#include <SelectMgr_EntityOwner.hxx>
#include <TopoDS_Edge.hxx>
#include <V3d_View.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

namespace lcnc::view {
namespace {

QString partKey(int operation, int axis)
{
    return QStringLiteral("%1:%2").arg(operation).arg(axis);
}

bool parsePartKey(const QString& key, int* operation, int* axis)
{
    const QStringList parts = key.split(QLatin1Char(':'));
    if (parts.size() != 2)
        return false;
    bool opOk = false;
    bool axisOk = false;
    const int parsedOperation = parts.at(0).toInt(&opOk);
    const int parsedAxis = parts.at(1).toInt(&axisOk);
    if (!opOk || !axisOk)
        return false;
    if (operation)
        *operation = parsedOperation;
    if (axis)
        *axis = parsedAxis;
    return true;
}

gp_Pnt centerPoint(const TransformGizmoState& state)
{
    return gp_Pnt(state.centerX, state.centerY, state.centerZ);
}

gp_Dir axisDir(int axis)
{
    switch (axis) {
    case 1:
        return gp_Dir(0.0, 1.0, 0.0);
    case 2:
        return gp_Dir(0.0, 0.0, 1.0);
    case 0:
    default:
        return gp_Dir(1.0, 0.0, 0.0);
    }
}

Quantity_Color axisColor(int axis)
{
    switch (axis) {
    case 1:
        return Quantity_Color(0.20, 0.85, 0.25, Quantity_TOC_RGB);
    case 2:
        return Quantity_Color(0.25, 0.45, 1.00, Quantity_TOC_RGB);
    case 0:
    default:
        return Quantity_Color(1.00, 0.25, 0.20, Quantity_TOC_RGB);
    }
}

TopoDS_Edge makeMoveAxis(const TransformGizmoState& state, int axis)
{
    const gp_Pnt center = centerPoint(state);
    gp_Pnt end = center;
    end.Translate(gp_Vec(axisDir(axis)).Multiplied(state.size));
    return BRepBuilderAPI_MakeEdge(center, end).Edge();
}

TopoDS_Edge makeRotateRing(const TransformGizmoState& state, int axis)
{
    const gp_Circ circle(gp_Ax2(centerPoint(state), axisDir(axis)), state.size * 0.72);
    return BRepBuilderAPI_MakeEdge(circle).Edge();
}

} // namespace

void TransformGizmoRenderer::setState(const TransformGizmoState& state)
{
    m_state = state;
}

void TransformGizmoRenderer::render(const Handle(AIS_InteractiveContext)& context, bool updateViewer)
{
    if (context.IsNull())
        return;

    clearObjects(context, false);
    if (!m_state.visible)
        return;

    for (int axis = 0; axis < 3; ++axis) {
        Handle(AIS_Shape) moveObject = new AIS_Shape(makeMoveAxis(m_state, axis));
        context->Display(moveObject, AIS_WireFrame, 0, false);
        context->SetColor(moveObject, axisColor(axis), false);
        context->SetWidth(moveObject, 5.0, false);
        const QString moveKey = partKey(0, axis);
        m_objectsByKey.insert(moveKey, moveObject);
        m_keysByObject.insert(moveObject.get(), moveKey);

        Handle(AIS_Shape) rotateObject = new AIS_Shape(makeRotateRing(m_state, axis));
        context->Display(rotateObject, AIS_WireFrame, 0, false);
        context->SetColor(rotateObject, axisColor(axis), false);
        context->SetWidth(rotateObject, 2.5, false);
        const QString rotateKey = partKey(1, axis);
        m_objectsByKey.insert(rotateKey, rotateObject);
        m_keysByObject.insert(rotateObject.get(), rotateKey);
    }

    if (updateViewer)
        context->UpdateCurrentViewer();
}

void TransformGizmoRenderer::clearObjects(const Handle(AIS_InteractiveContext)& context, bool updateViewer)
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

void TransformGizmoRenderer::clear()
{
    m_state = {};
    m_objectsByKey.clear();
    m_keysByObject.clear();
}

bool TransformGizmoRenderer::detectedPart(const Handle(AIS_InteractiveContext)& context,
                                          int* operation,
                                          int* axis) const
{
    if (context.IsNull() || !context->HasDetected())
        return false;
    const Handle(SelectMgr_EntityOwner)& owner = context->DetectedOwner();
    if (owner.IsNull())
        return false;
    const Handle(AIS_InteractiveObject) object =
        Handle(AIS_InteractiveObject)::DownCast(owner->Selectable());
    if (object.IsNull())
        return false;
    const QString key = m_keysByObject.value(object.get());
    if (key.isEmpty())
        return false;
    return parsePartKey(key, operation, axis);
}

bool TransformGizmoRenderer::axisScreenVector(const Handle(V3d_View)& view,
                                              int axis,
                                              double* outX,
                                              double* outY) const
{
    if (view.IsNull() || !outX || !outY)
        return false;

    const gp_Pnt center = centerPoint(m_state);
    gp_Pnt end = center;
    end.Translate(gp_Vec(axisDir(axis)).Multiplied(m_state.size));

    Standard_Integer centerX = 0;
    Standard_Integer centerY = 0;
    Standard_Integer endX = 0;
    Standard_Integer endY = 0;
    view->Convert(center.X(), center.Y(), center.Z(), centerX, centerY);
    view->Convert(end.X(), end.Y(), end.Z(), endX, endY);

    *outX = static_cast<double>(endX - centerX);
    *outY = static_cast<double>(endY - centerY);
    return (*outX * *outX + *outY * *outY) > 1.0e-9;
}

} // namespace lcnc::view
