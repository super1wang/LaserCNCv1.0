#pragma once

#include <QHash>
#include <QString>

#include <Standard_Handle.hxx>

class AIS_InteractiveContext;
class AIS_InteractiveObject;
class AIS_Shape;
class V3d_View;

namespace lcnc::view {

/// Lightweight state for the CAD transform gizmo in view coordinates.
struct TransformGizmoState {
    double centerX{0.0};
    double centerY{0.0};
    double centerZ{0.0};
    double size{60.0};
    bool visible{false};
};

/// Renders a selectable move/rotate transform gizmo without depending on CAD modules.
class TransformGizmoRenderer
{
public:
    /// Replace the current gizmo state.
    void setState(const TransformGizmoState& state);

    /// Current gizmo state.
    const TransformGizmoState& state() const { return m_state; }

    /// Display the transform gizmo in the supplied OCC context.
    void render(const Handle(AIS_InteractiveContext)& context, bool updateViewer = true);

    /// Remove displayed gizmo objects from the supplied OCC context.
    void clearObjects(const Handle(AIS_InteractiveContext)& context, bool updateViewer = true);

    /// Clear state and displayed object maps.
    void clear();

    /// Return true when the currently detected object belongs to the transform gizmo.
    bool detectedPart(const Handle(AIS_InteractiveContext)& context, int* operation, int* axis) const;

    /// Project a world-space axis into the view and return its screen direction.
    bool axisScreenVector(const Handle(V3d_View)& view, int axis, double* outX, double* outY) const;

private:
    TransformGizmoState m_state;
    QHash<QString, Handle(AIS_Shape)> m_objectsByKey;
    QHash<const AIS_InteractiveObject*, QString> m_keysByObject;
};

} // namespace lcnc::view
