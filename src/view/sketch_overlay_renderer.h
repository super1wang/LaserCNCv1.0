#pragma once

#include <QColor>
#include <QHash>
#include <QString>
#include <QVector>

#include <Standard_Handle.hxx>

class AIS_InteractiveContext;
class AIS_InteractiveObject;
class AIS_Shape;

namespace lcnc::view {

/// Lightweight sketch overlay item that does not depend on modules/cad types.
struct SketchOverlayItem {
    QString key;
    int kind{0};
    int plane{0};
    QVector<double> params;
    bool visible{true};
    bool draggable{false};
    QColor color{Qt::cyan};
};

/// Renders sketch overlay DTOs as selectable transient AIS objects.
class SketchOverlayRenderer
{
public:
    /// Replace all overlay items for the active document view.
    void setItems(const QVector<SketchOverlayItem>& items);

    /// Current overlay items, primarily for tests and future AIS rendering.
    const QVector<SketchOverlayItem>& items() const { return m_items; }

    /// Display the current overlay items in the supplied OCC context.
    void render(const Handle(AIS_InteractiveContext)& context, bool updateViewer = true);

    /// Remove currently displayed overlay AIS objects from the supplied context.
    void clearObjects(const Handle(AIS_InteractiveContext)& context, bool updateViewer = true);

    /// Clear all overlay items and displayed objects.
    void clear();

    /// Return the overlay key currently detected by the OCC context, if any.
    QString detectedKey(const Handle(AIS_InteractiveContext)& context) const;

    /// Return the sketch plane kind for an overlay key, or 0 when absent.
    int planeForKey(const QString& key) const;

    /// Return true when an overlay key is allowed to start a drag gesture.
    bool isDraggable(const QString& key) const;

private:
    QVector<SketchOverlayItem> m_items;
    QHash<QString, Handle(AIS_Shape)> m_objectsByKey;
    QHash<const AIS_InteractiveObject*, QString> m_keysByObject;
};

} // namespace lcnc::view