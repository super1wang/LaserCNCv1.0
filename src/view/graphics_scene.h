#pragma once

#include <QObject>
#include <Standard_Handle.hxx>
#include <AIS_InteractiveContext.hxx>
#include <V3d_Viewer.hxx>
#include <AIS_InteractiveObject.hxx>
#include <AIS_Shape.hxx>
#include <TopoDS_Shape.hxx>
#include <Quantity_Color.hxx>

/**
 * @brief Encapsulates the OCC 3D rendering scene.
 *
 * Owns the V3d_Viewer and AIS_InteractiveContext.  One GraphicsScene is
 * shared by all views (WidgetOccView instances) that display the same
 * document.  Shape display / removal / highlighting are routed through here.
 */
class GraphicsScene : public QObject
{
    Q_OBJECT
public:
    explicit GraphicsScene(QObject* parent = nullptr);
    ~GraphicsScene() override;

    // ── OCC accessors ─────────────────────────────────────────────────────────
    const Handle(V3d_Viewer)&           viewer()  const { return m_viewer;  }
    const Handle(AIS_InteractiveContext)& context() const { return m_context; }

    // ── Shape display ─────────────────────────────────────────────────────────
    Handle(AIS_Shape) displayShape(const TopoDS_Shape& shape,
                                   bool fitAll  = false,
                                   bool selectable = true,
                                   bool updateViewer = true);

    void redisplayShape(const Handle(AIS_Shape)& aisShape, bool updateViewer = true);
    void eraseShape(const Handle(AIS_Shape)& aisShape);
    void eraseAll();

    void setShapeColor(const Handle(AIS_Shape)& aisShape,
                       const Quantity_Color&     color,
                       bool                      updateViewer = true);

    // ── Selection ─────────────────────────────────────────────────────────────
    void clearSelection();

    // ── Generic AIS objects ───────────────────────────────────────────────────
    void displayObject(const Handle(AIS_InteractiveObject)& obj, bool update = true);
    void eraseObject(const Handle(AIS_InteractiveObject)& obj,   bool update = true);

    // ── Lighting / background ─────────────────────────────────────────────────
    void setDefaultLighting();
    void setGradientBackground(const Quantity_Color& top,
                               const Quantity_Color& bottom);
    void logOpenGlContextState(const char* owner) const;

signals:
    void selectionChanged();

private:
    void init();

    Handle(V3d_Viewer)            m_viewer;
    Handle(AIS_InteractiveContext) m_context;
};
