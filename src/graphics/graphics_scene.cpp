#include "graphics/graphics_scene.h"

#include <OpenGl_GraphicDriver.hxx>
#include <Aspect_DisplayConnection.hxx>
#include <V3d_DirectionalLight.hxx>
#include <V3d_AmbientLight.hxx>
#include <Prs3d_LineAspect.hxx>
#include <Graphic3d_AspectFillArea3d.hxx>

GraphicsScene::GraphicsScene(QObject* parent)
    : QObject(parent)
{
    init();
}

GraphicsScene::~GraphicsScene() = default;

void GraphicsScene::init()
{
    // Create OpenGL graphics driver
    Handle(Aspect_DisplayConnection) displayConn = new Aspect_DisplayConnection();
    Handle(OpenGl_GraphicDriver) driver = new OpenGl_GraphicDriver(displayConn, false);
    driver->ChangeOptions().buffersNoSwap     = false;
    driver->ChangeOptions().contextDebug      = false;

    // Create viewer
    m_viewer = new V3d_Viewer(driver);
    m_viewer->SetDefaultViewSize(1000.0);
    m_viewer->SetDefaultViewProj(V3d_XposYnegZpos);
    m_viewer->SetComputedMode(false);
    m_viewer->SetDefaultShadingModel(Graphic3d_TypeOfShadingModel_Phong);

    setDefaultLighting();
    setGradientBackground(Quantity_Color(0.30, 0.35, 0.42, Quantity_TOC_RGB),
                          Quantity_Color(0.12, 0.15, 0.20, Quantity_TOC_RGB));

    // Create interactive context
    m_context = new AIS_InteractiveContext(m_viewer);
    m_context->SetDisplayMode(AIS_Shaded, false);
    m_context->DefaultDrawer()->SetFaceBoundaryDraw(true);
    m_context->DefaultDrawer()->FaceBoundaryAspect()
        ->SetColor(Quantity_NOC_GRAY40);
}

void GraphicsScene::setDefaultLighting()
{
    m_viewer->SetLightOn();
    m_viewer->AddLight(new V3d_DirectionalLight(
        V3d_XposYnegZpos, Quantity_NOC_WHITE, true));
    m_viewer->AddLight(new V3d_AmbientLight(Quantity_Color(0.3, 0.3, 0.3, Quantity_TOC_RGB)));
}

void GraphicsScene::setGradientBackground(const Quantity_Color& top,
                                          const Quantity_Color& bottom)
{
    // Background is applied per-view in WidgetOccView::initOccView()
    // V3d_Viewer no longer supports SetGradientBackground in OCCT 7.7+
    (void)top; (void)bottom;
}

// ── Shape display ──────────────────────────────────────────────────────────────
Handle(AIS_Shape) GraphicsScene::displayShape(const TopoDS_Shape& shape,
                                              bool fitAll,
                                              bool /*selectable*/)
{
    Handle(AIS_Shape) aisShape = new AIS_Shape(shape);
    m_context->Display(aisShape, AIS_Shaded, 0, false);
    m_context->SetDisplayMode(aisShape, AIS_Shaded, false);
    (void)fitAll; // FitAll is handled per-view in WidgetOccView
    m_context->UpdateCurrentViewer();
    return aisShape;
}

void GraphicsScene::redisplayShape(const Handle(AIS_Shape)& aisShape)
{
    m_context->Redisplay(aisShape, false);
    m_context->UpdateCurrentViewer();
}

void GraphicsScene::eraseShape(const Handle(AIS_Shape)& aisShape)
{
    m_context->Erase(aisShape, false);
    m_context->UpdateCurrentViewer();
}

void GraphicsScene::eraseAll()
{
    m_context->EraseAll(false);
    m_context->UpdateCurrentViewer();
}

void GraphicsScene::setShapeColor(const Handle(AIS_Shape)& aisShape,
                                  const Quantity_Color&     color)
{
    m_context->SetColor(aisShape, color, false);
    m_context->UpdateCurrentViewer();
}

void GraphicsScene::clearSelection()
{
    m_context->ClearSelected(false);
    m_context->UpdateCurrentViewer();
}

void GraphicsScene::displayObject(const Handle(AIS_InteractiveObject)& obj, bool update)
{
    m_context->Display(obj, update);
}

void GraphicsScene::eraseObject(const Handle(AIS_InteractiveObject)& obj, bool update)
{
    m_context->Erase(obj, update);
}
