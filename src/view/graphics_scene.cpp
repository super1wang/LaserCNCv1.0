#include "view/graphics_scene.h"

#include "core/logging/logger.h"

#include <OpenGl_GraphicDriver.hxx>
#include <OpenGl_Context.hxx>
#include <Aspect_DisplayConnection.hxx>
#include <V3d_DirectionalLight.hxx>
#include <V3d_AmbientLight.hxx>
#include <Prs3d_LineAspect.hxx>
#include <Prs3d_ShadingAspect.hxx>
#include <Prs3d_Drawer.hxx>
#include <Graphic3d_AspectFillArea3d.hxx>

namespace {

void configureOpenGlDriver(const Handle(OpenGl_GraphicDriver)& driver)
{
    if (driver.IsNull())
        return;

    OpenGl_Caps& options = driver->ChangeOptions();
    options.buffersNoSwap = false;
    options.contextDebug = false;
    options.contextSyncDebug = false;
    options.contextNoAccel = false;
    options.contextNoExtensions = false;
    options.contextCompatible = false;
    options.ffpEnable = false;
    options.vboDisable = false;
    options.keepArrayData = false;
    options.swapInterval = 1;

    driver->EnableVBO(Standard_True);
    driver->SetVerticalSync(true);

    LCNC_INFO(lcnc::LogCode::Generic,
              "OCC OpenGL driver configured: vboDisabled={} coreCompatible={} ffp={} noAccel={} noExtensions={} vsync={}",
              static_cast<bool>(options.vboDisable),
              static_cast<bool>(options.contextCompatible),
              static_cast<bool>(options.ffpEnable),
              static_cast<bool>(options.contextNoAccel),
              static_cast<bool>(options.contextNoExtensions),
              driver->IsVerticalSync());
}

} // namespace

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
    configureOpenGlDriver(driver);

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

    // 选中高亮：红色（线框模式 → 红色线框；着色模式 → 红色填充 + 红色边线）。
    // SetDisplayMode(-1) 表示沿用对象当前 displayMode，避免选中时被强制切换。
    {
        const Quantity_Color hlColor(Quantity_NOC_RED);
        Handle(Prs3d_Drawer) selStyle =
            m_context->HighlightStyle(Prs3d_TypeOfHighlight_Selected);
        if (!selStyle.IsNull()) {
            selStyle->SetColor(hlColor);
            selStyle->SetDisplayMode(-1);
            selStyle->SetLineAspect(
                new Prs3d_LineAspect(hlColor, Aspect_TOL_SOLID, 2.0));
            // 红色 ShadingAspect → 着色模式下选中物呈红色填充。
            Handle(Prs3d_ShadingAspect) shading = new Prs3d_ShadingAspect();
            shading->SetColor(hlColor);
            selStyle->SetShadingAspect(shading);
            // 红色边线 → 着色带边模式下高亮一致。
            selStyle->SetFaceBoundaryDraw(true);
            selStyle->SetFaceBoundaryAspect(
                new Prs3d_LineAspect(hlColor, Aspect_TOL_SOLID, 2.0));
        }
        // Hover 高亮保持区分：橙色，避免与选中色混淆。
        Handle(Prs3d_Drawer) dynStyle =
            m_context->HighlightStyle(Prs3d_TypeOfHighlight_Dynamic);
        if (!dynStyle.IsNull()) {
            dynStyle->SetColor(Quantity_NOC_ORANGE);
            dynStyle->SetDisplayMode(-1);
        }
    }
}

void GraphicsScene::setDefaultLighting()
{
    // Add lights first, then enable them — SetLightOn() only activates
    // lights already in the viewer's defined-light list.
    Handle(V3d_DirectionalLight) dirLight = new V3d_DirectionalLight(
        V3d_XposYnegZpos, Quantity_NOC_WHITE, Standard_True);
    Handle(V3d_AmbientLight) ambLight = new V3d_AmbientLight(
        Quantity_Color(0.3, 0.3, 0.3, Quantity_TOC_RGB));
    m_viewer->AddLight(dirLight);
    m_viewer->AddLight(ambLight);
    m_viewer->SetLightOn();  // Enable all defined lights (must call AFTER AddLight)
}

void GraphicsScene::setGradientBackground(const Quantity_Color& top,
                                          const Quantity_Color& bottom)
{
    // Background is applied per-view in WidgetOccView::initOccView()
    // V3d_Viewer no longer supports SetGradientBackground in OCCT 7.7+
    (void)top; (void)bottom;
}

void GraphicsScene::logOpenGlContextState(const char* owner) const
{
    static bool s_loggedValidContext = false;
    if (s_loggedValidContext || m_viewer.IsNull())
        return;

    const Handle(OpenGl_GraphicDriver) driver =
        Handle(OpenGl_GraphicDriver)::DownCast(m_viewer->Driver());
    if (driver.IsNull()) {
        LCNC_WARN(lcnc::LogCode::Generic,
                  "OCC OpenGL diagnostics skipped: viewer driver is not OpenGl_GraphicDriver");
        return;
    }

    const Handle(OpenGl_Context)& glContext = driver->GetSharedContext(false);
    if (glContext.IsNull() || !glContext->IsValid()) {
        LCNC_DEBUG(lcnc::LogCode::Generic,
                   "OCC OpenGL context not initialized yet for {}",
                   owner ? owner : "unknown");
        return;
    }

    const OpenGl_Caps& options = driver->Options();
    const bool vboSupported = glContext->core15fwd != nullptr;
    const bool vboEnabled = glContext->ToUseVbo();
    LCNC_INFO(lcnc::LogCode::Generic,
              "OCC OpenGL context ready [{}]: version={}.{} vendor='{}' coreProfile={} vboSupported={} vboDisabled={} vboEnabled={} maxMsaaSamples={}",
              owner ? owner : "unknown",
              glContext->VersionMajor(),
              glContext->VersionMinor(),
              glContext->Vendor().ToCString(),
              glContext->core11ffp == nullptr,
              vboSupported,
              static_cast<bool>(options.vboDisable),
              vboEnabled,
              glContext->MaxMsaaSamples());
    if (!vboEnabled) {
        LCNC_WARN(lcnc::LogCode::Generic,
                  "OCC VBO is not active; rendering may fall back to client arrays");
    }
    s_loggedValidContext = true;
}

// ── Shape display ──────────────────────────────────────────────────────────────
Handle(AIS_Shape) GraphicsScene::displayShape(const TopoDS_Shape& shape,
                                              bool fitAll,
                                              bool /*selectable*/,
                                              bool updateViewer)
{
    Handle(AIS_Shape) aisShape = new AIS_Shape(shape);
    // 使用 context 当前默认 displayMode 显示对象，避免强制 AIS_Shaded 覆盖
    // 用户在"线框/着色/带边着色"中选择的全局模式。新对象以 -1（context 默认）显示。
    m_context->Display(aisShape, -1, 0, false);
    (void)fitAll; // FitAll is handled per-view in WidgetOccView
    if (updateViewer)
        m_context->UpdateCurrentViewer();
    return aisShape;
}

void GraphicsScene::redisplayShape(const Handle(AIS_Shape)& aisShape, bool updateViewer)
{
    m_context->Redisplay(aisShape, false);
    if (updateViewer)
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
                                  const Quantity_Color&     color,
                                  bool                      updateViewer)
{
    m_context->SetColor(aisShape, color, false);
    if (updateViewer)
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
