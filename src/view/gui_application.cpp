#include "view/gui_application.h"

#include "core/logging/logger.h"
#include "view/graphics_scene.h"
#include "view/gui_document.h"
#include "view/rendering_manager.h"

#include <AIS_InteractiveContext.hxx>
#include <AIS_ListIteratorOfListOfInteractive.hxx>
#include <AIS_ListOfInteractive.hxx>
#include <AIS_Shape.hxx>
#include <Prs3d_Drawer.hxx>

GuiApplication* GuiApplication::s_instance = nullptr;

GuiApplication::GuiApplication(QObject* parent)
    : QObject(parent)
    , m_workspaceGuiDocument(new GuiDocument(this))
{
    Q_ASSERT_X(!s_instance, "GuiApplication",
               "second GuiApplication instance — must be Kernel-owned only");
    s_instance = this;
    emit workspaceGuiDocumentReady();
}

GuiApplication::~GuiApplication()
{
    if (s_instance == this)
        s_instance = nullptr;
}

void GuiApplication::setCurrentDisplayMode(int displayMode, bool faceBoundary)
{
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "GuiApplication::setCurrentDisplayMode mode={} edges={}",
               displayMode, faceBoundary);
    m_currentDisplayMode = displayMode;
    m_currentFaceBoundary = faceBoundary;

    GuiDocument* workspace = workspaceGuiDocument();
    GraphicsScene* scene = workspace ? workspace->scene() : nullptr;
    if (!scene)
        return;

    const Handle(AIS_InteractiveContext)& ctx = scene->context();
    if (ctx.IsNull())
        return;

    ctx->DefaultDrawer()->SetFaceBoundaryDraw(faceBoundary);
    ctx->SetDisplayMode(displayMode, Standard_False);

    AIS_ListOfInteractive list;
    ctx->DisplayedObjects(list);
    for (AIS_ListIteratorOfListOfInteractive it(list); it.More(); it.Next()) {
        const Handle(AIS_InteractiveObject)& obj = it.Value();
        if (Handle(AIS_Shape) shape = Handle(AIS_Shape)::DownCast(obj); !shape.IsNull()) {
            ctx->SetDisplayMode(shape, displayMode, Standard_False);
            if (!shape->Attributes().IsNull())
                shape->Attributes()->SetFaceBoundaryDraw(faceBoundary);
            ctx->Redisplay(shape, Standard_False);
        }
    }
    ctx->UpdateCurrentViewer();
}

void GuiApplication::requestApplyRenderingSettings(
    const lcnc::RenderProfileSettings& cadProfile,
    const lcnc::RenderProfileSettings& camProfile,
    const lcnc::ColorSettings& colors,
    lcnc::view::RenderDirtyFlags dirtyFlags,
    bool applyCadViews,
    bool applyCamView)
{
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "GuiApplication::requestApplyRenderingSettings flags={} cad={} cam={}",
               static_cast<int>(dirtyFlags.toInt()), applyCadViews, applyCamView);
    if (!applyCadViews && !applyCamView)
        return;

    GuiDocument* workspace = workspaceGuiDocument();
    if (!workspace || !workspace->renderingManager())
        return;

    const bool preferCamProfile = applyCamView;
    workspace->renderingManager()->setMachineView(preferCamProfile);
    workspace->renderingManager()->configure(preferCamProfile ? camProfile : cadProfile, colors);
    workspace->renderingManager()->requestApply(dirtyFlags);
}
