#include "app/commands_display.h"

#include <QAction>

#include "app/widget_occ_view.h"
#include "gui/gui_document.h"
#include "gui/gui_application.h"
#include "graphics/graphics_scene.h"

#include <AIS_InteractiveContext.hxx>
#include <V3d_View.hxx>

// ── Helper: get the active view (safe) ────────────────────────────────────────
static Handle(V3d_View) activeView(IAppContext* ctx)
{
    // GuiDocument → scene → viewer → first active view
    if (auto* gd = ctx->activeGuiDocument()) {
        auto& viewer = gd->scene()->viewer();
        if (!viewer.IsNull()) {
            viewer->InitActiveLights();
            // We can't directly get an active view from V3d_Viewer in OCCT API;
            // views are owned by WidgetOccView.  Return null; callers that
            // need the view should obtain it from WidgetOccView directly.
        }
    }
    return Handle(V3d_View)();
}

// ── CmdFitAll ─────────────────────────────────────────────────────────────────
CmdFitAll::CmdFitAll(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/fit_all.svg"), tr("适合视图"), this);
    a->setShortcut(Qt::Key_F);
    a->setStatusTip(tr("调整视图以显示所有对象"));
    setAction(a);
}

void CmdFitAll::execute()
{
    // Handled by WidgetOccView listening to this action's triggered() signal.
    // Nothing else needed here — the connection is made in MainWindow.
}

// ── CmdViewOrient ─────────────────────────────────────────────────────────────
CmdViewOrient::CmdViewOrient(IAppContext* ctx,
                              V3d_TypeOfOrientation orient,
                              const QString& label,
                              const QIcon& icon)
    : CommandBase(ctx)
    , m_orient(orient)
{
    auto* a = new QAction(icon, label, this);
    setAction(a);
}

void CmdViewOrient::execute()
{
    // Handled by WidgetOccView.
}

// ── CmdToggleWireframe ────────────────────────────────────────────────────────
CmdToggleWireframe::CmdToggleWireframe(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/wireframe.svg"), tr("线框模式"), this);
    a->setCheckable(true);
    setAction(a);
}

void CmdToggleWireframe::execute()
{
    if (auto* gd = context()->activeGuiDocument()) {
        auto ctx = gd->scene()->context();
        ctx->SetDisplayMode(AIS_WireFrame, true);
    }
}

// ── CmdToggleShaded ───────────────────────────────────────────────────────────
CmdToggleShaded::CmdToggleShaded(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/shaded.svg"), tr("着色模式"), this);
    a->setCheckable(true);
    a->setChecked(true);
    setAction(a);
}

void CmdToggleShaded::execute()
{
    if (auto* gd = context()->activeGuiDocument()) {
        auto ctx = gd->scene()->context();
        ctx->SetDisplayMode(AIS_Shaded, true);
    }
}

// ── CmdToggleShadedWithEdges ──────────────────────────────────────────────────
CmdToggleShadedWithEdges::CmdToggleShadedWithEdges(IAppContext* ctx)
    : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/shaded_edges.svg"), tr("带边着色"), this);
    a->setCheckable(true);
    setAction(a);
}

void CmdToggleShadedWithEdges::execute()
{
    if (auto* gd = context()->activeGuiDocument()) {
        auto& occCtx = gd->scene()->context();
        occCtx->SetDisplayMode(AIS_Shaded, true);
        occCtx->DefaultDrawer()->SetFaceBoundaryDraw(true);
    }
}
