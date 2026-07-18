#include "app/commands/commands_display.h"
#include "app/app_command_context.h"
#include "app/dialog/dialog_options.h"

#include <QAction>
#include <QActionGroup>

#include "view/widget_occ_view.h"
#include "view/gui_document.h"
#include "view/gui_application.h"
#include "view/graphics_scene.h"
#include "view/rendering_manager.h"
#include "view/world_axes_renderer.h"
#include "core/logging/logger.h"
#include "core/kernel/kernel.h"

#include <AIS_InteractiveContext.hxx>
#include <V3d_View.hxx>

#include <memory>

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
    if (auto* view = context() ? context()->occView() : nullptr)
        view->fitAll();
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
    if (auto* view = context() ? context()->occView() : nullptr)
        view->setOrientation(m_orient);
}

// ── Helper: apply a given displayMode to current view only ───────────────────
static void applyDisplayModeToCurrentView(IAppContext* ctx, int displayMode, bool faceBoundary)
{
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "applyDisplayModeToCurrentView mode={} edges={}",
               displayMode, faceBoundary);
    if (!ctx) {
        LCNC_WARN(lcnc::LogCode::InternalUnexpectedState,
                  "applyDisplayModeToCurrentView: context null");
        return;
    }

    GuiDocument* gd = ctx->activeGuiDocument();
    if (!gd || !gd->renderingManager()) {
        LCNC_WARN(lcnc::LogCode::InternalUnexpectedState,
                  "applyDisplayModeToCurrentView: no active GuiDocument");
        return;
    }
    gd->renderingManager()->setRuntimeDisplayMode(displayMode, faceBoundary);
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
    LCNC_DEBUG(lcnc::LogCode::Generic, "CmdToggleWireframe::execute");
    applyDisplayModeToCurrentView(context(), AIS_WireFrame, false);
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
    LCNC_DEBUG(lcnc::LogCode::Generic, "CmdToggleShaded::execute");
    applyDisplayModeToCurrentView(context(), AIS_Shaded, false);
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
    LCNC_DEBUG(lcnc::LogCode::Generic, "CmdToggleShadedWithEdges::execute");
    applyDisplayModeToCurrentView(context(), AIS_Shaded, true);
}

// ── CmdToggleWorldAxes ────────────────────────────────────────────────────────

CmdToggleWorldAxes::CmdToggleWorldAxes(IAppContext* ctx) : CommandBase(ctx)
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "CmdToggleWorldAxes ctor");
    auto* a = new QAction(QIcon(":/icons/machine.svg"), tr("坐标系"), this);
    a->setCheckable(true);
    a->setChecked(false);
    a->setStatusTip(tr("以世界 0 点为中心绘制持久 XYZ 坐标轴；同时显示在机台与所有工件视图。"));
    setAction(a);
}

void CmdToggleWorldAxes::execute()
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "CmdToggleWorldAxes::execute begin");
    auto& renderer = lcnc::view::WorldAxesRenderer::instance();
    auto* guiApp = lcnc::Kernel::current().guiApp();
    if (!guiApp) {
        LCNC_WARN(lcnc::LogCode::Generic,
                  "CmdToggleWorldAxes: GuiApplication unavailable");
        if (action())
            action()->setChecked(false);
        return;
    }
    if (auto* workspace = guiApp->activeGuiDocument(); workspace && workspace->scene())
        renderer.attach(workspace->scene());

    const bool wantVisible = action() && action()->isChecked();
    renderer.setGloballyVisible(wantVisible);
    LCNC_INFO(lcnc::LogCode::Generic,
              "World axes toggled: visible={}", wantVisible);
}

// ── CmdShowOptions ────────────────────────────────────────────────────────────
CmdShowOptions::CmdShowOptions(IAppContext* ctx) : CommandBase(ctx)
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "CmdShowOptions ctor");
    auto* a = new QAction(QIcon(":/icons/options.svg"), tr("应用程序选项"), this);
    a->setStatusTip(tr("打开应用程序选项对话框（图形渲染 / 选择高亮 / 应用程序 / 机台构型）"));
    setAction(a);
}

void CmdShowOptions::execute()
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "CmdShowOptions::execute begin");
    lcnc::DialogOptions dlg(nullptr);
    dlg.exec();
    LCNC_DEBUG(lcnc::LogCode::Generic, "CmdShowOptions::execute end");
}
