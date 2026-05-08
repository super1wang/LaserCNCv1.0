#include "modules/cad/commands/commands_cad.h"

#include "app/app_command_context.h"
#include "view/widget_occ_view.h"

#include <QAction>
#include <QIcon>

// ── CAD view modeling aids ───────────────────────────────────────────────────
CmdToggleCadGrid::CmdToggleCadGrid(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/grid.svg"), tr("网格"), this);
    a->setCheckable(true);
    a->setStatusTip(tr("显示或隐藏建模网格"));
    setAction(a);
}

bool CmdToggleCadGrid::isEnabled() const
{
    return context()->occView() != nullptr;
}

void CmdToggleCadGrid::execute()
{
    if (auto* view = context()->occView())
        view->setGridVisible(action()->isChecked());
}

CmdToggleGridSnap::CmdToggleGridSnap(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/snap.svg"), tr("网格吸附"), this);
    a->setCheckable(true);
    a->setStatusTip(tr("启用或关闭网格点吸附"));
    setAction(a);
}

bool CmdToggleGridSnap::isEnabled() const
{
    return context()->occView() != nullptr;
}

void CmdToggleGridSnap::execute()
{
    if (auto* view = context()->occView())
        view->setGridSnapEnabled(action()->isChecked());
}

CmdSnapNone::CmdSnapNone(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(tr("无抓取"), this);
    a->setCheckable(true);
    a->setStatusTip(tr("关闭 CAD 几何抓取"));
    setAction(a);
}

bool CmdSnapNone::isEnabled() const
{
    return context()->occView() != nullptr;
}

void CmdSnapNone::execute()
{
    if (auto* view = context()->occView())
        view->setCadSnapMode(WidgetOccView::CadSnapMode::None);
}

CmdSnapVertex::CmdSnapVertex(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(tr("顶点"), this);
    a->setCheckable(true);
    a->setStatusTip(tr("抓取 CAD 顶点"));
    setAction(a);
}

bool CmdSnapVertex::isEnabled() const
{
    return context()->occView() != nullptr;
}

void CmdSnapVertex::execute()
{
    if (auto* view = context()->occView())
        view->setCadSnapMode(WidgetOccView::CadSnapMode::Vertex);
}

CmdSnapEdge::CmdSnapEdge(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(tr("边"), this);
    a->setCheckable(true);
    a->setStatusTip(tr("抓取 CAD 边"));
    setAction(a);
}

bool CmdSnapEdge::isEnabled() const
{
    return context()->occView() != nullptr;
}

void CmdSnapEdge::execute()
{
    if (auto* view = context()->occView())
        view->setCadSnapMode(WidgetOccView::CadSnapMode::Edge);
}

CmdSnapFace::CmdSnapFace(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(tr("面"), this);
    a->setCheckable(true);
    a->setChecked(true);
    a->setStatusTip(tr("抓取 CAD 面"));
    setAction(a);
}

bool CmdSnapFace::isEnabled() const
{
    return context()->occView() != nullptr;
}

void CmdSnapFace::execute()
{
    if (auto* view = context()->occView())
        view->setCadSnapMode(WidgetOccView::CadSnapMode::Face);
}
