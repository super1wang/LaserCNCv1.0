#include "modules/cad/commands/commands_cad.h"

#include "app/app_command_context.h"
#include "view/widget_occ_view.h"

#include <QAction>
#include <QIcon>

// ── CAD view modeling aids ───────────────────────────────────────────────────
CmdToggleCadGrid::CmdToggleCadGrid(IAppContext* ctx) : CommandBase(ctx)
{
    // 中文翻译：网格
    auto* a = new QAction(QIcon("themeicons:grid.svg"), tr("grid"), this);
    a->setCheckable(true);
    // 中文翻译：显示或隐藏建模网格
    a->setStatusTip(tr("Show or hide the modeling mesh"));
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
    // 中文翻译：网格吸附
    auto* a = new QAction(QIcon("themeicons:snap.svg"), tr("Grid adsorption"), this);
    a->setCheckable(true);
    // 中文翻译：启用或关闭网格点吸附
    a->setStatusTip(tr("Turn grid point snapping on or off"));
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
    // 中文翻译：无抓取
    auto* a = new QAction(tr("No crawling"), this);
    a->setCheckable(true);
    // 中文翻译：关闭 CAD 几何抓取
    a->setStatusTip(tr("Turn off CAD geometry grabbing"));
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
    // 中文翻译：顶点
    auto* a = new QAction(tr("vertex"), this);
    a->setCheckable(true);
    // 中文翻译：抓取 CAD 顶点
    a->setStatusTip(tr("Grab CAD vertices"));
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
    // 中文翻译：边
    auto* a = new QAction(tr("side"), this);
    a->setCheckable(true);
    // 中文翻译：抓取 CAD 边
    a->setStatusTip(tr("Grab CAD edges"));
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
    // 中文翻译：面
    auto* a = new QAction(tr("noodles"), this);
    a->setCheckable(true);
    a->setChecked(true);
    // 中文翻译：抓取 CAD 面
    a->setStatusTip(tr("Grab CAD face"));
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
