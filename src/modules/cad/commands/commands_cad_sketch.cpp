#include "modules/cad/commands/commands_cad.h"

#include "core/logging/logger.h"
#include "modules/cad/cad_module.h"

#include <QAction>
#include <QIcon>

// ── CmdNewSketch ──────────────────────────────────────────────────────────────
CmdNewSketch::CmdNewSketch(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/sketch.svg"), tr("新建草图"), this);
    a->setStatusTip(tr("在默认 XY 平面新建草图"));
    setAction(a);
}

void CmdNewSketch::execute()
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "CmdNewSketch::execute begin");
    if (context()->cadModule()->beginSketch(0))
        context()->updateCommandStates();
    LCNC_DEBUG(lcnc::LogCode::Generic, "CmdNewSketch::execute end");
}

// ── CmdFinishSketch ───────────────────────────────────────────────────────────
CmdFinishSketch::CmdFinishSketch(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/exit_sketch.svg"), tr("退出草图"), this);
    a->setStatusTip(tr("完成当前草图并生成建模轮廓"));
    setAction(a);
}

void CmdFinishSketch::execute()
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "CmdFinishSketch::execute begin");
    if (context()->cadModule()->finishSketch())
        context()->updateCommandStates();
    LCNC_DEBUG(lcnc::LogCode::Generic, "CmdFinishSketch::execute end");
}

// ── CmdCancelSketch ───────────────────────────────────────────────────────────
CmdCancelSketch::CmdCancelSketch(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/close.svg"), tr("取消草图"), this);
    a->setStatusTip(tr("取消当前草图，丢弃尚未提交的几何"));
    setAction(a);
}

void CmdCancelSketch::execute()
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "CmdCancelSketch::execute begin");
    context()->cadModule()->cancelModelingOperation();
    context()->updateCommandStates();
    LCNC_DEBUG(lcnc::LogCode::Generic, "CmdCancelSketch::execute end");
}

bool CmdCancelSketch::isEnabled() const
{
    return context()->cadModule()->isSketchEditing();
}

namespace {

void activateSketchTool(IAppContext* ctx, int toolKind)
{
    if (!ctx || !ctx->cadModule())
        return;
    ctx->cadModule()->setSketchTool(toolKind);
    ctx->updateCommandStates();
}

bool sketchToolEnabled(IAppContext* ctx)
{
    return ctx && ctx->cadModule() && ctx->cadModule()->isSketchEditing();
}

} // namespace

// ── Sketch tool palette commands ──────────────────────────────────────────────
CmdSketchPoint::CmdSketchPoint(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/sketch.svg"), tr("点"), this);
    a->setStatusTip(tr("草图工具：插入点"));
    setAction(a);
}
void CmdSketchPoint::execute() { activateSketchTool(context(), 1); }
bool CmdSketchPoint::isEnabled() const { return sketchToolEnabled(context()); }

CmdSketchLine::CmdSketchLine(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/sketch.svg"), tr("直线"), this);
    a->setStatusTip(tr("草图工具：绘制直线段"));
    setAction(a);
}
void CmdSketchLine::execute() { activateSketchTool(context(), 2); }
bool CmdSketchLine::isEnabled() const { return sketchToolEnabled(context()); }

CmdSketchArc::CmdSketchArc(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/sketch.svg"), tr("圆弧"), this);
    a->setStatusTip(tr("草图工具：三点圆弧"));
    setAction(a);
}
void CmdSketchArc::execute() { activateSketchTool(context(), 3); }
bool CmdSketchArc::isEnabled() const { return sketchToolEnabled(context()); }

CmdSketchCircleTool::CmdSketchCircleTool(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/sketch.svg"), tr("圆"), this);
    a->setStatusTip(tr("草图工具：圆"));
    setAction(a);
}
void CmdSketchCircleTool::execute() { activateSketchTool(context(), 4); }
bool CmdSketchCircleTool::isEnabled() const { return sketchToolEnabled(context()); }

CmdSketchRectangleTool::CmdSketchRectangleTool(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/sketch.svg"), tr("矩形"), this);
    a->setStatusTip(tr("草图工具：矩形"));
    setAction(a);
}
void CmdSketchRectangleTool::execute() { activateSketchTool(context(), 5); }
bool CmdSketchRectangleTool::isEnabled() const { return sketchToolEnabled(context()); }

CmdSketchPolygon::CmdSketchPolygon(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/sketch.svg"), tr("多边形"), this);
    a->setStatusTip(tr("草图工具：等边多边形"));
    setAction(a);
}
void CmdSketchPolygon::execute() { activateSketchTool(context(), 6); }
bool CmdSketchPolygon::isEnabled() const { return sketchToolEnabled(context()); }
