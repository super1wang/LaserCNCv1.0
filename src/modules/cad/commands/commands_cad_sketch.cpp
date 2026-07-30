#include "modules/cad/commands/commands_cad.h"

#include "app/app_command_context.h"
#include "core/logging/logger.h"
#include "modules/cad/cad_module.h"

#include <QAction>
#include <QIcon>

// ── CmdNewSketch ──────────────────────────────────────────────────────────────
CmdNewSketch::CmdNewSketch(IAppContext* ctx) : CommandBase(ctx)
{
    // 中文翻译：新建草图
    auto* a = new QAction(QIcon(":/icons/sketch.svg"), tr("Create a new sketch"), this);
    // 中文翻译：在默认 XY 平面新建草图
    a->setStatusTip(tr("Create a new sketch in the default XY plane"));
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
    // 中文翻译：退出草图
    auto* a = new QAction(QIcon(":/icons/exit_sketch.svg"), tr("Exit sketch"), this);
    // 中文翻译：完成当前草图并生成建模轮廓
    a->setStatusTip(tr("Complete the current sketch and generate the modeling outline"));
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
    // 中文翻译：取消草图
    auto* a = new QAction(QIcon(":/icons/close.svg"), tr("Cancel sketch"), this);
    // 中文翻译：取消当前草图，丢弃尚未提交的几何
    a->setStatusTip(tr("Cancels the current sketch, discarding uncommitted geometry"));
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
    // 中文翻译：点
    auto* a = new QAction(QIcon(":/icons/sketch.svg"), tr("point"), this);
    // 中文翻译：草图工具：插入点
    a->setStatusTip(tr("Sketch Tools: Insertion Point"));
    setAction(a);
}
void CmdSketchPoint::execute() { activateSketchTool(context(), 1); }
bool CmdSketchPoint::isEnabled() const { return sketchToolEnabled(context()); }

CmdSketchLine::CmdSketchLine(IAppContext* ctx) : CommandBase(ctx)
{
    // 中文翻译：直线
    auto* a = new QAction(QIcon(":/icons/sketch.svg"), tr("straight line"), this);
    // 中文翻译：草图工具：绘制直线段
    a->setStatusTip(tr("Sketch Tools: Draw Straight Segments"));
    setAction(a);
}
void CmdSketchLine::execute() { activateSketchTool(context(), 2); }
bool CmdSketchLine::isEnabled() const { return sketchToolEnabled(context()); }

CmdSketchArc::CmdSketchArc(IAppContext* ctx) : CommandBase(ctx)
{
    // 中文翻译：圆弧
    auto* a = new QAction(QIcon(":/icons/sketch.svg"), tr("Arc"), this);
    // 中文翻译：草图工具：三点圆弧
    a->setStatusTip(tr("Sketch Tools: Three-Point Arc"));
    setAction(a);
}
void CmdSketchArc::execute() { activateSketchTool(context(), 3); }
bool CmdSketchArc::isEnabled() const { return sketchToolEnabled(context()); }

CmdSketchCircleTool::CmdSketchCircleTool(IAppContext* ctx) : CommandBase(ctx)
{
    // 中文翻译：圆
    auto* a = new QAction(QIcon(":/icons/sketch.svg"), tr("round"), this);
    // 中文翻译：草图工具：圆
    a->setStatusTip(tr("Sketch Tools: Circle"));
    setAction(a);
}
void CmdSketchCircleTool::execute() { activateSketchTool(context(), 4); }
bool CmdSketchCircleTool::isEnabled() const { return sketchToolEnabled(context()); }

CmdSketchRectangleTool::CmdSketchRectangleTool(IAppContext* ctx) : CommandBase(ctx)
{
    // 中文翻译：矩形
    auto* a = new QAction(QIcon(":/icons/sketch.svg"), tr("Rectangle"), this);
    // 中文翻译：草图工具：矩形
    a->setStatusTip(tr("Sketch Tools: Rectangle"));
    setAction(a);
}
void CmdSketchRectangleTool::execute() { activateSketchTool(context(), 5); }
bool CmdSketchRectangleTool::isEnabled() const { return sketchToolEnabled(context()); }

CmdSketchPolygon::CmdSketchPolygon(IAppContext* ctx) : CommandBase(ctx)
{
    // 中文翻译：多边形
    auto* a = new QAction(QIcon(":/icons/sketch.svg"), tr("polygon"), this);
    // 中文翻译：草图工具：等边多边形
    a->setStatusTip(tr("Sketch Tools: Equilateral Polygon"));
    setAction(a);
}
void CmdSketchPolygon::execute() { activateSketchTool(context(), 6); }
bool CmdSketchPolygon::isEnabled() const { return sketchToolEnabled(context()); }
