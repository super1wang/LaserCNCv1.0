#include "modules/cad/commands/commands_cad.h"

#include "app/app_command_context.h"
#include "modules/cad/cad_module.h"

#include <QAction>
#include <QIcon>

// ── CmdCreateBox ──────────────────────────────────────────────────────────────
CmdCreateBox::CmdCreateBox(IAppContext* ctx) : CommandBase(ctx)
{
    // 中文翻译：长方体
    auto* a = new QAction(QIcon(":/icons/box.svg"), tr("cuboid"), this);
    // 中文翻译：创建长方体基本体
    a->setStatusTip(tr("Create a cuboid primitive"));
    setAction(a);
}

void CmdCreateBox::execute()
{
    if (auto* cad = context()->cadModule())
        cad->requestPrimitiveTool(0);
    context()->updateCommandStates();
}

// ── CmdCreateCylinder ─────────────────────────────────────────────────────────
CmdCreateCylinder::CmdCreateCylinder(IAppContext* ctx) : CommandBase(ctx)
{
    // 中文翻译：圆柱体
    auto* a = new QAction(QIcon(":/icons/cylinder.svg"), tr("cylinder"), this);
    // 中文翻译：创建圆柱体基本体
    a->setStatusTip(tr("Create a cylinder primitive"));
    setAction(a);
}

void CmdCreateCylinder::execute()
{
    if (auto* cad = context()->cadModule())
        cad->requestPrimitiveTool(1);
    context()->updateCommandStates();
}

// ── CmdCreateSphere ───────────────────────────────────────────────────────────
CmdCreateSphere::CmdCreateSphere(IAppContext* ctx) : CommandBase(ctx)
{
    // 中文翻译：球体
    auto* a = new QAction(QIcon(":/icons/sphere.svg"), tr("sphere"), this);
    // 中文翻译：创建球体基本体
    a->setStatusTip(tr("Create a sphere primitive"));
    setAction(a);
}

void CmdCreateSphere::execute()
{
    if (auto* cad = context()->cadModule())
        cad->requestPrimitiveTool(2);
    context()->updateCommandStates();
}

// ── CmdCreateCone ─────────────────────────────────────────────────────────────
CmdCreateCone::CmdCreateCone(IAppContext* ctx) : CommandBase(ctx)
{
    // 中文翻译：圆锥体
    auto* a = new QAction(QIcon(":/icons/cone.svg"), tr("cone"), this);
    // 中文翻译：创建圆锥体（或截锥体）
    a->setStatusTip(tr("Create a cone (or frustum)"));
    setAction(a);
}

void CmdCreateCone::execute()
{
    if (auto* cad = context()->cadModule())
        cad->requestPrimitiveTool(3);
    context()->updateCommandStates();
}

// ── CmdCreateTorus ────────────────────────────────────────────────────────────
CmdCreateTorus::CmdCreateTorus(IAppContext* ctx) : CommandBase(ctx)
{
    // 中文翻译：圆环体
    auto* a = new QAction(QIcon(":/icons/torus.svg"), tr("torus"), this);
    // 中文翻译：创建圆环体基本体
    a->setStatusTip(tr("Create a torus primitive"));
    setAction(a);
}

void CmdCreateTorus::execute()
{
    if (auto* cad = context()->cadModule())
        cad->requestPrimitiveTool(4);
    context()->updateCommandStates();
}
