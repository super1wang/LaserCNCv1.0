#include "modules/cad/commands/commands_cad.h"

#include "app/app_command_context.h"
#include "modules/cad/cad_module.h"

#include <QAction>
#include <QIcon>

// ── CmdCreateBox ──────────────────────────────────────────────────────────────
CmdCreateBox::CmdCreateBox(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/box.svg"), tr("长方体"), this);
    a->setStatusTip(tr("创建长方体基本体"));
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
    auto* a = new QAction(QIcon(":/icons/cylinder.svg"), tr("圆柱体"), this);
    a->setStatusTip(tr("创建圆柱体基本体"));
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
    auto* a = new QAction(QIcon(":/icons/sphere.svg"), tr("球体"), this);
    a->setStatusTip(tr("创建球体基本体"));
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
    auto* a = new QAction(QIcon(":/icons/cone.svg"), tr("圆锥体"), this);
    a->setStatusTip(tr("创建圆锥体（或截锥体）"));
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
    auto* a = new QAction(QIcon(":/icons/torus.svg"), tr("圆环体"), this);
    a->setStatusTip(tr("创建圆环体基本体"));
    setAction(a);
}

void CmdCreateTorus::execute()
{
    if (auto* cad = context()->cadModule())
        cad->requestPrimitiveTool(4);
    context()->updateCommandStates();
}
