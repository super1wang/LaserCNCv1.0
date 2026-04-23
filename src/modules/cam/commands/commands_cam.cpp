#include "modules/cam/commands/commands_cam.h"
#include "core/command/command_context.h"
#include "view/widget_occ_view.h"

#include "core/document/lcnc_application.h"
#include "core/document/lcnc_document.h"
#include "modules/cam/services/laser_toolpath.h"
#include "modules/cam/services/face_classifier.h"
#include "core/kinematics/machine_kinematics.h"
#include "core/document/xcaf_utils.h"
#include "view/gui_application.h"
#include "view/gui_document.h"
#include "view/graphics_scene.h"
#include "modules/cam/cam_module.h"

#include <QAction>
#include <QIcon>
#include <QMessageBox>
#include <QTimer>
#include <QToolTip>

#include <AIS_Shape.hxx>
#include <AIS_InteractiveContext.hxx>
#include <Quantity_Color.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Compound.hxx>
#include <BRep_Builder.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <TopExp_Explorer.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRep_Tool.hxx>
#include <StdSelect_BRepOwner.hxx>
#include <SelectMgr_EntityOwner.hxx>

#include <TDF_LabelSequence.hxx>
#include <XCAFDoc_ShapeTool.hxx>

// =============================================================================
// CmdGenerateToolpath
// =============================================================================

CmdGenerateToolpath::CmdGenerateToolpath(IAppContext* ctx)
    : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/toolpath.svg"), tr("生成刀路"), this);
    a->setStatusTip(tr("从挂载的工件中提取轮廓并生成激光刀路"));
    setAction(a);
}

bool CmdGenerateToolpath::isEnabled() const
{
    LcncDocument* doc = context()->machineDocument();
    return doc && doc->entityLabels(LcncDocument::EntityKind::Workpiece).Length() > 0;
}

void CmdGenerateToolpath::execute()
{
    CamModule* cam = context()->camModule();
    if (!cam->generateToolpath(cam->smoothAngle(),
                               cam->useFaceClassification(),
                               cam->deflection())) {
        QMessageBox::warning(nullptr, tr("生成刀路"),
            tr("机台文档中未找到工件，或未找到可用的轮廓边缘。"));
    }
    context()->updateCommandStates();
}

// =============================================================================
// CmdSetLeadIn
// =============================================================================

CmdSetLeadIn::CmdSetLeadIn(IAppContext* ctx)
    : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/toolpath_32.svg"), tr("选择引刀位置"), this);
    a->setStatusTip(tr("在3D视图中点击轮廓边缘选择引刀线起始位置"));
    setAction(a);
}

bool CmdSetLeadIn::isEnabled() const
{
    return context()->camModule()->hasToolpath();
}

void CmdSetLeadIn::execute()
{
    CamModule* cam = context()->camModule();
    WidgetOccView* occView = context()->occView();
    if (!occView)
        return;

    cam->requestMachineView();

    if (occView->isLeadInPickActive()) {
        occView->endLeadInPick();
        cam->cancelLeadInPreview();
        return;
    }

    occView->beginLeadInPick();
    QToolTip::showText(occView->mapToGlobal(QPoint(24, 24)),
                       tr("移动鼠标预览引刀线，左键确认，右键或 Esc 取消"),
                       occView);
}

// =============================================================================
// CmdToolpathPreview
// =============================================================================

CmdToolpathPreview::CmdToolpathPreview(IAppContext* ctx)
    : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/preview.svg"), tr("刀路预览"), this);
    a->setStatusTip(tr("切换刀路显示/隐藏"));
    a->setCheckable(true);
    a->setChecked(true);
    setAction(a);
}

bool CmdToolpathPreview::isEnabled() const
{
    return context()->camModule()->hasToolpath();
}

void CmdToolpathPreview::execute()
{
    CamModule* cam = context()->camModule();
    cam->setToolpathVisible(!cam->isToolpathVisible());
    if (action()->isCheckable())
        action()->setChecked(cam->isToolpathVisible());
}

// =============================================================================
// CmdRecalcToolpath
// =============================================================================

CmdRecalcToolpath::CmdRecalcToolpath(IAppContext* ctx)
    : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/toolpath_5x.svg"), tr("重新计算"), this);
    a->setStatusTip(tr("使用当前参数重新计算刀路引刀线"));
    setAction(a);
}

bool CmdRecalcToolpath::isEnabled() const
{
    return context()->camModule()->hasToolpath();
}

void CmdRecalcToolpath::execute()
{
    context()->camModule()->recalcToolpath();
}

// =============================================================================
// CmdSimulate — continuous machine animation
// =============================================================================

CmdSimulate::CmdSimulate(IAppContext* ctx)
    : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/simulate.svg"), tr("仿真"), this);
    a->setStatusTip(tr("按刀路坐标驱动机床轴进行仿真动画"));
    setAction(a);

    connect(context()->camModule(), &CamModule::simulationTick,
            this, &CmdSimulate::simulationTick);
    connect(context()->camModule(), &CamModule::simulationFinished,
            this, &CmdSimulate::simulationFinished);
}

bool CmdSimulate::isEnabled() const
{
    return context()->camModule()->hasToolpath();
}

void CmdSimulate::execute()
{
    if (isPlaying())
        pause();
    else
        play();
}

void CmdSimulate::play()
{
    context()->camModule()->simulatePlay();
}

void CmdSimulate::pause()
{
    context()->camModule()->simulatePause();
}

void CmdSimulate::stop()
{
    context()->camModule()->simulateStop();
}

void CmdSimulate::setSpeed(double factor)
{
    context()->camModule()->setSimulationSpeed(factor);
}

bool CmdSimulate::isPlaying() const { return context()->camModule()->isSimulating(); }
bool CmdSimulate::isPaused()  const { return context()->camModule()->isSimPaused(); }

void CmdSimulate::onTick()
{
}

