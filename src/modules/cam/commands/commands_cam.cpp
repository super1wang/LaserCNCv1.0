#include "modules/cam/commands/commands_cam.h"
#include "app/app_command_context.h"
#include "view/widget_occ_view.h"

#include "core/project/project_types.h"
#include "core/document/lcnc_document.h"
#include "core/algorithms/cam/laser_toolpath.h"
#include "core/algorithms/cam/face_classifier.h"
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
    auto* a = new QAction(QIcon(":/icons/toolpath.svg"), tr("全局生成刀路"), this);
    a->setStatusTip(tr("使用全局待应用参数重建全部激光刀路"));
    setAction(a);
}

bool CmdGenerateToolpath::isEnabled() const
{
    LcncDocument* doc = context()->workpieceDocument();
    return doc && doc->entityLabels(LcncDocument::EntityKind::Workpiece).Length() > 0;
}

void CmdGenerateToolpath::execute()
{
    CamModule* cam = context()->camModule();
    if (!cam->generateToolpath(cam->smoothAngle(),
                               cam->useFaceClassification(),
                               cam->deflection())) {
        QMessageBox::warning(nullptr, tr("生成刀路"),
            tr("项目工作区中未找到工件，或未找到可用的轮廓边缘。"));
    }
    context()->updateCommandStates();
}

// =============================================================================
// CmdSetLeadIn
// =============================================================================

CmdSetLeadIn::CmdSetLeadIn(IAppContext* ctx)
    : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/toolpath_32.svg"), tr("选择轮廓起点"), this);
    a->setStatusTip(tr("在3D视图中点击轮廓采样点设置真实加工起点"));
    setAction(a);
}

bool CmdSetLeadIn::isEnabled() const
{
    return context()->camModule()->hasToolpath();
}

void CmdSetLeadIn::execute()
{
    CamModule* cam = context()->camModule();
    cam->requestMachineView();
    WidgetOccView* occView = context()->occView();
    if (!occView)
        return;

    if (occView->isLeadInPickActive()) {
        occView->endLeadInPick();
        cam->cancelLeadInPreview();
        return;
    }

    occView->beginLeadInPick();
    QToolTip::showText(occView->mapToGlobal(QPoint(24, 24)),
                       tr("移动鼠标预览下刀线，左键确认轮廓起点，右键或 Esc 取消"),
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
    auto* a = new QAction(QIcon(":/icons/toolpath_5x.svg"), tr("重新计算当前轮廓"), this);
    a->setStatusTip(tr("应用当前轮廓的待应用参数并仅重建该轮廓"));
    setAction(a);
}

bool CmdRecalcToolpath::isEnabled() const
{
    return context()->camModule()->hasToolpath()
        && context()->camModule()->activeContourIndex() >= 0;
}

void CmdRecalcToolpath::execute()
{
    context()->camModule()->recalcToolpath();
}


