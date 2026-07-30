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
    if (cam->extractionStrategy() == static_cast<int>(ExtractionStrategy::ManualFaceSelection)
        && cam->machiningFaceCount() == 0) {
        QMessageBox::information(nullptr, tr("生成刀路"),
            tr("手动选面模式下请先点击工件表面拾取加工面（可多次拾取），再生成刀路。"));
        context()->updateCommandStates();
        return;
    }
    if (!cam->runAutoPipeline()) {
        QMessageBox::warning(nullptr, tr("生成刀路"),
            tr("加工流程未完成，请检查当前阶段的错误信息。"));
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
    context()->camModule()->recalcToolpathAsync();
}

// =============================================================================
// CmdSelectMachiningFace
// =============================================================================

CmdSelectMachiningFace::CmdSelectMachiningFace(IAppContext* ctx)
    : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/shape.svg"), tr("选择加工面"), this);
    a->setStatusTip(tr("在3D视图中点击工件表面拾取加工面（手动选面模式）"));
    setAction(a);
}

bool CmdSelectMachiningFace::isEnabled() const
{
    LcncDocument* doc = context()->workpieceDocument();
    return doc && doc->entityLabels(LcncDocument::EntityKind::Workpiece).Length() > 0;
}

void CmdSelectMachiningFace::execute()
{
    CamModule* cam = context()->camModule();
    cam->requestMachineView();
    WidgetOccView* occView = context()->occView();
    if (!occView)
        return;

    if (occView->isFacePickActive()) {
        occView->endFacePick();
        return;
    }

    occView->beginFacePick();
    QToolTip::showText(occView->mapToGlobal(QPoint(24, 24)),
                       tr("点击工件表面拾取加工面（可多次拾取），右键或 Esc 结束"),
                       occView);
}

// =============================================================================
// CmdClearMachiningFaces
// =============================================================================

CmdClearMachiningFaces::CmdClearMachiningFaces(IAppContext* ctx)
    : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/shape.svg"), tr("清除加工面"), this);
    a->setStatusTip(tr("清除所有手动拾取的加工面"));
    setAction(a);
}

bool CmdClearMachiningFaces::isEnabled() const
{
    return context()->camModule()->machiningFaceCount() > 0;
}

void CmdClearMachiningFaces::execute()
{
    context()->camModule()->clearMachiningFaces();
    context()->updateCommandStates();
}


