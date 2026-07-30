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
    // 中文翻译：全局生成刀路
    auto* a = new QAction(QIcon(":/icons/toolpath.svg"), tr("Generate toolpath globally"), this);
    // 中文翻译：使用全局待应用参数重建全部激光刀路
    a->setStatusTip(tr("Rebuild all laser tool paths using global parameters to be applied"));
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
        // 中文翻译：生成刀路
        QMessageBox::information(nullptr, tr("Generate tool path"),
            // 中文翻译：手动选面模式下请先点击工件表面拾取加工面（可多次拾取），再生成刀路。
            tr("In manual surface selection mode, please click on the workpiece surface first to pick the processing surface (can be picked multiple times), and then generate the tool path."));
        context()->updateCommandStates();
        return;
    }
    if (!cam->runAutoPipeline()) {
        // 中文翻译：生成刀路
        QMessageBox::warning(nullptr, tr("Generate tool path"),
            // 中文翻译：加工流程未完成，请检查当前阶段的错误信息。
            tr("The processing process is not completed, please check the error message at the current stage."));
    }
    context()->updateCommandStates();
}

// =============================================================================
// CmdSetLeadIn
// =============================================================================

CmdSetLeadIn::CmdSetLeadIn(IAppContext* ctx)
    : CommandBase(ctx)
{
    // 中文翻译：选择轮廓起点
    auto* a = new QAction(QIcon(":/icons/toolpath_32.svg"), tr("Select outline start point"), this);
    // 中文翻译：在3D视图中点击轮廓采样点设置真实加工起点
    a->setStatusTip(tr("Click the contour sampling point in the 3D view to set the real processing starting point"));
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
                       // 中文翻译：移动鼠标预览下刀线，左键确认轮廓起点，右键或 Esc 取消
                       tr("Move the mouse to preview the lower cut line, left-click to confirm the starting point of the outline, right-click or Esc to cancel."),
                       occView);
}

// =============================================================================
// CmdToolpathPreview
// =============================================================================

CmdToolpathPreview::CmdToolpathPreview(IAppContext* ctx)
    : CommandBase(ctx)
{
    // 中文翻译：刀路预览
    auto* a = new QAction(QIcon(":/icons/preview.svg"), tr("Tool path preview"), this);
    // 中文翻译：切换刀路显示/隐藏
    a->setStatusTip(tr("Switch tool path display/hide"));
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
    // 中文翻译：重新计算当前轮廓
    auto* a = new QAction(QIcon(":/icons/toolpath_5x.svg"), tr("Recalculate the current contour"), this);
    // 中文翻译：应用当前轮廓的待应用参数并仅重建该轮廓
    a->setStatusTip(tr("Apply the to-be-applied parameters of the current contour and rebuild only that contour"));
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
    // 中文翻译：选择加工面
    auto* a = new QAction(QIcon(":/icons/shape.svg"), tr("Select processing surface"), this);
    // 中文翻译：在3D视图中点击工件表面拾取加工面（手动选面模式）
    a->setStatusTip(tr("Click on the workpiece surface in the 3D view to select the processing surface (manual surface selection mode)"));
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
                       // 中文翻译：点击工件表面拾取加工面（可多次拾取），右键或 Esc 结束
                       tr("Click the workpiece surface to pick the processing surface (can be picked multiple times), right-click or Esc to end"),
                       occView);
}

// =============================================================================
// CmdClearMachiningFaces
// =============================================================================

CmdClearMachiningFaces::CmdClearMachiningFaces(IAppContext* ctx)
    : CommandBase(ctx)
{
    // 中文翻译：清除加工面
    auto* a = new QAction(QIcon(":/icons/shape.svg"), tr("Clear the machined surface"), this);
    // 中文翻译：清除所有手动拾取的加工面
    a->setStatusTip(tr("Clear all manually picked work surfaces"));
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


