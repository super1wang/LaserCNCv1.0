#include "app/commands_cam.h"
#include "app/i_app_context.h"

#include "base/lcnc_application.h"
#include "base/lcnc_document.h"
#include "base/laser_toolpath.h"
#include "base/face_classifier.h"
#include "base/machine_kinematics.h"
#include "base/xcaf_utils.h"
#include "gui/gui_application.h"
#include "gui/gui_document.h"
#include "graphics/graphics_scene.h"
#include "modules/cam_module.h"

#include <QAction>
#include <QIcon>
#include <QMessageBox>
#include <QTimer>

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
    if (!cam->generateToolpath(cam->smoothAngle(), cam->useFaceClassification())) {
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
    GuiDocument* gd = context()->machineGuiDocument();
    if (!gd) return;

    const Handle(AIS_InteractiveContext)& aisCtx = gd->context();
    if (aisCtx.IsNull()) return;

    LaserToolpath& toolpath = cam->toolpathRef();
    const QList<Handle(AIS_Shape)>& contourAis = cam->contourAis();

    // Activate edge-level selection on contour AIS shapes
    // Mode 2 = TopAbs_EDGE for AIS_Shape
    for (const auto& ais : contourAis) {
        if (!ais.IsNull()) {
            aisCtx->Activate(ais, 2);   // Edge sub-shape selection
        }
    }

    // Show a non-modal message instructing the user to click
    QMessageBox::information(nullptr, tr("选择引刀位置"),
        tr("请在3D视图中点击一条轮廓边缘。\n"
           "点击的位置将作为引刀线的进入点。\n\n"
           "点击确定后进入选择模式，单击视图中的轮廓边缘完成选择。"));

    // Check if user already has something selected (after the message box)
    if (aisCtx->NbSelected() == 0) {
        // The user needs to click — we set up a one-shot connection.
        // For simplicity, we query current selection after the message box.
        // In practice the user will click, and we handle it on next call.
        // Let's check DetectedShape for any recently picked edge.
    }

    // Process the current selection
    aisCtx->InitSelected();
    if (aisCtx->MoreSelected()) {
        Handle(SelectMgr_EntityOwner) owner = aisCtx->SelectedOwner();
        Handle(StdSelect_BRepOwner) brepOwner =
            Handle(StdSelect_BRepOwner)::DownCast(owner);

        if (!brepOwner.IsNull() && brepOwner->HasShape()) {
            TopoDS_Shape selectedShape = brepOwner->Shape();
            if (selectedShape.ShapeType() == TopAbs_EDGE) {
                TopoDS_Edge edge = TopoDS::Edge(selectedShape);

                // Get the clicked 3D point (use parameter midpoint of the edge)
                BRepAdaptor_Curve curve(edge);
                double uMid = (curve.FirstParameter() + curve.LastParameter()) / 2.0;
                gp_Pnt clickPt = curve.Value(uMid);

                // Find which contour contains this edge
                int bestContour = -1;
                double bestDist = 1e30;

                for (int ci = 0; ci < toolpath.contourCount(); ++ci) {
                    const LaserContour& c = toolpath.contour(ci);
                    for (const auto& tp : c.points) {
                        double d = clickPt.Distance(tp.position);
                        if (d < bestDist) {
                            bestDist = d;
                            bestContour = ci;
                        }
                    }
                }

                if (bestContour >= 0) {
                    cam->setLeadInEntry(bestContour, clickPt, uMid);
                }
            }
        }
    }

    // Deactivate edge selection mode, restore shape-level selection
    for (const auto& ais : contourAis) {
        if (!ais.IsNull()) {
            aisCtx->Deactivate(ais, 2);
            aisCtx->Activate(ais, 0);  // Restore shape-level
        }
    }
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

