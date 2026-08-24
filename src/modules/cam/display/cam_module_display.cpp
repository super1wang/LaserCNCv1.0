
#include "modules/cam/cam_module.h"
#include "modules/cam/internal/cam_module_support.h"
#include "view/toolpath_renderer.h"
#include "view/travel_path_renderer.h"
#include "view/contour_order_label_renderer.h"
#include "view/machine_guide_renderer.h"
#include "core/project/cam/cam_data_manager.h"
#include "core/project/cam/layer_container.h"
#include "core/project/cam/layer_manager.h"
#include "modules/cam/machine/machine_axis_detector.h"
#include "modules/cam/display/cam_display_projection_service.h"
#include "modules/cam/toolpath/toolpath_generation_service.h"
#include "modules/cam/machine/machine_io.h"
#include "modules/cam/interaction/reference_pick.h"
#include "modules/cam/integration/cam_service_adapters.h"
#include "modules/cam/collision/collision_geometry_cache.h"
#include "modules/cam/collision/cutter_collision_geometry.h"
#include "modules/cam/toolpath/toolpath_sequence_service.h"
#include "modules/cam/toolpath/toolpath_solve_service.h"
#include "core/machine/machine_workspace.h"
#include "modules/cam/contracts/cam_events.h"
#include "core/kinematics/machine_configuration_service.h"
#include "core/kernel/kernel.h"
#include "core/settings/app_settings.h"
#include "modules/cad/services/shape_service.h"

#include "modules/cam/settings/cam_config.h"
#include "core/algorithms/cam/face_classifier.h"
#include "core/algorithms/cam/travel_path_planner.h"
#include "core/document/lcnc_document.h"
#include "core/algorithms/cam/laser_toolpath.h"
#include "core/algorithms/cam/collision_scan_policy.h"
#include "core/algorithms/cam/collision_safety_domain.h"
#include "core/algorithms/cam/initial_approach_axis_planner.h"
#include "core/algorithms/cam/surface_collision_prefilter.h"
#include "core/algorithms/occt_exact_operation_lock.h"
#include "core/kinematics/machine_kinematics.h"
#include "core/kinematics/machine_pose.h"
#include "core/kernel/i_kernel.h"
#include "core/kernel/service_registry.h"
#include "core/services/selection_service.h"
#include "core/logging/logger.h"
#include "core/project/lcnc_project_manager.h"
#include "core/task/task_manager.h"
#include "core/document/xcaf_utils.h"
#include "view/gui_application.h"
#include "view/gui_document.h"
#include "view/widget_occ_view.h"
#include "view/graphics_scene.h"

#include <QElapsedTimer>
#include <QCoreApplication>
#include <QDateTime>
#include <QFile>
#include <QFileInfo>
#include <QMutex>
#include <QMutexLocker>
#include <QPoint>
#include <QPointer>
#include <QScopeGuard>
#include <QSignalBlocker>
#include <QByteArray>
#include <QThread>
#include <QTimer>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <utility>
#include <vector>

#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRep_Builder.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepTools.hxx>
#include <STEPControl_Reader.hxx>
#include <StlAPI_Reader.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <TopLoc_Location.hxx>
#include <gp_Trsf.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <AIS_DisplayMode.hxx>
#include <Aspect_PolygonOffsetMode.hxx>
#include <Aspect_TypeOfLine.hxx>
#include <Graphic3d_ZLayerId.hxx>
#include <Prs3d_Drawer.hxx>
#include <Prs3d_LineAspect.hxx>
#include <TDF_LabelSequence.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <Quantity_Color.hxx>
#include <Quantity_NameOfColor.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <Bnd_OBB.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <Precision.hxx>
#include <SelectMgr_EntityOwner.hxx>
#include <SelectMgr_SelectableObject.hxx>
#include <StdSelect_BRepOwner.hxx>
#include <gp_Vec.hxx>
#include <gp_Ax2.hxx>
#include <gp_Ax3.hxx>
#include <OSD_Parallel.hxx>
#include <OSD_ThreadPool.hxx>
#include <TopTools_MapOfShape.hxx>


namespace {
using lcnc::cam::detail::entityEntries;
} // namespace

int CamModule::machiningFaceCount() const
{
    return static_cast<int>(machiningFaces().size());
}

bool CamModule::hasManualMachiningFaces() const
{
    for (const auto& entry : machiningFaces())
        if (entry.manual)
            return true;
    return false;
}

QList<CamModule::MachiningFaceInfo> CamModule::machiningFacesForTree() const
{
    QList<MachiningFaceInfo> result;
    result.reserve(static_cast<int>(machiningFaces().size()));
    int autoIdx = 0, manualIdx = 0;
    for (const auto& entry : machiningFaces()) {
        // Cross sections are algorithmic reference faces.  They intentionally
        // stay out of the operator-facing machining-face tree.
        if (entry.role == lcnc::cam::MachiningFaceRole::CrossSection)
            continue;
        MachiningFaceInfo info;
        info.faceId = entry.faceId;
        info.workpieceEntry = entry.workpieceEntry;
        info.manual = entry.manual;
        info.role = entry.role;
        const QString roleName = [this, &entry]() {
            switch (entry.role) {
            // 中文翻译：加工面
            case lcnc::cam::MachiningFaceRole::MachiningSurface: return tr("Processing surface");
            // 中文翻译：横截面
            case lcnc::cam::MachiningFaceRole::CrossSection: return tr("cross section");
            }
            // 中文翻译：加工面
            return tr("Processing surface");
        }();
        info.displayName = entry.manual
            // 中文翻译：手动%1 %2
            ? tr("Manual %1 %2").arg(roleName).arg(++manualIdx)
            : tr("%1 %2").arg(roleName).arg(++autoIdx);
        result.append(info);
    }
    return result;
}

std::vector<TopoDS_Face> CamModule::manualMachiningFaces() const
{
    std::vector<TopoDS_Face> faces;
    for (const auto& entry : machiningFaces())
        if (entry.manual && !entry.face.IsNull())
            faces.push_back(entry.face);
    return faces;
}

void CamModule::addMachiningFace(const TopoDS_Face& face)
{
    // 中文翻译：编辑加工面
    if (rejectConflictingPipelineOperation(tr("Edit machining surface")))
        return;
    if (face.IsNull() || !m_machiningFacePipeline)
        return;
    QString workpieceEntry;
    for (const WorkpieceShapeSource& source : collectWorkpieceShapes()) {
        bool found = false;
        for (TopExp_Explorer exp(source.shape, TopAbs_FACE); exp.More(); exp.Next()) {
            if (TopoDS::Face(exp.Current()).IsSame(face)) {
                found = true;
                break;
            }
        }
        if (found) {
            workpieceEntry = source.workpieceEntry;
            break;
        }
    }
    if (!m_machiningFacePipeline->addManualFace(face, workpieceEntry))
        return;
    // An explicit pick must always be visible so the operator can confirm the
    // growing face set, even when the machining-face tree was hidden earlier.
    m_machiningFacesVisible = true;
    refreshMachiningFaceDisplay();
    pushMachiningFaceRecordsToCamData();
    if (m_camData) {
        m_camData->failPipelineStage(lcnc::cam::CamPipelineStage::FaceSeparation,
                                     // 中文翻译：加工面编辑尚未应用
                                     tr("Machining surface editing has not been applied yet"));
        m_camData->setGenerationParamsDirty(true);
    }
    emit machiningFacesChanged();
}

bool CamModule::removeMachiningFace(std::uint64_t faceId)
{
    // 中文翻译：编辑加工面
    if (rejectConflictingPipelineOperation(tr("Edit machining surface")))
        return false;
    if (!m_machiningFacePipeline || !m_machiningFacePipeline->removeFace(faceId))
        return false;
    pushMachiningFaceRecordsToCamData();
    if (m_camData) {
        m_camData->failPipelineStage(lcnc::cam::CamPipelineStage::FaceSeparation,
                                     // 中文翻译：加工面编辑尚未应用
                                     tr("Machining surface editing has not been applied yet"));
        m_camData->setGenerationParamsDirty(true);
    }
    refreshMachiningFaceDisplay();
    emit machiningFacesChanged();
    return true;
}

bool CamModule::setMachiningFaceRole(
    std::uint64_t faceId, lcnc::cam::MachiningFaceRole role)
{
    // 中文翻译：编辑加工面
    if (rejectConflictingPipelineOperation(tr("Edit machining surface")))
        return false;
    if (!m_machiningFacePipeline)
        return false;
    const auto result = m_machiningFacePipeline->setFaceRole(faceId, role);
    if (result == lcnc::cam::MachiningFacePipelineService::RoleChangeResult::NotFoundOrUnchanged)
        return false;
    if (result == lcnc::cam::MachiningFacePipelineService::RoleChangeResult::InvalidCrossSection) {
            // 中文翻译：设置横截面
            emit operationFailed(tr("Set cross section"),
                                 // 中文翻译：横截面必须与同一工件的加工面相交或共享边。
                                 tr("The cross section must intersect or share an edge with a machined surface of the same workpiece."));
            return false;
    }
    pushMachiningFaceRecordsToCamData();
    if (m_camData) {
        m_camData->failPipelineStage(lcnc::cam::CamPipelineStage::FaceSeparation,
                                     // 中文翻译：加工面角色编辑尚未应用
                                     tr("Machining surface role editing has not been applied yet"));
        m_camData->setGenerationParamsDirty(true);
    }
    refreshMachiningFaceDisplay();
    emit machiningFacesChanged();
    return true;
}

void CamModule::clearMachiningFaces()
{
    // 中文翻译：编辑加工面
    if (rejectConflictingPipelineOperation(tr("Edit machining surface")))
        return;
    if (machiningFaces().empty())
        return;
    m_machiningFacePipeline->clearEntries();
    pushMachiningFaceRecordsToCamData();
    if (m_camData) {
        m_camData->failPipelineStage(lcnc::cam::CamPipelineStage::FaceSeparation,
                                     // 中文翻译：加工面编辑尚未应用
                                     tr("Machining surface editing has not been applied yet"));
        m_camData->setGenerationParamsDirty(true);
    }
    refreshMachiningFaceDisplay();
    emit machiningFacesChanged();
}

void CamModule::refreshMachiningFaceDisplay()
{
    if (!m_displayProjectionService)
        return;
    std::vector<lcnc::cam::MachiningFaceDisplaySnapshot> snapshot;
    snapshot.reserve(machiningFaces().size());
    for (const auto& entry : machiningFaces()) {
        snapshot.push_back({entry.faceId, entry.face, entry.workpieceEntry,
                            entry.manual, entry.role});
    }
    m_displayProjectionService->refreshMachiningFaces(
        activeGuiDocument(), snapshot, m_machiningFacesVisible, kinematics());
}

void CamModule::setMachiningFacesVisible(bool visible)
{
    if (m_machiningFacesVisible == visible)
        return;
    m_machiningFacesVisible = visible;
    refreshMachiningFaceDisplay();
}

bool CamModule::machiningFacesVisible() const
{
    return m_machiningFacesVisible;
}

void CamModule::pushMachiningFaceRecordsToCamData()
{
    if (!m_camData || !m_machiningFacePipeline)
        return;
    m_camData->setMachiningFaceRecords(m_machiningFacePipeline->persistenceRecords());
    m_camData->markDirty(true);
}

void CamModule::rebindMachiningFacesFromRecords()
{
    if (!m_camData)
        return;
    const auto& records = m_camData->machiningFaceRecords();
    if (records.empty())
        return;

    std::vector<lcnc::cam::MachiningFacePipelineService::RebindSource> sources;
    for (const WorkpieceShapeSource& source : collectWorkpieceShapes()) {
        if (!source.shape.IsNull())
            sources.push_back({source.workpieceEntry, source.shape});
    }
    if (sources.empty()) {
        m_camData->failPipelineStage(lcnc::cam::CamPipelineStage::FaceSeparation,
                                     // 中文翻译：加工面无法在当前工件中重绑
                                     tr("The machining surface cannot be re-bound in the current workpiece"));
        m_camData->setGenerationParamsDirty(true);
        return;
    }

    const auto result = m_machiningFacePipeline->rebindFromRecords(records, sources);
    for (const auto& missing : result.missingFaces) {
            LCNC_WARN(lcnc::LogCode::Generic,
                      "cam.machiningFace: signature {:016x} not found in workpiece; "
                      "manual face {} dropped",
                      missing.signature, missing.faceId);
    }
    if (!result.missingFaces.empty()) {
        m_camData->failPipelineStage(lcnc::cam::CamPipelineStage::FaceSeparation,
                                     // 中文翻译：部分加工面或横截面无法在当前工件中重绑
                                     tr("Some machined surfaces or cross-sections cannot be re-bound in the current workpiece"));
        m_camData->setGenerationParamsDirty(true);
    }
    refreshMachiningFaceDisplay();
    emit machiningFacesChanged();

    LCNC_INFO(lcnc::LogCode::Generic,
              "cam.machiningFace: rebound {} faces from persisted signatures",
              machiningFaces().size());
}

bool CamModule::pickMachiningFace(WidgetOccView* view, const QPoint& pos, QString* error)
{
    if (!view || view->view().IsNull() || view->context().IsNull()) {
        // 中文翻译：没有可用的视图用于拾取加工面。
        if (error) *error = tr("There are no views available for picking work surfaces.");
        return false;
    }
    const Handle(AIS_InteractiveContext)& context = view->context();
    context->MoveTo(pos.x(), pos.y(), view->view(), Standard_False);
    const Handle(SelectMgr_EntityOwner) owner = context->DetectedOwner();
    const Handle(StdSelect_BRepOwner) brepOwner =
        Handle(StdSelect_BRepOwner)::DownCast(owner);
    if (brepOwner.IsNull() || !brepOwner->HasShape()) {
        // 中文翻译：未检测到面，请将光标放在工件表面上重试。
        if (error) *error = tr("No face detected, please place the cursor on the workpiece surface and try again.");
        return false;
    }
    const TopoDS_Shape picked = brepOwner->Shape();
    if (picked.IsNull() || picked.ShapeType() != TopAbs_FACE) {
        // 中文翻译：拾取到的不是面，请选择工件上的加工面。
        if (error) *error = tr("The picked up surface is not a surface, please select the processing surface on the workpiece.");
        return false;
    }
    bool belongsToWorkpiece = false;
    for (const WorkpieceShapeSource& source : collectWorkpieceShapes()) {
        for (TopExp_Explorer exp(source.shape, TopAbs_FACE); exp.More(); exp.Next()) {
            if (TopoDS::Face(exp.Current()).IsSame(picked)) {
                belongsToWorkpiece = true;
                break;
            }
        }
        if (belongsToWorkpiece)
            break;
    }
    if (!belongsToWorkpiece) {
        // 中文翻译：只能选择工件模型上的面，机台、刀路和辅助显示不可作为加工面。
        if (error) *error = tr("Only the surfaces on the workpiece model can be selected, and the machine table, tool path and auxiliary display cannot be used as processing surfaces.");
        return false;
    }
    addMachiningFace(TopoDS::Face(picked));
    return true;
}

void CamModule::displayAxisGuides()
{
    if (!m_guideRenderer)
        return;
    // 刀头外观（颜色/透明度/缩放）从 AppSettings 注入渲染器，保证启动与项目切换
    // 时刀头锥使用已保存的外观。RenderingManager 的 Colors 路径触及不到刀头锥
    // （它不在 GuiDocument 域形状 map 内），故由这里显式下发。
    if (auto* settings = lcnc::Kernel::current().appSettings()) {
        m_guideRenderer->setCutterHeadAppearance(
            lcnc::view::CutterHeadAppearance{
                settings->colors.cutterHeadColor,
                settings->colors.cutterHeadTransparency,
                settings->colors.cutterHeadScale});
    }
    if (m_cutterDisplayProxyShape.IsNull()) {
        QString proxyError;
        m_cutterDisplayProxyShape = lcnc::cam::buildCutterDisplayProxy(
            m_config, &proxyError);
        if (m_cutterDisplayProxyShape.IsNull() && !proxyError.isEmpty()) {
            LCNC_WARN(lcnc::LogCode::Generic,
                      "cam.cutterDisplayProxy: visual proxy unavailable: {}",
                      proxyError.toStdString());
        }
    }
    m_guideRenderer->setCutterDisplayProxy(m_cutterDisplayProxyShape);
    m_guideRenderer->refresh(activeGuiDocument(), kinematics(), cutterHeadWorldPosition());
}

void CamModule::refreshCutterHeadAppearance()
{
    displayAxisGuides();
}

void CamModule::updateAxisGuideTransforms()
{
    m_guideRenderer->updateTransforms(activeGuiDocument(), kinematics(), cutterHeadWorldPosition());
}

void CamModule::refreshToolpathDisplay()
{
    lcnc::view::ToolpathRenderer::LeadInPreview preview{
        m_previewLeadInContour, m_previewLeadInPointIndex, m_previewLeadInPoint, m_previewLeadInParam, m_previewLeadInValid
    };
    m_toolpathRenderer->refresh(activeGuiDocument(), toolpathRef(), kinematics(), preview);
}

void CamModule::eraseToolpathDisplay()
{
    m_toolpathRenderer->erase(activeGuiDocument());
}

void CamModule::resetProjectViewState()
{
    const auto activeId = lcnc::Kernel::current().projectManager()->activeWorkspaceId();
    m_machineModelVisible = m_machineVisibleWorkspaceIds.contains(activeId);
    m_machineVisibilityInitialized = false;
    m_visibleMachineEntries.clear();
    m_lastCamSelectionContourIds.clear();
    if (m_machineModelVisible) {
        refreshMachineDisplay(false);
    } else {
        if (auto* gd = activeGuiDocument())
            gd->eraseDomain(lcnc::ProjectDomain::Machine);
    }
    emit machineVisibilityChanged();
}

void CamModule::clearToolpathViewState(bool emitSignals)
{
    eraseToolpathDisplay();
    // CAM AIS 按 ContourId 管理，eraseAllContours 删 GuiDocument 注册项。
    if (GuiDocument* gd = activeGuiDocument())
        gd->eraseAllContours();

    // Clearing CAM/toolpath presentation must not discard the independently
    // loaded workpiece or its safety overlay.
    m_previewLeadInContour = -1;
    m_previewLeadInPointIndex = -1;
    m_previewLeadInParam = 0.0;
    m_previewLeadInValid = false;

    if (emitSignals) {
        emit toolpathCleared();
        emit toolpathLayersChanged();
        refreshCuttingOrderOverlays();
    }
}

const QList<Handle(AIS_Shape)>& CamModule::contourAis() const
{
    m_camContourAisCache.clear();
    GuiDocument* gd = activeGuiDocument();
    if (!gd)
        return m_camContourAisCache;

    // Phase C：AIS 直接按 ContourId 查找，不再读 CAM XCAF 标签。
    for (int i = 0; i < toolpathRef().contourCount(); ++i) {
        const LaserContour& contour = toolpathRef().contour(i);
        Handle(AIS_Shape) ais = gd->aisShapeForContour(contour.contourId);
        m_camContourAisCache.append(ais);
    }
    return m_camContourAisCache;
}

void CamModule::setAxisPosition(const QString& axisName, double value, bool refreshNow)
{
    const QString normalizedAxis = axisName.trimmed().toUpper();
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "CamModule::setAxisPosition {}={} refreshNow={}",
               normalizedAxis.toStdString(), value, refreshNow);
    if (!m_pose || !kinematics()) {
        LCNC_DEBUG(lcnc::LogCode::Generic,
                   "CamModule::setAxisPosition: no pose/kinematics, skip");
        return;
    }
    // 由 pose 统一写值；写入会同步回 kinematics 并 emit poseChanged。
    // 通过 m_inPoseSelfUpdate 抑制 coalescer，让本调用同步刷新（保留旧语义）。
    m_inPoseSelfUpdate = true;
    const bool changed = m_pose->setAxisValue(normalizedAxis, value, /*emitChanged*/ false);
    m_inPoseSelfUpdate = false;
    if (!changed)
        return;
    if (refreshNow) {
        refreshMachineTransforms(QStringList{normalizedAxis});
    } else {
        m_pendingDirtyAxes.insert(normalizedAxis);
        if (m_refreshCoalescer && !m_refreshCoalescer->isActive())
            m_refreshCoalescer->start();
    }
}

void CamModule::setEntityVisible(const QString& entry, bool visible)
{
    if (entry.isEmpty())
        return;

    if (visible)
        m_visibleMachineEntries.insert(entry);
    else
        m_visibleMachineEntries.remove(entry);
    m_machineVisibilityInitialized = true;

    if (auto* gd = activeGuiDocument()) {
        Handle(AIS_Shape) ais = gd->aisShape(machineDocumentId(), entry);
        if (ais.IsNull()) {
            if (visible && m_machineModelVisible)
                refreshMachineDisplay(false);
            return;
        }

        if (visible && m_machineModelVisible)
            gd->scene()->displayObject(ais);
        else
            gd->scene()->eraseObject(ais);

        if (gd->hasView())
            gd->view()->Redraw();
    }
}

QStringList CamModule::visibleMachineEntries() const
{
    return QStringList(m_visibleMachineEntries.cbegin(), m_visibleMachineEntries.cend());
}

bool CamModule::isMachineModelVisible() const
{
    return m_machineModelVisible;
}

void CamModule::setMachineModelVisible(bool visible)
{
    if (m_machineModelVisible == visible)
        return;

    const auto activeId = lcnc::Kernel::current().projectManager()->activeWorkspaceId();
    if (activeId != kInvalidProjectWorkspaceId) {
        if (visible)
            m_machineVisibleWorkspaceIds.insert(activeId);
        else
            m_machineVisibleWorkspaceIds.remove(activeId);
    }

    m_machineModelVisible = visible;
    if (visible) {
        refreshMachineDisplay(false);
        return;
    }

    if (auto* gd = activeGuiDocument()) {
        gd->eraseDomain(lcnc::ProjectDomain::Machine);
        if (gd->hasView())
            gd->view()->Redraw();
    }
    emit machineVisibilityChanged();
}

void CamModule::setRotaryAxisGuidesVisible(bool visible)
{
    m_guideRenderer->setRotaryAxisVisible(activeGuiDocument(), visible);
    displayAxisGuides();
}

bool CamModule::rotaryAxisGuidesVisible() const
{
    return m_guideRenderer->rotaryAxisVisible();
}

void CamModule::setCutterHeadGuideVisible(bool visible)
{
    m_guideRenderer->setCutterHeadVisible(activeGuiDocument(), visible);
    displayAxisGuides();
}

bool CamModule::cutterHeadGuideVisible() const
{
    return m_guideRenderer->cutterHeadVisible();
}

void CamModule::setSelectedEntries(const QStringList& entries)
{
    GuiDocument* gd = activeGuiDocument();
    if (!gd)
        return;

    const Handle(AIS_InteractiveContext)& ctx = gd->context();
    if (ctx.IsNull())
        return;

    ctx->ClearSelected(false);
    for (const QString& entry : entries) {
        Handle(AIS_Shape) ais = gd->aisShape(machineDocumentId(), entry);
        if (!ais.IsNull())
            ctx->AddOrRemoveSelected(ais, false);
    }

    if (gd->hasView())
        gd->view()->Redraw();

    emit selectionChanged(gd->selectedEntries(machineDocumentId()));
}

QStringList CamModule::selectedEntries() const
{
    if (auto* gd = activeGuiDocument())
        return gd->selectedEntries(machineDocumentId());

    return {};
}

void CamModule::syncSelectionFromView()
{
    GuiDocument* gd = activeGuiDocument();
    QStringList selectedEntries;
    if (gd) {
        selectedEntries = gd->selectedEntries(workpieceDocumentId());
        if (selectedEntries.isEmpty())
            selectedEntries = gd->selectedEntries(machineDocumentId());
    }
    emit selectionChanged(selectedEntries);

    const QList<int> contourIndexes = selectedCamContourIndexes();
    if (!contourIndexes.isEmpty()) {
        emit toolpathContoursSelected(contourIndexes);
        if (contourIndexes.size() == 1)
            emit toolpathContourSelected(contourIndexes.first());
    }

    // ── 推送选择顺序到 SelectionService（差分式）─────────────────────────
    auto selSvc = lcnc::Kernel::current()
                      .services()
                      .getService<lcnc::core::SelectionService>();
    if (selSvc) {
        QSet<std::uint64_t> nowSelected;
        QVector<lcnc::core::SelectionEntry> newlyAdded;
        for (int idx : contourIndexes) {
            if (idx < 0 || idx >= toolpathRef().contourCount()) continue;
            const std::uint64_t cid = toolpathRef().contour(idx).contourId;
            if (cid == 0) continue;
            nowSelected.insert(cid);
            if (!m_lastCamSelectionContourIds.contains(cid)) {
                lcnc::core::SelectionEntry e;
                e.contourId = cid;
                e.source    = lcnc::core::SelectionEntry::OccView;
                newlyAdded.append(e);
            }
        }
        if (!newlyAdded.isEmpty())
            selSvc->recordSelectedBatch(newlyAdded);
        for (auto cid : m_lastCamSelectionContourIds) {
            if (!nowSelected.contains(cid))
                selSvc->removeContour(cid);
        }
        m_lastCamSelectionContourIds = nowSelected;
    }
}

void CamModule::refreshMachineTransforms()
{
    if (auto* gd = activeGuiDocument()) {
        gd->updateMachineWorkspaceTransforms(machineDocument(), workpieceDocument());
        if (m_toolpathRenderer && m_toolpathRenderer->isVisible()) {
            applyCamContourTransforms();
            m_toolpathRenderer->updateTransforms(gd, toolpathRef(), kinematics());
        }
        updateAxisGuideTransforms();
        m_displayProjectionService->updateMachiningFaceTransforms(gd, kinematics());
        if (m_travelPathRenderer && m_travelPathRenderer->isVisible())
            m_travelPathRenderer->updateTransforms(gd, kinematics());
        if (m_contourOrderLabelRenderer && m_contourOrderLabelRenderer->isVisible())
            m_contourOrderLabelRenderer->updateTransforms(gd, kinematics());
        if (gd->hasView())
            gd->view()->Redraw();
    }
}

void CamModule::refreshMachineTransforms(const QStringList& dirtyAxes)
{
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "CamModule::refreshMachineTransforms(dirty) n={}", dirtyAxes.size());
    // 现阶段 GuiDocument::updateAxisTransforms 与 MachineGuideRenderer::updateTransforms
    // 内部已是就地 SetLocalTransformation，dirty 集合主要用于：
    //   1) 跳过 m_pose 与几何已一致的"无变化"刷新（上层早 return）；
    //   2) 为后续按子轴粒度的精细化刷新预留接入点。
    // dirty 为空时退化为全量。
    if (dirtyAxes.isEmpty()) {
        refreshMachineTransforms();
        return;
    }
    if (auto* gd = activeGuiDocument()) {
        gd->updateMachineWorkspaceTransforms(machineDocument(), workpieceDocument());
        if (m_toolpathRenderer && m_toolpathRenderer->isVisible()) {
            applyCamContourTransforms();
            m_toolpathRenderer->updateTransforms(gd, toolpathRef(), kinematics());
        }
        updateAxisGuideTransforms();
        m_displayProjectionService->updateMachiningFaceTransforms(gd, kinematics());
        if (m_travelPathRenderer && m_travelPathRenderer->isVisible())
            m_travelPathRenderer->updateTransforms(gd, kinematics());
        if (m_contourOrderLabelRenderer && m_contourOrderLabelRenderer->isVisible())
            m_contourOrderLabelRenderer->updateTransforms(gd, kinematics());
        if (gd->hasView())
            gd->view()->Redraw();
    }
}

void CamModule::setCamContoursVisible(bool visible, bool updateView)
{
    GuiDocument* gd = activeGuiDocument();
    if (!gd)
        return;

    // Phase C：直接迭代 toolpathRef().contours() 按 ContourId 取 AIS。
    for (int index = 0; index < toolpathRef().contourCount(); ++index) {
        const LaserContour& contour = toolpathRef().contour(index);
        Handle(AIS_Shape) ais = gd->aisShapeForContour(contour.contourId);
        if (ais.IsNull())
            continue;
        const bool showContour = visible && contour.enabled;
        if (showContour)
            gd->scene()->displayObject(ais, false);
        else
            gd->scene()->eraseObject(ais, false);
    }
    gd->restoreCamContourSelectionModes();

    if (!updateView)
        return;

    if (!gd->context().IsNull())
        gd->context()->UpdateCurrentViewer();
    if (gd->hasView())
        gd->view()->Redraw();
}

void CamModule::setCamContourVisible(int contourIndex, bool visible, bool updateView)
{
    GuiDocument* gd = activeGuiDocument();
    if (!gd || contourIndex < 0 || contourIndex >= toolpathRef().contourCount())
        return;

    const std::uint64_t contourId = toolpathRef().contour(contourIndex).contourId;
    Handle(AIS_Shape) ais = gd->aisShapeForContour(contourId);
    if (ais.IsNull())
        return;

    if (visible)
        gd->scene()->displayObject(ais, false);
    else
        gd->scene()->eraseObject(ais, false);
    if (visible)
        gd->restoreCamContourSelectionModes();

    if (!updateView)
        return;

    if (!gd->context().IsNull())
        gd->context()->UpdateCurrentViewer();
    if (gd->hasView())
        gd->view()->Redraw();
}

void CamModule::applyCamContourVisibility()
{
    setCamContoursVisible(m_toolpathRenderer->isVisible());
}

void CamModule::applyCamContourTransforms()
{
    GuiDocument* gd = activeGuiDocument();
    MachineKinematics* kin = kinematics();
    if (!gd || !kin)
        return;

    const Handle(AIS_InteractiveContext)& ctx = gd->context();
    if (ctx.IsNull())
        return;

    for (int index = 0; index < toolpathRef().contourCount(); ++index) {
        const LaserContour& contour = toolpathRef().contour(index);
        Handle(AIS_Shape) ais = gd->aisShapeForContour(contour.contourId);
        if (ais.IsNull())
            continue;

        const gp_Trsf transform = kin->computeWpcTransform(contour.workpieceEntry);

        ais->SetLocalTransformation(transform);
        ctx->RecomputePrsOnly(ais, Standard_False);
    }
}

QList<int> CamModule::selectedCamContourIndexes() const
{
    QList<int> result;
    GuiDocument* gd = activeGuiDocument();
    if (!gd || !m_camData)
        return result;

    // Phase C：AIS 直接以 ContourId 寻址，GuiDocument 反查得到 contourId 列表后
    // 通过 CamDataManager::contourIndexById 映射回 toolpath 中的位置。
    const QVector<std::uint64_t> selectedIds = gd->selectedContourIds();
    for (std::uint64_t cid : selectedIds) {
        const int idx = m_camData->contourIndexById(cid);
        if (idx >= 0)
            result.append(idx);
    }
    return result;
}

QList<lcnc::cam::ContourId> CamModule::selectedContourIds() const
{
    QList<lcnc::cam::ContourId> ids;
    for (int idx : selectedCamContourIndexes()) {
        const auto id = contourIdAt(idx);
        if (id != 0)
            ids.append(id);
    }
    return ids;
}

void CamModule::refreshMachineDisplay(bool notifyDomainChange)
{
    if (!m_machineModelVisible) {
        if (auto* gd = activeGuiDocument()) {
            gd->eraseDomain(lcnc::ProjectDomain::Machine);
            if (gd->hasView())
                gd->view()->Redraw();
        }
        emit machineVisibilityChanged();
        return;
    }

    if (auto* gd = activeGuiDocument()) {
        gd->rebuildDomain(lcnc::ProjectDomain::Machine, machineDocument());
        gd->updateMachineWorkspaceTransforms(machineDocument(), workpieceDocument());
        // Rebuilding even an empty Machine domain may cause OCC to recompute
        // presentations. Reapply the live WPC transform to CAM contours so a
        // view-only machine toggle cannot return them to their CAD home pose.
        // 中文翻译：即使机台域为空，重建显示也可能触发 OCC 表示重算；必须重新施加
        // 当前工件姿态，避免轮廓线回到模型初始位置。
        applyCamContourTransforms();
        if (m_toolpathRenderer && m_toolpathRenderer->isVisible())
            m_toolpathRenderer->updateTransforms(gd, toolpathRef(), kinematics());
        // Machining-face highlights are independent AIS objects, not part of
        // the contour renderer. Restore their live WPC posture as well.
        // 中文翻译：加工面高亮是独立 AIS 对象，不属于轮廓渲染器；机台显示切换后
        // 也必须恢复其当前工件姿态。
        if (m_displayProjectionService)
            m_displayProjectionService->updateMachiningFaceTransforms(gd, kinematics());
        if (m_travelPathRenderer && m_travelPathRenderer->isVisible())
            m_travelPathRenderer->updateTransforms(gd, kinematics());
        if (m_contourOrderLabelRenderer && m_contourOrderLabelRenderer->isVisible())
            m_contourOrderLabelRenderer->updateTransforms(gd, kinematics());
        if (machineDocument()) {
            const TDF_LabelSequence labels = machineDocument()->entityLabels(LcncDocument::EntityKind::Machine);
            if (!m_machineVisibilityInitialized) {
                m_visibleMachineEntries.clear();
                for (int i = 1; i <= labels.Length(); ++i)
                    m_visibleMachineEntries.insert(XcafUtils::entry(labels.Value(i)));
                m_machineVisibilityInitialized = true;
            }
            for (int i = 1; i <= labels.Length(); ++i) {
                const QString entry = XcafUtils::entry(labels.Value(i));
                Handle(AIS_Shape) ais = gd->aisShape(machineDocumentId(), entry);
                if (ais.IsNull())
                    continue;
                if (m_machineModelVisible && m_visibleMachineEntries.contains(entry))
                    gd->scene()->displayObject(ais, false);
                else
                    gd->scene()->eraseObject(ais, false);
            }
        }
        displayAxisGuides();
        if (gd->hasView())
            gd->view()->Redraw();
    }
    if (notifyDomainChange)
        lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Machine);
    emit machineVisibilityChanged();
}

void CamModule::syncCamDocumentContours(bool forceRebuild)
{
    // Phase C：AIS 现在按 ContourId 寻址，不再走 CAM 域的 XCAF 镜像。
    // 函数名沿用旧名以减少调用方扩散修改；内部仅做 contour body 的 Display 同步。
    GuiDocument* gd = activeGuiDocument();
    if (!gd)
        return;

    // 把"当前 toolpath 中存在的 contourId 集"与"GuiDocument 已注册的 contourId 集"对齐。
    // 文档切换/读档恢复必须走差分路径，避免把已有 AIS 全部擦掉后重建造成闪烁。
    QSet<std::uint64_t> alive;
    for (int index = 0; index < toolpathRef().contourCount(); ++index) {
        const LaserContour& contour = toolpathRef().contour(index);
        if (contour.wire.IsNull() || contour.contourId == 0)
            continue;
        alive.insert(contour.contourId);
    }

    if (forceRebuild) {
        gd->eraseAllContours();
    } else {
        const QVector<std::uint64_t> displayedIds = gd->displayedContourIds();
        for (std::uint64_t contourId : displayedIds) {
            if (!alive.contains(contourId))
                gd->eraseContour(contourId);
        }
    }

    for (int index = 0; index < toolpathRef().contourCount(); ++index) {
        const LaserContour& contour = toolpathRef().contour(index);
        if (contour.wire.IsNull() || contour.contourId == 0)
            continue;
        if (!forceRebuild && !gd->aisShapeForContour(contour.contourId).IsNull())
            continue;
        const QString name = contour.name.trimmed().isEmpty()
            // 中文翻译：轮廓 %1
            ? tr("Outline %1").arg(index + 1)
            : contour.name;
        // 刀路生成后可能一次新增数百/数千条轮廓。不能在这里逐条提交
        // UpdateCurrentViewer/Redraw，否则主线程会被每条 AIS 的重绘占满。
        // 选择结构统一由后续的 setCamContoursVisible() 建立一次。
        gd->displayContourBody(contour.contourId,
                               LaserToolpathBuilder::buildOffsetDisplayShape(contour),
                               name,
                               /*updateViewer=*/false,
                               /*configureSelection=*/false);
    }

    applyCamContourTransforms();
    applyToolpathLayerColors(false);
    applyCamContourVisibility(); // 此处统一建立选择结构并只提交一次视图更新。

}

void CamModule::applyToolpathLayerColors(bool updateView)
{
    GuiDocument* gd = activeGuiDocument();
    if (!gd || !gd->scene())
        return;

    for (int index = 0; index < toolpathRef().contourCount(); ++index) {
        const LaserContour& contour = toolpathRef().contour(index);
        const ToolpathLayer* layer = m_camData ? m_camData->toolpathLayer(contour.layerId) : nullptr;
        if (!layer || !layer->color.isValid())
            continue;

        Handle(AIS_Shape) ais = gd->aisShapeForContour(contour.contourId);
        if (ais.IsNull())
            continue;

        const QColor color = layer->color;
        gd->scene()->setShapeColor(
            ais,
            Quantity_Color(color.redF(), color.greenF(), color.blueF(), Quantity_TOC_RGB),
            false);
    }

    if (!updateView)
        return;
    if (!gd->context().IsNull())
        gd->context()->UpdateCurrentViewer();
    if (gd->hasView())
        gd->view()->Redraw();
}

// ============================================================================
// 工程核心 CAM 数据加载后的视图刷新（数据 IO 已下沉到 core/project/cam）
// ============================================================================

void CamModule::writeContourGeometryToDocument()
{
    // 统一工程文档：把每条轮廓的 wire 作为 EntityKind::Cam 实体写入工程 doc，
    // 并把返回的 label entry 记录到 LaserContour.xcafEntry，供持久化 + 读回时重连几何。
    LcncDocument* doc = workpieceDocument();
    if (!doc)
        return;
    doc->clearEntityKind(LcncDocument::EntityKind::Cam);
    for (int i = 0; i < toolpathRef().contourCount(); ++i) {
        LaserContour& c = toolpathRef().contour(i);
        if (c.wire.IsNull() || c.contourId == 0) {
            c.xcafEntry.clear();
            continue;
        }
        // 实体名编码 contourId（"cam:<id>"）——XCAF 名称会随 .xbf 导出/导入保留，
        // 而 label entry 字符串在导出/导入后会变；故 relink 以名称里的 contourId 为准。
        const QString name = QStringLiteral("cam:%1").arg(c.contourId);
        const TDF_Label lbl = doc->addShapeEntity(c.wire, name, LcncDocument::EntityKind::Cam);
        c.xcafEntry = XcafUtils::entry(lbl);
    }
}

void CamModule::relinkContourGeometryFromDocument()
{
    // 读档后：core 已恢复轮廓元数据(含 xcafEntry)+采样点；此处按 xcafEntry 从工程 doc
    // 的 Cam 实体取回 wire 几何（v3 起几何随 XCAF 持久化，无需运行时重算）。
    LcncDocument* doc = workpieceDocument();
    if (!doc)
        return;
    // 以实体名里编码的 contourId 关联（"cam:<id>"）——稳定且跨 .xbf 导出/导入有效。
    QHash<std::uint64_t, TopoDS_Shape> byContourId;
    const TDF_LabelSequence labels = doc->entityLabels(LcncDocument::EntityKind::Cam);
    for (int i = 1; i <= labels.Length(); ++i) {
        const TDF_Label lbl = labels.Value(i);
        const QString name = XcafUtils::name(lbl);
        if (!name.startsWith(QStringLiteral("cam:")))
            continue;
        bool ok = false;
        const std::uint64_t id = name.mid(4).toULongLong(&ok);
        if (ok && id != 0)
            byContourId.insert(id, XcafUtils::shape(lbl));
    }
    for (int i = 0; i < toolpathRef().contourCount(); ++i) {
        LaserContour& c = toolpathRef().contour(i);
        auto it = byContourId.constFind(c.contourId);
        if (it == byContourId.constEnd() || it.value().IsNull())
            continue;
        if (it.value().ShapeType() == TopAbs_WIRE)
            c.wire = TopoDS::Wire(it.value());
    }
}

void CamModule::pushGenerationParamsToCamData()
{
    // 把当前运行时生成参数固化到工程核心数据，供随工程持久化。
    if (!m_camData)
        return;
    auto& gp = m_camData->generationParams();
    gp.leadInLength         = toolpathRef().globalLeadInLength();
    gp.deflection           = m_deflection;
    gp.cuttingOffsetMm      = toolpathRef().globalCuttingOffsetMm();
    gp.rapidOffsetMm        = toolpathRef().globalRapidOffsetMm();
    gp.smoothAngle          = m_smoothAngle;
    gp.useFaceClassification = m_useFaceClassification;
    gp.extractionStrategy = m_extractionStrategy;
    if (m_toolpathRenderer)
        gp.normalSampleStep = m_toolpathRenderer->normalSampleStep();
}

void CamModule::applyGenerationParamsFromCamData()
{
    const auto& gp = m_camData->generationParams();
    toolpathRef().setGlobalLeadInLength(gp.leadInLength);
    toolpathRef().setGlobalCuttingOffsetMm(gp.cuttingOffsetMm);
    toolpathRef().setGlobalRapidOffsetMm(gp.rapidOffsetMm);
    m_deflection            = gp.deflection;
    m_smoothAngle           = gp.smoothAngle;
    m_useFaceClassification = gp.useFaceClassification;
    m_extractionStrategy = gp.extractionStrategy;
    if (m_toolpathRenderer) {
        m_toolpathRenderer->setShowNormals(m_config.showNormals());
        m_toolpathRenderer->setNormalSampleStep(m_config.normalSampleStep());
    }
}

void CamModule::onCamDataLoaded()
{
    // core 已把刀路灌入 CamDataManager；此处恢复程序级全局参数与工程轮廓 wire，
    // 再把数据映射到渲染层。
    applyGenerationParamsFromCamData();
    m_workpieceShape = collectWorkpieceShape();
    invalidateMachineEnvironment();
    scheduleWorkpieceSafetyOverlayPreparation();
    if (!m_camData || !m_camData->hasToolpath()) {
        clearToolpathViewState(/*emitSignals=*/true);
        LCNC_INFO(lcnc::LogCode::Generic,
                  "cam.toolpath: project has no cached CAM data; cleared view");
        return;
    }

    relinkContourGeometryFromDocument();
    // Rebind any persisted manual machining faces after project reload.
    rebindMachiningFacesFromRecords();
    const QList<WorkpieceShapeSource> sources = collectWorkpieceShapes();
    for (LaserContour& contour : toolpathRef().contours()) {
        for (const WorkpieceShapeSource& source : sources) {
            if (source.workpieceEntry == contour.workpieceEntry) {
                contour.sourceShape = source.shape;
                break;
            }
        }
        contour.leadIn.length = contour.appliedParams.leadInLength;
        if (contour.leadIn.valid
            && !contour.points.empty()
            && (!contour.points.front().crossSectionNormalValid
                || !contour.leadInSolution.valid)) {
            contour.needsRecalculation = true;
        }
    }
    syncCamDocumentContours();
    refreshToolpathDisplay();
    if (toolpathRef().contourCount() > 0)
        setActiveContourId(static_cast<lcnc::cam::ContourId>(toolpathRef().contour(0).contourId));
    emit toolpathGenerated();
    emit toolpathLayersChanged();
    refreshCuttingOrderOverlays();

    LCNC_INFO(lcnc::LogCode::Generic,
              "cam.toolpath: refreshed view for {} layers, {} contours",
              toolpathRef().layers().size(),
              toolpathRef().contourCount());
}

// ── Travel path 虚线显示 ─────────────────────────────────────────────────────

void CamModule::setTravelPathVisible(bool on)
{
    if (!m_travelPathRenderer) return;
    const bool changed = m_travelPathRenderer->isVisible() != on;
    if (changed)
        m_travelPathRenderer->setVisible(on);
    if (on) {
        // The AIS may have been erased by a project/toolpath refresh while the
        // logical toggle remained checked.  Always rebuild on an explicit ON
        // request, even when the boolean state did not change.
        refreshTravelPath();
    } else {
        m_travelPathRenderer->erase(activeGuiDocument());
        if (auto* gd = activeGuiDocument()) {
            if (gd->hasView()) gd->view()->Redraw();
        }
    }
}

bool CamModule::isTravelPathVisible() const
{
    return m_travelPathRenderer && m_travelPathRenderer->isVisible();
}

void CamModule::refreshTravelPath()
{
    if (!m_travelPathRenderer || !m_travelPathRenderer->isVisible()) return;
    GuiDocument* gd = activeGuiDocument();
    if (!gd) return;

    // CAM owns both the contour sequence and the rapid-plan cache.
    const auto orderedIds = contourSequenceSnapshot().orderedContourIds;
    if (orderedIds.size() < 2) {
        m_travelPathRenderer->refresh(gd, {});
        if (gd->hasView()) gd->view()->Redraw();
        return;
    }

    // Rendering is a cache-only operation.  It must never call
    // exportToolpathSnapshotForOrder(), because that performs continuous IK
    // and collision planning synchronously on the UI thread.
    // 中文翻译：虚线显示只读取已生成缓存，不能为了显示再次执行 IK 或碰撞规划。
    const auto& plannedPlan = m_travelPlanCache;
    std::uint64_t expectedOrderHash = 0;
    for (const auto contourId : orderedIds)
        expectedOrderHash = (expectedOrderHash * 1099511628211ull) ^ contourId;
    if (plannedPlan.key.orderHash != expectedOrderHash) {
        // An order mutation invalidates the previous rapid geometry.  Never
        // draw that stale path while the new CAM plan is being committed.
        m_travelPathRenderer->refresh(gd, {});
        if (gd->hasView()) gd->view()->Redraw();
        return;
    }
    if (!plannedPlan.transitions.isEmpty()) {
        QHash<std::uint64_t, const LaserContour*> contours;
        for (const LaserContour& contour : toolpathRef().contours())
            contours.insert(contour.contourId, &contour);
        QVector<lcnc::view::TravelPathRenderer::Segment> plannedSegments;
        plannedSegments.reserve(plannedPlan.transitions.size());
        for (const auto& transition : plannedPlan.transitions) {
            const auto source = contours.value(transition.fromContourId, nullptr);
            if (!source || transition.surfacePreviewPoints.size() < 2)
                continue;
            lcnc::view::TravelPathRenderer::Segment segment;
            segment.contourId = transition.toContourId;
            segment.collisionState = plannedPlan.fullEnvironmentVerificationPending
                ? lcnc::cam::CollisionValidationState::Pending
                : plannedPlan.collision.state;
            segment.workpieceEntry = source->workpieceEntry;

            // surfacePreviewPoints now describe the executable, offset path.
            // Never add a display-only tool offset here: it would make the
            // visible route differ from simulation and controller execution.
            // 中文翻译：空程显示直接使用可执行偏置轨迹，不再重复叠加显示偏置。
            const auto& previewPoints = transition.workpieceLocalPreviewPoints.isEmpty()
                ? transition.surfacePreviewPoints
                : transition.workpieceLocalPreviewPoints;
            for (int index = 0; index < previewPoints.size(); ++index) {
                const auto& preview = previewPoints.at(index);
                const auto phase = index > 0 && index - 1 < transition.segments.size()
                    ? transition.segments.at(index - 1).phase
                    : lcnc::cam::RapidSegmentPhase::Traverse;
                auto state = segment.collisionState;
                if (index > 0 && index - 1 < transition.collisionStates.size()) {
                    state = lcnc::cam::collisionValidationStateForCertificate(
                        transition.collisionStates.at(index - 1));
                }
                segment.waypoints.append(
                    {preview.x, preview.y, preview.z, phase, state});
            }
            plannedSegments.append(std::move(segment));
        }
        if (!plannedSegments.isEmpty()) {
            m_travelPathRenderer->refresh(gd, plannedSegments);
            m_travelPathRenderer->updateTransforms(gd, kinematics());
            if (gd->hasView()) gd->view()->Redraw();
            return;
        }
        LCNC_WARN(lcnc::LogCode::Generic,
                  "cam.travel: rapid plan has no drawable transition segments");
    }

    // The old endpoint-to-endpoint dashed line is not a valid representation
    // of a failed or missing surface rapid plan.  Hide it instead of making a
    // straight line look executable.
    if (plannedPlan.mode != lcnc::cam::TravelPlanningMode::None) {
        LCNC_WARN(lcnc::LogCode::Generic,
                  "cam.travel: no display path because rapid plan is invalid: {}",
                  plannedPlan.failureReason.toStdString());
        m_travelPathRenderer->refresh(gd, {});
        if (gd->hasView()) gd->view()->Redraw();
        return;
    }

    QHash<std::uint64_t, const LaserContour*> byId;
    byId.reserve(toolpathRef().contourCount());
    for (const LaserContour& contour : toolpathRef().contours()) {
        if (contour.contourId != 0)
            byId.insert(contour.contourId, &contour);
    }

    QVector<lcnc::view::TravelPathRenderer::Segment> segments;
    segments.reserve(orderedIds.size());
    for (auto id : orderedIds) {
        auto it = byId.find(id);
        if (it == byId.end()) continue;
        const LaserContour* c = it.value();
        if (!c || c->points.empty())
            continue;

        const gp_Pnt cutStartLocal = c->points.front().position;
        const gp_Pnt endLocal = c->points.back().position;
        gp_Pnt startLocal = cutStartLocal;
        if (c->leadInSolution.valid)
            startLocal = c->leadInSolution.point.position;

        lcnc::view::TravelPathRenderer::Segment s;
        s.contourId = id;
        s.workpieceEntry = c->workpieceEntry;
        s.sx = startLocal.X(); s.sy = startLocal.Y(); s.sz = startLocal.Z();
        s.ex = endLocal.X();   s.ey = endLocal.Y();   s.ez = endLocal.Z();
        segments.append(s);
    }
    m_travelPathRenderer->refresh(gd, segments);
    m_travelPathRenderer->updateTransforms(gd, kinematics());
    if (gd->hasView()) gd->view()->Redraw();
}

// ── 切割链表序号标注显示 ─────────────────────────────────────────────────────

void CamModule::setContourOrderLabelVisible(bool on)
{
    if (!m_contourOrderLabelRenderer) return;
    if (m_contourOrderLabelRenderer->isVisible() == on) return;
    m_contourOrderLabelRenderer->setVisible(on);
    if (on) {
        refreshContourOrderLabels();
    } else {
        m_contourOrderLabelRenderer->erase(activeGuiDocument());
        if (auto* gd = activeGuiDocument()) {
            if (gd->hasView()) gd->view()->Redraw();
        }
    }
}

bool CamModule::isContourOrderLabelVisible() const
{
    return m_contourOrderLabelRenderer && m_contourOrderLabelRenderer->isVisible();
}

void CamModule::refreshContourOrderLabels()
{
    if (!m_contourOrderLabelRenderer || !m_contourOrderLabelRenderer->isVisible()) return;
    GuiDocument* gd = activeGuiDocument();
    if (!gd) return;

    // 从 Process 端只读视图取顺序，再到 toolpath 取起点（与 refreshTravelPath 同源）。
    const auto orderedIds = contourSequenceSnapshot().orderedContourIds;
    if (orderedIds.isEmpty()) {
        m_contourOrderLabelRenderer->refresh(gd, nullptr, {});
        if (gd->hasView()) gd->view()->Redraw();
        return;
    }

    QHash<std::uint64_t, const LaserContour*> byId;
    byId.reserve(toolpathRef().contourCount());
    for (const LaserContour& contour : toolpathRef().contours()) {
        if (contour.contourId != 0)
            byId.insert(contour.contourId, &contour);
    }

    QVector<lcnc::view::ContourOrderLabelRenderer::Label> labels;
    labels.reserve(orderedIds.size());
    int order = 0;
    for (auto id : orderedIds) {
        auto it = byId.find(id);
        if (it == byId.end()) continue;
        const LaserContour* c = it.value();
        if (!c || c->points.empty())
            continue;

        ++order; // 1 起的加工序号
        const gp_Pnt cutStartLocal = c->points.front().position;
        gp_Pnt startLocal = cutStartLocal;
        if (c->leadInSolution.valid)
            startLocal = c->leadInSolution.point.position;

        lcnc::view::ContourOrderLabelRenderer::Label lbl;
        lbl.contourId = id;
        lbl.workpieceEntry = c->workpieceEntry;
        lbl.sx = startLocal.X(); lbl.sy = startLocal.Y(); lbl.sz = startLocal.Z();
        lbl.order = order;
        labels.append(lbl);
    }
    m_contourOrderLabelRenderer->refresh(gd, kinematics(), labels);
    if (gd->hasView()) gd->view()->Redraw();
}

void CamModule::refreshCuttingOrderOverlays()
{
    // 空程虚线与序号标注都派生自同一份"按加工顺序排好的轮廓起点"，
    // 几何/顺序变化时一并刷新；各自按自身可见性早退，互不影响。
    refreshTravelPath();
    refreshContourOrderLabels();
}
