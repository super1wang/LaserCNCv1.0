
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
#include <BRepExtrema_DistShapeShape.hxx>
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
using lcnc::cam::detail::translatedShapeCopy;
using lcnc::cam::detail::watchTask;
} // namespace

void CamModule::clearToolpath()
{
    clearToolpathViewState(/*emitSignals=*/false);
    m_camData->clearToolpath();
    // 加工面集合随刀路一并清空（避免上一工程/工件的高亮残留）。
    m_machiningFacePipeline->clearEntries();
    m_camData->setMachiningFaceRecords({});
    refreshMachiningFaceDisplay();
    // 统一工程文档：清掉轮廓几何(EntityKind::Cam)实体。
    if (LcncDocument* doc = workpieceDocument())
        doc->clearEntityKind(LcncDocument::EntityKind::Cam);
    m_clearingToolpath = true;
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Cam);
    m_clearingToolpath = false;
    emit toolpathCleared();
    emit toolpathLayersChanged();
    emit machiningFacesChanged();
}
bool CamModule::resolveReferencePlaneCenter(WidgetOccView* occView,
                                            const QPoint& screenPos,
                                            gp_Pnt& center,
                                            QString* errorMessage) const
{
    return lcnc::cam::reference_pick::resolveReferencePlaneCenter(occView, screenPos, center, errorMessage);
}

bool CamModule::ensureAcCenterCalibrationAvailable(QString* errorMessage) const
{
    const MachineKinematics* kin = kinematics();
    if (!kin) {
        if (errorMessage)
            // 中文翻译：找不到机台轴系配置。请先选择 AC 转台构型。
            *errorMessage = tr("The machine axis system configuration cannot be found. Please select the AC turntable configuration first.");
        return false;
    }

    if (kin->configType() != QStringLiteral("VERTICAL_AC_TABLE")) {
        if (errorMessage)
            // 中文翻译：当前仅 VERTICAL_AC_TABLE 构型支持 A/C 模型对齐。
            *errorMessage = tr("A/C model alignment is currently supported only for the VERTICAL_AC_TABLE configuration.");
        return false;
    }

    if (!kin->findAxis(QStringLiteral("A")) || !kin->findAxis(QStringLiteral("C"))) {
        if (errorMessage)
            // 中文翻译：当前 AC 转台轴定义不完整，缺少 A 轴或 C 轴。\n请先在应用程序选项的机台构型页完成配置。
            *errorMessage = tr("The current AC rotary table axis definition is incomplete, either the A or C axis is missing.\nPlease complete the configuration on the Machine Configuration page of the application options first.");
        return false;
    }

    return true;
}

void CamModule::translateToolpathWorldData(const gp_Vec& translation)
{
    if (translation.SquareMagnitude() < 1e-12)
        return;

    for (LaserContour& contour : toolpathRef().contours()) {
        const TopoDS_Shape movedWire = translatedShapeCopy(contour.wire, translation);
        if (!movedWire.IsNull() && movedWire.ShapeType() == TopAbs_WIRE)
            contour.wire = TopoDS::Wire(movedWire);
        contour.sourceShape = translatedShapeCopy(contour.sourceShape, translation);

        for (ToolpathPoint& point : contour.points)
            point.position.Translate(translation);

        if (contour.leadIn.valid)
            contour.leadIn.entryPoint.Translate(translation);
        if (contour.leadInSolution.valid)
            contour.leadInSolution.point.position.Translate(translation);
    }

    m_workpieceShape = translatedShapeCopy(m_workpieceShape, translation);
    m_travelCollisionGeometryCache.reset();

    if (m_previewLeadInValid)
        m_previewLeadInPoint.Translate(translation);
}

void CamModule::updateToolpathMachineCoordinates()
{
    const auto sequence = contourSequenceSnapshot();
    solveToolpathForOrder(QVector<std::uint64_t>(sequence.orderedContourIds.cbegin(),
                                                 sequence.orderedContourIds.cend()));
}

const LaserToolpath& CamModule::toolpath() const
{
    return toolpathRef();
}

lcnc::cam::CamDataManager* CamModule::camData()
{
    m_camData = lcnc::Kernel::current().projectManager()->camData();
    return m_camData;
}

const lcnc::cam::CamDataManager* CamModule::camData() const
{
    return lcnc::Kernel::current().projectManager()->camData();
}

LaserToolpath& CamModule::toolpathRef()
{
    static LaserToolpath emptyToolpath;
    if (auto* data = camData())
        return data->toolpath();
    emptyToolpath.clear();
    return emptyToolpath;
}

const LaserToolpath& CamModule::toolpathRef() const
{
    static const LaserToolpath emptyToolpath;
    if (auto* data = camData())
        return data->toolpath();
    return emptyToolpath;
}

bool CamModule::hasToolpath() const
{
    return toolpathRef().contourCount() > 0;
}

int CamModule::toolpathContourCount() const
{
    return toolpathRef().contourCount();
}

int CamModule::toolpathContourPointCount(int contourIndex) const
{
    if (contourIndex < 0 || contourIndex >= toolpathRef().contourCount())
        return 0;
    return static_cast<int>(toolpathRef().contour(contourIndex).points.size());
}

std::uint64_t CamModule::toolpathRevision() const
{
    return lcnc::cam::ToolpathSequenceService::computeToolpathRevision(
        toolpathRef(),
        m_camData && m_camData->generationParamsDirty(),
        m_camData && m_camData->hasCompletePipelineChain());
}

lcnc::cam::ContourSequenceSnapshot CamModule::contourSequenceSnapshot() const
{
    return lcnc::cam::ToolpathSequenceService::buildSequenceSnapshot(
        exportToolpathBaseSnapshot(),
        m_camData ? &m_camData->layerContainer() : nullptr);
}

bool CamModule::applyAutoContourSort(lcnc::cam::AutoSortAxis axis, QString* errorMessage)
{
    const auto ordered = planAutoContourOrder(axis, errorMessage);
    if (ordered.isEmpty())
        return false;
    const bool solved = solveToolpathForOrder(QVector<std::uint64_t>(ordered.cbegin(), ordered.cend()));
    if (!solved) {
        if (errorMessage) *errorMessage = tr("CAM cannot solve the automatic contour order");
        return false;
    }
    if (auto* manager = m_camData->layerManager()) {
        manager->setManualContourOrder(ordered);
        manager->setSortStrategy(lcnc::cam::CuttingPlanSortStrategy::Manual);
        manager->setLastAutoSortAxis(axis);
    }
    return rebuildTravelPlanForCurrentOrder(errorMessage);
}

QVector<lcnc::cam::ContourId> CamModule::planAutoContourOrder(
    lcnc::cam::AutoSortAxis axis, QString* errorMessage) const
{
    if (!m_camData) {
        if (errorMessage)
            *errorMessage = tr("CAM data is not available");
        return {};
    }
    return lcnc::cam::ToolpathSequenceService::planAutomaticOrder(
        toolpathRef(), m_camData->layerContainer(), kinematics(), axis, errorMessage);
}

bool CamModule::preparePersistedAutoSort(QString* errorMessage)
{
    if (!m_camData || !m_camData->layerManager()) {
        if (errorMessage) *errorMessage = tr("CAM contour sequence service is unavailable");
        return false;
    }
    const auto axis = m_camData->layerContainer().lastAutoSortAxis();
    const auto order = planAutoContourOrder(axis, errorMessage);
    if (order.isEmpty())
        return false;
    m_camData->layerManager()->setManualContourOrder(order);
    m_camData->layerManager()->setSortStrategy(lcnc::cam::CuttingPlanSortStrategy::Manual);
    m_camData->layerManager()->setLastAutoSortAxis(axis);
    return true;
}

QVector<lcnc::cam::ContourId> CamModule::manualContourOrder() const
{
    return m_camData ? m_camData->layerContainer().manualContourOrder()
                     : QVector<lcnc::cam::ContourId>{};
}

lcnc::cam::AutoSortAxis CamModule::lastAutoContourSortAxis() const
{
    return m_camData ? m_camData->layerContainer().lastAutoSortAxis()
                     : lcnc::cam::AutoSortAxis::XPos;
}

void CamModule::setLastAutoContourSortAxis(lcnc::cam::AutoSortAxis axis)
{
    if (!m_camData || !m_camData->layerManager()
        || m_camData->layerContainer().lastAutoSortAxis() == axis) {
        return;
    }

    m_camData->layerManager()->setLastAutoSortAxis(axis);
    m_camData->markDirty(true);
    if (auto* projectManager = lcnc::Kernel::current().projectManager())
        projectManager->notifyDomainChanged(lcnc::ProjectDomain::Cam);
}

bool CamModule::setManualContourOrder(
    const QVector<lcnc::cam::ContourId>& orderedContourIds,
    QString* errorMessage)
{
    if (!m_camData || !m_camData->layerManager()) {
        if (errorMessage) *errorMessage = tr("CAM contour sequence service is unavailable");
        return false;
    }
    if (!solveToolpathForOrder(
            QVector<std::uint64_t>(orderedContourIds.cbegin(), orderedContourIds.cend()))) {
        if (errorMessage) *errorMessage = tr("CAM cannot solve the manual contour order");
        return false;
    }
    m_camData->layerManager()->setManualContourOrder(orderedContourIds);
    m_camData->layerManager()->setSortStrategy(lcnc::cam::CuttingPlanSortStrategy::Manual);
    return rebuildTravelPlanForCurrentOrder(errorMessage);
}

bool CamModule::rebuildTravelPlanForCurrentOrder(QString* errorMessage)
{
    const auto order = contourSequenceSnapshot().orderedContourIds;
    const auto snapshot = exportToolpathSnapshotForOrder(
        QVector<std::uint64_t>(order.cbegin(), order.cend()));
    // A failed rapid plan is still a valid and committed CAM result: it is
    // rendered as unavailable and blocks Process preflight.  Do not roll back
    // the selected order merely because a safety check rejected a transition.
    // Pending CAM validation is an expected final generation phase, not a
    // rapid-planning failure.  Report only a completed/rejected plan here.
    // 中文翻译：等待 CAM 校验是生成刀路的正常收尾阶段，不应被当作空程规划失败提示；这里只报告已完成的拒绝结果。
    if (order.size() > 1 && !snapshot.travelPlan.isExecutable()
        && !snapshot.travelPlan.fullEnvironmentVerificationPending) {
        LCNC_WARN(lcnc::LogCode::Generic,
                  "cam.travel: committed non-executable rapid plan for contour order: {}",
                  snapshot.travelPlan.failureReason.toStdString());
        if (errorMessage && errorMessage->isEmpty())
            *errorMessage = snapshot.travelPlan.failureReason;
    }
    if (snapshot.travelPlan.fullEnvironmentVerificationPending)
        scheduleFullEnvironmentVerification(snapshot);
    else
        scheduleCollisionSafetyDomainPreparation(snapshot);
    emit contourOrderTravelPlanRebuilt(QVector<std::uint64_t>(order.cbegin(), order.cend()));
    refreshCuttingOrderOverlays();
    return true;
}

bool CamModule::solveToolpathForOrder(
    const QVector<std::uint64_t>& orderedContourIds)
{
    if (m_machineLoadPending.load()) {
        LCNC_WARN(lcnc::LogCode::Generic,
                  "cam.toolpath: rejected ordered solve while machine model is loading");
        return false;
    }
    MachineKinematics* kin = kinematics();
    if (!kin) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "cam.toolpath: cannot solve five-axis toolpath without machine kinematics");
        return false;
    }

    if (!m_machineConfig || !m_camData)
        return false;

    auto& contours = toolpathRef().contours();
    const lcnc::MachineModeDefinition definition =
        m_machineConfig->modeDefinition(m_camData->machiningMode());
    MachineKinematics planningKinematics;
    planningKinematics.setAxes(
        lcnc::cam_algo::offlinePlanningAxisBaseline(kin->axes(), definition),
        kin->configType());
    QString solveError;
    if (!lcnc::cam::ToolpathSolveService::solveTransactionally(
            &contours, orderedContourIds, &planningKinematics, definition,
            m_machineConfig->workpieceSetupTransform(),
            m_machineConfig->headToolGeometry(), &solveError)) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "cam.toolpath: ordered machine-coordinate solve failed: {}",
                 solveError.toStdString());
        m_camData->failPipelineStage(lcnc::cam::CamPipelineStage::MachineSolve, solveError);
        refreshToolpathDisplay();
        refreshCuttingOrderOverlays();
        return false;
    }
    m_camData->setMachineAxisLayout(definition.interpolatedAxes);
    m_camData->setSolverId(definition.solverId);
    m_camData->setSolverVersion(definition.solverVersion);
    m_camData->setSolvedMachineConfigurationFingerprint(
        m_machineConfig->configurationFingerprint());

    LCNC_INFO(lcnc::LogCode::Generic,
              "cam.toolpath: solved five-axis coordinates from cutting order, contours={}",
              orderedContourIds.size());
    refreshToolpathDisplay();
    refreshCuttingOrderOverlays();
    return true;
}

lcnc::cam::ToolpathExportSnapshot CamModule::exportToolpathBaseSnapshot() const
{
    return buildToolpathExportSnapshot(
        toolpathRef().contours(),
        toolpathRevision(),
        // 中文翻译：CAM 刀路快照已导出；CAM 当前无刀路
        hasToolpath() ? tr("CAM tool path snapshot exported") : tr("CAM currently has no tool path"));
}

void CamModule::applyToolMotionOffsets(lcnc::cam::ToolpathExportSnapshot& snapshot) const
{
    const MachineKinematics* machine = kinematics();
    const lcnc::MachineModeDefinition definition = m_machineConfig
        ? m_machineConfig->modeDefinition(snapshot.machiningMode)
        : lcnc::MachineModeDefinition{};
    const QList<MachineAxisDef> baseline = machine
        ? lcnc::cam_algo::offlinePlanningAxisBaseline(machine->axes(), definition)
        : QList<MachineAxisDef>{};
    MachineKinematics projection;
    if (machine) {
        projection.setAxes(baseline, machine->configType());
        projection.setWorkpieceSetupTransform(machine->workpieceSetupTransform());
        for (auto it = machine->wpcMounts().cbegin(); it != machine->wpcMounts().cend(); ++it)
            projection.mountWorkpiece(it.key(), it.value());
    }
    const auto applyPoint = [](lcnc::cam::ToolpathExportPoint* point, double offset) {
        if (!point || std::abs(offset) <= 1e-12)
            return;
        gp_Vec normal(point->normalX, point->normalY, point->normalZ);
        if (normal.SquareMagnitude() <= Precision::SquareConfusion())
            normal = gp_Vec(0.0, 0.0, 1.0);
        normal.Normalize();
        normal.Multiply(offset);
        point->x += normal.X();
        point->y += normal.Y();
        point->z += normal.Z();
        // A contour offset changes the TCP geometry.  Its former solved axes
        // are invalid until CAM recomputes the continuous motion sequence.
        // 中文翻译：轮廓偏置改变 TCP 几何，必须由 CAM 重新求解连续运动序列，旧轴坐标不再有效。
        point->machineCoordValid = false;
        point->machineFailureReason.clear();
    };
    const auto offsetWorldEndpoint = [machine, &projection](
                                         const QString& workpieceEntry,
                                         const lcnc::cam::ToolpathExportPoint& point,
                                         double offset) {
        gp_Vec normal(point.normalX, point.normalY, point.normalZ);
        if (normal.SquareMagnitude() <= Precision::SquareConfusion())
            normal = gp_Vec(0.0, 0.0, 1.0);
        if (machine)
            normal.Transform(projection.computeWpcTransform(workpieceEntry));
        normal.Normalize();
        normal.Multiply(offset);
        return normal;
    };
    for (auto& contour : snapshot.contours) {
        const double offset = contour.cuttingOffsetMm;
        auto points = snapshot.pointsByContourId.find(contour.contourId);
        const lcnc::cam::ToolpathExportPoint* firstPoint =
            points == snapshot.pointsByContourId.end() || points.value().isEmpty()
                ? nullptr : &points.value().front();
        const lcnc::cam::ToolpathExportPoint* lastPoint =
            points == snapshot.pointsByContourId.end() || points.value().isEmpty()
                ? nullptr : &points.value().back();
        const lcnc::cam::ToolpathExportPoint& startPoint = contour.hasLeadIn || !firstPoint
            ? contour.leadInPoint : *firstPoint;
        const lcnc::cam::ToolpathExportPoint& cuttingStartPoint = firstPoint
            ? *firstPoint : startPoint;
        const lcnc::cam::ToolpathExportPoint& endPoint = lastPoint
            ? *lastPoint : cuttingStartPoint;
        const gp_Vec startOffset = offsetWorldEndpoint(contour.workpieceEntry, startPoint, offset);
        const gp_Vec cuttingStartOffset = offsetWorldEndpoint(
            contour.workpieceEntry, cuttingStartPoint, offset);
        const gp_Vec endOffset = offsetWorldEndpoint(contour.workpieceEntry, endPoint, offset);

        applyPoint(&contour.leadInPoint, offset);
        // Endpoint coordinates are already world-space.  Offset them along
        // the machining normal, never along a fixed controller Z direction.
        // 中文翻译：端点已经是世界坐标，必须沿加工法线偏置，绝不能按固定控制器 Z 方向偏置。
        contour.startX += startOffset.X();
        contour.startY += startOffset.Y();
        contour.startZ += startOffset.Z();
        contour.cutStartX += cuttingStartOffset.X();
        contour.cutStartY += cuttingStartOffset.Y();
        contour.cutStartZ += cuttingStartOffset.Z();
        contour.endX += endOffset.X();
        contour.endY += endOffset.Y();
        contour.endZ += endOffset.Z();
        if (points == snapshot.pointsByContourId.end())
            continue;
        for (auto& point : points.value())
            applyPoint(&point, offset);
    }
}

void CamModule::attachMotionPlan(lcnc::cam::ToolpathExportSnapshot& snapshot) const
{
    auto& plan = snapshot.motionPlan;
    plan = {};
    plan.revision = snapshot.revision;
    plan.collision = snapshot.travelPlan.collision;
    // Keep the policy in the exported immutable snapshot even when a cached
    // validation result predates an application-option change.
    // 中文翻译：即使缓存的校验结果早于应用选项修改，导出的不可变快照仍使用当前警告阻断策略。
    plan.collision.blockWarning = m_config.blockMachiningOnCollisionWarning();
    if (plan.collision.key == 0) {
        const auto collision = collisionConfiguration();
        plan.collision.state = collision.enabled
            ? lcnc::cam::CollisionValidationState::Pending
            : lcnc::cam::CollisionValidationState::Disabled;
        plan.collision.complete = !collision.enabled;
    }

    const auto appendRapid = [&plan](const lcnc::cam::RapidMoveSegment& segment,
                                     std::uint64_t contourId) {
        lcnc::cam::CamMotionNode node;
        node.phase = lcnc::cam::CamMotionPhase::Rapid;
        node.rapidPhase = segment.phase;
        node.contourId = contourId;
        node.axes = segment.target.kinematicAxes;
        node.axisMask = segment.target.kinematicAxisMask;
        node.tcpX = segment.target.tcpX;
        node.tcpY = segment.target.tcpY;
        node.tcpZ = segment.target.tcpZ;
        node.normalX = segment.target.surfaceNormalX;
        node.normalY = segment.target.surfaceNormalY;
        node.normalZ = segment.target.surfaceNormalZ;
        node.estimatedTimeMs = segment.estimatedTimeMs;
        plan.nodes.append(std::move(node));
    };
    const MachineKinematics* machine = kinematics();
    MachineKinematics projectionMachine;
    MachineKinematics* projection = nullptr;
    QList<MachineAxisDef> projectionBaselineAxes;
    if (machine) {
        const lcnc::MachineModeDefinition definition = m_machineConfig
            ? m_machineConfig->modeDefinition(snapshot.machiningMode)
            : lcnc::MachineModeDefinition{};
        projectionBaselineAxes = lcnc::cam_algo::offlinePlanningAxisBaseline(
            machine->axes(), definition);
        projectionMachine.setAxes(projectionBaselineAxes, machine->configType());
        projectionMachine.setWorkpieceSetupTransform(machine->workpieceSetupTransform());
        for (auto it = machine->wpcMounts().cbegin(); it != machine->wpcMounts().cend(); ++it)
            projectionMachine.mountWorkpiece(it.key(), it.value());
        projection = &projectionMachine;
    }
    const lcnc::MachineAxisLayout projectionLayout = snapshot.machineAxisLayout;
    const auto appendPoint = [&plan, projection, projectionLayout,
                              projectionBaselineAxes](
                                 const lcnc::cam::ToolpathExportPoint& point,
                                 lcnc::cam::CamMotionPhase phase,
                                 std::uint64_t contourId,
                                 const QString& workpieceEntry) {
        if (!point.machineCoordValid)
            return;
        lcnc::cam::CamMotionNode node;
        node.phase = phase;
        node.contourId = contourId;
        node.axes = point.machineAxes;
        node.axisMask = point.machineAxisMask;
        node.tcpX = point.x;
        node.tcpY = point.y;
        node.tcpZ = point.z;
        node.normalX = point.normalX;
        node.normalY = point.normalY;
        node.normalZ = point.normalZ;
        // ToolpathExportPoint geometry is workpiece-local, while machine
        // bodies and rapid TCPs are already in machine-world coordinates.
        // Apply the complete axis-chain + installation transform exactly once
        // before publishing the collision/simulation motion node.
        // 中文翻译：轮廓点是工件局部坐标；碰撞机台与空程 TCP 是机床世界坐标。
        // 发布运动节点前必须且只能施加工件轴链与安装变换一次。
        if (projection) {
            // Each node is independent. Restore axes omitted from its solved
            // mask before applying the frozen physical layout, matching the
            // collision worker's pose reconstruction.
            // 中文翻译：每个节点独立恢复未参与插补的轴，再按冻结物理布局应用该节点轴值。
            lcnc::cam_algo::applyOfflineMotionPose(
                projection, projectionBaselineAxes, projectionLayout,
                point.machineAxes, point.machineAxisMask);
            lcnc::cam_algo::transformMotionNodeGeometry(
                &node, projection->computeWpcTransform(workpieceEntry));
        }
        node.estimatedTimeMs = phase == lcnc::cam::CamMotionPhase::Cutting ? 20.0 : 1.0;
        plan.nodes.append(std::move(node));
    };

    bool firstEnabledContour = true;
    for (const auto& contour : snapshot.contours) {
        if (!contour.enabled || !contour.layerEnabled)
            continue;
        // A committed CAM/offline plan starts at the first contour's lead-in.
        // Even a stale or malformed travel snapshot must not inject the live
        // machine-to-first-point approach; that belongs exclusively to Process.
        // 中文翻译：CAM/离线快照从首轮廓下刀点开始；即使空程快照异常，
        // 也不得在首轮廓前加入当前机头到首点的进入段，该段只属于 Process。
        if (!firstEnabledContour) {
            if (const auto* transition = snapshot.travelPlan.transitionTo(
                    contour.contourId)) {
                for (const auto& segment : transition->segments)
                    appendRapid(segment, contour.contourId);
            }
        }
        if (contour.hasLeadIn)
            appendPoint(contour.leadInPoint, lcnc::cam::CamMotionPhase::LeadIn,
                        contour.contourId, contour.workpieceEntry);
        for (const auto& point : snapshot.pointsByContourId.value(contour.contourId))
            appendPoint(point, lcnc::cam::CamMotionPhase::Cutting,
                        contour.contourId, contour.workpieceEntry);
        firstEnabledContour = false;
    }
    if (plan.collision.nodeStates.size() != plan.nodes.size())
        plan.collision.nodeStates.fill(plan.collision.state, plan.nodes.size());
}

void CamModule::attachTravelPlan(lcnc::cam::ToolpathExportSnapshot& snapshot) const
{
    if (m_machineLoadPending.load()) {
        lcnc::cam::TravelPlanSnapshot pending;
        pending.mode = lcnc::cam::TravelPlanningMode::FullEnvironment;
        // 中文翻译：机台模型正在加载，不能复用或生成已验证的空程计划。
        pending.failureReason = tr("The machine model is loading; a verified rapid travel plan is not available yet");
        pending.stale = false;
        snapshot.travelPlan = pending;
        return;
    }

    lcnc::cam_algo::TravelPlanningRequest request;
    request.proxySafetyRadiusMm = 0.0;
    request.minimumClearanceMm = m_config.cutterCollisionClearanceMm();
    request.maximumSafetyOffsetMm = m_config.maximumRapidSafetyOffsetMm();
    request.collisionSampleStepMm = 2.0;
    // The tool's rapid offset is resolved through the narrow CAM contract and
    // applied to geometric samples before continuous IK. Process must execute
    // the resulting axes verbatim and must not add another Z envelope.
    // 中文翻译：工具空程偏置在连续 IK 前合入；Process 必须原样执行已求解轴坐标。
    request.surfacePathStepMm = 0.5;
    QHash<std::uint64_t, double> rapidOffsetByContour;
    for (const auto& contour : snapshot.contours) {
        rapidOffsetByContour.insert(contour.contourId, contour.rapidOffsetMm);
    }
    const auto semanticMaskFor = [&snapshot](std::uint8_t layoutMask) {
        std::uint8_t result = 0;
        int rotary = 0;
        for (int index = 0; index < snapshot.machineAxisLayout.count; ++index) {
            if (!(layoutMask & (1u << index)))
                continue;
            switch (snapshot.machineAxisLayout.axes[index].role) {
            case lcnc::MachineAxisRole::LinearX: result |= 0x01u; break;
            case lcnc::MachineAxisRole::LinearY: result |= 0x02u; break;
            case lcnc::MachineAxisRole::LinearZ: result |= 0x04u; break;
            default:
                if (rotary == 0) result |= 0x08u;
                else if (rotary == 1) result |= 0x10u;
                ++rotary;
                break;
            }
        }
        return result;
    };
    const std::uint8_t activeMask = semanticMaskFor(
        snapshot.machineAxisLayout.count >= 8 ? 0xffu
            : static_cast<std::uint8_t>((1u << snapshot.machineAxisLayout.count) - 1u));
    request.motionProfile.supportedCoordinatedMask = activeMask;
    const auto assignSemanticPose = [&semanticMaskFor](lcnc::cam::RapidPose* pose,
                                                        const lcnc::cam::ToolpathExportPoint& point) {
        if (!pose) return;
        // Exported machineAxes use the configurable physical layout.  Process
        // command sinks use the fixed semantic order X/Y/Z/R1/R2 instead.
        pose->axes = {point.machineX, point.machineY, point.machineZ,
                      point.machineR1, point.machineR2};
        pose->activeMask = semanticMaskFor(point.machineAxisMask);
        pose->rotaryAxis1Name = point.rotaryAxis1Name;
        pose->rotaryAxis2Name = point.rotaryAxis2Name;
        pose->kinematicAxes = point.machineAxes;
        pose->kinematicAxisMask = point.machineAxisMask;
    };
    for (int i = 0; i < lcnc::MachineAxisLayout::kMaxAxes; ++i) {
        request.motionProfile.velocity[i] = 100.0;
        request.motionProfile.acceleration[i] = 1000.0;
        request.motionProfile.jerk[i] = 10000.0;
    }

    LcncDocument* machine = machineDocument();
    MachineKinematics* kin = kinematics();
    MachineKinematics offlineKinematics;
    QList<MachineAxisDef> offlineBaselineAxes;
    if (kin && m_machineConfig) {
        const lcnc::MachineModeDefinition offlineDefinition =
            m_machineConfig->modeDefinition(snapshot.machiningMode);
        offlineBaselineAxes = lcnc::cam_algo::offlinePlanningAxisBaseline(
            kin->axes(), offlineDefinition);
        offlineKinematics.setAxes(offlineBaselineAxes, kin->configType());
        offlineKinematics.setWorkpieceSetupTransform(kin->workpieceSetupTransform());
        for (auto it = kin->shapeAssignments().cbegin();
             it != kin->shapeAssignments().cend(); ++it) {
            offlineKinematics.assignShape(it.key(), it.value());
        }
        for (auto it = kin->wpcMounts().cbegin(); it != kin->wpcMounts().cend(); ++it)
            offlineKinematics.mountWorkpiece(it.key(), it.value());
        kin = &offlineKinematics;
    }
    const TDF_LabelSequence machineLabels = machine
        ? machine->entityLabels(LcncDocument::EntityKind::Machine)
        : TDF_LabelSequence{};
    const bool hasLoadedMachineGeometry = machineLabels.Length() > 0
        && !m_loadedMachineModelPath.trimmed().isEmpty();
    request.fullEnvironment = hasLoadedMachineGeometry;
    if (!m_workpieceShape.IsNull()) {
        // Exported contour endpoints are in machine coordinates.  Collision
        // geometry must use the same mounted workpiece transform; otherwise a
        // non-zero installation height makes the planner validate a path
        // against the original CAD position and forces every candidate through
        // the expensive exact-distance loop.
        // 中文翻译：导出的轮廓端点已经是机床坐标，碰撞工件必须应用同一安装变换；否则非零安装高度会在错误位置校验碰撞。
        gp_Trsf workpieceTransform;
        if (kin && !snapshot.contours.isEmpty())
            workpieceTransform = kin->computeWpcTransform(snapshot.contours.front().workpieceEntry);
        BRepBuilderAPI_Transform transformedWorkpiece(m_workpieceShape, workpieceTransform, true);
        request.workpiece = transformedWorkpiece.IsDone()
            ? transformedWorkpiece.Shape() : TopoDS_Shape{};
    }
    lcnc::cam::TravelPlanKey key;
    key.toolpathRevision = snapshot.revision;
    key.environmentRevision = machineSetupRevision()
        ^ static_cast<std::uint64_t>(qHash(activeMachineProfilePath()))
        ^ static_cast<std::uint64_t>(request.fullEnvironment ? 1u : 0u)
        ^ static_cast<std::uint64_t>(qHash(
            m_machineConfig ? m_machineConfig->configurationFingerprint() : QString()));
    const QFileInfo cutterModelInfo(m_config.cutterNozzleModelPath());
    const QString cutterProxyFingerprint = QStringLiteral("%1|%2|%3|%4|%5|%6|%7|%8|%9")
        .arg(static_cast<int>(m_config.cutterCollisionProxyMode()))
        .arg(m_config.cutterNozzleModelPath())
        .arg(m_config.simulatedConeLengthMm(), 0, 'g', 15)
        .arg(m_config.simulatedConeTipRadiusMm(), 0, 'g', 15)
        .arg(m_config.simulatedConeBaseRadiusMm(), 0, 'g', 15)
        .arg(m_config.cutterCollisionClearanceMm(), 0, 'g', 15)
        .arg(m_config.maximumRapidSafetyOffsetMm(), 0, 'g', 15)
        .arg(cutterModelInfo.exists() ? cutterModelInfo.size() : -1)
        .arg(cutterModelInfo.exists()
            ? cutterModelInfo.lastModified().toMSecsSinceEpoch() : -1);
    key.environmentRevision ^= static_cast<std::uint64_t>(qHash(cutterProxyFingerprint));
    key.environmentRevision ^= m_collisionConfigurationRevision;
    key.motionProfileHash = static_cast<std::uint64_t>(request.motionProfile.supportedCoordinatedMask);
    for (const auto& contour : snapshot.contours) {
        key.orderHash = (key.orderHash * 1099511628211ull) ^ contour.contourId;
        key.motionProfileHash = (key.motionProfileHash * 1099511628211ull)
            ^ static_cast<std::uint64_t>(qHash(QString::number(
                rapidOffsetByContour.value(contour.contourId), 'g', 17)));
        key.motionProfileHash = (key.motionProfileHash * 1099511628211ull)
            ^ static_cast<std::uint64_t>(qHash(QString::number(
                contour.cuttingOffsetMm, 'g', 17)));
    }

    // applyToolMotionOffsets() changes the local TCP geometry.  Re-solve only
    // contours whose old coordinates were invalidated, before a cached or new
    // rapid plan consumes their endpoints.  This includes a one-contour job.
    // 中文翻译：轮廓偏置会使旧 TCP 坐标失效；空程计划（包括缓存）读取端点前必须重算，并覆盖单轮廓任务。
    const auto assignExportCoordinate = [](lcnc::cam::ToolpathExportPoint* point,
                                           const MachineCoord& coordinate) {
        if (!point)
            return;
        point->machineX = coordinate.x;
        point->machineY = coordinate.y;
        point->machineZ = coordinate.z;
        point->machineR1 = coordinate.r1;
        point->machineR2 = coordinate.r2;
        point->rotaryAxis1Name = coordinate.r1Name;
        point->rotaryAxis2Name = coordinate.r2Name;
        point->machineCoordValid = coordinate.valid;
        point->machineAxes = coordinate.solvedPose.values;
        point->machineAxisMask = coordinate.solvedPose.activeMask;
        point->machineFailureReason = coordinate.solvedPose.failureReason;
    };
    const auto makeOffsetSolveFailure = [&key, &request, &snapshot, this](const QString& reason) {
        lcnc::cam::TravelPlanSnapshot failed;
        failed.mode = request.fullEnvironment
            ? lcnc::cam::TravelPlanningMode::FullEnvironment
            : lcnc::cam::TravelPlanningMode::WorkpieceProxy;
        failed.failureReason = reason;
        failed.stale = false;
        failed.key = key;
        snapshot.travelPlan = failed;
        m_travelPlanCache = failed;
    };
    if (!snapshot.contours.isEmpty()) {
        if (!kin || !m_machineConfig) {
            makeOffsetSolveFailure(tr("Machine kinematics are unavailable for the contour-offset motion solve"));
            return;
        }
        const lcnc::MachineModeDefinition definition =
            m_machineConfig->modeDefinition(snapshot.machiningMode);
        if (!definition.isValid()) {
            makeOffsetSolveFailure(tr("Machine mode definition is unavailable for the contour-offset motion solve"));
            return;
        }
        // Offset geometry must retain the same cross-contour rotary branch as
        // the committed ordered solve.  Re-solving every contour from an empty
        // pose lets opposite tube faces independently choose A=+90/-90 even
        // though the equivalent C rotation is continuous.  Carry the complete
        // physical-layout pose forward so TableSpin(C) is resolved while the
        // TableTilt(A/B) branch remains stable.
        // 中文翻译：刀具偏置后的重算必须延续整条有序刀路的旋转分支；逐轮廓从空姿态求解会让
        // 管材对侧轮廓各自选择 A=+90/-90。传递上一轮廓末位姿，优先由 C 轴吸收周向变化。
        lcnc::SolvedMachinePose offsetContinuity;
        for (auto& contour : snapshot.contours) {
            auto pointsIt = snapshot.pointsByContourId.find(contour.contourId);
            if (pointsIt == snapshot.pointsByContourId.end())
                continue;
            auto& contourPoints = pointsIt.value();
            const bool needsSolve = (contour.hasLeadIn && !contour.leadInPoint.machineCoordValid)
                || std::any_of(contourPoints.cbegin(), contourPoints.cend(),
                    [](const lcnc::cam::ToolpathExportPoint& point) {
                        return !point.machineCoordValid;
                    });
            if (!needsSolve) {
                if (!contourPoints.isEmpty() && contourPoints.back().machineCoordValid) {
                    offsetContinuity.values = contourPoints.back().machineAxes;
                    offsetContinuity.activeMask = contourPoints.back().machineAxisMask;
                    offsetContinuity.valid = true;
                }
                continue;
            }

            MachineKinematics solveKinematics;
            solveKinematics.setAxes(kin->axes(), kin->configType());
            solveKinematics.setWorkpieceSetupTransform(kin->workpieceSetupTransform());
            for (auto it = kin->shapeAssignments().cbegin(); it != kin->shapeAssignments().cend(); ++it)
                solveKinematics.assignShape(it.key(), it.value());
            for (auto it = kin->wpcMounts().cbegin(); it != kin->wpcMounts().cend(); ++it)
                solveKinematics.mountWorkpiece(it.key(), it.value());

            std::vector<ToolpathPoint> solvePoints;
            solvePoints.reserve(static_cast<std::size_t>(contourPoints.size())
                + (contour.hasLeadIn ? 1u : 0u));
            const auto appendPoint = [&solvePoints](const lcnc::cam::ToolpathExportPoint& point) {
                ToolpathPoint solved;
                solved.position = gp_Pnt(point.x, point.y, point.z);
                solved.normal = gp_Dir(point.normalX, point.normalY, point.normalZ);
                const gp_Vec tangent(point.tangentX, point.tangentY, point.tangentZ);
                if (tangent.SquareMagnitude() > Precision::SquareConfusion())
                    solved.tangent = gp_Dir(tangent);
                solvePoints.push_back(std::move(solved));
            };
            if (contour.hasLeadIn)
                appendPoint(contour.leadInPoint);
            for (const auto& point : std::as_const(contourPoints))
                appendPoint(point);
            if (solvePoints.empty())
                continue;

            QString solveError;
            if (!LaserToolpathBuilder::solveTransientMotionPath(
                    &solvePoints, &solveKinematics, definition,
                    m_machineConfig->workpieceSetupTransform(),
                    m_machineConfig->headToolGeometry(), &solveError,
                    offsetContinuity.valid ? &offsetContinuity : nullptr)) {
                makeOffsetSolveFailure(tr("Contour-offset five-axis solve failed for contour %1: %2")
                    .arg(contour.contourId).arg(solveError));
                return;
            }
            std::size_t solvedIndex = 0;
            if (contour.hasLeadIn)
                assignExportCoordinate(&contour.leadInPoint, solvePoints[solvedIndex++].machineCoord);
            for (auto& point : contourPoints)
                assignExportCoordinate(&point, solvePoints[solvedIndex++].machineCoord);
            offsetContinuity = solvePoints.back().machineCoord.solvedPose;
        }
    }
    if (!m_travelPlanCache.stale && m_travelPlanCache.key == key) {
        snapshot.travelPlan = m_travelPlanCache;
        return;
    }

    if (snapshot.contours.size() <= 1) {
        lcnc::cam::TravelPlanSnapshot emptyPlan;
        emptyPlan.mode = request.fullEnvironment
            ? lcnc::cam::TravelPlanningMode::FullEnvironment
            : lcnc::cam::TravelPlanningMode::WorkpieceProxy;
        emptyPlan.key = key;
        emptyPlan.stale = false;
        // A single contour has no inter-contour rapid, but it still contains
        // lead-in and cutting nodes.  Never bypass CAM collision validation
        // merely because the rapid-transition list is empty.
        // 中文翻译：单轮廓虽然没有轮廓间空程，但仍有引入和切割节点；不能因空程列表为空而跳过 CAM 碰撞校验。
        const auto collisionConfig = collisionConfiguration();
        emptyPlan.collision.key = key.toolpathRevision ^ key.orderHash ^ key.environmentRevision;
        emptyPlan.collision.blockWarning = m_config.blockMachiningOnCollisionWarning();
        if (!collisionConfig.enabled) {
            emptyPlan.collision.state = lcnc::cam::CollisionValidationState::Disabled;
            emptyPlan.collision.complete = true;
        } else if (!collisionConfig.valid) {
            emptyPlan.collision.state = lcnc::cam::CollisionValidationState::Indeterminate;
            emptyPlan.collision.complete = true;
            emptyPlan.collision.failureReason = tr("Collision detection is enabled but the source configuration is incomplete");
            emptyPlan.failureReason = emptyPlan.collision.failureReason;
        } else {
            emptyPlan.collision.state = lcnc::cam::CollisionValidationState::Pending;
            emptyPlan.collision.complete = false;
            emptyPlan.fullEnvironmentVerificationPending = true;
        }
        snapshot.travelPlan = emptyPlan;
        m_travelPlanCache = emptyPlan;
        return;
    }

    QString cutterProxyError;
    request.cutterCollisionProxy = lcnc::cam::buildCutterCollisionProxy(
        m_config, &cutterProxyError);
    if (request.cutterCollisionProxy.IsNull()) {
        lcnc::cam::TravelPlanSnapshot failed;
        failed.mode = request.fullEnvironment
            ? lcnc::cam::TravelPlanningMode::FullEnvironment
            : lcnc::cam::TravelPlanningMode::WorkpieceProxy;
        failed.failureReason = cutterProxyError.isEmpty()
            // 中文翻译：无法创建切割嘴碰撞代理
            ? tr("Unable to create the cutting nozzle collision proxy")
            : cutterProxyError;
        failed.stale = false;
        failed.key = key;
        snapshot.travelPlan = failed;
        m_travelPlanCache = failed;
        return;
    }

    for (int i = 1; i < snapshot.contours.size(); ++i) {
        const auto& previous = snapshot.contours.at(i - 1);
        const auto& next = snapshot.contours.at(i);
        const auto previousPoints = snapshot.pointsByContourId.value(previous.contourId);
        if (previousPoints.isEmpty() || !next.leadInPoint.machineCoordValid) {
            lcnc::cam::TravelPlanSnapshot failed;
            failed.mode = request.fullEnvironment
                ? lcnc::cam::TravelPlanningMode::FullEnvironment
                : lcnc::cam::TravelPlanningMode::WorkpieceProxy;
            // 中文翻译：轮廓空程端点缺少已求解的机床坐标
            failed.failureReason = tr("A contour travel endpoint lacks solved machine coordinates");
            failed.stale = false;
            failed.key = key;
            snapshot.travelPlan = failed;
            m_travelPlanCache = failed;
            return;
        }
        const auto& end = previousPoints.back();
        if (!end.machineCoordValid) {
            lcnc::cam::TravelPlanSnapshot failed;
            failed.mode = request.fullEnvironment
                ? lcnc::cam::TravelPlanningMode::FullEnvironment
                : lcnc::cam::TravelPlanningMode::WorkpieceProxy;
            // 中文翻译：轮廓空程端点缺少已求解的机床坐标
            failed.failureReason = tr("A contour travel endpoint lacks solved machine coordinates");
            failed.stale = false;
            failed.key = key;
            snapshot.travelPlan = failed;
            m_travelPlanCache = failed;
            return;
        }

        lcnc::cam_algo::TravelEndpoint source;
        source.contourId = previous.contourId;
        assignSemanticPose(&source.pose, end);
        source.pose.tcpX = previous.endX;
        source.pose.tcpY = previous.endY;
        source.pose.tcpZ = previous.endZ;
        gp_Dir sourceNormal(end.normalX, end.normalY, end.normalZ);
        sourceNormal.Transform(kin->computeWpcTransform(previous.workpieceEntry));
        source.pose.surfaceNormalX = sourceNormal.X();
        source.pose.surfaceNormalY = sourceNormal.Y();
        source.pose.surfaceNormalZ = sourceNormal.Z();
        lcnc::cam_algo::TravelEndpoint target;
        target.contourId = next.contourId;
        assignSemanticPose(&target.pose, next.leadInPoint);
        target.pose.tcpX = next.startX;
        target.pose.tcpY = next.startY;
        target.pose.tcpZ = next.startZ;
        gp_Dir targetNormal(next.leadInPoint.normalX, next.leadInPoint.normalY,
                            next.leadInPoint.normalZ);
        targetNormal.Transform(kin->computeWpcTransform(next.workpieceEntry));
        target.pose.surfaceNormalX = targetNormal.X();
        target.pose.surfaceNormalY = targetNormal.Y();
        target.pose.surfaceNormalZ = targetNormal.Z();
        request.transitions.append({source, target,
            rapidOffsetByContour.value(next.contourId, next.rapidOffsetMm),
            previous.cuttingOffsetMm,
            next.cuttingOffsetMm});
    }
    lcnc::cam::TravelPlanSnapshot plan = lcnc::cam_algo::TravelPathPlanner::plan(request);
    if (plan.isExecutable() && kin && m_machineConfig) {
        const lcnc::MachineModeDefinition definition =
            m_machineConfig->modeDefinition(snapshot.machiningMode);
        QHash<std::uint64_t, lcnc::cam::ToolpathExportContour*> contourById;
        contourById.reserve(snapshot.contours.size());
        for (auto& contour : snapshot.contours)
            contourById.insert(contour.contourId, &contour);

        // Planning emits surfacePreviewPoints in machine/world coordinates.
        // Freeze a local display copy now, using the posture at which the
        // curve was created.  The renderer later applies the current WPC
        // posture once, so table motion moves the preview with the workpiece
        // instead of preserving the old world-space curve.
        // 中文翻译：规划输出的是机床世界坐标；此处固化工件局部显示曲线，渲染时只施加一次当前工件姿态，使空程随工件旋转移动。
        for (lcnc::cam::RapidTransition& transition : plan.transitions) {
            const auto* sourceContour = contourById.value(transition.fromContourId, nullptr);
            if (!sourceContour || transition.surfacePreviewPoints.isEmpty())
                continue;
            const gp_Trsf inverseWpc =
                kin->computeWpcTransform(sourceContour->workpieceEntry).Inverted();
            transition.workpieceLocalPreviewPoints.reserve(
                transition.surfacePreviewPoints.size());
            for (const auto& point : transition.surfacePreviewPoints) {
                gp_Pnt localPoint(point.x, point.y, point.z);
                localPoint.Transform(inverseWpc);
                gp_Dir localNormal(point.normalX, point.normalY, point.normalZ);
                localNormal.Transform(inverseWpc);
                transition.workpieceLocalPreviewPoints.append({
                    localPoint.X(), localPoint.Y(), localPoint.Z(),
                    localNormal.X(), localNormal.Y(), localNormal.Z()});
            }
        }

        for (lcnc::cam::RapidTransition& transition : plan.transitions) {
            auto* sourceContour = contourById.value(transition.fromContourId, nullptr);
            auto* targetContour = contourById.value(transition.toContourId, nullptr);
            auto sourceIt = snapshot.pointsByContourId.find(transition.fromContourId);
            auto targetIt = snapshot.pointsByContourId.find(transition.toContourId);
            if (!sourceContour || !targetContour
                || sourceIt == snapshot.pointsByContourId.end()
                || targetIt == snapshot.pointsByContourId.end()
                || sourceIt.value().isEmpty() || targetIt.value().isEmpty()
                || sourceContour->workpieceEntry != targetContour->workpieceEntry) {
                // 中文翻译：每个轮廓空程过渡必须属于同一个已求解工件
                plan.failureReason = tr("Surface rapid planning requires a single resolved workpiece for each contour transition");
                plan.transitions.clear();
                break;
            }
            auto& sourcePoints = sourceIt.value();
            auto& targetPoints = targetIt.value();

            // One solver sequence owns the complete transition boundary:
            // previous contour end -> surface reference curve -> next lead-in
            // -> all points of the next contour.  This prevents the rapid and
            // target contour from independently selecting rotary branches.
            // 中文翻译：上一轮廓末点、空程曲线、下一引入点和下一轮廓必须在同一序列连续求解。
            const gp_Trsf inverseWpc = kin->computeWpcTransform(sourceContour->workpieceEntry).Inverted();
            std::vector<ToolpathPoint> continuousPoints;
            continuousPoints.reserve(static_cast<std::size_t>(
                transition.segments.size() + targetPoints.size() + 1));
            const auto appendWorldPoint = [&inverseWpc, &continuousPoints](
                                              double x, double y, double z,
                                              double nx, double ny, double nz) {
                ToolpathPoint point;
                point.position = gp_Pnt(x, y, z).Transformed(inverseWpc);
                gp_Dir normal(nx, ny, nz);
                normal.Transform(inverseWpc);
                point.normal = normal;
                continuousPoints.push_back(std::move(point));
            };
            const auto& source = sourcePoints.back();
            gp_Dir sourceNormalWorld(source.normalX, source.normalY, source.normalZ);
            sourceNormalWorld.Transform(kin->computeWpcTransform(sourceContour->workpieceEntry));
            appendWorldPoint(sourceContour->endX, sourceContour->endY, sourceContour->endZ,
                             sourceNormalWorld.X(), sourceNormalWorld.Y(), sourceNormalWorld.Z());
            for (const lcnc::cam::RapidMoveSegment& segment : transition.segments) {
                appendWorldPoint(segment.target.tcpX, segment.target.tcpY, segment.target.tcpZ,
                                 segment.target.surfaceNormalX, segment.target.surfaceNormalY,
                                 segment.target.surfaceNormalZ);
            }
            const std::size_t rapidEndIndex = continuousPoints.size() - 1u;
            for (const auto& targetPoint : std::as_const(targetPoints)) {
                ToolpathPoint point;
                // targetPoints already include the contour-normal cutting
                // offset; never add a controller-Z height here.
                // 中文翻译：目标轮廓点已经包含沿法线的切割偏置；此处绝不能再叠加控制器 Z 高度。
                point.position = gp_Pnt(targetPoint.x, targetPoint.y, targetPoint.z);
                point.normal = gp_Dir(targetPoint.normalX, targetPoint.normalY,
                                      targetPoint.normalZ);
                const gp_Vec tangent(targetPoint.tangentX, targetPoint.tangentY,
                                     targetPoint.tangentZ);
                if (tangent.SquareMagnitude() > Precision::SquareConfusion())
                    point.tangent = gp_Dir(tangent);
                continuousPoints.push_back(std::move(point));
            }

            lcnc::SolvedMachinePose initialPose;
            initialPose.values = source.machineAxes;
            initialPose.activeMask = source.machineAxisMask;
            initialPose.valid = source.machineCoordValid;
            QString solveError;
            if (!LaserToolpathBuilder::solveTransientMotionPath(
                    &continuousPoints, kin, definition, m_machineConfig->workpieceSetupTransform(),
                    m_machineConfig->headToolGeometry(), &solveError, &initialPose)) {
                // 中文翻译：轮廓间的空程及轮廓连续五轴求解失败
                plan.failureReason = tr("Continuous rapid and contour five-axis solve failed between contours %1 and %2: %3")
                    .arg(transition.fromContourId).arg(transition.toContourId).arg(solveError);
                break;
            }
            std::array<double, lcnc::MachineAxisLayout::kMaxAxes> previousAxes{
                source.machineX, source.machineY, source.machineZ,
                source.machineR1, source.machineR2};
            for (int index = 0; index < transition.segments.size(); ++index) {
                const MachineCoord& coordinate = continuousPoints[
                    static_cast<std::size_t>(index + 1)].machineCoord;
                lcnc::cam::RapidPose& pose = transition.segments[index].target;
                pose.axes = {coordinate.x, coordinate.y, coordinate.z,
                             coordinate.r1, coordinate.r2};
                pose.activeMask = semanticMaskFor(coordinate.solvedPose.activeMask);
                pose.rotaryAxis1Name = coordinate.r1Name;
                pose.rotaryAxis2Name = coordinate.r2Name;
                pose.kinematicAxes = coordinate.solvedPose.values;
                pose.kinematicAxisMask = coordinate.solvedPose.activeMask;
                std::uint8_t moved = 0;
                for (int axis = 0; axis < lcnc::MachineAxisLayout::kMaxAxes; ++axis) {
                    if (std::abs(pose.axes[axis] - previousAxes[axis]) > 1e-9)
                        moved |= static_cast<std::uint8_t>(1u << axis);
                }
                transition.segments[index].movingAxisMask = moved;
                transition.segments[index].synchronization =
                    lcnc::cam::RapidSynchronization::Coordinated;
                previousAxes = pose.axes;
            }

            assignExportCoordinate(&targetContour->leadInPoint,
                                   continuousPoints[rapidEndIndex].machineCoord);
            for (int index = 0; index < targetPoints.size(); ++index) {
                assignExportCoordinate(&targetPoints[index],
                    continuousPoints[rapidEndIndex + 1u
                        + static_cast<std::size_t>(index)].machineCoord);
            }
        }
        if (!plan.failureReason.isEmpty())
            plan.stale = false;
    }
    const auto collisionConfigSnapshot = this->collisionConfiguration();
    plan.collision.key = key.toolpathRevision ^ key.orderHash ^ key.environmentRevision;
    plan.collision.blockWarning = m_config.blockMachiningOnCollisionWarning();
    if (!collisionConfigSnapshot.enabled) {
        plan.collision.state = lcnc::cam::CollisionValidationState::Disabled;
        plan.collision.complete = true;
    } else {
        plan.collision.state = lcnc::cam::CollisionValidationState::Pending;
        plan.collision.complete = false;
    }
    if (plan.isExecutable() && collisionConfigSnapshot.enabled) {
        if (!collisionConfigSnapshot.valid) {
            plan.collision.state = lcnc::cam::CollisionValidationState::Indeterminate;
            plan.collision.complete = true;
            plan.collision.failureReason = tr("Collision detection is enabled but the source configuration is incomplete");
            plan.failureReason = plan.collision.failureReason;
        } else {
            // Collision validation belongs to the generated CAM motion plan,
            // not to the optional simulation UI.  This includes the
            // cutter/workpiece-only fallback when no machine model is loaded.
            // 中文翻译：碰撞校验归属于生成后的 CAM 运动计划，而非可选仿真界面；未加载机台时也校验切割头/工件组合。
            plan.fullEnvironmentVerificationPending = true;
        }
    }
    plan.key = key;
    snapshot.travelPlan = plan;
    m_travelPlanCache = plan;
}

lcnc::cam::ToolpathExportSnapshot CamModule::exportToolpathSnapshotForOrder(
    const QVector<std::uint64_t>& orderedContourIds) const
{
    if (orderedContourIds.isEmpty())
        return exportToolpathBaseSnapshot();

    std::vector<LaserContour> orderedContours;
    orderedContours.reserve(static_cast<std::size_t>(orderedContourIds.size()));
    for (std::uint64_t id : orderedContourIds) {
        const int contourIdx = contourIndexById(static_cast<lcnc::cam::ContourId>(id));
        if (contourIdx < 0 || contourIdx >= toolpathRef().contourCount())
            continue;
        orderedContours.push_back(toolpathRef().contour(contourIdx));
    }

    LCNC_INFO(lcnc::LogCode::Generic,
              "cam.toolpath: exported ordered snapshot with resolved machine coordinates, contours={}",
              orderedContours.size());

    auto snapshot = buildToolpathExportSnapshot(
        orderedContours,
        toolpathRevision(),
        // 中文翻译：CAM 当前无刀路；CAM 有序规划刀路快照已导出
        orderedContours.empty() ? tr("CAM currently has no tool path") : tr("CAM orderly planned tool path snapshot has been exported"));
    applyToolMotionOffsets(snapshot);
    attachTravelPlan(snapshot);
    attachMotionPlan(snapshot);
    return snapshot;
}

lcnc::cam::InitialApproachSnapshot CamModule::planInitialApproach(
    const lcnc::cam::InitialApproachRequest& request,
    std::atomic_bool* cancelRequested) const
{
    lcnc::cam::InitialApproachSnapshot result;
    result.toolpathRevision = request.toolpathRevision;
    result.targetContourId = request.targetContourId;
    result.machineConfigurationFingerprint = request.machineConfigurationFingerprint;
    QElapsedTimer planningElapsed;
    planningElapsed.start();
    LCNC_INFO(lcnc::LogCode::Generic,
              "stage=cam.initial_approach.plan event=begin mode={} contour={} toolpath_revision={}",
              request.planningMode == lcnc::cam::InitialApproachPlanningMode::Manual
                  ? "manual" : "automatic",
              request.targetContourId, request.toolpathRevision);
    const auto planningLog = qScopeGuard([&] {
        LCNC_INFO(lcnc::LogCode::Generic,
                  "stage=cam.initial_approach.plan event=end result={} mode={} contour={} segments={} collision_state={} elapsed_ms={} reason='{}'",
                  result.isExecutable(result.collision.blockWarning) ? "success" : "failed",
                  request.planningMode == lcnc::cam::InitialApproachPlanningMode::Manual
                      ? "manual" : "automatic",
                  request.targetContourId, result.transition.segments.size(),
                  static_cast<int>(result.collision.state), planningElapsed.elapsed(),
                  result.failureReason.toStdString());
    });
    const auto cancelled = [cancelRequested] { return cancelRequested && cancelRequested->load(); };
    if (cancelled()) {
        result.failureReason = tr("Initial approach planning was cancelled");
        return result;
    }

    // Capture all QObject/XCAF-owned state on the CAM thread.  The subsequent
    // OCC planning uses only detached values and is therefore safe to call
    // from the Process workflow thread.
    struct Captured {
        lcnc::cam::ToolpathExportSnapshot snapshot;
        QVector<MachineAxisDef> axes;
        QString configType;
        gp_Trsf setup;
        QMap<QString, QString> assignments;
        QMap<QString, QString> mounts;
        TopoDS_Shape workpiece;
        TopoDS_Shape cutter;
        gp_Pnt cutterModelPosition;
        lcnc::MachineModeDefinition definition;
        lcnc::WorkpieceSetupTransform workpieceSetup;
        lcnc::HeadToolGeometry headToolGeometry;
        bool hasMachineModel{false};
    } captured;
    auto capture = [this, &captured, &request] {
        const auto order = contourSequenceSnapshot();
        captured.snapshot = exportToolpathSnapshotForOrder(
            QVector<std::uint64_t>(order.orderedContourIds.cbegin(), order.orderedContourIds.cend()));
        if (const auto* kin = kinematics()) {
            captured.axes = kin->axes();
            captured.configType = kin->configType();
            captured.setup = kin->workpieceSetupTransform();
            captured.assignments = kin->shapeAssignments();
            captured.mounts = kin->wpcMounts();
        }
        captured.workpiece = m_workpieceShape;
        captured.cutter = const_cast<CamModule*>(this)->cutterCollisionProxyShape(nullptr);
        captured.cutterModelPosition = m_cutterHeadModelPosition;
        if (m_machineConfig) {
            captured.definition = m_machineConfig->modeDefinition(captured.snapshot.machiningMode);
            captured.workpieceSetup = m_machineConfig->workpieceSetupTransform();
            captured.headToolGeometry = m_machineConfig->headToolGeometry();
        }
        if (LcncDocument* machine = machineDocument()) {
            captured.hasMachineModel = machine->entityLabels(
                LcncDocument::EntityKind::Machine).Length() > 0;
        }
    };
    if (QThread::currentThread() == thread())
        capture();
    else
        QMetaObject::invokeMethod(const_cast<CamModule*>(this), capture, Qt::BlockingQueuedConnection);

    if (cancelled()) {
        result.failureReason = tr("Initial approach planning was cancelled");
        return result;
    }
    if (captured.snapshot.revision != request.toolpathRevision
        || captured.snapshot.machineConfigurationFingerprint != request.machineConfigurationFingerprint) {
        result.failureReason = tr("CAM toolpath or machine configuration changed before initial approach planning");
        return result;
    }
    const auto contourIt = std::find_if(captured.snapshot.contours.cbegin(), captured.snapshot.contours.cend(),
        [&request](const lcnc::cam::ToolpathExportContour& contour) {
            return contour.contourId == request.targetContourId;
        });
    if (contourIt == captured.snapshot.contours.cend()
        || !contourIt->leadInPoint.machineCoordValid) {
        result.failureReason = tr("The first contour has no solved lead-in machine coordinates");
        return result;
    }
    if (captured.cutter.IsNull() || captured.workpiece.IsNull() || !captured.definition.isValid()) {
        result.failureReason = tr("Initial approach planning geometry or machine definition is unavailable");
        return result;
    }

    MachineKinematics currentKinematics;
    currentKinematics.setAxes(captured.axes, captured.configType);
    currentKinematics.setWorkpieceSetupTransform(captured.setup);
    for (auto it = captured.assignments.cbegin(); it != captured.assignments.cend(); ++it)
        currentKinematics.assignShape(it.key(), it.value());
    for (auto it = captured.mounts.cbegin(); it != captured.mounts.cend(); ++it)
        currentKinematics.mountWorkpiece(it.key(), it.value());

    lcnc::SolvedMachinePose currentPose;
    for (int index = 0; index < captured.definition.interpolatedAxes.count; ++index) {
        const QString axisName = captured.definition.interpolatedAxes.axes[index].name;
        const auto value = request.axisPositions.constFind(axisName);
        if (value == request.axisPositions.cend() || !std::isfinite(value.value())) {
            result.failureReason = tr("Controller feedback is missing axis %1").arg(axisName);
            return result;
        }
        currentPose.setValue(index, value.value());
        currentKinematics.setAxisPosition(axisName, value.value());
    }
    currentPose.valid = true;

    // Stop -> Start is a fresh run. Keep measured APOS and the first contour's
    // committed lead-in pose as immutable physical-axis boundaries. Manual
    // mode uses the configured absolute-Z sequence; automatic mode builds a
    // joint-space safe-zone route (Z -> SafeXY -> SafeAC -> Z approach).
    // 中文翻译：停止后再开始属于全新加工；以实测 APOS 和首轮廓已提交下刀位姿
    // 作为不可替换的物理轴边界，重新生成三阶段首段，不复用上次运行的过渡段。
    QString cutterAxisName = QStringLiteral("Z");
    if (captured.definition.mode == lcnc::MachiningMode::SimultaneousHead5Axis) {
        QString primaryAxisName;
        QString secondaryAxisName;
        for (int index = 0; index < captured.definition.interpolatedAxes.count; ++index) {
            const auto& slot = captured.definition.interpolatedAxes.axes[index];
            if (slot.role == lcnc::MachineAxisRole::HeadTiltPrimary)
                primaryAxisName = slot.name;
            else if (slot.role == lcnc::MachineAxisRole::HeadTiltSecondary)
                secondaryAxisName = slot.name;
        }
        const MachineAxisDef* primary = currentKinematics.findAxis(primaryAxisName);
        const MachineAxisDef* secondary = currentKinematics.findAxis(secondaryAxisName);
        if (primary && secondary) {
            cutterAxisName = secondary->parentAxis == primary->name
                ? secondary->name
                : (primary->parentAxis == secondary->name ? primary->name : secondary->name);
        }
    }

    const int zAxisIndex = captured.definition.interpolatedAxes.indexOfRole(
        lcnc::MachineAxisRole::LinearZ);
    const QString zAxisName = zAxisIndex >= 0
        ? captured.definition.interpolatedAxes.axes[zAxisIndex].name : QString();
    const MachineAxisDef* zAxis = currentKinematics.findAxis(zAxisName);
    const bool automatic = request.planningMode
        == lcnc::cam::InitialApproachPlanningMode::Automatic;
    if (!zAxis || (!automatic && !std::isfinite(request.safetyAxisZ))
        || std::abs(zAxis->direction.Z()) <= 1e-9) {
        // 中文翻译：配置的首段 Z 方向或安全坐标无效。
        result.failureReason = tr("The configured initial-segment Z direction or safety coordinate is invalid");
        return result;
    }

    lcnc::SolvedMachinePose committedLeadInPose;
    committedLeadInPose.values = contourIt->leadInPoint.machineAxes;
    committedLeadInPose.activeMask = contourIt->leadInPoint.machineAxisMask;
    committedLeadInPose.valid = contourIt->leadInPoint.machineCoordValid;

    const auto rapidPoseFor = [&](const lcnc::SolvedMachinePose& physicalPose) {
        lcnc::cam::RapidPose pose;
        pose.kinematicAxes = physicalPose.values;
        pose.kinematicAxisMask = physicalPose.activeMask;
        int poseRotarySlot = 0;
        for (const MachineAxisDef& axis : captured.axes)
            currentKinematics.setAxisPosition(axis.name, axis.currentPos);
        for (int axisIndex = 0;
             axisIndex < captured.definition.interpolatedAxes.count;
             ++axisIndex) {
            const auto& slot = captured.definition.interpolatedAxes.axes[axisIndex];
            const double value = physicalPose.values[axisIndex];
            currentKinematics.setAxisPosition(slot.name, value);
            switch (slot.role) {
            case lcnc::MachineAxisRole::LinearX:
                pose.axes[0] = value; pose.activeMask |= 0x01u; break;
            case lcnc::MachineAxisRole::LinearY:
                pose.axes[1] = value; pose.activeMask |= 0x02u; break;
            case lcnc::MachineAxisRole::LinearZ:
                pose.axes[2] = value; pose.activeMask |= 0x04u; break;
            default:
                if (poseRotarySlot == 0) {
                    pose.axes[3] = value;
                    pose.rotaryAxis1Name = slot.name;
                    pose.activeMask |= 0x08u;
                } else if (poseRotarySlot == 1) {
                    pose.axes[4] = value;
                    pose.rotaryAxis2Name = slot.name;
                    pose.activeMask |= 0x10u;
                }
                ++poseRotarySlot;
                break;
            }
        }
        gp_Pnt tcp = captured.cutterModelPosition;
        tcp.Transform(currentKinematics.computeAxisTransform(cutterAxisName));
        gp_Vec normal(0.0, 0.0, 1.0);
        if (captured.definition.mode == lcnc::MachiningMode::SimultaneousHead5Axis) {
            normal = gp_Vec(-captured.headToolGeometry.zeroBeamX,
                            -captured.headToolGeometry.zeroBeamY,
                            -captured.headToolGeometry.zeroBeamZ);
            normal.Transform(currentKinematics.computeAxisTransform(cutterAxisName));
        }
        if (normal.SquareMagnitude() <= Precision::SquareConfusion())
            normal = gp_Vec(0.0, 0.0, 1.0);
        normal.Normalize();
        pose.tcpX = tcp.X(); pose.tcpY = tcp.Y(); pose.tcpZ = tcp.Z();
        pose.surfaceNormalX = normal.X();
        pose.surfaceNormalY = normal.Y();
        pose.surfaceNormalZ = normal.Z();
        return pose;
    };
    const auto semanticChangedMask = [](const lcnc::cam::RapidPose& lhs,
                                        const lcnc::cam::RapidPose& rhs) {
        std::uint8_t mask = 0;
        for (int axisIndex = 0; axisIndex < lcnc::MachineAxisLayout::kMaxAxes;
             ++axisIndex) {
            if (std::abs(lhs.axes[axisIndex] - rhs.axes[axisIndex]) > 1e-9)
                mask |= static_cast<std::uint8_t>(1u << axisIndex);
        }
        return mask;
    };

    const auto translatedAxisFailure = [this](QString detail) {
        if (detail == QStringLiteral("Initial approach axis input is invalid"))
            return tr("Initial approach axis input is invalid");
        if (detail == QStringLiteral("Initial approach axis layout is invalid"))
            return tr("Initial approach axis layout is invalid");
        if (detail == QStringLiteral("Initial approach requires an active linear Z axis"))
            return tr("Initial approach requires an active linear Z axis");
        if (detail == QStringLiteral("Initial approach contains a non-finite axis value"))
            return tr("Initial approach contains a non-finite axis value");
        if (detail == QStringLiteral("Initial approach safety Z is below the first cutting height"))
            return tr("Initial approach safety Z is below the first cutting height");
        return detail;
    };
    const auto buildTransition = [&](const lcnc::cam_algo::InitialApproachAxisPlan& axisPlan,
                                     lcnc::cam::RapidPose* initialSource) {
        lcnc::cam::RapidTransition transition;
        transition.fromContourId = 0;
        transition.toContourId = contourIt->contourId;
        transition.pathKind = automatic
            ? lcnc::cam::RapidPathKind::InitialSafeZone
            : lcnc::cam::RapidPathKind::InitialAxisThreePhase;
        lcnc::SolvedMachinePose previousPhysical = currentPose;
        lcnc::cam::RapidPose previousRapid = rapidPoseFor(previousPhysical);
        if (initialSource)
            *initialSource = previousRapid;
        transition.surfacePreviewPoints.append(
            {previousRapid.tcpX, previousRapid.tcpY, previousRapid.tcpZ,
             previousRapid.surfaceNormalX, previousRapid.surfaceNormalY,
             previousRapid.surfaceNormalZ});
        for (const auto& waypoint : axisPlan.waypoints) {
            const int divisions = automatic ? std::clamp([&] {
                double greatestDelta = 0.0;
                for (int axisIndex = 0;
                     axisIndex < captured.definition.interpolatedAxes.count;
                     ++axisIndex) {
                    greatestDelta = std::max(greatestDelta,
                        std::abs(waypoint.pose.values[axisIndex]
                                 - previousPhysical.values[axisIndex]));
                }
                return static_cast<int>(std::ceil(greatestDelta / 2.0));
            }(), 1, 128) : 1;
            for (int division = 1; division <= divisions; ++division) {
                const double t = static_cast<double>(division) / divisions;
                lcnc::SolvedMachinePose sample = previousPhysical;
                for (int axisIndex = 0;
                     axisIndex < captured.definition.interpolatedAxes.count;
                     ++axisIndex) {
                    sample.values[axisIndex] = previousPhysical.values[axisIndex]
                        + (waypoint.pose.values[axisIndex]
                           - previousPhysical.values[axisIndex]) * t;
                }
                sample.activeMask = previousPhysical.activeMask | waypoint.pose.activeMask;
                sample.valid = true;
                lcnc::cam::RapidPose rapid = rapidPoseFor(sample);
                const std::uint8_t movingMask = semanticChangedMask(previousRapid, rapid);
                transition.segments.append(
                    {rapid, movingMask, lcnc::cam::RapidSynchronization::Coordinated,
                     0.0, m_config.cutterCollisionClearanceMm(), waypoint.phase});
                transition.pathLengthMm += gp_Pnt(previousRapid.tcpX,
                    previousRapid.tcpY, previousRapid.tcpZ).Distance(
                        gp_Pnt(rapid.tcpX, rapid.tcpY, rapid.tcpZ));
                transition.surfacePreviewPoints.append(
                    {rapid.tcpX, rapid.tcpY, rapid.tcpZ,
                     rapid.surfaceNormalX, rapid.surfaceNormalY,
                     rapid.surfaceNormalZ});
                previousRapid = rapid;
            }
            previousPhysical = waypoint.pose;
        }
        transition.minimumClearanceMm = m_config.cutterCollisionClearanceMm();
        return transition;
    };
    struct InitialCollisionPathRequest {
        lcnc::cam::CollisionSafetyPathRequest request;
        QVector<lcnc::cam::RapidSegmentPhase> phases;
    };
    const auto collisionRequestFor = [&](const lcnc::cam::RapidTransition& transition,
                                         const lcnc::cam::RapidPose& initialSource) {
        InitialCollisionPathRequest collisionPath;
        auto& collisionRequest = collisionPath.request;
        collisionRequest.environmentRevision =
            captured.snapshot.travelPlan.key.environmentRevision;
        collisionRequest.scope = lcnc::cam::CollisionSafetyScope::MachineOnly;
        collisionRequest.clearanceMm = m_config.cutterCollisionClearanceMm();
        collisionRequest.blockWarning = m_config.blockMachiningOnCollisionWarning();
        lcnc::cam::RapidPose collisionSource = initialSource;
        collisionRequest.poses.append({collisionSource, contourIt->workpieceEntry,
                                       lcnc::cam::CamMotionPhase::Rapid});
        collisionPath.phases.append(transition.segments.isEmpty()
            ? lcnc::cam::RapidSegmentPhase::Retract
            : transition.segments.constFirst().phase);
        for (const auto& segment : transition.segments) {
            double greatestNormalizedDelta = 0.0;
            for (int axisIndex = 0;
                 axisIndex < captured.definition.interpolatedAxes.count;
                 ++axisIndex) {
                const auto role = captured.definition.interpolatedAxes.axes[axisIndex].role;
                const double step = role == lcnc::MachineAxisRole::LinearX
                        || role == lcnc::MachineAxisRole::LinearY
                        || role == lcnc::MachineAxisRole::LinearZ ? 1.0 : 0.5;
                greatestNormalizedDelta = std::max(greatestNormalizedDelta,
                    std::abs(segment.target.kinematicAxes[axisIndex]
                             - collisionSource.kinematicAxes[axisIndex]) / step);
            }
            const int divisions = std::clamp(
                static_cast<int>(std::ceil(greatestNormalizedDelta)), 1, 512);
            for (int division = 1; division <= divisions; ++division) {
                const double t = static_cast<double>(division) / divisions;
                lcnc::SolvedMachinePose sample;
                for (int axisIndex = 0;
                     axisIndex < captured.definition.interpolatedAxes.count;
                     ++axisIndex) {
                    sample.values[axisIndex] = collisionSource.kinematicAxes[axisIndex]
                        + (segment.target.kinematicAxes[axisIndex]
                           - collisionSource.kinematicAxes[axisIndex]) * t;
                }
                sample.activeMask = collisionSource.kinematicAxisMask
                    | segment.target.kinematicAxisMask;
                sample.valid = true;
                collisionRequest.poses.append({rapidPoseFor(sample),
                    contourIt->workpieceEntry, lcnc::cam::CamMotionPhase::Rapid});
                collisionPath.phases.append(segment.phase);
            }
            collisionSource = segment.target;
        }
        return collisionPath;
    };

    const bool validateMachine =
        lcnc::cam_algo::shouldValidateInitialApproachMachine(
            captured.hasMachineModel,
            automatic ? lcnc::cam_algo::InitialApproachAxisMode::AutomaticSafeZone
                      : lcnc::cam_algo::InitialApproachAxisMode::Manual,
            request.collisionCheckEnabled);
    if (validateMachine) {
        lcnc::cam::CollisionSafetyPathRequest currentPoseRequest;
        currentPoseRequest.environmentRevision =
            captured.snapshot.travelPlan.key.environmentRevision;
        currentPoseRequest.scope = lcnc::cam::CollisionSafetyScope::MachineOnly;
        currentPoseRequest.clearanceMm = m_config.cutterCollisionClearanceMm();
        currentPoseRequest.blockWarning = m_config.blockMachiningOnCollisionWarning();
        currentPoseRequest.poses.append({rapidPoseFor(currentPose),
            contourIt->workpieceEntry, lcnc::cam::CamMotionPhase::Rapid});
        auto currentPoseCollision = validateCollisionPath(currentPoseRequest,
                                                          cancelRequested);
        if (currentPoseCollision.blocksExecution(currentPoseCollision.blockWarning)) {
            result.collision = std::move(currentPoseCollision);
            result.failureReason = result.collision.failureReason;
            return result;
        }
    }

    QVector<double> safetyCandidates;
    if (!automatic) {
        safetyCandidates.append(request.safetyAxisZ);
    } else {
        // Automatic mode derives a physical safe-height search interval from
        // the committed first point, current APOS, rapid offset and Z limits.
        // It never interprets the manual absolute-Z setting as its solution.
        // 中文翻译：自动模式根据首点、当前 APOS、空程高度和 Z 限位搜索安全域，禁止复用手动绝对 Z 设置。
        const double liftMm = std::max({1.0, contourIt->rapidOffsetMm,
                                       m_config.cutterCollisionClearanceMm()});
        const double stepMm = std::clamp(liftMm, 2.0, 10.0);
        const auto candidates = lcnc::cam_algo::planAutomaticSafetyZCandidates(
            currentPose.values[zAxisIndex],
            committedLeadInPose.values[zAxisIndex], zAxis->direction.Z(),
            zAxis->minVal, zAxis->maxVal, liftMm, stepMm);
        if (!candidates.isValid()) {
            // 中文翻译：自动首段规划无法在 Z 轴限位内找到安全高度
            result.failureReason = candidates.failureReason
                    == QStringLiteral("Automatic initial approach cannot find a safe Z height within machine limits")
                ? tr("Automatic initial approach cannot find a safe Z height within machine limits")
                : tr("Automatic initial approach Z search input is invalid");
            return result;
        }
        safetyCandidates = candidates.axisCoordinates;
    }

    QString lastFailure;
    lcnc::cam::CollisionValidationSnapshot lastCollisionResult;
    bool hasCollisionResult = false;
    for (double safetyAxisZ : std::as_const(safetyCandidates)) {
        if (cancelled()) {
            result.failureReason = tr("Initial approach planning was cancelled");
            return result;
        }
        const auto axisMode = automatic
            ? lcnc::cam_algo::InitialApproachAxisMode::AutomaticSafeZone
            : lcnc::cam_algo::InitialApproachAxisMode::Manual;
        const auto axisPlan = lcnc::cam_algo::planInitialApproachAxes(
            captured.definition.interpolatedAxes, currentPose,
            committedLeadInPose, safetyAxisZ, zAxis->direction.Z(), axisMode);
        if (!axisPlan.isValid()) {
            lastFailure = tr("Initial segment planning failed: %1")
                .arg(translatedAxisFailure(axisPlan.failureReason));
            if (!automatic)
                break;
            continue;
        }
        if (axisPlan.resolvedSafetyAxisZ < zAxis->minVal - 1e-9
            || axisPlan.resolvedSafetyAxisZ > zAxis->maxVal + 1e-9) {
            // 中文翻译：配置的首段 Z 安全坐标超出机床限位。
            lastFailure = tr("The configured initial-segment Z safety coordinate is outside the machine limits");
            if (!automatic)
                break;
            continue;
        }

        lcnc::cam::RapidPose initialSource;
        lcnc::cam::RapidTransition transition = buildTransition(axisPlan, &initialSource);
        if (!transition.isValid()) {
            lastFailure = tr("CAM could not create an initial approach path");
            if (!automatic)
                break;
            continue;
        }
        const auto& terminalPose = transition.segments.constLast().target;
        bool terminalMatches = true;
        for (int axisIndex = 0;
             axisIndex < captured.definition.interpolatedAxes.count;
             ++axisIndex) {
            if (std::abs(terminalPose.kinematicAxes[axisIndex]
                         - committedLeadInPose.values[axisIndex]) > 0.01) {
                terminalMatches = false;
                break;
            }
        }
        if (!terminalMatches) {
            lastFailure = tr("Initial segment did not reach the committed first cutting pose");
            if (!automatic)
                break;
            continue;
        }

        lcnc::cam::CollisionValidationSnapshot collisionResult;
        collisionResult.key = result.toolpathRevision ^ result.targetContourId;
        collisionResult.blockWarning = m_config.blockMachiningOnCollisionWarning();
        InitialCollisionPathRequest collisionPath;
        if (!validateMachine) {
            // No machine geometry means there is no machine collision domain
            // to validate. Workpiece/full-toolpath validation remains separate.
            // 中文翻译：未加载机台模型时跳过首段机台碰撞校验，不把工件碰撞源误当成机台源。
            collisionResult.state = lcnc::cam::CollisionValidationState::Disabled;
            collisionResult.complete = true;
            collisionResult.nodeStates.fill(collisionResult.state,
                                             transition.segments.size());
        } else {
            collisionPath = collisionRequestFor(transition, initialSource);
            collisionResult = validateCollisionPath(collisionPath.request,
                                                     cancelRequested);
        }
        if (!collisionResult.blocksExecution(collisionResult.blockWarning)) {
            result.transition = std::move(transition);
            result.collision = std::move(collisionResult);
            return result;
        }
        lastCollisionResult = collisionResult;
        hasCollisionResult = true;
        lastFailure = collisionResult.failureReason;
        if (!automatic)
            break;
        // All automatic safety-Z candidates are ordered outward and share the
        // first candidate's retract prefix. Once that prefix is blocked, every
        // farther candidate must cross the same forbidden segment.
        // 中文翻译：自动安全 Z 候选按向外方向递增，并共享首个候选的退回前缀；
        // 该前缀一旦碰撞，继续尝试更远的候选只会重复同一禁区。
        lcnc::cam::RapidSegmentPhase firstBlockedPhase =
            lcnc::cam::RapidSegmentPhase::Traverse;
        bool hasBlockedPhase = false;
        if (!collisionResult.intervals.isEmpty()) {
            const int blockedPose = collisionResult.intervals.constFirst().firstNode;
            if (blockedPose >= 0 && blockedPose < collisionPath.phases.size()) {
                firstBlockedPhase = collisionPath.phases.at(blockedPose);
                hasBlockedPhase = true;
            }
        }
        if (!lcnc::cam_algo::shouldRetryAutomaticSafetyZCandidate(
                collisionResult.state, hasBlockedPhase, firstBlockedPhase)) {
            break;
        }
    }

    if (lastFailure.isEmpty()) {
        // 中文翻译：自动首段规划在机台安全域内没有找到可执行路径
        lastFailure = tr("Automatic initial approach found no executable path in the machine safety domain");
    }
    result.failureReason = lastFailure;
    if (hasCollisionResult)
        result.collision = std::move(lastCollisionResult);
    else {
        result.collision.state = lcnc::cam::CollisionValidationState::Indeterminate;
        result.collision.complete = true;
    }
    result.collision.failureReason = lastFailure;
    return result;
}

QList<lcnc::MachiningMode> CamModule::supportedMachiningModes() const
{
    return m_machineConfig ? m_machineConfig->supportedMachiningModes()
                           : QList<lcnc::MachiningMode>{};
}

lcnc::MachiningMode CamModule::machiningMode() const
{
    return m_camData ? m_camData->machiningMode() : lcnc::MachiningMode::Planar3Axis;
}

bool CamModule::setMachiningMode(lcnc::MachiningMode mode)
{
    if (!m_camData || !m_machineConfig || !m_machineConfig->supportsMachiningMode(mode))
        return false;
    if (m_camData->machiningMode() == mode) return true;
    const lcnc::MachineModeDefinition definition = m_machineConfig->modeDefinition(mode);
    m_camData->setMachiningMode(mode);
    m_camData->setMachineAxisLayout(definition.interpolatedAxes);
    m_camData->setSolverId(definition.solverId);
    m_camData->setSolverVersion(definition.solverVersion);
    m_camData->setSolvedMachineConfigurationFingerprint(QString());
    m_camData->invalidatePipelineAfter(lcnc::cam::CamPipelineStage::GeometricToolpath,
        QStringLiteral("Machining mode changed; machine coordinates must be solved again"));
    for (LaserContour& contour : toolpathRef().contours()) {
        for (ToolpathPoint& point : contour.points) point.machineCoord = {};
        if (contour.leadInSolution.valid) contour.leadInSolution.point.machineCoord = {};
    }
    m_camData->markDirty(true);
    if (m_travelPathRenderer) m_travelPathRenderer->erase(activeGuiDocument());
    refreshToolpathDisplay();
    emit machiningModeChanged(mode);
    emit pipelineStageChanged(lcnc::cam::CamPipelineStage::MachineSolve);
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Cam);
    return true;
}

lcnc::WorkpieceSetupTransform CamModule::workpieceSetupTransform() const
{
    return m_machineConfig ? m_machineConfig->workpieceSetupTransform()
                           : lcnc::WorkpieceSetupTransform{};
}

bool CamModule::setWorkpieceSetupTransform(const lcnc::WorkpieceSetupTransform& setup)
{
    if (!m_machineConfig) return false;
    const auto current = m_machineConfig->workpieceSetupTransform();
    const auto close = [](double lhs, double rhs) { return std::abs(lhs - rhs) <= 1e-9; };
    if (close(current.x, setup.x) && close(current.y, setup.y) && close(current.z, setup.z)
        && close(current.rotationXDeg, setup.rotationXDeg)
        && close(current.rotationYDeg, setup.rotationYDeg)
        && close(current.rotationZDeg, setup.rotationZDeg)) return true;
    // MachineConfigurationService is the single authority. Its synchronous
    // change signal refreshes kinematics/view state and invalidates any active
    // project's solved-machine-coordinate stage.
    // 中文翻译：机床配置服务是唯一权威；其同步变更信号负责刷新运动学/视图并使当前工程机床坐标阶段失效。
    m_machineConfig->setWorkpieceSetupTransform(setup);
    emit workpieceSetupTransformChanged();
    return true;
}

lcnc::cam::ToolpathExportSnapshot CamModule::buildToolpathExportSnapshot(
    const std::vector<LaserContour>& contours,
    std::uint64_t revision,
    const QString& description) const
{
    lcnc::cam::ToolpathExportSnapshot snapshot;
    snapshot.revision = revision;
    snapshot.description = description;
    if (m_camData) {
        snapshot.machiningMode = m_camData->machiningMode();
        snapshot.machineAxisLayout = m_camData->machineAxisLayout();
        snapshot.solverId = m_camData->solverId();
        snapshot.solverVersion = m_camData->solverVersion();
    }
    if (m_camData)
        snapshot.machineConfigurationFingerprint =
            m_camData->solvedMachineConfigurationFingerprint();

    auto layerForId = [this](std::uint64_t layerId) -> const ToolpathLayer* {
        for (const ToolpathLayer& layer : toolpathRef().layers()) {
            if (layer.layerId == layerId)
                return &layer;
        }
        return nullptr;
    };

    MachineKinematics* kin = kinematics();
    const lcnc::MachineModeDefinition definition = m_machineConfig
        ? m_machineConfig->modeDefinition(snapshot.machiningMode)
        : lcnc::MachineModeDefinition{};
    const QList<MachineAxisDef> projectionBaselineAxes = kin
        ? lcnc::cam_algo::offlinePlanningAxisBaseline(kin->axes(), definition)
        : QList<MachineAxisDef>{};
    MachineKinematics projectionKinematics;
    if (kin) {
        projectionKinematics.setAxes(projectionBaselineAxes, kin->configType());
        projectionKinematics.setWorkpieceSetupTransform(kin->workpieceSetupTransform());
        for (auto it = kin->wpcMounts().cbegin(); it != kin->wpcMounts().cend(); ++it)
            projectionKinematics.mountWorkpiece(it.key(), it.value());
    }
    const auto workpieceReferenceTransform = [kin, &projectionKinematics](
                                                  const QString& workpieceEntry) {
        if (!kin)
            return gp_Trsf{};
        return projectionKinematics.computeWpcTransform(workpieceEntry);
    };

    for (const LaserContour& contour : contours) {
        const ToolpathLayer* layer = layerForId(contour.layerId);
        lcnc::cam::ToolpathExportContour exportedContour;
        exportedContour.contourId = contour.contourId;
        exportedContour.layerId = contour.layerId;
        exportedContour.contourName = contour.name;
        exportedContour.layerName = layer ? layer->name : QString();
        exportedContour.toolName = layer ? layer->toolName : QString();
        exportedContour.workpieceEntry = contour.workpieceEntry;
        exportedContour.enabled = contour.enabled;
        exportedContour.layerEnabled = layer ? layer->enabled : true;
        const bool machineSolveReady = m_camData && m_camData->hasCompletePipelineChain();
        exportedContour.needsRecalculation = contour.needsRecalculation
            || (m_camData && m_camData->generationParamsDirty())
            || !machineSolveReady;
        if (contour.needsRecalculation)
            // 中文翻译：轮廓参数或起点尚未重新计算
            exportedContour.recalculationReason = tr("Contour parameters or starting point have not been recalculated");
        else if (m_camData && m_camData->generationParamsDirty())
            // 中文翻译：全局生成参数尚未应用
            exportedContour.recalculationReason = tr("Global build parameters have not been applied yet");
        else if (!machineSolveReady)
            // 中文翻译：五阶段 CAM 流程未完成或上游版本链不一致
            exportedContour.recalculationReason = tr("The five-stage CAM process is incomplete or the upstream version chain is inconsistent");
        exportedContour.pointCount = static_cast<int>(contour.points.size());
        exportedContour.cuttingOffsetMm = contour.appliedParams.cuttingOffsetMm;
        exportedContour.rapidOffsetMm = contour.appliedParams.rapidOffsetMm;

        // 计算世界坐标系下的几何端点：cut start = points.front()；end = points.back()；
        // start = leadIn 起点（若 leadIn.valid）否则等于 cut start。
        if (!contour.points.empty()) {
            const gp_Pnt cutStartLocal = contour.points.front().position;
            const gp_Pnt endLocal      = contour.points.back().position;
            gp_Pnt cutStartWorld = cutStartLocal;
            gp_Pnt endWorld      = endLocal;
            gp_Pnt startWorld    = cutStartLocal;
            bool   hasLeadIn = false;

            // Rapid reference geometry must use one common frozen workpiece
            // posture.  Per-node solved A/C poses belong to execution and
            // collision nodes; mixing them into these endpoints creates a
            // curve whose samples live in different coordinate frames.
            // 中文翻译：空程参考几何必须统一使用冻结工件姿态；各节点已求解 A/C
            // 只属于执行和碰撞节点，不能混入端点，否则曲线各点会落在不同坐标系。
            const gp_Trsf referenceWpc =
                workpieceReferenceTransform(contour.workpieceEntry);
            cutStartWorld.Transform(referenceWpc);
            endWorld.Transform(referenceWpc);

            if (contour.leadInSolution.valid) {
                gp_Pnt leadStartWorld = contour.leadInSolution.point.position;
                leadStartWorld.Transform(referenceWpc);
                startWorld = leadStartWorld;
                hasLeadIn = contour.leadInSolution.point.machineCoord.valid;
            } else {
                startWorld = cutStartWorld;
            }

            exportedContour.cutStartX = cutStartWorld.X();
            exportedContour.cutStartY = cutStartWorld.Y();
            exportedContour.cutStartZ = cutStartWorld.Z();
            exportedContour.endX      = endWorld.X();
            exportedContour.endY      = endWorld.Y();
            exportedContour.endZ      = endWorld.Z();
            exportedContour.startX    = startWorld.X();
            exportedContour.startY    = startWorld.Y();
            exportedContour.startZ    = startWorld.Z();
            exportedContour.hasLeadIn = hasLeadIn;
            exportedContour.leadInError = contour.leadInSolution.error;
            if (contour.leadInSolution.valid && !hasLeadIn
                && exportedContour.leadInError.isEmpty()) {
                // 中文翻译：下刀点尚未重新计算五轴坐标
                exportedContour.leadInError = tr("The five-axis coordinates of the tool lowering point have not been recalculated.");
            }
            if (hasLeadIn) {
                const ToolpathPoint& lead = contour.leadInSolution.point;
                exportedContour.leadInPoint.x = lead.position.X();
                exportedContour.leadInPoint.y = lead.position.Y();
                exportedContour.leadInPoint.z = lead.position.Z();
                exportedContour.leadInPoint.normalX = lead.normal.X();
                exportedContour.leadInPoint.normalY = lead.normal.Y();
                exportedContour.leadInPoint.normalZ = lead.normal.Z();
                exportedContour.leadInPoint.tangentX = lead.tangent.X();
                exportedContour.leadInPoint.tangentY = lead.tangent.Y();
                exportedContour.leadInPoint.tangentZ = lead.tangent.Z();
                exportedContour.leadInPoint.machineX = lead.machineCoord.x;
                exportedContour.leadInPoint.machineY = lead.machineCoord.y;
                exportedContour.leadInPoint.machineZ = lead.machineCoord.z;
                exportedContour.leadInPoint.machineR1 = lead.machineCoord.r1;
                exportedContour.leadInPoint.machineR2 = lead.machineCoord.r2;
                exportedContour.leadInPoint.rotaryAxis1Name = lead.machineCoord.r1Name;
                exportedContour.leadInPoint.rotaryAxis2Name = lead.machineCoord.r2Name;
                exportedContour.leadInPoint.machineCoordValid = lead.machineCoord.valid;
                exportedContour.leadInPoint.machineAxes = lead.machineCoord.solvedPose.values;
                exportedContour.leadInPoint.machineAxisMask = lead.machineCoord.solvedPose.activeMask;
                exportedContour.leadInPoint.machineFailureReason = lead.machineCoord.solvedPose.failureReason;
            }
            exportedContour.endpointsValid = true;
        }

        snapshot.contours.append(exportedContour);

        QVector<lcnc::cam::ToolpathExportPoint> points;
        points.reserve(static_cast<int>(contour.points.size()));
        for (const ToolpathPoint& point : contour.points) {
            lcnc::cam::ToolpathExportPoint exportedPoint;
            exportedPoint.x = point.position.X();
            exportedPoint.y = point.position.Y();
            exportedPoint.z = point.position.Z();
            exportedPoint.normalX = point.normal.X();
            exportedPoint.normalY = point.normal.Y();
            exportedPoint.normalZ = point.normal.Z();
            exportedPoint.tangentX = point.tangent.X();
            exportedPoint.tangentY = point.tangent.Y();
            exportedPoint.tangentZ = point.tangent.Z();
            exportedPoint.curveParam = point.param;
            exportedPoint.machineX = point.machineCoord.x;
            exportedPoint.machineY = point.machineCoord.y;
            exportedPoint.machineZ = point.machineCoord.z;
            exportedPoint.machineR1 = point.machineCoord.r1;
            exportedPoint.machineR2 = point.machineCoord.r2;
            exportedPoint.rotaryAxis1Name = point.machineCoord.r1Name;
            exportedPoint.rotaryAxis2Name = point.machineCoord.r2Name;
            exportedPoint.machineCoordValid = point.machineCoord.valid;
            exportedPoint.machineAxes = point.machineCoord.solvedPose.values;
            exportedPoint.machineAxisMask = point.machineCoord.solvedPose.activeMask;
            exportedPoint.machineFailureReason = point.machineCoord.solvedPose.failureReason;
            points.append(exportedPoint);
        }
        snapshot.pointsByContourId.insert(contour.contourId, points);
    }

    return snapshot;
}

void CamModule::setLeadInLength(double mm)
{
    if (mm <= 0.0)
        return;
    toolpathRef().setGlobalLeadInLength(mm);
    // 参数只记录为待应用值；已持久化/显示的下刀点仅在“重新计算”时重建。
    LCNC_INFO(lcnc::LogCode::Generic,
              "cam.toolpath: lead-in length changed to {:.3f} mm; apply on explicit recalculation",
              mm);
    pushGenerationParamsToCamData();
    if (m_camData)
        m_camData->setGenerationParamsDirty(true);
    if (m_camData)
        m_camData->markDirty(true);
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Cam);
}

double CamModule::leadInLength() const
{
    return toolpathRef().globalLeadInLength();
}

void CamModule::setDeflection(double mm)
{
    if (mm <= 0.0)
        return;

    m_deflection = mm;
    pushGenerationParamsToCamData();
    if (m_camData) {
        m_camData->setGenerationParamsDirty(true);
        m_camData->markDirty(true);
    }
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Cam);
}

double CamModule::deflection() const
{
    return m_deflection;
}

bool CamModule::setCuttingOffset(double mm)
{
    if (!std::isfinite(mm) || toolpathRef().globalRapidOffsetMm() <= mm)
        return false;
    toolpathRef().setGlobalCuttingOffsetMm(mm);
    for (LaserContour& contour : toolpathRef().contours()) {
        contour.pendingParams.cuttingOffsetMm = mm;
        contour.dirtyStages = contour.dirtyStages | ContourDirtyStage::MotionOffset
            | ContourDirtyStage::MachineSolve | ContourDirtyStage::AdjacentRapid
            | ContourDirtyStage::Collision;
        contour.needsRecalculation = true;
    }
    pushGenerationParamsToCamData();
    if (m_camData) {
        m_camData->setGenerationParamsDirty(true);
        m_camData->markDirty(true);
    }
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Cam);
    return true;
}

double CamModule::cuttingOffset() const
{
    return toolpathRef().globalCuttingOffsetMm();
}

bool CamModule::setRapidOffset(double mm)
{
    if (!std::isfinite(mm) || mm < 0.0 || mm <= toolpathRef().globalCuttingOffsetMm())
        return false;
    toolpathRef().setGlobalRapidOffsetMm(mm);
    for (LaserContour& contour : toolpathRef().contours()) {
        contour.pendingParams.rapidOffsetMm = mm;
        contour.dirtyStages = contour.dirtyStages | ContourDirtyStage::AdjacentRapid
            | ContourDirtyStage::MachineSolve | ContourDirtyStage::Collision;
        contour.needsRecalculation = true;
    }
    pushGenerationParamsToCamData();
    if (m_camData) {
        m_camData->setGenerationParamsDirty(true);
        m_camData->markDirty(true);
    }
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Cam);
    return true;
}

double CamModule::rapidOffset() const
{
    return toolpathRef().globalRapidOffsetMm();
}

bool CamModule::showNormals() const
{
    return m_toolpathRenderer->showNormals();
}

void CamModule::setShowNormals(bool on)
{
    if (m_toolpathRenderer->showNormals() == on)
        return;
    m_toolpathRenderer->setShowNormals(on);
    m_config.setShowNormals(on);
    m_toolpathRenderer->refreshNormals(activeGuiDocument(), toolpathRef(), kinematics());
}

double CamModule::normalSampleStep() const
{
    return m_toolpathRenderer->normalSampleStep();
}

void CamModule::setNormalSampleStep(double mm)
{
    const double current = m_toolpathRenderer->normalSampleStep();
    if (mm <= 0.0)
        return;
    if (qFuzzyCompare(current + 1.0, mm + 1.0))
        return;
    m_toolpathRenderer->setNormalSampleStep(mm);
    m_config.setNormalSampleStep(mm);
    if (m_toolpathRenderer->showNormals())
        m_toolpathRenderer->refreshNormals(activeGuiDocument(), toolpathRef(), kinematics());
}

bool CamModule::resolveLeadInHit(WidgetOccView* occView,
                                 const QPoint& screenPos,
                                 int& contourIdx,
                                 int& pointIdx,
                                 gp_Pnt& entryPoint,
                                 double& entryParam) const
{
    return lcnc::cam::reference_pick::resolveLeadInHit(occView, screenPos, toolpathRef(),
                                                       contourIdx, pointIdx, entryPoint, entryParam);
}

void CamModule::setActiveContourId(lcnc::cam::ContourId contourId)
{
    if (contourId != 0 && contourIndexById(contourId) < 0)
        contourId = 0;
    if (m_activeContourId == contourId)
        return;
    m_activeContourId = contourId;
    emit activeToolpathContourChanged(m_activeContourId, activeContourIndex());
    emit activeContourParametersChanged();
}

void CamModule::clearToolpathSelectionState()
{
    setActiveContourId(0);
    m_lastCamSelectionContourIds.clear();
    if (auto selection = lcnc::Kernel::current().services()
            .getService<lcnc::core::SelectionService>()) {
        selection->clear();
    }
    if (auto* gd = activeGuiDocument(); gd && !gd->context().IsNull()) {
        gd->context()->ClearSelected(Standard_False);
        if (gd->hasView())
            gd->view()->Redraw();
    }
}

int CamModule::activeContourIndex() const
{
    return contourIndexById(m_activeContourId);
}

bool CamModule::setActiveContourLeadInLength(double mm)
{
    const int index = activeContourIndex();
    if (mm <= 0.0 || index < 0 || index >= toolpathRef().contourCount())
        return false;
    LaserContour& contour = toolpathRef().contour(index);
    contour.pendingParams.leadInLength = mm;
    contour.dirtyStages = contour.dirtyStages | ContourDirtyStage::LeadInGeometry
        | ContourDirtyStage::MachineSolve | ContourDirtyStage::AdjacentRapid
        | ContourDirtyStage::Collision;
    contour.needsRecalculation = true;
    m_camData->markDirty(true);
    emit activeContourParametersChanged();
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Cam);
    return true;
}

bool CamModule::setActiveContourDeflection(double mm)
{
    const int index = activeContourIndex();
    if (mm <= 0.0 || index < 0 || index >= toolpathRef().contourCount())
        return false;
    LaserContour& contour = toolpathRef().contour(index);
    contour.pendingParams.deflection = mm;
    contour.dirtyStages = ContourDirtyStage::All;
    contour.needsRecalculation = true;
    m_camData->markDirty(true);
    emit activeContourParametersChanged();
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Cam);
    return true;
}

bool CamModule::setActiveContourCuttingOffset(double mm)
{
    const int index = activeContourIndex();
    if (!std::isfinite(mm) || index < 0 || index >= toolpathRef().contourCount())
        return false;
    LaserContour& contour = toolpathRef().contour(index);
    if (contour.pendingParams.rapidOffsetMm <= mm)
        return false;
    contour.pendingParams.cuttingOffsetMm = mm;
    contour.dirtyStages = contour.dirtyStages | ContourDirtyStage::MotionOffset
        | ContourDirtyStage::MachineSolve | ContourDirtyStage::AdjacentRapid
        | ContourDirtyStage::Collision;
    contour.needsRecalculation = true;
    m_camData->markDirty(true);
    emit activeContourParametersChanged();
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Cam);
    return true;
}

bool CamModule::setActiveContourRapidOffset(double mm)
{
    const int index = activeContourIndex();
    if (!std::isfinite(mm) || mm < 0.0 || index < 0 || index >= toolpathRef().contourCount())
        return false;
    LaserContour& contour = toolpathRef().contour(index);
    if (mm <= contour.pendingParams.cuttingOffsetMm)
        return false;
    contour.pendingParams.rapidOffsetMm = mm;
    contour.dirtyStages = contour.dirtyStages | ContourDirtyStage::AdjacentRapid
        | ContourDirtyStage::MachineSolve | ContourDirtyStage::Collision;
    contour.needsRecalculation = true;
    m_camData->markDirty(true);
    emit activeContourParametersChanged();
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Cam);
    return true;
}

bool CamModule::updateLeadInPreview(WidgetOccView* occView, const QPoint& screenPos)
{
    int contourIdx = -1;
    int pointIdx = -1;
    gp_Pnt entryPoint;
    double entryParam = 0.0;

    if (!resolveLeadInHit(occView, screenPos, contourIdx, pointIdx, entryPoint, entryParam)) {
        if (m_previewLeadInValid)
            cancelLeadInPreview();
        return false;
    }

    if (m_previewLeadInValid
        && m_previewLeadInContour == contourIdx
        && m_previewLeadInPointIndex == pointIdx
        && m_previewLeadInPoint.Distance(entryPoint) < 1e-6
        && std::abs(m_previewLeadInParam - entryParam) < 1e-6) {
        return true;
    }

    m_previewLeadInContour = contourIdx;
    m_previewLeadInPointIndex = pointIdx;
    m_previewLeadInPoint = entryPoint;
    m_previewLeadInParam = entryParam;
    m_previewLeadInValid = true;
    lcnc::view::ToolpathRenderer::LeadInPreview preview{
        m_previewLeadInContour, m_previewLeadInPointIndex, m_previewLeadInPoint, m_previewLeadInParam, m_previewLeadInValid
    };
    m_toolpathRenderer->refreshLeadIns(activeGuiDocument(), toolpathRef(), kinematics(), preview);
    return true;
}

bool CamModule::commitLeadInPreview(WidgetOccView* occView, const QPoint& screenPos)
{
    if (!updateLeadInPreview(occView, screenPos) || !m_previewLeadInValid)
        return false;

    if (m_previewLeadInContour < 0 || m_previewLeadInContour >= toolpathRef().contourCount())
        return false;

    LaserContour& contour = toolpathRef().contour(m_previewLeadInContour);
    LaserContour updated = contour;
    QString startError;
    if (!LaserToolpathBuilder::setContourStart(
            updated, m_previewLeadInPointIndex, &startError)) {
        // 中文翻译：设置轮廓起点失败
        emit operationFailed(tr("Failed to set outline start point"), startError);
        return false;
    }
    if (!updated.leadInSolution.valid) {
        // 中文翻译：设置轮廓起点失败
        emit operationFailed(tr("Failed to set outline start point"), updated.leadInSolution.error);
        return false;
    }
    contour = std::move(updated);
    contour.needsRecalculation = true;
    if (contour.leadInSolution.valid)
        contour.leadInSolution.point.machineCoord = {};
    for (ToolpathPoint& point : contour.points)
        point.machineCoord = {};
    setActiveContourId(static_cast<lcnc::cam::ContourId>(contour.contourId));

    m_previewLeadInContour = -1;
    m_previewLeadInPointIndex = -1;
    m_previewLeadInParam = 0.0;
    m_previewLeadInValid = false;
    setActiveContourId(0);
    // 仅更新当前轮廓的下刀几何；五轴解算留给用户显式点击“重新计算”。
    m_toolpathRenderer->refreshLeadIns(activeGuiDocument(), toolpathRef(), kinematics());
    refreshCuttingOrderOverlays();
    if (m_camData)
        m_camData->markDirty(true);
    emit activeContourParametersChanged();
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Cam);
    return true;
}

void CamModule::cancelLeadInPreview()
{
    if (!m_previewLeadInValid)
        return;

    m_previewLeadInContour = -1;
    m_previewLeadInPointIndex = -1;
    m_previewLeadInParam = 0.0;
    m_previewLeadInValid = false;
    m_toolpathRenderer->refreshLeadIns(activeGuiDocument(), toolpathRef(), kinematics());
}

void CamModule::setContourEnabled(int contourIdx, bool enabled)
{
    if (contourIdx < 0 || contourIdx >= toolpathRef().contourCount())
        return;

    LaserContour& contour = toolpathRef().contour(contourIdx);
    if (contour.enabled == enabled)
        return;

    contour.enabled = enabled;
    setCamContourVisible(contourIdx, enabled && m_toolpathRenderer->isVisible(), false);
    lcnc::view::ToolpathRenderer::LeadInPreview preview{
        m_previewLeadInContour, m_previewLeadInPointIndex, m_previewLeadInPoint, m_previewLeadInParam, m_previewLeadInValid
    };
    m_toolpathRenderer->refreshContour(activeGuiDocument(), toolpathRef(), kinematics(), contourIdx, preview);
    emit toolpathLayersChanged();
}

void CamModule::setAllContoursEnabled(bool enabled)
{
    bool changed = false;
    for (int index = 0; index < toolpathRef().contourCount(); ++index) {
        LaserContour& contour = toolpathRef().contour(index);
        if (contour.enabled == enabled)
            continue;
        contour.enabled = enabled;
        changed = true;
    }

    if (!changed)
        return;

    setCamContoursVisible(m_toolpathRenderer->isVisible(), false);
    lcnc::view::ToolpathRenderer::LeadInPreview preview{
        m_previewLeadInContour, m_previewLeadInPointIndex, m_previewLeadInPoint, m_previewLeadInParam, m_previewLeadInValid
    };
    m_toolpathRenderer->refresh(activeGuiDocument(), toolpathRef(), kinematics(), preview);
    emit toolpathLayersChanged();
}

lcnc::cam::ContourId CamModule::contourIdAt(int contourIdx) const
{
    return m_camData ? m_camData->contourIdAt(contourIdx) : 0;
}

int CamModule::contourIndexById(lcnc::cam::ContourId contourId) const
{
    return m_camData ? m_camData->contourIndexById(contourId) : -1;
}

void CamModule::reorderContoursById(const QList<lcnc::cam::ContourId>& order)
{
    if (!m_camData || !m_camData->reorderContoursById(order))
        return;

    syncCamDocumentContours();
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Cam);
    if (m_toolpathRenderer->isVisible())
        refreshToolpathDisplay();
    emit toolpathLayersChanged();
}

void CamModule::reorderContours(const QList<int>& order)
{
    if (!m_camData || !m_camData->reorderContours(order))
        return;

    syncCamDocumentContours();
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Cam);
    if (m_toolpathRenderer->isVisible())
        refreshToolpathDisplay();
    emit toolpathLayersChanged();
}

const std::vector<ToolpathLayer>& CamModule::toolpathLayers() const
{
    static const std::vector<ToolpathLayer> empty;
    return m_camData ? m_camData->toolpathLayers() : empty;
}

lcnc::cam::ProjectExplorerSnapshot CamModule::projectExplorerSnapshot() const
{
    lcnc::cam::ProjectExplorerSnapshot snapshot;
    snapshot.documentId = camDocumentId();
    const LaserToolpath& path = toolpath();
    snapshot.contours.reserve(path.contourCount());
    for (int index = 0; index < path.contourCount(); ++index) {
        const LaserContour& contour = path.contour(index);
        snapshot.contours.append({static_cast<lcnc::cam::ContourId>(contour.contourId),
                                  contour.layerId,
                                  contour.name,
                                  contour.sourceInfo,
                                  static_cast<int>(contour.points.size()),
                                  index,
                                  contour.enabled});
    }
    for (const ToolpathLayer& layer : toolpathLayers()) {
        lcnc::cam::ProjectExplorerLayer projected;
        projected.layerId = layer.layerId;
        projected.name = layer.name;
        projected.toolName = layer.toolName;
        projected.color = layer.color;
        projected.enabled = layer.enabled;
        for (const std::uint64_t contourId : layer.contourIds)
            projected.contourIds.append(static_cast<lcnc::cam::ContourId>(contourId));
        snapshot.layers.append(std::move(projected));
    }
    for (const MachiningFaceInfo& face : machiningFacesForTree())
        snapshot.faces.append({face.faceId, face.displayName, face.manual, face.role});
    snapshot.facesVisible = machiningFacesVisible();
    for (int index = 0; index < static_cast<int>(lcnc::cam::CamPipelineStage::Count); ++index) {
        snapshot.stages[index] = pipelineStageState(
            static_cast<lcnc::cam::CamPipelineStage>(index));
    }
    return snapshot;
}

std::uint64_t CamModule::addToolpathLayer(const QString& name, const QColor& color)
{
    if (!m_camData)
        return 0;
    const std::uint64_t id = m_camData->addLayer(name, color);
    emit toolpathLayersChanged();
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Cam);
    return id;
}

bool CamModule::removeToolpathLayerWithContours(std::uint64_t layerId)
{
    if (!m_camData)
        return false;

    // 记录被删轮廓的 contourId，用于清理选择/预览状态。
    std::vector<std::uint64_t> removedContourIds;
    if (const ToolpathLayer* layer = m_camData->toolpathLayer(layerId))
        removedContourIds = layer->contourIds;

    if (!m_camData->removeLayerWithContours(layerId))
        return false;

    // 重建 XCAF Cam 实体（丢弃被删轮廓的 wire）并同步 AIS 显示。
    writeContourGeometryToDocument();
    syncCamDocumentContours(false); // 擦除已删轮廓的线体 AIS，重显存活轮廓（含颜色/可见性）
    if (m_toolpathRenderer->isVisible())
        refreshToolpathDisplay();   // 刷新刀路折线/法向（clearAis+rebuild 会丢弃已删轮廓）

    // 清理被删轮廓的残留选择状态。
    auto selSvc = lcnc::Kernel::current()
                      .services()
                      .getService<lcnc::core::SelectionService>();
    if (selSvc) {
        for (std::uint64_t cid : removedContourIds)
            selSvc->removeContour(cid);
    }
    for (std::uint64_t cid : removedContourIds)
        m_lastCamSelectionContourIds.remove(cid);
    if (m_activeContourId != 0) {
        for (std::uint64_t cid : removedContourIds) {
            if (cid == m_activeContourId) {
                setActiveContourId(0);
                break;
            }
        }
    }
    // 轮廓索引随删除发生平移，重置引线预览避免指向错误轮廓。
    m_previewLeadInContour = -1;
    m_previewLeadInValid = false;

    emit toolpathLayersChanged();
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Cam);
    return true;
}

bool CamModule::assignContoursToLayer(const QList<lcnc::cam::ContourId>& contourIds,
                                      std::uint64_t layerId)
{
    if (!m_camData || !m_camData->assignContoursToLayer(contourIds, layerId))
        return false;
    applyToolpathLayerColors();
    if (m_toolpathRenderer->isVisible())
        refreshToolpathDisplay();
    emit toolpathLayersChanged();
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Cam);
    return true;
}

QList<int> CamModule::contourIndexesInLayer(std::uint64_t layerId) const
{
    return m_camData ? m_camData->contourIndexesInLayer(layerId) : QList<int>{};
}

bool CamModule::updateToolpathLayer(std::uint64_t layerId,
                                    const QString& name,
                                    const QColor& color,
                                    const QString& toolName)
{
    if (!m_camData || !m_camData->updateToolpathLayer(layerId, name, color, toolName))
        return false;

    applyToolpathLayerColors();
    emit toolpathLayersChanged();
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Cam);
    return true;
}

bool CamModule::updateToolpathLayer(std::uint64_t layerId,
                                    const QString& name,
                                    const QColor& color)
{
    // 不动 toolName 字段：透传当前值（兼容老项目读写）。
    const ToolpathLayer* layer = nullptr;
    if (m_camData) {
        layer = m_camData->toolpathLayer(layerId);
    }
    const QString preservedToolName = layer ? layer->toolName : QString();
    return updateToolpathLayer(layerId, name, color, preservedToolName);
}

bool CamModule::setToolpathLayerEnabled(std::uint64_t layerId, bool enabled)
{
    if (!m_camData || !m_camData->setToolpathLayerEnabled(layerId, enabled))
        return false;

    setCamContoursVisible(m_toolpathRenderer->isVisible(), false);
    lcnc::view::ToolpathRenderer::LeadInPreview preview{
        m_previewLeadInContour, m_previewLeadInPointIndex, m_previewLeadInPoint, m_previewLeadInParam, m_previewLeadInValid
    };
    m_toolpathRenderer->refresh(activeGuiDocument(), toolpathRef(), kinematics(), preview);
    emit toolpathLayersChanged();
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Cam);
    return true;
}

TaskId CamModule::recalcToolpathAsync()
{
    const int contourIndex = activeContourIndex();
    if (contourIndex < 0 || contourIndex >= toolpathRef().contourCount()) {
        // 中文翻译：重新计算当前轮廓；请先在项目树中选择一条轮廓
        emit operationFailed(tr("Recalculate the current contour"), tr("Please select a profile in the project tree first"));
        return kInvalidTaskId;
    }
    auto* taskManager = lcnc::Kernel::current().taskManager();
    MachineKinematics* machine = kinematics();
    if (!taskManager || !machine) {
        // 中文翻译：重新计算当前轮廓；后台任务或机台运动学服务不可用
        emit operationFailed(tr("Recalculate the current contour"), tr("Background tasks or machine kinematics services are not available"));
        return kInvalidTaskId;
    }

    const LaserContour current = toolpathRef().contour(contourIndex);
    const TopoDS_Shape sourceShape = current.sourceShape.IsNull()
        ? m_workpieceShape : current.sourceShape;
    if (sourceShape.IsNull()) {
        // 中文翻译：重新计算当前轮廓；当前轮廓缺少工件几何
        emit operationFailed(tr("Recalculate the current contour"), tr("The current profile is missing workpiece geometry"));
        return kInvalidTaskId;
    }

    QVector<lcnc::cam::ContourId> order = contourSequenceSnapshot().orderedContourIds;

    MachineCoord continuity;
    const auto currentId = static_cast<lcnc::cam::ContourId>(current.contourId);
    const int orderIndex = order.indexOf(currentId);
    for (int i = orderIndex - 1; i >= 0; --i) {
        const int previousIndex = contourIndexById(order.at(i));
        if (previousIndex < 0)
            continue;
        const LaserContour& previous = toolpathRef().contour(previousIndex);
        if (!previous.enabled || previous.points.empty())
            continue;
        if (previous.points.back().machineCoord.valid) {
            continuity = previous.points.back().machineCoord;
            break;
        }
    }

    struct RecalcResult {
        LaserContour contour;
        QString error;
        bool ok{false};
    };
    const auto result = std::make_shared<RecalcResult>();
    const auto appliedGlobal = m_camData->appliedGenerationParams();
    const QString configType = machine->configType();
    const lcnc::MachineModeDefinition modeDefinition =
        m_machineConfig->modeDefinition(m_camData->machiningMode());
    const QList<MachineAxisDef> axes =
        lcnc::cam_algo::offlinePlanningAxisBaseline(machine->axes(), modeDefinition);
    const lcnc::WorkpieceSetupTransform workpieceSetup = m_machineConfig->workpieceSetupTransform();
    const lcnc::HeadToolGeometry headToolGeometry = m_machineConfig->headToolGeometry();
    // Recalculation works from an immutable contour snapshot.  Do not apply its
    // result if any project-level toolpath, face-pipeline, or machine setup
    // input changed while the worker was running.
    const std::uint64_t capturedToolpathRevision = toolpathRevision();
    const std::uint64_t capturedFaceRevision = machiningFaceSetRevision();
    const std::uint64_t capturedSetupRevision = machineSetupRevision();
    const std::uint64_t originalSignature = current.signature;
    const ContourGenerationParams originalPendingParams = current.pendingParams;
    const lcnc::cam::ContourId targetId = currentId;

    TaskSpec spec;
    // 中文翻译：重新计算当前轮廓
    spec.label = tr("Recalculate the current contour");
    spec.scope = QStringLiteral("cam.toolpath");
    spec.priority = TaskPriority::Normal;
    spec.cancellable = true;
    const TaskId taskId = taskManager->run(spec,
        [current, sourceShape, appliedGlobal, continuity, axes, configType, modeDefinition,
         workpieceSetup, headToolGeometry, result](TaskProgress* progress) {
            progress->setRange(0, 100);
            // 中文翻译：正在离散轮廓
            progress->setStepName(QObject::tr("discretizing contours"));
            LaserContour updated = current;
            const bool rebuildBaseGeometry = hasDirtyStage(
                updated.dirtyStages, ContourDirtyStage::Discretization)
                || hasDirtyStage(updated.dirtyStages, ContourDirtyStage::LeadInGeometry);
            if (!rebuildBaseGeometry) {
                // Height-only edits reuse immutable base samples and normals.
                // The committed rapid-plan rebuild below derives offset
                // geometry and solves the continuous motion sequence.
                updated.appliedParams = updated.pendingParams;
                updated.dirtyStages = ContourDirtyStage::None;
                updated.needsRecalculation = false;
                result->contour = std::move(updated);
                result->ok = true;
                progress->setValue(100);
                return;
            }
            if (appliedGlobal.useFaceClassification) {
                FaceClassification classification =
                    FaceClassifier::classifyFaces(sourceShape, appliedGlobal.smoothAngle);
                std::vector<TopoDS_Face> outerFaces;
                std::vector<TopoDS_Face> crossFaces;
                if (classification.outerGroup())
                    outerFaces = classification.outerGroup()->faces;
                for (const auto* group : classification.crossSectionGroups())
                    if (group)
                        crossFaces.insert(crossFaces.end(), group->faces.begin(), group->faces.end());
                if (!outerFaces.empty() && !crossFaces.empty()) {
                    LaserToolpathBuilder::bindLeadInSurfaceContext(
                        updated, outerFaces, crossFaces);
                    LaserToolpathBuilder::discretizeContourWithClassification(
                        updated, outerFaces, crossFaces, updated.pendingParams.deflection);
                } else {
                    LaserToolpathBuilder::discretizeContour(updated, sourceShape, updated.pendingParams.deflection);
                }
            } else {
                LaserToolpathBuilder::discretizeContour(updated, sourceShape, updated.pendingParams.deflection);
            }
            if (progress->isAbortRequested())
                // 中文翻译：轮廓重新计算已取消
                throw std::runtime_error("Contour recalculation canceled");
            if (updated.points.empty()) {
                // 中文翻译：轮廓 "%1" 离散后没有可用点
                result->error = QObject::tr("Contour \"%1\" has no available points after discretization").arg(updated.name);
                return;
            }
            auto selected = std::find_if(updated.points.begin(), updated.points.end(),
                [&updated](const ToolpathPoint& point) {
                    return point.sourceEdgeIndex == updated.leadIn.entryEdgeIndex
                        && std::abs(point.param - updated.leadIn.entryParam) <= 1e-10;
                });
            if (selected == updated.points.end() && updated.leadIn.entryEdgeIndex < 0) {
                selected = std::min_element(updated.points.begin(), updated.points.end(),
                    [&updated](const ToolpathPoint& a, const ToolpathPoint& b) {
                        return a.position.SquareDistance(updated.leadIn.entryPoint)
                            < b.position.SquareDistance(updated.leadIn.entryPoint);
                    });
            }
            if (selected == updated.points.end()) {
                // 中文翻译：轮廓 "%1" 无法恢复人工起点
                result->error = QObject::tr("Contour \"%1\" cannot restore artificial starting point").arg(updated.name);
                return;
            }
            if (current.leadIn.entryEdgeIndex >= 0
                && selected->position.Distance(current.leadIn.entryPoint) > 1e-6) {
                // 中文翻译：轮廓 "%1" 的起点拓扑锚点已变化
                result->error = QObject::tr("The starting topology anchor point of contour \"%1\" has changed").arg(updated.name);
                return;
            }
            updated.leadIn.length = updated.pendingParams.leadInLength;
            QString startError;
            if (!LaserToolpathBuilder::setContourStart(
                    updated, static_cast<int>(std::distance(updated.points.begin(), selected)), &startError)
                || !updated.leadInSolution.valid) {
                result->error = startError.isEmpty() ? updated.leadInSolution.error : startError;
                return;
            }
            progress->setValue(65);
            // 中文翻译：正在求解机台坐标
            progress->setStepName(QObject::tr("Solving for machine coordinates"));
            MachineKinematics workerKinematics;
            workerKinematics.setAxes(axes, configType);
            std::vector<LaserContour*> singleContour{&updated};
            if (!LaserToolpathBuilder::solveToolpathForOrder(
                    singleContour, &workerKinematics, gp_Trsf(), modeDefinition,
                    workpieceSetup, headToolGeometry, &result->error,
                    continuity.valid ? &continuity.solvedPose : nullptr)) {
                return;
            }
            const bool coordinatesValid = updated.leadInSolution.point.machineCoord.valid
                && std::all_of(updated.points.begin(), updated.points.end(), [](const ToolpathPoint& point) {
                    return point.machineCoord.valid;
                });
            if (!coordinatesValid) {
                // 中文翻译：轮廓 "%1" 五轴坐标求解失败
                result->error = QObject::tr("Contour \"%1\" five-axis coordinate solution failed").arg(updated.name);
                return;
            }
            updated.appliedParams = updated.pendingParams;
            updated.leadIn.length = updated.appliedParams.leadInLength;
            updated.needsRecalculation = false;
            updated.dirtyStages = ContourDirtyStage::None;
            result->contour = std::move(updated);
            result->ok = true;
            progress->setValue(100);
        });

    watchTask(this, taskId,
        [this, result, targetId, capturedToolpathRevision, capturedFaceRevision,
         capturedSetupRevision, originalSignature, originalPendingParams, sourceShape](bool success) {
        const int latestIndex = contourIndexById(targetId);
        if (!success || !result->ok) {
            // 中文翻译：重新计算当前轮廓
            emit operationFailed(tr("Recalculate the current contour"),
                                 // 中文翻译：轮廓重新计算失败或已取消
                                 result->error.isEmpty() ? tr("Contour recalculation failed or was canceled") : result->error);
            return;
        }
        const TopoDS_Shape latestSourceShape = latestIndex < 0
            ? TopoDS_Shape{}
            : (toolpathRef().contour(latestIndex).sourceShape.IsNull()
                ? m_workpieceShape : toolpathRef().contour(latestIndex).sourceShape);
        if (latestIndex < 0 || latestSourceShape.IsNull()
            || toolpathRevision() != capturedToolpathRevision
            || machiningFaceSetRevision() != capturedFaceRevision
            || machineSetupRevision() != capturedSetupRevision
            || toolpathRef().contour(latestIndex).signature != originalSignature
            || !latestSourceShape.IsSame(sourceShape)
            || std::abs(toolpathRef().contour(latestIndex).pendingParams.leadInLength
                        - originalPendingParams.leadInLength) > 1e-12
            || std::abs(toolpathRef().contour(latestIndex).pendingParams.deflection
                        - originalPendingParams.deflection) > 1e-12
            || std::abs(toolpathRef().contour(latestIndex).pendingParams.cuttingOffsetMm
                        - originalPendingParams.cuttingOffsetMm) > 1e-12
            || std::abs(toolpathRef().contour(latestIndex).pendingParams.rapidOffsetMm
                        - originalPendingParams.rapidOffsetMm) > 1e-12) {
            // 中文翻译：重新计算当前轮廓；轮廓在计算期间已变更，后台结果已丢弃
            emit operationFailed(tr("Recalculate the current contour"), tr("The contour changed during calculation and the background results were discarded"));
            return;
        }
        toolpathRef().contour(latestIndex) = std::move(result->contour);
        QString rapidError;
        rebuildTravelPlanForCurrentOrder(&rapidError);
        syncCamDocumentContours(/*forceRebuild=*/true);
        m_camData->markDirty(true);
        lcnc::view::ToolpathRenderer::LeadInPreview preview{
            m_previewLeadInContour, m_previewLeadInPointIndex, m_previewLeadInPoint,
            m_previewLeadInParam, m_previewLeadInValid
        };
        m_toolpathRenderer->refreshContour(
            lcnc::Kernel::current().guiApp()->activeGuiDocument(),
            toolpathRef(), kinematics(), latestIndex, preview);
        refreshCuttingOrderOverlays();
        emit activeContourParametersChanged();
        lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Cam);
    });
    return taskId;
}

void CamModule::setToolpathVisible(bool visible)
{
    if (m_toolpathRenderer->isVisible() == visible) {
        setCamContoursVisible(visible);
        return;
    }
    m_toolpathRenderer->setVisible(activeGuiDocument(), visible);
    setCamContoursVisible(visible);
    if (visible)
        refreshToolpathDisplay();
    emit toolpathVisibilityChanged(visible);
}

bool CamModule::isToolpathVisible() const
{
    return m_toolpathRenderer->isVisible();
}

double CamModule::smoothAngle() const
{
    return m_smoothAngle;
}

void CamModule::setSmoothAngle(double deg)
{
    if (qFuzzyCompare(m_smoothAngle + 1.0, deg + 1.0))
        return;
    m_smoothAngle = deg;
    m_config.setSmoothAngle(deg);
    pushGenerationParamsToCamData();
    if (m_camData) {
        m_camData->setGenerationParamsDirty(true);
        m_camData->markDirty(true);
    }
}
bool CamModule::useFaceClassification() const
{
    return m_useFaceClassification;
}

void CamModule::setUseFaceClassification(bool on)
{
    if (m_useFaceClassification == on)
        return;
    m_useFaceClassification = on;
    m_config.setUseFaceClassification(on);
    pushGenerationParamsToCamData();
    if (m_camData) {
        m_camData->setGenerationParamsDirty(true);
        m_camData->markDirty(true);
    }
}

gp_Dir CamModule::beamDirectionWpc(const QString& wpcEntry) const
{
    const MachineKinematics* kin = kinematics();
    if (!kin)
        return gp_Dir(0.0, 0.0, -1.0);
    // wpcHome maps workpiece -> machine at the home posture; the inverse maps
    // the machine-space beam direction into workpiece coordinates. Directions
    // are translation-invariant, so only the rotation matters.
    const gp_Trsf wpcHome = kin->computeWpcTransformHome(wpcEntry);
    return kin->nominalBeamDirectionMachine().Transformed(wpcHome.Inverted());
}

int CamModule::extractionStrategy() const
{
    return m_extractionStrategy;
}

void CamModule::setExtractionStrategy(int strategy)
{
    strategy = static_cast<int>(extractionStrategyFromPersistedValue(strategy));
    if (m_extractionStrategy == strategy)
        return;
    m_extractionStrategy = strategy;
    m_config.setExtractionStrategy(strategy);  // persist to cam.toml
    if (m_camData) {
        m_camData->setGenerationParamsDirty(true);
        m_camData->markDirty(true);
    }
}
