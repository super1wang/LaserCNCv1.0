// Verification for the strategy-based contour extraction:
//   - PlanarFaceWires on a perforated plate yields N+1 contours (1 outer + N
//     holes), all holes classified InnerHole, ordered holes-first / outer-last.
//   - The largest-smooth-connected-surface default produces the same N+1
//     result without relying on a machine-specific posture.
//   - The same default on a cylinder yields its smooth outer-surface boundary.

#include "core/algorithms/cam/laser_toolpath.h"
#include "core/algorithms/cad/primitives.h"
#include "core/algorithms/cam/face_classifier.h"
#include "core/project/cam/cam_data_manager.h"
#include "modules/cam/pipeline/machining_face_pipeline_service.h"
#include "modules/cam/toolpath/toolpath_generation_service.h"
#include "modules/cam/toolpath/toolpath_solve_service.h"
#include "modules/cam/toolpath/toolpath_sequence_service.h"
#include "core/kinematics/machine_kinematics.h"
#include "core/kernel/kernel.h"
#include "core/project/lcnc_project_manager.h"
#include "modules/cam/cam_module.h"
#include "view/gui_application.h"

#include <QCoreApplication>
#include <QEventLoop>
#include <QTimer>
#include <QTextStream>

#include <BRepAlgoAPI_Cut.hxx>
#include <BRep_Builder.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Solid.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <algorithm>
#include <cmath>
#include <vector>

// Exercise the actual owner capture/publication boundaries without starting a
// controller, showing a window or bypassing the production solve service.
struct CamMotionCompilationTestAccess
{
    static QString verify()
    {
        lcnc::Kernel kernel;
        kernel.registerCoreServices();
        kernel.service<lcnc::MachineConfigurationService>()->applyPreset(QStringLiteral("XYZ"));
        kernel.projectManager()->ensureProject();
        GuiApplication gui;
        kernel.setGuiApp(&gui);
        CamModule cam;
        cam.toolpathRef().setGlobalCuttingOffsetMm(0.0);
        LaserContour contour;
        contour.contourId = 42;
        contour.geometrySamplingComplete = true;
        contour.geometrySamplingEvidence = {true, true, true, {}};
        contour.needsRecalculation = false;
        contour.leadInSolution.valid = true;
        contour.leadInSolution.point.position = gp_Pnt(9, 20, 30);
        for (int index = 0; index < 2; ++index) {
            ToolpathPoint point;
            point.position = gp_Pnt(10 + index, 20, 30);
            point.sourceEdgeIndex = 0;
            point.param = index;
            contour.points.push_back(point);
        }
        cam.toolpathRef().contours() = {contour};
        cam.m_camData->ensureToolpathLayers();
        const auto id = cam.toolpathRef().contours().front().contourId;
        const auto captured = cam.captureMotionCompilationInput();
        const auto oldDeflection = cam.m_deflection;
        cam.m_deflection = oldDeflection * 0.5;
        const auto current = cam.captureMotionCompilationInput();
        if (lcnc::cam::ToolpathGenerationService::sameMotionAuthority(*captured, *current)
            || cam.solveToolpathForOrder({id}, captured))
            return QStringLiteral("Owner accepted result after sampling authority changed");
        cam.m_deflection = oldDeflection;
        if (!cam.solveToolpathForOrder({id}, captured))
            return QStringLiteral("Owner rejected unchanged frozen computation input");
        auto snapshot = cam.exportToolpathBaseSnapshot();
        cam.attachMotionPlan(snapshot);
        if (snapshot.motionPlan.contextHash != captured->capturedContextHash
            || snapshot.motionPlan.planHash.isEmpty())
            return QStringLiteral("Published plan did not retain pre-computation context: ")
                + snapshot.motionPlan.failureReason;
        const auto differentOrder = cam.exportToolpathSnapshotForOrder({id, id});
        if (!differentOrder.motionPlan.planHash.isEmpty() || differentOrder.motionPlan.failureReason.isEmpty())
            return QStringLiteral("Owner reused solved coordinates for a different requested contour order");
        // Dispatch the real asynchronous worker, then change policy before
        // the owner event loop can adopt its result.
        using lcnc::cam::CamPipelineStage;
        std::uint64_t upstream = 0;
        for (const auto stage : {CamPipelineStage::FaceSeparation, CamPipelineStage::ContourExtraction,
             CamPipelineStage::PointDiscretization, CamPipelineStage::GeometricToolpath}) {
            cam.m_camData->commitPipelineStage(stage, upstream);
            upstream = cam.m_camData->pipelineStageState(stage).revision;
        }
        QEventLoop loop;
        QString asynchronousError;
        const auto failedConnection = QObject::connect(&cam, &CamModule::operationFailed, &loop,
            [&](const QString&, const QString& reason) { asynchronousError = reason; loop.quit(); });
        const auto successConnection = QObject::connect(&cam, &CamModule::toolpathGenerated, &loop, &QEventLoop::quit);
        if (cam.solveCurrentGeometricToolpathAsync() == kInvalidTaskId)
            return QStringLiteral("Owner fixture could not dispatch the actual solve worker: ") + asynchronousError;
        cam.m_deflection = oldDeflection * 0.5;
        QTimer::singleShot(10000, &loop, &QEventLoop::quit);
        loop.exec();
        QObject::disconnect(failedConnection);
        QObject::disconnect(successConnection);
        if (!asynchronousError.contains(QStringLiteral("authority changed")))
            return QStringLiteral("Actual worker adopted changed authority or failed for another reason: ") + asynchronousError;
        cam.m_deflection = oldDeflection * 0.5;
        cam.attachMotionPlan(snapshot);
        if (!snapshot.motionPlan.planHash.isEmpty() || snapshot.motionPlan.failureReason.isEmpty())
            return QStringLiteral("Owner publication recaptured changed policy as a fresh plan");
        cam.m_deflection = oldDeflection;
        auto& edited = cam.toolpathRef().contours().front();
        auto samples = edited.points;
        samples.front().position.SetX(100.0);
        LaserToolpathBuilder::replaceGeometrySamples(edited, std::move(samples), true);
        const auto rejected = cam.exportToolpathSnapshotForOrder({id});
        if (!rejected.motionPlan.planHash.isEmpty() || rejected.motionPlan.failureReason.isEmpty())
            return QStringLiteral("Actual export published geometry with invalidated solved poses");
        return {};
    }
};

namespace {

int fail(const QString& message)
{
    QTextStream(stderr) << message << '\n';
    return 1;
}

/// A 100x100x10 plate (z: 0..10) with N through-holes drilled along +Z.
TopoDS_Shape makePerforatedPlate(int holeCount)
{
    const TopoDS_Solid box = BRepPrimAPI_MakeBox(
        gp_Pnt(0.0, 0.0, 0.0), gp_Pnt(100.0, 100.0, 10.0)).Solid();

    TopoDS_Shape result = box;
    for (int i = 0; i < holeCount; ++i) {
        const double cx = 20.0 + 30.0 * i;
        const double cy = 50.0;
        const TopoDS_Shape cyl = BRepPrimAPI_MakeCylinder(
            gp_Ax2(gp_Pnt(cx, cy, -5.0), gp_Dir(0.0, 0.0, 1.0)), 5.0, 20.0).Shape();
        result = BRepAlgoAPI_Cut(result, cyl).Shape();
    }
    return result;
}

int countByKind(const std::vector<LaserContour>& contours, ContourKind kind)
{
    int n = 0;
    for (const auto& c : contours)
        if (static_cast<ContourKind>(c.contourType) == kind)
            ++n;
    return n;
}

/// Assert a perforated plate (holeCount holes) extracts to holeCount+1 contours:
/// 1 outer boundary, holeCount inner holes, holes ordered before the outer ring.
int verifyPerforatedPlate(const TopoDS_Shape& plate, int holeCount,
                          ExtractionStrategy strategy, const QString& label,
                          const gp_Dir& reportedBeam = gp_Dir(0.0, 0.0, -1.0))
{
    ContourExtractionParams params;
    params.smoothAngleThresholdDeg = 5.0;
    params.deflection = 0.1;
    params.strategy = strategy;
    params.machiningBeamDirection = reportedBeam;

    FaceClassification classification;
    auto contours = LaserToolpathBuilder::extractContours(plate, params, &classification);

    const int expected = holeCount + 1;
    if (static_cast<int>(contours.size()) != expected)
        return fail(label + QStringLiteral(": expected %1 contours, got %2")
                        .arg(expected).arg(contours.size()));
    if (countByKind(contours, ContourKind::OuterBoundary) != 1)
        return fail(label + QStringLiteral(": expected exactly 1 outer boundary"));
    if (countByKind(contours, ContourKind::InnerHole) != holeCount)
        return fail(label + QStringLiteral(": expected %1 inner holes, got %2")
                        .arg(holeCount).arg(countByKind(contours, ContourKind::InnerHole)));

    // Holes first, outer last: every contour before the last must be a hole,
    // and the last must be the outer boundary.
    for (std::size_t i = 0; i + 1 < contours.size(); ++i)
        if (static_cast<ContourKind>(contours[i].contourType) != ContourKind::InnerHole)
            return fail(label + QStringLiteral(": non-hole contour before the outer ring"));
    if (static_cast<ContourKind>(contours.back().contourType) != ContourKind::OuterBoundary)
        return fail(label + QStringLiteral(": last contour is not the outer boundary"));

    return 0;
}

int verifyLargestSmoothSurfaceRegression(const TopoDS_Shape& cylinder, const QString& label)
{
    ContourExtractionParams params;
    params.smoothAngleThresholdDeg = 5.0;
    params.deflection = 0.1;
    params.strategy = ExtractionStrategy::LargestSmoothConnectedSurface;

    FaceClassification classification;
    auto contours = LaserToolpathBuilder::extractContours(cylinder, params, &classification);
    if (contours.empty())
        return fail(label + QStringLiteral(": largest smooth surface produced no contours"));
    if (!classification.hasOuter() || !classification.hasCrossSection())
        return fail(label + QStringLiteral(": smooth-surface classification incomplete"));
    for (const auto& c : contours)
        if (static_cast<ContourKind>(c.contourType) != ContourKind::TubeCrossSection)
        return fail(label + QStringLiteral(": smooth-surface contour mislabeled"));
    for (const auto& c : contours) {
        if (c.points.empty())
            return fail(label + QStringLiteral(": contour points were not discretized"));
        if (!c.points.front().crossSectionNormalValid)
            return fail(label + QStringLiteral(": adjacent section normal was not retained"));
    }

    const auto* outer = classification.outerGroup();
    if (!outer || outer->faces.empty())
        return fail(label + QStringLiteral(": missing outer face group for manual regression"));
    auto manualContours = LaserToolpathBuilder::extractContoursFromFaces(
        cylinder, outer->faces, gp_Dir(0.0, 0.0, -1.0), params);
    if (manualContours.size() != contours.size())
        return fail(label + QStringLiteral(": manual face-group boundary differs from tube extraction"));
    for (const auto& c : manualContours)
        if (static_cast<ContourKind>(c.contourType) != ContourKind::TubeCrossSection)
            return fail(label + QStringLiteral(": manual tube boundary mislabeled"));
    return 0;
}

int verifyTopVisibleFaceSelection()
{
    // The upper box completely shadows the lower one in XY projection.  No
    // face from the lower box may survive the +Z parallel-light selection.
    const TopoDS_Solid lower = BRepPrimAPI_MakeBox(
        gp_Pnt(0.0, 0.0, 0.0), gp_Pnt(100.0, 100.0, 5.0)).Solid();
    const TopoDS_Solid upper = BRepPrimAPI_MakeBox(
        gp_Pnt(0.0, 0.0, 10.0), gp_Pnt(100.0, 100.0, 15.0)).Solid();
    TopoDS_Compound stacked;
    BRep_Builder builder;
    builder.MakeCompound(stacked);
    builder.Add(stacked, lower);
    builder.Add(stacked, upper);

    const auto visibleFaces = LaserToolpathBuilder::selectTopVisibleFacesFromPositiveZ(stacked);
    if (visibleFaces.empty())
        return fail(QStringLiteral("top-visible selection found no upper face"));
    for (TopExp_Explorer lowerExp(lower, TopAbs_FACE); lowerExp.More(); lowerExp.Next()) {
        const TopoDS_Face lowerFace = TopoDS::Face(lowerExp.Current());
        for (const TopoDS_Face& selected : visibleFaces)
            if (selected.IsSame(lowerFace))
                return fail(QStringLiteral("top-visible selection retained an occluded lower face"));
    }
    const TopoDS_Shape perforated = makePerforatedPlate(8);
    const auto perforatedVisibleFaces =
        LaserToolpathBuilder::selectTopVisibleFacesFromPositiveZ(perforated);
    if (perforatedVisibleFaces.empty())
        return fail(QStringLiteral("perforated plate has no visible top face"));
    if (std::any_of(perforatedVisibleFaces.cbegin(), perforatedVisibleFaces.cend(),
                    [](const TopoDS_Face& face) {
                        return !LaserToolpathBuilder::isPlanarFace(face);
                    })) {
        return fail(QStringLiteral("perforated plate retained a vertical hole wall"));
    }

    // A blind-hole floor is visible through its opening, but it is a cavity
    // face rather than part of the exterior machining shell.  The Z-light
    // strategy must retain only the plate's upper face.
    const TopoDS_Solid solidPlate = BRepPrimAPI_MakeBox(
        gp_Pnt(0.0, 0.0, 0.0), gp_Pnt(100.0, 100.0, 10.0)).Solid();
    const TopoDS_Shape blindHole = BRepPrimAPI_MakeCylinder(
        gp_Ax2(gp_Pnt(50.0, 50.0, 5.0), gp_Dir(0.0, 0.0, 1.0)), 10.0, 5.0).Shape();
    BRepAlgoAPI_Cut blindCut(solidPlate, blindHole);
    blindCut.Build();
    if (!blindCut.IsDone())
        return fail(QStringLiteral("blind-hole boolean cut failed"));
    const auto blindHoleVisibleFaces =
        LaserToolpathBuilder::selectTopVisibleFacesFromPositiveZ(blindCut.Shape());
    if (blindHoleVisibleFaces.size() != 1
        || !LaserToolpathBuilder::isPlanarFace(blindHoleVisibleFaces.front())) {
        return fail(QStringLiteral("top-visible selection retained a blind-hole floor"));
    }

    // The lateral wall is the largest smooth group on a vertical cylinder,
    // but Z-light extraction must retain the smaller +Z cap instead.
    const TopoDS_Shape verticalCylinder = BRepPrimAPI_MakeCylinder(
        gp_Ax2(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)), 10.0, 60.0).Shape();
    const auto verticalVisibleFaces =
        LaserToolpathBuilder::selectTopVisibleFacesFromPositiveZ(verticalCylinder);
    if (verticalVisibleFaces.size() != 1
        || !LaserToolpathBuilder::isPlanarFace(verticalVisibleFaces.front())) {
        return fail(QStringLiteral("vertical cylinder did not select only its +Z top cap"));
    }

    // A horizontal-axis cylinder exposes its curved lateral face to vertical
    // light; top visibility must not be restricted to horizontal planar faces.
    const TopoDS_Shape horizontalCylinder = BRepPrimAPI_MakeCylinder(
        gp_Ax2(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(1.0, 0.0, 0.0)), 10.0, 60.0).Shape();
    const auto curvedVisibleFaces =
        LaserToolpathBuilder::selectTopVisibleFacesFromPositiveZ(horizontalCylinder);
    if (std::none_of(curvedVisibleFaces.cbegin(), curvedVisibleFaces.cend(),
                     [](const TopoDS_Face& face) {
                         return !LaserToolpathBuilder::isPlanarFace(face);
                     })) {
        return fail(QStringLiteral("top-visible selection did not retain the curved upper surface"));
    }
    return 0;
}

int verifyPipelineChain()
{
    lcnc::cam::CamDataManager data;
    using lcnc::cam::CamPipelineStage;
    data.commitPipelineStage(CamPipelineStage::FaceSeparation);
    const auto face = data.pipelineStageState(CamPipelineStage::FaceSeparation).revision;
    data.commitPipelineStage(CamPipelineStage::ContourExtraction, face);
    const auto contour = data.pipelineStageState(CamPipelineStage::ContourExtraction).revision;
    data.commitPipelineStage(CamPipelineStage::PointDiscretization, contour);
    const auto points = data.pipelineStageState(CamPipelineStage::PointDiscretization).revision;
    data.commitPipelineStage(CamPipelineStage::GeometricToolpath, points);
    const auto path = data.pipelineStageState(CamPipelineStage::GeometricToolpath).revision;
    data.commitPipelineStage(CamPipelineStage::MachineSolve, path);
    if (!data.hasCompletePipelineChain())
        return fail(QStringLiteral("pipeline chain should be valid"));

    auto invalid = data.pipelineStageState(CamPipelineStage::MachineSolve);
    invalid.inputRevision = path + 1;
    data.restorePipelineStageState(CamPipelineStage::MachineSolve, invalid);
    if (data.hasCompletePipelineChain())
        return fail(QStringLiteral("pipeline chain accepted mismatched machine-solve input"));
    return 0;
}

int verifyAsynchronousResultContracts()
{
    lcnc::cam::ToolpathGenerationStamp captured;
    captured.toolpathRevision = 10;
    captured.machiningFaceRevision = 20;
    captured.machineSetupRevision = 30;
    captured.leadInLength = 4.0;
    captured.smoothAngle = 5.0;
    captured.deflection = 0.1;
    captured.cuttingOffsetMm = 1.0;
    captured.rapidOffsetMm = 5.0;
    captured.useFaceClassification = true;
    captured.extractionStrategy = 2;
    captured.contourIds = {101, 102};
    captured.sources.push_back({
        QStringLiteral("0:1"), 0, lcnc::cad_algo::makeBox(10.0, 10.0, 2.0)});

    using lcnc::cam::ToolpathGenerationService;
    if (!ToolpathGenerationService::acceptsResult(captured, captured, true, false))
        return fail(QStringLiteral("Unchanged asynchronous CAM result was rejected"));

    const auto mustReject = [&captured](lcnc::cam::ToolpathGenerationStamp changed,
                                        const QString& label) {
        if (ToolpathGenerationService::acceptsResult(captured, changed, true, false))
            return fail(QStringLiteral("Stale CAM result accepted after %1 changed").arg(label));
        return 0;
    };
    auto changed = captured;
    ++changed.machiningFaceRevision;
    if (const int rc = mustReject(changed, QStringLiteral("machining faces")); rc != 0)
        return rc;
    changed = captured;
    ++changed.machineSetupRevision;
    if (const int rc = mustReject(changed, QStringLiteral("machine setup")); rc != 0)
        return rc;
    changed = captured;
    changed.contourIds.pop_back();
    if (const int rc = mustReject(changed, QStringLiteral("contour selection")); rc != 0)
        return rc;
    changed = captured;
    changed.sources.front().shape = lcnc::cad_algo::makeBox(11.0, 10.0, 2.0);
    if (const int rc = mustReject(changed, QStringLiteral("source geometry")); rc != 0)
        return rc;
    if (ToolpathGenerationService::acceptsResult(captured, captured, false, false)
        || ToolpathGenerationService::acceptsResult(captured, captured, true, true)) {
        return fail(QStringLiteral("Failed or cancelled CAM result was accepted"));
    }

    lcnc::cam::MotionCompilationContext motionContext;
    motionContext.sourceToolpathRevision = captured.toolpathRevision;
    motionContext.controllerQualification.sourceId =
        QStringLiteral("legacy/default-physical-axes-unqualified");
    motionContext.controllerCapabilityHash =
        lcnc::cam::controllerQualificationSnapshotHash(
            motionContext.controllerQualification);
    QVector<lcnc::cam::MotionCompilationInput::Parameter> policy{
        {QStringLiteral("optimizationMode"), QStringLiteral("Off"),
         QStringLiteral("Off"), QStringLiteral("compiler/builtin"),
         QStringLiteral("enum"), 1, true}};
    const auto motionInput = ToolpathGenerationService::captureMotionInput(
        captured, motionContext, policy);
    policy[0].requested = QStringLiteral("Full");
    if (motionInput.parameters.constFirst().requested != QStringLiteral("Off")
        || !ToolpathGenerationService::acceptsMotionResult(
            motionInput, captured, motionInput.context, true, false)) {
        return fail(QStringLiteral("Motion compilation input was not frozen at capture"));
    }
    auto staleMotionContext = motionInput.context;
    staleMotionContext.controllerQualification.state =
        lcnc::cam::ControllerQualificationState::Qualified;
    if (ToolpathGenerationService::acceptsMotionResult(
            motionInput, captured, staleMotionContext, true, false)
        || ToolpathGenerationService::acceptsMotionResult(
            motionInput, changed, motionInput.context, true, false)
        || ToolpathGenerationService::acceptsMotionResult(
            motionInput, captured, motionInput.context, true, true)) {
        return fail(QStringLiteral("Stale or cancelled motion worker result was accepted"));
    }

    lcnc::cam::MachiningFacePipelineService faces;
    const TopoDS_Shape source = lcnc::cad_algo::makeBox(8.0, 6.0, 2.0);
    TopExp_Explorer explorer(source, TopAbs_FACE);
    if (!explorer.More())
        return fail(QStringLiteral("Face-pipeline fixture has no face"));
    const TopoDS_Face face = TopoDS::Face(explorer.Current());
    const lcnc::cam::MachiningFacePipelineService::Candidate candidate{
        face, QStringLiteral("0:2"), lcnc::cam::MachiningFaceRole::MachiningSurface};
    if (!faces.replaceAutomaticFaces({candidate}) || faces.entries().size() != 1)
        return fail(QStringLiteral("Automatic machining face was not accepted"));
    const auto stableId = faces.entries().front().faceId;

    const TopoDS_Shape rebuilt = lcnc::cad_algo::makeBox(8.0, 6.0, 2.0);
    TopExp_Explorer rebuiltExplorer(rebuilt, TopAbs_FACE);
    if (!rebuiltExplorer.More()
        || !faces.replaceAutomaticFaces({{
            TopoDS::Face(rebuiltExplorer.Current()), QStringLiteral("0:2"),
            lcnc::cam::MachiningFaceRole::MachiningSurface}})
        || faces.entries().front().faceId != stableId) {
        return fail(QStringLiteral("Equivalent automatic face did not preserve its stable id"));
    }
    const auto records = faces.persistenceRecords();
    lcnc::cam::MachiningFacePipelineService rebound;
    const auto result = rebound.rebindFromRecords(
        records, {{QStringLiteral("0:2"), rebuilt}});
    if (result.reboundCount != 1 || !result.missingFaces.empty()
        || rebound.entries().front().faceId != stableId) {
        return fail(QStringLiteral("Persisted machining face could not be rebound"));
    }
    return 0;
}

int verifyTransactionalOrderedSolve()
{
    LaserContour contour;
    contour.contourId = 42;
    ToolpathPoint point;
    point.position = gp_Pnt(3.0, 2.0, 1.0);
    point.machineCoord.x = 123.0;
    point.machineCoord.valid = true;
    contour.points.push_back(point);
    std::vector<LaserContour> contours{contour};

    MachineKinematics planningKinematics;
    QString error;
    const bool solved = lcnc::cam::ToolpathSolveService::solveTransactionally(
        &contours, QVector<std::uint64_t>{42}, &planningKinematics,
        lcnc::MachineModeDefinition{}, lcnc::WorkpieceSetupTransform{},
        lcnc::HeadToolGeometry{}, &error);
    if (solved || error.isEmpty())
        return fail(QStringLiteral("Invalid ordered solve unexpectedly succeeded"));
    if (contours.size() != 1 || contours.front().points.size() != 1
        || !contours.front().points.front().machineCoord.valid
        || std::abs(contours.front().points.front().machineCoord.x - 123.0) > 1e-12
        || contours.front().points.front().position.Distance(gp_Pnt(3.0, 2.0, 1.0)) > 1e-12) {
        return fail(QStringLiteral("Failed ordered solve mutated the committed contour"));
    }
    return 0;
}

int verifyFrozenSolveAndInvalidation()
{
    using namespace lcnc::cam;
    MachineKinematics live;
    live.loadPreset(QStringLiteral("XYZ"));
    MotionCompilationContext context;
    context.machineKinematicsHash = QByteArrayLiteral("xyz-fixture");
    auto input = ToolpathGenerationService::captureMotionInput({}, context, {});
    input.machineAxes = live.axes();
    input.machineConfigType = live.configType();
    input.modeDefinition.mode = lcnc::MachiningMode::Planar3Axis;
    input.modeDefinition.solverId = QStringLiteral("Planar3Axis");
    input.modeDefinition.solverVersion = lcnc::machiningModeSolverVersion(input.modeDefinition.mode);
    input.modeDefinition.interpolatedAxes.append(QStringLiteral("X"), lcnc::MachineAxisRole::LinearX);
    input.modeDefinition.interpolatedAxes.append(QStringLiteral("Y"), lcnc::MachineAxisRole::LinearY);
    input.modeDefinition.interpolatedAxes.append(QStringLiteral("Z"), lcnc::MachineAxisRole::LinearZ);
    input.workpieceSetup.x = 5.0;
    // The same frozen payload used by the production worker must not consult
    // the mutable machine after dispatch.
    live.findAxis(QStringLiteral("X"))->direction = gp_Dir(-1, 0, 0);
    LaserContour contour;
    contour.contourId = 42;
    contour.geometrySamplingComplete = true;
    contour.geometrySamplingEvidence = {true, true, true, {}};
    contour.needsRecalculation = false;
    ToolpathPoint point;
    point.position = gp_Pnt(10, 20, 30);
    contour.points.push_back(point);
    std::vector<LaserContour> contours{contour};
    QString error;
    if (!ToolpathSolveService::solveFrozen(&contours, {42}, input, &error)
        || std::abs(contours.front().points.front().machineCoord.solvedPose.value(0) - 15.0) > 1e-9)
        return fail(QStringLiteral("Frozen worker did not use captured machine/setup: ") + error);
    if (!ToolpathSolveService::geometryHasCurrentSolve(contours))
        return fail(QStringLiteral("Fresh authoritative solution failed export gate"));
    auto changedContext = context;
    changedContext.machineKinematicsHash = QByteArrayLiteral("changed-live-machine");
    const auto current = ToolpathGenerationService::captureMotionInput({}, changedContext, {});
    if (ToolpathGenerationService::sameMotionAuthority(input, current))
        return fail(QStringLiteral("Changed live authority accepted frozen result"));
    auto points = contours.front().points;
    points.front().position.SetX(11.0);
    LaserToolpathBuilder::replaceGeometrySamples(contours.front(), std::move(points), true);
    if (ToolpathSolveService::geometryHasCurrentSolve(contours))
        return fail(QStringLiteral("Geometry replacement still passed production export gate"));
    if (!ToolpathSolveService::solveFrozen(&contours, {42}, input, &error))
        return fail(QStringLiteral("Authoritative re-solve after invalidation failed: ") + error);
    contours.front().needsRecalculation = false;
    if (!ToolpathSolveService::geometryHasCurrentSolve(contours)
        || std::abs(contours.front().points.front().machineCoord.solvedPose.value(0) - 16.0) > 1e-9)
        return fail(QStringLiteral("Re-solve did not restore valid export with updated coordinates"));
    return 0;
}

int verifyExecutionRevisionCoverage()
{
    LaserToolpath toolpath;
    LaserContour contour;
    contour.contourId = 7;
    contour.layerId = 11;
    contour.name = QStringLiteral("profile");
    contour.workpieceEntry = QStringLiteral("0:1");
    contour.points.push_back(ToolpathPoint{});
    toolpath.contours().push_back(contour);
    ToolpathLayer layer;
    layer.layerId = 11;
    layer.name = QStringLiteral("layer");
    layer.toolName = QStringLiteral("tool-a");
    layer.contourIds = {7};
    toolpath.layers().push_back(layer);

    const auto revision = lcnc::cam::ToolpathSequenceService::computeToolpathRevision(
        toolpath, false, true);
    auto changed = toolpath;
    changed.contours().front().appliedParams.cuttingOffsetMm += 0.25;
    if (lcnc::cam::ToolpathSequenceService::computeToolpathRevision(changed, false, true)
        == revision) {
        return fail(QStringLiteral("Cutting offset did not invalidate CAM revision"));
    }
    changed = toolpath;
    changed.contours().front().appliedParams.rapidOffsetMm += 1.0;
    if (lcnc::cam::ToolpathSequenceService::computeToolpathRevision(changed, false, true)
        == revision) {
        return fail(QStringLiteral("Rapid offset did not invalidate CAM revision"));
    }
    changed = toolpath;
    changed.layers().front().toolName = QStringLiteral("tool-b");
    if (lcnc::cam::ToolpathSequenceService::computeToolpathRevision(changed, false, true)
        == revision) {
        return fail(QStringLiteral("Tool mapping did not invalidate CAM revision"));
    }
    if (lcnc::cam::ToolpathSequenceService::computeToolpathRevision(toolpath, false, false)
        == revision) {
        return fail(QStringLiteral("Incomplete CAM pipeline did not invalidate CAM revision"));
    }
    return 0;
}

} // namespace

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);
    const int holeCount = 3;
    const TopoDS_Shape plate = makePerforatedPlate(holeCount);

    if (int rc = verifyPerforatedPlate(plate, holeCount,
                                       ExtractionStrategy::PlanarFaceWires,
                                       QStringLiteral("PlanarZLight"),
                                       // The planar strategy is fixed to the workpiece
                                       // XY/Z coordinate system, not machine posture.
                                       gp_Dir(1.0, 0.0, 0.0)))
        return rc;

    if (int rc = verifyPerforatedPlate(plate, holeCount,
                                       ExtractionStrategy::LargestSmoothConnectedSurface,
                                       QStringLiteral("LargestSmoothConnectedSurface")))
        return rc;

    const TopoDS_Shape cylinder = BRepPrimAPI_MakeCylinder(
        gp_Ax2(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)), 20.0, 80.0).Shape();
    if (int rc = verifyLargestSmoothSurfaceRegression(
            cylinder, QStringLiteral("LargestSmoothConnectedSurface")))
        return rc;
    if (int rc = verifyTopVisibleFaceSelection())
        return rc;
    if (int rc = verifyPipelineChain())
        return rc;
    if (int rc = verifyAsynchronousResultContracts())
        return rc;
    if (int rc = verifyTransactionalOrderedSolve())
        return rc;
    if (int rc = verifyFrozenSolveAndInvalidation())
        return rc;
    if (const auto error = CamMotionCompilationTestAccess::verify(); !error.isEmpty())
        return fail(error);
    if (int rc = verifyExecutionRevisionCoverage())
        return rc;

    QTextStream(stderr) << "cam_algorithm_pipeline_test: ok\n";
    return 0;
}
