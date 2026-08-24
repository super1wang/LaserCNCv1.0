#include "modules/cam/collision/coal_collision_backend.h"
#include "modules/cam/collision/collision_geometry_cache.h"
#include "modules/cam/collision/continuous_motion_certificate_builder.h"
#include "modules/cam/collision/cutter_collision_geometry.h"
#include "modules/cam/collision/workpiece_clearance_field.h"
#include "core/kinematics/machine_kinematics.h"
#include "modules/cam/contracts/toolpath_export_dto.h"
#include "modules/cam/settings/cam_config.h"

#include <BRepPrimAPI_MakeBox.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <string>

int main()
{
    const auto require = [](bool condition, const char* message) {
        if (!condition)
            std::cerr << message << '\n';
        return condition;
    };
    if (!require(lcnc::cam::coalCollisionBackendAvailable(),
                 "Coal backend is unavailable")) return 1;

    const TopoDS_Shape firstShape = BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();
    const TopoDS_Shape secondShape = BRepPrimAPI_MakeBox(5.0, 5.0, 5.0).Shape();
    const auto firstSurface = lcnc::cam_algo::SurfaceCollisionModel::build(
        firstShape, 0.05);
    const auto secondSurface = lcnc::cam_algo::SurfaceCollisionModel::build(
        secondShape, 0.05);
    if (!require(firstSurface.isValid(), "First surface mesh is invalid")) return 2;
    if (!require(secondSurface.isValid(), "Second surface mesh is invalid")) return 3;

    std::string error;
    const auto first = lcnc::cam::buildCoalCollisionBodyModel(firstSurface, &error);
    if (!require(first && error.empty(), "First Coal BVH is invalid")) return 4;
    const auto second = lcnc::cam::buildCoalCollisionBodyModel(secondSurface, &error);
    if (!require(second && error.empty(), "Second Coal BVH is invalid")) return 5;
    const auto pair = lcnc::cam::buildCoalCollisionPair(first, second);
    if (!require(static_cast<bool>(pair), "Coal pair is invalid")) return 6;

    gp_Trsf identity;
    gp_Trsf separatedTransform;
    separatedTransform.SetTranslation(gp_Vec(20.0, 0.0, 0.0));
    const auto separated = lcnc::cam::queryCoalCollisionPair(
        pair, identity, separatedTransform, 0.7);
    if (!require(separated.available, "Separated query is unavailable")) return 7;
    if (!require(separated.definitelySeparated, "Separated boxes were not certified")) return 8;
    if (!require(separated.distanceLowerBoundMm > 0.7,
                 "Separated lower bound is insufficient")) return 9;

    gp_Trsf overlappingTransform;
    overlappingTransform.SetTranslation(gp_Vec(8.0, 0.0, 0.0));
    const auto overlapping = lcnc::cam::queryCoalCollisionPair(
        pair, identity, overlappingTransform, 0.7);
    if (!require(overlapping.available, "Overlapping query is unavailable")) return 10;
    if (!require(!overlapping.definitelySeparated,
                 "Overlapping boxes were falsely certified safe")) return 11;

    auto geometry = std::make_shared<lcnc::cam::TravelCollisionGeometryCache>();
    lcnc::cam::TravelCollisionBody movingBody;
    movingBody.entry = QStringLiteral("axis-z");
    movingBody.source = QStringLiteral("axis:Z");
    movingBody.active = true;
    movingBody.sourceShape = secondShape;
    lcnc::cam::buildCollisionGeometry(&movingBody, 0.05);
    lcnc::cam::TravelCollisionBody fixedBody;
    fixedBody.entry = QStringLiteral("fixed");
    fixedBody.source = QStringLiteral("axis:BASE");
    fixedBody.passive = true;
    fixedBody.workpiece = true;
    fixedBody.sourceShape = firstShape;
    lcnc::cam::buildCollisionGeometry(&fixedBody, 0.05);
    geometry->bodies = {movingBody, fixedBody};
    MachineKinematics xyzKinematics;
    xyzKinematics.loadPreset(QStringLiteral("XYZ"));
    geometry->axes = xyzKinematics.axes();
    geometry->configType = xyzKinematics.configType();
    geometry->assignments.insert(movingBody.entry, QStringLiteral("Z"));
    geometry->mounts.insert(QStringLiteral("workpiece"), QStringLiteral("BASE"));
    lcnc::cam::buildCoalCollisionPairs(geometry.get());

    const auto certificatesFor = [&](double firstX, double lastX,
                                     std::uint64_t intervalBudget = 1024,
                                     bool machinePackageRequired = false) {
        lcnc::cam::ToolpathExportSnapshot snapshot;
        snapshot.collisionSafety.enabled = true;
        snapshot.collisionSafety.machinePackageRequired =
            machinePackageRequired;
        snapshot.collisionSafety.jobOverlayRequired = true;
        snapshot.motionPlan.revision = 7;
        snapshot.machineAxisLayout.append(
            QStringLiteral("X"), lcnc::MachineAxisRole::LinearX);
        snapshot.machineAxisLayout.append(
            QStringLiteral("Y"), lcnc::MachineAxisRole::LinearY);
        snapshot.machineAxisLayout.append(
            QStringLiteral("Z"), lcnc::MachineAxisRole::LinearZ);
        lcnc::cam::ToolpathExportContour contour;
        contour.contourId = 1;
        contour.workpieceEntry = QStringLiteral("workpiece");
        snapshot.contours.append(contour);
        lcnc::cam::CamMotionNode firstNode;
        firstNode.phase = lcnc::cam::CamMotionPhase::Rapid;
        firstNode.contourId = 1;
        firstNode.tcpX = firstX;
        firstNode.axes[0] = firstX;
        firstNode.axisMask = 0x07;
        lcnc::cam::CamMotionNode lastNode = firstNode;
        lastNode.tcpX = lastX;
        lastNode.axes[0] = lastX;
        snapshot.motionPlan.nodes = {firstNode, lastNode};
        lcnc::cam::ContinuousMotionCertificateBuildContext context;
        context.geometry = geometry;
        context.maximumSubdivisionDepth = 4;
        context.maximumIntervalsPerEdge = intervalBudget;
        return lcnc::cam::buildContinuousMotionCertificates(snapshot, context);
    };
    const auto safeCertificates = certificatesFor(20.0, 21.0);
    if (!require(safeCertificates.size() == 1
                     && safeCertificates.constFirst().state
                         == lcnc::cam::CamMotionCertificateState::CertifiedSafe,
                 "Production continuous builder did not certify a separated edge")) {
        return 12;
    }
    const auto jobOnlyCannotPromoteMachineUnknown = certificatesFor(
        20.0, 21.0, 1024, true);
    if (!require(jobOnlyCannotPromoteMachineUnknown.size() == 1
                     && jobOnlyCannotPromoteMachineUnknown.constFirst().state
                         == lcnc::cam::CamMotionCertificateState::BoundaryUnknown,
                 "Job-only geometry promoted LMSI uncertainty to safe")) {
        return 28;
    }
    const auto blockedCertificates = certificatesFor(0.0, 1.0);
    if (!require(blockedCertificates.size() == 1
                     && blockedCertificates.constFirst().state
                         != lcnc::cam::CamMotionCertificateState::CertifiedSafe,
                 "Production continuous builder falsely certified a penetrating edge")) {
        return 13;
    }
    const auto budgetedCertificates = certificatesFor(10.0, 20.0, 1);
    if (!require(budgetedCertificates.size() == 1
                     && budgetedCertificates.constFirst().state
                         == lcnc::cam::CamMotionCertificateState::BoundaryUnknown
                     && budgetedCertificates.constFirst().budgetExhausted
                     && budgetedCertificates.constFirst().intervalQueries == 1,
                 "Continuous builder did not fail closed at its query budget")) {
        return 14;
    }

    // Reproduce the GUI regression: LMSI is Unknown, but the complete machine
    // fallback pair is conservatively separated. The geometric proof must be
    // allowed to certify the edge instead of returning the stale LMSI state.
    auto machineFallbackGeometry = std::make_shared<
        lcnc::cam::TravelCollisionGeometryCache>();
    lcnc::cam::TravelCollisionBody fallbackActive;
    fallbackActive.entry = QStringLiteral("fallback-active");
    fallbackActive.source = QStringLiteral("axis:Z");
    fallbackActive.active = true;
    fallbackActive.sourceShape = BRepPrimAPI_MakeBox(
        gp_Pnt(30.0, 0.0, 0.0), 5.0, 5.0, 5.0).Shape();
    lcnc::cam::buildCollisionGeometry(&fallbackActive, 0.05);
    lcnc::cam::TravelCollisionBody fallbackPassive;
    fallbackPassive.entry = QStringLiteral("fallback-passive");
    fallbackPassive.source = QStringLiteral("axis:A");
    fallbackPassive.passive = true;
    fallbackPassive.sourceShape = firstShape;
    lcnc::cam::buildCollisionGeometry(&fallbackPassive, 0.05);
    machineFallbackGeometry->bodies = {fallbackActive, fallbackPassive};
    machineFallbackGeometry->machineAxisPairKeys.insert(
        lcnc::cam::collisionAxisPairKey(
            fallbackActive.source, fallbackPassive.source));

    lcnc::cam::ToolpathExportSnapshot machineFallbackSnapshot;
    machineFallbackSnapshot.collisionSafety.enabled = true;
    machineFallbackSnapshot.collisionSafety.machinePackageRequired = true;
    machineFallbackSnapshot.motionPlan.revision = 11;
    lcnc::cam::ToolpathExportContour machineFallbackContour;
    machineFallbackContour.contourId = 6;
    machineFallbackContour.workpieceEntry = QStringLiteral("workpiece");
    machineFallbackSnapshot.contours.append(machineFallbackContour);
    lcnc::cam::CamMotionNode machineFallbackNode;
    machineFallbackNode.phase = lcnc::cam::CamMotionPhase::Rapid;
    machineFallbackNode.contourId = 6;
    machineFallbackSnapshot.motionPlan.nodes = {
        machineFallbackNode, machineFallbackNode};
    lcnc::cam::ContinuousMotionCertificateBuildContext machineFallbackContext;
    machineFallbackContext.geometry = machineFallbackGeometry;
    machineFallbackContext.maximumOcctExactQueriesPerEdge = 0;
    const auto machineFallbackCertificates =
        lcnc::cam::buildContinuousMotionCertificates(
            machineFallbackSnapshot, machineFallbackContext);
    if (!require(machineFallbackCertificates.size() == 1
                     && machineFallbackCertificates.constFirst().state
                         == lcnc::cam::CamMotionCertificateState::CertifiedSafe
                     && machineFallbackCertificates.constFirst()
                            .broadPhaseRejected == 1
                     && machineFallbackCertificates.constFirst()
                            .occtExactQueries == 0,
                 "Complete geometric fallback did not resolve LMSI uncertainty")) {
        return 29;
    }

    // Reproduce the production AC-table chain. A stationary workpiece mounted
    // on C must not subdivide merely because A/C have large configured travel
    // ranges; document-local geometry is already positioned at the home pose.
    MachineKinematics tableKinematics;
    tableKinematics.loadPreset(QStringLiteral("VERTICAL_AC_TABLE"));
    geometry->axes = tableKinematics.axes();
    geometry->configType = tableKinematics.configType();
    geometry->mounts.insert(QStringLiteral("workpiece"), QStringLiteral("C"));
    lcnc::cam::ToolpathExportSnapshot stationarySnapshot;
    stationarySnapshot.collisionSafety.enabled = true;
    stationarySnapshot.collisionSafety.jobOverlayRequired = true;
    stationarySnapshot.machineAxisLayout.append(
        QStringLiteral("X"), lcnc::MachineAxisRole::LinearX);
    stationarySnapshot.machineAxisLayout.append(
        QStringLiteral("Y"), lcnc::MachineAxisRole::LinearY);
    stationarySnapshot.machineAxisLayout.append(
        QStringLiteral("Z"), lcnc::MachineAxisRole::LinearZ);
    stationarySnapshot.machineAxisLayout.append(
        QStringLiteral("A"), lcnc::MachineAxisRole::TableTilt);
    stationarySnapshot.machineAxisLayout.append(
        QStringLiteral("C"), lcnc::MachineAxisRole::TableSpin);
    lcnc::cam::ToolpathExportContour stationaryContour;
    stationaryContour.contourId = 2;
    stationaryContour.workpieceEntry = QStringLiteral("workpiece");
    stationarySnapshot.contours.append(stationaryContour);
    lcnc::cam::CamMotionNode stationaryFirst;
    stationaryFirst.phase = lcnc::cam::CamMotionPhase::Rapid;
    stationaryFirst.contourId = 2;
    stationaryFirst.tcpX = 20.0;
    stationaryFirst.axes[0] = 20.0;
    stationaryFirst.axisMask = 0x1f;
    lcnc::cam::CamMotionNode stationaryLast = stationaryFirst;
    stationarySnapshot.motionPlan.nodes = {stationaryFirst, stationaryLast};
    lcnc::cam::ContinuousMotionCertificateBuildContext stationaryContext;
    stationaryContext.geometry = geometry;
    const auto stationaryCertificates =
        lcnc::cam::buildContinuousMotionCertificates(
            stationarySnapshot, stationaryContext);
    if (!require(stationaryCertificates.size() == 1
                     && stationaryCertificates.constFirst().state
                         == lcnc::cam::CamMotionCertificateState::CertifiedSafe
                     && stationaryCertificates.constFirst().intervalQueries == 1
                     && stationaryCertificates.constFirst().broadPhaseRejected == 1
                     && stationaryCertificates.constFirst().surfaceBvhQueries == 0
                     && stationaryCertificates.constFirst().surfaceBvhRejected == 0
                     && stationaryCertificates.constFirst().coalPairQueries == 0
                     && stationaryCertificates.constFirst().maximumSubdivisionDepth == 0,
                 "Stationary AC-table edge was exponentially subdivided")) {
        return 15;
    }

    // Match the node count of the reported production path. Every stationary,
    // well-separated edge must remain one conservative query; this prevents a
    // future fixed motion-bound term from restoring exponential subdivision.
    stationarySnapshot.motionPlan.nodes.fill(stationaryFirst, 4814);
    auto serialContext = stationaryContext;
    serialContext.maximumParallelWorkers = 1;
    const auto serialPathStarted = std::chrono::steady_clock::now();
    const auto serialPathCertificates =
        lcnc::cam::buildContinuousMotionCertificates(
            stationarySnapshot, serialContext);
    const auto serialPathElapsedMs = std::chrono::duration_cast<
        std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - serialPathStarted).count();
    const auto largePathStarted = std::chrono::steady_clock::now();
    const auto largePathCertificates =
        lcnc::cam::buildContinuousMotionCertificates(
            stationarySnapshot, stationaryContext);
    const auto largePathElapsedMs = std::chrono::duration_cast<
        std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - largePathStarted).count();
    std::uint64_t largePathIntervals = 0;
    std::uint64_t largePathBroadPhaseRejected = 0;
    std::uint64_t largePathSurfaceBvhQueries = 0;
    std::uint64_t largePathSurfaceBvhRejected = 0;
    std::uint64_t largePathCoalQueries = 0;
    bool largePathSafe = largePathCertificates.size() == 4813;
    for (const auto& certificate : largePathCertificates) {
        largePathIntervals += certificate.intervalQueries;
        largePathBroadPhaseRejected += certificate.broadPhaseRejected;
        largePathSurfaceBvhQueries += certificate.surfaceBvhQueries;
        largePathSurfaceBvhRejected += certificate.surfaceBvhRejected;
        largePathCoalQueries += certificate.coalPairQueries;
        largePathSafe = largePathSafe
            && certificate.state
                == lcnc::cam::CamMotionCertificateState::CertifiedSafe;
    }
    std::cout << "continuous_edges=" << largePathCertificates.size()
              << " serial_elapsed_ms=" << serialPathElapsedMs
              << " intervals=" << largePathIntervals
              << " broad_phase_rejected=" << largePathBroadPhaseRejected
              << " surface_bvh_queries=" << largePathSurfaceBvhQueries
              << " surface_bvh_rejected=" << largePathSurfaceBvhRejected
              << " coal_queries=" << largePathCoalQueries
              << " elapsed_ms=" << largePathElapsedMs << '\n';
    const bool serialPathSafe = serialPathCertificates.size() == 4813
        && std::all_of(serialPathCertificates.cbegin(),
                       serialPathCertificates.cend(),
                       [](const auto& certificate) {
                           return certificate.state
                               == lcnc::cam::CamMotionCertificateState::CertifiedSafe;
                       });
    if (!require(serialPathSafe
                     && largePathSafe
                     && largePathIntervals == 4813
                     && largePathBroadPhaseRejected == 4813
                     && largePathSurfaceBvhQueries == 0
                     && largePathSurfaceBvhRejected == 0
                     && largePathCoalQueries == 0,
                 "Large stationary path did not retain one-query certificates")) {
        return 16;
    }

    // Exercise independent per-worker Coal contexts on a path that cannot be
    // rejected by the surface BVH. Every edge must remain fail-closed while
    // parallel workers query the same immutable body models.
    lcnc::cam::ToolpathExportSnapshot parallelBlockedSnapshot;
    parallelBlockedSnapshot.collisionSafety.enabled = true;
    parallelBlockedSnapshot.collisionSafety.jobOverlayRequired = true;
    parallelBlockedSnapshot.collisionSafety.machinePackageRequired = true;
    parallelBlockedSnapshot.motionPlan.revision = 8;
    lcnc::cam::ToolpathExportContour blockedContour;
    blockedContour.contourId = 3;
    blockedContour.workpieceEntry = QStringLiteral("workpiece");
    parallelBlockedSnapshot.contours.append(blockedContour);
    lcnc::cam::CamMotionNode blockedNode;
    blockedNode.phase = lcnc::cam::CamMotionPhase::Rapid;
    blockedNode.contourId = 3;
    parallelBlockedSnapshot.motionPlan.nodes.fill(blockedNode, 513);
    auto coalDemotionGeometry = std::make_shared<
        lcnc::cam::TravelCollisionGeometryCache>();
    auto compactActive = movingBody;
    compactActive.entry = QStringLiteral("compact-active");
    compactActive.source = QStringLiteral("compact-active");
    auto compactPassive = fixedBody;
    compactPassive.entry = QStringLiteral("compact-passive");
    compactPassive.source = QStringLiteral("compact-passive");
    compactPassive.workpiece = false;
    coalDemotionGeometry->bodies = {compactActive, compactPassive};
    coalDemotionGeometry->machineAxisPairKeys.insert(
        lcnc::cam::collisionAxisPairKey(
            compactActive.source, compactPassive.source));
    lcnc::cam::buildCoalCollisionPairs(coalDemotionGeometry.get());
    std::cout << "compact_coal_pairs="
              << coalDemotionGeometry->coalPairs.size() << '\n';
    lcnc::cam::ContinuousMotionCertificateBuildContext parallelBlockedContext;
    parallelBlockedContext.geometry = coalDemotionGeometry;
    parallelBlockedContext.maximumSubdivisionDepth = 2;
    parallelBlockedContext.maximumParallelWorkers = 8;
    parallelBlockedContext.maximumParallelCoalQueries = 1;
    parallelBlockedContext.maximumCoalQueryWallTimeMs = 0;
    const auto parallelBlockedCertificates =
        lcnc::cam::buildContinuousMotionCertificates(
            parallelBlockedSnapshot, parallelBlockedContext);
    std::uint64_t parallelBlockedCoalQueries = 0;
    std::uint64_t parallelBlockedOcctQueries = 0;
    bool parallelBlockedClosed = parallelBlockedCertificates.size() == 512;
    for (const auto& certificate : parallelBlockedCertificates) {
        parallelBlockedCoalQueries += certificate.coalPairQueries;
        parallelBlockedOcctQueries += certificate.occtExactQueries;
        parallelBlockedClosed = parallelBlockedClosed
            && certificate.state
                == lcnc::cam::CamMotionCertificateState::BoundaryUnknown;
    }
    if (!require(parallelBlockedClosed
                     && parallelBlockedCoalQueries <= 1
                     && parallelBlockedOcctQueries == 0,
                 "Unresolved mesh/Coal pair did not remain online fail-closed")) {
        std::cout << "parallel_blocked_coal_queries="
                  << parallelBlockedCoalQueries
                  << " occt_queries=" << parallelBlockedOcctQueries << '\n';
        return 17;
    }

    // Compact fixed machine-body pairs can use Coal as an offline residual.
    auto machineGeometry = std::make_shared<
        lcnc::cam::TravelCollisionGeometryCache>();
    lcnc::cam::TravelCollisionBody machineZ;
    machineZ.entry = QStringLiteral("axis:Z");
    machineZ.source = QStringLiteral("axis:Z");
    machineZ.active = true;
    machineZ.sourceShape = secondShape;
    lcnc::cam::buildCollisionGeometry(&machineZ, 0.05);
    lcnc::cam::TravelCollisionBody machineAxis;
    machineAxis.entry = QStringLiteral("axis:A");
    machineAxis.source = QStringLiteral("axis:A");
    machineAxis.passive = true;
    machineAxis.sourceShape = firstShape;
    lcnc::cam::buildCollisionGeometry(&machineAxis, 0.05);
    machineGeometry->bodies = {machineZ, machineAxis};
    machineGeometry->machineAxisPairKeys.insert(
        lcnc::cam::collisionAxisPairKey(
            machineZ.source, machineAxis.source));
    lcnc::cam::buildCoalCollisionPairs(machineGeometry.get());
    if (!require(machineGeometry->coalPairs.size() == 1
                     && machineGeometry->bodies.at(0).coalModel
                     && machineGeometry->bodies.at(1).coalModel,
                 "Compact machine pair did not build a Coal BVH")) {
        return 18;
    }
    const auto machineCertificatesFor = [&](double cutterX) {
        lcnc::cam::ToolpathExportSnapshot snapshot;
        snapshot.collisionSafety.enabled = true;
        snapshot.collisionSafety.machinePackageRequired = true;
        snapshot.motionPlan.revision = 9;
        lcnc::cam::ToolpathExportContour contour;
        contour.contourId = 4;
        contour.workpieceEntry = QStringLiteral("workpiece");
        snapshot.contours.append(contour);
        lcnc::cam::CamMotionNode node;
        node.phase = lcnc::cam::CamMotionPhase::Rapid;
        node.contourId = 4;
        node.tcpX = cutterX;
        snapshot.motionPlan.nodes = {node, node};
        lcnc::cam::ContinuousMotionCertificateBuildContext context;
        context.geometry = machineGeometry;
        context.maximumSubdivisionDepth = 0;
        return lcnc::cam::buildContinuousMotionCertificates(snapshot, context);
    };
    const auto exactSafeCertificates = machineCertificatesFor(10.55);
    if (!exactSafeCertificates.isEmpty()) {
        std::cout << "machine_pair_safe_state="
                  << static_cast<int>(exactSafeCertificates.constFirst().state)
                  << " surface_queries="
                  << exactSafeCertificates.constFirst().surfaceBvhQueries
                  << " surface_rejected="
                  << exactSafeCertificates.constFirst().surfaceBvhRejected
                  << " coal_queries="
                  << exactSafeCertificates.constFirst().coalPairQueries
                  << " occt_queries="
                  << exactSafeCertificates.constFirst().occtExactQueries
                  << '\n';
    }
    if (!require(exactSafeCertificates.size() == 1
                     && exactSafeCertificates.constFirst().state
                         == lcnc::cam::CamMotionCertificateState::BoundaryUnknown
                     && exactSafeCertificates.constFirst().surfaceBvhQueries > 0
                     && exactSafeCertificates.constFirst().occtExactQueries == 0,
                 "Mesh/Coal uncertainty band did not remain online fail-closed")) {
        return 19;
    }
    const auto exactBlockedCertificates = machineCertificatesFor(8.0);
    if (!require(exactBlockedCertificates.size() == 1
                     && exactBlockedCertificates.constFirst().state
                         == lcnc::cam::CamMotionCertificateState::BoundaryUnknown
                     && exactBlockedCertificates.constFirst().occtExactQueries == 0,
                 "Unresolved compact machine pair did not remain fail-closed")) {
        return 20;
    }
    lcnc::cam::ToolpathExportSnapshot sweptMachineSnapshot;
    sweptMachineSnapshot.collisionSafety.enabled = true;
    sweptMachineSnapshot.collisionSafety.machinePackageRequired = true;
    sweptMachineSnapshot.motionPlan.revision = 10;
    lcnc::cam::ToolpathExportContour sweptContour;
    sweptContour.contourId = 5;
    sweptContour.workpieceEntry = QStringLiteral("workpiece");
    sweptMachineSnapshot.contours.append(sweptContour);
    lcnc::cam::CamMotionNode sweptFirst;
    sweptFirst.phase = lcnc::cam::CamMotionPhase::Rapid;
    sweptFirst.contourId = 5;
    sweptFirst.tcpX = 10.6;
    lcnc::cam::CamMotionNode sweptLast = sweptFirst;
    sweptLast.tcpX = 20.6;
    sweptMachineSnapshot.motionPlan.nodes = {sweptFirst, sweptLast};
    lcnc::cam::ContinuousMotionCertificateBuildContext sweptContext;
    sweptContext.geometry = machineGeometry;
    sweptContext.maximumSubdivisionDepth = 4;
    const auto sweptCertificates =
        lcnc::cam::buildContinuousMotionCertificates(
            sweptMachineSnapshot, sweptContext);
    if (!require(sweptCertificates.size() == 1
                     && sweptCertificates.constFirst().state
                         == lcnc::cam::CamMotionCertificateState::BoundaryUnknown
                     && sweptCertificates.constFirst().occtExactQueries == 0,
                 "Large-motion uncertainty did not remain fail-closed")) {
        return 21;
    }

    const auto fieldModel = lcnc::cam_algo::SurfaceCollisionModel::build(
        BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape(), 0.005);
    lcnc::cam::LocalClearanceField field(fieldModel, 0.5, 0.02);
    if (!require(field.isValid()
                     && field.certifiesSphereSeparated(
                         gp_Pnt(20.0, 5.0, 5.0), 1.0, 0.5),
                 "Local clearance field did not certify a separated sphere")) {
        return 26;
    }
    if (!require(!field.certifiesSphereSeparated(
                         gp_Pnt(5.0, 5.0, 5.0), 0.1, 0.0)
                     && !field.certifiesSphereSeparated(
                         gp_Pnt(10.1, 5.0, 5.0), 0.2, 0.0),
                 "Local clearance field produced a false-safe inside/boundary result")) {
        return 27;
    }
    const auto cover = lcnc::cam::buildConservativeSphereCover(fieldModel, 12);
    const auto soup = fieldModel.triangleSoup();
    const auto pointCovered = [&cover](const gp_Pnt& point) {
        bool covered = false;
        for (const auto& sphere : cover) {
            if (point.Distance(sphere.center) <= sphere.radiusMm + 1.0e-9) {
                covered = true;
                break;
            }
        }
        return covered;
    };
    bool allSamplesCovered = !cover.isEmpty();
    for (const auto& vertex : soup.vertices) {
        allSamplesCovered = allSamplesCovered && pointCovered(
            gp_Pnt(vertex[0], vertex[1], vertex[2]));
    }
    for (const auto& triangle : soup.triangles) {
        const auto& triangleFirst = soup.vertices[triangle[0]];
        const auto& triangleSecond = soup.vertices[triangle[1]];
        const auto& triangleThird = soup.vertices[triangle[2]];
        for (int u = 0; u <= 4; ++u) {
            for (int v = 0; v <= 4 - u; ++v) {
                const double firstWeight = static_cast<double>(u) / 4.0;
                const double secondWeight = static_cast<double>(v) / 4.0;
                const double thirdWeight = 1.0 - firstWeight - secondWeight;
                allSamplesCovered = allSamplesCovered && pointCovered(gp_Pnt(
                    firstWeight * triangleFirst[0]
                        + secondWeight * triangleSecond[0]
                        + thirdWeight * triangleThird[0],
                    firstWeight * triangleFirst[1]
                        + secondWeight * triangleSecond[1]
                        + thirdWeight * triangleThird[1],
                    firstWeight * triangleFirst[2]
                        + secondWeight * triangleSecond[2]
                        + thirdWeight * triangleThird[2]));
            }
        }
    }
    if (!require(allSamplesCovered,
                 "Conservative sphere cover omitted collision-mesh samples")) {
        return 28;
    }
    lcnc::cam::TravelCollisionBody machineActive;
    machineActive.source = QStringLiteral("axis:Z");
    machineActive.active = true;
    lcnc::cam::TravelCollisionBody machinePassive;
    machinePassive.source = QStringLiteral("axis:A");
    // A/C belong to the workpiece's rigid chain and are not Job Overlay
    // passive sources. The immutable package pair list still owns Z-A.
    auto jobActive = movingBody;
    jobActive.source = QStringLiteral("axis:X");
    lcnc::cam::TravelCollisionGeometryCache proofGeometry;
    proofGeometry.bodies = {machineActive, machinePassive, jobActive,
                            fixedBody};
    proofGeometry.machineAxisPairKeys.insert(
        lcnc::cam::collisionAxisPairKey(
            machineActive.source, machinePassive.source));
    if (!require(lcnc::cam::collisionPairProofOwner(proofGeometry, 0, 1)
                     == lcnc::cam::CollisionPairProofOwner::MachinePackage,
                 "Machine-only pair was not assigned to the package")) {
        return 29;
    }
    if (!require(lcnc::cam::collisionPairProofOwner(proofGeometry, 2, 3)
                     == lcnc::cam::CollisionPairProofOwner::JobOverlay,
                 "Machine/workpiece pair was not assigned to the overlay")) {
        return 30;
    }
    lcnc::cam::TravelCollisionBody mountedPassive;
    mountedPassive.source = QStringLiteral("axis:C");
    mountedPassive.passive = true;
    proofGeometry.bodies.append(mountedPassive);
    if (!require(lcnc::cam::collisionPairProofOwner(proofGeometry, 1, 4)
                     == lcnc::cam::CollisionPairProofOwner::None,
                 "Passive A-C chain was incorrectly scheduled for collision")) {
        return 31;
    }
    if (!require(lcnc::cam::collisionPairProofOwner(proofGeometry, 4, 3)
                     == lcnc::cam::CollisionPairProofOwner::None,
                 "Mounted C-workpiece pair was incorrectly scheduled for collision")) {
        return 32;
    }
    const auto machineOnlyPairs = lcnc::cam::collisionProofPairs(
        proofGeometry, true);
    const auto fullEnvironmentPairs = lcnc::cam::collisionProofPairs(
        proofGeometry, false);
    if (!require(machineOnlyPairs.size() == 1
                     && machineOnlyPairs.constFirst() == qMakePair(0, 1),
                 "Machine-only fallback did not select the package pair")) {
        return 33;
    }
    if (!require(fullEnvironmentPairs.size() == 3,
                 "Full-environment fallback lost the package or Job Overlay pair")) {
        return 34;
    }
    return 0;
}
