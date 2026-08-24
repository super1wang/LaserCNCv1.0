#include "core/algorithms/cam/laser_toolpath.h"
#include "core/algorithms/cam/collision_scan_policy.h"
#include "core/algorithms/cam/machine_safety_index.h"
#include "core/algorithms/cam/travel_path_planner.h"
#include "core/kinematics/machine_configuration_service.h"
#include "core/kinematics/machine_kinematics.h"
#include "core/machine/machine_safety_package.h"
#include "modules/cam/collision/collision_geometry_cache.h"
#include "modules/cam/collision/continuous_motion_certificate_builder.h"
#include "modules/cam/contracts/toolpath_export_dto.h"

#include <IFSelect_ReturnStatus.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <STEPControl_Reader.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDataStd_Name.hxx>
#include <TDocStd_Document.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS_Shape.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_Location.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <gp_Dir.hxx>
#include <gp_Trsf.hxx>

#include <QCoreApplication>
#include <QCryptographicHash>
#include <QElapsedTimer>
#include <QFile>
#include <QFileInfo>
#include <QSet>
#include <QTemporaryDir>
#include <QTextStream>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

namespace {

constexpr qint64 kMaximumCertificateElapsedMs = 30'000;

struct MachineBodySource
{
    QString name;
    QString axis;
    TopoDS_Shape shape;
};

int fail(const QString& message)
{
    QTextStream(stderr) << message << '\n';
    return 1;
}

void reportStage(const QString& name, qint64 elapsedMs)
{
    QTextStream output(stdout);
    output << "stage=" << name << " elapsed_ms=" << elapsedMs << '\n';
    output.flush();
}

QString optionValue(const QStringList& arguments, const QString& option)
{
    const int index = arguments.indexOf(option);
    return index >= 0 && index + 1 < arguments.size()
        ? arguments.at(index + 1) : QString{};
}

QString labelName(const TDF_Label& label)
{
    Handle(TDataStd_Name) attribute;
    if (!label.FindAttribute(TDataStd_Name::GetID(), attribute))
        return {};
    return QString::fromStdU16String(
        reinterpret_cast<const char16_t*>(attribute->Get().ToExtString()));
}

QString axisFromName(const QString& name)
{
    const QString upper = name.trimmed().toUpper();
    const QString prefix = QStringLiteral("LCNC_AXIS_");
    if (!upper.startsWith(prefix))
        return {};
    const QString axis = upper.mid(prefix.size());
    static const QSet<QString> supported{
        QStringLiteral("BASE"), QStringLiteral("X"), QStringLiteral("Y"),
        QStringLiteral("Z"), QStringLiteral("A"), QStringLiteral("B"),
        QStringLiteral("C")};
    return supported.contains(axis) ? axis : QString{};
}

void appendMachineBody(const QString& fallbackName,
                       const TDF_Label& label,
                       const Handle(XCAFDoc_ShapeTool)& shapes,
                       const TopLoc_Location& inheritedLocation,
                       std::vector<MachineBodySource>* bodies)
{
    QString name = labelName(label);
    TDF_Label referred;
    TopoDS_Shape shape;
    if (shapes->GetReferredShape(label, referred)) {
        if (name.isEmpty())
            name = labelName(referred);
        shape = shapes->GetShape(referred);
    } else {
        shape = shapes->GetShape(label);
    }
    if (shape.IsNull())
        return;
    TopLoc_Location location = inheritedLocation;
    Handle(XCAFDoc_Location) locationAttribute;
    if (label.FindAttribute(XCAFDoc_Location::GetID(), locationAttribute))
        location = locationAttribute->Get() * location;
    if (!location.IsIdentity())
        shape = shape.Located(location * shape.Location());
    if (name.isEmpty())
        name = fallbackName;
    const QString axis = axisFromName(name);
    if (!axis.isEmpty())
        bodies->push_back({name, axis, shape});
}

bool loadMachineBodies(const QString& path,
                       std::vector<MachineBodySource>* bodies,
                       QString* error)
{
    Handle(TDocStd_Document) document =
        new TDocStd_Document(TCollection_ExtendedString("BinXCAF"));
    XCAFDoc_DocumentTool::Set(document->Main());
    STEPCAFControl_Reader reader;
    reader.SetNameMode(Standard_True);
    if (reader.ReadFile(path.toUtf8().constData()) != IFSelect_RetDone
        || !reader.Transfer(document)) {
        *error = QStringLiteral("Cannot read the AC-table STEP model");
        return false;
    }
    const Handle(XCAFDoc_ShapeTool) shapes =
        XCAFDoc_DocumentTool::ShapeTool(document->Main());
    TDF_LabelSequence roots;
    shapes->GetFreeShapes(roots);
    for (int rootIndex = 1; rootIndex <= roots.Length(); ++rootIndex) {
        const TDF_Label root = roots.Value(rootIndex);
        TDF_LabelSequence components;
        shapes->GetComponents(root, components);
        if (components.IsEmpty()) {
            appendMachineBody(QStringLiteral("Part_%1").arg(rootIndex),
                              root, shapes, {}, bodies);
            continue;
        }
        for (int componentIndex = 1;
             componentIndex <= components.Length(); ++componentIndex) {
            appendMachineBody(
                QStringLiteral("Part_%1_%2")
                    .arg(rootIndex).arg(componentIndex),
                components.Value(componentIndex), shapes, {}, bodies);
        }
    }
    if (bodies->empty()) {
        *error = QStringLiteral("The machine STEP contains no LCNC_AXIS_* body");
        return false;
    }
    return true;
}

bool readWorkpiece(const QString& path, TopoDS_Shape* workpiece,
                   QString* error)
{
    STEPControl_Reader reader;
    if (reader.ReadFile(path.toUtf8().constData()) != IFSelect_RetDone
        || reader.TransferRoots() == 0) {
        *error = QStringLiteral("Cannot read the half-sphere STEP model");
        return false;
    }
    *workpiece = reader.OneShape();
    if (workpiece->IsNull()) {
        *error = QStringLiteral("The half-sphere STEP has no usable shape");
        return false;
    }
    return true;
}

bool buildRealToolpathSnapshot(
    const TopoDS_Shape& workpiece,
    const lcnc::MachineModeDefinition& definition,
    lcnc::cam::ToolpathExportSnapshot* snapshot,
    int* contourCount, QString* error)
{
    QString faceInfo;
    const std::vector<TopoDS_Face> faces =
        LaserToolpathBuilder::selectTopVisibleFacesFromPositiveZ(
            workpiece, &faceInfo);
    if (faces.empty()) {
        *error = QStringLiteral("No machinable half-sphere face: %1")
                     .arg(faceInfo);
        return false;
    }
    ContourExtractionParams parameters;
    parameters.strategy = ExtractionStrategy::ManualFaceSelection;
    parameters.selectedMachiningFaces = faces;
    // Match the production default, and deliberately retain thousands of
    // edges so the 30-second gate cannot pass on a toy path.
    parameters.deflection = 0.10;
    auto contours = LaserToolpathBuilder::extractContoursFromFaces(
        workpiece, faces, gp_Dir(0.0, 0.0, -1.0), parameters);
    std::vector<LaserContour*> ordered;
    for (LaserContour& contour : contours) {
        contour.sourceShape = workpiece;
        contour.workpieceEntry = QStringLiteral("workpiece");
        LaserToolpathBuilder::discretizeContour(
            contour, workpiece, parameters.deflection);
        if (contour.points.size() >= 2)
            ordered.push_back(&contour);
    }
    if (ordered.empty()) {
        *error = QStringLiteral("The half-sphere produced no sampled contour");
        return false;
    }

    MachineKinematics kinematics;
    kinematics.loadPreset(QStringLiteral("VERTICAL_AC_TABLE"));
    kinematics.mountWorkpiece(QStringLiteral("workpiece"),
                              QStringLiteral("C"));
    QString solveError;
    if (!LaserToolpathBuilder::solveToolpathForOrder(
            ordered, &kinematics, gp_Trsf{}, definition,
            {}, {}, &solveError)) {
        *error = QStringLiteral("Half-sphere five-axis solve failed: %1")
                     .arg(solveError);
        return false;
    }
    MachineKinematics projection;
    projection.loadPreset(QStringLiteral("VERTICAL_AC_TABLE"));
    const auto projectionAxes = lcnc::cam_algo::offlinePlanningAxisBaseline(
        projection.axes(), definition);
    projection.setAxes(projectionAxes, projection.configType());
    projection.mountWorkpiece(QStringLiteral("workpiece"),
                              QStringLiteral("C"));

    snapshot->revision = 1;
    snapshot->machiningMode = lcnc::MachiningMode::SimultaneousTable5Axis;
    snapshot->machineAxisLayout = definition.interpolatedAxes;
    snapshot->collisionSafety.enabled = true;
    snapshot->collisionSafety.machinePackageRequired = true;
    snapshot->collisionSafety.machinePackageReady = true;
    snapshot->collisionSafety.jobOverlayRequired = true;
    snapshot->collisionSafety.jobOverlayReady = true;
    snapshot->motionPlan.revision = 1;
    snapshot->travelPlan.key.environmentRevision = 1;
    snapshot->travelPlan.key.motionProfileHash = 1;

    std::uint64_t nextContourId = 1;
    for (const LaserContour* contour : ordered) {
        lcnc::cam::ToolpathExportContour exported;
        exported.contourId = nextContourId;
        exported.workpieceEntry = QStringLiteral("workpiece");
        exported.enabled = true;
        exported.layerEnabled = true;
        snapshot->contours.append(exported);
        for (const ToolpathPoint& point : contour->points) {
            if (!point.machineCoord.valid)
                continue;
            lcnc::cam::CamMotionNode node;
            node.phase = lcnc::cam::CamMotionPhase::Cutting;
            node.contourId = nextContourId;
            constexpr double kProductionCuttingOffsetMm = 1.0;
            node.tcpX = point.position.X()
                + point.normal.X() * kProductionCuttingOffsetMm;
            node.tcpY = point.position.Y()
                + point.normal.Y() * kProductionCuttingOffsetMm;
            node.tcpZ = point.position.Z()
                + point.normal.Z() * kProductionCuttingOffsetMm;
            node.normalX = point.normal.X();
            node.normalY = point.normal.Y();
            node.normalZ = point.normal.Z();
            const int axisCount = std::min(
                static_cast<int>(definition.interpolatedAxes.count),
                lcnc::MachineAxisLayout::kMaxAxes);
            for (int axis = 0; axis < axisCount; ++axis)
                node.axes[axis] = point.machineCoord.solvedPose.value(axis);
            node.axisMask = static_cast<std::uint8_t>(
                axisCount >= 8 ? 0xffu : ((1u << axisCount) - 1u));
            lcnc::cam_algo::applyOfflineMotionPose(
                &projection, projectionAxes,
                definition.interpolatedAxes, node.axes, node.axisMask);
            lcnc::cam_algo::transformMotionNodeGeometry(
                &node, projection.computeWpcTransform(
                           QStringLiteral("workpiece")));
            snapshot->motionPlan.nodes.append(node);
        }
        ++nextContourId;
    }
    *contourCount = static_cast<int>(ordered.size());
    if (snapshot->motionPlan.nodes.size() < 2) {
        *error = QStringLiteral("The half-sphere path has fewer than two nodes");
        return false;
    }
    return true;
}

} // namespace

int main(int argc, char** argv)
{
    QCoreApplication application(argc, argv);
    QString indexPath = optionValue(
        application.arguments(), QStringLiteral("--index"));
    QString machinePath = QString::fromUtf8(LCNC_AC_TABLE_MODEL_PATH);
    QTemporaryDir extractedPackage;
    const QString packagePath = optionValue(
        application.arguments(), QStringLiteral("--package"));
    if (!packagePath.isEmpty()) {
        if (!extractedPackage.isValid())
            return fail(QStringLiteral("Cannot create package extraction directory"));
        lcnc::MachineSafetyPackageLoadResult package;
        QString packageError;
        if (!lcnc::MachineSafetyPackage::extractAndValidate(
                packagePath, extractedPackage.path(), &package,
                &packageError)) {
            return fail(QStringLiteral("Cannot validate LMSP: %1")
                            .arg(packageError));
        }
        indexPath = package.safetyIndexPath;
        machinePath = package.modelPath;
    }
    if (indexPath.isEmpty()) {
        return fail(QStringLiteral(
            "Usage: --package <machine.lmsp> or --index <machine.lmsi>"));
    }

    const QString workpiecePath = QString::fromUtf8(LCNC_HALF_SPHERE_MODEL_PATH);
    QString error;
    auto index = std::make_shared<lcnc::cam_algo::MachineSafetyIndex>();
    QElapsedTimer indexTimer;
    indexTimer.start();
    if (!index->load(indexPath, &error))
        return fail(QStringLiteral("Cannot load LMSI: %1").arg(error));
    const qint64 indexLoadMs = indexTimer.elapsed();
    reportStage(QStringLiteral("index_load"), indexLoadMs);
    QFile machineFile(machinePath);
    if (!machineFile.open(QIODevice::ReadOnly)
        || QCryptographicHash::hash(machineFile.readAll(), QCryptographicHash::Sha256)
            != index->sourceSha256()) {
        return fail(QStringLiteral("LMSI and machine STEP fingerprints differ"));
    }

    std::vector<MachineBodySource> machineBodies;
    QElapsedTimer machineTimer;
    machineTimer.start();
    if (!loadMachineBodies(machinePath, &machineBodies, &error))
        return fail(error);
    const qint64 machineLoadMs = machineTimer.elapsed();
    reportStage(QStringLiteral("machine_load"), machineLoadMs);

    TopoDS_Shape workpiece;
    if (!readWorkpiece(workpiecePath, &workpiece, &error))
        return fail(error);
    lcnc::MachineConfigurationService configuration;
    configuration.applyPreset(QStringLiteral("VERTICAL_AC_TABLE"));
    const auto definition = configuration.modeDefinition(
        lcnc::MachiningMode::SimultaneousTable5Axis);
    MachineKinematics baseline;
    baseline.loadPreset(QStringLiteral("VERTICAL_AC_TABLE"));

    auto geometry =
        std::make_shared<lcnc::cam::TravelCollisionGeometryCache>();
    geometry->axes = lcnc::cam_algo::offlinePlanningAxisBaseline(
        baseline.axes(), definition);
    geometry->configType = baseline.configType();
    geometry->definition = definition;
    geometry->mounts.insert(QStringLiteral("workpiece"), QStringLiteral("C"));
    geometry->key.environmentRevision = 1;
    geometry->machineAxisPairKeys =
        lcnc::cam::machineCollisionAxisPairKeys(*index);
    int persistedOrdinal = 0;
    QHash<QString, int> axisOrdinal;
    QElapsedTimer geometryTimer;
    geometryTimer.start();
    for (const MachineBodySource& source : machineBodies) {
        lcnc::cam::TravelCollisionBody body;
        body.entry = QStringLiteral("machine:%1").arg(persistedOrdinal++);
        body.source = lcnc::cam::collisionAxisSourceId(source.axis);
        const QString normalizedAxis = source.axis.trimmed().toUpper();
        // The complete head/nozzle geometry belongs to the Z-axis machine
        // body.  A/C/BASE form the workpiece's rigid mount chain; only the
        // remaining machine bodies can collide with that workpiece.
        body.active = normalizedAxis != QStringLiteral("A")
            && normalizedAxis != QStringLiteral("C")
            && normalizedAxis != QStringLiteral("BASE");
        body.sourceShape = source.shape;
        geometry->assignments.insert(body.entry, source.axis);
        const int requestedOrdinal = axisOrdinal.value(source.axis);
        int matchingOrdinal = 0;
        for (int persistedIndex = 0;
             persistedIndex < index->bodies().size(); ++persistedIndex) {
            if (index->bodies().at(persistedIndex).axisName.compare(
                    source.axis, Qt::CaseInsensitive) != 0) {
                continue;
            }
            if (matchingOrdinal++ != requestedOrdinal)
                continue;
            if (const auto* persisted =
                    index->persistedSurfaceModel(persistedIndex)) {
                body.surfaceModel = *persisted;
            }
            break;
        }
        axisOrdinal[source.axis] = requestedOrdinal + 1;
        lcnc::cam::buildCollisionGeometry(&body, 0.05);
        if (body.leaves.isEmpty())
            return fail(QStringLiteral("Machine collision body has no leaves"));
        geometry->bodies.append(std::move(body));
    }

    lcnc::cam::TravelCollisionBody workpieceBody;
    workpieceBody.entry = QStringLiteral("__workpiece__");
    workpieceBody.source = QStringLiteral("workpiece");
    workpieceBody.passive = true;
    workpieceBody.workpiece = true;
    workpieceBody.sourceShape = workpiece;
    lcnc::cam::buildCollisionGeometry(&workpieceBody, 0.005);
    geometry->bodies.append(std::move(workpieceBody));
    lcnc::cam::buildCoalCollisionPairs(geometry.get());
    const qint64 geometryBuildMs = geometryTimer.elapsed();
    reportStage(QStringLiteral("geometry_build"), geometryBuildMs);

    lcnc::cam::ToolpathExportSnapshot snapshot;
    int contourCount = 0;
    QElapsedTimer pathTimer;
    pathTimer.start();
    if (!buildRealToolpathSnapshot(
            workpiece, definition, &snapshot, &contourCount, &error)) {
        return fail(error);
    }
    const qint64 pathBuildMs = pathTimer.elapsed();
    reportStage(QStringLiteral("path_build"), pathBuildMs);

    const auto rapidPoseForNode = [&snapshot](
                                      const lcnc::cam::CamMotionNode& node) {
        lcnc::cam::RapidPose pose;
        pose.kinematicAxes = node.axes;
        pose.kinematicAxisMask = node.axisMask;
        int rotarySlot = 0;
        for (int index = 0; index < snapshot.machineAxisLayout.count; ++index) {
            if (!(node.axisMask & (1u << index)))
                continue;
            const double value = node.axes[index];
            switch (snapshot.machineAxisLayout.axes[index].role) {
            case lcnc::MachineAxisRole::LinearX:
                pose.axes[0] = value; pose.activeMask |= 0x01u; break;
            case lcnc::MachineAxisRole::LinearY:
                pose.axes[1] = value; pose.activeMask |= 0x02u; break;
            case lcnc::MachineAxisRole::LinearZ:
                pose.axes[2] = value; pose.activeMask |= 0x04u; break;
            default:
                if (rotarySlot == 0) {
                    pose.axes[3] = value; pose.activeMask |= 0x08u;
                } else if (rotarySlot == 1) {
                    pose.axes[4] = value; pose.activeMask |= 0x10u;
                }
                ++rotarySlot;
                break;
            }
        }
        pose.tcpX = node.tcpX;
        pose.tcpY = node.tcpY;
        pose.tcpZ = node.tcpZ;
        pose.surfaceNormalX = node.normalX;
        pose.surfaceNormalY = node.normalY;
        pose.surfaceNormalZ = node.normalZ;
        return pose;
    };
    lcnc::cam_algo::TravelPlanningRequest rapidRequest;
    rapidRequest.workpiece = workpiece;
    rapidRequest.fullEnvironment = true;
    rapidRequest.minimumClearanceMm = 0.5;
    rapidRequest.maximumSafetyOffsetMm = 100.0;
    rapidRequest.surfacePathStepMm = 0.5;
    rapidRequest.motionProfile.supportedCoordinatedMask = 0x1fu;
    rapidRequest.motionProfile.velocity.fill(100.0);
    rapidRequest.motionProfile.acceleration.fill(1000.0);
    rapidRequest.motionProfile.jerk.fill(10000.0);
    QVector<QPair<int, int>> contourNodeRanges;
    int rangeFirst = 0;
    while (rangeFirst < snapshot.motionPlan.nodes.size()) {
        const std::uint64_t contourId =
            snapshot.motionPlan.nodes.at(rangeFirst).contourId;
        int rangeAfterLast = rangeFirst + 1;
        while (rangeAfterLast < snapshot.motionPlan.nodes.size()
               && snapshot.motionPlan.nodes.at(rangeAfterLast).contourId
                   == contourId) {
            ++rangeAfterLast;
        }
        contourNodeRanges.append({rangeFirst, rangeAfterLast});
        rangeFirst = rangeAfterLast;
    }
    for (int range = 1; range < contourNodeRanges.size(); ++range) {
        const auto previous = contourNodeRanges.at(range - 1);
        const auto next = contourNodeRanges.at(range);
        lcnc::cam_algo::TravelEndpoint source;
        source.contourId = snapshot.motionPlan.nodes.at(
            previous.second - 1).contourId;
        source.pose = rapidPoseForNode(
            snapshot.motionPlan.nodes.at(previous.second - 1));
        lcnc::cam_algo::TravelEndpoint target;
        target.contourId = snapshot.motionPlan.nodes.at(next.first).contourId;
        target.pose = rapidPoseForNode(
            snapshot.motionPlan.nodes.at(next.first));
        rapidRequest.transitions.append(
            {source, target, 5.0, 1.0, 1.0});
    }
    QElapsedTimer rapidPlanTimer;
    rapidPlanTimer.start();
    const auto rapidPlan =
        lcnc::cam_algo::TravelPathPlanner::plan(rapidRequest);
    const qint64 rapidPlanMs = rapidPlanTimer.elapsed();
    reportStage(QStringLiteral("rapid_plan"), rapidPlanMs);
    if (!rapidPlan.isExecutable()
        || rapidPlan.transitions.size() != rapidRequest.transitions.size()) {
        return fail(QStringLiteral("Real rapid plan failed: %1")
                        .arg(rapidPlan.failureReason));
    }
    lcnc::cam::ToolpathExportSnapshot productionSnapshot = snapshot;
    productionSnapshot.motionPlan.nodes.clear();
    for (int range = 0; range < contourNodeRanges.size(); ++range) {
        if (range > 0) {
            const auto& transition = rapidPlan.transitions.at(range - 1);
            for (const auto& segment : transition.segments) {
                lcnc::cam::CamMotionNode node;
                node.phase = lcnc::cam::CamMotionPhase::Rapid;
                node.contourId = transition.toContourId;
                node.axes = segment.target.kinematicAxes;
                node.axisMask = segment.target.kinematicAxisMask;
                node.tcpX = segment.target.tcpX;
                node.tcpY = segment.target.tcpY;
                node.tcpZ = segment.target.tcpZ;
                node.normalX = segment.target.surfaceNormalX;
                node.normalY = segment.target.surfaceNormalY;
                node.normalZ = segment.target.surfaceNormalZ;
                productionSnapshot.motionPlan.nodes.append(node);
            }
        }
        const auto nodeRange = contourNodeRanges.at(range);
        productionSnapshot.motionPlan.nodes += snapshot.motionPlan.nodes.mid(
            nodeRange.first, nodeRange.second - nodeRange.first);
    }
    int pointCertifiedSafe = 0;
    int pointCollision = 0;
    int pointUnknown = 0;
    int pointOutOfRange = 0;
    for (const auto& node : std::as_const(productionSnapshot.motionPlan.nodes)) {
        lcnc::cam_algo::MachineSafetyPose pose;
        pose.count = static_cast<std::uint8_t>(index->axes().size());
        for (int axis = 0; axis < index->axes().size(); ++axis) {
            const QString& axisName = index->axes().at(axis).name;
            const int layoutIndex = productionSnapshot.machineAxisLayout.indexOfName(
                axisName);
            if (layoutIndex >= 0 && (node.axisMask & (1u << layoutIndex))) {
                pose.values[axis] = node.axes[layoutIndex];
                const auto& grid = index->axes().at(axis);
                if (grid.rotary
                    && grid.maximum - grid.minimum >= 359.0) {
                    const double canonicalCenter =
                        0.5 * (grid.minimum + grid.maximum);
                    pose.values[axis] -= 360.0 * std::round(
                        (pose.values[axis] - canonicalCenter) / 360.0);
                }
            }
        }
        const auto query = index->query(pose);
        if (!query.inRange)
            ++pointOutOfRange;
        if (query.state == lcnc::cam_algo::MachineSafetyIndexState::CertifiedSafe)
            ++pointCertifiedSafe;
        else if (query.state
                 == lcnc::cam_algo::MachineSafetyIndexState::CollisionSample)
            ++pointCollision;
        else
            ++pointUnknown;
    }
    const bool splitDiagnostics = application.arguments().contains(
        QStringLiteral("--split-diagnostics"));

    lcnc::cam::ContinuousMotionCertificateBuildContext context;
    context.machineIndex = index;
    // Standalone .lmsi input has no .lmsp manifest. Supply a stable non-empty
    // policy identity so the production certificate contract exercises the
    // index instead of correctly rejecting an incomplete runtime key.
    context.runtimeConfigurationSha256 = QCryptographicHash::hash(
        QByteArrayLiteral("production-benchmark-machine-policy-v2"),
        QCryptographicHash::Sha256);
    context.geometry = geometry;
    context.clearanceMm = 0.5;
    context.maximumParallelWorkers = 8;
    context.maximumParallelCoalQueries = 1;
    context.maximumCoalQueryWallTimeMs = 50;
    QElapsedTimer certificateTimer;
    certificateTimer.start();
    QVector<lcnc::cam::CamMotionEdgeCertificate> certificates;
    int firstNode = 0;
    while (splitDiagnostics
           && firstNode < snapshot.motionPlan.nodes.size()) {
        const std::uint64_t contourId =
            snapshot.motionPlan.nodes.at(firstNode).contourId;
        int afterLastNode = firstNode + 1;
        while (afterLastNode < snapshot.motionPlan.nodes.size()
               && snapshot.motionPlan.nodes.at(afterLastNode).contourId
                   == contourId) {
            ++afterLastNode;
        }
        if (afterLastNode - firstNode >= 2) {
            auto contourSnapshot = snapshot;
            contourSnapshot.motionPlan.nodes = snapshot.motionPlan.nodes.mid(
                firstNode, afterLastNode - firstNode);
            const auto contourCertificates =
                lcnc::cam::buildContinuousMotionCertificates(
                    contourSnapshot, context,
                    [&certificateTimer] {
                        return certificateTimer.elapsed()
                            > kMaximumCertificateElapsedMs + 5'000;
                    });
            certificates += contourCertificates;
        }
        firstNode = afterLastNode;
        if (certificateTimer.elapsed()
            > kMaximumCertificateElapsedMs + 5'000) {
            break;
        }
    }
    if (!splitDiagnostics) {
        certificates = lcnc::cam::buildContinuousMotionCertificates(
            productionSnapshot, context,
            [&certificateTimer] {
                return certificateTimer.elapsed()
                    > kMaximumCertificateElapsedMs + 5'000;
            });
    }
    const qint64 certificateMs = certificateTimer.elapsed();
    reportStage(QStringLiteral("certificate_build"), certificateMs);

    QElapsedTimer rapidCertificateTimer;
    rapidCertificateTimer.start();
    QVector<lcnc::cam::CamMotionEdgeCertificate> rapidCertificates;
    for (const auto& transition : rapidPlan.transitions) {
        if (!splitDiagnostics)
            break;
        lcnc::cam::ToolpathExportSnapshot rapidSnapshot = snapshot;
        rapidSnapshot.motionPlan.nodes.clear();
        const auto sourceRange = std::find_if(
            contourNodeRanges.cbegin(), contourNodeRanges.cend(),
            [&snapshot, &transition](const QPair<int, int>& range) {
                return snapshot.motionPlan.nodes.at(range.first).contourId
                    == transition.fromContourId;
            });
        if (sourceRange == contourNodeRanges.cend())
            return fail(QStringLiteral("Rapid transition source contour is missing"));
        lcnc::cam::CamMotionNode sourceNode =
            snapshot.motionPlan.nodes.at(sourceRange->second - 1);
        rapidSnapshot.motionPlan.nodes.append(sourceNode);
        for (const auto& segment : transition.segments) {
            lcnc::cam::CamMotionNode node;
            node.phase = lcnc::cam::CamMotionPhase::Rapid;
            node.contourId = transition.toContourId;
            node.axes = segment.target.kinematicAxes;
            node.axisMask = segment.target.kinematicAxisMask;
            node.tcpX = segment.target.tcpX;
            node.tcpY = segment.target.tcpY;
            node.tcpZ = segment.target.tcpZ;
            node.normalX = segment.target.surfaceNormalX;
            node.normalY = segment.target.surfaceNormalY;
            node.normalZ = segment.target.surfaceNormalZ;
            rapidSnapshot.motionPlan.nodes.append(node);
        }
        const auto transitionCertificates =
            lcnc::cam::buildContinuousMotionCertificates(
                rapidSnapshot, context,
                [&rapidCertificateTimer] {
                    return rapidCertificateTimer.elapsed()
                        > kMaximumCertificateElapsedMs + 5'000;
                });
        rapidCertificates += transitionCertificates;
        if (rapidCertificateTimer.elapsed()
            > kMaximumCertificateElapsedMs + 5'000) {
            break;
        }
    }
    const qint64 rapidCertificateMs = rapidCertificateTimer.elapsed();
    reportStage(QStringLiteral("rapid_certificate_build"),
                rapidCertificateMs);

    std::uint64_t intervals = 0;
    std::uint64_t broadPhaseRejected = 0;
    std::uint64_t localFieldQueries = 0;
    std::uint64_t localFieldRejected = 0;
    std::uint64_t surfaceQueries = 0;
    std::uint64_t coalQueries = 0;
    std::uint64_t exactQueries = 0;
    std::uint64_t surfaceNs = 0;
    std::uint64_t localFieldNs = 0;
    std::uint64_t coalNs = 0;
    std::uint64_t exactNs = 0;
    std::uint64_t lockWaitNs = 0;
    std::uint64_t maximumExactNs = 0;
    QString slowestExactPair;
    int safe = 0;
    int blocked = 0;
    int unknown = 0;
    int maximumDepth = 0;
    QHash<QString, int> unknownPairs;
    QVector<int> unknownEdgeIndices;
    for (int certificateIndex = 0;
         certificateIndex < certificates.size(); ++certificateIndex) {
        const auto& certificate = certificates.at(certificateIndex);
        intervals += certificate.intervalQueries;
        broadPhaseRejected += certificate.broadPhaseRejected;
        localFieldQueries += certificate.localFieldQueries;
        localFieldRejected += certificate.localFieldRejected;
        surfaceQueries += certificate.surfaceBvhQueries;
        coalQueries += certificate.coalPairQueries;
        exactQueries += certificate.occtExactQueries;
        surfaceNs += certificate.surfaceBvhQueryNs;
        localFieldNs += certificate.localFieldQueryNs;
        coalNs += certificate.coalQueryNs;
        exactNs += certificate.occtExactQueryNs;
        lockWaitNs += certificate.occtExactLockWaitNs;
        if (certificate.occtExactMaximumQueryNs > maximumExactNs) {
            maximumExactNs = certificate.occtExactMaximumQueryNs;
            slowestExactPair = certificate.occtExactSlowestPair;
        }
        if (certificate.state
            == lcnc::cam::CamMotionCertificateState::CertifiedSafe) {
            ++safe;
        } else if (certificate.state
                   == lcnc::cam::CamMotionCertificateState::Blocked) {
            ++blocked;
        } else {
            ++unknown;
            unknownEdgeIndices.append(certificateIndex);
            ++unknownPairs[certificate.fallbackSourcePair];
        }
        maximumDepth = std::max(
            maximumDepth, certificate.maximumSubdivisionDepth);
    }
    int rapidSafe = 0;
    int rapidUnknown = 0;
    int rapidBlocked = 0;
    int rapidMaximumDepth = 0;
    std::uint64_t rapidIntervals = 0;
    std::uint64_t rapidSurfaceQueries = 0;
    std::uint64_t rapidExactQueries = 0;
    QHash<QString, int> rapidUnknownPairs;
    for (const auto& certificate : rapidCertificates) {
        rapidIntervals += certificate.intervalQueries;
        rapidSurfaceQueries += certificate.surfaceBvhQueries;
        rapidExactQueries += certificate.occtExactQueries;
        rapidMaximumDepth = std::max(
            rapidMaximumDepth, certificate.maximumSubdivisionDepth);
        if (certificate.state
            == lcnc::cam::CamMotionCertificateState::CertifiedSafe) {
            ++rapidSafe;
        } else if (certificate.state
                   == lcnc::cam::CamMotionCertificateState::Blocked) {
            ++rapidBlocked;
        } else {
            ++rapidUnknown;
            ++rapidUnknownPairs[certificate.fallbackSourcePair];
        }
    }
    QTextStream out(stdout);
    out << "machine_bodies=" << machineBodies.size() << '\n'
        << "index_load_ms=" << indexLoadMs << '\n'
        << "machine_load_ms=" << machineLoadMs << '\n'
        << "geometry_build_ms=" << geometryBuildMs << '\n'
        << "coal_pairs=" << geometry->coalPairs.size() << '\n'
        << "real_contours=" << contourCount << '\n'
        << "real_nodes=" << snapshot.motionPlan.nodes.size() << '\n'
        << "production_nodes="
        << productionSnapshot.motionPlan.nodes.size() << '\n'
        << "point_certified_safe=" << pointCertifiedSafe << '\n'
        << "point_collision=" << pointCollision << '\n'
        << "point_boundary_unknown=" << pointUnknown << '\n'
        << "point_out_of_range=" << pointOutOfRange << '\n'
        << "real_edges=" << certificates.size() << '\n'
        << "path_build_ms=" << pathBuildMs << '\n'
        << "rapid_plan_ms=" << rapidPlanMs << '\n'
        << "certificate_ms=" << certificateMs << '\n'
        << "rapid_edges=" << rapidCertificates.size() << '\n'
        << "rapid_certificate_ms=" << rapidCertificateMs << '\n'
        << "rapid_certified_safe=" << rapidSafe << '\n'
        << "rapid_blocked=" << rapidBlocked << '\n'
        << "rapid_boundary_unknown=" << rapidUnknown << '\n'
        << "rapid_maximum_depth=" << rapidMaximumDepth << '\n'
        << "rapid_intervals=" << rapidIntervals << '\n'
        << "rapid_surface_queries=" << rapidSurfaceQueries << '\n'
        << "rapid_occt_exact_queries=" << rapidExactQueries << '\n'
        << "certified_safe=" << safe << '\n'
        << "blocked=" << blocked << '\n'
        << "boundary_unknown=" << unknown << '\n'
        << "maximum_depth=" << maximumDepth << '\n'
        << "intervals=" << intervals << '\n'
        << "broad_phase_rejected=" << broadPhaseRejected << '\n'
        << "local_field_queries=" << localFieldQueries << '\n'
        << "local_field_rejected=" << localFieldRejected << '\n'
        << "local_field_cpu_ms=" << (localFieldNs / 1'000'000.0) << '\n'
        << "surface_queries=" << surfaceQueries << '\n'
        << "surface_cpu_ms=" << (surfaceNs / 1'000'000.0) << '\n'
        << "coal_queries=" << coalQueries << '\n'
        << "coal_cpu_ms=" << (coalNs / 1'000'000.0) << '\n'
        << "occt_exact_queries=" << exactQueries << '\n'
        << "occt_exact_cpu_ms=" << (exactNs / 1'000'000.0) << '\n'
        << "occt_lock_wait_ms=" << (lockWaitNs / 1'000'000.0) << '\n'
        << "occt_exact_max_ms=" << (maximumExactNs / 1'000'000.0) << '\n'
        << "occt_exact_slowest_pair=" << slowestExactPair << '\n';
    for (auto it = unknownPairs.cbegin(); it != unknownPairs.cend(); ++it)
        out << "unknown_pair[" << it.key() << "]=" << it.value() << '\n';
    for (const int edge : std::as_const(unknownEdgeIndices)) {
        if (edge + 1 >= productionSnapshot.motionPlan.nodes.size())
            continue;
        out << "unknown_edge[" << edge << "]=";
        const auto& first = productionSnapshot.motionPlan.nodes.at(edge);
        const auto& last = productionSnapshot.motionPlan.nodes.at(edge + 1);
        for (int axis = 0;
             axis < productionSnapshot.machineAxisLayout.count; ++axis) {
            if (axis > 0)
                out << ',';
            out << productionSnapshot.machineAxisLayout.axes[axis].name
                << ':' << first.axes[axis] << "->" << last.axes[axis];
        }
        out << '\n';
    }
    for (const auto& body : geometry->bodies) {
        out << "body[" << body.source << "].triangles="
            << body.surfaceModel.triangleCount() << '\n'
            << "body[" << body.source << "].sphere_cover="
            << body.sphereCover.size() << '\n'
            << "body[" << body.source << "].sphere_hierarchy="
            << body.sphereHierarchy.size() << '\n'
            << "body[" << body.source << "].local_field="
            << (body.localClearanceField
                    && body.localClearanceField->isValid() ? 1 : 0)
            << '\n';
    }
    for (auto it = rapidUnknownPairs.cbegin();
         it != rapidUnknownPairs.cend(); ++it) {
        out << "rapid_unknown_pair[" << it.key() << "]="
            << it.value() << '\n';
    }
    out.flush();

    const int expectedCertificates = splitDiagnostics
        ? snapshot.motionPlan.nodes.size() - contourCount
        : productionSnapshot.motionPlan.nodes.size() - 1;
    if (certificates.size() != expectedCertificates) {
        return fail(QStringLiteral("Certificates do not cover every production edge"));
    }
    if (certificateMs >= kMaximumCertificateElapsedMs) {
        return fail(QStringLiteral("Production certificate build exceeded 30 seconds: %1 ms")
                        .arg(certificateMs));
    }
    if (splitDiagnostics
        && certificateMs + rapidCertificateMs
        >= kMaximumCertificateElapsedMs) {
        return fail(QStringLiteral("Production cutting + rapid certificates exceeded 30 seconds: %1 ms")
                        .arg(certificateMs + rapidCertificateMs));
    }
    // This benchmark does not use an external oracle for every real edge.
    // Unknown remains fail-closed; the synthetic penetration tests separately
    // prove that the same builder never promotes a collision to safety.
    if (blocked != 0)
        return fail(QStringLiteral("LMSI reported a known collision on the real path"));
    return 0;
}
