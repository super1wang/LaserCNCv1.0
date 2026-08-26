#include "core/algorithms/cam/laser_toolpath.h"
#include "core/algorithms/cam/machine_motion_certificate.h"
#include "core/algorithms/cam/machine_safety_index.h"
#include "core/kinematics/machine_configuration_service.h"
#include "core/kinematics/machine_kinematics.h"

#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <NCollection_Sequence.hxx>
#include <Poly_Triangle.hxx>
#include <Poly_Triangulation.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <STEPControl_Reader.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TDF_Label.hxx>
#include <TDataStd_Name.hxx>
#include <TDocStd_Document.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_Location.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#if defined(LCNC_COLLISION_BACKEND_FCL)
#include <fcl/fcl.h>
#elif defined(LCNC_COLLISION_BACKEND_COAL)
#include <coal/BV/OBBRSS.h>
#include <coal/BVH/BVH_model.h>
#include <coal/collision.h>
#include <coal/distance.h>
#include <coal/math/transform.h>
#else
#error A collision backend must be selected
#endif

#include <QCoreApplication>
#include <QCryptographicHash>
#include <QElapsedTimer>
#include <QFile>
#include <QFileInfo>
#include <QRegularExpression>
#include <QSet>
#include <QTextStream>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kMeshDeflectionMm = 0.10;
constexpr double kRequiredClearanceMm = 0.50;
// This feasibility benchmark has no measured calibration/tracking allowance.
// Production use must add those errors; a result below this threshold remains
// BoundaryUnknown and therefore fail-closed.
constexpr double kCertificationThresholdMm =
    kRequiredClearanceMm + 2.0 * kMeshDeflectionMm;

#if defined(LCNC_COLLISION_BACKEND_FCL)
using BackendVector = fcl::Vector3d;
using BackendTriangle = fcl::Triangle;
using BackendTransform = fcl::Transform3d;
using BackendModel = fcl::BVHModel<fcl::OBBRSSd>;
constexpr const char* kBackendName = "FCL 0.7.0";
#else
using BackendVector = coal::Vec3s;
using BackendTriangle = coal::Triangle;
using BackendTransform = coal::Transform3s;
using BackendModel = coal::BVHModel<coal::OBBRSS>;
constexpr const char* kBackendName = "Coal 3.0.4";
#endif

struct PlainMesh
{
    std::vector<BackendVector> vertices;
    std::vector<BackendTriangle> triangles;
};

struct Body
{
    QString name;
    QString axis;
    TopoDS_Shape shape;
    lcnc::cam_algo::SurfaceCollisionModel surfaceModel;
    std::shared_ptr<BackendModel> model;
    std::size_t triangleCount{0};
    double meshDeflectionMm{kMeshDeflectionMm};
    std::vector<int> axisChain;
    Bnd_Box localBounds;
};

struct MotionEdgeSample
{
    lcnc::cam_algo::MachineSafetyPose first;
    lcnc::cam_algo::MachineSafetyPose last;
};

struct AuditSample
{
    int sample{0};
    bool exactCollision{false};
    bool exactNear{false};
    double exactDistanceMm{0.0};
    lcnc::cam_algo::MachineSafetyPose pose;
};

struct QueryResult
{
    bool blockedByThreshold{false};
    bool certifiedSafe{false};
    double minimumDistanceMm{std::numeric_limits<double>::infinity()};
    int evaluatedPairs{0};
};

struct PreparedPair
{
    std::size_t first{0};
    std::size_t second{0};
#if defined(LCNC_COLLISION_BACKEND_COAL)
    std::unique_ptr<coal::ComputeCollision> collision;
    coal::CollisionRequest request{coal::DISTANCE_LOWER_BOUND, 1};
#endif
};

struct QueryEngine
{
    const std::vector<Body>* bodies{nullptr};
    const QVector<lcnc::cam_algo::MachineSafetyAxisGrid>* axes{nullptr};
    std::vector<PreparedPair> pairs;
    std::vector<BackendTransform> transforms;
    std::vector<gp_Trsf> occtTransforms;
};

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

void appendBody(const QString& fallbackName,
                const TDF_Label& label,
                const Handle(XCAFDoc_ShapeTool)& shapes,
                const TopLoc_Location& inheritedLocation,
                std::vector<Body>* bodies)
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
    if (!axis.isEmpty()) {
        Body body;
        body.name = name;
        body.axis = axis;
        body.shape = shape;
        bodies->push_back(std::move(body));
    }
}

bool loadMachineBodies(const QString& path, std::vector<Body>* bodies,
                       QString* error)
{
    Handle(TDocStd_Document) document =
        new TDocStd_Document(TCollection_ExtendedString("BinXCAF"));
    XCAFDoc_DocumentTool::Set(document->Main());
    STEPCAFControl_Reader reader;
    reader.SetNameMode(true);
    if (reader.ReadFile(path.toUtf8().constData()) != IFSelect_RetDone
        || !reader.Transfer(document)) {
        *error = QStringLiteral("无法读取或转换 STEP 机台文件");
        return false;
    }
    const Handle(XCAFDoc_ShapeTool) shapes =
        XCAFDoc_DocumentTool::ShapeTool(document->Main());
    NCollection_Sequence<TDF_Label> roots;
    shapes->GetFreeShapes(roots);
    for (int rootIndex = 1; rootIndex <= roots.Length(); ++rootIndex) {
        const TDF_Label root = roots.Value(rootIndex);
        NCollection_Sequence<TDF_Label> components;
        shapes->GetComponents(root, components);
        if (components.IsEmpty()) {
            appendBody(QStringLiteral("Part_%1").arg(rootIndex), root, shapes,
                       {}, bodies);
            continue;
        }
        for (int componentIndex = 1; componentIndex <= components.Length();
             ++componentIndex) {
            appendBody(QStringLiteral("Part_%1_%2")
                           .arg(rootIndex).arg(componentIndex),
                       components.Value(componentIndex), shapes, {}, bodies);
        }
    }
    if (bodies->empty()) {
        *error = QStringLiteral("机台文件没有 LCNC_AXIS_* 命名部件");
        return false;
    }
    return true;
}

bool triangulate(const TopoDS_Shape& source, PlainMesh* mesh, QString* error)
{
    try {
        TopoDS_Shape shape = BRepBuilderAPI_Copy(source).Shape();
        BRepMesh_IncrementalMesh mesher(shape, kMeshDeflectionMm, false, 0.35, true);
        mesher.Perform();
        for (TopExp_Explorer explorer(shape, TopAbs_FACE);
             explorer.More(); explorer.Next()) {
            const TopoDS_Face face = TopoDS::Face(explorer.Current());
            TopLoc_Location location;
            const Handle(Poly_Triangulation) triangulation =
                BRep_Tool::Triangulation(face, location);
            if (triangulation.IsNull())
                continue;
            const std::size_t vertexOffset = mesh->vertices.size();
            for (int node = 1; node <= triangulation->NbNodes(); ++node) {
                gp_Pnt point = triangulation->Node(node);
                point.Transform(location.Transformation());
                mesh->vertices.emplace_back(point.X(), point.Y(), point.Z());
            }
            for (int index = 1; index <= triangulation->NbTriangles(); ++index) {
                int first = 0, second = 0, third = 0;
                triangulation->Triangle(index).Get(first, second, third);
                if (face.Orientation() == TopAbs_REVERSED)
                    std::swap(second, third);
                mesh->triangles.emplace_back(
                    vertexOffset + static_cast<std::size_t>(first - 1),
                    vertexOffset + static_cast<std::size_t>(second - 1),
                    vertexOffset + static_cast<std::size_t>(third - 1));
            }
        }
    } catch (const Standard_Failure& failure) {
        *error = QStringLiteral("OCCT 网格化失败: %1").arg(QString::fromUtf8(failure.what()));
        return false;
    }
    if (mesh->triangles.empty()) {
        *error = QStringLiteral("机台部件网格为空");
        return false;
    }
    return true;
}

bool buildBackendModels(std::vector<Body>* bodies, QString* error)
{
    for (Body& body : *bodies) {
        std::string surfaceError;
        body.surfaceModel = lcnc::cam_algo::SurfaceCollisionModel::build(
            body.shape, kMeshDeflectionMm, &surfaceError, 2'000'000);
        if (!body.surfaceModel.isValid()) {
            *error = body.name + QStringLiteral(": ")
                + QString::fromStdString(surfaceError);
            return false;
        }
        const auto soup = body.surfaceModel.triangleSoup();
        PlainMesh mesh;
        mesh.vertices.reserve(soup.vertices.size());
        mesh.triangles.reserve(soup.triangles.size());
        for (const auto& point : soup.vertices)
            mesh.vertices.emplace_back(point[0], point[1], point[2]);
        for (const auto& triangle : soup.triangles)
            mesh.triangles.emplace_back(triangle[0], triangle[1], triangle[2]);
        auto model = std::make_shared<BackendModel>();
        if (model->beginModel(static_cast<int>(mesh.triangles.size()),
                              static_cast<int>(mesh.vertices.size())) != 0
            || model->addSubModel(mesh.vertices, mesh.triangles) != 0
            || model->endModel() != 0) {
            *error = body.name + QStringLiteral(": 后端 BVH 构建失败");
            return false;
        }
        model->computeLocalAABB();
        body.triangleCount = mesh.triangles.size();
        body.meshDeflectionMm = body.surfaceModel.linearDeflectionMm();
        body.model = std::move(model);
        for (const auto& point : soup.vertices)
            body.localBounds.Add(gp_Pnt(point[0], point[1], point[2]));
    }
    return true;
}

bool buildBackendModelsFromPersistedIndex(
    const lcnc::cam_algo::MachineSafetyIndex& index,
    const QString& modelPath, std::vector<Body>* bodies,
    int* persistedBodyCount, int* stepBodyCount, QString* error)
{
    if (persistedBodyCount)
        *persistedBodyCount = 0;
    if (stepBodyCount)
        *stepBodyCount = 0;
    bool allPersisted = true;
    for (int bodyIndex = 0; bodyIndex < index.bodies().size(); ++bodyIndex) {
        const auto* surface = index.persistedSurfaceModel(bodyIndex);
        allPersisted = allPersisted && surface && surface->isValid();
    }
    if (allPersisted) {
        bodies->clear();
        bodies->reserve(static_cast<std::size_t>(index.bodies().size()));
        for (const auto& summary : index.bodies()) {
            Body body;
            body.name = summary.name;
            body.axis = summary.axisName;
            bodies->push_back(std::move(body));
        }
    } else {
        if (!loadMachineBodies(modelPath, bodies, error))
            return false;
        if (bodies->size() != static_cast<std::size_t>(index.bodies().size())) {
            *error = QStringLiteral("STEP 与安全索引的机台部件数量不一致");
            return false;
        }
    }
    for (int bodyIndex = 0; bodyIndex < index.bodies().size(); ++bodyIndex) {
        Body& body = bodies->at(static_cast<std::size_t>(bodyIndex));
        const auto& summary = index.bodies().at(bodyIndex);
        if (body.name != summary.name
            || body.axis.compare(summary.axisName, Qt::CaseInsensitive) != 0) {
            *error = QStringLiteral("STEP 与安全索引的机台部件顺序不一致: %1")
                         .arg(summary.name);
            return false;
        }
        const auto* surface = index.persistedSurfaceModel(bodyIndex);
        if (!surface || !surface->isValid()) {
            PlainMesh mesh;
            if (!triangulate(body.shape, &mesh, error)) {
                *error = body.name + QStringLiteral(": ") + *error;
                return false;
            }
            auto model = std::make_shared<BackendModel>();
            if (model->beginModel(static_cast<int>(mesh.triangles.size()),
                                  static_cast<int>(mesh.vertices.size())) != 0
                || model->addSubModel(mesh.vertices, mesh.triangles) != 0
                || model->endModel() != 0) {
                *error = body.name + QStringLiteral(": 后端 BVH 构建失败");
                return false;
            }
            model->computeLocalAABB();
            body.model = std::move(model);
            body.triangleCount = mesh.triangles.size();
            body.meshDeflectionMm = kMeshDeflectionMm;
            BRepBndLib::Add(body.shape, body.localBounds);
            if (stepBodyCount)
                ++(*stepBodyCount);
            continue;
        }
        const auto soup = surface->triangleSoup();
        if (soup.vertices.empty() || soup.triangles.empty()) {
            *error = QStringLiteral("安全索引的持久三角网格为空");
            return false;
        }
        std::vector<BackendVector> vertices;
        std::vector<BackendTriangle> triangles;
        vertices.reserve(soup.vertices.size());
        triangles.reserve(soup.triangles.size());
        Bnd_Box bounds;
        for (const auto& point : soup.vertices) {
            vertices.emplace_back(point[0], point[1], point[2]);
            bounds.Add(gp_Pnt(point[0], point[1], point[2]));
        }
        for (const auto& triangle : soup.triangles)
            triangles.emplace_back(triangle[0], triangle[1], triangle[2]);
        auto model = std::make_shared<BackendModel>();
        if (model->beginModel(static_cast<int>(triangles.size()),
                              static_cast<int>(vertices.size())) != 0
            || model->addSubModel(vertices, triangles) != 0
            || model->endModel() != 0) {
            *error = index.bodies().at(bodyIndex).name
                + QStringLiteral(": 持久网格的后端 BVH 构建失败");
            return false;
        }
        model->computeLocalAABB();
        body.model = std::move(model);
        body.surfaceModel = *surface;
        body.triangleCount = triangles.size();
        body.meshDeflectionMm = surface->linearDeflectionMm();
        body.localBounds = bounds;
        if (persistedBodyCount)
            ++(*persistedBodyCount);
    }
    return !bodies->empty();
}

int axisIndex(const QVector<lcnc::cam_algo::MachineSafetyAxisGrid>& axes,
              const QString& name)
{
    for (int index = 0; index < axes.size(); ++index) {
        if (axes.at(index).name.compare(name, Qt::CaseInsensitive) == 0)
            return index;
    }
    return -1;
}

gp_Trsf localAxisTransform(
    const lcnc::cam_algo::MachineSafetyAxisGrid& axis, double value)
{
    gp_Trsf result;
    const gp_Dir direction(axis.direction[0], axis.direction[1],
                           axis.direction[2]);
    if (axis.rotary) {
        result.SetRotation(gp_Ax1(gp_Pnt(axis.origin[0], axis.origin[1],
                                        axis.origin[2]), direction),
                           value * kPi / 180.0);
    } else {
        result.SetTranslation(gp_Vec(direction) * value);
    }
    return result;
}

gp_Trsf chainTransform(
    const QVector<lcnc::cam_algo::MachineSafetyAxisGrid>& axes,
    const lcnc::cam_algo::MachineSafetyPose& pose,
    const std::vector<int>& axisChain)
{
    gp_Trsf result;
    for (const int index : axisChain) {
        if (index >= 0 && index < pose.count)
            result = result.Multiplied(
                localAxisTransform(axes.at(index), pose.values[index]));
    }
    return result;
}

bool prepareBodyChains(
    const QVector<lcnc::cam_algo::MachineSafetyAxisGrid>& axes,
    std::vector<Body>* bodies, QString* error)
{
    for (Body& body : *bodies) {
        std::vector<int> reverseChain;
        QString current = body.axis;
        QSet<QString> seen;
        while (!current.isEmpty()
               && current.compare(QStringLiteral("BASE"), Qt::CaseInsensitive) != 0
               && !seen.contains(current)) {
            seen.insert(current);
            const int index = axisIndex(axes, current);
            if (index < 0) {
                *error = QStringLiteral("部件 %1 的轴链包含未知轴 %2")
                             .arg(body.name, current);
                return false;
            }
            reverseChain.push_back(index);
            current = axes.at(index).parentAxis;
        }
        body.axisChain.assign(reverseChain.rbegin(), reverseChain.rend());
    }
    return true;
}

BackendTransform backendTransform(const gp_Trsf& source)
{
#if defined(LCNC_COLLISION_BACKEND_FCL)
    BackendTransform result = BackendTransform::Identity();
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column)
            result.linear()(row, column) = source.Value(row + 1, column + 1);
        result.translation()(row) = source.Value(row + 1, 4);
    }
    return result;
#else
    coal::Matrix3s rotation;
    coal::Vec3s translation;
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column)
            rotation(row, column) = source.Value(row + 1, column + 1);
        translation(row) = source.Value(row + 1, 4);
    }
    return BackendTransform(rotation, translation);
#endif
}

bool matchesAxisPair(const QString& first, const QString& second,
                     const QPair<QString, QString>& requested)
{
    return (first.compare(requested.first, Qt::CaseInsensitive) == 0
            && second.compare(requested.second, Qt::CaseInsensitive) == 0)
        || (first.compare(requested.second, Qt::CaseInsensitive) == 0
            && second.compare(requested.first, Qt::CaseInsensitive) == 0);
}

bool prepareQueryEngine(
    const std::vector<Body>& bodies,
    const QVector<lcnc::cam_algo::MachineSafetyAxisGrid>& axes,
    const QVector<QPair<QString, QString>>& axisPairs,
    bool useCachedGuess, QueryEngine* engine, QString* error)
{
#if defined(LCNC_COLLISION_BACKEND_FCL)
    Q_UNUSED(useCachedGuess);
#endif
    engine->bodies = &bodies;
    engine->axes = &axes;
    engine->transforms.resize(bodies.size());
    engine->occtTransforms.resize(bodies.size());
    for (const auto& requested : axisPairs) {
        bool found = false;
        for (std::size_t first = 0; first < bodies.size(); ++first) {
            for (std::size_t second = first + 1; second < bodies.size(); ++second) {
                if (!matchesAxisPair(bodies[first].axis, bodies[second].axis,
                                     requested))
                    continue;
                PreparedPair pair;
                pair.first = first;
                pair.second = second;
#if defined(LCNC_COLLISION_BACKEND_COAL)
                pair.collision = std::make_unique<coal::ComputeCollision>(
                    bodies[first].model.get(), bodies[second].model.get());
                pair.request.enable_contact = false;
                pair.request.security_margin = kCertificationThresholdMm;
                pair.request.break_distance = kCertificationThresholdMm;
                pair.request.distance_upper_bound = kCertificationThresholdMm;
                pair.request.gjk_initial_guess = useCachedGuess
                    ? coal::GJKInitialGuess::CachedGuess
                    : coal::GJKInitialGuess::DefaultGuess;
#endif
                engine->pairs.push_back(std::move(pair));
                found = true;
            }
        }
        if (!found) {
            *error = QStringLiteral("未找到请求的碰撞对 %1-%2")
                         .arg(requested.first, requested.second);
            return false;
        }
    }
    return !engine->pairs.empty();
}

bool queryEngineMatchesIndexPairs(
    const QueryEngine& engine,
    const lcnc::cam_algo::MachineSafetyIndex& index,
    QString* error)
{
    if (engine.pairs.size() != static_cast<std::size_t>(index.pairs().size())) {
        *error = QStringLiteral("LMSI 碰撞对数量与 Coal 查询引擎不一致");
        return false;
    }
    const auto sameNames = [](const Body& actualFirst, const Body& actualSecond,
                              const auto& expectedFirst, const auto& expectedSecond) {
        return (actualFirst.name == expectedFirst.name
                && actualSecond.name == expectedSecond.name)
            || (actualFirst.name == expectedSecond.name
                && actualSecond.name == expectedFirst.name);
    };
    for (int pairIndex = 0; pairIndex < index.pairs().size(); ++pairIndex) {
        const auto& expected = index.pairs().at(pairIndex);
        if (expected.firstBody >= index.bodies().size()
            || expected.secondBody >= index.bodies().size()) {
            *error = QStringLiteral("LMSI 碰撞对引用了无效部件");
            return false;
        }
        const auto& actual = engine.pairs[static_cast<std::size_t>(pairIndex)];
        if (!sameNames(engine.bodies->at(actual.first),
                       engine.bodies->at(actual.second),
                       index.bodies().at(expected.firstBody),
                       index.bodies().at(expected.secondBody))) {
            *error = QStringLiteral("LMSI 与 Coal 的碰撞对顺序不一致: pair=%1")
                         .arg(pairIndex);
            return false;
        }
    }
    return true;
}

struct PairQueryResult
{
    bool beyondCertificationThreshold{false};
    double distanceLowerBoundMm{0.0};
};

double pairCertificationThresholdMm(const PreparedPair& pair,
                                    const std::vector<Body>& bodies)
{
    return kRequiredClearanceMm
        + bodies[pair.first].meshDeflectionMm
        + bodies[pair.second].meshDeflectionMm;
}

PairQueryResult backendPairQuery(
    PreparedPair* pair, const std::vector<Body>& bodies,
    const std::vector<BackendTransform>& transforms,
    const std::vector<gp_Trsf>& occtTransforms,
    double requiredSeparationMm)
{
    const BackendTransform& firstTf = transforms[pair->first];
    const BackendTransform& secondTf = transforms[pair->second];
#if defined(LCNC_COLLISION_BACKEND_FCL)
    const BackendModel* first = bodies[pair->first].model.get();
    const BackendModel* second = bodies[pair->second].model.get();
    fcl::DistanceRequestd request(false, 0.0, 0.0,
                                  fcl::GJKSolverType::GST_INDEP);
    fcl::DistanceResultd result;
    const double distance =
        fcl::distance(first, firstTf, second, secondTf, request, result);
    return {distance > requiredSeparationMm, distance};
#else
    Q_UNUSED(bodies);
    // Coal can stop as soon as it proves separation beyond the complete
    // requested certificate threshold. This is the intended BoundaryUnknown use, and
    // avoids paying for an exact mesh-to-mesh minimum distance.
    coal::CollisionResult result;
    pair->request.security_margin = requiredSeparationMm;
    pair->request.break_distance = requiredSeparationMm;
    pair->request.distance_upper_bound = requiredSeparationMm;
    const std::size_t contacts =
        (*pair->collision)(firstTf, secondTf, pair->request, result);
    pair->request.updateGuess(result);
    bool separated = contacts == 0
        && result.distance_lower_bound > requiredSeparationMm;
    if (separated
        && lcnc::cam_algo::surfaceCollisionContainmentDetected(
            bodies[pair->first].surfaceModel, occtTransforms[pair->first],
            bodies[pair->second].surfaceModel, occtTransforms[pair->second])) {
        separated = false;
    }
    return {separated, separated ? result.distance_lower_bound : 0.0};
#endif
}

void updateTransforms(QueryEngine* engine,
                      const lcnc::cam_algo::MachineSafetyPose& pose)
{
    const std::vector<Body>& bodies = *engine->bodies;
    const auto& axes = *engine->axes;
    for (std::size_t index = 0; index < bodies.size(); ++index) {
        engine->occtTransforms[index] = chainTransform(
            axes, pose, bodies[index].axisChain);
        engine->transforms[index] = backendTransform(
            engine->occtTransforms[index]);
    }
}

QueryResult queryPose(
    QueryEngine* engine, const lcnc::cam_algo::MachineSafetyPose& pose,
    int preferredPair = -1)
{
    QueryResult result;
    const std::vector<Body>& bodies = *engine->bodies;
    updateTransforms(engine, pose);
    const auto evaluate = [&](int index) {
        const PairQueryResult pair = backendPairQuery(
            &engine->pairs[static_cast<std::size_t>(index)], bodies,
            engine->transforms, engine->occtTransforms,
            pairCertificationThresholdMm(
                engine->pairs[static_cast<std::size_t>(index)], bodies));
        ++result.evaluatedPairs;
        result.minimumDistanceMm = std::min(result.minimumDistanceMm,
                                             pair.distanceLowerBoundMm);
        if (!pair.beyondCertificationThreshold)
            result.blockedByThreshold = true;
    };
    if (preferredPair >= 0
        && preferredPair < static_cast<int>(engine->pairs.size()))
        evaluate(preferredPair);
    for (int index = 0;
         !result.blockedByThreshold
             && index < static_cast<int>(engine->pairs.size()); ++index) {
        if (index != preferredPair)
            evaluate(index);
    }
    result.certifiedSafe = !result.blockedByThreshold;
    return result;
}

Bnd_Box transformedBounds(const Bnd_Box& local, const gp_Trsf& transform)
{
    Bnd_Box result;
    if (local.IsVoid())
        return result;
    double minimumX = 0.0, minimumY = 0.0, minimumZ = 0.0;
    double maximumX = 0.0, maximumY = 0.0, maximumZ = 0.0;
    local.Get(minimumX, minimumY, minimumZ,
              maximumX, maximumY, maximumZ);
    for (double x : {minimumX, maximumX}) {
        for (double y : {minimumY, maximumY}) {
            for (double z : {minimumZ, maximumZ}) {
                gp_Pnt point(x, y, z);
                point.Transform(transform);
                result.Add(point);
            }
        }
    }
    return result;
}

double maximumRadiusFromAxis(const Bnd_Box& box,
                             const gp_Pnt& origin,
                             const gp_Dir& direction)
{
    if (box.IsVoid())
        return 0.0;
    double minimumX = 0.0, minimumY = 0.0, minimumZ = 0.0;
    double maximumX = 0.0, maximumY = 0.0, maximumZ = 0.0;
    box.Get(minimumX, minimumY, minimumZ,
            maximumX, maximumY, maximumZ);
    const gp_Vec axis(direction);
    double radius = 0.0;
    for (double x : {minimumX, maximumX}) {
        for (double y : {minimumY, maximumY}) {
            for (double z : {minimumZ, maximumZ}) {
                const gp_Vec radial(origin, gp_Pnt(x, y, z));
                radius = std::max(radius, radial.Crossed(axis).Magnitude());
            }
        }
    }
    return radius;
}

double bodyIntervalMotionBound(
    const Body& body,
    const QVector<lcnc::cam_algo::MachineSafetyAxisGrid>& axes,
    const lcnc::cam_algo::MachineSafetyPose& first,
    const lcnc::cam_algo::MachineSafetyPose& last,
    const lcnc::cam_algo::MachineSafetyPose& midpoint,
    const std::array<double, lcnc::cam_algo::kMachineSafetyMaximumAxes>&
        deviationTolerance)
{
    std::vector<gp_Trsf> parentTransforms;
    parentTransforms.reserve(body.axisChain.size());
    gp_Trsf running;
    for (const int axisIndexValue : body.axisChain) {
        parentTransforms.push_back(running);
        running = running.Multiplied(localAxisTransform(
            axes.at(axisIndexValue), midpoint.values[axisIndexValue]));
    }
    const Bnd_Box centerBounds = transformedBounds(body.localBounds, running);
    double descendantBound = 0.0;
    for (int offset = static_cast<int>(body.axisChain.size()) - 1;
         offset >= 0; --offset) {
        const int axisIndexValue = body.axisChain[static_cast<std::size_t>(offset)];
        const auto& axis = axes.at(axisIndexValue);
        const double halfDelta = std::abs(
            last.values[axisIndexValue] - first.values[axisIndexValue]) * 0.5
            + deviationTolerance[axisIndexValue];
        double contribution = halfDelta;
        if (axis.rotary) {
            gp_Pnt origin(axis.origin[0], axis.origin[1], axis.origin[2]);
            gp_Dir direction(axis.direction[0], axis.direction[1], axis.direction[2]);
            origin.Transform(parentTransforms[static_cast<std::size_t>(offset)]);
            direction.Transform(parentTransforms[static_cast<std::size_t>(offset)]);
            const double radius = maximumRadiusFromAxis(
                centerBounds, origin, direction) + descendantBound;
            contribution = halfDelta >= 180.0 ? 2.0 * radius
                : 2.0 * radius * std::sin(halfDelta * kPi / 360.0);
        }
        descendantBound += contribution;
    }
    return descendantBound;
}

lcnc::cam_algo::MachineSafetyPose interpolatedPose(
    const lcnc::cam_algo::MachineSafetyPose& first,
    const lcnc::cam_algo::MachineSafetyPose& last,
    double parameter)
{
    lcnc::cam_algo::MachineSafetyPose result;
    result.count = first.count;
    for (int axis = 0; axis < result.count; ++axis)
        result.values[axis] = first.values[axis]
            + (last.values[axis] - first.values[axis]) * parameter;
    return result;
}

struct AdaptiveCoalMetrics
{
    std::uint64_t intervalQueries{0};
    std::uint64_t pairQueries{0};
    std::uint64_t splits{0};
    std::uint64_t certifiedIntervals{0};
    std::uint64_t blockedLeaves{0};
    int maximumDepth{0};
};

bool coalCertifiesMotionInterval(
    QueryEngine* engine,
    const lcnc::cam_algo::MachineSafetyPose& first,
    const lcnc::cam_algo::MachineSafetyPose& last,
    int preferredPair,
    const std::array<double, lcnc::cam_algo::kMachineSafetyMaximumAxes>&
        deviationTolerance,
    AdaptiveCoalMetrics* metrics)
{
    const auto midpoint = interpolatedPose(first, last, 0.5);
    updateTransforms(engine, midpoint);
    const auto& bodies = *engine->bodies;
    const auto& axes = *engine->axes;
    const auto evaluate = [&](int pairIndex) {
        PreparedPair* pair = &engine->pairs[static_cast<std::size_t>(pairIndex)];
        const double motionBound = bodyIntervalMotionBound(
            bodies[pair->first], axes, first, last, midpoint,
            deviationTolerance)
            + bodyIntervalMotionBound(
                bodies[pair->second], axes, first, last, midpoint,
                deviationTolerance);
        ++metrics->pairQueries;
        return backendPairQuery(pair, bodies, engine->transforms,
                                engine->occtTransforms,
                                pairCertificationThresholdMm(*pair, bodies)
                                    + motionBound)
            .beyondCertificationThreshold;
    };
    if (preferredPair >= 0
        && preferredPair < static_cast<int>(engine->pairs.size())
        && !evaluate(preferredPair)) {
        return false;
    }
    for (int pair = 0; pair < static_cast<int>(engine->pairs.size()); ++pair) {
        if (pair != preferredPair && !evaluate(pair))
            return false;
    }
    return true;
}

double maximumPairIntervalMotionBound(
    QueryEngine* engine,
    const lcnc::cam_algo::MachineSafetyPose& first,
    const lcnc::cam_algo::MachineSafetyPose& last,
    const std::array<double, lcnc::cam_algo::kMachineSafetyMaximumAxes>&
        deviationTolerance)
{
    const auto midpoint = interpolatedPose(first, last, 0.5);
    const auto& bodies = *engine->bodies;
    const auto& axes = *engine->axes;
    double result = 0.0;
    for (const PreparedPair& pair : engine->pairs) {
        result = std::max(result,
            bodyIntervalMotionBound(
                bodies[pair.first], axes, first, last, midpoint,
                deviationTolerance)
            + bodyIntervalMotionBound(
                bodies[pair.second], axes, first, last, midpoint,
                deviationTolerance));
    }
    return result;
}

bool adaptivelyCertifyCoalInterval(
    QueryEngine* engine,
    const lcnc::cam_algo::MachineSafetyPose& edgeFirst,
    const lcnc::cam_algo::MachineSafetyPose& edgeLast,
    double parameterFirst, double parameterLast,
    int preferredPair, int depth, int maximumDepth,
    const std::array<double, lcnc::cam_algo::kMachineSafetyMaximumAxes>&
        deviationTolerance,
    AdaptiveCoalMetrics* metrics)
{
    metrics->maximumDepth = std::max(metrics->maximumDepth, depth);
    ++metrics->intervalQueries;
    const auto first = interpolatedPose(edgeFirst, edgeLast, parameterFirst);
    const auto last = interpolatedPose(edgeFirst, edgeLast, parameterLast);
    bool hasPathSpan = false;
    for (int axis = 0; axis < first.count; ++axis)
        hasPathSpan = hasPathSpan
            || std::abs(last.values[axis] - first.values[axis]) > 1e-12;
    // Inflating Coal's security margin by a large rotary sweep makes the BVH
    // traverse most of both meshes. Split geometrically first so every Coal
    // proof remains a narrow local certificate.
    constexpr double kMaximumQueryMotionBoundMm = 0.25;
    if (hasPathSpan && depth < maximumDepth
        && maximumPairIntervalMotionBound(
               engine, first, last, deviationTolerance)
            > kMaximumQueryMotionBoundMm) {
        ++metrics->splits;
        const double parameterMidpoint = (parameterFirst + parameterLast) * 0.5;
        return adaptivelyCertifyCoalInterval(
                   engine, edgeFirst, edgeLast,
                   parameterFirst, parameterMidpoint,
                   preferredPair, depth + 1, maximumDepth,
                   deviationTolerance, metrics)
            && adaptivelyCertifyCoalInterval(
                   engine, edgeFirst, edgeLast,
                   parameterMidpoint, parameterLast,
                   preferredPair, depth + 1, maximumDepth,
                   deviationTolerance, metrics);
    }
    if (coalCertifiesMotionInterval(engine, first, last, preferredPair,
                                    deviationTolerance, metrics)) {
        ++metrics->certifiedIntervals;
        return true;
    }
    if (!hasPathSpan || depth >= maximumDepth) {
        ++metrics->blockedLeaves;
        return false;
    }
    ++metrics->splits;
    const double parameterMidpoint = (parameterFirst + parameterLast) * 0.5;
    return adaptivelyCertifyCoalInterval(
               engine, edgeFirst, edgeLast, parameterFirst, parameterMidpoint,
               preferredPair, depth + 1, maximumDepth,
               deviationTolerance, metrics)
        && adaptivelyCertifyCoalInterval(
               engine, edgeFirst, edgeLast, parameterMidpoint, parameterLast,
               preferredPair, depth + 1, maximumDepth,
               deviationTolerance, metrics);
}

bool loadAuditSamples(const QString& path,
                      const QVector<lcnc::cam_algo::MachineSafetyAxisGrid>& axes,
                      std::vector<AuditSample>* samples, QString* error)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        *error = QStringLiteral("无法打开 Unknown 审计集: %1").arg(file.errorString());
        return false;
    }
    QTextStream stream(&file);
    stream.readLine();
    const QRegularExpression poseExpression(
        QStringLiteral("([XYZABC])=([-+0-9.eE]+)"));
    while (!stream.atEnd()) {
        const QString line = stream.readLine().trimmed();
        if (line.isEmpty())
            continue;
        const QStringList columns = line.split(QLatin1Char(','));
        if (columns.size() < 8) {
            *error = QStringLiteral("Unknown 审计 CSV 行格式错误");
            return false;
        }
        AuditSample sample;
        sample.sample = columns.at(0).toInt();
        sample.exactCollision = columns.at(4) == QStringLiteral("Collision");
        sample.exactNear = columns.at(4) == QStringLiteral("ExactSafeNear");
        sample.exactDistanceMm = columns.at(5).toDouble();
        sample.pose.count = static_cast<std::uint8_t>(axes.size());
        QRegularExpressionMatchIterator matches = poseExpression.globalMatch(line);
        while (matches.hasNext()) {
            const QRegularExpressionMatch match = matches.next();
            const int index = axisIndex(axes, match.captured(1));
            if (index >= 0)
                sample.pose.values[index] = match.captured(2).toDouble();
        }
        samples->push_back(sample);
    }
    if (samples->size() != 64) {
        *error = QStringLiteral("Unknown 审计集期望 64 行，实际 %1")
                     .arg(samples->size());
        return false;
    }
    return true;
}

bool loadHalfSphereMotionEdges(
    const QString& path,
    const QVector<lcnc::cam_algo::MachineSafetyAxisGrid>& axes,
    std::vector<MotionEdgeSample>* edges,
    int* contourCount, int* solvedPointCount,
    QString* error)
{
    STEPControl_Reader reader;
    if (reader.ReadFile(path.toUtf8().constData()) != IFSelect_RetDone
        || reader.TransferRoots() == 0) {
        *error = QStringLiteral("无法读取半球真实工件模型");
        return false;
    }
    const TopoDS_Shape workpiece = reader.OneShape();
    if (workpiece.IsNull()) {
        *error = QStringLiteral("半球工件模型没有有效形状");
        return false;
    }
    QString faceInfo;
    const std::vector<TopoDS_Face> faces =
        LaserToolpathBuilder::selectTopVisibleFacesFromPositiveZ(
            workpiece, &faceInfo);
    if (faces.empty()) {
        *error = QStringLiteral("半球模型没有可加工顶面: %1").arg(faceInfo);
        return false;
    }
    ContourExtractionParams parameters;
    parameters.strategy = ExtractionStrategy::ManualFaceSelection;
    parameters.selectedMachiningFaces = faces;
    parameters.deflection = 0.20;
    auto contours = LaserToolpathBuilder::extractContoursFromFaces(
        workpiece, faces, gp_Dir(0.0, 0.0, -1.0), parameters);
    if (contours.empty()) {
        *error = QStringLiteral("半球模型没有生成加工轮廓");
        return false;
    }
    std::vector<LaserContour*> ordered;
    for (LaserContour& contour : contours) {
        contour.sourceShape = workpiece;
        LaserToolpathBuilder::discretizeContour(
            contour, workpiece, parameters.deflection);
        if (contour.points.size() >= 2)
            ordered.push_back(&contour);
    }
    if (ordered.empty()) {
        *error = QStringLiteral("半球加工轮廓离散后没有有效线段");
        return false;
    }
    MachineKinematics kinematics;
    kinematics.loadPreset(QStringLiteral("VERTICAL_AC_TABLE"));
    lcnc::MachineConfigurationService configuration;
    configuration.applyPreset(QStringLiteral("VERTICAL_AC_TABLE"));
    const auto definition = configuration.modeDefinition(
        lcnc::MachiningMode::SimultaneousTable5Axis);
    if (definition.interpolatedAxes.count != axes.size()) {
        *error = QStringLiteral("真实刀路轴布局与 LMSI 不一致");
        return false;
    }
    for (int axis = 0; axis < axes.size(); ++axis) {
        if (definition.interpolatedAxes.axes[axis].name
                .compare(axes.at(axis).name, Qt::CaseInsensitive) != 0) {
            *error = QStringLiteral("真实刀路轴顺序与 LMSI 不一致");
            return false;
        }
    }
    QString solveError;
    if (!LaserToolpathBuilder::solveToolpathForOrder(
            ordered, &kinematics, gp_Trsf{}, definition,
            {}, {}, &solveError)) {
        *error = QStringLiteral("半球五轴刀路求解失败: %1").arg(solveError);
        return false;
    }

    int pointCount = 0;
    for (const LaserContour* contour : ordered) {
        std::vector<lcnc::cam_algo::MachineSafetyPose> poses;
        poses.reserve(contour->points.size());
        for (const ToolpathPoint& point : contour->points) {
            if (!point.machineCoord.valid)
                continue;
            lcnc::cam_algo::MachineSafetyPose pose;
            pose.count = static_cast<std::uint8_t>(axes.size());
            bool inRange = true;
            for (int axis = 0; axis < axes.size(); ++axis) {
                pose.values[axis] = point.machineCoord.solvedPose.value(axis);
                const bool periodicSpin = axes.at(axis).rotary
                    && axes.at(axis).name.compare(
                        QStringLiteral("C"), Qt::CaseInsensitive) == 0;
                inRange = inRange && (periodicSpin
                    || (pose.values[axis] >= axes.at(axis).minimum - 1e-9
                        && pose.values[axis] <= axes.at(axis).maximum + 1e-9));
            }
            if (!inRange) {
                *error = QStringLiteral("半球真实刀路超出 LMSI 非周期物理轴域");
                return false;
            }
            poses.push_back(pose);
        }
        pointCount += static_cast<int>(poses.size());
        for (std::size_t point = 1; point < poses.size(); ++point)
            edges->push_back({poses[point - 1], poses[point]});
    }
    *contourCount = static_cast<int>(ordered.size());
    *solvedPointCount = pointCount;
    if (edges->empty()) {
        *error = QStringLiteral("半球真实刀路没有连续运动边");
        return false;
    }
    return true;
}

double percentile(std::vector<double> values, double fraction)
{
    if (values.empty())
        return 0.0;
    std::sort(values.begin(), values.end());
    const std::size_t index = static_cast<std::size_t>(
        std::ceil(fraction * static_cast<double>(values.size())) - 1.0);
    return values[std::min(index, values.size() - 1)];
}

struct CombinedDecision
{
    bool allowed{false};
    bool usedCoal{false};
    bool coalBlocked{false};
    int evaluatedPairs{0};
};

struct WorkloadStats
{
    std::uint64_t total{0};
    std::uint64_t lmsiSafe{0};
    std::uint64_t lmsiCollision{0};
    std::uint64_t lmsiUnknown{0};
    std::uint64_t coalSafe{0};
    std::uint64_t coalBlocked{0};
    std::uint64_t evaluatedPairs{0};
    std::vector<double> timingsUs;
    std::vector<double> lmsiTimingsUs;
    std::vector<double> coalTimingsUs;
};

struct PathDecisionCache
{
    std::vector<std::uint8_t> allowed;
    WorkloadStats buildStats;
    double buildMs{0.0};
};

CombinedDecision combinedQuery(
    const lcnc::cam_algo::MachineSafetyIndex& index, QueryEngine* engine,
    const lcnc::cam_algo::MachineSafetyPose& pose,
    WorkloadStats* stats = nullptr)
{
    const auto indexed = index.query(pose);
    CombinedDecision decision;
    if (indexed.state == lcnc::cam_algo::MachineSafetyIndexState::CertifiedSafe) {
        decision.allowed = true;
        if (stats)
            ++stats->lmsiSafe;
        return decision;
    }
    if (indexed.state == lcnc::cam_algo::MachineSafetyIndexState::CollisionSample) {
        if (stats)
            ++stats->lmsiCollision;
        return decision;
    }
    if (stats)
        ++stats->lmsiUnknown;
    const QueryResult coal = queryPose(engine, pose, indexed.blockingPair);
    decision.allowed = coal.certifiedSafe;
    decision.usedCoal = true;
    decision.coalBlocked = coal.blockedByThreshold;
    decision.evaluatedPairs = coal.evaluatedPairs;
    if (stats) {
        stats->evaluatedPairs += static_cast<std::uint64_t>(coal.evaluatedPairs);
        if (coal.certifiedSafe)
            ++stats->coalSafe;
        else
            ++stats->coalBlocked;
    }
    return decision;
}

std::uint64_t nextRandom(std::uint64_t* state)
{
    std::uint64_t value = *state;
    value ^= value << 13;
    value ^= value >> 7;
    value ^= value << 17;
    *state = value;
    return value;
}

double randomUnit(std::uint64_t* state)
{
    return static_cast<double>(nextRandom(state) >> 11)
        * (1.0 / 9007199254740992.0);
}

std::vector<lcnc::cam_algo::MachineSafetyPose> makeGlobalRandomPoses(
    const QVector<lcnc::cam_algo::MachineSafetyAxisGrid>& axes,
    std::size_t count)
{
    std::vector<lcnc::cam_algo::MachineSafetyPose> poses(count);
    std::uint64_t randomState = 0x4c4d5349434f414cULL;
    for (auto& pose : poses) {
        pose.count = static_cast<std::uint8_t>(axes.size());
        for (int axis = 0; axis < axes.size(); ++axis) {
            const auto& grid = axes.at(axis);
            pose.values[axis] = grid.minimum
                + (grid.maximum - grid.minimum) * randomUnit(&randomState);
        }
    }
    return poses;
}

struct OfflineCoalCoverageComparison
{
    std::uint64_t total{0};
    std::uint64_t lmsiSafe{0};
    std::uint64_t coalCurrentLeafSafe{0};
    std::uint64_t coalHotChildSafe{0};
    std::uint64_t remainingUnknown{0};
    std::uint64_t currentLeafPairQueries{0};
    std::uint64_t hotChildPairQueries{0};
    std::uint64_t currentLeafNs{0};
    std::uint64_t hotChildNs{0};
};

std::pair<lcnc::cam_algo::MachineSafetyPose,
          lcnc::cam_algo::MachineSafetyPose>
containingSafetyCellBounds(
    const QVector<lcnc::cam_algo::MachineSafetyAxisGrid>& axes,
    const lcnc::cam_algo::MachineSafetyPose& pose, int refinementLevel)
{
    lcnc::cam_algo::MachineSafetyPose first;
    lcnc::cam_algo::MachineSafetyPose last;
    first.count = pose.count;
    last.count = pose.count;
    const double subdivision = std::ldexp(1.0, refinementLevel);
    for (int axisIndexValue = 0; axisIndexValue < pose.count; ++axisIndexValue) {
        const auto& axis = axes.at(axisIndexValue);
        const double cellStep = axis.step / subdivision;
        const double clamped = std::min(
            std::nextafter(axis.maximum, axis.minimum),
            std::max(axis.minimum, pose.values[axisIndexValue]));
        const double ordinal = std::floor(
            (clamped - axis.minimum) / cellStep);
        first.values[axisIndexValue] = axis.minimum + ordinal * cellStep;
        last.values[axisIndexValue] = std::min(
            axis.maximum, first.values[axisIndexValue] + cellStep);
    }
    return {first, last};
}

bool coalCertifiesContainingSafetyCell(
    QueryEngine* engine,
    const lcnc::cam_algo::MachineSafetyPose& pose,
    int refinementLevel, std::uint64_t* pairQueries)
{
    const auto bounds = containingSafetyCellBounds(
        *engine->axes, pose, refinementLevel);
    const auto midpoint = interpolatedPose(bounds.first, bounds.second, 0.5);
    const auto& bodies = *engine->bodies;
    const auto& axes = *engine->axes;
    const std::array<double, lcnc::cam_algo::kMachineSafetyMaximumAxes>
        noAdditionalDeviation{};
    bool aabbCertified = true;
    for (const PreparedPair& pair : engine->pairs) {
        const double motionBound = bodyIntervalMotionBound(
            bodies[pair.first], axes, bounds.first, bounds.second, midpoint,
            noAdditionalDeviation)
            + bodyIntervalMotionBound(
                bodies[pair.second], axes, bounds.first, bounds.second,
                midpoint, noAdditionalDeviation);
        const Bnd_Box firstBounds = transformedBounds(
            bodies[pair.first].localBounds,
            chainTransform(axes, midpoint, bodies[pair.first].axisChain));
        const Bnd_Box secondBounds = transformedBounds(
            bodies[pair.second].localBounds,
            chainTransform(axes, midpoint, bodies[pair.second].axisChain));
        if (firstBounds.Distance(secondBounds)
            <= pairCertificationThresholdMm(pair, bodies) + motionBound) {
            aabbCertified = false;
            break;
        }
    }
    if (aabbCertified)
        return true;

    // Large security margins make Coal traverse almost the complete pair of
    // triangle soups. Keep the offline pass budgeted and fail closed; deeper
    // hot-cell refinement can reduce the interval below this threshold.
    constexpr double kMaximumCoalCellMotionBoundMm = 0.25;
    if (maximumPairIntervalMotionBound(
            engine, bounds.first, bounds.second, noAdditionalDeviation)
        > kMaximumCoalCellMotionBoundMm) {
        return false;
    }
    AdaptiveCoalMetrics metrics;
    const bool certified = coalCertifiesMotionInterval(
        engine, bounds.first, bounds.second, -1,
        noAdditionalDeviation, &metrics);
    if (pairQueries)
        *pairQueries += metrics.pairQueries;
    return certified;
}

OfflineCoalCoverageComparison compareOfflineCoalCoverage(
    const std::vector<lcnc::cam_algo::MachineSafetyPose>& poses,
    const lcnc::cam_algo::MachineSafetyIndex& index,
    QueryEngine* engine)
{
    OfflineCoalCoverageComparison result;
    result.total = poses.size();
    for (const auto& pose : poses) {
        const auto indexed = index.query(pose);
        if (indexed.state
            == lcnc::cam_algo::MachineSafetyIndexState::CertifiedSafe) {
            ++result.lmsiSafe;
            continue;
        }
        if (indexed.state
            == lcnc::cam_algo::MachineSafetyIndexState::CollisionSample) {
            ++result.remainingUnknown;
            continue;
        }
        const auto currentStarted = std::chrono::steady_clock::now();
        const bool currentLeafSafe = coalCertifiesContainingSafetyCell(
            engine, pose, indexed.refinementLevel,
            &result.currentLeafPairQueries);
        result.currentLeafNs += static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now() - currentStarted).count());
        if (currentLeafSafe) {
            ++result.coalCurrentLeafSafe;
            continue;
        }
        constexpr int kOfflineHotRefinementLevel = 3;
        const auto hotStarted = std::chrono::steady_clock::now();
        const bool hotChildSafe = coalCertifiesContainingSafetyCell(
            engine, pose, kOfflineHotRefinementLevel,
            &result.hotChildPairQueries);
        result.hotChildNs += static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now() - hotStarted).count());
        if (hotChildSafe)
            ++result.coalHotChildSafe;
        else
            ++result.remainingUnknown;
    }
    return result;
}

std::vector<lcnc::cam_algo::MachineSafetyPose> makeContinuousStressPoses(
    const std::vector<AuditSample>& samples, int stepsPerSegment)
{
    std::vector<lcnc::cam_algo::MachineSafetyPose> poses;
    poses.reserve(samples.size() * static_cast<std::size_t>(stepsPerSegment));
    for (std::size_t sample = 0; sample < samples.size(); ++sample) {
        const auto& first = samples[sample].pose;
        const auto& second = samples[(sample + 1) % samples.size()].pose;
        for (int step = 0; step < stepsPerSegment; ++step) {
            const double fraction = static_cast<double>(step) / stepsPerSegment;
            lcnc::cam_algo::MachineSafetyPose pose;
            pose.count = first.count;
            for (int axis = 0; axis < pose.count; ++axis)
                pose.values[axis] = first.values[axis]
                    + (second.values[axis] - first.values[axis]) * fraction;
            poses.push_back(pose);
        }
    }
    return poses;
}

WorkloadStats runCombinedWorkload(
    const std::vector<lcnc::cam_algo::MachineSafetyPose>& poses,
    const lcnc::cam_algo::MachineSafetyIndex& index, QueryEngine* engine)
{
    WorkloadStats stats;
    stats.total = poses.size();
    stats.timingsUs.reserve(poses.size());
    stats.lmsiTimingsUs.reserve(poses.size());
    stats.coalTimingsUs.reserve(poses.size());
    for (const auto& pose : poses) {
        const auto start = std::chrono::steady_clock::now();
        const CombinedDecision decision = combinedQuery(index, engine, pose, &stats);
        const auto end = std::chrono::steady_clock::now();
        if (decision.allowed && decision.coalBlocked)
            std::abort();
        const double elapsedUs = std::chrono::duration<double, std::micro>(
            end - start).count();
        stats.timingsUs.push_back(elapsedUs);
        if (decision.usedCoal)
            stats.coalTimingsUs.push_back(elapsedUs);
        else
            stats.lmsiTimingsUs.push_back(elapsedUs);
    }
    return stats;
}

PathDecisionCache buildPathDecisionCache(
    const std::vector<lcnc::cam_algo::MachineSafetyPose>& poses,
    const lcnc::cam_algo::MachineSafetyIndex& index, QueryEngine* engine)
{
    PathDecisionCache cache;
    cache.allowed.reserve(poses.size());
    cache.buildStats.total = poses.size();
    const auto start = std::chrono::steady_clock::now();
    for (const auto& pose : poses) {
        const CombinedDecision decision = combinedQuery(
            index, engine, pose, &cache.buildStats);
        cache.allowed.push_back(decision.allowed ? 1U : 0U);
    }
    const auto end = std::chrono::steady_clock::now();
    cache.buildMs = std::chrono::duration<double, std::milli>(end - start).count();
    return cache;
}

WorkloadStats benchmarkPathCacheLookup(const PathDecisionCache& cache)
{
    WorkloadStats stats;
    stats.total = cache.allowed.size();
    stats.timingsUs.reserve(cache.allowed.size());
    volatile std::uint64_t allowedChecksum = 0;
    for (std::size_t index = 0; index < cache.allowed.size(); ++index) {
        const auto start = std::chrono::steady_clock::now();
        allowedChecksum += cache.allowed[index];
        const auto end = std::chrono::steady_clock::now();
        stats.timingsUs.push_back(std::chrono::duration<double, std::micro>(
                                     end - start).count());
    }
    if (allowedChecksum > cache.allowed.size())
        std::abort();
    return stats;
}

struct MotionCertificateWorkload
{
    std::vector<lcnc::cam_algo::MachineMotionEdgeCertificate> certificates;
    AdaptiveCoalMetrics coal;
    std::uint64_t lmsiSafeEdges{0};
    std::uint64_t lmsiBlockedEdges{0};
    std::uint64_t lmsiUnknownEdges{0};
    std::uint64_t finalSafeEdges{0};
    std::uint64_t finalBlockedEdges{0};
    std::uint64_t lmsiVisitedCells{0};
    std::uint64_t lmsiParameterIntervals{0};
    double buildMs{0.0};
};

MotionCertificateWorkload buildMotionCertificates(
    const std::vector<MotionEdgeSample>& edges,
    const lcnc::cam_algo::MachineSafetyIndex& index,
    QueryEngine* engine,
    const lcnc::cam_algo::MachineMotionCertificationOptions& options,
    std::uint64_t pathRevision)
{
    MotionCertificateWorkload workload;
    workload.certificates.reserve(edges.size());
    const QByteArray profileHash = QCryptographicHash::hash(
        QByteArrayLiteral("coordinated-linear-axes-v1"),
        QCryptographicHash::Sha256);
    const auto start = std::chrono::steady_clock::now();
    for (std::size_t edgeIndex = 0; edgeIndex < edges.size(); ++edgeIndex) {
        lcnc::cam_algo::MachineMotionCertificateKey key;
        key.machineSourceSha256 = index.sourceSha256();
        key.safetyIndexSha256 = index.contentSha256();
        key.safetyPolicySha256 = QCryptographicHash::hash(
            QByteArrayLiteral("clearance=0.5;mesh=0.1;threshold=0.7;tube-policy=v1"),
            QCryptographicHash::Sha256);
        key.motionProfileSha256 = profileHash;
        key.pathRevision = pathRevision;
        key.edgeId = edgeIndex;
        auto edgeOptions = options;
        for (int axis = 0; axis < edges[edgeIndex].first.count; ++axis) {
            const auto& grid = index.axes().at(axis);
            if (!grid.rotary
                || grid.name.compare(QStringLiteral("C"), Qt::CaseInsensitive) != 0)
                continue;
            const double midpoint = (edges[edgeIndex].first.values[axis]
                                     + edges[edgeIndex].last.values[axis]) * 0.5;
            const double domainCenter = (grid.minimum + grid.maximum) * 0.5;
            edgeOptions.canonicalAxisOffset[axis] =
                std::round((midpoint - domainCenter) / 360.0) * 360.0;
        }
        auto certificate = lcnc::cam_algo::certifyLinearMotionEdgeWithIndex(
            index, key, edges[edgeIndex].first, edges[edgeIndex].last,
            edgeOptions);
        workload.lmsiVisitedCells += certificate.visitedFineCells;
        workload.lmsiParameterIntervals += certificate.parameterIntervals;
        if (certificate.state
            == lcnc::cam_algo::MachineMotionCertificateState::CertifiedSafe) {
            ++workload.lmsiSafeEdges;
        } else if (certificate.state
                   == lcnc::cam_algo::MachineMotionCertificateState::Blocked) {
            ++workload.lmsiBlockedEdges;
        } else {
            ++workload.lmsiUnknownEdges;
            bool allCertified = certificate.state
                == lcnc::cam_algo::MachineMotionCertificateState::BoundaryUnknown;
            auto canonicalFirst = certificate.first;
            auto canonicalLast = certificate.last;
            for (int axis = 0; axis < canonicalFirst.count; ++axis) {
                canonicalFirst.values[axis] -= certificate.canonicalAxisOffset[axis];
                canonicalLast.values[axis] -= certificate.canonicalAxisOffset[axis];
            }
            for (const auto& interval : certificate.unknownIntervals) {
                if (!adaptivelyCertifyCoalInterval(
                        engine, canonicalFirst, canonicalLast,
                        interval.parameterFirst, interval.parameterLast,
                        interval.limitingPair, 0, 12,
                        options.deviationTolerance, &workload.coal)) {
                    allCertified = false;
                    break;
                }
            }
            if (allCertified) {
                certificate.state =
                    lcnc::cam_algo::MachineMotionCertificateState::CertifiedSafe;
                certificate.unknownIntervals.clear();
            }
        }
        if (certificate.state
            == lcnc::cam_algo::MachineMotionCertificateState::CertifiedSafe) {
            ++workload.finalSafeEdges;
        } else {
            ++workload.finalBlockedEdges;
        }
        workload.certificates.push_back(std::move(certificate));
    }
    const auto end = std::chrono::steady_clock::now();
    workload.buildMs = std::chrono::duration<double, std::milli>(end - start).count();
    return workload;
}

std::vector<double> benchmarkMotionCertificateLookup(
    const MotionCertificateWorkload& workload,
    const std::vector<MotionEdgeSample>& edges,
    std::uint64_t* matched)
{
    std::vector<double> timings;
    timings.reserve(workload.certificates.size());
    *matched = 0;
    for (std::size_t index = 0; index < workload.certificates.size(); ++index) {
        const auto& certificate = workload.certificates[index];
        const auto actual = interpolatedPose(edges[index].first,
                                             edges[index].last, 0.5);
        const auto start = std::chrono::steady_clock::now();
        const bool allowed = lcnc::cam_algo::machinePoseMatchesLinearMotionCertificate(
            certificate, certificate.key, 0.5, actual);
        const auto end = std::chrono::steady_clock::now();
        if (allowed)
            ++(*matched);
        timings.push_back(std::chrono::duration<double, std::micro>(
                              end - start).count());
    }
    return timings;
}

void printWorkload(const char* name, const WorkloadStats& stats)
{
    const double fallbackPercent = stats.total == 0 ? 0.0
        : 100.0 * stats.lmsiUnknown / static_cast<double>(stats.total);
    const double meanPairs = stats.lmsiUnknown == 0 ? 0.0
        : static_cast<double>(stats.evaluatedPairs) / stats.lmsiUnknown;
    const double totalUs = std::accumulate(
        stats.timingsUs.begin(), stats.timingsUs.end(), 0.0);
    const double coalTotalUs = std::accumulate(
        stats.coalTimingsUs.begin(), stats.coalTimingsUs.end(), 0.0);
    std::cout << name << "_poses=" << stats.total << '\n'
              << name << "_lmsi_safe=" << stats.lmsiSafe << '\n'
              << name << "_lmsi_collision=" << stats.lmsiCollision << '\n'
              << name << "_coal_fallback=" << stats.lmsiUnknown << '\n'
              << name << "_coal_fallback_percent=" << fallbackPercent << '\n'
              << name << "_coal_safe=" << stats.coalSafe << '\n'
              << name << "_coal_blocked=" << stats.coalBlocked << '\n'
              << name << "_coal_mean_pairs=" << meanPairs << '\n'
              << name << "_lmsi_p99_us="
              << percentile(stats.lmsiTimingsUs, 0.99) << '\n'
              << name << "_coal_p50_us="
              << percentile(stats.coalTimingsUs, 0.50) << '\n'
              << name << "_coal_p95_us="
              << percentile(stats.coalTimingsUs, 0.95) << '\n'
              << name << "_coal_p99_us="
              << percentile(stats.coalTimingsUs, 0.99) << '\n'
              << name << "_coal_total_ms=" << (coalTotalUs / 1000.0) << '\n'
              << name << "_mean_us="
              << (stats.total == 0 ? 0.0 : totalUs / stats.total) << '\n'
              << name << "_total_ms=" << (totalUs / 1000.0) << '\n'
              << name << "_p50_us=" << percentile(stats.timingsUs, 0.50) << '\n'
              << name << "_p95_us=" << percentile(stats.timingsUs, 0.95) << '\n'
              << name << "_p99_us=" << percentile(stats.timingsUs, 0.99) << '\n'
              << name << "_p999_us=" << percentile(stats.timingsUs, 0.999) << '\n'
              << name << "_max_us="
              << *std::max_element(stats.timingsUs.begin(),
                                   stats.timingsUs.end()) << '\n';
}

QString commandLineIndexPath(const QStringList& arguments)
{
    for (int index = 1; index + 1 < arguments.size(); ++index) {
        if (arguments.at(index) == QStringLiteral("--index"))
            return arguments.at(index + 1);
    }
    return {};
}

QString commandLineMeshSource(const QStringList& arguments)
{
    for (int index = 1; index + 1 < arguments.size(); ++index) {
        if (arguments.at(index) == QStringLiteral("--mesh-source"))
            return arguments.at(index + 1).trimmed().toLower();
    }
    return QStringLiteral("step");
}

bool hasCommandLineOption(const QStringList& arguments, const QString& option)
{
    return arguments.contains(option);
}

} // namespace

int main(int argc, char** argv)
{
    QCoreApplication application(argc, argv);
    QString error;
    const QString modelPath = QString::fromUtf8(LCNC_AC_TABLE_MODEL_PATH);
    const QString indexPath = commandLineIndexPath(application.arguments());
    const QString meshSource = commandLineMeshSource(application.arguments());
    if (meshSource != QStringLiteral("step")
        && meshSource != QStringLiteral("persisted")) {
        std::cerr << "--mesh-source must be step or persisted\n";
        return 1;
    }
    const bool useCachedGuess = hasCommandLineOption(
        application.arguments(), QStringLiteral("--gjk-warm-start"));
    lcnc::cam_algo::MachineSafetyIndex safetyIndex;
    qint64 indexLoadMs = 0;
    if (!indexPath.isEmpty()) {
        QElapsedTimer indexLoadTimer;
        indexLoadTimer.start();
        if (!safetyIndex.load(indexPath, &error)) {
            std::cerr << "无法加载 LMSI: " << error.toStdString() << '\n';
            return 1;
        }
        indexLoadMs = indexLoadTimer.elapsed();
        QFile source(modelPath);
        if (!source.open(QIODevice::ReadOnly)
            || QCryptographicHash::hash(source.readAll(), QCryptographicHash::Sha256)
                != safetyIndex.sourceSha256()) {
            std::cerr << "LMSI 与 STEP 机台源文件指纹不一致\n";
            return 1;
        }
    }
    if (meshSource == QStringLiteral("persisted") && indexPath.isEmpty()) {
        std::cerr << "--mesh-source persisted requires --index\n";
        return 1;
    }
    const auto options = lcnc::cam_algo::defaultAcTableSafetyBuildOptions();
    const auto& queryAxes = indexPath.isEmpty() ? options.axes : safetyIndex.axes();
    std::vector<Body> bodies;
    QElapsedTimer initializationTimer;
    initializationTimer.start();
    QElapsedTimer modelBuildTimer;
    modelBuildTimer.start();
    int persistedBodyCount = 0;
    int stepMeshBodyCount = 0;
    const bool modelsReady = meshSource == QStringLiteral("persisted")
        ? buildBackendModelsFromPersistedIndex(
              safetyIndex, modelPath, &bodies, &persistedBodyCount,
              &stepMeshBodyCount, &error)
        : (loadMachineBodies(modelPath, &bodies, &error)
           && buildBackendModels(&bodies, &error));
    if (meshSource == QStringLiteral("step"))
        stepMeshBodyCount = static_cast<int>(bodies.size());
    const qint64 modelBuildMs = modelBuildTimer.elapsed();
    if (!modelsReady) {
        std::cerr << error.toStdString() << '\n';
        return 1;
    }
    QueryEngine engine;
    if (!prepareBodyChains(queryAxes, &bodies, &error)
        || !prepareQueryEngine(bodies, queryAxes, options.axisPairs,
                               useCachedGuess,
                               &engine, &error)) {
        std::cerr << error.toStdString() << '\n';
        return 2;
    }
    if (!indexPath.isEmpty()
        && !queryEngineMatchesIndexPairs(engine, safetyIndex, &error)) {
        std::cerr << error.toStdString() << '\n';
        return 2;
    }
    const qint64 initializationMs = initializationTimer.elapsed();
    std::vector<AuditSample> samples;
    if (!loadAuditSamples(QString::fromUtf8(LCNC_UNKNOWN_AUDIT_PATH),
                           queryAxes, &samples, &error)) {
        std::cerr << error.toStdString() << '\n';
        return 2;
    }

    std::size_t totalTriangles = 0;
    double minimumMeshDeflectionMm = std::numeric_limits<double>::infinity();
    double maximumMeshDeflectionMm = 0.0;
    for (const Body& body : bodies)
        totalTriangles += body.triangleCount;
    for (const Body& body : bodies) {
        minimumMeshDeflectionMm = std::min(minimumMeshDeflectionMm,
                                            body.meshDeflectionMm);
        maximumMeshDeflectionMm = std::max(maximumMeshDeflectionMm,
                                            body.meshDeflectionMm);
    }

    int exactSafe = 0;
    int exactCollisions = 0;
    int recoveredSafe = 0;
    int falseSafe = 0;
    int collisionBlocked = 0;
    int nearSafeRecovered = 0;
    for (const AuditSample& sample : samples) {
        const QueryResult result = queryPose(&engine, sample.pose);
        if (sample.exactCollision) {
            ++exactCollisions;
            if (result.certifiedSafe)
                ++falseSafe;
            else
                ++collisionBlocked;
        } else {
            ++exactSafe;
            if (result.certifiedSafe) {
                ++recoveredSafe;
                if (sample.exactNear)
                    ++nearSafeRecovered;
            }
        }
    }

    constexpr int kTimingRepeats = 20;
    std::vector<double> timingsUs;
    timingsUs.reserve(samples.size() * kTimingRepeats);
    for (int repeat = 0; repeat < kTimingRepeats; ++repeat) {
        for (const AuditSample& sample : samples) {
            const auto start = std::chrono::steady_clock::now();
            const QueryResult result = queryPose(&engine, sample.pose);
            const auto end = std::chrono::steady_clock::now();
            if (result.certifiedSafe && result.blockedByThreshold)
                return 3;
            timingsUs.push_back(std::chrono::duration<double, std::micro>(
                                    end - start).count());
        }
    }

    std::cout << "backend=" << kBackendName << '\n'
              << "mesh_source=" << meshSource.toStdString() << '\n'
              << "gjk_warm_start=" << (useCachedGuess ? 1 : 0) << '\n'
              << "machine_bodies=" << bodies.size() << '\n'
              << "persisted_mesh_bodies=" << persistedBodyCount << '\n'
              << "step_mesh_bodies=" << stepMeshBodyCount << '\n'
              << "triangles=" << totalTriangles << '\n'
              << "mesh_deflection_min_mm=" << minimumMeshDeflectionMm << '\n'
              << "mesh_deflection_max_mm=" << maximumMeshDeflectionMm << '\n'
              << "certificate_threshold_min_mm="
              << (kRequiredClearanceMm + 2.0 * minimumMeshDeflectionMm) << '\n'
              << "certificate_threshold_max_mm="
              << (kRequiredClearanceMm + 2.0 * maximumMeshDeflectionMm) << '\n'
              << "index_load_ms=" << indexLoadMs << '\n'
              << "model_build_ms=" << modelBuildMs << '\n'
              << "initialization_ms=" << initializationMs << '\n'
              << "audit_samples=" << samples.size() << '\n'
              << "exact_safe=" << exactSafe << '\n'
              << "exact_collision=" << exactCollisions << '\n'
              << "safe_recovered=" << recoveredSafe << '\n'
              << "safe_recovery_percent="
              << (100.0 * recoveredSafe / std::max(1, exactSafe)) << '\n'
              << "near_safe_recovered=" << nearSafeRecovered << '\n'
              << "collision_blocked=" << collisionBlocked << '\n'
              << "false_safe=" << falseSafe << '\n'
              << "query_p50_us=" << percentile(timingsUs, 0.50) << '\n'
              << "query_p95_us=" << percentile(timingsUs, 0.95) << '\n'
              << "query_p99_us=" << percentile(timingsUs, 0.99) << '\n'
              << "query_max_us=" << *std::max_element(timingsUs.begin(),
                                                         timingsUs.end()) << '\n';
    if (falseSafe != 0 || collisionBlocked != exactCollisions) {
        std::cerr << "Fail-closed safety regression in mesh backend\n";
        return 4;
    }
    if (!indexPath.isEmpty()) {
        int combinedFalseSafe = 0;
        int combinedSafeRecovered = 0;
        int combinedCollisionBlocked = 0;
        WorkloadStats combinedAuditStats;
        combinedAuditStats.total = samples.size();
        for (const AuditSample& sample : samples) {
            const CombinedDecision decision = combinedQuery(
                safetyIndex, &engine, sample.pose, &combinedAuditStats);
            if (sample.exactCollision) {
                if (decision.allowed)
                    ++combinedFalseSafe;
                else
                    ++combinedCollisionBlocked;
            } else if (decision.allowed) {
                ++combinedSafeRecovered;
            }
        }
        const auto randomPoses = makeGlobalRandomPoses(queryAxes, 100'000);
        const auto offlineComparePoses = makeGlobalRandomPoses(
            queryAxes, 1'000);
        const OfflineCoalCoverageComparison offlineCoverage =
            compareOfflineCoalCoverage(offlineComparePoses, safetyIndex,
                                       &engine);
        int offlineCurrentLeafFalseSafe = 0;
        int offlineHotChildFalseSafe = 0;
        int offlineCurrentLeafSafeRecovered = 0;
        int offlineHotChildSafeRecovered = 0;
        for (const AuditSample& sample : samples) {
            const auto indexed = safetyIndex.query(sample.pose);
            if (indexed.state
                == lcnc::cam_algo::MachineSafetyIndexState::CertifiedSafe) {
                continue;
            }
            if (indexed.state
                == lcnc::cam_algo::MachineSafetyIndexState::CollisionSample) {
                continue;
            }
            const bool currentLeafSafe = coalCertifiesContainingSafetyCell(
                &engine, sample.pose, indexed.refinementLevel, nullptr);
            bool hotChildSafe = false;
            if (!currentLeafSafe) {
                hotChildSafe = coalCertifiesContainingSafetyCell(
                    &engine, sample.pose, 3, nullptr);
            }
            if (sample.exactCollision) {
                if (currentLeafSafe)
                    ++offlineCurrentLeafFalseSafe;
                if (hotChildSafe)
                    ++offlineHotChildFalseSafe;
            } else {
                if (currentLeafSafe)
                    ++offlineCurrentLeafSafeRecovered;
                if (hotChildSafe)
                    ++offlineHotChildSafeRecovered;
            }
        }
        const auto continuousPoses = makeContinuousStressPoses(samples, 128);
        const WorkloadStats randomStats = runCombinedWorkload(
            randomPoses, safetyIndex, &engine);
        const WorkloadStats continuousStats = runCombinedWorkload(
            continuousPoses, safetyIndex, &engine);
        const PathDecisionCache pathCache = buildPathDecisionCache(
            continuousPoses, safetyIndex, &engine);
        const WorkloadStats pathLookupStats = benchmarkPathCacheLookup(pathCache);
        std::vector<MotionEdgeSample> realEdges;
        int realContourCount = 0;
        int realPointCount = 0;
        QElapsedTimer realPathTimer;
        realPathTimer.start();
        if (!loadHalfSphereMotionEdges(
                QString::fromUtf8(LCNC_HALF_SPHERE_MODEL_PATH), queryAxes,
                &realEdges, &realContourCount, &realPointCount, &error)) {
            std::cerr << error.toStdString() << '\n';
            return 5;
        }
        const qint64 realPathBuildMs = realPathTimer.elapsed();
        lcnc::cam_algo::MachineMotionCertificationOptions motionOptions;
        motionOptions.deviationTolerance = {
            0.02, 0.02, 0.02, 0.01, 0.01};
        const MotionCertificateWorkload motionCertificates =
            buildMotionCertificates(realEdges, safetyIndex, &engine,
                                    motionOptions, 1);
        std::vector<MotionEdgeSample> auditEdges;
        auditEdges.reserve(samples.size());
        for (const AuditSample& sample : samples)
            auditEdges.push_back({sample.pose, sample.pose});
        lcnc::cam_algo::MachineMotionCertificationOptions auditMotionOptions;
        const MotionCertificateWorkload auditMotionCertificates =
            buildMotionCertificates(auditEdges, safetyIndex, &engine,
                                    auditMotionOptions, 2);
        int motionAuditFalseSafe = 0;
        int motionAuditSafeRecovered = 0;
        int motionAuditCollisionBlocked = 0;
        for (std::size_t index = 0; index < samples.size(); ++index) {
            const bool allowed = auditMotionCertificates.certificates[index].state
                == lcnc::cam_algo::MachineMotionCertificateState::CertifiedSafe;
            if (samples[index].exactCollision) {
                if (allowed)
                    ++motionAuditFalseSafe;
                else
                    ++motionAuditCollisionBlocked;
            } else if (allowed) {
                ++motionAuditSafeRecovered;
            }
        }
        std::uint64_t matchedCertificates = 0;
        const std::vector<double> motionLookupTimings =
            benchmarkMotionCertificateLookup(
                motionCertificates, realEdges, &matchedCertificates);
        bool invalidationPassed = true;
        for (std::size_t index = 0;
             index < motionCertificates.certificates.size(); ++index) {
            const auto& certificate = motionCertificates.certificates[index];
            if (certificate.state
                != lcnc::cam_algo::MachineMotionCertificateState::CertifiedSafe)
                continue;
            auto midpoint = interpolatedPose(realEdges[index].first,
                                             realEdges[index].last, 0.5);
            midpoint.values[0] += motionOptions.deviationTolerance[0] + 0.001;
            auto staleKey = certificate.key;
            ++staleKey.pathRevision;
            invalidationPassed =
                !lcnc::cam_algo::machinePoseMatchesLinearMotionCertificate(
                    certificate, certificate.key, 0.5, midpoint)
                && !lcnc::cam_algo::machinePoseMatchesLinearMotionCertificate(
                    certificate, staleKey, 0.5,
                    interpolatedPose(realEdges[index].first,
                                     realEdges[index].last, 0.5));
            break;
        }
        std::cout << "lmsi_path=" << indexPath.toStdString() << '\n'
                  << "lmsi_bytes=" << QFileInfo(indexPath).size() << '\n'
                  << "lmsi_decision_coverage_percent="
                  << (safetyIndex.decisionCoverage() * 100.0) << '\n'
                  << "combined_audit_lmsi_safe=" << combinedAuditStats.lmsiSafe << '\n'
                  << "combined_audit_coal_fallback="
                  << combinedAuditStats.lmsiUnknown << '\n'
                  << "combined_audit_safe_recovered=" << combinedSafeRecovered << '\n'
                  << "combined_audit_collision_blocked="
                  << combinedCollisionBlocked << '\n'
                  << "combined_audit_false_safe=" << combinedFalseSafe << '\n';
        const double profileACoverage = offlineCoverage.total == 0 ? 0.0
            : 100.0 * offlineCoverage.lmsiSafe / offlineCoverage.total;
        const double profileBCoverage = offlineCoverage.total == 0 ? 0.0
            : 100.0 * (offlineCoverage.lmsiSafe
                       + offlineCoverage.coalCurrentLeafSafe)
                / offlineCoverage.total;
        const double profileCCoverage = offlineCoverage.total == 0 ? 0.0
            : 100.0 * (offlineCoverage.lmsiSafe
                       + offlineCoverage.coalCurrentLeafSafe
                       + offlineCoverage.coalHotChildSafe)
                / offlineCoverage.total;
        std::cout << "offline_compare_samples=" << offlineCoverage.total << '\n'
                  << "offline_cell_coal_motion_budget_mm=0.25\n"
                  << "offline_profile_c_refinement_level=3\n"
                  << "offline_profile_a_lmsi_coverage_pct="
                  << profileACoverage << '\n'
                  << "offline_profile_b_lmsi_coal_coverage_pct="
                  << profileBCoverage << '\n'
                  << "offline_profile_b_gain_points="
                  << (profileBCoverage - profileACoverage) << '\n'
                  << "offline_profile_c_hot_level_coverage_pct="
                  << profileCCoverage << '\n'
                  << "offline_profile_c_gain_over_a_points="
                  << (profileCCoverage - profileACoverage) << '\n'
                  << "offline_profile_c_gain_over_b_points="
                  << (profileCCoverage - profileBCoverage) << '\n'
                  << "offline_profile_b_new_safe_samples="
                  << offlineCoverage.coalCurrentLeafSafe << '\n'
                  << "offline_profile_c_new_safe_samples="
                  << offlineCoverage.coalHotChildSafe << '\n'
                  << "offline_remaining_unknown_samples="
                  << offlineCoverage.remainingUnknown << '\n'
                  << "offline_profile_b_pair_queries="
                  << offlineCoverage.currentLeafPairQueries << '\n'
                  << "offline_profile_c_pair_queries="
                  << offlineCoverage.hotChildPairQueries << '\n'
                  << "offline_profile_b_query_ms="
                  << (offlineCoverage.currentLeafNs / 1'000'000.0) << '\n'
                  << "offline_profile_c_query_ms="
                  << (offlineCoverage.hotChildNs / 1'000'000.0) << '\n'
                  << "offline_profile_b_audit_safe_recovered="
                  << offlineCurrentLeafSafeRecovered << '\n'
                  << "offline_profile_c_audit_safe_recovered="
                  << offlineHotChildSafeRecovered << '\n'
                  << "offline_profile_b_audit_false_safe="
                  << offlineCurrentLeafFalseSafe << '\n'
                  << "offline_profile_c_audit_false_safe="
                  << offlineHotChildFalseSafe << '\n';
        printWorkload("global", randomStats);
        printWorkload("continuous_stress", continuousStats);
        const double pathLookupP99Us = percentile(
            pathLookupStats.timingsUs, 0.99);
        std::cout << "path_cache_build_ms=" << pathCache.buildMs << '\n'
                  << "path_cache_entries=" << pathCache.allowed.size() << '\n'
                  << "path_cache_bytes_unpacked=" << pathCache.allowed.size() << '\n'
                  << "path_cache_bytes_bitpacked="
                  << ((pathCache.allowed.size() + 7) / 8) << '\n'
                  << "path_cache_build_coal_fallback="
                  << pathCache.buildStats.lmsiUnknown << '\n'
                  << "path_cache_lookup_p50_us="
                  << percentile(pathLookupStats.timingsUs, 0.50) << '\n'
                  << "path_cache_lookup_p95_us="
                  << percentile(pathLookupStats.timingsUs, 0.95) << '\n'
                  << "path_cache_lookup_p99_us="
                  << pathLookupP99Us << '\n'
                  << "path_cache_lookup_max_us="
                  << *std::max_element(pathLookupStats.timingsUs.begin(),
                                       pathLookupStats.timingsUs.end()) << '\n';
        const double motionLookupP99Us = percentile(motionLookupTimings, 0.99);
        const double realSafePercent = realEdges.empty() ? 0.0
            : 100.0 * motionCertificates.finalSafeEdges / realEdges.size();
        std::cout << "real_workpiece=半球.stp\n"
                  << "real_contours=" << realContourCount << '\n'
                  << "real_solved_points=" << realPointCount << '\n'
                  << "real_motion_edges=" << realEdges.size() << '\n'
                  << "real_cam_path_build_ms=" << realPathBuildMs << '\n'
                  << "motion_tube_xyz_mm=0.02\n"
                  << "motion_tube_ac_deg=0.01\n"
                  << "motion_lmsi_safe_edges="
                  << motionCertificates.lmsiSafeEdges << '\n'
                  << "motion_lmsi_unknown_edges="
                  << motionCertificates.lmsiUnknownEdges << '\n'
                  << "motion_lmsi_blocked_edges="
                  << motionCertificates.lmsiBlockedEdges << '\n'
                  << "motion_final_safe_edges="
                  << motionCertificates.finalSafeEdges << '\n'
                  << "motion_final_blocked_edges="
                  << motionCertificates.finalBlockedEdges << '\n'
                  << "motion_final_safe_percent=" << realSafePercent << '\n'
                  << "motion_lmsi_visited_fine_cells="
                  << motionCertificates.lmsiVisitedCells << '\n'
                  << "motion_lmsi_parameter_intervals="
                  << motionCertificates.lmsiParameterIntervals << '\n'
                  << "motion_coal_interval_queries="
                  << motionCertificates.coal.intervalQueries << '\n'
                  << "motion_coal_pair_queries="
                  << motionCertificates.coal.pairQueries << '\n'
                  << "motion_coal_splits="
                  << motionCertificates.coal.splits << '\n'
                  << "motion_coal_blocked_leaves="
                  << motionCertificates.coal.blockedLeaves << '\n'
                  << "motion_coal_max_depth="
                  << motionCertificates.coal.maximumDepth << '\n'
                  << "motion_certificate_build_ms="
                  << motionCertificates.buildMs << '\n'
                  << "motion_certificate_static_bytes="
                  << motionCertificates.certificates.size()
                         * sizeof(lcnc::cam_algo::MachineMotionEdgeCertificate)
                  << '\n'
                  << "motion_certificate_lookup_matched="
                  << matchedCertificates << '\n'
                  << "motion_certificate_lookup_p99_us="
                  << motionLookupP99Us << '\n'
                  << "motion_certificate_invalidation_passed="
                  << (invalidationPassed ? 1 : 0) << '\n'
                  << "motion_audit_safe_recovered="
                  << motionAuditSafeRecovered << '\n'
                  << "motion_audit_collision_blocked="
                  << motionAuditCollisionBlocked << '\n'
                  << "motion_audit_false_safe="
                  << motionAuditFalseSafe << '\n';
        if (combinedFalseSafe != 0
            || combinedCollisionBlocked != exactCollisions
            || offlineCurrentLeafFalseSafe != 0
            || offlineHotChildFalseSafe != 0) {
            std::cerr << "LMSI + Coal 联合决策出现安全回归\n";
            return 5;
        }
        constexpr qint64 kMaximumPackageBytes = 100LL * 1024 * 1024;
        if (QFileInfo(indexPath).size() > kMaximumPackageBytes
            || pathLookupP99Us > 10.0) {
            std::cerr << "LMSI 包体积或轨迹缓存实时查询门禁失败\n";
            return 6;
        }
        if (realSafePercent < 99.0 || motionLookupP99Us > 10.0
            || !invalidationPassed || motionAuditFalseSafe != 0
            || motionAuditCollisionBlocked != exactCollisions) {
            std::cerr << "真实刀路 Motion Certificate 生产门禁失败\n";
            return 7;
        }
    }
    return 0;
}
