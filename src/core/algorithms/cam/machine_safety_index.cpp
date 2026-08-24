#include "core/algorithms/cam/machine_safety_index.h"

#include "core/algorithms/cam/surface_collision_prefilter.h"
#include "core/algorithms/occt_exact_operation_lock.h"
#include "core/logging/logger.h"

#include <BRepBndLib.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <Bnd_OBB.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Precision.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <Standard_Failure.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDataStd_Name.hxx>
#include <TDocStd_Document.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopTools_MapOfShape.hxx>
#include <TopoDS_Shape.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_Location.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include <QBuffer>
#include <QCryptographicHash>
#include <QDataStream>
#include <QElapsedTimer>
#include <QFile>
#include <QFileInfo>
#include <QSaveFile>
#include <QSet>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <mutex>
#include <numeric>
#include <thread>
#include <utility>

namespace lcnc::cam_algo {
namespace {

constexpr char kIndexMagic[8] = {'L', 'C', 'N', 'C', 'M', 'S', 'I', '1'};
constexpr quint32 kIndexSchema = 5;
constexpr int kChecksumBytes = 32;
constexpr double kPoseTolerance = 1e-9;
constexpr double kPi = 3.14159265358979323846;

struct SourceBody
{
    struct Leaf
    {
        TopoDS_Shape shape;
        Bnd_Box localAabb;
        Bnd_OBB localObb;
    };

    struct LeafBvhNode
    {
        Bnd_Box localAabb;
        int left{-1};
        int right{-1};
        int leaf{-1};
    };

    QString name;
    QString axisName;
    TopoDS_Shape shape;
    Bnd_Box localAabb;
    QVector<Leaf> leaves;
    QVector<LeafBvhNode> leafBvh;
    SurfaceCollisionModel surfaceModel;
};

struct ExactCandidate
{
    std::uint64_t cellIndex{0};
    int pairIndex{-1};
    double overlapVolume{0.0};
};

struct LeafPairCandidate
{
    int firstLeaf{-1};
    int secondLeaf{-1};
    double overlap{0.0};
};

struct PairExactEvaluation
{
    bool done{false};
    double distanceMm{std::numeric_limits<double>::infinity()};
    std::uint64_t leafCandidatePairs{0};
    std::uint64_t leafExactQueries{0};
    std::uint64_t wholeShapeExactQueries{0};
};

struct DirectionalMotionBound
{
    std::array<double, 3> components{};
};

double scalarMagnitude(const DirectionalMotionBound& bound)
{
    return std::sqrt(bound.components[0] * bound.components[0]
        + bound.components[1] * bound.components[1]
        + bound.components[2] * bound.components[2]);
}

QString labelName(const TDF_Label& label)
{
    Handle(TDataStd_Name) attribute;
    if (!label.FindAttribute(TDataStd_Name::GetID(), attribute))
        return {};
    return QString::fromStdU16String(
        reinterpret_cast<const char16_t*>(attribute->Get().ToExtString()));
}

QString axisFromPartName(const QString& name)
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

std::array<double, 3> boxCenterCoordinates(const Bnd_Box& box)
{
    Standard_Real xmin = 0.0, ymin = 0.0, zmin = 0.0;
    Standard_Real xmax = 0.0, ymax = 0.0, zmax = 0.0;
    box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    return {(xmin + xmax) * 0.5,
            (ymin + ymax) * 0.5,
            (zmin + zmax) * 0.5};
}

void buildLeafBvh(SourceBody* body)
{
    if (!body || body->leaves.isEmpty())
        return;
    QVector<int> indices(body->leaves.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::function<int(int, int)> build = [&](int begin, int end) -> int {
        SourceBody::LeafBvhNode node;
        for (int offset = begin; offset < end; ++offset)
            node.localAabb.Add(body->leaves.at(indices.at(offset)).localAabb);
        const int nodeIndex = body->leafBvh.size();
        body->leafBvh.append(node);
        if (end - begin == 1) {
            body->leafBvh[nodeIndex].leaf = indices.at(begin);
            return nodeIndex;
        }
        Standard_Real xmin = 0.0, ymin = 0.0, zmin = 0.0;
        Standard_Real xmax = 0.0, ymax = 0.0, zmax = 0.0;
        node.localAabb.Get(xmin, ymin, zmin, xmax, ymax, zmax);
        const std::array<double, 3> extents{xmax - xmin, ymax - ymin, zmax - zmin};
        const int splitAxis = static_cast<int>(std::distance(
            extents.begin(), std::max_element(extents.begin(), extents.end())));
        const int middle = begin + (end - begin) / 2;
        std::nth_element(indices.begin() + begin, indices.begin() + middle,
            indices.begin() + end, [&](int lhs, int rhs) {
                return boxCenterCoordinates(body->leaves.at(lhs).localAabb)[splitAxis]
                    < boxCenterCoordinates(body->leaves.at(rhs).localAabb)[splitAxis];
            });
        const int left = build(begin, middle);
        const int right = build(middle, end);
        body->leafBvh[nodeIndex].left = left;
        body->leafBvh[nodeIndex].right = right;
        return nodeIndex;
    };
    build(0, indices.size());
}

bool hashFile(const QString& filePath, QByteArray* digest, QString* errorMessage)
{
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Unable to open machine file for hashing: %1")
                                .arg(file.errorString());
        return false;
    }
    QCryptographicHash hash(QCryptographicHash::Sha256);
    if (!hash.addData(&file)) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Unable to hash machine file");
        return false;
    }
    *digest = hash.result();
    return true;
}

bool appendRootOrComponents(const Handle(XCAFDoc_ShapeTool)& shapes,
                            const TDF_Label& root,
                            int rootIndex,
                            QVector<SourceBody>* bodies)
{
    TDF_LabelSequence components;
    shapes->GetComponents(root, components);
    if (components.IsEmpty()) {
        const TopoDS_Shape shape = shapes->GetShape(root);
        if (shape.IsNull())
            return true;
        QString name = labelName(root);
        if (name.isEmpty())
            name = QStringLiteral("Part_%1").arg(rootIndex);
        const QString axis = axisFromPartName(name);
        if (!axis.isEmpty())
            bodies->append({name, axis, shape, {}});
        return true;
    }

    for (int componentIndex = 1; componentIndex <= components.Length(); ++componentIndex) {
        const TDF_Label component = components.Value(componentIndex);
        QString name = labelName(component);
        TopLoc_Location location;
        Handle(XCAFDoc_Location) locationAttribute;
        if (component.FindAttribute(XCAFDoc_Location::GetID(), locationAttribute))
            location = locationAttribute->Get();

        TDF_Label referred;
        TopoDS_Shape shape;
        if (shapes->GetReferredShape(component, referred)) {
            if (name.isEmpty())
                name = labelName(referred);
            shape = shapes->GetShape(referred);
        } else {
            shape = shapes->GetShape(component);
        }
        if (shape.IsNull())
            continue;
        if (!location.IsIdentity())
            shape = shape.Located(location * shape.Location());
        if (name.isEmpty())
            name = QStringLiteral("Part_%1_%2").arg(rootIndex).arg(componentIndex);
        const QString axis = axisFromPartName(name);
        if (!axis.isEmpty())
            bodies->append({name, axis, shape, {}});
    }
    return true;
}

bool readMachineBodies(const QString& filePath,
                       QVector<SourceBody>* bodies,
                       QString* errorMessage)
{
    if (!bodies)
        return false;
    bodies->clear();
    Handle(TDocStd_Document) document =
        new TDocStd_Document(TCollection_ExtendedString("BinXCAF"));
    XCAFDoc_DocumentTool::Set(document->Main());
    STEPCAFControl_Reader reader;
    reader.SetNameMode(Standard_True);
    if (reader.ReadFile(filePath.toUtf8().constData()) != IFSelect_RetDone
        || !reader.Transfer(document)) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Unable to read or transfer STEP machine model");
        return false;
    }
    const Handle(XCAFDoc_ShapeTool) shapes =
        XCAFDoc_DocumentTool::ShapeTool(document->Main());
    TDF_LabelSequence roots;
    shapes->GetFreeShapes(roots);
    for (int index = 1; index <= roots.Length(); ++index)
        appendRootOrComponents(shapes, roots.Value(index), index, bodies);
    if (bodies->isEmpty()) {
        if (errorMessage) {
            *errorMessage = QStringLiteral(
                "Machine model contains no LCNC_AXIS_* named bodies");
        }
        return false;
    }
    for (SourceBody& body : *bodies) {
        BRepBndLib::AddOptimal(body.shape, body.localAabb,
                               Standard_False, Standard_False);
        if (body.localAabb.IsVoid()) {
            if (errorMessage)
                *errorMessage = QStringLiteral("Machine body has no usable bounds: %1")
                                    .arg(body.name);
            return false;
        }
        TopTools_MapOfShape seen;
        const auto appendLeaf = [&body, &seen](const TopoDS_Shape& shape) {
            if (shape.IsNull() || !seen.Add(shape))
                return;
            SourceBody::Leaf leaf;
            leaf.shape = shape;
            BRepBndLib::AddOptimal(shape, leaf.localAabb,
                                   Standard_False, Standard_False);
            BRepBndLib::AddOBB(shape, leaf.localObb,
                               Standard_False, Standard_False, Standard_False);
            if (!leaf.localAabb.IsVoid() && !leaf.localObb.IsVoid())
                body.leaves.append(std::move(leaf));
        };
        for (TopExp_Explorer explorer(body.shape, TopAbs_FACE);
             explorer.More(); explorer.Next()) {
            appendLeaf(explorer.Current());
        }
        if (body.leaves.isEmpty())
            appendLeaf(body.shape);
        buildLeafBvh(&body);
    }
    return true;
}

int axisIndex(const QVector<MachineSafetyAxisGrid>& axes, const QString& name)
{
    for (int index = 0; index < axes.size(); ++index) {
        if (axes.at(index).name.compare(name, Qt::CaseInsensitive) == 0)
            return index;
    }
    return -1;
}

gp_Trsf axisLocalTransform(const MachineSafetyAxisGrid& axis, double value)
{
    gp_Trsf transform;
    const gp_Dir direction(axis.direction[0], axis.direction[1], axis.direction[2]);
    if (axis.rotary) {
        const gp_Pnt origin(axis.origin[0], axis.origin[1], axis.origin[2]);
        transform.SetRotation(gp_Ax1(origin, direction), value * kPi / 180.0);
    } else {
        transform.SetTranslation(gp_Vec(direction) * value);
    }
    return transform;
}

gp_Trsf axisChainTransform(const QVector<MachineSafetyAxisGrid>& axes,
                           const MachineSafetyPose& pose,
                           const QString& axisName)
{
    QStringList chain;
    QString current = axisName;
    QSet<QString> seen;
    while (!current.isEmpty() && current.compare(QStringLiteral("BASE"),
                                                  Qt::CaseInsensitive) != 0) {
        if (seen.contains(current))
            break;
        seen.insert(current);
        chain.prepend(current);
        const int index = axisIndex(axes, current);
        if (index < 0)
            break;
        current = axes.at(index).parentAxis;
    }
    gp_Trsf result;
    for (const QString& name : chain) {
        const int index = axisIndex(axes, name);
        if (index >= 0 && index < pose.count)
            result = result.Multiplied(axisLocalTransform(axes.at(index),
                                                          pose.values[index]));
    }
    return result;
}

QVector<int> axisChainIndices(const QVector<MachineSafetyAxisGrid>& axes,
                              const QString& axisName)
{
    QVector<int> chain;
    QString current = axisName;
    QSet<QString> seen;
    while (!current.isEmpty()
           && current.compare(QStringLiteral("BASE"), Qt::CaseInsensitive) != 0) {
        if (seen.contains(current))
            break;
        seen.insert(current);
        const int index = axisIndex(axes, current);
        if (index < 0)
            break;
        chain.prepend(index);
        current = axes.at(index).parentAxis;
    }
    return chain;
}

std::uint8_t bodyAxisDependencyMask(
    const QVector<MachineSafetyAxisGrid>& axes,
    const QString& axisName)
{
    std::uint8_t result = 0;
    for (const int axis : axisChainIndices(axes, axisName)) {
        if (axis >= 0 && axis < axes.size()) {
            const int childBit = axes.size() - axis - 1;
            result |= static_cast<std::uint8_t>(1u << childBit);
        }
    }
    return result;
}

Bnd_Box transformBox(const Bnd_Box& local, const gp_Trsf& transform)
{
    Bnd_Box result;
    if (local.IsVoid())
        return result;
    Standard_Real xmin = 0.0, ymin = 0.0, zmin = 0.0;
    Standard_Real xmax = 0.0, ymax = 0.0, zmax = 0.0;
    local.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    for (double x : {xmin, xmax}) {
        for (double y : {ymin, ymax}) {
            for (double z : {zmin, zmax}) {
                gp_Pnt point(x, y, z);
                point.Transform(transform);
                result.Add(point);
            }
        }
    }
    return result;
}

Bnd_OBB transformObb(const Bnd_OBB& local,
                     const gp_Trsf& transform,
                     double enlargementMm)
{
    if (local.IsVoid())
        return {};
    gp_Pnt center(local.Center());
    center.Transform(transform);
    gp_Dir xDirection(local.XDirection());
    gp_Dir yDirection(local.YDirection());
    gp_Dir zDirection(local.ZDirection());
    xDirection.Transform(transform);
    yDirection.Transform(transform);
    zDirection.Transform(transform);
    Bnd_OBB result(center, xDirection, yDirection, zDirection,
                   local.XHSize(), local.YHSize(), local.ZHSize());
    result.Enlarge(enlargementMm);
    return result;
}

double maximumRadius(const Bnd_Box& box, const gp_Pnt& origin)
{
    Standard_Real xmin = 0.0, ymin = 0.0, zmin = 0.0;
    Standard_Real xmax = 0.0, ymax = 0.0, zmax = 0.0;
    box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    double result = 0.0;
    for (double x : {xmin, xmax}) {
        for (double y : {ymin, ymax}) {
            for (double z : {zmin, zmax})
                result = std::max(result, origin.Distance(gp_Pnt(x, y, z)));
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
    Standard_Real xmin = 0.0, ymin = 0.0, zmin = 0.0;
    Standard_Real xmax = 0.0, ymax = 0.0, zmax = 0.0;
    box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    const gp_Vec axis(direction);
    double result = 0.0;
    for (double x : {xmin, xmax}) {
        for (double y : {ymin, ymax}) {
            for (double z : {zmin, zmax}) {
                const gp_Vec radial(origin, gp_Pnt(x, y, z));
                result = std::max(result, radial.Crossed(axis).Magnitude());
            }
        }
    }
    return result;
}

double bodyCellMotionBound(const SourceBody& body,
                           const QVector<MachineSafetyAxisGrid>& axes)
{
    double result = 0.0;
    QString current = body.axisName;
    QSet<QString> seen;
    while (!current.isEmpty() && current.compare(QStringLiteral("BASE"),
                                                  Qt::CaseInsensitive) != 0) {
        if (seen.contains(current))
            break;
        seen.insert(current);
        const int index = axisIndex(axes, current);
        if (index < 0)
            break;
        const auto& axis = axes.at(index);
        const double halfWidth = std::min(axis.step,
                                          axis.maximum - axis.minimum) * 0.5;
        if (axis.rotary) {
            const gp_Pnt origin(axis.origin[0], axis.origin[1], axis.origin[2]);
            const double radius = maximumRadius(body.localAabb, origin);
            result += 2.0 * radius * std::sin(std::abs(halfWidth) * kPi / 360.0);
        } else {
            result += std::abs(halfWidth);
        }
        current = axis.parentAxis;
    }
    return result;
}

DirectionalMotionBound bodyCellDirectionalMotionBound(
    const SourceBody& body,
    const QVector<MachineSafetyAxisGrid>& axes,
    const MachineSafetyPose& pose,
    const Bnd_Box& centerBox,
    double cellScale)
{
    DirectionalMotionBound result;
    const QVector<int> chain = axisChainIndices(axes, body.axisName);
    QVector<gp_Trsf> parentTransforms;
    parentTransforms.reserve(chain.size());
    gp_Trsf running;
    for (const int axisValue : chain) {
        parentTransforms.append(running);
        if (axisValue >= 0 && axisValue < pose.count) {
            running = running.Multiplied(axisLocalTransform(
                axes.at(axisValue), pose.values[axisValue]));
        }
    }

    // Process child motion first. When a parent rotates, the accumulated
    // descendant displacement enlarges its possible radius, preserving a
    // conservative bound for simultaneous multi-axis motion.
    // 中文翻译：先累计子轴运动；父轴旋转时把子轴位移加入可能半径，
    // 从而对多轴同时运动仍保持保守上界。
    double descendantScalarBound = 0.0;
    for (int chainOffset = chain.size() - 1; chainOffset >= 0; --chainOffset) {
        const auto& axis = axes.at(chain.at(chainOffset));
        const double halfWidth = std::min(axis.step,
                                          axis.maximum - axis.minimum)
            * 0.5 * cellScale;
        gp_Dir worldDirection(axis.direction[0], axis.direction[1],
                              axis.direction[2]);
        worldDirection.Transform(parentTransforms.at(chainOffset));
        double scalarContribution = std::abs(halfWidth);
        if (axis.rotary) {
            gp_Pnt worldOrigin(axis.origin[0], axis.origin[1], axis.origin[2]);
            worldOrigin.Transform(parentTransforms.at(chainOffset));
            const double radius = maximumRadiusFromAxis(
                centerBox, worldOrigin, worldDirection) + descendantScalarBound;
            scalarContribution = 2.0 * radius
                * std::sin(std::abs(halfWidth) * kPi / 360.0);
            const std::array<double, 3> directionComponents{
                worldDirection.X(), worldDirection.Y(), worldDirection.Z()};
            for (int component = 0; component < 3; ++component) {
                const double perpendicularFactor = std::sqrt(std::max(
                    0.0, 1.0 - directionComponents[component]
                                     * directionComponents[component]));
                result.components[component] +=
                    scalarContribution * perpendicularFactor;
            }
        } else {
            result.components[0] += scalarContribution * std::abs(worldDirection.X());
            result.components[1] += scalarContribution * std::abs(worldDirection.Y());
            result.components[2] += scalarContribution * std::abs(worldDirection.Z());
        }
        descendantScalarBound += scalarContribution;
    }
    return result;
}

Bnd_Box enlargeBoxDirectional(const Bnd_Box& box,
                              const DirectionalMotionBound& bound)
{
    Bnd_Box result;
    if (box.IsVoid())
        return result;
    Standard_Real xmin = 0.0, ymin = 0.0, zmin = 0.0;
    Standard_Real xmax = 0.0, ymax = 0.0, zmax = 0.0;
    box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    result.Update(xmin - bound.components[0],
                  ymin - bound.components[1],
                  zmin - bound.components[2],
                  xmax + bound.components[0],
                  ymax + bound.components[1],
                  zmax + bound.components[2]);
    return result;
}

double boxDistance(const Bnd_Box& first, const Bnd_Box& second)
{
    if (first.IsVoid() || second.IsVoid())
        return std::numeric_limits<double>::infinity();
    Standard_Real ax0 = 0.0, ay0 = 0.0, az0 = 0.0;
    Standard_Real ax1 = 0.0, ay1 = 0.0, az1 = 0.0;
    Standard_Real bx0 = 0.0, by0 = 0.0, bz0 = 0.0;
    Standard_Real bx1 = 0.0, by1 = 0.0, bz1 = 0.0;
    first.Get(ax0, ay0, az0, ax1, ay1, az1);
    second.Get(bx0, by0, bz0, bx1, by1, bz1);
    const double dx = std::max({0.0, ax0 - bx1, bx0 - ax1});
    const double dy = std::max({0.0, ay0 - by1, by0 - ay1});
    const double dz = std::max({0.0, az0 - bz1, bz0 - az1});
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool boxContains(const Bnd_Box& outer, const Bnd_Box& inner)
{
    if (outer.IsVoid() || inner.IsVoid())
        return false;
    Standard_Real ax0 = 0.0, ay0 = 0.0, az0 = 0.0;
    Standard_Real ax1 = 0.0, ay1 = 0.0, az1 = 0.0;
    Standard_Real bx0 = 0.0, by0 = 0.0, bz0 = 0.0;
    Standard_Real bx1 = 0.0, by1 = 0.0, bz1 = 0.0;
    outer.Get(ax0, ay0, az0, ax1, ay1, az1);
    inner.Get(bx0, by0, bz0, bx1, by1, bz1);
    return ax0 <= bx0 && ay0 <= by0 && az0 <= bz0
        && ax1 >= bx1 && ay1 >= by1 && az1 >= bz1;
}

double boxVolume(const Bnd_Box& box)
{
    if (box.IsVoid())
        return 0.0;
    Standard_Real xmin = 0.0, ymin = 0.0, zmin = 0.0;
    Standard_Real xmax = 0.0, ymax = 0.0, zmax = 0.0;
    box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    return std::max(0.0, xmax - xmin)
        * std::max(0.0, ymax - ymin)
        * std::max(0.0, zmax - zmin);
}

bool certifyPairSeparatedByLeafBvh(
    const SourceBody& firstBody,
    const gp_Trsf& firstTransform,
    const SourceBody& secondBody,
    const gp_Trsf& secondTransform,
    const QVector<MachineSafetyAxisGrid>& axes,
    const MachineSafetyPose& pose,
    double cellScale,
    double clearanceMm)
{
    if (firstBody.leafBvh.isEmpty() || secondBody.leafBvh.isEmpty())
        return false;
    const Bnd_Box firstRootCenter = transformBox(
        firstBody.leafBvh.front().localAabb, firstTransform);
    const Bnd_Box secondRootCenter = transformBox(
        secondBody.leafBvh.front().localAabb, secondTransform);
    // Disjoint surface leaves do not exclude complete solid containment.
    // 中文翻译：表面叶片互相分离不能排除完整实体包含。
    if (boxContains(firstRootCenter, secondRootCenter)
        || boxContains(secondRootCenter, firstRootCenter)) {
        return false;
    }

    QVector<Bnd_Box> firstSwept(firstBody.leafBvh.size());
    QVector<Bnd_Box> secondSwept(secondBody.leafBvh.size());
    QVector<std::uint8_t> firstReady(firstBody.leafBvh.size(), 0);
    QVector<std::uint8_t> secondReady(secondBody.leafBvh.size(), 0);
    const auto sweptNode = [&](const SourceBody& body,
                               const gp_Trsf& transform,
                               int nodeIndex,
                               QVector<Bnd_Box>* boxes,
                               QVector<std::uint8_t>* ready) -> const Bnd_Box& {
        if (!ready->at(nodeIndex)) {
            const Bnd_Box center = transformBox(
                body.leafBvh.at(nodeIndex).localAabb, transform);
            (*boxes)[nodeIndex] = enlargeBoxDirectional(center,
                bodyCellDirectionalMotionBound(
                    body, axes, pose, center, cellScale));
            (*ready)[nodeIndex] = 1;
        }
        return boxes->at(nodeIndex);
    };
    struct NodePair { int first; int second; };
    QVector<NodePair> stack;
    stack.append({0, 0});
    while (!stack.isEmpty()) {
        const NodePair pair = stack.takeLast();
        const Bnd_Box& firstBox = sweptNode(
            firstBody, firstTransform, pair.first, &firstSwept, &firstReady);
        const Bnd_Box& secondBox = sweptNode(
            secondBody, secondTransform, pair.second, &secondSwept, &secondReady);
        if (boxDistance(firstBox, secondBox) > clearanceMm)
            continue;
        const auto& firstNode = firstBody.leafBvh.at(pair.first);
        const auto& secondNode = secondBody.leafBvh.at(pair.second);
        if (firstNode.leaf >= 0 && secondNode.leaf >= 0)
            return false;
        if (firstNode.leaf >= 0) {
            stack.append({pair.first, secondNode.left});
            stack.append({pair.first, secondNode.right});
        } else if (secondNode.leaf >= 0) {
            stack.append({firstNode.left, pair.second});
            stack.append({firstNode.right, pair.second});
        } else if (boxVolume(firstBox) >= boxVolume(secondBox)) {
            stack.append({firstNode.left, pair.second});
            stack.append({firstNode.right, pair.second});
        } else {
            stack.append({pair.first, secondNode.left});
            stack.append({pair.first, secondNode.right});
        }
    }
    return true;
}

double overlapVolume(const Bnd_Box& first, const Bnd_Box& second)
{
    if (first.IsVoid() || second.IsVoid())
        return 0.0;
    Standard_Real ax0 = 0.0, ay0 = 0.0, az0 = 0.0;
    Standard_Real ax1 = 0.0, ay1 = 0.0, az1 = 0.0;
    Standard_Real bx0 = 0.0, by0 = 0.0, bz0 = 0.0;
    Standard_Real bx1 = 0.0, by1 = 0.0, bz1 = 0.0;
    first.Get(ax0, ay0, az0, ax1, ay1, az1);
    second.Get(bx0, by0, bz0, bx1, by1, bz1);
    const double dx = std::max(0.0, std::min(ax1, bx1) - std::max(ax0, bx0));
    const double dy = std::max(0.0, std::min(ay1, by1) - std::max(ay0, by0));
    const double dz = std::max(0.0, std::min(az1, bz1) - std::max(az0, bz0));
    return dx * dy * dz;
}


PairExactEvaluation evaluatePairExact(const SourceBody& firstBody,
                                      const gp_Trsf& firstTransform,
                                      const SourceBody& secondBody,
                                      const gp_Trsf& secondTransform,
                                      double clearanceMm,
                                      bool useLeafExact)
{
    PairExactEvaluation result;
    if (useLeafExact && !firstBody.leaves.isEmpty() && !secondBody.leaves.isEmpty()) {
        QVector<Bnd_Box> firstBoxes;
        QVector<Bnd_OBB> firstObbs;
        QVector<Bnd_Box> secondBoxes;
        QVector<Bnd_OBB> secondObbs;
        firstBoxes.reserve(firstBody.leaves.size());
        firstObbs.reserve(firstBody.leaves.size());
        secondBoxes.reserve(secondBody.leaves.size());
        secondObbs.reserve(secondBody.leaves.size());
        const double halfClearance = std::max(0.0, clearanceMm) * 0.5;
        for (const auto& leaf : firstBody.leaves) {
            Bnd_Box box = transformBox(leaf.localAabb, firstTransform);
            box.Enlarge(halfClearance);
            firstBoxes.append(std::move(box));
            firstObbs.append(transformObb(leaf.localObb, firstTransform,
                                          halfClearance));
        }
        for (const auto& leaf : secondBody.leaves) {
            Bnd_Box box = transformBox(leaf.localAabb, secondTransform);
            box.Enlarge(halfClearance);
            secondBoxes.append(std::move(box));
            secondObbs.append(transformObb(leaf.localObb, secondTransform,
                                           halfClearance));
        }
        QVector<LeafPairCandidate> candidates;
        for (int firstLeaf = 0; firstLeaf < firstBody.leaves.size(); ++firstLeaf) {
            for (int secondLeaf = 0; secondLeaf < secondBody.leaves.size(); ++secondLeaf) {
                if (firstBoxes.at(firstLeaf).IsOut(secondBoxes.at(secondLeaf))
                    || firstObbs.at(firstLeaf).IsOut(secondObbs.at(secondLeaf))) {
                    continue;
                }
                candidates.append({firstLeaf, secondLeaf,
                    overlapVolume(firstBoxes.at(firstLeaf),
                                  secondBoxes.at(secondLeaf))});
            }
        }
        result.leafCandidatePairs = static_cast<std::uint64_t>(candidates.size());
        std::sort(candidates.begin(), candidates.end(),
            [](const LeafPairCandidate& lhs, const LeafPairCandidate& rhs) {
                return lhs.overlap > rhs.overlap;
            });
        if (!candidates.isEmpty()) {
            lcnc::OcctExactOperationLock exactOperationLock;
            for (const auto& candidate : std::as_const(candidates)) {
                const TopoDS_Shape first = firstBody.leaves.at(candidate.firstLeaf)
                    .shape.Moved(TopLoc_Location(firstTransform));
                const TopoDS_Shape second = secondBody.leaves.at(candidate.secondLeaf)
                    .shape.Moved(TopLoc_Location(secondTransform));
                BRepExtrema_DistShapeShape distance(first, second);
                distance.SetDeflection(0.025);
                distance.SetMultiThread(Standard_False);
                distance.Perform();
                ++result.leafExactQueries;
                if (!distance.IsDone())
                    return result;
                result.distanceMm = std::min(result.distanceMm, distance.Value());
                if (distance.Value() <= clearanceMm + Precision::Confusion()) {
                    result.done = true;
                    return result;
                }
            }
        }
        // Face bounds prove ordinary separation, but not solid containment.
        // Preserve the whole-shape fallback when no leaf reports clearance.
        // 中文翻译：面级包围体可以证明普通分离，但不能排除实体包含；
        // 未发现间隙内叶片时保留整形精确回退。
    }

    const TopoDS_Shape first = firstBody.shape.Moved(
        TopLoc_Location(firstTransform));
    const TopoDS_Shape second = secondBody.shape.Moved(
        TopLoc_Location(secondTransform));
    lcnc::OcctExactOperationLock exactOperationLock;
    BRepExtrema_DistShapeShape distance(first, second);
    distance.SetDeflection(0.025);
    distance.SetMultiThread(Standard_False);
    distance.Perform();
    ++result.wholeShapeExactQueries;
    if (!distance.IsDone())
        return result;
    result.distanceMm = std::min(result.distanceMm, distance.Value());
    result.done = true;
    return result;
}

std::uint64_t flatCellCount(const QVector<MachineSafetyAxisGrid>& axes)
{
    std::uint64_t result = 1;
    for (const auto& axis : axes) {
        if (axis.cellCount == 0
            || result > std::numeric_limits<std::uint64_t>::max() / axis.cellCount) {
            return 0;
        }
        result *= axis.cellCount;
    }
    return axes.isEmpty() ? 0 : result;
}

int refinedChildrenPerCell(const QVector<MachineSafetyAxisGrid>& axes)
{
    return axes.isEmpty() || axes.size() > kMachineSafetyMaximumAxes
        ? 0 : (1 << axes.size());
}

MachineSafetyPose refinedCellCenter(const QVector<MachineSafetyAxisGrid>& axes,
                                    std::uint64_t baseCellIndex,
                                    int childIndex)
{
    MachineSafetyPose result;
    const int children = refinedChildrenPerCell(axes);
    if (baseCellIndex >= flatCellCount(axes)
        || childIndex < 0 || childIndex >= children) {
        return result;
    }
    result.count = static_cast<std::uint8_t>(axes.size());
    std::uint64_t remaining = baseCellIndex;
    std::array<std::uint32_t, kMachineSafetyMaximumAxes> coordinates{};
    for (int axisIndex = axes.size() - 1; axisIndex >= 0; --axisIndex) {
        coordinates[axisIndex] = static_cast<std::uint32_t>(
            remaining % axes.at(axisIndex).cellCount);
        remaining /= axes.at(axisIndex).cellCount;
    }
    for (int axisIndex = 0; axisIndex < axes.size(); ++axisIndex) {
        const auto& axis = axes.at(axisIndex);
        const double cellMinimum = axis.minimum
            + coordinates[axisIndex] * axis.step;
        const double cellMaximum = std::min(axis.maximum,
                                             cellMinimum + axis.step);
        const int bit = (childIndex >> (axes.size() - axisIndex - 1)) & 1;
        result.values[axisIndex] = cellMinimum
            + (bit == 0 ? 0.25 : 0.75) * (cellMaximum - cellMinimum);
    }
    return result;
}

void baseCellBounds(
    const QVector<MachineSafetyAxisGrid>& axes,
    std::uint64_t baseCellIndex,
    std::array<double, kMachineSafetyMaximumAxes>* minimums,
    std::array<double, kMachineSafetyMaximumAxes>* maximums)
{
    std::uint64_t remaining = baseCellIndex;
    std::array<std::uint32_t, kMachineSafetyMaximumAxes> coordinates{};
    for (int axisIndex = axes.size() - 1; axisIndex >= 0; --axisIndex) {
        coordinates[axisIndex] = static_cast<std::uint32_t>(
            remaining % axes.at(axisIndex).cellCount);
        remaining /= axes.at(axisIndex).cellCount;
    }
    for (int axisIndex = 0; axisIndex < axes.size(); ++axisIndex) {
        const auto& axis = axes.at(axisIndex);
        (*minimums)[axisIndex] = axis.minimum
            + coordinates[axisIndex] * axis.step;
        (*maximums)[axisIndex] = std::min(
            axis.maximum, (*minimums)[axisIndex] + axis.step);
    }
}

void selectRefinedChildBounds(
    int axisCount,
    int childIndex,
    std::array<double, kMachineSafetyMaximumAxes>* minimums,
    std::array<double, kMachineSafetyMaximumAxes>* maximums)
{
    for (int axisIndex = 0; axisIndex < axisCount; ++axisIndex) {
        const double midpoint = ((*minimums)[axisIndex]
                                 + (*maximums)[axisIndex]) * 0.5;
        const int bit = (childIndex >> (axisCount - axisIndex - 1)) & 1;
        if (bit == 0)
            (*maximums)[axisIndex] = midpoint;
        else
            (*minimums)[axisIndex] = midpoint;
    }
}

MachineSafetyPose refinedPathCenter(
    const QVector<MachineSafetyAxisGrid>& axes,
    std::uint64_t baseCellIndex,
    const QVector<int>& path)
{
    MachineSafetyPose result;
    if (baseCellIndex >= flatCellCount(axes) || path.isEmpty())
        return result;
    std::array<double, kMachineSafetyMaximumAxes> minimums{};
    std::array<double, kMachineSafetyMaximumAxes> maximums{};
    baseCellBounds(axes, baseCellIndex, &minimums, &maximums);
    for (const int child : path)
        selectRefinedChildBounds(axes.size(), child, &minimums, &maximums);
    result.count = static_cast<std::uint8_t>(axes.size());
    for (int axis = 0; axis < axes.size(); ++axis)
        result.values[axis] = (minimums[axis] + maximums[axis]) * 0.5;
    return result;
}

void writeAxis(QDataStream& stream, const MachineSafetyAxisGrid& axis)
{
    stream << axis.name << axis.parentAxis << axis.rotary;
    for (double value : axis.direction)
        stream << value;
    for (double value : axis.origin)
        stream << value;
    stream << axis.minimum << axis.maximum << axis.step << axis.cellCount;
}

void readAxis(QDataStream& stream, MachineSafetyAxisGrid* axis)
{
    stream >> axis->name >> axis->parentAxis >> axis->rotary;
    for (double& value : axis->direction)
        stream >> value;
    for (double& value : axis->origin)
        stream >> value;
    stream >> axis->minimum >> axis->maximum >> axis->step >> axis->cellCount;
}

} // namespace

struct MachineSafetyIndexCompiler::Impl
{
    QString sourcePath;
    QByteArray sourceSha256;
    QVector<SourceBody> bodies;
    qint64 importMs{0};
};

QString machineSafetyIndexStateName(MachineSafetyIndexState state)
{
    switch (state) {
    case MachineSafetyIndexState::CertifiedSafe:
        return QStringLiteral("CertifiedSafe");
    case MachineSafetyIndexState::CollisionSample:
        return QStringLiteral("CollisionSample");
    case MachineSafetyIndexState::Unknown:
    default:
        return QStringLiteral("Unknown");
    }
}

MachineSafetyBuildOptions defaultAcTableSafetyBuildOptions()
{
    MachineSafetyBuildOptions result;
    const auto axis = [](const QString& name, const QString& parent, bool rotary,
                         std::array<double, 3> direction,
                         double minimum, double maximum, double step) {
        MachineSafetyAxisGrid value;
        value.name = name;
        value.parentAxis = parent;
        value.rotary = rotary;
        value.direction = direction;
        value.minimum = minimum;
        value.maximum = maximum;
        value.step = step;
        value.cellCount = static_cast<std::uint32_t>(
            std::ceil((maximum - minimum) / step));
        return value;
    };
    result.axes = {
        axis(QStringLiteral("X"), QStringLiteral("Y"), false, {1.0, 0.0, 0.0},
             -500.0, 500.0, 100.0),
        axis(QStringLiteral("Y"), QStringLiteral("BASE"), false, {0.0, 1.0, 0.0},
             -400.0, 400.0, 80.0),
        axis(QStringLiteral("Z"), QStringLiteral("X"), false, {0.0, 0.0, 1.0},
             -300.0, 300.0, 60.0),
        axis(QStringLiteral("A"), QStringLiteral("BASE"), true, {1.0, 0.0, 0.0},
             -120.0, 120.0, 12.0),
        axis(QStringLiteral("C"), QStringLiteral("A"), true, {0.0, 0.0, 1.0},
             -180.0, 180.0, 18.0)};
    result.axisPairs = {
        {QStringLiteral("Z"), QStringLiteral("A")},
        {QStringLiteral("Z"), QStringLiteral("C")},
        {QStringLiteral("X"), QStringLiteral("A")},
        {QStringLiteral("X"), QStringLiteral("C")}};
    return result;
}

bool MachineSafetyIndex::isValid() const
{
    const std::uint64_t cells = flatCellCount(m_axes);
    return m_sourceSha256.size() == kChecksumBytes && cells > 0
        && cells == static_cast<std::uint64_t>(m_states.size())
        && m_states.size() == m_clearanceLowerBounds.size()
        && m_states.size() == m_blockingPairs.size()
        && !m_bodies.isEmpty() && !m_pairs.isEmpty()
        && (m_persistedSurfaceModels.isEmpty()
            || m_persistedSurfaceModels.size() == m_bodies.size())
        && (m_refinementOffsets.isEmpty()
            || m_refinementOffsets.size() == m_states.size())
        && m_refinedStates.size() == m_refinedClearanceLowerBounds.size()
        && m_refinedStates.size() == m_refinedBlockingPairs.size()
        && m_refinedStates.size() == m_refinedOffsets.size()
        && (m_refinedStates.isEmpty()
            || (!m_refinementOffsets.isEmpty()
                && m_refinedStates.size()
                    % refinedChildrenPerCell(m_axes) == 0));
}

std::uint64_t MachineSafetyIndex::cellCount() const
{
    return static_cast<std::uint64_t>(m_states.size());
}

std::uint64_t MachineSafetyIndex::stateCount(MachineSafetyIndexState state) const
{
    return static_cast<std::uint64_t>(std::count(
        m_states.cbegin(), m_states.cend(), static_cast<std::uint8_t>(state)));
}

std::uint64_t MachineSafetyIndex::effectiveCellCount() const
{
    if (m_refinementOffsets.isEmpty())
        return cellCount();
    const std::uint64_t refinedParents = static_cast<std::uint64_t>(std::count_if(
        m_refinementOffsets.cbegin(), m_refinementOffsets.cend(),
        [](qint32 value) { return value >= 0; }));
    const std::uint64_t refinedInternal = static_cast<std::uint64_t>(std::count_if(
        m_refinedOffsets.cbegin(), m_refinedOffsets.cend(),
        [](qint32 value) { return value >= 0; }));
    return cellCount() - refinedParents
        + static_cast<std::uint64_t>(m_refinedStates.size()) - refinedInternal;
}

std::uint64_t MachineSafetyIndex::effectiveStateCount(
    MachineSafetyIndexState state) const
{
    const auto value = static_cast<std::uint8_t>(state);
    std::uint64_t result = 0;
    for (int index = 0; index < m_states.size(); ++index) {
        if (!m_refinementOffsets.isEmpty() && m_refinementOffsets.at(index) >= 0)
            continue;
        if (m_states.at(index) == value)
            ++result;
    }
    for (int index = 0; index < m_refinedStates.size(); ++index) {
        if (m_refinedOffsets.at(index) < 0 && m_refinedStates.at(index) == value)
            ++result;
    }
    return result;
}

double MachineSafetyIndex::decisionCoverage() const
{
    if (cellCount() == 0)
        return 0.0;
    const int children = refinedChildrenPerCell(m_axes);
    double coveredBaseVolumes = 0.0;
    for (int index = 0; index < m_states.size(); ++index) {
        const qint32 offset = m_refinementOffsets.isEmpty()
            ? -1 : m_refinementOffsets.at(index);
        if (offset < 0) {
            // CollisionSample is valid only at the exact cell center and has
            // zero configuration-space volume. Do not count it as a whole
            // cell decision certificate.
            // 中文翻译：CollisionSample 仅对单元中心的精确姿态有效，
            // 在配置空间中体积为零，不能按整个单元计入决策覆盖率。
            if (m_states.at(index) == static_cast<std::uint8_t>(
                    MachineSafetyIndexState::CertifiedSafe)) {
                coveredBaseVolumes += 1.0;
            }
            continue;
        }
        const auto covered = [&](auto&& self, int blockOffset) -> double {
            double volume = 0.0;
            for (int child = 0; child < children; ++child) {
                const int refinedIndex = blockOffset + child;
                const qint32 deeper = m_refinedOffsets.at(refinedIndex);
                if (deeper >= 0)
                    volume += self(self, deeper) / children;
                else if (m_refinedStates.at(refinedIndex)
                         == static_cast<std::uint8_t>(
                             MachineSafetyIndexState::CertifiedSafe))
                    volume += 1.0 / children;
            }
            return volume;
        };
        coveredBaseVolumes += covered(covered, offset);
    }
    return coveredBaseVolumes / static_cast<double>(cellCount());
}

MachineSafetyPose MachineSafetyIndex::cellCenter(std::uint64_t cellIndex) const
{
    MachineSafetyPose result;
    if (cellIndex >= cellCount() || m_axes.size() > kMachineSafetyMaximumAxes)
        return result;
    result.count = static_cast<std::uint8_t>(m_axes.size());
    std::uint64_t remaining = cellIndex;
    std::array<std::uint32_t, kMachineSafetyMaximumAxes> coordinates{};
    for (int index = m_axes.size() - 1; index >= 0; --index) {
        coordinates[index] = static_cast<std::uint32_t>(
            remaining % m_axes.at(index).cellCount);
        remaining /= m_axes.at(index).cellCount;
    }
    for (int index = 0; index < m_axes.size(); ++index) {
        const auto& axis = m_axes.at(index);
        const double cellMinimum = axis.minimum + coordinates[index] * axis.step;
        const double cellMaximum = std::min(axis.maximum, cellMinimum + axis.step);
        result.values[index] = (cellMinimum + cellMaximum) * 0.5;
    }
    return result;
}

MachineSafetyPose MachineSafetyIndex::firstPose(MachineSafetyIndexState state) const
{
    const auto value = static_cast<std::uint8_t>(state);
    for (int index = 0; index < m_states.size(); ++index) {
        if ((!m_refinementOffsets.isEmpty()
             && m_refinementOffsets.at(index) >= 0))
            continue;
        if (m_states.at(index) == value)
            return cellCenter(static_cast<std::uint64_t>(index));
    }
    const int children = refinedChildrenPerCell(m_axes);
    if (children > 0 && !m_refinementOffsets.isEmpty()) {
        for (int base = 0; base < m_refinementOffsets.size(); ++base) {
            const qint32 offset = m_refinementOffsets.at(base);
            if (offset < 0)
                continue;
            QVector<int> path;
            MachineSafetyPose found;
            const auto findInBlock = [&](auto&& self, int blockOffset) -> bool {
                for (int child = 0; child < children; ++child) {
                    const int refinedIndex = blockOffset + child;
                    path.append(child);
                    const qint32 deeper = m_refinedOffsets.at(refinedIndex);
                    if (deeper >= 0) {
                        if (self(self, deeper))
                            return true;
                    } else if (m_refinedStates.at(refinedIndex) == value) {
                        found = refinedPathCenter(
                            m_axes, static_cast<std::uint64_t>(base), path);
                        return true;
                    }
                    path.removeLast();
                }
                return false;
            };
            if (findInBlock(findInBlock, offset))
                return found;
        }
    }
    return {};
}

MachineSafetyQueryResult MachineSafetyIndex::query(const MachineSafetyPose& pose) const
{
    MachineSafetyQueryResult result;
    if (!isValid() || pose.count != m_axes.size())
        return result;
    std::uint64_t flat = 0;
    std::array<double, kMachineSafetyMaximumAxes> minimums{};
    std::array<double, kMachineSafetyMaximumAxes> maximums{};
    for (int index = 0; index < m_axes.size(); ++index) {
        const auto& axis = m_axes.at(index);
        const double value = pose.values[index];
        if (!std::isfinite(value) || value < axis.minimum - kPoseTolerance
            || value > axis.maximum + kPoseTolerance) {
            return result;
        }
        std::uint32_t coordinate = value >= axis.maximum
            ? axis.cellCount - 1
            : static_cast<std::uint32_t>(std::floor((value - axis.minimum) / axis.step));
        coordinate = std::min(coordinate, axis.cellCount - 1);
        flat = flat * axis.cellCount + coordinate;
        const double cellMinimum = axis.minimum + coordinate * axis.step;
        const double cellMaximum = std::min(axis.maximum,
                                             cellMinimum + axis.step);
        minimums[index] = cellMinimum;
        maximums[index] = cellMaximum;
    }
    result.inRange = true;
    result.cellIndex = flat;
    int storageIndex = static_cast<int>(flat);
    bool refined = !m_refinementOffsets.isEmpty()
        && m_refinementOffsets.at(storageIndex) >= 0;
    MachineSafetyIndexState stored;
    if (refined) {
        qint32 blockOffset = m_refinementOffsets.at(storageIndex);
        while (blockOffset >= 0) {
            int refinedChild = 0;
            for (int axis = 0; axis < m_axes.size(); ++axis) {
                const double midpoint = (minimums[axis] + maximums[axis]) * 0.5;
                const bool upper = pose.values[axis] >= midpoint;
                refinedChild = refinedChild * 2 + (upper ? 1 : 0);
                if (upper)
                    minimums[axis] = midpoint;
                else
                    maximums[axis] = midpoint;
            }
            storageIndex = blockOffset + refinedChild;
            ++result.refinementLevel;
            blockOffset = m_refinedOffsets.at(storageIndex);
        }
        result.clearanceLowerBoundMm = m_refinedClearanceLowerBounds.at(storageIndex);
        result.blockingPair = m_refinedBlockingPairs.at(storageIndex);
        stored = static_cast<MachineSafetyIndexState>(m_refinedStates.at(storageIndex));
    } else {
        result.clearanceLowerBoundMm =
            m_clearanceLowerBounds.at(storageIndex);
        result.blockingPair = m_blockingPairs.at(storageIndex);
        stored = static_cast<MachineSafetyIndexState>(m_states.at(storageIndex));
    }
    if (stored == MachineSafetyIndexState::CollisionSample) {
        MachineSafetyPose center = cellCenter(flat);
        if (refined) {
            center.count = static_cast<std::uint8_t>(m_axes.size());
            for (int axis = 0; axis < m_axes.size(); ++axis)
                center.values[axis] = (minimums[axis] + maximums[axis]) * 0.5;
        }
        bool same = true;
        for (int index = 0; index < pose.count; ++index)
            same = same && std::abs(center.values[index] - pose.values[index]) <= kPoseTolerance;
        result.exactPoseCertificate = same;
        result.state = same ? stored : MachineSafetyIndexState::Unknown;
    } else {
        result.state = stored;
    }
    return result;
}

int MachineSafetyIndex::maximumRefinementLevel() const
{
    if (m_refinementOffsets.isEmpty())
        return 0;
    const int children = refinedChildrenPerCell(m_axes);
    const auto depth = [&](auto&& self, int blockOffset) -> int {
        int result = 1;
        for (int child = 0; child < children; ++child) {
            const qint32 deeper = m_refinedOffsets.at(blockOffset + child);
            if (deeper >= 0)
                result = std::max(result, 1 + self(self, deeper));
        }
        return result;
    };
    int result = 0;
    for (const qint32 offset : m_refinementOffsets) {
        if (offset >= 0)
            result = std::max(result, depth(depth, offset));
    }
    return result;
}

const SurfaceCollisionModel* MachineSafetyIndex::persistedSurfaceModel(
    int bodyIndex) const
{
    if (bodyIndex < 0 || bodyIndex >= m_persistedSurfaceModels.size()
        || !m_persistedSurfaceModels.at(bodyIndex).isValid()) {
        return nullptr;
    }
    return &m_persistedSurfaceModels.at(bodyIndex);
}

bool MachineSafetyIndex::save(const QString& filePath, QString* errorMessage) const
{
    if (!isValid()) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Machine safety index is incomplete");
        return false;
    }
    QByteArray payload;
    QBuffer buffer(&payload);
    buffer.open(QIODevice::WriteOnly);
    QDataStream stream(&buffer);
    stream.setVersion(QDataStream::Qt_6_5);
    stream.setByteOrder(QDataStream::LittleEndian);
    stream << m_sourceSha256 << m_clearanceMm;
    stream << static_cast<quint32>(m_axes.size());
    for (const auto& axis : m_axes)
        writeAxis(stream, axis);
    stream << static_cast<quint32>(m_bodies.size());
    for (const auto& body : m_bodies)
        stream << body.name << body.axisName;
    stream << static_cast<quint32>(m_bodies.size());
    for (int bodyIndex = 0; bodyIndex < m_bodies.size(); ++bodyIndex) {
        const bool available = bodyIndex < m_persistedSurfaceModels.size()
            && m_persistedSurfaceModels.at(bodyIndex).isValid();
        stream << available;
        if (!available)
            continue;
        const auto& model = m_persistedSurfaceModels.at(bodyIndex);
        const SurfaceTriangleSoup soup = model.triangleSoup();
        const SurfaceTriangleSoup containmentSoup =
            model.containmentTriangleSoup();
        stream << model.linearDeflectionMm() << model.isClosedSolid()
               << static_cast<quint64>(soup.triangles.size())
               << static_cast<quint64>(containmentSoup.triangles.size());
        stream.setFloatingPointPrecision(QDataStream::SinglePrecision);
        const auto writeSoup = [&stream](const SurfaceTriangleSoup& triangleSoup) {
            for (const auto& triangle : triangleSoup.triangles) {
                for (const std::uint32_t vertexIndex : triangle) {
                    const auto& vertex = triangleSoup.vertices.at(vertexIndex);
                    for (double coordinate : vertex)
                        stream << static_cast<float>(coordinate);
                }
            }
        };
        writeSoup(soup);
        writeSoup(containmentSoup);
        stream.setFloatingPointPrecision(QDataStream::DoublePrecision);
    }
    stream << static_cast<quint32>(m_pairs.size());
    for (const auto& pair : m_pairs)
        stream << pair.firstBody << pair.secondBody;
    stream << static_cast<quint64>(m_states.size());
    stream.setFloatingPointPrecision(QDataStream::SinglePrecision);
    for (int index = 0; index < m_states.size(); ++index) {
        stream << static_cast<quint8>(m_states.at(index));
        stream << m_clearanceLowerBounds.at(index);
        stream << m_blockingPairs.at(index);
    }
    stream << static_cast<quint64>(m_refinedStates.size());
    stream << static_cast<quint64>(m_refinementOffsets.isEmpty()
        ? 0 : m_states.size());
    if (m_refinementOffsets.isEmpty()) {
    } else {
        for (const qint32 offset : m_refinementOffsets)
            stream << offset;
    }
    for (int index = 0; index < m_refinedStates.size(); ++index) {
        stream << static_cast<quint8>(m_refinedStates.at(index));
        stream << m_refinedClearanceLowerBounds.at(index);
        stream << m_refinedBlockingPairs.at(index);
        stream << m_refinedOffsets.at(index);
    }
    if (stream.status() != QDataStream::Ok) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Unable to serialize machine safety index");
        return false;
    }
    buffer.close();
    const QByteArray checksum = QCryptographicHash::hash(payload,
                                                          QCryptographicHash::Sha256);
    QSaveFile file(filePath);
    if (!file.open(QIODevice::WriteOnly)) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Unable to create machine safety index: %1")
                                .arg(file.errorString());
        return false;
    }
    QDataStream outer(&file);
    outer.setVersion(QDataStream::Qt_6_5);
    outer.setByteOrder(QDataStream::LittleEndian);
    outer.writeRawData(kIndexMagic, sizeof(kIndexMagic));
    outer << kIndexSchema << static_cast<quint64>(payload.size());
    outer.writeRawData(checksum.constData(), checksum.size());
    outer.writeRawData(payload.constData(), payload.size());
    if (outer.status() != QDataStream::Ok || !file.commit()) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Unable to commit machine safety index");
        return false;
    }
    return true;
}

bool MachineSafetyIndex::load(const QString& filePath, QString* errorMessage)
{
    *this = {};
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Unable to open machine safety index: %1")
                                .arg(file.errorString());
        return false;
    }
    QDataStream outer(&file);
    outer.setVersion(QDataStream::Qt_6_5);
    outer.setByteOrder(QDataStream::LittleEndian);
    char magic[sizeof(kIndexMagic)]{};
    if (outer.readRawData(magic, sizeof(magic)) != sizeof(magic)
        || std::memcmp(magic, kIndexMagic, sizeof(magic)) != 0) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Machine safety index magic is invalid");
        return false;
    }
    quint32 schema = 0;
    quint64 payloadSize = 0;
    outer >> schema >> payloadSize;
    QByteArray expectedChecksum(kChecksumBytes, Qt::Uninitialized);
    if (outer.readRawData(expectedChecksum.data(), expectedChecksum.size())
            != expectedChecksum.size()
        || schema < 1 || schema > kIndexSchema
        || payloadSize > static_cast<quint64>(std::numeric_limits<int>::max())) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Machine safety index header is invalid");
        return false;
    }
    const QByteArray payload = file.read(static_cast<qint64>(payloadSize));
    if (payload.size() != static_cast<int>(payloadSize)
        || QCryptographicHash::hash(payload, QCryptographicHash::Sha256)
            != expectedChecksum) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Machine safety index checksum is invalid");
        return false;
    }
    m_contentSha256 = expectedChecksum;
    QBuffer buffer;
    buffer.setData(payload);
    buffer.open(QIODevice::ReadOnly);
    QDataStream stream(&buffer);
    stream.setVersion(QDataStream::Qt_6_5);
    stream.setByteOrder(QDataStream::LittleEndian);
    stream >> m_sourceSha256 >> m_clearanceMm;
    quint32 axisCount = 0;
    stream >> axisCount;
    if (axisCount == 0 || axisCount > kMachineSafetyMaximumAxes) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Machine safety index axis count is invalid");
        return false;
    }
    m_axes.resize(static_cast<int>(axisCount));
    for (auto& axis : m_axes)
        readAxis(stream, &axis);
    quint32 bodyCount = 0;
    stream >> bodyCount;
    m_bodies.resize(static_cast<int>(bodyCount));
    for (auto& body : m_bodies)
        stream >> body.name >> body.axisName;
    if (schema >= 3) {
        quint32 meshBodyCount = 0;
        stream >> meshBodyCount;
        if (meshBodyCount != bodyCount) {
            if (errorMessage)
                *errorMessage = QStringLiteral("Machine safety persisted BVH body count is invalid");
            return false;
        }
        m_persistedSurfaceModels.resize(static_cast<int>(meshBodyCount));
        for (int bodyIndex = 0; bodyIndex < m_persistedSurfaceModels.size(); ++bodyIndex) {
            bool available = false;
            stream >> available;
            if (!available)
                continue;
            if (schema <= 4) {
                if (errorMessage) {
                    *errorMessage = QStringLiteral(
                        "Legacy persisted collision mesh must be rebuilt for safe containment");
                }
                return false;
            }
            double deflection = 0.0;
            bool closedSolid = false;
            quint64 triangleCount = 0;
            quint64 containmentTriangleCount = 0;
            stream >> deflection >> closedSolid >> triangleCount
                   >> containmentTriangleCount;
            if (!std::isfinite(deflection) || deflection <= 0.0
                || triangleCount == 0 || triangleCount > 2'000'000
                || containmentTriangleCount > 2'000'000
                || triangleCount + containmentTriangleCount > 2'000'000) {
                if (errorMessage)
                    *errorMessage = QStringLiteral("Machine safety persisted BVH header is invalid");
                return false;
            }
            const auto readSoup = [&stream](quint64 storedTriangleCount) {
                SurfaceTriangleSoup soup;
                soup.vertices.reserve(static_cast<std::size_t>(storedTriangleCount) * 3);
                soup.triangles.reserve(static_cast<std::size_t>(storedTriangleCount));
                for (quint64 triangle = 0; triangle < storedTriangleCount; ++triangle) {
                    const std::uint32_t first = static_cast<std::uint32_t>(
                        soup.vertices.size());
                    for (int point = 0; point < 3; ++point) {
                        std::array<double, 3> vertex{};
                        for (double& coordinate : vertex) {
                            float stored = 0.0f;
                            stream >> stored;
                            coordinate = stored;
                        }
                        soup.vertices.push_back(vertex);
                    }
                    soup.triangles.push_back({first, first + 1, first + 2});
                }
                return soup;
            };
            stream.setFloatingPointPrecision(QDataStream::SinglePrecision);
            const SurfaceTriangleSoup soup = readSoup(triangleCount);
            const SurfaceTriangleSoup containmentSoup =
                readSoup(containmentTriangleCount);
            stream.setFloatingPointPrecision(QDataStream::DoublePrecision);
            std::string meshError;
            m_persistedSurfaceModels[bodyIndex] =
                SurfaceCollisionModel::buildFromTriangleSoup(
                    soup, containmentSoup, deflection, closedSolid, &meshError);
            if (!m_persistedSurfaceModels.at(bodyIndex).isValid()) {
                if (errorMessage)
                    *errorMessage = QStringLiteral("Unable to restore persisted machine BVH: %1")
                        .arg(QString::fromStdString(meshError));
                return false;
            }
        }
    }
    quint32 pairCount = 0;
    stream >> pairCount;
    m_pairs.resize(static_cast<int>(pairCount));
    for (auto& pair : m_pairs)
        stream >> pair.firstBody >> pair.secondBody;
    quint64 storedCells = 0;
    stream >> storedCells;
    if (storedCells == 0 || storedCells != flatCellCount(m_axes)
        || storedCells > static_cast<quint64>(std::numeric_limits<int>::max())) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Machine safety index cell count is invalid");
        return false;
    }
    m_states.resize(static_cast<int>(storedCells));
    m_clearanceLowerBounds.resize(static_cast<int>(storedCells));
    m_blockingPairs.resize(static_cast<int>(storedCells));
    if (schema >= 2)
        stream.setFloatingPointPrecision(QDataStream::SinglePrecision);
    for (int index = 0; index < m_states.size(); ++index) {
        quint8 state = 0;
        stream >> state >> m_clearanceLowerBounds[index] >> m_blockingPairs[index];
        if (state > static_cast<quint8>(MachineSafetyIndexState::CollisionSample)) {
            if (errorMessage)
                *errorMessage = QStringLiteral("Machine safety index contains an invalid state");
            return false;
        }
        m_states[index] = state;
    }
    if (schema == 2) {
        quint32 refinedBlockCount = 0;
        stream >> refinedBlockCount;
        const int children = refinedChildrenPerCell(m_axes);
        const quint64 refinedCellCount = static_cast<quint64>(refinedBlockCount)
            * static_cast<quint64>(children);
        if (refinedBlockCount > storedCells
            || refinedCellCount > static_cast<quint64>(std::numeric_limits<int>::max())) {
            if (errorMessage)
                *errorMessage = QStringLiteral("Machine safety refinement count is invalid");
            return false;
        }
        m_refinementOffsets.fill(-1, static_cast<int>(storedCells));
        m_refinedStates.resize(static_cast<int>(refinedCellCount));
        m_refinedClearanceLowerBounds.resize(static_cast<int>(refinedCellCount));
        m_refinedBlockingPairs.resize(static_cast<int>(refinedCellCount));
        m_refinedOffsets.fill(-1, static_cast<int>(refinedCellCount));
        for (quint32 block = 0; block < refinedBlockCount; ++block) {
            quint32 base = 0;
            stream >> base;
            if (base >= storedCells || m_refinementOffsets.at(static_cast<int>(base)) >= 0) {
                if (errorMessage)
                    *errorMessage = QStringLiteral("Machine safety refinement index is invalid");
                return false;
            }
            const int offset = static_cast<int>(block) * children;
            m_refinementOffsets[static_cast<int>(base)] = offset;
            for (int child = 0; child < children; ++child) {
                quint8 state = 0;
                stream >> state
                       >> m_refinedClearanceLowerBounds[offset + child]
                       >> m_refinedBlockingPairs[offset + child];
                if (state > static_cast<quint8>(MachineSafetyIndexState::CollisionSample)) {
                    if (errorMessage) {
                        *errorMessage = QStringLiteral(
                            "Machine safety refinement contains an invalid state");
                    }
                    return false;
                }
                m_refinedStates[offset + child] = state;
            }
        }
    } else if (schema >= 3) {
        quint64 refinedCellCount = 0;
        quint64 baseOffsetCount = 0;
        stream >> refinedCellCount >> baseOffsetCount;
        if ((baseOffsetCount != 0 && baseOffsetCount != storedCells)
            || refinedCellCount > static_cast<quint64>(
                std::numeric_limits<int>::max())) {
            if (errorMessage)
                *errorMessage = QStringLiteral("Machine safety sparse refinement header is invalid");
            return false;
        }
        m_refinementOffsets.resize(static_cast<int>(baseOffsetCount));
        for (qint32& offset : m_refinementOffsets)
            stream >> offset;
        m_refinedStates.resize(static_cast<int>(refinedCellCount));
        m_refinedClearanceLowerBounds.resize(static_cast<int>(refinedCellCount));
        m_refinedBlockingPairs.resize(static_cast<int>(refinedCellCount));
        m_refinedOffsets.resize(static_cast<int>(refinedCellCount));
        for (int index = 0; index < m_refinedStates.size(); ++index) {
            quint8 state = 0;
            stream >> state
                   >> m_refinedClearanceLowerBounds[index]
                   >> m_refinedBlockingPairs[index]
                   >> m_refinedOffsets[index];
            if (state > static_cast<quint8>(MachineSafetyIndexState::CollisionSample)
                || m_refinedOffsets[index] < -1
                || (m_refinedOffsets[index] >= 0
                    && m_refinedOffsets[index]
                        + refinedChildrenPerCell(m_axes) > m_refinedStates.size())) {
                if (errorMessage)
                    *errorMessage = QStringLiteral("Machine safety sparse refinement payload is invalid");
                return false;
            }
            m_refinedStates[index] = state;
        }
        for (const qint32 offset : std::as_const(m_refinementOffsets)) {
            if (offset < -1 || (offset >= 0
                && offset + refinedChildrenPerCell(m_axes) > m_refinedStates.size())) {
                if (errorMessage)
                    *errorMessage = QStringLiteral("Machine safety sparse base refinement offset is invalid");
                return false;
            }
        }
    }
    if (stream.status() != QDataStream::Ok || !isValid()) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Machine safety index payload is invalid");
        *this = {};
        return false;
    }
    return true;
}

MachineSafetyIndexCompiler::MachineSafetyIndexCompiler()
    : m_impl(std::make_unique<Impl>())
{
}

MachineSafetyIndexCompiler::~MachineSafetyIndexCompiler() = default;

bool MachineSafetyIndexCompiler::loadMachine(const QString& machineFilePath,
                                             QString* errorMessage)
{
    QElapsedTimer timer;
    timer.start();
    try {
        QVector<SourceBody> bodies;
        QByteArray digest;
        if (!hashFile(machineFilePath, &digest, errorMessage)
            || !readMachineBodies(machineFilePath, &bodies, errorMessage)) {
            return false;
        }
        m_impl->sourcePath = QFileInfo(machineFilePath).absoluteFilePath();
        m_impl->sourceSha256 = digest;
        m_impl->bodies = std::move(bodies);
        m_impl->importMs = timer.elapsed();
        return true;
    } catch (const Standard_Failure& failure) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "Machine safety index import OCCT failure: {}",
                 failure.GetMessageString());
        if (errorMessage)
            *errorMessage = QString::fromUtf8(failure.GetMessageString());
    } catch (const std::exception& failure) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "Machine safety index import failure: {}", failure.what());
        if (errorMessage)
            *errorMessage = QString::fromUtf8(failure.what());
    } catch (...) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "Machine safety index import unknown failure");
        if (errorMessage)
            *errorMessage = QStringLiteral("Unknown machine safety index import failure");
    }
    return false;
}

bool MachineSafetyIndexCompiler::build(const MachineSafetyBuildOptions& options,
                                       MachineSafetyIndex* index,
                                       MachineSafetyBuildMetrics* metrics,
                                       QString* errorMessage) const
{
    if (!index || m_impl->bodies.isEmpty()) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Machine safety compiler has no loaded source model");
        return false;
    }
    QElapsedTimer totalTimer;
    totalTimer.start();
    MachineSafetyBuildMetrics localMetrics;
    localMetrics.importMs = m_impl->importMs;
    std::mutex progressMutex;
    int lastProgress = -1;
    QString lastProgressStage;
    const auto reportProgress = [&](int percent, const QString& stage) {
        if (!options.progressCallback)
            return;
        const int bounded = std::clamp(percent, 0, 100);
        std::lock_guard<std::mutex> lock(progressMutex);
        if (bounded < lastProgress
            || (bounded == lastProgress && stage == lastProgressStage)) {
            return;
        }
        lastProgress = bounded;
        lastProgressStage = stage;
        try {
            options.progressCallback(bounded, stage);
        } catch (...) {
            // Progress reporting must never invalidate a conservative build.
            // 中文翻译：进度回调异常不得破坏保守构建，但必须留下审计日志。
            LCNC_ERR(lcnc::LogCode::Generic,
                     "Machine safety index progress callback failed");
        }
    };
    reportProgress(0, QStringLiteral("preparing_index"));
    for (const auto& body : m_impl->bodies) {
        localMetrics.geometryLeaves +=
            static_cast<std::uint64_t>(body.leaves.size());
    }
    try {
        MachineSafetyIndex result;
        result.m_sourceSha256 = m_impl->sourceSha256;
        result.m_axes = options.axes;
        result.m_clearanceMm = options.clearanceMm;
        if (result.m_axes.isEmpty() || result.m_axes.size() > kMachineSafetyMaximumAxes
            || !std::isfinite(options.clearanceMm) || options.clearanceMm < 0.0) {
            if (errorMessage)
                *errorMessage = QStringLiteral("Machine safety build options are invalid");
            return false;
        }
        if (options.refinementLevels < 0 || options.refinementLevels > 3
            || options.refinementThreads < 1 || options.refinementThreads > 32
            || (options.refinementLevels > 1 && options.hotPoses.isEmpty())
            || !std::isfinite(options.hotRefinementRadiusCells)
            || options.hotRefinementRadiusCells < 0.0
            || (options.refinementLevels > 0
                && !options.useDirectionalMotionBounds)
            || (options.useSurfaceBvhCertification
                && (!options.useDirectionalMotionBounds
                    || !std::isfinite(options.surfaceMeshDeflectionMm)
                    || options.surfaceMeshDeflectionMm <= 0.0))
            || (options.useLeafBvhCertification
                && !options.useDirectionalMotionBounds)) {
            if (errorMessage) {
                *errorMessage = QStringLiteral(
                    "Machine safety refinement requires directional bounds and supports up to three levels");
            }
            return false;
        }
        for (auto& axis : result.m_axes) {
            if (!std::isfinite(axis.minimum) || !std::isfinite(axis.maximum)
                || !std::isfinite(axis.step) || axis.maximum <= axis.minimum
                || axis.step <= 0.0) {
                if (errorMessage)
                    *errorMessage = QStringLiteral("Machine safety axis grid is invalid: %1")
                                        .arg(axis.name);
                return false;
            }
            axis.cellCount = static_cast<std::uint32_t>(
                std::ceil((axis.maximum - axis.minimum) / axis.step));
        }
        const std::uint64_t cells = flatCellCount(result.m_axes);
        if (cells == 0 || cells > options.maximumCells
            || cells > static_cast<std::uint64_t>(std::numeric_limits<int>::max())) {
            if (errorMessage) {
                *errorMessage = QStringLiteral("Machine safety grid contains too many cells: %1")
                                    .arg(cells);
            }
            return false;
        }
        for (const SourceBody& body : m_impl->bodies)
            result.m_bodies.append({body.name, body.axisName});
        for (const auto& requestedPair : options.axisPairs) {
            for (int first = 0; first < m_impl->bodies.size(); ++first) {
                if (m_impl->bodies.at(first).axisName.compare(
                        requestedPair.first, Qt::CaseInsensitive) != 0) {
                    continue;
                }
                for (int second = 0; second < m_impl->bodies.size(); ++second) {
                    if (m_impl->bodies.at(second).axisName.compare(
                            requestedPair.second, Qt::CaseInsensitive) == 0) {
                        result.m_pairs.append({static_cast<std::uint16_t>(first),
                                               static_cast<std::uint16_t>(second)});
                    }
                }
            }
        }
        if (result.m_pairs.isEmpty()) {
            if (errorMessage)
                *errorMessage = QStringLiteral("Machine safety build found no requested body pairs");
            return false;
        }
        int completedRefinementLevel = -1;
        if (options.resumeCheckpoint && !options.checkpointPath.isEmpty()
            && QFileInfo::exists(options.checkpointPath)) {
            MachineSafetyIndex checkpoint;
            QString checkpointError;
            const auto axesMatch = [](const QVector<MachineSafetyAxisGrid>& lhs,
                                      const QVector<MachineSafetyAxisGrid>& rhs) {
                if (lhs.size() != rhs.size())
                    return false;
                for (int axis = 0; axis < lhs.size(); ++axis) {
                    const auto& a = lhs.at(axis);
                    const auto& b = rhs.at(axis);
                    if (a.name != b.name || a.parentAxis != b.parentAxis
                        || a.rotary != b.rotary || a.direction != b.direction
                        || a.origin != b.origin || a.minimum != b.minimum
                        || a.maximum != b.maximum || a.step != b.step
                        || a.cellCount != b.cellCount) {
                        return false;
                    }
                }
                return true;
            };
            const auto bodiesMatch = [](const auto& lhs, const auto& rhs) {
                if (lhs.size() != rhs.size())
                    return false;
                for (int body = 0; body < lhs.size(); ++body) {
                    if (lhs.at(body).name != rhs.at(body).name
                        || lhs.at(body).axisName != rhs.at(body).axisName) {
                        return false;
                    }
                }
                return true;
            };
            const auto pairsMatch = [](const auto& lhs, const auto& rhs) {
                if (lhs.size() != rhs.size())
                    return false;
                for (int pair = 0; pair < lhs.size(); ++pair) {
                    if (lhs.at(pair).firstBody != rhs.at(pair).firstBody
                        || lhs.at(pair).secondBody != rhs.at(pair).secondBody) {
                        return false;
                    }
                }
                return true;
            };
            if (checkpoint.load(options.checkpointPath, &checkpointError)
                && checkpoint.m_sourceSha256 == result.m_sourceSha256
                && checkpoint.m_clearanceMm == result.m_clearanceMm
                && axesMatch(checkpoint.m_axes, result.m_axes)
                && bodiesMatch(checkpoint.m_bodies, result.m_bodies)
                && pairsMatch(checkpoint.m_pairs, result.m_pairs)) {
                result = std::move(checkpoint);
                completedRefinementLevel = result.maximumRefinementLevel();
                localMetrics.resumedFromCheckpoint = true;
                localMetrics.checkpointRefinementLevel =
                    completedRefinementLevel;
                for (int bodyIndex = 0;
                     bodyIndex < m_impl->bodies.size(); ++bodyIndex) {
                    if (const auto* surface = result.persistedSurfaceModel(
                            bodyIndex)) {
                        m_impl->bodies[bodyIndex].surfaceModel = *surface;
                    }
                }
                reportProgress(15, QStringLiteral("resuming_checkpoint"));
            } else if (!checkpointError.isEmpty()) {
                LCNC_WARN(lcnc::LogCode::Generic,
                          "Ignoring incompatible machine safety checkpoint: {}",
                          checkpointError.toStdString());
            }
        }
        if (options.useSurfaceBvhCertification) {
            QElapsedTimer surfaceCompileTimer;
            surfaceCompileTimer.start();
            int surfaceBodyIndex = 0;
            for (auto& body : m_impl->bodies) {
                if (body.surfaceModel.isValid()) {
                    ++surfaceBodyIndex;
                    reportProgress(5 + 10 * surfaceBodyIndex
                                       / std::max(1, static_cast<int>(
                                           m_impl->bodies.size())),
                                   QStringLiteral("surface_bvh"));
                    continue;
                }
                std::string surfaceError;
                body.surfaceModel = SurfaceCollisionModel::build(
                    body.shape, options.surfaceMeshDeflectionMm,
                    &surfaceError, 2'000'000);
                if (!body.surfaceModel.isValid()) {
                    if (errorMessage) {
                        *errorMessage = QStringLiteral(
                            "Unable to compile machine surface BVH for %1: %2")
                                .arg(body.name,
                                     QString::fromStdString(surfaceError));
                    }
                    return false;
                }
                ++surfaceBodyIndex;
                reportProgress(5 + 10 * surfaceBodyIndex
                                   / std::max(1, static_cast<int>(
                                       m_impl->bodies.size())),
                               QStringLiteral("surface_bvh"));
            }
            localMetrics.surfaceBvhCompileMs = surfaceCompileTimer.elapsed();
        }
        result.m_persistedSurfaceModels.resize(m_impl->bodies.size());
        for (int bodyIndex = 0; bodyIndex < m_impl->bodies.size(); ++bodyIndex) {
            if (m_impl->bodies.at(bodyIndex).surfaceModel.isValid()) {
                result.m_persistedSurfaceModels[bodyIndex] =
                    m_impl->bodies.at(bodyIndex).surfaceModel;
            }
        }
        if (completedRefinementLevel < 0) {
            result.m_states.fill(
                static_cast<std::uint8_t>(MachineSafetyIndexState::Unknown),
                static_cast<int>(cells));
            result.m_clearanceLowerBounds.fill(-1.0f, static_cast<int>(cells));
            result.m_blockingPairs.fill(-1, static_cast<int>(cells));
        }

        QVector<double> motionBounds;
        if (!options.useDirectionalMotionBounds) {
            motionBounds.reserve(m_impl->bodies.size());
            for (const auto& body : m_impl->bodies)
                motionBounds.append(bodyCellMotionBound(body, result.m_axes));
        }

        QVector<ExactCandidate> candidates;
        qint64 surfaceBvhQueryNs = 0;
        qint64 leafBvhQueryNs = 0;
        QElapsedTimer gridTimer;
        gridTimer.start();
        if (completedRefinementLevel < 0) {
        reportProgress(15, QStringLiteral("base_grid"));
        const std::uint64_t gridProgressInterval =
            std::max<std::uint64_t>(1, cells / 100);
        for (std::uint64_t cellIndex = 0; cellIndex < cells; ++cellIndex) {
            const MachineSafetyPose pose = result.cellCenter(cellIndex);
            QVector<Bnd_Box> centerBoxes;
            QVector<Bnd_Box> sweptBoxes;
            QVector<gp_Trsf> transforms;
            QVector<DirectionalMotionBound> directionalBounds;
            centerBoxes.reserve(m_impl->bodies.size());
            sweptBoxes.reserve(m_impl->bodies.size());
            transforms.reserve(m_impl->bodies.size());
            directionalBounds.reserve(m_impl->bodies.size());
            for (int bodyIndex = 0; bodyIndex < m_impl->bodies.size(); ++bodyIndex) {
                const auto& body = m_impl->bodies.at(bodyIndex);
                const gp_Trsf transform = axisChainTransform(result.m_axes, pose,
                                                               body.axisName);
                Bnd_Box center = transformBox(body.localAabb, transform);
                Bnd_Box swept;
                if (options.useDirectionalMotionBounds) {
                    const DirectionalMotionBound bound =
                        bodyCellDirectionalMotionBound(
                            body, result.m_axes, pose, center, 1.0);
                    swept = enlargeBoxDirectional(center, bound);
                    directionalBounds.append(bound);
                } else {
                    swept = center;
                    swept.Enlarge(motionBounds.at(bodyIndex));
                }
                transforms.append(transform);
                centerBoxes.append(std::move(center));
                sweptBoxes.append(std::move(swept));
            }
            bool certifiedSafe = true;
            double clearanceLowerBound = std::numeric_limits<double>::infinity();
            for (int pairIndex = 0; pairIndex < result.m_pairs.size(); ++pairIndex) {
                const auto& pair = result.m_pairs.at(pairIndex);
                const double sweptDistance = boxDistance(
                    sweptBoxes.at(pair.firstBody), sweptBoxes.at(pair.secondBody));
                clearanceLowerBound = std::min(clearanceLowerBound, sweptDistance);
                if (sweptDistance <= options.clearanceMm) {
                    if (options.useLeafBvhCertification) {
                        const auto queryStarted = std::chrono::steady_clock::now();
                        const bool separated = certifyPairSeparatedByLeafBvh(
                            m_impl->bodies.at(pair.firstBody),
                            transforms.at(pair.firstBody),
                            m_impl->bodies.at(pair.secondBody),
                            transforms.at(pair.secondBody),
                            result.m_axes, pose, 1.0, options.clearanceMm);
                        leafBvhQueryNs +=
                            std::chrono::duration_cast<std::chrono::nanoseconds>(
                                std::chrono::steady_clock::now() - queryStarted).count();
                        ++localMetrics.leafBvhQueries;
                        if (separated) {
                            ++localMetrics.leafBvhCertifiedPairs;
                            continue;
                        }
                    }
                    if (options.useSurfaceBvhCertification) {
                        const auto& firstBody = m_impl->bodies.at(pair.firstBody);
                        const auto& secondBody = m_impl->bodies.at(pair.secondBody);
                        const double motionGuard = scalarMagnitude(
                            directionalBounds.at(pair.firstBody))
                            + scalarMagnitude(directionalBounds.at(pair.secondBody));
                        const double meshGuard = firstBody.surfaceModel.linearDeflectionMm()
                            + secondBody.surfaceModel.linearDeflectionMm();
                        const auto queryStarted = std::chrono::steady_clock::now();
                        const auto prefilter = prefilterSurfaceCollision(
                            firstBody.surfaceModel, transforms.at(pair.firstBody),
                            secondBody.surfaceModel, transforms.at(pair.secondBody),
                            options.clearanceMm + motionGuard + meshGuard);
                        surfaceBvhQueryNs +=
                            std::chrono::duration_cast<std::chrono::nanoseconds>(
                                std::chrono::steady_clock::now() - queryStarted).count();
                        ++localMetrics.surfaceBvhQueries;
                        if (prefilter.valid && prefilter.definitelySeparated) {
                            ++localMetrics.surfaceBvhCertifiedPairs;
                            continue;
                        }
                    }
                    certifiedSafe = false;
                    if (boxDistance(centerBoxes.at(pair.firstBody),
                                    centerBoxes.at(pair.secondBody)) <= options.clearanceMm) {
                        candidates.append({cellIndex, pairIndex,
                            overlapVolume(centerBoxes.at(pair.firstBody),
                                          centerBoxes.at(pair.secondBody))});
                    }
                }
            }
            if (certifiedSafe) {
                result.m_states[static_cast<int>(cellIndex)] =
                    static_cast<std::uint8_t>(MachineSafetyIndexState::CertifiedSafe);
                result.m_clearanceLowerBounds[static_cast<int>(cellIndex)] =
                    static_cast<float>(clearanceLowerBound);
            }
            if ((cellIndex + 1) % gridProgressInterval == 0
                || cellIndex + 1 == cells) {
                reportProgress(15 + static_cast<int>(40 * (cellIndex + 1)
                                                     / cells),
                               QStringLiteral("base_grid"));
            }
        }
        } else {
            reportProgress(55, QStringLiteral("base_grid_reused"));
        }
        localMetrics.gridMs = gridTimer.elapsed();
        localMetrics.surfaceBvhQueryMs = surfaceBvhQueryNs / 1'000'000;
        localMetrics.leafBvhQueryMs = leafBvhQueryNs / 1'000'000;

        if (completedRefinementLevel < 0) {
        reportProgress(56, QStringLiteral("exact_sampling"));
        std::sort(candidates.begin(), candidates.end(),
            [](const ExactCandidate& lhs, const ExactCandidate& rhs) {
                if (lhs.overlapVolume != rhs.overlapVolume)
                    return lhs.overlapVolume > rhs.overlapVolume;
                if (lhs.cellIndex != rhs.cellIndex)
                    return lhs.cellIndex < rhs.cellIndex;
                return lhs.pairIndex < rhs.pairIndex;
            });
        QElapsedTimer exactTimer;
        exactTimer.start();
        QSet<std::uint64_t> testedCandidateKeys;
        for (const ExactCandidate& candidate : std::as_const(candidates)) {
            if (localMetrics.exactQueries
                >= static_cast<std::uint64_t>(std::max(0, options.maximumExactQueries))) {
                break;
            }
            const std::uint64_t candidateKey = candidate.cellIndex
                * static_cast<std::uint64_t>(result.m_pairs.size())
                + static_cast<std::uint64_t>(candidate.pairIndex);
            if (testedCandidateKeys.contains(candidateKey))
                continue;
            testedCandidateKeys.insert(candidateKey);
            const auto& pair = result.m_pairs.at(candidate.pairIndex);
            const auto& firstBody = m_impl->bodies.at(pair.firstBody);
            const auto& secondBody = m_impl->bodies.at(pair.secondBody);
            const MachineSafetyPose pose = result.cellCenter(candidate.cellIndex);
            const gp_Trsf firstTransform = axisChainTransform(
                result.m_axes, pose, firstBody.axisName);
            const gp_Trsf secondTransform = axisChainTransform(
                result.m_axes, pose, secondBody.axisName);
            ++localMetrics.exactQueries;
            const PairExactEvaluation evaluation = evaluatePairExact(
                firstBody, firstTransform, secondBody, secondTransform,
                options.clearanceMm, options.useLeafExact);
            localMetrics.leafCandidatePairs += evaluation.leafCandidatePairs;
            localMetrics.leafExactQueries += evaluation.leafExactQueries;
            localMetrics.wholeShapeExactQueries +=
                evaluation.wholeShapeExactQueries;
            if (!evaluation.done) {
                ++localMetrics.exactFailures;
                continue;
            }
            if (evaluation.distanceMm
                <= options.clearanceMm + Precision::Confusion()) {
                const int flat = static_cast<int>(candidate.cellIndex);
                result.m_states[flat] = static_cast<std::uint8_t>(
                    MachineSafetyIndexState::CollisionSample);
                result.m_clearanceLowerBounds[flat] =
                    static_cast<float>(evaluation.distanceMm);
                result.m_blockingPairs[flat] = static_cast<qint16>(candidate.pairIndex);
            }
        }
        localMetrics.exactMs = exactTimer.elapsed();

        if (!options.checkpointPath.isEmpty()) {
            reportProgress(58, QStringLiteral("saving_checkpoint"));
            QString checkpointError;
            if (!result.save(options.checkpointPath, &checkpointError)) {
                if (errorMessage)
                    *errorMessage = QStringLiteral("Unable to save base-grid checkpoint: %1")
                        .arg(checkpointError);
                return false;
            }
        }
        completedRefinementLevel = 0;
        }

        if (options.refinementLevels > completedRefinementLevel) {
            const int children = refinedChildrenPerCell(result.m_axes);
            QElapsedTimer refinementTimer;
            refinementTimer.start();
            std::atomic_uint64_t refinedBodyEvaluations{0};
            std::atomic_uint64_t refinedBodyCacheHits{0};
            std::atomic_uint64_t refinedPairEvaluations{0};
            std::atomic_uint64_t refinedPairCacheHits{0};
            if (completedRefinementLevel < 1) {
            reportProgress(60, QStringLiteral("refinement_level_1"));
            const std::uint64_t refinedParents = result.stateCount(
                MachineSafetyIndexState::Unknown);
            const std::uint64_t refinedCells = refinedParents
                * static_cast<std::uint64_t>(children);
            if (refinedCells > options.maximumRefinedCells
                || refinedCells > static_cast<std::uint64_t>(
                    std::numeric_limits<int>::max())) {
                if (errorMessage) {
                    *errorMessage = QStringLiteral(
                        "Machine safety refinement contains too many cells: %1")
                            .arg(refinedCells);
                }
                return false;
            }
            result.m_refinementOffsets.fill(-1, static_cast<int>(cells));
            result.m_refinedStates.fill(
                static_cast<std::uint8_t>(MachineSafetyIndexState::Unknown),
                static_cast<int>(refinedCells));
            result.m_refinedClearanceLowerBounds.fill(
                -1.0f, static_cast<int>(refinedCells));
            result.m_refinedBlockingPairs.fill(-1, static_cast<int>(refinedCells));
            result.m_refinedOffsets.fill(-1, static_cast<int>(refinedCells));

            int nextOffset = 0;
            for (std::uint64_t baseCell = 0; baseCell < cells; ++baseCell) {
                const int baseIndex = static_cast<int>(baseCell);
                if (result.m_states.at(baseIndex)
                    != static_cast<std::uint8_t>(MachineSafetyIndexState::Unknown)) {
                    continue;
                }
                result.m_refinementOffsets[baseIndex] = nextOffset;
                nextOffset += children;
            }

            auto* refinedStates = result.m_refinedStates.data();
            auto* refinedClearance = result.m_refinedClearanceLowerBounds.data();
            std::atomic_uint64_t refinedLeafQueries{0};
            std::atomic_uint64_t refinedLeafCertifiedPairs{0};
            std::atomic_uint64_t refinedLeafQueryNs{0};
            std::atomic_uint64_t refinedBaseCellsProcessed{0};
            const std::uint64_t refinementProgressInterval =
                std::max<std::uint64_t>(1, cells / 100);
            QVector<std::uint8_t> bodyDependencyMasks;
            bodyDependencyMasks.reserve(m_impl->bodies.size());
            for (const auto& body : m_impl->bodies) {
                bodyDependencyMasks.append(options.useDependencyCache
                    ? bodyAxisDependencyMask(result.m_axes, body.axisName)
                    : static_cast<std::uint8_t>((1u << result.m_axes.size()) - 1u));
            }
            QVector<std::uint8_t> pairDependencyMasks;
            pairDependencyMasks.reserve(result.m_pairs.size());
            for (const auto& pair : result.m_pairs) {
                pairDependencyMasks.append(static_cast<std::uint8_t>(
                    bodyDependencyMasks.at(pair.firstBody)
                    | bodyDependencyMasks.at(pair.secondBody)));
            }
            const int threadCount = std::min(options.refinementThreads,
                std::max(1, static_cast<int>(cells)));
            std::vector<std::thread> workers;
            workers.reserve(static_cast<std::size_t>(threadCount));
            for (int worker = 0; worker < threadCount; ++worker) {
                const std::uint64_t begin = cells
                    * static_cast<std::uint64_t>(worker) / threadCount;
                const std::uint64_t end = cells
                    * static_cast<std::uint64_t>(worker + 1) / threadCount;
                workers.emplace_back([&, begin, end]() {
                struct CachedBodyEvaluation
                {
                    gp_Trsf transform;
                    Bnd_Box sweptBox;
                    std::uint64_t generation{0};
                };
                struct CachedPairEvaluation
                {
                    double sweptDistance{0.0};
                    bool safe{false};
                    std::uint64_t generation{0};
                };
                QVector<std::array<CachedBodyEvaluation, 32>> bodyCache(
                    m_impl->bodies.size());
                QVector<std::array<CachedPairEvaluation, 32>> pairCache(
                    result.m_pairs.size());
                std::uint64_t cacheGeneration = 0;
                for (std::uint64_t baseCell = begin; baseCell < end; ++baseCell) {
                    const std::uint64_t processed =
                        refinedBaseCellsProcessed.fetch_add(
                            1, std::memory_order_relaxed) + 1;
                    if (processed % refinementProgressInterval == 0
                        || processed == cells) {
                        reportProgress(60 + static_cast<int>(28 * processed
                                                             / cells),
                                       QStringLiteral("refinement_level_1"));
                    }
                    const int nextOffset = result.m_refinementOffsets.at(
                        static_cast<int>(baseCell));
                    if (nextOffset < 0)
                        continue;
                    ++cacheGeneration;
                    for (int child = 0; child < children; ++child) {
                    const MachineSafetyPose pose = refinedCellCenter(
                        result.m_axes, baseCell, child);
                    bool certifiedSafe = true;
                    double clearanceLowerBound =
                        std::numeric_limits<double>::infinity();
                    for (int pairIndex = 0;
                         pairIndex < result.m_pairs.size(); ++pairIndex) {
                        const auto& pair = result.m_pairs.at(pairIndex);
                        const int pairKey = child
                            & pairDependencyMasks.at(pairIndex);
                        auto& cachedPair = pairCache[pairIndex][pairKey];
                        if (cachedPair.generation != cacheGeneration) {
                            const auto ensureBody = [&](int bodyIndex)
                                -> CachedBodyEvaluation& {
                                const int bodyKey = child
                                    & bodyDependencyMasks.at(bodyIndex);
                                auto& cachedBody = bodyCache[bodyIndex][bodyKey];
                                if (cachedBody.generation == cacheGeneration) {
                                    refinedBodyCacheHits.fetch_add(
                                        1, std::memory_order_relaxed);
                                    return cachedBody;
                                }
                                const auto& body = m_impl->bodies.at(bodyIndex);
                                cachedBody.transform = axisChainTransform(
                                    result.m_axes, pose, body.axisName);
                                const Bnd_Box center = transformBox(
                                    body.localAabb, cachedBody.transform);
                                cachedBody.sweptBox = enlargeBoxDirectional(
                                    center, bodyCellDirectionalMotionBound(
                                        body, result.m_axes, pose, center, 0.5));
                                cachedBody.generation = cacheGeneration;
                                refinedBodyEvaluations.fetch_add(
                                    1, std::memory_order_relaxed);
                                return cachedBody;
                            };
                            auto& firstBody = ensureBody(pair.firstBody);
                            auto& secondBody = ensureBody(pair.secondBody);
                            cachedPair.sweptDistance = boxDistance(
                                firstBody.sweptBox, secondBody.sweptBox);
                            cachedPair.safe = cachedPair.sweptDistance
                                > options.clearanceMm;
                            if (!cachedPair.safe
                                && options.useLeafBvhCertification) {
                                const auto queryStarted =
                                    std::chrono::steady_clock::now();
                                cachedPair.safe = certifyPairSeparatedByLeafBvh(
                                    m_impl->bodies.at(pair.firstBody),
                                    firstBody.transform,
                                    m_impl->bodies.at(pair.secondBody),
                                    secondBody.transform,
                                    result.m_axes, pose, 0.5,
                                    options.clearanceMm);
                                refinedLeafQueryNs.fetch_add(
                                    static_cast<std::uint64_t>(std::chrono::duration_cast<
                                    std::chrono::nanoseconds>(
                                        std::chrono::steady_clock::now()
                                        - queryStarted).count()),
                                    std::memory_order_relaxed);
                                refinedLeafQueries.fetch_add(
                                    1, std::memory_order_relaxed);
                                if (cachedPair.safe) {
                                    refinedLeafCertifiedPairs.fetch_add(
                                        1, std::memory_order_relaxed);
                                }
                            }
                            cachedPair.generation = cacheGeneration;
                            refinedPairEvaluations.fetch_add(
                                1, std::memory_order_relaxed);
                        } else {
                            refinedPairCacheHits.fetch_add(
                                1, std::memory_order_relaxed);
                        }
                        const double sweptDistance = cachedPair.sweptDistance;
                        clearanceLowerBound = std::min(clearanceLowerBound,
                                                       sweptDistance);
                        if (!cachedPair.safe)
                            certifiedSafe = false;
                    }
                    const int refinedIndex = nextOffset + child;
                    if (certifiedSafe) {
                        refinedStates[refinedIndex] =
                            static_cast<std::uint8_t>(
                                MachineSafetyIndexState::CertifiedSafe);
                        refinedClearance[refinedIndex] =
                            static_cast<float>(clearanceLowerBound);
                    }
                }
                }
                });
            }
            for (auto& worker : workers)
                worker.join();
            reportProgress(88, QStringLiteral("refinement_level_1"));
            leafBvhQueryNs += static_cast<qint64>(
                refinedLeafQueryNs.load(std::memory_order_relaxed));
            localMetrics.leafBvhQueries +=
                refinedLeafQueries.load(std::memory_order_relaxed);
            localMetrics.leafBvhCertifiedPairs +=
                refinedLeafCertifiedPairs.load(std::memory_order_relaxed);
            if (!options.checkpointPath.isEmpty()) {
                QString checkpointError;
                if (!result.save(options.checkpointPath, &checkpointError)) {
                    if (errorMessage)
                        *errorMessage = QStringLiteral("Unable to save level-1 refinement checkpoint: %1")
                            .arg(checkpointError);
                    return false;
                }
            }
            completedRefinementLevel = 1;
            }

            struct DeepRefinementParent
            {
                std::uint64_t baseCell{0};
                QVector<int> path;
                int refinedIndex{-1};
            };
            QVector<DeepRefinementParent> parents;
            if (options.refinementLevels > completedRefinementLevel) {
                const auto collectFrontier = [&](auto&& self,
                                                 std::uint64_t baseCell,
                                                 QVector<int> path,
                                                 int refinedIndex,
                                                 int depth) -> void {
                    if (depth == completedRefinementLevel) {
                        if (result.m_refinedOffsets.at(refinedIndex) < 0
                            && result.m_refinedStates.at(refinedIndex)
                                == static_cast<std::uint8_t>(
                                    MachineSafetyIndexState::Unknown)) {
                            parents.append({baseCell, std::move(path),
                                            refinedIndex});
                        }
                        return;
                    }
                    const qint32 deeper = result.m_refinedOffsets.at(
                        refinedIndex);
                    if (deeper < 0)
                        return;
                    for (int child = 0; child < children; ++child) {
                        QVector<int> childPath = path;
                        childPath.append(child);
                        self(self, baseCell, std::move(childPath),
                             deeper + child, depth + 1);
                    }
                };
                for (std::uint64_t baseCell = 0; baseCell < cells; ++baseCell) {
                    const qint32 offset = result.m_refinementOffsets.at(
                        static_cast<int>(baseCell));
                    if (offset < 0)
                        continue;
                    for (int child = 0; child < children; ++child) {
                        const int refinedIndex = offset + child;
                        collectFrontier(collectFrontier, baseCell,
                                        QVector<int>{child}, refinedIndex, 1);
                    }
                }
            }
            const auto nearHotPose = [&](const MachineSafetyPose& pose) {
                for (const MachineSafetyPose& hot : options.hotPoses) {
                    if (hot.count != pose.count)
                        continue;
                    double normalizedSquared = 0.0;
                    for (int axis = 0; axis < pose.count; ++axis) {
                        double delta = std::abs(pose.values[axis]
                                                - hot.values[axis]);
                        if (result.m_axes.at(axis).rotary)
                            delta = std::min(delta, std::abs(360.0 - delta));
                        const double step = std::max(
                            result.m_axes.at(axis).step, 1.0e-12);
                        normalizedSquared += (delta / step) * (delta / step);
                    }
                    if (std::sqrt(normalizedSquared)
                        <= options.hotRefinementRadiusCells) {
                        return true;
                    }
                }
                return false;
            };
            for (int level = completedRefinementLevel + 1;
                 level <= options.refinementLevels; ++level) {
                QVector<DeepRefinementParent> selected;
                selected.reserve(parents.size());
                for (const auto& parent : std::as_const(parents)) {
                    if (nearHotPose(refinedPathCenter(
                            result.m_axes, parent.baseCell, parent.path))) {
                        selected.append(parent);
                    }
                }
                if (selected.isEmpty())
                    break;
                const std::uint64_t additional =
                    static_cast<std::uint64_t>(selected.size())
                    * static_cast<std::uint64_t>(children);
                if (static_cast<std::uint64_t>(result.m_refinedStates.size())
                        + additional > options.maximumRefinedCells
                    || additional > static_cast<std::uint64_t>(
                        std::numeric_limits<int>::max())) {
                    if (errorMessage) {
                        *errorMessage = QStringLiteral(
                            "Machine safety hot refinement exceeds the refined-cell budget at level %1")
                                .arg(level);
                    }
                    return false;
                }
                const int oldSize = result.m_refinedStates.size();
                const int newSize = oldSize + static_cast<int>(additional);
                result.m_refinedStates.resize(newSize);
                result.m_refinedClearanceLowerBounds.resize(newSize);
                result.m_refinedBlockingPairs.resize(newSize);
                result.m_refinedOffsets.resize(newSize);
                std::fill(result.m_refinedStates.begin() + oldSize,
                          result.m_refinedStates.end(),
                          static_cast<std::uint8_t>(
                              MachineSafetyIndexState::Unknown));
                std::fill(result.m_refinedClearanceLowerBounds.begin() + oldSize,
                          result.m_refinedClearanceLowerBounds.end(), -1.0f);
                std::fill(result.m_refinedBlockingPairs.begin() + oldSize,
                          result.m_refinedBlockingPairs.end(), -1);
                std::fill(result.m_refinedOffsets.begin() + oldSize,
                          result.m_refinedOffsets.end(), -1);
                for (int parentIndex = 0; parentIndex < selected.size(); ++parentIndex) {
                    result.m_refinedOffsets[selected.at(parentIndex).refinedIndex]
                        = oldSize + parentIndex * children;
                }

                const double cellScale = std::ldexp(1.0, -level);
                std::atomic_int nextParent{0};
                std::atomic_int deepParentsProcessed{0};
                std::atomic_uint64_t deepLeafQueries{0};
                std::atomic_uint64_t deepLeafCertified{0};
                std::vector<std::thread> deepWorkers;
                const int deepThreadCount = std::min(
                    options.refinementThreads,
                    static_cast<int>(selected.size()));
                deepWorkers.reserve(static_cast<std::size_t>(deepThreadCount));
                for (int worker = 0; worker < deepThreadCount; ++worker) {
                    deepWorkers.emplace_back([&] {
                        while (true) {
                            const int parentIndex = nextParent.fetch_add(1);
                            if (parentIndex >= selected.size())
                                break;
                            const auto& parent = selected.at(parentIndex);
                            const int blockOffset = oldSize
                                + parentIndex * children;
                            for (int child = 0; child < children; ++child) {
                                QVector<int> path = parent.path;
                                path.append(child);
                                const MachineSafetyPose pose = refinedPathCenter(
                                    result.m_axes, parent.baseCell, path);
                                QVector<gp_Trsf> transforms;
                                QVector<Bnd_Box> sweptBoxes;
                                transforms.reserve(m_impl->bodies.size());
                                sweptBoxes.reserve(m_impl->bodies.size());
                                for (const auto& body : m_impl->bodies) {
                                    const gp_Trsf transform = axisChainTransform(
                                        result.m_axes, pose, body.axisName);
                                    const Bnd_Box center = transformBox(
                                        body.localAabb, transform);
                                    transforms.append(transform);
                                    sweptBoxes.append(enlargeBoxDirectional(
                                        center, bodyCellDirectionalMotionBound(
                                            body, result.m_axes, pose, center,
                                            cellScale)));
                                }
                                bool safe = true;
                                double lowerBound =
                                    std::numeric_limits<double>::infinity();
                                for (const auto& pair : result.m_pairs) {
                                    const double sweptDistance = boxDistance(
                                        sweptBoxes.at(pair.firstBody),
                                        sweptBoxes.at(pair.secondBody));
                                    lowerBound = std::min(lowerBound,
                                                          sweptDistance);
                                    if (sweptDistance > options.clearanceMm)
                                        continue;
                                    bool pairSafe = false;
                                    if (options.useLeafBvhCertification) {
                                        deepLeafQueries.fetch_add(
                                            1, std::memory_order_relaxed);
                                        pairSafe = certifyPairSeparatedByLeafBvh(
                                            m_impl->bodies.at(pair.firstBody),
                                            transforms.at(pair.firstBody),
                                            m_impl->bodies.at(pair.secondBody),
                                            transforms.at(pair.secondBody),
                                            result.m_axes, pose, cellScale,
                                            options.clearanceMm);
                                        if (pairSafe) {
                                            deepLeafCertified.fetch_add(
                                                1, std::memory_order_relaxed);
                                        }
                                    }
                                    if (!pairSafe) {
                                        safe = false;
                                        break;
                                    }
                                }
                                if (safe) {
                                    result.m_refinedStates[blockOffset + child]
                                        = static_cast<std::uint8_t>(
                                            MachineSafetyIndexState::CertifiedSafe);
                                    result.m_refinedClearanceLowerBounds[
                                        blockOffset + child]
                                        = static_cast<float>(lowerBound);
                                }
                            }
                            const int processed = deepParentsProcessed.fetch_add(
                                1, std::memory_order_relaxed) + 1;
                            const int levelStart = level == 2 ? 89 : 94;
                            const int levelSpan = level == 2 ? 5 : 3;
                            reportProgress(levelStart + levelSpan * processed
                                               / std::max(1, static_cast<int>(
                                                   selected.size())),
                                           QStringLiteral("hot_refinement_%1")
                                               .arg(level));
                        }
                    });
                }
                for (auto& worker : deepWorkers)
                    worker.join();
                localMetrics.leafBvhQueries += deepLeafQueries.load();
                localMetrics.leafBvhCertifiedPairs +=
                    deepLeafCertified.load();

                parents.clear();
                for (int parentIndex = 0; parentIndex < selected.size(); ++parentIndex) {
                    const int blockOffset = oldSize + parentIndex * children;
                    for (int child = 0; child < children; ++child) {
                        const int refinedIndex = blockOffset + child;
                        if (result.m_refinedStates.at(refinedIndex)
                            != static_cast<std::uint8_t>(
                                MachineSafetyIndexState::Unknown)) {
                            continue;
                        }
                        QVector<int> path = selected.at(parentIndex).path;
                        path.append(child);
                        parents.append({selected.at(parentIndex).baseCell,
                                        std::move(path), refinedIndex});
                    }
                }
                if (!options.checkpointPath.isEmpty()) {
                    QString checkpointError;
                    if (!result.save(options.checkpointPath, &checkpointError)) {
                        if (errorMessage) {
                            *errorMessage = QStringLiteral(
                                "Unable to save refinement checkpoint at level %1: %2")
                                    .arg(level).arg(checkpointError);
                        }
                        return false;
                    }
                }
            }
            localMetrics.refinementMs = refinementTimer.elapsed();
            // Internal sparse nodes are storage only. Report leaf counts so a
            // deeper hot-zone refinement does not inflate Unknown coverage.
            localMetrics.refinedParentCells = static_cast<std::uint64_t>(
                result.m_refinedStates.size() / children);
            for (int refinedIndex = 0;
                 refinedIndex < result.m_refinedStates.size(); ++refinedIndex) {
                if (result.m_refinedOffsets.at(refinedIndex) >= 0)
                    continue;
                ++localMetrics.refinedCells;
                const auto state = static_cast<MachineSafetyIndexState>(
                    result.m_refinedStates.at(refinedIndex));
                if (state == MachineSafetyIndexState::CertifiedSafe)
                    ++localMetrics.refinedSafeCells;
                else if (state == MachineSafetyIndexState::Unknown)
                    ++localMetrics.refinedUnknownCells;
            }
            localMetrics.refinementBodyEvaluations = refinedBodyEvaluations.load(
                std::memory_order_relaxed);
            localMetrics.refinementBodyCacheHits = refinedBodyCacheHits.load(
                std::memory_order_relaxed);
            localMetrics.refinementPairEvaluations = refinedPairEvaluations.load(
                std::memory_order_relaxed);
            localMetrics.refinementPairCacheHits = refinedPairCacheHits.load(
                std::memory_order_relaxed);
        }
        localMetrics.leafBvhQueryMs = leafBvhQueryNs / 1'000'000;
        localMetrics.totalCells = cells;
        localMetrics.certifiedSafeCells = result.stateCount(
            MachineSafetyIndexState::CertifiedSafe);
        localMetrics.collisionSamples = result.stateCount(
            MachineSafetyIndexState::CollisionSample);
        localMetrics.unknownCells = result.stateCount(MachineSafetyIndexState::Unknown);
        localMetrics.totalMs = totalTimer.elapsed();
        reportProgress(99, QStringLiteral("finalizing_index"));
        *index = std::move(result);
        if (metrics)
            *metrics = localMetrics;
        reportProgress(100, QStringLiteral("index_complete"));
        return true;
    } catch (const Standard_Failure& failure) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "Machine safety index build OCCT failure: {}",
                 failure.GetMessageString());
        if (errorMessage)
            *errorMessage = QString::fromUtf8(failure.GetMessageString());
    } catch (const std::exception& failure) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "Machine safety index build failure: {}", failure.what());
        if (errorMessage)
            *errorMessage = QString::fromUtf8(failure.what());
    } catch (...) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "Machine safety index build unknown failure");
        if (errorMessage)
            *errorMessage = QStringLiteral("Unknown machine safety index build failure");
    }
    return false;
}

MachineSafetyExactResult MachineSafetyIndexCompiler::validatePoseExact(
    const MachineSafetyIndex& index, const MachineSafetyPose& pose) const
{
    MachineSafetyExactResult result;
    if (!index.isValid() || m_impl->bodies.isEmpty()
        || pose.count != index.axes().size()) {
        result.failureReason = QStringLiteral("Exact machine safety pose request is invalid");
        return result;
    }
    result.state = MachineSafetyIndexState::CertifiedSafe;
    result.minimumDistanceMm = std::numeric_limits<double>::infinity();
    try {
        QVector<gp_Trsf> transforms;
        QVector<Bnd_Box> boxes;
        transforms.reserve(m_impl->bodies.size());
        boxes.reserve(m_impl->bodies.size());
        for (const auto& body : m_impl->bodies) {
            const gp_Trsf transform = axisChainTransform(index.axes(), pose, body.axisName);
            transforms.append(transform);
            boxes.append(transformBox(body.localAabb, transform));
        }
        for (int pairIndex = 0; pairIndex < index.pairs().size(); ++pairIndex) {
            const auto& pair = index.pairs().at(pairIndex);
            const double broadDistance = boxDistance(boxes.at(pair.firstBody),
                                                     boxes.at(pair.secondBody));
            if (broadDistance > index.clearanceMm()) {
                // A separated broad-phase box provides a valid conservative
                // lower bound. For overlapping boxes, however, zero is not an
                // exact shape distance and must not overwrite the exact result.
                // 中文翻译：已分离的宽相包围盒可提供保守下界；
                // 但包围盒相交时的零距离不是形体精确距离，不应覆盖 exact 结果。
                result.minimumDistanceMm = std::min(result.minimumDistanceMm,
                                                    broadDistance);
                continue;
            }
            const PairExactEvaluation evaluation = evaluatePairExact(
                m_impl->bodies.at(pair.firstBody), transforms.at(pair.firstBody),
                m_impl->bodies.at(pair.secondBody), transforms.at(pair.secondBody),
                index.clearanceMm(), true);
            if (!evaluation.done) {
                result.state = MachineSafetyIndexState::Unknown;
                result.failureReason = QStringLiteral("Exact machine safety distance failed");
                return result;
            }
            result.minimumDistanceMm = std::min(result.minimumDistanceMm,
                                                evaluation.distanceMm);
            if (evaluation.distanceMm
                <= index.clearanceMm() + Precision::Confusion()) {
                result.state = MachineSafetyIndexState::CollisionSample;
                result.blockingPair = pairIndex;
                return result;
            }
        }
        return result;
    } catch (const Standard_Failure& failure) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "Machine safety exact pose OCCT failure: {}",
                 failure.GetMessageString());
        result.state = MachineSafetyIndexState::Unknown;
        result.failureReason = QString::fromUtf8(failure.GetMessageString());
    } catch (const std::exception& failure) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "Machine safety exact pose failure: {}", failure.what());
        result.state = MachineSafetyIndexState::Unknown;
        result.failureReason = QString::fromUtf8(failure.what());
    } catch (...) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "Machine safety exact pose unknown failure");
        result.state = MachineSafetyIndexState::Unknown;
        result.failureReason = QStringLiteral("Unknown exact machine safety failure");
    }
    return result;
}

} // namespace lcnc::cam_algo
