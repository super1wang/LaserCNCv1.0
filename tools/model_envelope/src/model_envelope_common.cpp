#include "model_envelope_common.h"

#include <BRepBuilderAPI_Copy.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <IMeshTools_Parameters.hxx>
#include <NCollection_Sequence.hxx>
#include <Poly_Triangle.hxx>
#include <Poly_Triangulation.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <Standard_Failure.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TDF_Label.hxx>
#include <TDataStd_Name.hxx>
#include <TDocStd_Document.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Face.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_Location.hxx>
#include <XCAFDoc_ShapeTool.hxx>

#include <QDataStream>
#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QMap>
#include <QRegularExpression>
#include <QSet>
#include <QStringList>

#ifdef Q_OS_WIN
#include <windows.h>
#include <psapi.h>
#endif

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <limits>
#include <numeric>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace lcnc::tools::model_envelope {
namespace {

constexpr double kVertexWeldToleranceMm = 1.0e-6;
constexpr double kRayEpsilon = 1.0e-9;

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

struct LocatedPart
{
    QString name;
    QString axis;
    TopoDS_Shape shape;
};

void appendRootOrComponents(const Handle(XCAFDoc_ShapeTool)& shapes,
                            const TDF_Label& root,
                            int rootIndex,
                            QVector<LocatedPart>* parts)
{
    NCollection_Sequence<TDF_Label> components;
    shapes->GetComponents(root, components);
    if (components.IsEmpty()) {
        const TopoDS_Shape shape = shapes->GetShape(root);
        if (shape.IsNull())
            return;
        QString name = labelName(root);
        if (name.isEmpty())
            name = QStringLiteral("Part_%1").arg(rootIndex);
        const QString axis = axisFromPartName(name);
        if (!axis.isEmpty())
            parts->append({name, axis, shape});
        return;
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
            parts->append({name, axis, shape});
    }
}

struct VertexKey
{
    qint64 x{0};
    qint64 y{0};
    qint64 z{0};

    bool operator==(const VertexKey& other) const noexcept
    {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct VertexKeyHash
{
    std::size_t operator()(const VertexKey& key) const noexcept
    {
        std::size_t seed = std::hash<qint64>{}(key.x);
        seed ^= std::hash<qint64>{}(key.y) + 0x9e3779b9u + (seed << 6) + (seed >> 2);
        seed ^= std::hash<qint64>{}(key.z) + 0x9e3779b9u + (seed << 6) + (seed >> 2);
        return seed;
    }
};

VertexKey vertexKey(const gp_Pnt& point)
{
    const double inverse = 1.0 / kVertexWeldToleranceMm;
    return {static_cast<qint64>(std::llround(point.X() * inverse)),
            static_cast<qint64>(std::llround(point.Y() * inverse)),
            static_cast<qint64>(std::llround(point.Z() * inverse))};
}

bool appendMeshedShape(const TopoDS_Shape& source,
                       double linearDeflectionMm,
                       double angularDeflectionRad,
                       TriangleMesh* output,
                       QString* errorMessage)
{
    std::fprintf(stderr, "  detaching B-Rep\n");
    std::fflush(stderr);
    BRepBuilderAPI_Copy copy(source, true, true);
    if (!copy.IsDone()) {
        *errorMessage = QStringLiteral("Unable to detach machine body before tessellation");
        return false;
    }
    const TopoDS_Shape privateShape = copy.Shape();
    std::fprintf(stderr, "  tessellating B-Rep\n");
    std::fflush(stderr);
    IMeshTools_Parameters parameters;
    parameters.Deflection = linearDeflectionMm;
    parameters.Angle = angularDeflectionRad;
    parameters.Relative = false;
    // Keep benchmark input deterministic and match the collision-surface
    // compiler's proven meshing path for this large assembly.
    parameters.InParallel = false;
    BRepMesh_IncrementalMesh mesher(privateShape, parameters);
    if (!mesher.IsDone()) {
        *errorMessage = QStringLiteral("OCC tessellation did not complete");
        return false;
    }
    std::fprintf(stderr, "  extracting triangles\n");
    std::fflush(stderr);

    std::unordered_map<VertexKey, std::uint32_t, VertexKeyHash> vertices;
    vertices.reserve(100'000);
    const auto addVertex = [&](const gp_Pnt& point) -> std::uint32_t {
        const VertexKey key = vertexKey(point);
        const auto found = vertices.find(key);
        if (found != vertices.end())
            return found->second;
        if (output->vertices.size() >= std::numeric_limits<std::uint32_t>::max())
            throw std::overflow_error("Envelope source vertex count exceeds uint32 range");
        const auto index = static_cast<std::uint32_t>(output->vertices.size());
        output->vertices.push_back({point.X(), point.Y(), point.Z()});
        vertices.emplace(key, index);
        return index;
    };

    for (TopExp_Explorer explorer(privateShape, TopAbs_FACE);
         explorer.More(); explorer.Next()) {
        const TopoDS_Face face = TopoDS::Face(explorer.Current());
        TopLoc_Location location;
        const Handle(Poly_Triangulation) triangulation =
            BRep_Tool::Triangulation(face, location);
        if (triangulation.IsNull())
            continue;
        const gp_Trsf transform = location.Transformation();
        for (int triangleIndex = 1;
             triangleIndex <= triangulation->NbTriangles(); ++triangleIndex) {
            int first = 0;
            int second = 0;
            int third = 0;
            triangulation->Triangle(triangleIndex).Get(first, second, third);
            if (face.Orientation() == TopAbs_REVERSED)
                std::swap(second, third);
            const std::array<std::uint32_t, 3> triangle{
                addVertex(triangulation->Node(first).Transformed(transform)),
                addVertex(triangulation->Node(second).Transformed(transform)),
                addVertex(triangulation->Node(third).Transformed(transform))};
            if (triangle[0] == triangle[1] || triangle[1] == triangle[2]
                || triangle[2] == triangle[0]) {
                continue;
            }
            output->triangles.push_back(triangle);
        }
    }
    if (output->triangles.empty()) {
        *errorMessage = QStringLiteral("Machine body tessellation produced no triangles");
        return false;
    }
    return true;
}

struct EdgeKey
{
    std::uint32_t first{0};
    std::uint32_t second{0};

    bool operator==(const EdgeKey& other) const noexcept
    {
        return first == other.first && second == other.second;
    }
};

struct EdgeKeyHash
{
    std::size_t operator()(const EdgeKey& edge) const noexcept
    {
        return (static_cast<std::size_t>(edge.first) << 32)
            ^ static_cast<std::size_t>(edge.second);
    }
};

EdgeKey edgeKey(std::uint32_t first, std::uint32_t second)
{
    return first < second ? EdgeKey{first, second} : EdgeKey{second, first};
}

Point3d operator+(const Point3d& first, const Point3d& second)
{
    return {first.x + second.x, first.y + second.y, first.z + second.z};
}

Point3d operator-(const Point3d& first, const Point3d& second)
{
    return {first.x - second.x, first.y - second.y, first.z - second.z};
}

Point3d operator*(const Point3d& value, double scalar)
{
    return {value.x * scalar, value.y * scalar, value.z * scalar};
}

double dot(const Point3d& first, const Point3d& second)
{
    return first.x * second.x + first.y * second.y + first.z * second.z;
}

Point3d cross(const Point3d& first, const Point3d& second)
{
    return {first.y * second.z - first.z * second.y,
            first.z * second.x - first.x * second.z,
            first.x * second.y - first.y * second.x};
}

double lengthSquared(const Point3d& value)
{
    return dot(value, value);
}

struct Bounds
{
    Point3d minimum{std::numeric_limits<double>::infinity(),
                    std::numeric_limits<double>::infinity(),
                    std::numeric_limits<double>::infinity()};
    Point3d maximum{-std::numeric_limits<double>::infinity(),
                    -std::numeric_limits<double>::infinity(),
                    -std::numeric_limits<double>::infinity()};

    void add(const Point3d& point)
    {
        minimum.x = std::min(minimum.x, point.x);
        minimum.y = std::min(minimum.y, point.y);
        minimum.z = std::min(minimum.z, point.z);
        maximum.x = std::max(maximum.x, point.x);
        maximum.y = std::max(maximum.y, point.y);
        maximum.z = std::max(maximum.z, point.z);
    }

    void add(const Bounds& bounds)
    {
        add(bounds.minimum);
        add(bounds.maximum);
    }
};

double component(const Point3d& point, int axis)
{
    return axis == 0 ? point.x : (axis == 1 ? point.y : point.z);
}

struct ValidationTriangle
{
    std::array<Point3d, 3> points;
    Bounds bounds;
    Point3d centroid;
};

struct BvhNode
{
    Bounds bounds;
    int begin{0};
    int end{0};
    int left{-1};
    int right{-1};

    bool leaf() const noexcept { return left < 0 && right < 0; }
};

class TriangleBvh
{
public:
    explicit TriangleBvh(const TriangleMesh& mesh)
    {
        m_triangles.reserve(mesh.triangles.size());
        for (const auto& indices : mesh.triangles) {
            if (indices[0] >= mesh.vertices.size()
                || indices[1] >= mesh.vertices.size()
                || indices[2] >= mesh.vertices.size()) {
                continue;
            }
            ValidationTriangle triangle;
            for (int point = 0; point < 3; ++point) {
                triangle.points[point] = mesh.vertices[indices[point]];
                triangle.bounds.add(triangle.points[point]);
            }
            triangle.centroid = (triangle.points[0] + triangle.points[1]
                                 + triangle.points[2]) * (1.0 / 3.0);
            m_triangles.push_back(triangle);
        }
        m_order.resize(m_triangles.size());
        std::iota(m_order.begin(), m_order.end(), 0);
        if (!m_order.empty())
            build(0, static_cast<int>(m_order.size()));
    }

    double distance(const Point3d& point) const
    {
        if (m_nodes.empty())
            return std::numeric_limits<double>::infinity();
        double best = std::numeric_limits<double>::infinity();
        std::vector<int> stack{0};
        while (!stack.empty()) {
            const int nodeIndex = stack.back();
            stack.pop_back();
            const BvhNode& node = m_nodes[nodeIndex];
            if (pointBoundsDistanceSquared(point, node.bounds) >= best)
                continue;
            if (node.leaf()) {
                for (int offset = node.begin; offset < node.end; ++offset) {
                    best = std::min(best, pointTriangleDistanceSquared(
                        point, m_triangles[m_order[offset]]));
                }
            } else {
                stack.push_back(node.left);
                stack.push_back(node.right);
            }
        }
        return std::sqrt(best);
    }

    bool inside(const Point3d& point) const
    {
        if (m_nodes.empty())
            return false;
        const std::array<Point3d, 3> directions{
            Point3d{1.0, 0.3713906763541037, 0.1591549430918953},
            Point3d{0.2718281828459045, 1.0, 0.1414213562373095},
            Point3d{0.1732050807568877, 0.2236067977499789, 1.0}};
        int insideVotes = 0;
        for (const Point3d& direction : directions) {
            int intersections = 0;
            std::vector<int> stack{0};
            while (!stack.empty()) {
                const int nodeIndex = stack.back();
                stack.pop_back();
                const BvhNode& node = m_nodes[nodeIndex];
                if (!rayIntersectsBounds(point, direction, node.bounds))
                    continue;
                if (node.leaf()) {
                    for (int offset = node.begin; offset < node.end; ++offset) {
                        if (rayIntersectsTriangle(point, direction,
                                                  m_triangles[m_order[offset]])) {
                            ++intersections;
                        }
                    }
                } else {
                    stack.push_back(node.left);
                    stack.push_back(node.right);
                }
            }
            if ((intersections % 2) != 0)
                ++insideVotes;
        }
        return insideVotes >= 2;
    }

private:
    int build(int begin, int end)
    {
        BvhNode node;
        node.begin = begin;
        node.end = end;
        for (int offset = begin; offset < end; ++offset)
            node.bounds.add(m_triangles[m_order[offset]].bounds);
        const int nodeIndex = static_cast<int>(m_nodes.size());
        m_nodes.push_back(node);
        if (end - begin <= 8)
            return nodeIndex;
        const Point3d extent = node.bounds.maximum - node.bounds.minimum;
        const std::array<double, 3> extents{extent.x, extent.y, extent.z};
        const int axis = static_cast<int>(std::distance(
            extents.begin(), std::max_element(extents.begin(), extents.end())));
        const int middle = begin + (end - begin) / 2;
        std::nth_element(m_order.begin() + begin, m_order.begin() + middle,
                         m_order.begin() + end, [&](int lhs, int rhs) {
            return component(m_triangles[lhs].centroid, axis)
                < component(m_triangles[rhs].centroid, axis);
        });
        const int left = build(begin, middle);
        const int right = build(middle, end);
        m_nodes[nodeIndex].left = left;
        m_nodes[nodeIndex].right = right;
        return nodeIndex;
    }

    static double pointBoundsDistanceSquared(const Point3d& point, const Bounds& bounds)
    {
        double result = 0.0;
        for (int axis = 0; axis < 3; ++axis) {
            const double value = component(point, axis);
            const double minimum = component(bounds.minimum, axis);
            const double maximum = component(bounds.maximum, axis);
            const double gap = value < minimum ? minimum - value
                             : (value > maximum ? value - maximum : 0.0);
            result += gap * gap;
        }
        return result;
    }

    static double pointTriangleDistanceSquared(const Point3d& point,
                                                const ValidationTriangle& triangle)
    {
        const Point3d a = triangle.points[0];
        const Point3d b = triangle.points[1];
        const Point3d c = triangle.points[2];
        const Point3d ab = b - a;
        const Point3d ac = c - a;
        const Point3d ap = point - a;
        const double d1 = dot(ab, ap);
        const double d2 = dot(ac, ap);
        if (d1 <= 0.0 && d2 <= 0.0)
            return lengthSquared(ap);
        const Point3d bp = point - b;
        const double d3 = dot(ab, bp);
        const double d4 = dot(ac, bp);
        if (d3 >= 0.0 && d4 <= d3)
            return lengthSquared(bp);
        const double vc = d1 * d4 - d3 * d2;
        if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
            const double v = d1 / (d1 - d3);
            return lengthSquared(point - (a + ab * v));
        }
        const Point3d cp = point - c;
        const double d5 = dot(ab, cp);
        const double d6 = dot(ac, cp);
        if (d6 >= 0.0 && d5 <= d6)
            return lengthSquared(cp);
        const double vb = d5 * d2 - d1 * d6;
        if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
            const double w = d2 / (d2 - d6);
            return lengthSquared(point - (a + ac * w));
        }
        const double va = d3 * d6 - d5 * d4;
        if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
            const Point3d bc = c - b;
            const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            return lengthSquared(point - (b + bc * w));
        }
        const double inverse = 1.0 / (va + vb + vc);
        const double v = vb * inverse;
        const double w = vc * inverse;
        return lengthSquared(point - (a + ab * v + ac * w));
    }

    static bool rayIntersectsTriangle(const Point3d& origin,
                                      const Point3d& direction,
                                      const ValidationTriangle& triangle)
    {
        const Point3d edge1 = triangle.points[1] - triangle.points[0];
        const Point3d edge2 = triangle.points[2] - triangle.points[0];
        const Point3d p = cross(direction, edge2);
        const double determinant = dot(edge1, p);
        if (std::abs(determinant) <= kRayEpsilon)
            return false;
        const double inverse = 1.0 / determinant;
        const Point3d offset = origin - triangle.points[0];
        const double u = dot(offset, p) * inverse;
        if (u < kRayEpsilon || u > 1.0 - kRayEpsilon)
            return false;
        const Point3d q = cross(offset, edge1);
        const double v = dot(direction, q) * inverse;
        if (v < kRayEpsilon || u + v > 1.0 - kRayEpsilon)
            return false;
        return dot(edge2, q) * inverse > kRayEpsilon;
    }

    static bool rayIntersectsBounds(const Point3d& origin,
                                    const Point3d& direction,
                                    const Bounds& bounds)
    {
        double nearValue = 0.0;
        double farValue = std::numeric_limits<double>::infinity();
        for (int axis = 0; axis < 3; ++axis) {
            const double delta = component(direction, axis);
            const double value = component(origin, axis);
            const double minimum = component(bounds.minimum, axis);
            const double maximum = component(bounds.maximum, axis);
            if (std::abs(delta) <= kRayEpsilon) {
                if (value < minimum || value > maximum)
                    return false;
                continue;
            }
            double first = (minimum - value) / delta;
            double second = (maximum - value) / delta;
            if (first > second)
                std::swap(first, second);
            nearValue = std::max(nearValue, first);
            farValue = std::min(farValue, second);
            if (nearValue > farValue)
                return false;
        }
        return farValue > kRayEpsilon;
    }

    std::vector<ValidationTriangle> m_triangles;
    std::vector<int> m_order;
    std::vector<BvhNode> m_nodes;
};

} // namespace

bool loadMachineBodyMeshes(const QString& stepPath,
                           double linearDeflectionMm,
                           double angularDeflectionRad,
                           QVector<MachineBodyMesh>* bodies,
                           ImportMetrics* metrics,
                           QString* errorMessage)
{
    if (bodies)
        bodies->clear();
    if (metrics)
        *metrics = {};
    if (!bodies || !QFileInfo(stepPath).isFile()
        || !std::isfinite(linearDeflectionMm) || linearDeflectionMm <= 0.0
        || !std::isfinite(angularDeflectionRad) || angularDeflectionRad <= 0.0) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Invalid machine envelope import request");
        return false;
    }
    try {
        QElapsedTimer timer;
        timer.start();
        Handle(TDocStd_Document) document =
            new TDocStd_Document(TCollection_ExtendedString("BinXCAF"));
        XCAFDoc_DocumentTool::Set(document->Main());
        STEPCAFControl_Reader reader;
        reader.SetNameMode(true);
        if (reader.ReadFile(stepPath.toUtf8().constData()) != IFSelect_RetDone
            || !reader.Transfer(document)) {
            if (errorMessage)
                *errorMessage = QStringLiteral("Unable to read or transfer STEP machine model");
            return false;
        }
        std::fprintf(stderr, "STEP transfer complete\n");
        std::fflush(stderr);
        if (metrics)
            metrics->stepImportMs = timer.elapsed();

        const Handle(XCAFDoc_ShapeTool) shapes =
            XCAFDoc_DocumentTool::ShapeTool(document->Main());
        NCollection_Sequence<TDF_Label> roots;
        shapes->GetFreeShapes(roots);
        QVector<LocatedPart> parts;
        for (int index = 1; index <= roots.Length(); ++index)
            appendRootOrComponents(shapes, roots.Value(index), index, &parts);
        std::fprintf(stderr, "Found %lld named machine bodies\n",
                     static_cast<long long>(parts.size()));
        std::fflush(stderr);
        if (parts.isEmpty()) {
            if (errorMessage) {
                *errorMessage = QStringLiteral(
                    "Machine model contains no LCNC_AXIS_* named bodies");
            }
            return false;
        }

        QMap<QString, QVector<LocatedPart>> byAxis;
        for (const LocatedPart& part : std::as_const(parts))
            byAxis[part.axis].append(part);
        static const QStringList axisOrder{
            QStringLiteral("BASE"), QStringLiteral("X"), QStringLiteral("Y"),
            QStringLiteral("Z"), QStringLiteral("A"), QStringLiteral("B"),
            QStringLiteral("C")};
        timer.restart();
        for (const QString& axis : axisOrder) {
            const auto group = byAxis.value(axis);
            if (group.isEmpty())
                continue;
            MachineBodyMesh body;
            body.axisName = axis;
            body.name = QStringLiteral("LCNC_AXIS_%1").arg(axis);
            for (const LocatedPart& part : group) {
                std::fprintf(stderr, "Meshing %s\n", body.name.toLatin1().constData());
                std::fflush(stderr);
                QString localError;
                if (!appendMeshedShape(part.shape, linearDeflectionMm,
                                       angularDeflectionRad, &body.mesh,
                                       &localError)) {
                    if (errorMessage) {
                        *errorMessage = QStringLiteral("%1: %2")
                            .arg(body.name, localError);
                    }
                    return false;
                }
            }
            if (metrics) {
                metrics->sourceVertices += body.mesh.vertices.size();
                metrics->sourceTriangles += body.mesh.triangles.size();
            }
            bodies->append(std::move(body));
        }
        if (metrics)
            metrics->tessellationMs = timer.elapsed();
        return !bodies->isEmpty();
    } catch (const Standard_Failure& failure) {
        if (errorMessage)
            *errorMessage = QString::fromUtf8(failure.what());
    } catch (const std::exception& failure) {
        if (errorMessage)
            *errorMessage = QString::fromUtf8(failure.what());
    } catch (...) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Unknown machine envelope import failure");
    }
    return false;
}

MeshValidationMetrics validateEnvelope(const TriangleMesh& source,
                                       const TriangleMesh& envelope,
                                       std::size_t maximumSamples)
{
    MeshValidationMetrics result;
    result.nonEmpty = !envelope.vertices.empty() && !envelope.triangles.empty();
    result.finite = result.nonEmpty;
    std::unordered_map<EdgeKey, std::uint32_t, EdgeKeyHash> edges;
    edges.reserve(envelope.triangles.size() * 2);
    for (const auto& point : envelope.vertices) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y)
            || !std::isfinite(point.z)) {
            result.finite = false;
        }
    }
    for (const auto& triangle : envelope.triangles) {
        if (triangle[0] >= envelope.vertices.size()
            || triangle[1] >= envelope.vertices.size()
            || triangle[2] >= envelope.vertices.size()) {
            result.finite = false;
            continue;
        }
        ++edges[edgeKey(triangle[0], triangle[1])];
        ++edges[edgeKey(triangle[1], triangle[2])];
        ++edges[edgeKey(triangle[2], triangle[0])];
    }
    for (const auto& [edge, count] : edges) {
        Q_UNUSED(edge);
        if (count == 1)
            ++result.boundaryEdges;
        else if (count != 2)
            ++result.nonManifoldEdges;
    }
    result.watertight = result.nonEmpty && result.boundaryEdges == 0;
    result.twoManifold = result.watertight && result.nonManifoldEdges == 0;
    result.minimumSampledMarginMm = std::numeric_limits<double>::infinity();
    if (!result.twoManifold || source.vertices.empty()) {
        result.minimumSampledMarginMm = 0.0;
        return result;
    }

    TriangleBvh bvh(envelope);
    const std::size_t sampleLimit = std::max<std::size_t>(1, maximumSamples);
    const std::size_t stride = std::max<std::size_t>(1,
        (source.vertices.size() + sampleLimit - 1) / sampleLimit);
    double sourceDistanceSum = 0.0;
    for (std::size_t index = 0; index < source.vertices.size(); index += stride) {
        const Point3d& point = source.vertices[index];
        const double distance = bvh.distance(point);
        const bool covered = distance <= kVertexWeldToleranceMm || bvh.inside(point);
        ++result.sampledSourcePoints;
        if (!covered)
            ++result.outsideSourcePoints;
        sourceDistanceSum += distance;
        result.maximumSampledSourceDistanceMm = std::max(
            result.maximumSampledSourceDistanceMm, distance);
        result.minimumSampledMarginMm = std::min(
            result.minimumSampledMarginMm, covered ? distance : -distance);
    }
    if (result.sampledSourcePoints > 0) {
        result.meanSampledSourceDistanceMm = sourceDistanceSum
            / static_cast<double>(result.sampledSourcePoints);
    }
    TriangleBvh sourceBvh(source);
    const std::size_t outputStride = std::max<std::size_t>(1,
        (envelope.vertices.size() + sampleLimit - 1) / sampleLimit);
    std::uint64_t sampledOutputPoints = 0;
    double outputDistanceSum = 0.0;
    for (std::size_t index = 0; index < envelope.vertices.size(); index += outputStride) {
        const double distance = sourceBvh.distance(envelope.vertices[index]);
        outputDistanceSum += distance;
        result.maximumSampledOutputDistanceMm = std::max(
            result.maximumSampledOutputDistanceMm, distance);
        ++sampledOutputPoints;
    }
    if (sampledOutputPoints > 0) {
        result.meanSampledOutputDistanceMm = outputDistanceSum
            / static_cast<double>(sampledOutputPoints);
    }
    if (!std::isfinite(result.minimumSampledMarginMm))
        result.minimumSampledMarginMm = 0.0;
    return result;
}

bool writeBinaryPly(const QString& path,
                    const TriangleMesh& mesh,
                    QString* errorMessage)
{
    QFile file(path);
    if (!QDir().mkpath(QFileInfo(path).absolutePath())
        || !file.open(QIODevice::WriteOnly)) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Unable to open PLY output: %1").arg(path);
        return false;
    }
    const QByteArray header = QStringLiteral(
        "ply\nformat binary_little_endian 1.0\n"
        "element vertex %1\nproperty float x\nproperty float y\nproperty float z\n"
        "element face %2\nproperty list uchar uint vertex_indices\nend_header\n")
        .arg(mesh.vertices.size()).arg(mesh.triangles.size()).toLatin1();
    if (file.write(header) != header.size()) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Unable to write PLY header");
        return false;
    }
    QDataStream stream(&file);
    stream.setByteOrder(QDataStream::LittleEndian);
    stream.setFloatingPointPrecision(QDataStream::SinglePrecision);
    for (const Point3d& point : mesh.vertices)
        stream << static_cast<float>(point.x)
               << static_cast<float>(point.y)
               << static_cast<float>(point.z);
    for (const auto& triangle : mesh.triangles) {
        stream << static_cast<quint8>(3)
               << static_cast<quint32>(triangle[0])
               << static_cast<quint32>(triangle[1])
               << static_cast<quint32>(triangle[2]);
    }
    if (stream.status() != QDataStream::Ok) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Unable to write PLY body");
        return false;
    }
    return true;
}

QString sanitizedBodyFileName(const QString& axisName, const QString& name)
{
    QString result = axisName.trimmed().toUpper() + QLatin1Char('_') + name.trimmed();
    result.replace(QRegularExpression(QStringLiteral("[^A-Za-z0-9_.-]+")),
                   QStringLiteral("_"));
    return result.isEmpty() ? QStringLiteral("body") : result;
}

QJsonObject importMetricsJson(const ImportMetrics& metrics)
{
    return {{QStringLiteral("step_import_ms"), metrics.stepImportMs},
            {QStringLiteral("tessellation_ms"), metrics.tessellationMs},
            {QStringLiteral("source_vertices"),
             static_cast<qint64>(metrics.sourceVertices)},
            {QStringLiteral("source_triangles"),
             static_cast<qint64>(metrics.sourceTriangles)}};
}

QJsonObject validationMetricsJson(const MeshValidationMetrics& metrics)
{
    return {{QStringLiteral("non_empty"), metrics.nonEmpty},
            {QStringLiteral("finite"), metrics.finite},
            {QStringLiteral("watertight"), metrics.watertight},
            {QStringLiteral("two_manifold"), metrics.twoManifold},
            {QStringLiteral("boundary_edges"),
             static_cast<qint64>(metrics.boundaryEdges)},
            {QStringLiteral("non_manifold_edges"),
             static_cast<qint64>(metrics.nonManifoldEdges)},
            {QStringLiteral("sampled_source_points"),
             static_cast<qint64>(metrics.sampledSourcePoints)},
            {QStringLiteral("outside_source_points"),
             static_cast<qint64>(metrics.outsideSourcePoints)},
            {QStringLiteral("minimum_sampled_margin_mm"),
             metrics.minimumSampledMarginMm},
            {QStringLiteral("mean_sampled_source_distance_mm"),
             metrics.meanSampledSourceDistanceMm},
            {QStringLiteral("maximum_sampled_source_distance_mm"),
             metrics.maximumSampledSourceDistanceMm},
            {QStringLiteral("mean_sampled_output_distance_mm"),
             metrics.meanSampledOutputDistanceMm},
            {QStringLiteral("maximum_sampled_output_distance_mm"),
             metrics.maximumSampledOutputDistanceMm}};
}

std::uint64_t peakWorkingSetBytes()
{
#ifdef Q_OS_WIN
    PROCESS_MEMORY_COUNTERS_EX counters{};
    counters.cb = sizeof(counters);
    if (GetProcessMemoryInfo(GetCurrentProcess(),
                             reinterpret_cast<PROCESS_MEMORY_COUNTERS*>(&counters),
                             sizeof(counters))) {
        return static_cast<std::uint64_t>(counters.PeakWorkingSetSize);
    }
#endif
    return 0;
}

std::uint64_t peakPrivateBytes()
{
#ifdef Q_OS_WIN
    PROCESS_MEMORY_COUNTERS_EX counters{};
    counters.cb = sizeof(counters);
    if (GetProcessMemoryInfo(GetCurrentProcess(),
                             reinterpret_cast<PROCESS_MEMORY_COUNTERS*>(&counters),
                             sizeof(counters))) {
        return static_cast<std::uint64_t>(counters.PrivateUsage);
    }
#endif
    return 0;
}

} // namespace lcnc::tools::model_envelope
