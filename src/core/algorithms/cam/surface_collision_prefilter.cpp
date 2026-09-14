#include "core/algorithms/cam/surface_collision_prefilter.h"

#include "core/logging/logger.h"

#include <BRepBuilderAPI_Copy.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <NCollection_Map.hxx>
#include <Poly_Triangle.hxx>
#include <Poly_Triangulation.hxx>
#include <Standard_Failure.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <algorithm>
#include <array>
#include <cmath>
#include <gp_Pnt.hxx>
#include <limits>
#include <numeric>
#include <utility>
#include <vector>

namespace lcnc::cam_algo {
namespace {

constexpr double kIntersectionToleranceSquared = 1.0e-12;
constexpr int kLeafTriangleCount = 8;

struct Vec3
{
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 operator*(const Vec3& a, double value) { return {a.x * value, a.y * value, a.z * value}; }

double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
Vec3 cross(const Vec3& a, const Vec3& b)
{
    return {a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
}
double lengthSquared(const Vec3& value) { return dot(value, value); }

Vec3 transformed(const Vec3& value, const gp_Trsf& transform)
{
    gp_Pnt point(value.x, value.y, value.z);
    point.Transform(transform);
    return {point.X(), point.Y(), point.Z()};
}

struct Aabb
{
    Vec3 minimum{std::numeric_limits<double>::infinity(),
                 std::numeric_limits<double>::infinity(),
                 std::numeric_limits<double>::infinity()};
    Vec3 maximum{-std::numeric_limits<double>::infinity(),
                 -std::numeric_limits<double>::infinity(),
                 -std::numeric_limits<double>::infinity()};

    bool valid() const
    {
        return minimum.x <= maximum.x && minimum.y <= maximum.y && minimum.z <= maximum.z;
    }

    void add(const Vec3& point)
    {
        minimum.x = std::min(minimum.x, point.x);
        minimum.y = std::min(minimum.y, point.y);
        minimum.z = std::min(minimum.z, point.z);
        maximum.x = std::max(maximum.x, point.x);
        maximum.y = std::max(maximum.y, point.y);
        maximum.z = std::max(maximum.z, point.z);
    }

    void add(const Aabb& other)
    {
        if (!other.valid()) return;
        add(other.minimum);
        add(other.maximum);
    }
};

double component(const Vec3& value, int axis)
{
    return axis == 0 ? value.x : (axis == 1 ? value.y : value.z);
}

double aabbDistanceSquared(const Aabb& first, const Aabb& second)
{
    double result = 0.0;
    for (int axis = 0; axis < 3; ++axis) {
        const double firstMin = component(first.minimum, axis);
        const double firstMax = component(first.maximum, axis);
        const double secondMin = component(second.minimum, axis);
        const double secondMax = component(second.maximum, axis);
        const double gap = firstMax < secondMin ? secondMin - firstMax
                          : (secondMax < firstMin ? firstMin - secondMax : 0.0);
        result += gap * gap;
    }
    return result;
}

double pointAabbDistanceSquared(const Vec3& point, const Aabb& box)
{
    double result = 0.0;
    for (int axis = 0; axis < 3; ++axis) {
        const double value = component(point, axis);
        const double minimum = component(box.minimum, axis);
        const double maximum = component(box.maximum, axis);
        const double gap = value < minimum ? minimum - value
                         : (value > maximum ? value - maximum : 0.0);
        result += gap * gap;
    }
    return result;
}

double aabbSurfaceArea(const Aabb& box)
{
    const Vec3 extent = box.maximum - box.minimum;
    return 2.0 * (extent.x * extent.y + extent.y * extent.z + extent.z * extent.x);
}

bool aabbContains(const Aabb& outer, const Aabb& inner)
{
    return outer.minimum.x <= inner.minimum.x && outer.minimum.y <= inner.minimum.y
        && outer.minimum.z <= inner.minimum.z && outer.maximum.x >= inner.maximum.x
        && outer.maximum.y >= inner.maximum.y && outer.maximum.z >= inner.maximum.z;
}

Aabb transformed(const Aabb& box, const gp_Trsf& transform)
{
    Aabb result;
    for (const double x : {box.minimum.x, box.maximum.x})
        for (const double y : {box.minimum.y, box.maximum.y})
            for (const double z : {box.minimum.z, box.maximum.z})
                result.add(transformed(Vec3{x, y, z}, transform));
    return result;
}

struct Triangle
{
    std::array<Vec3, 3> points;
    Aabb bounds;
    Vec3 centroid;
};

double pointTriangleDistanceSquared(const Vec3& point, const Triangle& triangle)
{
    const Vec3 a = triangle.points[0];
    const Vec3 b = triangle.points[1];
    const Vec3 c = triangle.points[2];
    const Vec3 ab = b - a;
    const Vec3 ac = c - a;
    const Vec3 ap = point - a;
    const double d1 = dot(ab, ap);
    const double d2 = dot(ac, ap);
    if (d1 <= 0.0 && d2 <= 0.0) return lengthSquared(ap);

    const Vec3 bp = point - b;
    const double d3 = dot(ab, bp);
    const double d4 = dot(ac, bp);
    if (d3 >= 0.0 && d4 <= d3) return lengthSquared(bp);

    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
        const double v = d1 / (d1 - d3);
        return lengthSquared(point - (a + ab * v));
    }

    const Vec3 cp = point - c;
    const double d5 = dot(ab, cp);
    const double d6 = dot(ac, cp);
    if (d6 >= 0.0 && d5 <= d6) return lengthSquared(cp);

    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
        const double w = d2 / (d2 - d6);
        return lengthSquared(point - (a + ac * w));
    }

    const double va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
        const Vec3 bc = c - b;
        const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return lengthSquared(point - (b + bc * w));
    }

    const double denominator = 1.0 / (va + vb + vc);
    const double v = vb * denominator;
    const double w = vc * denominator;
    return lengthSquared(point - (a + ab * v + ac * w));
}

double segmentSegmentDistanceSquared(const Vec3& p1, const Vec3& q1,
                                     const Vec3& p2, const Vec3& q2)
{
    constexpr double epsilon = 1.0e-15;
    const Vec3 d1 = q1 - p1;
    const Vec3 d2 = q2 - p2;
    const Vec3 r = p1 - p2;
    const double a = dot(d1, d1);
    const double e = dot(d2, d2);
    const double f = dot(d2, r);
    double s = 0.0;
    double t = 0.0;
    if (a <= epsilon && e <= epsilon) return lengthSquared(r);
    if (a <= epsilon) {
        t = std::clamp(f / e, 0.0, 1.0);
    } else {
        const double c = dot(d1, r);
        if (e <= epsilon) {
            s = std::clamp(-c / a, 0.0, 1.0);
        } else {
            const double b = dot(d1, d2);
            const double denominator = a * e - b * b;
            if (denominator != 0.0)
                s = std::clamp((b * f - c * e) / denominator, 0.0, 1.0);
            t = (b * s + f) / e;
            if (t < 0.0) {
                t = 0.0;
                s = std::clamp(-c / a, 0.0, 1.0);
            } else if (t > 1.0) {
                t = 1.0;
                s = std::clamp((b - c) / a, 0.0, 1.0);
            }
        }
    }
    return lengthSquared((p1 + d1 * s) - (p2 + d2 * t));
}

bool segmentIntersectsTriangle(const Vec3& start, const Vec3& end,
                               const Triangle& triangle)
{
    constexpr double epsilon = 1.0e-12;
    const Vec3 direction = end - start;
    const Vec3 edge1 = triangle.points[1] - triangle.points[0];
    const Vec3 edge2 = triangle.points[2] - triangle.points[0];
    const Vec3 p = cross(direction, edge2);
    const double determinant = dot(edge1, p);
    if (std::abs(determinant) <= epsilon)
        return false;
    const double inverse = 1.0 / determinant;
    const Vec3 offset = start - triangle.points[0];
    const double u = dot(offset, p) * inverse;
    if (u < -epsilon || u > 1.0 + epsilon)
        return false;
    const Vec3 q = cross(offset, edge1);
    const double v = dot(direction, q) * inverse;
    if (v < -epsilon || u + v > 1.0 + epsilon)
        return false;
    const double t = dot(edge2, q) * inverse;
    return t >= -epsilon && t <= 1.0 + epsilon;
}

bool rayIntersectsTriangle(const Vec3& origin, const Vec3& direction,
                           const Triangle& triangle, double* distance)
{
    constexpr double epsilon = 1.0e-12;
    const Vec3 edge1 = triangle.points[1] - triangle.points[0];
    const Vec3 edge2 = triangle.points[2] - triangle.points[0];
    const Vec3 p = cross(direction, edge2);
    const double determinant = dot(edge1, p);
    if (std::abs(determinant) <= epsilon)
        return false;
    const double inverse = 1.0 / determinant;
    const Vec3 offset = origin - triangle.points[0];
    const double u = dot(offset, p) * inverse;
    if (u < -epsilon || u > 1.0 + epsilon)
        return false;
    const Vec3 q = cross(offset, edge1);
    const double v = dot(direction, q) * inverse;
    if (v < -epsilon || u + v > 1.0 + epsilon)
        return false;
    const double t = dot(edge2, q) * inverse;
    if (t <= epsilon)
        return false;
    if (distance) *distance = t;
    return true;
}

bool rayIntersectsAabb(const Vec3& origin, const Vec3& direction, const Aabb& box)
{
    double nearValue = 0.0;
    double farValue = std::numeric_limits<double>::infinity();
    for (int axis = 0; axis < 3; ++axis) {
        const double value = component(origin, axis);
        const double delta = component(direction, axis);
        const double minimum = component(box.minimum, axis);
        const double maximum = component(box.maximum, axis);
        if (std::abs(delta) <= 1.0e-15) {
            if (value < minimum || value > maximum) return false;
            continue;
        }
        double first = (minimum - value) / delta;
        double second = (maximum - value) / delta;
        if (first > second) std::swap(first, second);
        nearValue = std::max(nearValue, first);
        farValue = std::min(farValue, second);
        if (nearValue > farValue) return false;
    }
    return farValue >= 0.0;
}

double triangleDistanceSquared(const Triangle& first, const Triangle& second)
{
    for (int edge = 0; edge < 3; ++edge) {
        if (segmentIntersectsTriangle(first.points[edge], first.points[(edge + 1) % 3], second)
            || segmentIntersectsTriangle(second.points[edge], second.points[(edge + 1) % 3], first)) {
            return 0.0;
        }
    }
    double result = std::numeric_limits<double>::infinity();
    for (const Vec3& point : first.points)
        result = std::min(result, pointTriangleDistanceSquared(point, second));
    for (const Vec3& point : second.points)
        result = std::min(result, pointTriangleDistanceSquared(point, first));
    for (int firstEdge = 0; firstEdge < 3; ++firstEdge)
        for (int secondEdge = 0; secondEdge < 3; ++secondEdge)
            result = std::min(result, segmentSegmentDistanceSquared(
                first.points[firstEdge], first.points[(firstEdge + 1) % 3],
                second.points[secondEdge], second.points[(secondEdge + 1) % 3]));
    return result;
}

Triangle transformed(const Triangle& source, const gp_Trsf& transform)
{
    Triangle result;
    for (int point = 0; point < 3; ++point) {
        result.points[point] = transformed(source.points[point], transform);
        result.bounds.add(result.points[point]);
    }
    result.centroid = (result.points[0] + result.points[1] + result.points[2]) * (1.0 / 3.0);
    return result;
}

} // namespace

struct SurfaceCollisionModel::Impl
{
    struct Node {
        Aabb bounds;
        int left{-1};
        int right{-1};
        int begin{0};
        int count{0};
        bool isLeaf() const { return left < 0; }
    };

    double linearDeflectionMm{0.0};
    std::vector<Triangle> triangles;
    std::vector<int> order;
    std::vector<Node> nodes;
    bool closedSolid{false};
    std::shared_ptr<const Impl> containmentModel;

    int buildNode(int begin, int end)
    {
        Node node;
        Aabb centroidBounds;
        for (int index = begin; index < end; ++index) {
            const Triangle& triangle = triangles[static_cast<std::size_t>(order[index])];
            node.bounds.add(triangle.bounds);
            centroidBounds.add(triangle.centroid);
        }
        const int nodeIndex = static_cast<int>(nodes.size());
        nodes.push_back(node);
        const int count = end - begin;
        if (count <= kLeafTriangleCount) {
            nodes[static_cast<std::size_t>(nodeIndex)].begin = begin;
            nodes[static_cast<std::size_t>(nodeIndex)].count = count;
            return nodeIndex;
        }
        const Vec3 extent = centroidBounds.maximum - centroidBounds.minimum;
        const int axis = extent.x >= extent.y && extent.x >= extent.z ? 0
                       : (extent.y >= extent.z ? 1 : 2);
        const int middle = begin + count / 2;
        std::nth_element(order.begin() + begin, order.begin() + middle, order.begin() + end,
            [this, axis](int first, int second) {
                return component(triangles[static_cast<std::size_t>(first)].centroid, axis)
                     < component(triangles[static_cast<std::size_t>(second)].centroid, axis);
            });
        const int left = buildNode(begin, middle);
        const int right = buildNode(middle, end);
        nodes[static_cast<std::size_t>(nodeIndex)].left = left;
        nodes[static_cast<std::size_t>(nodeIndex)].right = right;
        return nodeIndex;
    }

    bool containsPointRaw(const Vec3& point) const
    {
        if (nodes.empty())
            return false;
        const Vec3 direction{0.932381, 0.337129, 0.129731};
        std::vector<int> stack{0};
        std::vector<double> hits;
        while (!stack.empty()) {
            const int nodeIndex = stack.back();
            stack.pop_back();
            const Node& node = nodes[static_cast<std::size_t>(nodeIndex)];
            if (!rayIntersectsAabb(point, direction, node.bounds))
                continue;
            if (!node.isLeaf()) {
                stack.push_back(node.left);
                stack.push_back(node.right);
                continue;
            }
            for (int offset = 0; offset < node.count; ++offset) {
                const int triangleIndex = order[static_cast<std::size_t>(node.begin + offset)];
                double distance = 0.0;
                if (rayIntersectsTriangle(point, direction,
                                          triangles[static_cast<std::size_t>(triangleIndex)],
                                          &distance))
                    hits.push_back(distance);
            }
        }
        if (hits.empty())
            return false;
        std::sort(hits.begin(), hits.end());
        int uniqueHits = 0;
        double previous = -std::numeric_limits<double>::infinity();
        for (const double hit : hits) {
            if (std::abs(hit - previous) <= 1.0e-8)
                continue;
            previous = hit;
            ++uniqueHits;
        }
        return (uniqueHits & 1) != 0;
    }

    bool containsPoint(const Vec3& point) const
    {
        if (!closedSolid)
            return false;
        // A mixed body keeps open faces for distance queries, while point
        // containment must use only the union of its closed-solid faces.
        // 中文翻译：混合部件保留开放面用于距离查询，但点包含判定只能使用封闭实体面集合。
        return containmentModel ? containmentModel->containsPointRaw(point)
                                : containsPointRaw(point);
    }
};

SurfaceCollisionModel::SurfaceCollisionModel() = default;
SurfaceCollisionModel::~SurfaceCollisionModel() = default;
SurfaceCollisionModel::SurfaceCollisionModel(const SurfaceCollisionModel&) = default;
SurfaceCollisionModel& SurfaceCollisionModel::operator=(const SurfaceCollisionModel&) = default;
SurfaceCollisionModel::SurfaceCollisionModel(SurfaceCollisionModel&&) noexcept = default;
SurfaceCollisionModel& SurfaceCollisionModel::operator=(SurfaceCollisionModel&&) noexcept = default;
SurfaceCollisionModel::SurfaceCollisionModel(std::shared_ptr<const Impl> impl) : m_impl(std::move(impl)) {}

SurfaceCollisionModel SurfaceCollisionModel::build(const TopoDS_Shape& shape,
                                                   double linearDeflectionMm,
                                                   std::string* error,
                                                   std::size_t maximumTriangles)
{
    return buildInternal(shape, linearDeflectionMm, error,
                         maximumTriangles, true);
}

SurfaceCollisionModel SurfaceCollisionModel::buildInternal(
    const TopoDS_Shape& shape,
    double linearDeflectionMm,
    std::string* error,
    std::size_t maximumTriangles,
    bool buildContainmentModel)
{
    if (error) error->clear();
    if (shape.IsNull() || !std::isfinite(linearDeflectionMm) || linearDeflectionMm <= 0.0) {
        if (error) *error = "Invalid collision surface mesh input";
        return {};
    }
    try {
        BRepBuilderAPI_Copy copy(shape, true, true);
        if (!copy.IsDone()) {
            if (error) *error = "Cannot copy collision surface geometry";
            return {};
        }
        const TopoDS_Shape privateShape = copy.Shape();
        int solidCount = 0;
        NCollection_Map<TopoDS_Shape, TopTools_ShapeMapHasher> solidFaces;
        TopoDS_Compound solidCompound;
        BRep_Builder solidBuilder;
        solidBuilder.MakeCompound(solidCompound);
        for (TopExp_Explorer explorer(privateShape, TopAbs_SOLID);
             explorer.More(); explorer.Next()) {
            ++solidCount;
            solidBuilder.Add(solidCompound, explorer.Current());
            for (TopExp_Explorer faceExplorer(explorer.Current(), TopAbs_FACE);
                 faceExplorer.More(); faceExplorer.Next()) {
                solidFaces.Add(faceExplorer.Current());
            }
        }
        bool hasNonSolidFaces = false;
        for (TopExp_Explorer explorer(privateShape, TopAbs_FACE);
             explorer.More(); explorer.Next()) {
            if (!solidFaces.Contains(explorer.Current())) {
                hasNonSolidFaces = true;
                break;
            }
        }
        // Preserve every collision face. Restricting a mixed body to solid
        // outer shells drops its open shells/faces and can create false-safe
        // mesh queries even though the remaining solid mesh is closed.
        // 中文翻译：保留全部碰撞面；混合部件只取实体外壳会丢失开放壳/面，
        // 从而让网格查询产生假安全。
        const TopoDS_Shape surfaceShape = privateShape;
        BRepMesh_IncrementalMesh mesh(surfaceShape, linearDeflectionMm, false, 0.35, false);
        auto impl = std::make_shared<Impl>();
        impl->linearDeflectionMm = linearDeflectionMm;
        impl->closedSolid = solidCount > 0;
        for (TopExp_Explorer explorer(surfaceShape, TopAbs_FACE); explorer.More(); explorer.Next()) {
            TopLoc_Location location;
            const Handle(Poly_Triangulation) triangulation =
                BRep_Tool::Triangulation(TopoDS::Face(explorer.Current()), location);
            if (triangulation.IsNull())
                continue;
            const gp_Trsf faceTransform = location.Transformation();
            for (int index = 1; index <= triangulation->NbTriangles(); ++index) {
                if (impl->triangles.size() >= maximumTriangles) {
                    if (error) *error = "Collision surface mesh exceeds triangle limit";
                    return {};
                }
                int first = 0, second = 0, third = 0;
                triangulation->Triangle(index).Get(first, second, third);
                Triangle triangle;
                const std::array<int, 3> indices{first, second, third};
                for (int point = 0; point < 3; ++point) {
                    const gp_Pnt node = triangulation->Node(indices[point]).Transformed(faceTransform);
                    triangle.points[point] = {node.X(), node.Y(), node.Z()};
                    triangle.bounds.add(triangle.points[point]);
                }
                if (lengthSquared(cross(triangle.points[1] - triangle.points[0],
                                        triangle.points[2] - triangle.points[0])) <= 1.0e-24)
                    continue;
                triangle.centroid = (triangle.points[0] + triangle.points[1] + triangle.points[2])
                                  * (1.0 / 3.0);
                impl->triangles.push_back(std::move(triangle));
            }
        }
        if (impl->triangles.empty()) {
            if (error) *error = "Collision surface mesh contains no triangles";
            return {};
        }
        impl->order.resize(impl->triangles.size());
        std::iota(impl->order.begin(), impl->order.end(), 0);
        impl->nodes.reserve(impl->triangles.size() * 2);
        impl->buildNode(0, static_cast<int>(impl->triangles.size()));
        if (buildContainmentModel && solidCount > 0 && hasNonSolidFaces) {
            std::string containmentError;
            SurfaceCollisionModel containment = buildInternal(
                solidCompound, linearDeflectionMm, &containmentError,
                maximumTriangles, false);
            if (!containment.isValid()) {
                if (error) {
                    *error = "Cannot build closed-solid containment mesh: "
                        + containmentError;
                }
                return {};
            }
            impl->containmentModel = containment.m_impl;
        }
        return SurfaceCollisionModel(std::move(impl));
    } catch (const Standard_Failure& failure) {
        LCNC_ERR(lcnc::LogCode::Generic, "OCCT failed to build collision surface mesh: {}",
                 failure.what());
        if (error)
            *error = failure.what();
    } catch (const std::exception& exception) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "Failed to build collision surface mesh: {}", exception.what());
        if (error) *error = exception.what();
    } catch (...) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "Unknown failure while building collision surface mesh");
        if (error) *error = "Unknown collision surface mesh failure";
    }
    return {};
}

SurfaceCollisionModel SurfaceCollisionModel::buildFromTriangleSoup(
    const SurfaceTriangleSoup& soup,
    double linearDeflectionMm,
    bool closedSolid,
    std::string* error)
{
    return buildFromTriangleSoup(soup, {}, linearDeflectionMm,
                                 closedSolid, error);
}

SurfaceCollisionModel SurfaceCollisionModel::buildFromTriangleSoup(
    const SurfaceTriangleSoup& soup,
    const SurfaceTriangleSoup& containmentSoup,
    double linearDeflectionMm,
    bool closedSolid,
    std::string* error)
{
    if (error)
        error->clear();
    if (!std::isfinite(linearDeflectionMm) || linearDeflectionMm <= 0.0
        || soup.vertices.empty() || soup.triangles.empty()) {
        if (error)
            *error = "Invalid persisted collision surface mesh";
        return {};
    }
    try {
        auto impl = std::make_shared<Impl>();
        impl->linearDeflectionMm = linearDeflectionMm;
        impl->closedSolid = closedSolid;
        impl->triangles.reserve(soup.triangles.size());
        for (const auto& indices : soup.triangles) {
            if (indices[0] >= soup.vertices.size()
                || indices[1] >= soup.vertices.size()
                || indices[2] >= soup.vertices.size()) {
                if (error)
                    *error = "Persisted collision surface mesh index is invalid";
                return {};
            }
            Triangle triangle;
            for (int point = 0; point < 3; ++point) {
                const auto& vertex = soup.vertices[indices[point]];
                triangle.points[point] = {vertex[0], vertex[1], vertex[2]};
                triangle.bounds.add(triangle.points[point]);
            }
            if (lengthSquared(cross(triangle.points[1] - triangle.points[0],
                                    triangle.points[2] - triangle.points[0])) <= 1.0e-24)
                continue;
            triangle.centroid = (triangle.points[0] + triangle.points[1]
                                 + triangle.points[2]) * (1.0 / 3.0);
            impl->triangles.push_back(std::move(triangle));
        }
        if (impl->triangles.empty()) {
            if (error)
                *error = "Persisted collision surface mesh contains no usable triangles";
            return {};
        }
        impl->order.resize(impl->triangles.size());
        std::iota(impl->order.begin(), impl->order.end(), 0);
        impl->nodes.reserve(impl->triangles.size() * 2);
        impl->buildNode(0, static_cast<int>(impl->triangles.size()));
        if (!containmentSoup.triangles.empty()) {
            if (!closedSolid) {
                if (error)
                    *error = "Open collision mesh cannot carry a containment mesh";
                return {};
            }
            std::string containmentError;
            SurfaceCollisionModel containment = buildFromTriangleSoup(
                containmentSoup, linearDeflectionMm, true, &containmentError);
            if (!containment.isValid()) {
                if (error) {
                    *error = "Cannot restore closed-solid containment mesh: "
                        + containmentError;
                }
                return {};
            }
            impl->containmentModel = containment.m_impl;
        }
        return SurfaceCollisionModel(std::move(impl));
    } catch (const std::exception& exception) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "Failed to restore collision surface mesh: {}", exception.what());
        if (error)
            *error = exception.what();
    } catch (...) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "Unknown failure while restoring collision surface mesh");
        if (error)
            *error = "Unknown persisted collision surface mesh failure";
    }
    return {};
}

bool SurfaceCollisionModel::isValid() const
{
    return m_impl && !m_impl->triangles.empty() && !m_impl->nodes.empty();
}

double SurfaceCollisionModel::linearDeflectionMm() const
{
    return m_impl ? m_impl->linearDeflectionMm : 0.0;
}

std::size_t SurfaceCollisionModel::triangleCount() const
{
    return m_impl ? m_impl->triangles.size() : 0;
}

bool SurfaceCollisionModel::isClosedSolid() const
{
    return m_impl && m_impl->closedSolid;
}

SurfaceTriangleSoup SurfaceCollisionModel::triangleSoup() const
{
    SurfaceTriangleSoup result;
    if (!m_impl)
        return result;
    result.vertices.reserve(m_impl->triangles.size() * 3);
    result.triangles.reserve(m_impl->triangles.size());
    for (const Triangle& triangle : m_impl->triangles) {
        const std::uint32_t first = static_cast<std::uint32_t>(result.vertices.size());
        for (const Vec3& point : triangle.points)
            result.vertices.push_back({point.x, point.y, point.z});
        result.triangles.push_back({first, first + 1, first + 2});
    }
    return result;
}

SurfaceTriangleSoup SurfaceCollisionModel::containmentTriangleSoup() const
{
    SurfaceTriangleSoup result;
    if (!m_impl || !m_impl->containmentModel)
        return result;
    SurfaceCollisionModel containment(m_impl->containmentModel);
    return containment.triangleSoup();
}

SurfacePointDistanceResult SurfaceCollisionModel::pointDistance(
    const gp_Pnt& localPoint) const
{
    SurfacePointDistanceResult result;
    if (!isValid())
        return result;
    const Vec3 point{localPoint.X(), localPoint.Y(), localPoint.Z()};
    double bestSquared = std::numeric_limits<double>::infinity();
    std::vector<int> stack{0};
    while (!stack.empty()) {
        const int nodeIndex = stack.back();
        stack.pop_back();
        const Impl::Node& node = m_impl->nodes[
            static_cast<std::size_t>(nodeIndex)];
        ++result.bvhNodeTests;
        if (pointAabbDistanceSquared(point, node.bounds) >= bestSquared)
            continue;
        if (!node.isLeaf()) {
            const double leftDistance = pointAabbDistanceSquared(
                point, m_impl->nodes[static_cast<std::size_t>(node.left)].bounds);
            const double rightDistance = pointAabbDistanceSquared(
                point, m_impl->nodes[static_cast<std::size_t>(node.right)].bounds);
            if (leftDistance < rightDistance) {
                stack.push_back(node.right);
                stack.push_back(node.left);
            } else {
                stack.push_back(node.left);
                stack.push_back(node.right);
            }
            continue;
        }
        for (int offset = 0; offset < node.count; ++offset) {
            const int triangleIndex = m_impl->order[
                static_cast<std::size_t>(node.begin + offset)];
            const Triangle& triangle = m_impl->triangles[
                static_cast<std::size_t>(triangleIndex)];
            if (pointAabbDistanceSquared(point, triangle.bounds) >= bestSquared)
                continue;
            ++result.triangleTests;
            bestSquared = std::min(
                bestSquared, pointTriangleDistanceSquared(point, triangle));
        }
    }
    result.valid = std::isfinite(bestSquared);
    result.meshDistanceMm = result.valid
        ? std::sqrt(std::max(0.0, bestSquared))
        : std::numeric_limits<double>::infinity();
    result.insideClosedSolid = result.valid && m_impl->containsPoint(point);
    return result;
}

bool surfaceCollisionContainmentDetected(
    const SurfaceCollisionModel& active,
    const gp_Trsf& activeTransform,
    const SurfaceCollisionModel& passive,
    const gp_Trsf& passiveTransform)
{
    if (!active.isValid() || !passive.isValid()
        || !active.m_impl->closedSolid || !passive.m_impl->closedSolid) {
        return false;
    }
    gp_Trsf relative = passiveTransform.Inverted();
    relative.Multiply(activeTransform);
    const Aabb movedActiveRoot = transformed(
        active.m_impl->nodes.front().bounds, relative);
    const Aabb& passiveRoot = passive.m_impl->nodes.front().bounds;
    if (aabbContains(passiveRoot, movedActiveRoot)) {
        const Vec3 activeCenter =
            (movedActiveRoot.minimum + movedActiveRoot.maximum) * 0.5;
        if (passive.m_impl->containsPoint(activeCenter))
            return true;
    }
    if (aabbContains(movedActiveRoot, passiveRoot)) {
        const gp_Trsf inverseRelative = relative.Inverted();
        const Vec3 passiveCenter =
            (passiveRoot.minimum + passiveRoot.maximum) * 0.5;
        if (active.m_impl->containsPoint(
                transformed(passiveCenter, inverseRelative))) {
            return true;
        }
    }
    return false;
}

SurfaceCollisionPrefilterResult prefilterSurfaceCollision(
    const SurfaceCollisionModel& active,
    const gp_Trsf& activeTransform,
    const SurfaceCollisionModel& passive,
    const gp_Trsf& passiveTransform,
    double searchDistanceMm)
{
    SurfaceCollisionPrefilterResult result;
    if (!active.isValid() || !passive.isValid()
        || !std::isfinite(searchDistanceMm) || searchDistanceMm < 0.0)
        return result;
    result.valid = true;

    gp_Trsf relative = passiveTransform.Inverted();
    relative.Multiply(activeTransform);
    const double thresholdSquared = searchDistanceMm * searchDistanceMm;
    struct NodePair { int active; int passive; };
    std::vector<NodePair> stack;
    stack.push_back({0, 0});
    std::vector<Aabb> transformedActiveBounds(active.m_impl->nodes.size());
    std::vector<unsigned char> transformedActiveBoundsReady(active.m_impl->nodes.size(), 0);
    std::vector<Triangle> transformedActiveTriangles(active.m_impl->triangles.size());
    std::vector<unsigned char> transformedActiveTrianglesReady(active.m_impl->triangles.size(), 0);
    const auto activeBounds = [&](int nodeIndex) -> const Aabb& {
        const std::size_t index = static_cast<std::size_t>(nodeIndex);
        if (!transformedActiveBoundsReady[index]) {
            transformedActiveBounds[index] = transformed(active.m_impl->nodes[index].bounds, relative);
            transformedActiveBoundsReady[index] = 1;
        }
        return transformedActiveBounds[index];
    };
    const auto activeTriangle = [&](int triangleIndex) -> const Triangle& {
        const std::size_t index = static_cast<std::size_t>(triangleIndex);
        if (!transformedActiveTrianglesReady[index]) {
            transformedActiveTriangles[index] = transformed(active.m_impl->triangles[index], relative);
            transformedActiveTrianglesReady[index] = 1;
        }
        return transformedActiveTriangles[index];
    };
    double bestSquared = std::numeric_limits<double>::infinity();
    while (!stack.empty()) {
        const NodePair pair = stack.back();
        stack.pop_back();
        ++result.bvhPairTests;
        const auto& activeNode = active.m_impl->nodes[static_cast<std::size_t>(pair.active)];
        const auto& passiveNode = passive.m_impl->nodes[static_cast<std::size_t>(pair.passive)];
        const Aabb& movedActiveNodeBounds = activeBounds(pair.active);
        if (aabbDistanceSquared(movedActiveNodeBounds, passiveNode.bounds)
            > thresholdSquared)
            continue;
        if (activeNode.isLeaf() && passiveNode.isLeaf()) {
            for (int activeOffset = 0; activeOffset < activeNode.count; ++activeOffset) {
                const int activeTriangleIndex = active.m_impl->order[
                    static_cast<std::size_t>(activeNode.begin + activeOffset)];
                const Triangle& movedActive = activeTriangle(activeTriangleIndex);
                for (int passiveOffset = 0; passiveOffset < passiveNode.count; ++passiveOffset) {
                    const int passiveTriangleIndex = passive.m_impl->order[
                        static_cast<std::size_t>(passiveNode.begin + passiveOffset)];
                    const Triangle& passiveTriangle = passive.m_impl->triangles[
                        static_cast<std::size_t>(passiveTriangleIndex)];
                    if (aabbDistanceSquared(movedActive.bounds, passiveTriangle.bounds)
                        > thresholdSquared)
                        continue;
                    ++result.trianglePairTests;
                    bestSquared = std::min(bestSquared,
                        triangleDistanceSquared(movedActive, passiveTriangle));
                    if (bestSquared <= kIntersectionToleranceSquared) {
                        result.meshIntersection = true;
                        result.meshDistanceMm = 0.0;
                        return result;
                    }
                }
            }
            continue;
        }
        if (activeNode.isLeaf()) {
            stack.push_back({pair.active, passiveNode.left});
            stack.push_back({pair.active, passiveNode.right});
        } else if (passiveNode.isLeaf()) {
            stack.push_back({activeNode.left, pair.passive});
            stack.push_back({activeNode.right, pair.passive});
        } else if (aabbSurfaceArea(movedActiveNodeBounds) >= aabbSurfaceArea(passiveNode.bounds)) {
            // Split one hierarchy at a time. Expanding both sides creates four
            // pairs and repeatedly transforms the same active node; splitting
            // the spatially larger side preserves pruning while keeping the
            // traversal close to linear in the relevant surface patch.
            stack.push_back({activeNode.left, pair.passive});
            stack.push_back({activeNode.right, pair.passive});
        } else {
            stack.push_back({pair.active, passiveNode.left});
            stack.push_back({pair.active, passiveNode.right});
        }
    }
    result.meshDistanceMm = std::sqrt(bestSquared);
    result.definitelySeparated = !std::isfinite(bestSquared) || bestSquared > thresholdSquared;
    if (result.definitelySeparated
        && surfaceCollisionContainmentDetected(
            active, activeTransform, passive, passiveTransform)) {
        result.definitelySeparated = false;
    }
    return result;
}

} // namespace lcnc::cam_algo
