#pragma once

#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

#include <cstddef>
#include <cstdint>
#include <array>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace lcnc::cam_algo {

struct SurfaceCollisionPrefilterResult;

struct SurfacePointDistanceResult
{
    bool valid{false};
    bool insideClosedSolid{false};
    double meshDistanceMm{std::numeric_limits<double>::infinity()};
    std::uint64_t bvhNodeTests{0};
    std::uint64_t triangleTests{0};
};

struct SurfaceTriangleSoup
{
    std::vector<std::array<double, 3>> vertices;
    std::vector<std::array<std::uint32_t, 3>> triangles;
};

/// Read-only surface mesh and BVH used to reject clearly separated collision
/// poses before entering OCCT's process-wide exact-operation gate.
/// 中文翻译：只读表面网格与 BVH，用于在进入 OCCT 全局精确运算锁前排除明显安全姿态。
class SurfaceCollisionModel final
{
public:
    SurfaceCollisionModel();
    ~SurfaceCollisionModel();
    SurfaceCollisionModel(const SurfaceCollisionModel&);
    SurfaceCollisionModel& operator=(const SurfaceCollisionModel&);
    SurfaceCollisionModel(SurfaceCollisionModel&&) noexcept;
    SurfaceCollisionModel& operator=(SurfaceCollisionModel&&) noexcept;

    static SurfaceCollisionModel build(const TopoDS_Shape& shape,
                                       double linearDeflectionMm,
                                       std::string* error = nullptr,
                                       std::size_t maximumTriangles = 1'000'000);
    static SurfaceCollisionModel buildFromTriangleSoup(
        const SurfaceTriangleSoup& soup,
        double linearDeflectionMm,
        bool closedSolid,
        std::string* error = nullptr);
    static SurfaceCollisionModel buildFromTriangleSoup(
        const SurfaceTriangleSoup& soup,
        const SurfaceTriangleSoup& containmentSoup,
        double linearDeflectionMm,
        bool closedSolid,
        std::string* error = nullptr);

    bool isValid() const;
    double linearDeflectionMm() const;
    std::size_t triangleCount() const;
    bool isClosedSolid() const;
    SurfaceTriangleSoup triangleSoup() const;
    SurfaceTriangleSoup containmentTriangleSoup() const;

    /// Returns the unsigned point-to-mesh distance in this model's local
    /// coordinates. For a closed solid, insideClosedSolid is also classified.
    /// The mesh deflection remains the caller's conservative error budget.
    SurfacePointDistanceResult pointDistance(const gp_Pnt& localPoint) const;

private:
    struct Impl;
    std::shared_ptr<const Impl> m_impl;

    explicit SurfaceCollisionModel(std::shared_ptr<const Impl> impl);
    static SurfaceCollisionModel buildInternal(const TopoDS_Shape& shape,
                                               double linearDeflectionMm,
                                               std::string* error,
                                               std::size_t maximumTriangles,
                                               bool buildContainmentModel);
    friend struct SurfaceCollisionPrefilterResult;
    friend SurfaceCollisionPrefilterResult prefilterSurfaceCollision(
        const SurfaceCollisionModel&, const gp_Trsf&,
        const SurfaceCollisionModel&, const gp_Trsf&, double);
    friend bool surfaceCollisionContainmentDetected(
        const SurfaceCollisionModel&, const gp_Trsf&,
        const SurfaceCollisionModel&, const gp_Trsf&);
};

struct SurfaceCollisionPrefilterResult
{
    bool valid{false};
    bool definitelySeparated{false};
    bool meshIntersection{false};
    double meshDistanceMm{std::numeric_limits<double>::infinity()};
    std::uint64_t bvhPairTests{0};
    std::uint64_t trianglePairTests{0};
};

/// Tests only surface triangles. A `definitelySeparated` result is conservative
/// when `searchDistanceMm` includes both models' mesh deflection. Any other
/// result must be confirmed by the existing exact BRep distance path.
/// 中文翻译：仅检测表面三角形；调用方把两侧网格误差加入搜索距离后，明确分离可安全跳过；
/// 其余结果仍由现有 BRep 精确距离复检。
SurfaceCollisionPrefilterResult prefilterSurfaceCollision(
    const SurfaceCollisionModel& active,
    const gp_Trsf& activeTransform,
    const SurfaceCollisionModel& passive,
    const gp_Trsf& passiveTransform,
    double searchDistanceMm);

/// Detects the closed-solid containment case that triangle-surface collision
/// libraries cannot classify as contact when the two surfaces do not cross.
/// 中文翻译：检测封闭实体完全包含但表面不相交的情况；三角面碰撞库不会把它识别为接触。
bool surfaceCollisionContainmentDetected(
    const SurfaceCollisionModel& active,
    const gp_Trsf& activeTransform,
    const SurfaceCollisionModel& passive,
    const gp_Trsf& passiveTransform);

} // namespace lcnc::cam_algo
