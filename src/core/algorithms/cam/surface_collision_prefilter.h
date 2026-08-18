#pragma once

#include <TopoDS_Shape.hxx>
#include <gp_Trsf.hxx>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>

namespace lcnc::cam_algo {

struct SurfaceCollisionPrefilterResult;

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

    bool isValid() const;
    double linearDeflectionMm() const;
    std::size_t triangleCount() const;

private:
    struct Impl;
    std::shared_ptr<const Impl> m_impl;

    explicit SurfaceCollisionModel(std::shared_ptr<const Impl> impl);
    friend struct SurfaceCollisionPrefilterResult;
    friend SurfaceCollisionPrefilterResult prefilterSurfaceCollision(
        const SurfaceCollisionModel&, const gp_Trsf&,
        const SurfaceCollisionModel&, const gp_Trsf&, double);
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

} // namespace lcnc::cam_algo
