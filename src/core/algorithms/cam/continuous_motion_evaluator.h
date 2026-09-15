#pragma once

#include "core/project/cam/collision_validation_contracts.h"

#include <array>
#include <functional>

namespace lcnc::cam_algo {

struct EvaluatedMotionState
{
    std::array<double, MachineAxisLayout::kMaxAxes> physicalAxes{};
    std::uint8_t physicalAxisMask{0};
    double worldTcpX{0.0};
    double worldTcpY{0.0};
    double worldTcpZ{0.0};
    double referenceTcpX{0.0};
    double referenceTcpY{0.0};
    double referenceTcpZ{0.0};
    bool referenceTcpValid{false};
    double processDirectionX{0.0};
    double processDirectionY{0.0};
    double processDirectionZ{1.0};
    double sourceParameter{0.0};
};

struct MotionIntervalBound
{
    std::array<double, MachineAxisLayout::kMaxAxes> minimumPhysicalAxes{};
    std::array<double, MachineAxisLayout::kMaxAxes> maximumPhysicalAxes{};
    double minimumWorldTcpX{0.0};
    double minimumWorldTcpY{0.0};
    double minimumWorldTcpZ{0.0};
    double maximumWorldTcpX{0.0};
    double maximumWorldTcpY{0.0};
    double maximumWorldTcpZ{0.0};
    double maximumPositionErrorMm{0.0};
    double maximumOrientationErrorDegrees{0.0};
    bool refinable{false};
    bool valid{false};
};

/// Model callbacks are supplied by the CAM-owned kinematics/controller
/// qualification layer. They contain no collision backend and must implement
/// the interpolation model named by the plan identity.
struct MotionEvaluationContext
{
    std::uint32_t interpolationModelVersion{0};
    std::function<bool(
        const std::array<double, MachineAxisLayout::kMaxAxes>&,
        std::uint8_t, EvaluatedMotionState*)> evaluatePhysicalAxes;
    std::function<bool(const lcnc::cam::CamMotionBlock&, double,
                       EvaluatedMotionState*)> evaluateRtcp;
    std::function<bool(const lcnc::cam::CamMotionBlock&, double, double,
                       MotionIntervalBound*)> boundPhysicalAxes;
    std::function<bool(const lcnc::cam::CamMotionBlock&, double, double,
                       MotionIntervalBound*)> boundRtcp;
};

class ContinuousMotionEvaluator final
{
public:
    bool evaluate(const lcnc::cam::CamMotionBlock& block, double u,
                  const MotionEvaluationContext& context,
                  EvaluatedMotionState* state,
                  QString* errorMessage = nullptr) const;

    bool bound(const lcnc::cam::CamMotionBlock& block, double u0, double u1,
               const MotionEvaluationContext& context,
               MotionIntervalBound* result,
               QString* errorMessage = nullptr) const;
};

} // namespace lcnc::cam_algo
