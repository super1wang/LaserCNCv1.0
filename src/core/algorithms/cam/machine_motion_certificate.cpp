#include "core/algorithms/cam/machine_motion_certificate.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <vector>

namespace lcnc::cam_algo {
namespace {

constexpr double kParameterTolerance = 1e-12;
constexpr double kPoseTolerance = 1e-9;

std::vector<double> fineBoundaries(const MachineSafetyAxisGrid& axis)
{
    std::vector<double> result;
    result.reserve(static_cast<std::size_t>(axis.cellCount) * 2 + 1);
    result.push_back(axis.minimum);
    for (std::uint32_t cell = 0; cell < axis.cellCount; ++cell) {
        const double cellMinimum = axis.minimum + cell * axis.step;
        const double cellMaximum = std::min(axis.maximum,
                                             cellMinimum + axis.step);
        const double midpoint = (cellMinimum + cellMaximum) * 0.5;
        if (midpoint > result.back() + kPoseTolerance)
            result.push_back(midpoint);
        if (cellMaximum > result.back() + kPoseTolerance)
            result.push_back(cellMaximum);
    }
    return result;
}

void appendCrossingParameters(const std::vector<double>& boundaries,
                              double first, double last, double offset,
                              std::vector<double>* parameters)
{
    const double delta = last - first;
    if (std::abs(delta) <= kPoseTolerance)
        return;
    for (std::size_t boundary = 1; boundary + 1 < boundaries.size(); ++boundary) {
        const double parameter = (boundaries[boundary] - offset - first) / delta;
        if (parameter > kParameterTolerance
            && parameter < 1.0 - kParameterTolerance) {
            parameters->push_back(parameter);
        }
    }
}

std::vector<int> intersectedFineCells(const std::vector<double>& boundaries,
                                      double minimum, double maximum)
{
    std::vector<int> result;
    for (int cell = 0; cell + 1 < static_cast<int>(boundaries.size()); ++cell) {
        if (boundaries[static_cast<std::size_t>(cell + 1)] < minimum - kPoseTolerance
            || boundaries[static_cast<std::size_t>(cell)] > maximum + kPoseTolerance) {
            continue;
        }
        result.push_back(cell);
    }
    return result;
}

} // namespace

bool MachineMotionCertificateKey::operator==(
    const MachineMotionCertificateKey& other) const
{
    return machineSourceSha256 == other.machineSourceSha256
        && safetyIndexSha256 == other.safetyIndexSha256
        && safetyPolicySha256 == other.safetyPolicySha256
        && motionProfileSha256 == other.motionProfileSha256
        && pathRevision == other.pathRevision
        && edgeId == other.edgeId
        && interpolation == other.interpolation
        && unwrap == other.unwrap;
}

MachineMotionEdgeCertificate certifyLinearMotionEdgeWithIndex(
    const MachineSafetyIndex& index,
    const MachineMotionCertificateKey& key,
    const MachineSafetyPose& first,
    const MachineSafetyPose& last,
    const MachineMotionCertificationOptions& options)
{
    MachineMotionEdgeCertificate certificate;
    certificate.key = key;
    certificate.first = first;
    certificate.last = last;
    certificate.deviationTolerance = options.deviationTolerance;
    certificate.canonicalAxisOffset = options.canonicalAxisOffset;
    if (!index.isValid() || key.machineSourceSha256 != index.sourceSha256()
        || key.safetyIndexSha256.isEmpty()
        || key.safetyIndexSha256 != index.contentSha256()
        || key.safetyPolicySha256.isEmpty()
        || first.count != index.axes().size() || last.count != first.count
        || first.count == 0 || options.maximumVisitedFineCells == 0) {
        return certificate;
    }

    std::array<std::vector<double>, kMachineSafetyMaximumAxes> boundaries;
    std::vector<double> parameters{0.0, 1.0};
    for (int axis = 0; axis < first.count; ++axis) {
        const double tolerance = options.deviationTolerance[axis];
        if (!std::isfinite(first.values[axis]) || !std::isfinite(last.values[axis])
            || !std::isfinite(tolerance) || tolerance < 0.0) {
            return certificate;
        }
        const auto& grid = index.axes().at(axis);
        const double canonicalFirst = first.values[axis]
            - options.canonicalAxisOffset[axis];
        const double canonicalLast = last.values[axis]
            - options.canonicalAxisOffset[axis];
        const double pathMinimum = std::min(canonicalFirst, canonicalLast)
            - tolerance;
        const double pathMaximum = std::max(canonicalFirst, canonicalLast)
            + tolerance;
        if (pathMinimum < grid.minimum - kPoseTolerance
            || pathMaximum > grid.maximum + kPoseTolerance) {
            certificate.state = MachineMotionCertificateState::BoundaryUnknown;
            certificate.unknownIntervals.append({0.0, 1.0, -1});
            return certificate;
        }
        boundaries[axis] = fineBoundaries(grid);
        appendCrossingParameters(boundaries[axis], canonicalFirst,
                                 canonicalLast, tolerance, &parameters);
        if (tolerance > 0.0) {
            appendCrossingParameters(boundaries[axis], canonicalFirst,
                                     canonicalLast, -tolerance, &parameters);
        }
    }
    std::sort(parameters.begin(), parameters.end());
    parameters.erase(std::unique(parameters.begin(), parameters.end(),
                     [](double lhs, double rhs) {
                         return std::abs(lhs - rhs) <= kParameterTolerance;
                     }), parameters.end());

    certificate.state = MachineMotionCertificateState::CertifiedSafe;
    MachineSafetyPose query;
    query.count = first.count;
    for (std::size_t interval = 0; interval + 1 < parameters.size(); ++interval) {
        const double parameterFirst = parameters[interval];
        const double parameterLast = parameters[interval + 1];
        const double midpoint = (parameterFirst + parameterLast) * 0.5;
        ++certificate.parameterIntervals;
        std::array<std::vector<int>, kMachineSafetyMaximumAxes> cells;
        std::uint64_t combinations = 1;
        for (int axis = 0; axis < first.count; ++axis) {
            const double value = first.values[axis]
                + (last.values[axis] - first.values[axis]) * midpoint;
            const double canonicalValue = value
                - options.canonicalAxisOffset[axis];
            const double tolerance = options.deviationTolerance[axis];
            cells[axis] = intersectedFineCells(boundaries[axis],
                                               canonicalValue - tolerance,
                                               canonicalValue + tolerance);
            if (cells[axis].empty()) {
                certificate.state = MachineMotionCertificateState::BoundaryUnknown;
                certificate.unknownIntervals.append(
                    {parameterFirst, parameterLast, -1});
                combinations = 0;
                break;
            }
            if (combinations > options.maximumVisitedFineCells
                               / cells[axis].size()) {
                certificate.state = MachineMotionCertificateState::BoundaryUnknown;
                certificate.unknownIntervals.append(
                    {parameterFirst, parameterLast, -1});
                combinations = 0;
                break;
            }
            combinations *= cells[axis].size();
        }
        if (combinations == 0)
            continue;
        if (certificate.visitedFineCells + combinations
            > options.maximumVisitedFineCells) {
            certificate.state = MachineMotionCertificateState::BoundaryUnknown;
            certificate.unknownIntervals.append({parameterFirst, 1.0, -1});
            break;
        }

        bool intervalUnknown = false;
        int intervalPair = -1;
        std::function<bool(int)> visit = [&](int axis) {
            if (axis == query.count) {
                ++certificate.visitedFineCells;
                const auto result = index.query(query);
                if (result.state == MachineSafetyIndexState::CollisionSample) {
                    certificate.state = MachineMotionCertificateState::Blocked;
                    certificate.limitingPair = result.blockingPair;
                    return false;
                }
                if (result.state != MachineSafetyIndexState::CertifiedSafe) {
                    intervalUnknown = true;
                    if (intervalPair < 0)
                        intervalPair = result.blockingPair;
                }
                return true;
            }
            for (const int cell : cells[axis]) {
                const auto& axisBoundaries = boundaries[axis];
                query.values[axis] = (axisBoundaries[static_cast<std::size_t>(cell)]
                    + axisBoundaries[static_cast<std::size_t>(cell + 1)]) * 0.5;
                if (!visit(axis + 1))
                    return false;
            }
            return true;
        };
        if (!visit(0))
            return certificate;
        if (intervalUnknown) {
            certificate.state = MachineMotionCertificateState::BoundaryUnknown;
            certificate.unknownIntervals.append(
                {parameterFirst, parameterLast, intervalPair});
            if (certificate.limitingPair < 0)
                certificate.limitingPair = intervalPair;
        }
    }
    return certificate;
}

bool machinePoseMatchesLinearMotionCertificate(
    const MachineMotionEdgeCertificate& certificate,
    const MachineMotionCertificateKey& expectedKey,
    double edgeParameter,
    const MachineSafetyPose& actualPose)
{
    if (certificate.state != MachineMotionCertificateState::CertifiedSafe
        || certificate.key != expectedKey || !std::isfinite(edgeParameter)
        || edgeParameter < 0.0 || edgeParameter > 1.0
        || actualPose.count != certificate.first.count
        || actualPose.count != certificate.last.count) {
        return false;
    }
    for (int axis = 0; axis < actualPose.count; ++axis) {
        const double planned = certificate.first.values[axis]
            + (certificate.last.values[axis] - certificate.first.values[axis])
                * edgeParameter;
        if (!std::isfinite(actualPose.values[axis])
            || std::abs(actualPose.values[axis] - planned)
                > certificate.deviationTolerance[axis] + kPoseTolerance) {
            return false;
        }
    }
    return true;
}

} // namespace lcnc::cam_algo
