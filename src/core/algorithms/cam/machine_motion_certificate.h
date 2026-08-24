#pragma once

#include "core/algorithms/cam/machine_safety_index.h"

#include <QByteArray>
#include <QVector>

#include <array>
#include <cstdint>

namespace lcnc::cam_algo {

enum class MachineMotionInterpolationMode : std::uint8_t
{
    CoordinatedLinearAxes = 0
};

enum class MachineMotionAxisUnwrapMode : std::uint8_t
{
    PhysicalUnwrapped = 0
};

enum class MachineMotionCertificateState : std::uint8_t
{
    Invalid = 0,
    CertifiedSafe,
    Blocked,
    BoundaryUnknown
};

struct MachineMotionCertificateKey
{
    QByteArray machineSourceSha256;
    QByteArray safetyIndexSha256;
    QByteArray safetyPolicySha256;
    QByteArray motionProfileSha256;
    std::uint64_t pathRevision{0};
    std::uint64_t edgeId{0};
    MachineMotionInterpolationMode interpolation{
        MachineMotionInterpolationMode::CoordinatedLinearAxes};
    MachineMotionAxisUnwrapMode unwrap{
        MachineMotionAxisUnwrapMode::PhysicalUnwrapped};

    bool operator==(const MachineMotionCertificateKey& other) const;
    bool operator!=(const MachineMotionCertificateKey& other) const
    {
        return !(*this == other);
    }
};

struct MachineMotionUnknownInterval
{
    double parameterFirst{0.0};
    double parameterLast{0.0};
    int limitingPair{-1};
};

struct MachineMotionEdgeCertificate
{
    MachineMotionCertificateKey key;
    MachineSafetyPose first;
    MachineSafetyPose last;
    std::array<double, kMachineSafetyMaximumAxes> deviationTolerance{};
    /// Per-edge whole-turn offset used only for periodic index/geometry lookup.
    /// The physical unwrapped endpoints above remain authoritative online.
    std::array<double, kMachineSafetyMaximumAxes> canonicalAxisOffset{};
    MachineMotionCertificateState state{MachineMotionCertificateState::Invalid};
    QVector<MachineMotionUnknownInterval> unknownIntervals;
    std::uint64_t visitedFineCells{0};
    std::uint32_t parameterIntervals{0};
    int limitingPair{-1};
};

struct MachineMotionCertificationOptions
{
    std::array<double, kMachineSafetyMaximumAxes> deviationTolerance{};
    std::array<double, kMachineSafetyMaximumAxes> canonicalAxisOffset{};
    std::uint64_t maximumVisitedFineCells{250'000};
};

MachineMotionEdgeCertificate certifyLinearMotionEdgeWithIndex(
    const MachineSafetyIndex& index,
    const MachineMotionCertificateKey& key,
    const MachineSafetyPose& first,
    const MachineSafetyPose& last,
    const MachineMotionCertificationOptions& options = {});

bool machinePoseMatchesLinearMotionCertificate(
    const MachineMotionEdgeCertificate& certificate,
    const MachineMotionCertificateKey& expectedKey,
    double edgeParameter,
    const MachineSafetyPose& actualPose);

} // namespace lcnc::cam_algo
