#pragma once

#include "modules/process/runtime/prepared_device_program.h"

namespace lcnc::process {
bool validateGtnLoweringProfile(const PreparedDeviceProgram&, QString* error);

struct GtnEncodedDynamics {
    double velocity{0};
    double acceleration{0};
    double jerk{0};
    GtnFeedMetric metric{GtnFeedMetric::Unavailable};
};

struct GtnExactCommand {
    enum class Kind { LinearAbsolute, Fence };
    Kind kind{Kind::LinearAbsolute};
    qint32 userTag{0};
    std::uint64_t blockId{0};
    std::uint64_t contourId{0};
    QByteArray blockHash;
    int knotIndex{-1};
    int sourceEdgeIndex{-1};
    double sourceParameter{0};
    int departureSourceEdgeIndex{-1};
    double departureSourceParameter{0};
    cam::CamMotionPhase phase{cam::CamMotionPhase::Cutting};
    std::uint8_t activeGroupSlotMask{0};
    std::array<double, 5> target{};
    std::array<double, 5> predictedPhysical{};
    GtnEncodedDynamics dynamics;
    cam::MotionProcessFence fence;
};

// Endpoint checking remains mandatory for RTCP and is separate from the
// qualification of continuous interpolation. The production implementation
// calls ValidateGroupRtcpTarget on the existing device queue after Group setup.
using GtnRtcpTargetValidator = std::function<bool(
    const std::array<double, 5>&, const std::array<double, 5>&, QString*)>;

class GtnEncodedSection final {
public:
    static std::shared_ptr<const GtnEncodedSection> lower(
        const PreparedDeviceProgram& program, int ordinal,
        const GtnRtcpTargetValidator& validateRtcp,
        const std::function<bool()>& cancelled, QString* error);

    const QVector<GtnExactCommand>& commands() const { return m_commands; }
    const GtnLoweringProfile& profile() const { return m_profile; }
    const FrozenToolExecutionRecipe& tool() const { return m_tool; }
    const QByteArray& encodingHash() const { return m_encodingHash; }
    const QByteArray& planHash() const { return m_planHash; }
    const QByteArray& contextHash() const { return m_contextHash; }
    const QByteArray& recipeRevision() const { return m_recipeRevision; }
    std::uint64_t runEpoch() const { return m_runEpoch; }
    int ordinal() const { return m_ordinal; }
    // Full Group membership is independent of each command's moving mask.
    static constexpr std::uint8_t groupMembershipMask() { return 0x1f; }

private:
    GtnEncodedSection() = default;
    QVector<GtnExactCommand> m_commands;
    GtnLoweringProfile m_profile;
    FrozenToolExecutionRecipe m_tool;
    QByteArray m_encodingHash, m_planHash, m_contextHash, m_recipeRevision;
    std::uint64_t m_runEpoch{0};
    int m_ordinal{-1};
};

// Admit the complete finite run before any section can be submitted. A later
// unsupported cell/target must not be discovered after earlier motion starts.
class GtnEncodedProgram final {
public:
    static std::shared_ptr<const GtnEncodedProgram> lower(
        const PreparedDeviceProgram&, const GtnRtcpTargetValidator&,
        const std::function<bool()>& cancelled, QString* error);
    const QVector<std::shared_ptr<const GtnEncodedSection>>& sections() const { return m_sections; }
    const QByteArray& encodingHash() const { return m_encodingHash; }
private:
    GtnEncodedProgram() = default;
    QVector<std::shared_ptr<const GtnEncodedSection>> m_sections;
    QByteArray m_encodingHash;
};

} // namespace lcnc::process
