#include "modules/process/runtime/gtn_exact_plan_lowering.h"

#include <QCryptographicHash>
#include <QDataStream>
#include <QIODevice>
#include <QSet>
#include <cmath>
#include <limits>

namespace lcnc::process {
namespace {
bool reject(QString* error, const char* reason)
{
    if (error) *error = QString::fromLatin1(reason);
    return false;
}

bool rotary(MachineAxisRole role)
{
    switch (role) {
    case MachineAxisRole::WorkpieceRotary:
    case MachineAxisRole::TableTilt:
    case MachineAxisRole::TableSpin:
    case MachineAxisRole::HeadTiltPrimary:
    case MachineAxisRole::HeadTiltSecondary: return true;
    default: return false;
    }
}

bool validateProfile(const PreparedDeviceProgram& program, QString* error)
{
    const auto& p = program.recipe().gtnLowering;
    const auto& context = program.plan().context;
    const auto& q = context.controllerQualification;
    if (p.state != cam::ControllerQualificationState::Qualified || !p.revision
        || p.sourceId.isEmpty() || p.groupSemanticsId.isEmpty()
        || !cam::controllerQualificationIsQualified(q))
        return reject(error, "GTN lowering qualification unavailable");
    if (q.capabilityFingerprint != gtnLoweringProfileHash(p)
        || context.controllerCapabilityHash != cam::controllerQualificationSnapshotHash(q)
        || q.qualificationRevision != p.revision || q.sourceId != p.sourceId
        || q.requestedMode != p.mode || context.controllerMode != p.mode)
        return reject(error, "GTN lowering qualification identity mismatch");
    if (p.mode != cam::ControllerMotionMode::PhysicalAxes && p.mode != cam::ControllerMotionMode::RTCP)
        return reject(error, "Unsupported GTN controller mode");
    if (context.interpolationModelVersion != 1)
        return reject(error, "Unsupported GTN interpolation model version");
    if (!p.absoluteRotaryTurnsQualified)
        return reject(error, "Absolute unwrapped rotary semantics are unqualified");
    if (program.layout().count != 5)
        return reject(error, "Qualified GTN Group requires an explicit five-axis layout");
    QSet<int> physical, controller;
    const MachineAxisRole linearRoles[] = {MachineAxisRole::LinearX, MachineAxisRole::LinearY,
                                          MachineAxisRole::LinearZ};
    for (int slot = 0; slot < 5; ++slot) {
        const auto& axis = p.axes[slot];
        if (axis.physicalIndex < 0 || axis.physicalIndex >= 5
            || physical.contains(axis.physicalIndex) || axis.controllerAxis < 1
            || axis.controllerAxis > 32 || controller.contains(axis.controllerAxis))
            return reject(error, "Missing, duplicate or invalid GTN axis mapping");
        const auto& expected = program.layout().axes[axis.physicalIndex];
        if (axis.name != expected.name || axis.role != expected.role
            || (slot < 3 ? axis.role != linearRoles[slot] : !rotary(axis.role)))
            return reject(error, "GTN axis mapping name/role mismatch");
        if (!std::isfinite(axis.minimum) || !std::isfinite(axis.maximum) || axis.minimum >= axis.maximum)
            return reject(error, "GTN physical axis limits unavailable");
        physical.insert(axis.physicalIndex);
        controller.insert(axis.controllerAxis);
    }
    if (!std::isfinite(p.rapidFeedMmPerSecond) || p.rapidFeedMmPerSecond <= 0
        || !std::isfinite(p.surfaceRadiusMm) || p.surfaceRadiusMm < 0)
        return reject(error, "Invalid qualified feed parameters");
    for (double ratio : p.referenceRatios)
        if (!std::isfinite(ratio) || ratio < 0)
            return reject(error, "Invalid qualified velocity reference ratios");
    switch (p.metric) {
    case GtnFeedMetric::LinearMillimetres:
        if (p.referenceRatios != std::array<double, 5>{1, 1, 1, 0, 0} || p.surfaceRadiusMm != 0)
            return reject(error, "Linear feed requires XYZ millimetre reference axes");
        break;
    case GtnFeedMetric::WeightedMillimetres:
        if (p.referenceRatios[0] != 1 || p.referenceRatios[1] != 1 || p.referenceRatios[2] != 1
            || p.referenceRatios[3] <= 0 || p.referenceRatios[4] <= 0 || p.surfaceRadiusMm != 0)
            return reject(error, "Mixed feed requires qualified mm/degree ratios");
        break;
    case GtnFeedMetric::RotaryDegrees:
        if (p.mode != cam::ControllerMotionMode::PhysicalAxes || p.surfaceRadiusMm <= 0
            || p.referenceRatios[0] != 0 || p.referenceRatios[1] != 0 || p.referenceRatios[2] != 0
            || !((p.referenceRatios[3] == 1 && p.referenceRatios[4] == 0)
                || (p.referenceRatios[3] == 0 && p.referenceRatios[4] == 1)))
            return reject(error, "Rotary feed requires a qualified radius and one rotary reference axis");
        break;
    default: return reject(error, "GTN feed metric is unqualified");
    }
    return true;
}

class Encoder final : public ExactPlanConsumer {
public:
    const PreparedDeviceProgram& program;
    const GtnLoweringProfile& profile;
    const GtnRtcpTargetValidator& validateRtcp;
    QVector<GtnExactCommand> commands;
    qint64 nextTag{1};
    GtnEncodedDynamics dynamics;
    std::array<double, 5> previousPhysical{}, previousTarget{};
    bool previousValid{false};

    Encoder(const PreparedDeviceProgram& p, const GtnRtcpTargetValidator& validator)
        : program(p), profile(p.recipe().gtnLowering), validateRtcp(validator) {}

    bool begin(const PreparedDeviceProgram&, QString*) override { return true; }

    bool encodeNode(const cam::CamMotionNode& node, std::array<double, 5>& target,
                    std::array<double, 5>& predicted, QString* error)
    {
        if (node.axisMask != 0x1f) return reject(error, "GTN knot physical layout mismatch");
        for (int slot = 0; slot < 5; ++slot) {
            const auto& axis = profile.axes[slot];
            const double value = node.axes[axis.physicalIndex];
            if (!std::isfinite(value) || value < axis.minimum || value > axis.maximum)
                return reject(error, "GTN physical target is non-finite or out of limits");
            predicted[slot] = value;
        }
        target = predicted;
        if (profile.mode == cam::ControllerMotionMode::RTCP) {
            if (!node.referenceTcpValid || !std::isfinite(node.referenceTcpX)
                || !std::isfinite(node.referenceTcpY) || !std::isfinite(node.referenceTcpZ))
                return reject(error, "Selected knot lacks finite table-zero reference TCP");
            target[0] = node.referenceTcpX;
            target[1] = node.referenceTcpY;
            target[2] = node.referenceTcpZ;
            if (!validateRtcp || !validateRtcp(target, predicted, error))
                return reject(error, "GTN RTCP target validation unavailable or failed");
        }
        return true;
    }

    std::uint8_t slotMask(std::uint8_t physicalMask) const
    {
        std::uint8_t result = 0;
        for (int slot = 0; slot < 5; ++slot)
            if (physicalMask & (1u << profile.axes[slot].physicalIndex)) result |= (1u << slot);
        return result;
    }

    bool block(const cam::CamMotionBlock& b, const FrozenToolExecutionRecipe& tool, QString* error) override
    {
        const auto cell = static_cast<unsigned>(b.motionClass);
        if (cell >= profile.cells.size() || !profile.cells[cell].supported
            || !profile.cells[cell].continuousInterpolationQualified
            || !profile.cells[cell].feedMappingQualified
            || (b.activeAxisMask != 0x1f && !profile.cells[cell].inactiveAxisHoldQualified))
            return reject(error, "Unsupported MotionClass/controller-mode cell or continuous semantics");
        const auto interpolation = profile.mode == cam::ControllerMotionMode::PhysicalAxes
            ? cam::MotionInterpolationKind::PhysicalAxisLine : cam::MotionInterpolationKind::RtcpLine;
        if (b.interpolation != interpolation)
            return reject(error, "Selected interpolation cannot be reinterpreted by GTN lowering");
        const auto moving = slotMask(b.activeAxisMask);
        if (profile.metric == GtnFeedMetric::LinearMillimetres
            && profile.mode == cam::ControllerMotionMode::PhysicalAxes && (moving & 0x18))
            return reject(error, "Mixed physical linear/rotary feed requires qualified metric");
        if (profile.metric == GtnFeedMetric::RotaryDegrees
            && (b.motionClass != cam::MotionClass::SingleAxis
                || moving != (profile.referenceRatios[3] == 1 ? 0x08 : 0x10)))
            return reject(error, "Angular surface feed is qualified only for its single rotary axis");
        if (!validateToolExecutionRecipe(tool, true, error)) return false;
        if (b.feed.nominalFeedPerMinute < 0)
            return reject(error, "Invalid published feed");
        if (!b.feed.profileHash.isEmpty() && b.feed.profileHash != program.plan().context.dynamicsSemanticHash)
            return reject(error, "Published feed profile identity mismatch");
        double velocity = b.phase == cam::CamMotionPhase::Rapid
            ? profile.rapidFeedMmPerSecond : tool.lineVelocity;
        if (b.feed.nominalFeedPerMinute > 0) {
            if (b.feed.profileHash.isEmpty()) return reject(error, "Published feed lacks qualified profile identity");
            velocity = b.feed.nominalFeedPerMinute / 60.0;
        }
        const double scale = profile.metric == GtnFeedMetric::RotaryDegrees
            ? 180.0 / (3.14159265358979323846 * profile.surfaceRadiusMm) : 1.0;
        dynamics = {velocity * program.recipe().feedOverride * scale,
            (b.phase == cam::CamMotionPhase::Rapid ? tool.rapidAcceleration : tool.lineAcceleration) * scale,
            (b.phase == cam::CamMotionPhase::Rapid ? tool.rapidJerk : tool.lineJerk) * scale, profile.metric};
        if (!std::isfinite(dynamics.velocity) || dynamics.velocity <= 0
            || !std::isfinite(dynamics.acceleration) || dynamics.acceleration <= 0
            || !std::isfinite(dynamics.jerk) || dynamics.jerk <= 0)
            return reject(error, "Invalid effective GTN feed/dynamics");
        previousValid = b.hasEntryBoundary;
        if (previousValid && !encodeNode(b.entryBoundary, previousTarget, previousPhysical, error)) return false;
        return true;
    }

    bool append(GtnExactCommand command, QString* error)
    {
        if (nextTag > std::numeric_limits<qint32>::max()) return reject(error, "GTN userTag space exhausted");
        command.userTag = static_cast<qint32>(nextTag++);
        commands.append(std::move(command));
        return true;
    }

    bool knot(const cam::CamMotionBlock& b, int index, QString* error) override
    {
        const auto& node = b.physicalKnots[index];
        GtnExactCommand command;
        command.blockId = b.blockId;
        command.contourId = b.contourId;
        command.blockHash = b.blockHash;
        command.knotIndex = index;
        command.sourceEdgeIndex = node.sourceEdgeIndex;
        command.sourceParameter = node.sourceParameter;
        command.departureSourceEdgeIndex = node.departureSourceEdgeIndex;
        command.departureSourceParameter = node.departureSourceParameter;
        command.phase = b.phase;
        command.activeGroupSlotMask = slotMask(b.activeAxisMask);
        command.dynamics = dynamics;
        if (!encodeNode(node, command.target, command.predictedPhysical, error)) return false;
        if (previousValid) {
            bool physicalChange = false;
            double distance = 0;
            for (int slot = 0; slot < 5; ++slot) {
                const double delta = command.predictedPhysical[slot] - previousPhysical[slot];
                if (!(command.activeGroupSlotMask & (1u << slot)) && delta != 0)
                    return reject(error, "Inactive physical axis is not held exactly");
                physicalChange |= delta != 0;
                distance = std::hypot(distance, (command.target[slot] - previousTarget[slot])
                                                   * profile.referenceRatios[slot]);
            }
            if (!std::isfinite(distance) || (physicalChange && distance == 0))
                return reject(error, "Motion has no finite qualified feed reference distance");
        }
        previousTarget = command.target;
        previousPhysical = command.predictedPhysical;
        previousValid = true;
        // No normalization, duplicate suppression, densification or legacy lineTo.
        return append(std::move(command), error);
    }

    bool fence(const cam::CamMotionBlock& b, const cam::MotionProcessFence& f, QString* error) override
    {
        GtnExactCommand command;
        command.kind = GtnExactCommand::Kind::Fence;
        command.blockId = b.blockId;
        command.blockHash = b.blockHash;
        command.contourId = b.contourId;
        command.knotIndex = f.knotIndex;
        command.phase = b.phase;
        command.fence = f; // Structural fences are not implicitly turned into IO.
        return append(std::move(command), error);
    }
    bool seal(QString*) override { return true; }
    void discard() noexcept override { commands.clear(); }
};
} // namespace

QByteArray gtnLoweringProfileHash(const GtnLoweringProfile& p)
{
    QByteArray bytes;
    QDataStream s(&bytes, QIODevice::WriteOnly);
    s.setVersion(QDataStream::Qt_6_0);
    s << QByteArray("gtn-lowering-profile-v2") << quint8(p.state) << quint64(p.revision)
      << p.sourceId << p.groupSemanticsId << quint8(p.mode) << p.absoluteRotaryTurnsQualified;
    for (const auto& axis : p.axes)
        s << qint32(axis.physicalIndex) << axis.name << qint32(axis.role) << qint32(axis.controllerAxis)
          << axis.minimum << axis.maximum;
    for (const auto& cell : p.cells)
        s << cell.supported << cell.continuousInterpolationQualified << cell.inactiveAxisHoldQualified
          << cell.feedMappingQualified;
    s << quint8(p.metric);
    for (double ratio : p.referenceRatios) s << ratio;
    s << p.surfaceRadiusMm << p.rapidFeedMmPerSecond;
    const auto& g = p.group;
    s << g.finiteListAndIoQualified << qint32(g.groupIndex) << qint32(g.listIndex);
    const auto& k = g.kinematics;
    s << qint16(k.modelType) << k.primaryAxisName << k.slaveAxisName << qint16(k.directionMode)
      << k.machineKinematicsFingerprint << k.calibrationFingerprint << k.toolCalibrationFingerprint
      << k.calibrationMachineVerified << k.calibrationConfigurationDerived;
    for (double v : k.primaryAxisPointMcs) s << v;
    for (double v : k.slaveAxisPointMcs) s << v;
    for (double v : k.toolLocationPointMcs) s << v;
    for (int i = 0; i < 5; ++i) {
        s << qint16(k.directions[i]) << qint16(k.physicalAxisIndices[i]);
        for (double v : k.axisVectorsMcs[i]) s << v;
        s << qint16(k.scales[i].count) << k.scales[i].alpha << k.scales[i].beta
          << g.axisVelocity[i] << g.axisAcceleration[i] << g.axisJerk[i] << g.axisDvMax[i];
    }
    s << g.orientationVelocity << g.orientationAcceleration << g.orientationJerk
      << g.pathVelocityLimit << g.pathAccelerationLimit << g.pathJerkLimit
      << g.smoothTimeMs << g.smoothK << qint32(g.lookAheadSegments)
      << g.lookAheadTime << g.lookAheadRadiusRatio << g.startPositionTolerance
      << g.rtcpAxisTolerance << qint32(g.orientationDirection);
    for (const auto& output : g.outputs)
        s << qint32(output.index) << output.expanded << qint32(output.onValue) << qint32(output.offValue);
    return QCryptographicHash::hash(bytes, QCryptographicHash::Sha256);
}

bool validateGtnLoweringProfile(const PreparedDeviceProgram& program, QString* error)
{
    return validateProfile(program, error);
}

std::shared_ptr<const GtnEncodedSection> GtnEncodedSection::lower(
    const PreparedDeviceProgram& program, int ordinal, const GtnRtcpTargetValidator& validateRtcp,
    const std::function<bool()>& cancelled, QString* error)
{
    if (error) error->clear();
    if (!validateProfile(program, error)) return {};
    if (ordinal < 0 || ordinal >= program.sections().size()) {
        reject(error, "Invalid GTN section ordinal");
        return {};
    }
    Encoder encoder(program, validateRtcp);
    const auto& section = program.sections()[ordinal];
    // Tags are unique across the run, even when sections are encoded separately.
    for (int i = 0; i < section.firstBlock; ++i)
        encoder.nextTag += program.plan().blocks[i].physicalKnots.size() + program.plan().blocks[i].fences.size();
    if (!consumeExactSection(program, ordinal, encoder, cancelled, error)) return {};
    auto result = std::shared_ptr<GtnEncodedSection>(new GtnEncodedSection);
    result->m_commands = std::move(encoder.commands);
    result->m_profile = program.recipe().gtnLowering;
    result->m_tool = program.recipe().toolsByContour.value(section.contourId);
    result->m_planHash = program.plan().planHash;
    result->m_contextHash = program.plan().contextHash;
    result->m_recipeRevision = program.recipe().revision;
    result->m_runEpoch = program.runEpoch();
    result->m_ordinal = ordinal;
    QByteArray bytes;
    QDataStream s(&bytes, QIODevice::WriteOnly);
    s.setVersion(QDataStream::Qt_6_0);
    s << QByteArray("gtn-exact-section-v1") << result->m_planHash << result->m_contextHash
      << result->m_recipeRevision << quint64(result->m_runEpoch) << qint32(ordinal)
      << gtnLoweringProfileHash(result->m_profile) << quint64(result->m_commands.size());
    for (const auto& c : result->m_commands) {
        s << qint32(c.kind) << c.userTag << quint64(c.blockId) << quint64(c.contourId) << c.blockHash
          << qint32(c.knotIndex) << qint32(c.sourceEdgeIndex) << c.sourceParameter
          << qint32(c.departureSourceEdgeIndex) << c.departureSourceParameter
          << quint8(c.phase) << quint8(c.activeGroupSlotMask);
        for (double value : c.target) s << value;
        for (double value : c.predictedPhysical) s << value;
        s << c.dynamics.velocity << c.dynamics.acceleration << c.dynamics.jerk << quint8(c.dynamics.metric)
          << qint32(c.fence.knotIndex) << c.fence.blockStart << c.fence.blockEnd
          << c.fence.laserEnabledAfterFence << c.fence.changesLaserState << c.fence.requiredStop;
    }
    result->m_encodingHash = QCryptographicHash::hash(bytes, QCryptographicHash::Sha256);
    return result;
}

std::shared_ptr<const GtnEncodedProgram> GtnEncodedProgram::lower(
    const PreparedDeviceProgram& program, const GtnRtcpTargetValidator& validator,
    const std::function<bool()>& cancelled, QString* error)
{
    auto result = std::shared_ptr<GtnEncodedProgram>(new GtnEncodedProgram);
    QByteArray bytes;
    QDataStream stream(&bytes, QIODevice::WriteOnly);
    stream.setVersion(QDataStream::Qt_6_0);
    stream << QByteArray("gtn-exact-program-v1") << quint64(program.sections().size());
    for (int i = 0; i < program.sections().size(); ++i) {
        auto section = GtnEncodedSection::lower(program, i, validator, cancelled, error);
        if (!section) return {}; // no partially encoded run escapes
        stream << section->encodingHash();
        result->m_sections.append(std::move(section));
    }
    result->m_encodingHash = QCryptographicHash::hash(bytes, QCryptographicHash::Sha256);
    return result;
}
} // namespace lcnc::process
