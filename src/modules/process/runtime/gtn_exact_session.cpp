#include "modules/process/runtime/gtn_exact_session.h"

#include "core/logging/logger.h"
#include <algorithm>
#include <cmath>
#include <QSet>

namespace lcnc::process {
QString gtnParameterAudit(const QString& parameter, double requested, double effective,
    const QString& unit, const GtnLoweringProfile& profile)
{
    return QStringLiteral("gtn.parameter name=%1 requested=%2 effective=%3 unit=%4 source=%5 revision=%6")
        .arg(parameter).arg(requested, 0, 'g', 17).arg(effective, 0, 'g', 17)
        .arg(unit, profile.sourceId).arg(qulonglong(profile.revision));
}

bool validateGtnGroupProfile(const PreparedDeviceProgram& program, QString* error)
{
    const auto fail = [&](const char* why) { if (error) *error = QString::fromLatin1(why); return false; };
    if (!validateGtnLoweringProfile(program, error)) return false;
    const auto& p = program.recipe().gtnLowering;
    const auto& g = p.group;
    const auto positive = [](double v) { return std::isfinite(v) && v > 0; };
    if (!g.finiteListAndIoQualified || g.groupIndex < 1 || g.groupIndex > 2
        || g.listIndex < 1 || g.listIndex > 4 || g.lookAheadSegments <= 0
        || !positive(g.lookAheadTime) || !positive(g.lookAheadRadiusRatio)
        || !positive(g.smoothTimeMs) || !positive(g.smoothK)
        || !positive(g.pathVelocityLimit) || !positive(g.pathAccelerationLimit) || !positive(g.pathJerkLimit)
        || !positive(g.orientationVelocity) || !positive(g.orientationAcceleration) || !positive(g.orientationJerk)
        || !positive(g.startPositionTolerance) || !positive(g.rtcpAxisTolerance)
        || (g.orientationDirection != 0 && g.orientationDirection != 1))
        return fail("GTN finite-list Group/IO/profile qualification is invalid");
    const auto& k = g.kinematics;
    if (k.modelType < 0 || k.directionMode != 1 || k.machineKinematicsFingerprint.isEmpty()
        || (p.mode == cam::ControllerMotionMode::RTCP && (!k.calibrationMachineVerified || k.calibrationFingerprint.isEmpty())))
        return fail("GTN frozen kinematics/calibration unavailable");
    for (int i = 0; i < 5; ++i) {
        if (k.physicalAxisIndices[i] != p.axes[i].controllerAxis || k.scales[i].count != 1
            || k.scales[i].alpha != 1 || !positive(k.scales[i].beta)
            || !positive(g.axisVelocity[i]) || !positive(g.axisAcceleration[i]) || !positive(g.axisJerk[i]) || !positive(g.axisDvMax[i]))
            return fail("GTN frozen axis profile is invalid");
        double norm = 0;
        for (double v : k.axisVectorsMcs[i]) { if (!std::isfinite(v)) return fail("Non-finite GTN kinematics"); norm += v * v; }
        if (!std::isfinite(norm) || norm <= 0) return fail("GTN axis vector unavailable");
    }
    for (const auto& point : {k.primaryAxisPointMcs, k.slaveAxisPointMcs, k.toolLocationPointMcs})
        for (double v : point) if (!std::isfinite(v)) return fail("Non-finite GTN model point");
    QSet<int> io;
    for (const auto& out : g.outputs) {
        if (out.index == 0) continue;
        const int key = out.index + (out.expanded ? 100 : 0);
        if (out.index < 1 || out.index > (out.expanded ? 12 : 16) || io.contains(key)
            || (out.onValue != 0 && out.onValue != 1) || out.offValue != 1 - out.onValue)
            return fail("Invalid or aliased GTN frozen IO mapping");
        io.insert(key);
    }
    for (const auto& tool : program.recipe().toolsByContour) {
        if (!g.outputs[0].index || !g.outputs[tool.blow2 ? 2 : 1].index)
            return fail("GTN laser/selected gas output missing");
        // Qualified first implementation supports line dynamics and timed
        // digital gates. Non-neutral unmapped device features are rejected.
        if (tool.junctionVelocity != 0 || tool.junctionAngle != 0 || tool.endVelocity != 0
            || tool.cutSmoothTime != 0 || tool.cutSmoothK != 0 || tool.axisSmoothTime != 0 || tool.axisSmoothK != 0
            || tool.laserEnergy != 0 || tool.laserFrequency != 0 || tool.laserPulseWidth != 0
            || tool.attenuatorPercentage != 0 || tool.pulseDivider != 0 || tool.laserDelay != 0
            || tool.analogValue != 0 || tool.analogLaserValue != 0 || tool.powerSetpoint != 0
            || tool.repetitionRate != 0 || tool.pulsePickerDivider != 0 || tool.energySwitch
            || tool.rapidXVelocity != 0 || tool.rapidYVelocity != 0 || tool.rapidZVelocity != 0
            || tool.rapidAVelocity != 0 || tool.rapidA1Velocity != 0 || tool.rapidCVelocity != 0
            || tool.rapidX1Velocity != 0 || tool.rapidY1Velocity != 0)
            return fail("GTN exact recipe contains an unsupported non-neutral device parameter");
        for (double delay : {tool.beforeOn, tool.afterOn, tool.beforeOff, tool.afterOff, tool.blowDelay})
            if (!std::isfinite(delay * 1000.0)) return fail("GTN delay conversion overflow");
    }
    return true;
}
bool GtnExactSession::matches(const PreparedDeviceProgram& program, int ordinal) const
{
    if (!m_program || ordinal != m_ordinal || ordinal < 0 || ordinal >= m_program->sections().size()) return false;
    const auto& frozen = *m_program->sections()[ordinal];
    return frozen.planHash() == program.plan().planHash && frozen.contextHash() == program.plan().contextHash
        && frozen.recipeRevision() == program.recipe().revision && frozen.runEpoch() == program.runEpoch();
}

bool GtnExactSession::fail(const QString& reason, QString* error)
{
    m_state = GtnSessionState::Faulted; // retain no-replay state even if release succeeds
    QString releaseError;
    if (m_ownsGroup && !m_cleanupAttempted) {
        m_cleanupAttempted = true;
        if (m_backend.stopRelease(true, &releaseError)) m_ownsGroup = false;
    }
    if (error) *error = (reason.isEmpty() ? QStringLiteral("GTN exact backend operation failed") : reason)
        + (releaseError.isEmpty() ? QString{} : "; " + releaseError);
    return false;
}

bool GtnExactSession::prepare(const PreparedDeviceProgram& program, int ordinal,
    const std::function<bool()>& cancelled, QString* error)
{
    if (error) error->clear();
    if (m_state != GtnSessionState::Idle && m_state != GtnSessionState::Complete)
        return fail(QStringLiteral("GTN section preparation replay/state mismatch"), error);
    if (ordinal != m_ordinal || ordinal < 0 || ordinal >= program.sections().size()
        || (m_program && !matches(program, ordinal)) || (cancelled && cancelled()))
        return fail(QStringLiteral("GTN run identity/order changed or stopped"), error);
    if (!validateGtnGroupProfile(program, error) || !m_backend.admit(program, ordinal, error)) {
        m_state = GtnSessionState::Faulted;
        return false;
    }
    // Acquire can partially assign axes before an SDK failure. Own before call.
    m_ownsGroup = true;
    m_cleanupAttempted = false;
    QString reason;
    if (!m_backend.acquire(program, &reason)) return fail(reason, error);
    if (!m_program) {
        m_program = GtnEncodedProgram::lower(program,
            [&](const auto& target, const auto& physical, QString* e) {
                return m_backend.validateRtcp(target, physical, e);
            }, cancelled, &reason);
        if (!m_program) return fail(reason, error);
        const auto& g = program.recipe().gtnLowering.group;
        for (const auto& section : m_program->sections())
            for (const auto& command : section->commands())
                if (command.kind == GtnExactCommand::Kind::LinearAbsolute
                    && (command.dynamics.velocity > g.pathVelocityLimit
                        || command.dynamics.acceleration > g.pathAccelerationLimit
                        || command.dynamics.jerk > g.pathJerkLimit))
                    return fail(QStringLiteral("GTN effective dynamics exceed the frozen qualified profile"), error);
    } else if (program.recipe().gtnLowering.mode == cam::ControllerMotionMode::RTCP) {
        // Reacquired Group must validate the selected section again.
        auto checked = GtnEncodedSection::lower(program, ordinal,
            [&](const auto& target, const auto& physical, QString* e) {
                return m_backend.validateRtcp(target, physical, e);
            }, cancelled, &reason);
        if (!checked || checked->encodingHash() != m_program->sections()[ordinal]->encodingHash())
            return fail(QStringLiteral("GTN reacquired RTCP section validation failed: ") + reason, error);
    }
    if (cancelled && cancelled()) return fail(QStringLiteral("GTN preparation stopped"), error);
    m_cursor = 0;
    m_sealAttempts = 0;
    m_state = GtnSessionState::Filling;
    return true;
}

bool GtnExactSession::fill(const PreparedDeviceProgram& program, int ordinal, bool& complete,
    const std::function<bool()>& cancelled, QString* error)
{
    complete = false;
    if (error) error->clear();
    if (!matches(program, ordinal) || (m_state != GtnSessionState::Filling && m_state != GtnSessionState::Sealing))
        return fail(QStringLiteral("GTN finite-list fill identity/state mismatch"), error);
    QString reason;
    const auto& section = *m_program->sections()[ordinal];
    // Return to DeviceCommandQueue after at most 32 semantic commands. IO
    // expansion is bounded by the adapter; Stop can run before the next slice.
    const int end = std::min(m_cursor + 32, int(section.commands().size()));
    while (m_cursor < end) {
        if (cancelled && cancelled()) return fail(QStringLiteral("GTN fill stopped"), error);
        if (!m_backend.append(section, section.commands()[m_cursor], &reason)) return fail(reason, error);
        ++m_cursor;
    }
    if (m_cursor != section.commands().size()) return true;
    m_state = GtnSessionState::Sealing;
    if (cancelled && cancelled()) return fail(QStringLiteral("GTN seal stopped"), error);
    const auto result = m_backend.seal(&reason);
    if (result == GtnSealResult::Failed) return fail(reason, error);
    if (result == GtnSealResult::Pending) {
        if (++m_sealAttempts >= 256) return fail(QStringLiteral("GTN DataEnd did not seal within the bounded attempts"), error);
        return true;
    }
    if (cancelled && cancelled()) return fail(QStringLiteral("GTN stop after DataEnd"), error);
    m_state = GtnSessionState::Sealed;
    complete = true;
    return true;
}

bool GtnExactSession::start(const PreparedDeviceProgram& program, int ordinal,
    const std::function<bool()>& cancelled, QString* error)
{
    if (error) error->clear();
    if (!matches(program, ordinal) || m_state != GtnSessionState::Sealed)
        return fail(QStringLiteral("GTN Start requires the same sealed finite section; replay forbidden"), error);
    QString reason;
    if ((cancelled && cancelled()) || !m_backend.admit(program, ordinal, &reason))
        return fail(QStringLiteral("GTN final admission failed/stopped: ") + reason, error);
    if (cancelled && cancelled()) return fail(QStringLiteral("GTN Start stopped"), error);
    m_state = GtnSessionState::Faulted; // before the non-idempotent call
    if (!m_backend.start(&reason))
        return fail(QStringLiteral("GTN Start outcome indeterminate; no replay: ") + reason, error);
    if (cancelled && cancelled()) return fail(QStringLiteral("GTN Stop arrived during Start"), error);
    m_state = GtnSessionState::Running;
    return true;
}

bool GtnExactSession::poll(const PreparedDeviceProgram& program, int ordinal, bool& running,
    const std::function<bool()>& cancelled, QString* error)
{
    running = false;
    if (error) error->clear();
    if (!matches(program, ordinal) || m_state != GtnSessionState::Running)
        return fail(QStringLiteral("GTN polling identity/state mismatch"), error);
    if (cancelled && cancelled()) return fail(QStringLiteral("GTN running section stopped"), error);
    QString reason;
    if (!m_backend.poll(running, &reason)) return fail(reason, error);
    if (running) return true;
    m_cleanupAttempted = true;
    if (!m_backend.stopRelease(false, &reason)) return fail(reason, error);
    m_ownsGroup = false;
    m_state = GtnSessionState::Complete;
    ++m_ordinal; // next section only after drained, safe outputs, confirmed release
    return true;
}

bool GtnExactSession::abort(QString* error)
{
    m_state = GtnSessionState::Faulted;
    if (!m_ownsGroup) return true;
    if (m_cleanupAttempted) {
        if (error) *error = QStringLiteral("GTN ownership retained after failed cleanup; explicit controller recovery required");
        return false;
    }
    m_cleanupAttempted = true;
    const bool released = m_backend.stopRelease(true, error);
    if (released) m_ownsGroup = false;
    return released;
}
} // namespace lcnc::process
