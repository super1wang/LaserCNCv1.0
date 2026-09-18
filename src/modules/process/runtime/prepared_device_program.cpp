#include "modules/process/runtime/prepared_device_program.h"

#include <QCryptographicHash>
#include <QDataStream>
#include <QIODevice>
#include <QSet>
#include <cmath>
#include <algorithm>

namespace lcnc::process {
namespace {
// TOML tables are unordered; encode recursively with sorted keys.
void encode(QDataStream& stream, const toml::value& value)
{
    stream << static_cast<qint32>(value.type());
    if (value.is_table()) {
        QStringList keys;
        for (const auto& item : value.as_table()) keys.append(QString::fromStdString(item.first));
        keys.sort();
        stream << keys;
        for (const auto& key : keys) encode(stream, value.at(key.toStdString()));
    } else if (value.is_array()) {
        stream << quint64(value.as_array().size());
        for (const auto& item : value.as_array()) encode(stream, item);
    } else {
        stream << QByteArray::fromStdString(toml::format(value));
    }
}
bool finiteValues(const toml::value& value)
{
    if (value.is_floating()) return std::isfinite(value.as_floating());
    if (value.is_table()) {
        for (const auto& item : value.as_table()) if (!finiteValues(item.second)) return false;
    } else if (value.is_array()) {
        for (const auto& item : value.as_array()) if (!finiteValues(item)) return false;
    }
    return true;
}
}

QByteArray deviceRunRecipeHash(const DeviceRunRecipe& recipe)
{
    QByteArray bytes;
    QDataStream stream(&bytes, QIODevice::WriteOnly);
    stream.setVersion(QDataStream::Qt_6_0);
    stream << QByteArray("device-run-recipe-v2") << recipe.planHash << recipe.contextHash
           << recipe.sourceId << recipe.feedOverride;
    encode(stream, recipe.processIoProfile);
    auto keys = recipe.toolsByContour.keys();
    std::sort(keys.begin(), keys.end());
    stream << quint64(keys.size());
    for (auto id : keys) {
        stream << quint64(id);
        stream << frozenToolExecutionRecipeHash(recipe.toolsByContour.value(id));
    }
    return QCryptographicHash::hash(bytes, QCryptographicHash::Sha256);
}

std::shared_ptr<const PreparedDeviceProgram> PreparedDeviceProgram::prepare(
    const cam::ToolpathExportSnapshot& snapshot, const DeviceRunRecipe& recipe,
    std::uint64_t runEpoch, bool realMachine, QString* error)
{
    if (error) error->clear();
    const auto reject = [&](const char* why) -> std::shared_ptr<const PreparedDeviceProgram> {
        if (error) *error = QString::fromLatin1(why);
        return {};
    };
    const auto& plan = snapshot.motionPlan;
    if (!runEpoch || plan.blocks.isEmpty() || !cam::finalMotionPlanIdentityIsCurrent(plan)
        || plan.revision != snapshot.revision || !plan.failureReason.isEmpty())
        return reject("Missing or stale FinalMotionPlan; legacy points are not executable");
    auto checked = plan;
    QString validation;
    if (!cam::finalizeMotionPlan(&checked, &validation)
        || checked.planHash != plan.planHash || !snapshot.machineAxisLayout.isValid())
        return reject("Invalid FinalMotionPlan or physical axis layout");
    if (recipe.planHash != plan.planHash || recipe.contextHash != plan.contextHash
        || recipe.sourceId.isEmpty() || recipe.revision.isEmpty()
        || recipe.revision != deviceRunRecipeHash(recipe)
        || !std::isfinite(recipe.feedOverride) || recipe.feedOverride <= 0
        || !recipe.processIoProfile.is_table() || recipe.processIoProfile.as_table().empty()
        || !finiteValues(recipe.processIoProfile))
        return reject("Missing, invalid or stale frozen run recipe");
    if (recipe.processIoProfile.contains("Setting")
        && recipe.processIoProfile.at("Setting").is_table()
        && recipe.processIoProfile.at("Setting").contains("Tool"))
        return reject("Legacy Tool settings cannot bypass the frozen execution recipe");
    const auto layoutMask = (1u << snapshot.machineAxisLayout.count) - 1u;
    for (const auto& block : plan.blocks) {
        if (block.optimizationState == cam::MotionOptimizationState::Raw)
            return reject("Raw motion is not a FinalMotionPlan execution fallback");
        for (const auto& knot : block.physicalKnots)
            if (knot.axisMask != layoutMask) return reject("Physical layout mismatch");
        const auto tool = recipe.toolsByContour.constFind(block.contourId);
        if (tool == recipe.toolsByContour.cend())
            return reject("Missing or invalid contour recipe; default substitution is forbidden");
    }
    for (auto tool = recipe.toolsByContour.cbegin(); tool != recipe.toolsByContour.cend(); ++tool)
        if (!validateToolExecutionRecipe(tool.value(), realMachine, error)) return {};
    // The policy belongs to the hashed compilation context, not mutable UI or
    // a late safety attachment. Required proof must bind this exact plan.
    const auto mode = plan.context.collisionMode;
    if (mode == cam::CollisionVerificationMode::Required) {
        if (snapshot.travelPlan.verifiedMotionPlanHash != plan.planHash
            || !plan.collision.complete
            || plan.collision.blocksExecution(plan.collision.blockWarning)
            || plan.collision.state == cam::CollisionValidationState::Disabled
            || plan.edgeCertificates.size() != plan.nodes.size() - 1)
            return reject("Required collision proof is missing, stale or unsafe");
        for (int i = 0; i < plan.edgeCertificates.size(); ++i) {
            const auto& cert = plan.edgeCertificates[i];
            if (cert.state != cam::CamMotionCertificateState::CertifiedSafe
                || cert.firstNode != i || cert.lastNode != i + 1)
                return reject("Required collision edge proof is incomplete");
        }
        const auto& safety = snapshot.collisionSafety;
        if (realMachine && ((safety.machinePackageRequired && (!safety.machinePackageReady || safety.packageBuildInProgress))
            || (safety.jobOverlayRequired && (!safety.jobOverlayReady || safety.jobOverlayBuildInProgress))))
            return reject("Required collision safety assets are unavailable");
    } else if (mode != cam::CollisionVerificationMode::Disabled && mode != cam::CollisionVerificationMode::Optional) {
        return reject("Unknown committed collision policy");
    }
    if (realMachine && (!cam::controllerQualificationIsQualified(plan.context.controllerQualification)
        || plan.context.controllerCapabilityHash != cam::controllerQualificationSnapshotHash(plan.context.controllerQualification)
        || plan.context.controllerMode != plan.context.controllerQualification.requestedMode))
        return reject("Controller semantics are unqualified; real execution is unavailable");
    auto program = std::shared_ptr<PreparedDeviceProgram>(new PreparedDeviceProgram);
    program->m_plan = plan;
    program->m_layout = snapshot.machineAxisLayout;
    program->m_recipe = recipe;
    program->m_runEpoch = runEpoch;
    program->m_realMachine = realMachine;
    QSet<std::uint64_t> seenContours;
    bool laserEnabled = false; // device admission establishes safe outputs
    for (int i = 0; i < plan.blocks.size(); ++i) {
        const auto& block = plan.blocks[i];
        if (program->m_sections.isEmpty() || program->m_sections.back().contourId != block.contourId) {
            if (laserEnabled || seenContours.contains(block.contourId))
                return reject("Contour boundary is not pause-safe or contour order is non-contiguous");
            seenContours.insert(block.contourId);
            program->m_sections.append({int(program->m_sections.size()), block.contourId,
                i, i, block.blockId, block.blockId, block.blockHash, block.blockHash, plan.planHash, runEpoch});
        }
        auto& section = program->m_sections.back();
        section.lastBlock = i;
        section.lastBlockId = block.blockId;
        section.lastBlockHash = block.blockHash;
        for (const auto& fence : block.fences)
            if (fence.changesLaserState) laserEnabled = fence.laserEnabledAfterFence;
    }
    if (laserEnabled) return reject("Final execution section leaves laser enabled");
    return program;
}

static bool consumeBlocks(const PreparedDeviceProgram& program, int firstBlock, int lastBlock,
    ExactPlanConsumer& consumer, const std::function<bool()>& cancelled, QString* error)
{
    bool sealed = false;
    struct Rollback {
        ExactPlanConsumer& consumer;
        bool& sealed;
        ~Rollback() { if (!sealed) consumer.discard(); }
    } rollback{consumer, sealed};
    const auto stopped = [&] {
        if (!cancelled || !cancelled()) return false;
        if (error) *error = QStringLiteral("Exact-plan preparation cancelled");
        return true;
    };
    if (stopped() || !consumer.begin(program, error)) return false;
    for (int blockIndex = firstBlock; blockIndex <= lastBlock; ++blockIndex) {
        const auto& block = program.plan().blocks[blockIndex];
        if (stopped() || !consumer.block(block, program.recipe().toolsByContour[block.contourId], error)) return false;
        const auto fences = [&](int index) {
            for (const auto& fence : block.fences)
                if (fence.knotIndex == index && (stopped() || !consumer.fence(block, fence, error))) return false;
            return true;
        };
        if (!fences(-1)) return false;
        for (int i = 0; i < block.physicalKnots.size(); ++i) {
            if (stopped() || !consumer.knot(block, i, error) || !fences(i)) return false;
        }
    }
    if (stopped() || !consumer.seal(error)) return false;
    sealed = true;
    return true;
}

bool consumeExactPlan(const PreparedDeviceProgram& program, ExactPlanConsumer& consumer,
                      const std::function<bool()>& cancelled, QString* error)
{
    return consumeBlocks(program, 0, int(program.plan().blocks.size()) - 1, consumer, cancelled, error);
}

bool consumeExactSection(const PreparedDeviceProgram& program, int ordinal,
    ExactPlanConsumer& consumer, const std::function<bool()>& cancelled, QString* error)
{
    if (ordinal < 0 || ordinal >= program.sections().size()) {
        if (error) *error = QStringLiteral("Invalid exact-section ordinal");
        consumer.discard();
        return false;
    }
    const auto& section = program.sections()[ordinal];
    return consumeBlocks(program, section.firstBlock, section.lastBlock, consumer, cancelled, error);
}
} // namespace lcnc::process
