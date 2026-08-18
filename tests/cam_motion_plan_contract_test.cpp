#include "core/project/cam/collision_validation_contracts.h"

#include <QCoreApplication>
#include <QTextStream>

namespace {

int fail(const QString& message)
{
    QTextStream(stderr) << message << '\n';
    return 1;
}

} // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    lcnc::cam::CollisionValidationSnapshot validation;
    validation.state = lcnc::cam::CollisionValidationState::Disabled;
    validation.complete = true;
    if (validation.blocksExecution())
        return fail(QStringLiteral("Disabled collision detection must not block execution"));

    validation.state = lcnc::cam::CollisionValidationState::Warning;
    if (!validation.blocksExecution(true) || validation.blocksExecution(false))
        return fail(QStringLiteral("Clearance warning policy is not configurable"));

    validation.state = lcnc::cam::CollisionValidationState::Collision;
    if (!validation.blocksExecution(false))
        return fail(QStringLiteral("Confirmed collision must always block execution"));

    lcnc::cam::CamMotionPlanSnapshot plan;
    plan.revision = 42;
    lcnc::cam::CamMotionNode rapid;
    rapid.phase = lcnc::cam::CamMotionPhase::Rapid;
    rapid.rapidPhase = lcnc::cam::RapidSegmentPhase::Retract;
    rapid.contourId = 7;
    rapid.axisMask = 0x1f;
    plan.nodes.append(rapid);
    plan.collision = validation;
    if (plan.nodes.size() != 1 || plan.nodes.constFirst().contourId != 7
        || plan.nodes.constFirst().phase != lcnc::cam::CamMotionPhase::Rapid)
        return fail(QStringLiteral("Motion node contract did not preserve final CAM data"));
    if (plan.nodes.constFirst().rapidPhase != lcnc::cam::RapidSegmentPhase::Retract)
        return fail(QStringLiteral("Motion node contract lost the CAM rapid segment phase"));
    return 0;
}
