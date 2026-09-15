#include "modules/cam/collision/continuous_motion_certificate_builder.h"
#include "modules/cam/contracts/toolpath_export_dto.h"
#include "modules/cam/settings/cam_config.h"

#include <QCoreApplication>
#include <QTextStream>

namespace {

int fail(const QString& message)
{
    QTextStream(stderr) << message << '\n';
    return 1;
}

class TestCamConfig final : public CamConfig
{
public:
    using CamConfig::readFrom;
    using CamConfig::writeTo;
};

} // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    TestCamConfig config;
    toml::value legacy(toml::table{});
    toml::value legacyProfile(toml::table{});
    legacyProfile["path"] = std::string("C:/machines/legacy.step");
    legacyProfile["collisionDetectionEnabled"] = true;
    legacy["machineProfile"] = toml::array{legacyProfile};
    config.readFrom(legacy);
    if (config.collisionVerificationModeForMachine(
            QStringLiteral("C:/machines/legacy.step"))
        != lcnc::cam::CollisionVerificationMode::Required) {
        return fail(QStringLiteral("Legacy collision enabled=true did not migrate to Required"));
    }
    toml::value migrated(toml::table{});
    config.writeTo(migrated);
    const auto& migratedProfile = migrated.at("machineProfile").as_array().front();
    if (!migratedProfile.contains("collisionVerificationMode")
        || migratedProfile.at("collisionVerificationMode").as_string()
            != std::string("required")
        || migratedProfile.contains("collisionDetectionEnabled")) {
        return fail(QStringLiteral("Collision mode did not serialize using the v3 policy field"));
    }

    toml::value optionalRoot(toml::table{});
    toml::value optionalProfile(toml::table{});
    optionalProfile["path"] = std::string("C:/machines/optional.step");
    optionalProfile["collisionVerificationMode"] = std::string("optional");
    optionalRoot["machineProfile"] = toml::array{optionalProfile};
    config.readFrom(optionalRoot);
    if (config.collisionVerificationModeForMachine(
            QStringLiteral("C:/machines/optional.step"))
        != lcnc::cam::CollisionVerificationMode::Optional) {
        return fail(QStringLiteral("Optional collision verification mode did not round-trip"));
    }

    lcnc::cam::ToolpathExportSnapshot snapshot;
    snapshot.revision = 7;
    snapshot.motionPlan.revision = 7;
    snapshot.travelPlan.key.environmentRevision = 11;
    snapshot.collisionSafety.verificationMode =
        lcnc::cam::CollisionVerificationMode::Disabled;
    snapshot.collisionSafety.enabled = false;
    lcnc::cam::CamMotionNode first;
    first.contourId = 1;
    first.axisMask = 0x1f;
    lcnc::cam::CamMotionNode last = first;
    last.axes[0] = 10.0;
    snapshot.motionPlan.nodes = {first, last};
    int progressCalls = 0;
    const auto certificates = lcnc::cam::buildContinuousMotionCertificates(
        snapshot, {}, {}, [&progressCalls](int complete, int total) {
            if (complete == total)
                ++progressCalls;
        });
    if (certificates.size() != 1
        || certificates.constFirst().state
            != lcnc::cam::CamMotionCertificateState::Disabled
        || certificates.constFirst().intervalQueries != 0
        || certificates.constFirst().surfaceBvhQueries != 0
        || certificates.constFirst().coalPairQueries != 0
        || certificates.constFirst().occtExactQueries != 0
        || progressCalls != 1) {
        return fail(QStringLiteral(
            "Disabled collision mode constructed or queried a collision backend"));
    }
    return 0;
}
