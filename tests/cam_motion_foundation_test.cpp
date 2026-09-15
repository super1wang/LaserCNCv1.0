#include "core/algorithms/cam/collision_policy.h"
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

    if (lcnc::cam_algo::automaticCollisionWorkEnabled(
            lcnc::cam::CollisionVerificationMode::Disabled)
        || !lcnc::cam_algo::automaticCollisionWorkEnabled(
            lcnc::cam::CollisionVerificationMode::Optional)
        || !lcnc::cam_algo::automaticCollisionWorkEnabled(
            lcnc::cam::CollisionVerificationMode::Required)) {
        return fail(QStringLiteral(
            "Automatic collision-work policy does not isolate Disabled mode"));
    }

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

    toml::value legacyDisabled(toml::table{});
    toml::value legacyDisabledProfile(toml::table{});
    legacyDisabledProfile["path"] = std::string("C:/machines/legacy-disabled.step");
    legacyDisabledProfile["collisionDetectionEnabled"] = false;
    legacyDisabled["machineProfile"] = toml::array{legacyDisabledProfile};
    config.readFrom(legacyDisabled);
    if (config.collisionVerificationModeForMachine(
            QStringLiteral("C:/machines/legacy-disabled.step"))
        != lcnc::cam::CollisionVerificationMode::Disabled) {
        return fail(QStringLiteral("Legacy collision enabled=false did not migrate to Disabled"));
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

    const auto verifyExplicitMode = [&config](const char* path, const char* value,
                                               lcnc::cam::CollisionVerificationMode expected) {
        toml::value root(toml::table{});
        toml::value profile(toml::table{});
        profile["path"] = std::string(path);
        profile["collisionVerificationMode"] = std::string(value);
        root["machineProfile"] = toml::array{profile};
        config.readFrom(root);
        const QString machinePath = QString::fromUtf8(path);
        return config.collisionVerificationModeValidForMachine(machinePath)
            && config.collisionVerificationModeForMachine(machinePath) == expected;
    };
    if (!verifyExplicitMode("C:/machines/disabled.step", "disabled",
                            lcnc::cam::CollisionVerificationMode::Disabled)
        || !verifyExplicitMode("C:/machines/required.step", "required",
                               lcnc::cam::CollisionVerificationMode::Required)) {
        return fail(QStringLiteral("Explicit collision verification mode parsing failed"));
    }

    toml::value invalidRoot(toml::table{});
    toml::value invalidProfile(toml::table{});
    invalidProfile["path"] = std::string("C:/machines/invalid.step");
    invalidProfile["collisionVerificationMode"] = std::string("requried");
    invalidProfile["collisionDetectionEnabled"] = false;
    invalidRoot["machineProfile"] = toml::array{invalidProfile};
    config.readFrom(invalidRoot);
    const QString invalidPath = QStringLiteral("C:/machines/invalid.step");
    if (config.collisionVerificationModeValidForMachine(invalidPath)
        || config.collisionVerificationModeForMachine(invalidPath)
            != lcnc::cam::CollisionVerificationMode::Required) {
        return fail(QStringLiteral("Invalid explicit collision mode did not fail closed"));
    }
    toml::value invalidRoundTrip(toml::table{});
    config.writeTo(invalidRoundTrip);
    if (invalidRoundTrip.at("machineProfile").as_array().front()
            .at("collisionVerificationMode").as_string()
        != std::string("requried")) {
        return fail(QStringLiteral("Invalid explicit collision mode was silently healed"));
    }

    toml::value emptyRoot(toml::table{});
    toml::value emptyProfile(toml::table{});
    emptyProfile["path"] = std::string("C:/machines/empty.step");
    emptyProfile["collisionVerificationMode"] = std::string();
    emptyRoot["machineProfile"] = toml::array{emptyProfile};
    config.readFrom(emptyRoot);
    if (config.collisionVerificationModeValidForMachine(
            QStringLiteral("C:/machines/empty.step"))) {
        return fail(QStringLiteral("Explicit empty collision mode was accepted"));
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
