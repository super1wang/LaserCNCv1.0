#include "core/algorithms/kinematics/axis_line_calibration_solver.h"
#include "core/kinematics/machine_calibration_service.h"
#include "core/kinematics/controller_kinematics_snapshot.h"
#include "core/kinematics/machine_configuration_service.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>

#include <gp_Ax1.hxx>

#include <cmath>
#include <iostream>

namespace {

bool close(double lhs, double rhs, double tolerance)
{
    return std::abs(lhs - rhs) <= tolerance;
}

QVector<lcnc::kinematics::CalibrationSample> circleSamples(
    const QString& axis, int slot, double noise)
{
    QVector<lcnc::kinematics::CalibrationSample> samples;
    const std::array<double, 7> angles{-150.0, -100.0, -50.0, 0.0, 50.0, 100.0, 150.0};
    for (int index = 0; index < static_cast<int>(angles.size()); ++index) {
        const double radians = angles[index] * 3.14159265358979323846 / 180.0;
        lcnc::kinematics::CalibrationSample sample;
        sample.sampleId = QStringLiteral("%1-%2").arg(axis).arg(index);
        sample.targetAxisName = axis;
        sample.targetAxisSlot = slot;
        sample.actualAxes[slot] = angles[index];
        sample.measuredReferencePointMcs = {
            10.0 + 50.0 * std::cos(radians),
            20.0 + 50.0 * std::sin(radians),
            30.0 + ((index % 2) ? noise : -noise)};
        sample.measurementSource = QStringLiteral("test");
        sample.timestampUtc = QStringLiteral("2026-09-01T00:00:00Z");
        samples.append(sample);
    }
    return samples;
}

int fail(const char* message)
{
    std::cerr << message << '\n';
    return 1;
}

} // namespace

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);
    using namespace lcnc::kinematics;

    const QVector<CalibrationSample> samples = circleSamples(QStringLiteral("C"), 4, 0.01);
    AxisLineCalibrationOptions options;
    options.maximumRmsResidualMm = 0.05;
    const AxisLineCalibrationResult fitted = fitRotaryAxisLine(
        samples, QStringLiteral("C"), options);
    if (!fitted.success)
        return fail(fitted.error.toStdString().c_str());
    if (!close(fitted.fit.pointMcs[0], 10.0, 0.02)
        || !close(fitted.fit.pointMcs[1], 20.0, 0.02)
        || !close(fitted.fit.pointMcs[2], 30.0, 0.02)
        || !close(fitted.fit.fittedRadiusMm, 50.0, 0.02)
        || fitted.fit.angularCoverageDeg < 250.0
        || fitted.fit.unitVectorMcs[2] < 0.99) {
        return fail("fitted axis does not match the synthetic circle");
    }

    AxisLineCalibrationOptions insufficientCoverage = options;
    if (fitted.fit.rmsResidualMm < 0.005)
        return fail("out-of-plane noise was omitted from the reported 3D residual");
    AxisLineCalibrationOptions spatialLimits = options;
    spatialLimits.maximumRmsResidualMm = 0.20;
    spatialLimits.maximumResidualMm = 0.50;
    if (fitRotaryAxisLine(circleSamples(QStringLiteral("C"), 4, 1.0),
                         QStringLiteral("C"), spatialLimits).success)
        return fail("one millimetre out-of-plane error was accepted");
    insufficientCoverage.minimumAngularCoverageDeg = 320.0;
    if (fitRotaryAxisLine(samples, QStringLiteral("C"), insufficientCoverage).success)
        return fail("insufficient angular coverage was accepted");

    QVector<CalibrationSample> movingFixedAxis = samples;
    movingFixedAxis.back().actualAxes[3] = 1.0;
    if (fitRotaryAxisLine(movingFixedAxis, QStringLiteral("C"), options).success)
        return fail("movement of the fixed rotary axis was accepted");

    QTemporaryDir temporary;
    if (!temporary.isValid()) return fail("temporary directory unavailable");
    MachineCalibrationService service(temporary.path());
    MachineCalibrationRecord record;
    record.machineIdentity = QStringLiteral("test-machine");
    record.nominalConfigurationFingerprint = QStringLiteral("nominal");
    record.controllerModelType = QStringLiteral("RW_C_ON_A");
    record.primaryAxis = fitted.fit;
    record.primaryAxis.axisName = QStringLiteral("A");
    record.slaveAxis = fitted.fit;
    record.samples = samples;
    record.tool.toolId = QStringLiteral("laser-focus");
    record.verification.state = CalibrationVerificationState::Computed;
    QString id;
    QString error;
    if (!service.saveCandidate(record, &id, &error))
        return fail(error.toStdString().c_str());
    MachineCalibrationRecord loaded;
    if (!service.record(id, &loaded, &error))
        return fail(error.toStdString().c_str());
    if (loaded.calibrationFingerprint.isEmpty()
        || loaded.rawSamplesSha256.isEmpty()
        || loaded.samples.size() != samples.size()) {
        return fail("calibration record round trip lost immutable data");
    }

    lcnc::MachineConfigurationService machine;
    machine.applyPreset(QStringLiteral("VERTICAL_AC_TABLE"));
    auto machineAxes = machine.axisConfigurations();
    for (auto& axis : machineAxes) {
        if (axis.axis.name == QStringLiteral("X")) axis.controllerIndex = 3;
        if (axis.axis.name == QStringLiteral("Y")) axis.controllerIndex = 4;
        if (axis.axis.name == QStringLiteral("Z")) axis.controllerIndex = 5;
        if (axis.axis.name == QStringLiteral("A")) axis.controllerIndex = 6;
        if (axis.axis.name == QStringLiteral("C")) axis.controllerIndex = 7;
        if (axis.axis.role == lcnc::MachineAxisRole::TableTilt)
            axis.axis.origin = gp_Pnt(123.0, -45.0, 67.0);
    }
    machine.setAxisConfigurations(machineAxes);
    gp_Pnt expectedPrimaryWorld;
    if (!machine.axisCoordinatesToWorld(gp_Pnt(123.0, -45.0, 67.0), &expectedPrimaryWorld))
        return fail("cannot convert the nominal rotary center");
    ControllerKinematicsSnapshot controller;
    if (!buildControllerKinematicsSnapshot(
            machine, nullptr, ControllerCalibrationRequirement::None,
            &controller, &error)
        || controller.modelType != 3
        || !close(controller.primaryAxisPointMcs[0], expectedPrimaryWorld.X(), 1e-9)
        || !close(controller.primaryAxisPointMcs[1], expectedPrimaryWorld.Y(), 1e-9)
        || !close(controller.primaryAxisPointMcs[2], expectedPrimaryWorld.Z(), 1e-9)
        || controller.physicalAxisIndices != std::array<short, 5>{3, 4, 5, 6, 7}
        || controller.physicalAxisIndices[0] == controller.physicalAxisIndices[3]
        || controller.scales[0].beta <= 0.0) {
        const QString detail = QStringLiteral(
            "nominal AC table did not produce a valid GTN controller snapshot: %1 axes=[%2,%3,%4,%5,%6]")
                                   .arg(error)
                                   .arg(controller.physicalAxisIndices[0])
                                   .arg(controller.physicalAxisIndices[1])
                                   .arg(controller.physicalAxisIndices[2])
                                   .arg(controller.physicalAxisIndices[3])
                                   .arg(controller.physicalAxisIndices[4]);
        return fail(detail.toStdString().c_str());
    }
    error.clear();
    if (buildControllerKinematicsSnapshot(
            machine, nullptr, ControllerCalibrationRequirement::MachineVerified,
            &controller, &error)
        || error.isEmpty()) {
        return fail("RTCP snapshot accepted a missing MachineVerified calibration");
    }
    QString derivedId;
    error.clear();
    const std::array<double, 3> coarseTcp{0.0, 0.0, 0.0};
    if (!service.createConfigurationDerivedCandidate(
            machine, coarseTcp, &derivedId, &error)) {
        return fail(error.toStdString().c_str());
    }
    MachineCalibrationRecord derived;
    if (!service.record(derivedId, &derived, &error)
        || derived.schemaVersion != MachineCalibrationRecord::kCurrentSchemaVersion
        || derived.verification.state != CalibrationVerificationState::ConfigurationDerived
        || !derived.samples.isEmpty()
        || derived.primaryAxis.axisName != QStringLiteral("A")
        || derived.slaveAxis.axisName != QStringLiteral("C")) {
        return fail("configuration-derived RTCP candidate is invalid");
    }

    // Reproduce the reported BC configuration: XYZ = +X,+Y,-Z and C = +Z.
    // Axis coordinates are left-handed, but GTN MCS and CAM TCPs are not.
    machine.applyPreset(QStringLiteral("VERTICAL_BC_TABLE"));
    machineAxes = machine.axisConfigurations();
    if (machineAxes.size() != 5) return fail("BC fixture does not have five physical axes");
    const std::array<gp_Dir, 5> directions{
        gp_Dir(1, 0, 0), gp_Dir(0, 1, 0), gp_Dir(0, 0, -1),
        gp_Dir(0, 1, 0), gp_Dir(0, 0, 1)};
    for (int i = 0; i < machineAxes.size(); ++i) {
        auto& axis = machineAxes[i];
        axis.controllerIndex = i + 3;
        axis.axis.direction = directions[i];
        if (i >= 3) axis.axis.origin = gp_Pnt(290, 40, 30);
    }
    machine.setAxisConfigurations(machineAxes);
    const auto verifyVectors = [&directions](const ControllerKinematicsSnapshot& value) {
        for (int i = 0; i < 5; ++i) {
            if (!close(value.axisVectorsMcs[i][0], directions[i].X(), 1e-12)
                || !close(value.axisVectorsMcs[i][1], directions[i].Y(), 1e-12)
                || !close(value.axisVectorsMcs[i][2], directions[i].Z(), 1e-12))
                return false;
        }
        return true;
    };
    if (!buildControllerKinematicsSnapshot(machine, nullptr,
            ControllerCalibrationRequirement::None, &controller, &error)
        || controller.modelType != 0 || controller.directionMode != 1
        || !verifyVectors(controller)
        || controller.primaryAxisPointMcs != std::array<double, 3>{290, 40, -30}
        || controller.slaveAxisPointMcs != std::array<double, 3>{290, 40, -30}
        || controller.physicalAxisIndices != std::array<short, 5>{3, 4, 5, 6, 7}
        || controller.directions != std::array<short, 5>{0, 0, 0, 0, 0}) {
        return fail("Z-down BC snapshot mirrored C or mixed axis centers with world MCS");
    }
    MachineKinematics referenceKinematics;
    referenceKinematics.setAxes(machine.axisDefinitions());
    for (double b : {-27.0, 0.0, 31.0}) {
        for (double c : {-93.0, 0.0, 89.0}) {
            const auto rotation = [](const std::array<double, 3>& center,
                                     const std::array<double, 3>& vector, double angle) {
                gp_Trsf result;
                result.SetRotation(gp_Ax1(gp_Pnt(center[0], center[1], center[2]),
                    gp_Dir(vector[0], vector[1], vector[2])),
                    angle * 3.14159265358979323846 / 180.0);
                return result;
            };
            const auto groupGeometry = rotation(controller.primaryAxisPointMcs,
                    controller.axisVectorsMcs[3], b)
                .Multiplied(rotation(controller.slaveAxisPointMcs,
                    controller.axisVectorsMcs[4], c));
            const gp_Pnt probe(317, 59, -11);
            if (probe.Transformed(groupGeometry).Distance(probe.Transformed(
                    referenceKinematics.computeAxisTransform(QStringLiteral("C"),
                        {{QStringLiteral("B"), b}, {QStringLiteral("C"), c}}))) > 1e-9)
                return fail("snapshot rotary geometry disagrees with CAM world kinematics");
        }
    }

    const auto nominalFingerprint = machine.configurationFingerprint();
    const std::array<double, 3> asymmetricTcp{12, -34, 56};
    if (!service.createConfigurationDerivedCandidate(machine, asymmetricTcp, &derivedId, &error)
        || !service.activate(derivedId, &machine, &error)
        || machine.configurationFingerprint() != nominalFingerprint
        || !buildControllerKinematicsSnapshot(machine, &service,
            ControllerCalibrationRequirement::MachineVerifiedOrConfigurationDerived,
            &controller, &error)
        || !verifyVectors(controller)
        || controller.toolLocationPointMcs != asymmetricTcp
        || controller.primaryAxisPointMcs != std::array<double, 3>{290, 40, -30}) {
        return fail("V2 calibration activation changed the configured axes or MCS frame");
    }

    // Genuine negative/oblique rotary directions must survive. This is a
    // coordinate-frame fix, not abs(C), a fixed C sign, or axis-name guessing.
    machineAxes = machine.axisConfigurations();
    machineAxes[3].axis.direction = gp_Dir(-1, 2, 3);
    machineAxes[4].axis.direction = gp_Dir(0, 0, -1);
    machine.setAxisConfigurations(machineAxes);
    if (!buildControllerKinematicsSnapshot(machine, nullptr,
            ControllerCalibrationRequirement::None, &controller, &error)
        || controller.axisVectorsMcs[4][2] != -1
        || !close(controller.axisVectorsMcs[3][0], -1 / std::sqrt(14.0), 1e-12)
        || !close(controller.axisVectorsMcs[3][2], 3 / std::sqrt(14.0), 1e-12)) {
        return fail("a real negative or oblique rotary direction was overwritten");
    }

    // Preserve legacy JSON as evidence. It must not block or override ACS,
    // but must still be rejected for either RTCP admission policy.
    if (!service.record(derivedId, &derived, &error)) return fail("cannot load V2 fixture");
    derived.schemaVersion = 1;
    error.clear();
    if (service.saveCandidate(derived, nullptr, &error) || error.isEmpty())
        return fail("legacy calibration was accepted as a new candidate");
    const QString recordPath = QDir(service.storageRoot()).filePath(derivedId + ".json");
    QFile jsonFile(recordPath);
    if (!jsonFile.open(QIODevice::ReadOnly)) return fail("cannot read legacy fixture");
    auto json = QJsonDocument::fromJson(jsonFile.readAll()).object();
    jsonFile.close();
    derived.calibrationFingerprint = MachineCalibrationService::computeCalibrationFingerprint(derived);
    json["schemaVersion"] = 1;
    json["calibrationFingerprint"] = derived.calibrationFingerprint;
    if (!jsonFile.open(QIODevice::WriteOnly | QIODevice::Truncate)
        || jsonFile.write(QJsonDocument(json).toJson()) < 0)
        return fail("cannot write legacy fixture");
    jsonFile.close();
    if (!service.record(derivedId, &derived, &error) || derived.schemaVersion != 1)
        return fail("legacy evidence is no longer readable");
    error.clear();
    if (service.activate(derivedId, &machine, &error) || error.isEmpty())
        return fail("legacy calibration activation was not rejected");
    QJsonObject pointer{
        {"schemaVersion", 1}, {"activeCalibrationId", derivedId},
        {"calibrationFingerprint", derived.calibrationFingerprint},
        {"machineConfigurationFingerprint", machine.configurationFingerprint()}};
    QFile pointerFile(QDir(service.storageRoot()).filePath("active.json"));
    if (!pointerFile.open(QIODevice::WriteOnly | QIODevice::Truncate)
        || pointerFile.write(QJsonDocument(pointer).toJson()) < 0)
        return fail("cannot write legacy active pointer");
    pointerFile.close();
    error.clear();
    if (!buildControllerKinematicsSnapshot(machine, &service,
            ControllerCalibrationRequirement::None, &controller, &error)
        || !error.isEmpty() || !controller.calibrationFingerprint.isEmpty()
        || controller.axisVectorsMcs[4][2] != -1)
        return fail("legacy RTCP calibration blocked or overrode non-RTCP axes");
    for (const auto requirement : {ControllerCalibrationRequirement::MachineVerified,
             ControllerCalibrationRequirement::MachineVerifiedOrConfigurationDerived}) {
        error.clear();
        if (buildControllerKinematicsSnapshot(machine, &service, requirement,
                &controller, &error) || error.isEmpty())
            return fail("active legacy calibration reached an RTCP Group snapshot");
    }
    return 0;
}
