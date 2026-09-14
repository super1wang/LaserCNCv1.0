#pragma once

#include <QDateTime>
#include <QString>
#include <QStringList>
#include <QVector>

#include <array>

namespace lcnc::kinematics {

enum class CalibrationVerificationState
{
    Draft,
    ConfigurationDerived,
    Computed,
    ControllerVerified,
    MachineVerified
};

struct CalibrationSample
{
    QString sampleId;
    QString targetAxisName;
    int targetAxisSlot{-1};
    std::array<double, 5> actualAxes{};
    std::array<double, 3> measuredReferencePointMcs{};
    QString measurementSource;
    QString timestampUtc;
    bool accepted{true};
};

struct AxisLineFit
{
    QString axisName;
    std::array<double, 3> pointMcs{};
    std::array<double, 3> unitVectorMcs{};
    int sampleCount{0};
    double angularCoverageDeg{0.0};
    double fittedRadiusMm{0.0};
    double rmsResidualMm{0.0};
    double maxResidualMm{0.0};
    double conditionMetric{0.0};
    QStringList suggestedRejectedSampleIds;
};

struct ToolCalibrationSnapshot
{
    QString toolId;
    std::array<double, 3> toolLocationPointMcs{};
    QString fingerprint;
};

struct CalibrationVerificationResult
{
    CalibrationVerificationState state{CalibrationVerificationState::Draft};
    double controllerTransformMaxAxisError{0.0};
    double fixedTcpRmsErrorMm{0.0};
    double fixedTcpMaxErrorMm{0.0};
    QString verifiedAtUtc;
    QString notes;
};

struct MachineCalibrationRecord
{
    // V1 mixed controller-axis points/vectors with world-space CAM TCPs.
    // V2 defines every MCS point and direction in right-handed machine world.
    // Legacy records remain readable evidence, but must not drive motion.
    static constexpr int kCurrentSchemaVersion = 2;
    int schemaVersion{kCurrentSchemaVersion};
    QString calibrationId;
    QString machineIdentity;
    QString nominalConfigurationFingerprint;
    QString controllerModelType;
    AxisLineFit primaryAxis;
    AxisLineFit slaveAxis;
    ToolCalibrationSnapshot tool;
    QVector<CalibrationSample> samples;
    CalibrationVerificationResult verification;
    QString rawSamplesSha256;
    QString calibrationFingerprint;
    QString createdAtUtc;
    QString operatorName;
    QString measurementDevice;
    QString softwareVersion;
};

QString calibrationVerificationStateName(CalibrationVerificationState state);
bool calibrationVerificationStateFromName(const QString& name,
                                          CalibrationVerificationState* state);

} // namespace lcnc::kinematics
