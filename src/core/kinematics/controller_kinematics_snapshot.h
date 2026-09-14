#pragma once

#include <QString>

#include <array>

namespace lcnc {

class MachineConfigurationService;

namespace kinematics {

class MachineCalibrationService;

enum class ControllerCalibrationRequirement
{
    None,
    MachineVerified,
    MachineVerifiedOrConfigurationDerived
};

struct ControllerProfileScale
{
    short count{1};
    double alpha{1.0};
    double beta{1.0};
};

struct ControllerKinematicsSnapshot
{
    // MCS model points/vectors use the right-handed, Z-up ZERO-pose machine
    // reference. Table RTCP targets must remove posed carrier motion first;
    // scene-world TCP is not an interchangeable command. Axis values stay native.
    // 中文翻译：MCS 使用零位右手机床参考系，RTCP 指令点须先去除工件载体运动。
    short modelType{-1};
    QString primaryAxisName;
    QString slaveAxisName;
    std::array<double, 3> primaryAxisPointMcs{};
    std::array<double, 3> slaveAxisPointMcs{};
    std::array<double, 3> toolLocationPointMcs{};
    short directionMode{1};
    std::array<short, 5> directions{};
    std::array<std::array<double, 3>, 5> axisVectorsMcs{};
    std::array<short, 5> physicalAxisIndices{};
    std::array<ControllerProfileScale, 5> scales{};
    QString machineKinematicsFingerprint;
    QString calibrationFingerprint;
    QString toolCalibrationFingerprint;
    bool calibrationMachineVerified{false};
    bool calibrationConfigurationDerived{false};
};

bool buildControllerKinematicsSnapshot(
    const MachineConfigurationService& machine,
    const MachineCalibrationService* calibration,
    ControllerCalibrationRequirement calibrationRequirement,
    ControllerKinematicsSnapshot* snapshot,
    QString* error = nullptr);

} // namespace kinematics
} // namespace lcnc
