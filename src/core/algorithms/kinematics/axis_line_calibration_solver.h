#pragma once

#include "core/kinematics/machine_calibration_record.h"

#include <QString>

namespace lcnc::kinematics {

struct AxisLineCalibrationOptions
{
    int minimumSamples{3};
    double minimumAngularCoverageDeg{60.0};
    double minimumRadiusMm{1.0};
    double maximumRmsResidualMm{1.0};
    double maximumResidualMm{2.0};
    double maximumFixedRotaryDriftDeg{0.25};
    double outlierSuggestionFactor{3.0};
};

struct AxisLineCalibrationResult
{
    bool success{false};
    AxisLineFit fit;
    QString error;
};

AxisLineCalibrationResult fitRotaryAxisLine(
    const QVector<CalibrationSample>& samples,
    const QString& axisName,
    const AxisLineCalibrationOptions& options = {});

} // namespace lcnc::kinematics
