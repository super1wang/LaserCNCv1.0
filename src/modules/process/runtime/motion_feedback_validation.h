#pragma once

#include <algorithm>
#include <cmath>
#include <QCoreApplication>
#include <QString>

namespace lcnc::process {

// Axis-space units: mm for linear axes and degrees for rotary axes. Never
// modulo rotary differences: +360 and -360 are different multi-turn positions.
inline double stationaryFeedbackTolerance(double pulsesPerUnit)
{
    return std::max(0.05, 2.0 / pulsesPerUnit);
}

inline bool stationaryFeedbackMatches(double profilePulse, double encoderPulse,
                                      double pulsesPerUnit)
{
    return std::isfinite(profilePulse) && std::isfinite(encoderPulse)
        && std::isfinite(pulsesPerUnit) && pulsesPerUnit > 0.0
        && std::abs(profilePulse - encoderPulse) / pulsesPerUnit
            <= stationaryFeedbackTolerance(pulsesPerUnit);
}

// First-fault evidence survives Stop/Reset and Group teardown. It is cleared
// only when a new connection is explicitly established.
struct StationaryFeedbackFault
{
    bool captured{false};
    QString axis;
    int physicalAxis{0};
    double profilePulse{0.0};
    double encoderPulse{0.0};
    double pulsesPerUnit{1.0};
    bool rotary{false};

    void capture(const QString& name, int index, double profile, double encoder,
                 double resolution, bool isRotary)
    {
        if (captured) return;
        captured = true;
        axis = name;
        physicalAxis = index;
        profilePulse = profile;
        encoderPulse = encoder;
        pulsesPerUnit = resolution;
        rotary = isRotary;
    }

    QString message() const
    {
        if (!captured) return {};
        // 中文翻译：GTN %1 轴（物理轴 %2）位置反馈校验失败：规划 %3，反馈 %4，偏差 %5 %6，容差 %7 %6。已阻止继续加工；停止/复位保留故障证据，请检查方向和反馈比例。
        return QCoreApplication::translate("GTNMotionControl",
            "GTN axis %1 (physical %2) feedback validation failed: profile %3, feedback %4, deviation %5 %6, tolerance %7 %6. Further machining is blocked; Stop/Reset preserves fault evidence. Check direction and feedback scaling.")
            .arg(axis).arg(physicalAxis)
            .arg(profilePulse / pulsesPerUnit, 0, 'f', 6)
            .arg(encoderPulse / pulsesPerUnit, 0, 'f', 6)
            .arg((encoderPulse - profilePulse) / pulsesPerUnit, 0, 'f', 6)
            .arg(rotary ? QStringLiteral("deg") : QStringLiteral("mm"))
            .arg(stationaryFeedbackTolerance(pulsesPerUnit), 0, 'f', 6);
    }
};

} // namespace lcnc::process
