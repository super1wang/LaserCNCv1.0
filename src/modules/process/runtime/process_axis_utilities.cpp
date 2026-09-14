#include "modules/process/runtime/process_axis_utilities.h"

#include <algorithm>
#include <cmath>

namespace lcnc::process {

QStringList homeOrderForAxes(const QList<MachineAxisDef>& axes)
{
    QStringList order{QStringLiteral("Z")};
    for (const MachineAxisDef& axis : axes) {
        const QString key = axis.name.trimmed().toUpper();
        if (!key.isEmpty() && key != QStringLiteral("BASE") && !order.contains(key))
            order.append(key);
    }
    return order;
}

bool sameAxisDefinitions(const QList<MachineAxisDef>& lhs,
                         const QList<MachineAxisDef>& rhs)
{
    if (lhs.size() != rhs.size())
        return false;

    for (int i = 0; i < lhs.size(); ++i) {
        const MachineAxisDef& a = lhs.at(i);
        const MachineAxisDef& b = rhs.at(i);
        if (a.name != b.name || a.motionType != b.motionType || a.parentAxis != b.parentAxis
            || std::abs(a.minVal - b.minVal) > 1e-9 || std::abs(a.maxVal - b.maxVal) > 1e-9)
            return false;
    }
    return true;
}

double simulatedAxisValue(const MachineAxisDef& axis,
                          double phase,
                          int linearIndex,
                          int rotaryIndex)
{
    const double axisLimit = std::max(std::abs(axis.minVal), std::abs(axis.maxVal));
    if (axis.motionType == MachineAxisDef::Linear) {
        const double amplitude = axisLimit > 1e-6 ? std::clamp(axisLimit * 0.12, 5.0, 60.0) : 20.0;
        const double wave = std::sin(phase * (0.55 + 0.18 * linearIndex) + linearIndex * 0.8);
        return axis.name == QStringLiteral("Z") ? amplitude + wave * amplitude * 0.7 : wave * amplitude;
    }

    const double amplitude = axisLimit >= 9000.0 ? 90.0 : std::max(10.0, axisLimit * 0.35);
    return std::sin(phase * (0.35 + 0.12 * rotaryIndex) + rotaryIndex * 0.6) * amplitude;
}

} // namespace lcnc::process
