#include "modules/process/runtime/axis_map.h"
#include "modules/process/runtime/machine_pose5.h"

#include "core/kinematics/machine_configuration_service.h"

#include <QHash>
#include <QStringBuilder>

namespace lcnc::process {

AxisMap AxisMap::from(lcnc::MachineConfigurationService* machineConfig,
                      const lcnc::MachineAxisLayout& layout)
{
    AxisMap map;

    if (!machineConfig || !layout.isValid()) {
        return map;
    }

    // 把构型展开为：名字 → (controllerIndex, vel, acc, jerk, motionType)
    struct AxisRow {
        int controllerIndex;
        double velocity, acceleration, jerk;
    };
    QHash<QString, AxisRow> rows;
    for (const auto& cfg : machineConfig->axisConfigurations()) {
        const QString name = cfg.axis.name.trimmed().toUpper();
        if (name.isEmpty() || name == QStringLiteral("BASE")) continue;
        AxisRow r;
        r.controllerIndex = cfg.controllerIndex;
        r.velocity        = cfg.highSpeed;
        r.acceleration    = cfg.acceleration;
        r.jerk            = cfg.jerk;
        rows.insert(name, r);
    }

    auto fill = [&](SemanticAxis sa, const QString& name) {
        auto it = rows.find(name);
        if (it == rows.end()) return;
        PerAxis& slot = map.m_axes[sa];
        slot.name = name;
        slot.controllerIndex = it.value().controllerIndex;
        slot.velocity        = it.value().velocity;
        slot.acceleration    = it.value().acceleration;
        slot.jerk            = it.value().jerk;
    };

    static constexpr SemanticAxis semanticOrder[Count] = {X, Y, Z, R1, R2};
    for (int index = 0; index < layout.count; ++index)
        fill(semanticOrder[index], layout.axes[index].name);

    return map;
}

bool AxisMap::isFiveAxis() const
{
    return isPresent(X) && isPresent(Y) && isPresent(Z)
        && isPresent(R1) && isPresent(R2);
}

int AxisMap::activeCount() const
{
    int n = 0;
    for (const auto& a : m_axes)
        if (a.controllerIndex >= 0)
            ++n;
    return n;
}

QString AxisMap::axisTupleText(std::uint8_t mask) const
{
    QString out = QStringLiteral("(");
    bool first = true;
    static constexpr SemanticAxis order[Count] = { X, Y, Z, R1, R2 };
    static constexpr std::uint8_t bits[Count]  = {
        MachinePose5::Bx, MachinePose5::By, MachinePose5::Bz,
        MachinePose5::Br1, MachinePose5::Br2
    };
    for (int i = 0; i < Count; ++i) {
        if (!(mask & bits[i])) continue;
        if (m_axes[order[i]].controllerIndex < 0) continue;
        if (!first) out += QStringLiteral(", ");
        out += QString::number(m_axes[order[i]].controllerIndex);
        first = false;
    }
    out += QLatin1Char(')');
    return out;
}

QVector<int> AxisMap::activeControllerIndices(std::uint8_t mask) const
{
    QVector<int> out;
    out.reserve(Count);
    static constexpr SemanticAxis order[Count] = { X, Y, Z, R1, R2 };
    static constexpr std::uint8_t bits[Count]  = {
        MachinePose5::Bx, MachinePose5::By, MachinePose5::Bz,
        MachinePose5::Br1, MachinePose5::Br2
    };
    for (int i = 0; i < Count; ++i) {
        if (!(mask & bits[i])) continue;
        const int idx = m_axes[order[i]].controllerIndex;
        if (idx >= 0)
            out.push_back(idx);
    }
    return out;
}

} // namespace lcnc::process
