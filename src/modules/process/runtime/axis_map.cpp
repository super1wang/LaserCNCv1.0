#include "modules/process/runtime/axis_map.h"
#include "modules/process/runtime/machine_pose5.h"

#include "core/kinematics/machine_configuration_service.h"

#include <QStringBuilder>

namespace lcnc::process {

namespace {

// 把构型里"BASE/X/Y/Z/A/B/C"轴名映射到语义轴位。
AxisMap::SemanticAxis classify(const QString& nameRaw)
{
    const QString n = nameRaw.trimmed().toUpper();
    if (n == QStringLiteral("X")) return AxisMap::X;
    if (n == QStringLiteral("Y")) return AxisMap::Y;
    if (n == QStringLiteral("Z")) return AxisMap::Z;
    // 旋转轴：A / R1 视为第一旋转轴，B/C/R2 视为第二旋转轴。
    if (n == QStringLiteral("A") || n == QStringLiteral("R1"))
        return AxisMap::R1;
    if (n == QStringLiteral("B") || n == QStringLiteral("C") || n == QStringLiteral("R2"))
        return AxisMap::R2;
    return AxisMap::Count;  // 其它（BASE 之类）不入表
}

} // namespace

AxisMap AxisMap::from(lcnc::MachineConfigurationService* machineConfig)
{
    AxisMap map;

    if (machineConfig) {
        for (const auto& cfg : machineConfig->axisConfigurations()) {
            const SemanticAxis sa = classify(cfg.axis.name);
            if (sa == Count)
                continue;
            PerAxis& slot = map.m_axes[sa];
            slot.controllerIndex = cfg.controllerIndex;
            slot.velocity        = cfg.highSpeed;
            slot.acceleration    = cfg.acceleration;
            slot.jerk            = cfg.jerk;
        }
    }

    // 兜底：构型为空 / 调用方未注入 MachineConfigurationService。
    // 用 X=0,Y=1,Z=2,R1=3,R2=4 这套常见 5 轴序号，避免下游崩 (Fix-line)。
    if (!map.isPresent(X) && !map.isPresent(Y)) {
        for (int i = 0; i < Count; ++i) {
            map.m_axes[i].controllerIndex = i;
            if (map.m_axes[i].velocity == 0.0)     map.m_axes[i].velocity     = 10.0;
            if (map.m_axes[i].acceleration == 0.0) map.m_axes[i].acceleration = 200.0;
        }
    }

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
