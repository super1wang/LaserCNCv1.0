#include "modules/process/runtime/axis_map.h"
#include "modules/process/runtime/machine_pose5.h"

#include "core/kinematics/machine_configuration_service.h"

#include <QHash>
#include <QStringBuilder>

namespace lcnc::process {

namespace {

// 判断 candidate 是否为 ancestor 的子孙（沿 parentAxis 链路）。
bool isDescendant(const QHash<QString, QString>& parentOf,
                   const QString& candidate,
                   const QString& ancestor)
{
    QString cur = parentOf.value(candidate);
    while (!cur.isEmpty()) {
        if (cur == ancestor) return true;
        cur = parentOf.value(cur);
    }
    return false;
}

} // namespace

AxisMap AxisMap::from(lcnc::MachineConfigurationService* machineConfig)
{
    AxisMap map;

    if (!machineConfig) {
        // 兜底：构型为空 / 调用方未注入 MachineConfigurationService。
        // 用 X=0,Y=1,Z=2,R1=3,R2=4 这套常见 5 轴序号，避免下游崩。
        for (int i = 0; i < Count; ++i) {
            map.m_axes[i].name = QString::number(i);
            map.m_axes[i].controllerIndex = i;
            map.m_axes[i].velocity     = 10.0;
            map.m_axes[i].acceleration = 200.0;
        }
        return map;
    }

    // 把构型展开为：名字 → (controllerIndex, vel, acc, jerk, motionType)
    struct AxisRow {
        int controllerIndex;
        double velocity, acceleration, jerk;
        bool isRotary;
        QString parentAxis;
    };
    QHash<QString, AxisRow> rows;
    QHash<QString, QString> parentOf;
    for (const auto& cfg : machineConfig->axisConfigurations()) {
        const QString name = cfg.axis.name.trimmed().toUpper();
        if (name.isEmpty() || name == QStringLiteral("BASE")) continue;
        AxisRow r;
        r.controllerIndex = cfg.controllerIndex;
        r.velocity        = cfg.highSpeed;
        r.acceleration    = cfg.acceleration;
        r.jerk            = cfg.jerk;
        r.isRotary        = (cfg.axis.motionType == MachineAxisDef::Rotary);
        r.parentAxis      = cfg.axis.parentAxis.trimmed().toUpper();
        rows.insert(name, r);
        parentOf.insert(name, r.parentAxis);
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

    // 线性轴按名字直接映射。
    if (rows.contains(QStringLiteral("X"))) fill(X, QStringLiteral("X"));
    if (rows.contains(QStringLiteral("Y"))) fill(Y, QStringLiteral("Y"));
    if (rows.contains(QStringLiteral("Z"))) fill(Z, QStringLiteral("Z"));

    // 旋转轴按 parent-child 关系映射，与 IKSolver::solve 保持一致：
    //   - R1（语义 "第一旋转轴"）= 子轴（chainTrsf 中先作用于工件 / 切割头的旋转）
    //   - R2（语义 "第二旋转轴"）= 父轴
    QStringList rotaryAxes;
    for (auto it = rows.begin(); it != rows.end(); ++it) {
        if (it.value().isRotary) rotaryAxes.append(it.key());
    }
    if (rotaryAxes.size() == 1) {
        // 单旋转轴时无父子关系，直接挂到 R1。
        fill(R1, rotaryAxes.first());
    } else if (rotaryAxes.size() >= 2) {
        QString a = rotaryAxes[0];
        QString b = rotaryAxes[1];
        QString child, parent;
        if (isDescendant(parentOf, a, b)) { child = a; parent = b; }
        else if (isDescendant(parentOf, b, a)) { child = b; parent = a; }
        else { /* 兄弟轴或同级：兜底按名字 */ child = a; parent = b; }
        fill(R1, child);
        fill(R2, parent);
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
