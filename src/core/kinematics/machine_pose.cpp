#include "core/kinematics/machine_pose.h"

#include "core/kinematics/machine_kinematics.h"
#include "core/logging/logger.h"

namespace lcnc {

MachinePose::MachinePose(QObject* parent) : QObject(parent)
{
    LCNC_DEBUG(LogCode::Generic, "MachinePose ctor");
}

MachinePose::~MachinePose()
{
    LCNC_DEBUG(LogCode::Generic, "MachinePose dtor");
}

void MachinePose::setKinematics(MachineKinematics* kin)
{
    LCNC_DEBUG(LogCode::Generic,
               "MachinePose::setKinematics kin={}", static_cast<const void*>(kin));
    // MachineConfigurationService updates the axis list in-place on the same
    // MachineKinematics instance when a physical preset changes. Pointer
    // identity alone therefore cannot decide whether the pose is current.
    // Rebuild on every injection so an XYZ(A) pose never rejects C/B feedback
    // after switching to an AC/BC table.
    m_kin = kin;
    rebuildFromKinematics();
    LCNC_INFO(LogCode::Generic,
              "MachinePose: kinematics updated, supported axes = [{}]",
              m_supportedAxes.join(QStringLiteral(",")).toStdString());
    emit supportedAxesChanged(m_supportedAxes);
    if (!m_supportedAxes.isEmpty())
        emit poseChanged(m_supportedAxes);
}

void MachinePose::rebuildFromKinematics()
{
    m_supportedAxes.clear();
    m_values.clear();
    if (!m_kin)
        return;
    for (const auto& def : m_kin->axes()) {
        m_supportedAxes.append(def.name);
        m_values.insert(def.name, def.currentPos);
    }
}

bool MachinePose::setAxisValue(const QString& axis, double value, bool emitChanged)
{
    if (!m_kin) {
        LCNC_WARN(LogCode::Generic,
                  "MachinePose::setAxisValue rejected: no kinematics (axis={})",
                  axis.toStdString());
        return false;
    }
    auto it = m_values.find(axis);
    if (it == m_values.end()) {
        LCNC_WARN(LogCode::Generic,
                  "MachinePose::setAxisValue: unsupported axis '{}' (supported=[{}])",
                  axis.toStdString(),
                  m_supportedAxes.join(QStringLiteral(",")).toStdString());
        return false;
    }
    if (qFuzzyCompare(1.0 + it.value(), 1.0 + value))
        return false;
    it.value() = value;
    // 同步写回 kinematics，使其几何变换链立即可用。
    m_kin->setAxisPosition(axis, value);
    if (emitChanged) {
        LCNC_DEBUG(LogCode::Generic,
                   "MachinePose::setAxisValue {}={}", axis.toStdString(), value);
        emit poseChanged({axis});
    }
    return true;
}

void MachinePose::setAxisValuesBatch(const QHash<QString, double>& values)
{
    LCNC_DEBUG(LogCode::Generic,
               "MachinePose::setAxisValuesBatch n={}", values.size());
    if (!m_kin) {
        LCNC_WARN(LogCode::Generic,
                  "MachinePose::setAxisValuesBatch rejected: no kinematics");
        return;
    }
    QStringList dirty;
    dirty.reserve(values.size());
    for (auto it = values.cbegin(); it != values.cend(); ++it) {
        if (setAxisValue(it.key(), it.value(), /*emitChanged*/ false))
            dirty.append(it.key());
    }
    if (!dirty.isEmpty()) {
        LCNC_DEBUG(LogCode::Generic,
                   "MachinePose batch dirty axes = [{}]",
                   dirty.join(QStringLiteral(",")).toStdString());
        emit poseChanged(dirty);
    }
}

double MachinePose::axisValue(const QString& axis) const
{
    auto it = m_values.constFind(axis);
    if (it == m_values.cend()) {
        LCNC_DEBUG(LogCode::Generic,
                   "MachinePose::axisValue: unknown axis '{}', return 0",
                   axis.toStdString());
        return 0.0;
    }
    return it.value();
}

} // namespace lcnc
