#include "core/kinematics/machine_configuration_service.h"

#include "core/logging/logger.h"

#include <QCoreApplication>
#include <QCryptographicHash>
#include <QDir>
#include <QFileInfo>

#include <algorithm>
#include <cmath>

namespace lcnc {
namespace {

constexpr double kDefaultAxisJerk = 10000.0;
// A workpiece rotary has no travel stop.  Keep a finite planning envelope so
// controller-facing configuration stays representable, while allowing many
// full turns and avoiding the legacy table-tilt +/-120 degree default.
// 中文翻译：工件回转轴无行程止挡；使用有限规划范围以兼容控制器配置，并避免误用转台倾斜轴的 +/-120 度默认值。
constexpr double kContinuousRotaryPlanningLimitDeg = 9999.0;

void sanitizeAxisMotionParameters(MachineAxisRuntimeConfig& config)
{
    // ACS rejects a zero S-curve jerk value.  Earlier table presets saved
    // zero for A/C, so migrate those persisted values as well as new input.
    // 中文翻译：ACS 不接受零加加速度；迁移旧版 A/C 配置并保护新输入。
    if (!std::isfinite(config.jerk) || config.jerk <= 0.0)
        config.jerk = kDefaultAxisJerk;
}

QString motionTypeName(MachineAxisDef::MotionType type)
{
    return type == MachineAxisDef::Rotary ? QStringLiteral("Rotary") : QStringLiteral("Linear");
}

MachineAxisDef::MotionType motionTypeFromName(const QString& name)
{
    const QString normalized = name.trimmed().toLower();
    // 中文翻译：旋转
    return normalized == QStringLiteral("rotary") || normalized == QStringLiteral("rotate")
        ? MachineAxisDef::Rotary
        : MachineAxisDef::Linear;
}

bool sameAxisConfig(const MachineAxisRuntimeConfig& lhs, const MachineAxisRuntimeConfig& rhs)
{
    return lhs.axis.name == rhs.axis.name
        && lhs.axis.motionType == rhs.axis.motionType
        && lhs.axis.parentAxis == rhs.axis.parentAxis
        && lhs.axis.role == rhs.axis.role
        && std::abs(lhs.axis.direction.X() - rhs.axis.direction.X()) < 1e-9
        && std::abs(lhs.axis.direction.Y() - rhs.axis.direction.Y()) < 1e-9
        && std::abs(lhs.axis.direction.Z() - rhs.axis.direction.Z()) < 1e-9
        && std::abs(lhs.axis.origin.X() - rhs.axis.origin.X()) < 1e-9
        && std::abs(lhs.axis.origin.Y() - rhs.axis.origin.Y()) < 1e-9
        && std::abs(lhs.axis.origin.Z() - rhs.axis.origin.Z()) < 1e-9
        && std::abs(lhs.axis.minVal - rhs.axis.minVal) < 1e-9
        && std::abs(lhs.axis.maxVal - rhs.axis.maxVal) < 1e-9
        && lhs.controllerIndex == rhs.controllerIndex
        && lhs.homeIndex == rhs.homeIndex
        && std::abs(lhs.resolution - rhs.resolution) < 1e-9
        && std::abs(lhs.motionSpeed - rhs.motionSpeed) < 1e-9
        && std::abs(lhs.lowSpeed - rhs.lowSpeed) < 1e-9
        && std::abs(lhs.mediumSpeed - rhs.mediumSpeed) < 1e-9
        && std::abs(lhs.highSpeed - rhs.highSpeed) < 1e-9
        && std::abs(lhs.acceleration - rhs.acceleration) < 1e-9
        && std::abs(lhs.jerk - rhs.jerk) < 1e-9
        && std::abs(lhs.pipeDiameter - rhs.pipeDiameter) < 1e-9;
}

void copyHardwareConfig(MachineAxisRuntimeConfig& target, const MachineAxisRuntimeConfig& source)
{
    target.controllerIndex = source.controllerIndex;
    target.homeIndex = source.homeIndex;
    target.resolution = source.resolution;
    target.motionSpeed = source.motionSpeed;
    target.axis.minVal = source.axis.minVal;
    target.axis.maxVal = source.axis.maxVal;
    target.lowSpeed = source.lowSpeed;
    target.mediumSpeed = source.mediumSpeed;
    target.highSpeed = source.highSpeed;
    target.acceleration = source.acceleration;
    target.jerk = source.jerk;
    target.pipeDiameter = source.pipeDiameter;
}

} // namespace

QString machineToolpathAlgorithmName(MachineToolpathAlgorithm algorithm)
{
    switch (algorithm) {
    case MachineToolpathAlgorithm::ThreeAxis:
        return QStringLiteral("ThreeAxis");
    case MachineToolpathAlgorithm::FiveAxisTable:
        return QStringLiteral("FiveAxisTable");
    case MachineToolpathAlgorithm::FiveAxisHead:
        return QStringLiteral("FiveAxisHead");
    }
    return QStringLiteral("ThreeAxis");
}

MachineConfigurationService::MachineConfigurationService(QObject* parent)
    : QObject(parent)
{
    setPresetDefaults(m_presetName);
}

QString MachineConfigurationService::defaultFilePath()
{
    const QString exeDir = QFileInfo(QCoreApplication::applicationFilePath()).absolutePath();
    return QDir(exeDir).filePath(QStringLiteral("config/machine.toml"));
}

bool MachineConfigurationService::loadDefault()
{
    const bool ok = load(defaultFilePath());
    if (m_axisConfigs.isEmpty())
        setPresetDefaults(m_presetName);
    return ok;
}

bool MachineConfigurationService::saveDefault() const
{
    return save(defaultFilePath());
}

void MachineConfigurationService::applyPreset(const QString& presetName)
{
    setPresetDefaults(presetName.trimmed().isEmpty() ? QStringLiteral("VERTICAL_AC_TABLE") : presetName.trimmed());
    notifyChanged();
}

void MachineConfigurationService::setAxisConfigurations(const QVector<MachineAxisRuntimeConfig>& configs)
{
    QVector<MachineAxisRuntimeConfig> sanitized;
    for (const MachineAxisRuntimeConfig& config : configs) {
        MachineAxisRuntimeConfig next = config;
        next.axis.name = next.axis.name.trimmed().toUpper();
        next.axis.parentAxis = next.axis.parentAxis.trimmed().toUpper();
        if (next.axis.name.isEmpty() || next.axis.name == QStringLiteral("BASE"))
            continue;
        if (next.axis.minVal > next.axis.maxVal)
            std::swap(next.axis.minVal, next.axis.maxVal);
        if (next.controllerIndex < 0)
            next.controllerIndex = sanitized.size();
        if (next.homeIndex < 0)
            next.homeIndex = next.controllerIndex;
        sanitizeAxisMotionParameters(next);
        sanitized.append(next);
    }
    if (sanitized.size() == m_axisConfigs.size()) {
        bool same = true;
        for (int index = 0; index < sanitized.size(); ++index) {
            if (!sameAxisConfig(sanitized.at(index), m_axisConfigs.at(index))) {
                same = false;
                break;
            }
        }
        if (same)
            return;
    }
    m_axisConfigs = sanitized;
    notifyChanged();
}

void MachineConfigurationService::setMachineAxisDefinitions(const QString& presetName,
                                                            const QList<MachineAxisDef>& axes)
{
    const QVector<MachineAxisRuntimeConfig> nextConfigs = mergedAxisConfigurations(axes);

    const QString nextPreset = presetName.trimmed().isEmpty()
        ? QStringLiteral("VERTICAL_AC_TABLE")
        : presetName.trimmed();
    bool same = (nextPreset == m_presetName) && (nextConfigs.size() == m_axisConfigs.size());
    if (same) {
        for (int i = 0; i < nextConfigs.size(); ++i) {
            if (!sameAxisConfig(nextConfigs.at(i), m_axisConfigs.at(i))) {
                same = false;
                break;
            }
        }
    }
    if (same)
        return;
    m_presetName = nextPreset;
    m_axisConfigs = nextConfigs;
    notifyChanged();
}

QVector<MachineAxisRuntimeConfig> MachineConfigurationService::mergedAxisConfigurations(
    const QList<MachineAxisDef>& axes) const
{
    QMap<QString, MachineAxisRuntimeConfig> previous;
    for (const MachineAxisRuntimeConfig& config : m_axisConfigs)
        previous.insert(config.axis.name.toUpper(), config);

    QVector<MachineAxisRuntimeConfig> nextConfigs;
    int index = 0;
    for (MachineAxisDef axis : axes) {
        axis.name = axis.name.trimmed().toUpper();
        axis.parentAxis = axis.parentAxis.trimmed().toUpper();
        if (axis.name.isEmpty() || axis.name == QStringLiteral("BASE"))
            continue;
        MachineAxisRuntimeConfig config = defaultRuntimeConfig(axis, index);
        if (previous.contains(axis.name))
            copyHardwareConfig(config, previous.value(axis.name));
        sanitizeAxisMotionParameters(config);
        config.axis.name = axis.name;
        config.axis.motionType = axis.motionType;
        config.axis.role = axis.role;
        config.axis.direction = axis.direction;
        config.axis.origin = axis.origin;
        config.axis.parentAxis = axis.parentAxis.isEmpty() ? QStringLiteral("BASE") : axis.parentAxis;
        nextConfigs.append(config);
        ++index;
    }
    return nextConfigs;
}

void MachineConfigurationService::setAxisHardwareConfigurations(const QVector<MachineAxisRuntimeConfig>& configs)
{
    QMap<QString, MachineAxisRuntimeConfig> incoming;
    for (MachineAxisRuntimeConfig config : configs) {
        config.axis.name = config.axis.name.trimmed().toUpper();
        if (!config.axis.name.isEmpty())
            incoming.insert(config.axis.name, config);
    }

    QVector<MachineAxisRuntimeConfig> nextConfigs = m_axisConfigs;
    bool changed = false;
    for (MachineAxisRuntimeConfig& config : nextConfigs) {
        const QString key = config.axis.name.toUpper();
        if (!incoming.contains(key))
            continue;
        MachineAxisRuntimeConfig next = config;
        copyHardwareConfig(next, incoming.value(key));
        if (next.axis.minVal > next.axis.maxVal)
            std::swap(next.axis.minVal, next.axis.maxVal);
        if (!sameAxisConfig(config, next)) {
            config = next;
            changed = true;
        }
    }
    if (!changed)
        return;
    m_axisConfigs = nextConfigs;
    notifyChanged();
}

QList<MachineAxisDef> MachineConfigurationService::axisDefinitions() const
{
    QList<MachineAxisDef> axes;
    MachineAxisDef base;
    base.name = QStringLiteral("BASE");
    base.motionType = MachineAxisDef::Linear;
    base.direction = gp_Dir(0, 0, 1);
    base.minVal = 0.0;
    base.maxVal = 0.0;
    axes.append(base);
    for (const MachineAxisRuntimeConfig& config : m_axisConfigs)
        axes.append(config.axis);
    return axes;
}

MachineToolpathAlgorithm MachineConfigurationService::toolpathAlgorithm() const
{
    switch (defaultMachiningMode()) {
    case MachiningMode::SimultaneousHead5Axis: return MachineToolpathAlgorithm::FiveAxisHead;
    case MachiningMode::SimultaneousTable5Axis:
    case MachiningMode::RotaryTube4Axis:       return MachineToolpathAlgorithm::FiveAxisTable;
    case MachiningMode::Planar3Axis:
    default:                                   return MachineToolpathAlgorithm::ThreeAxis;
    }
}

QList<MachiningMode> MachineConfigurationService::supportedMachiningModes() const
{
    const auto hasRole = [this](MachineAxisRole role) {
        return std::any_of(m_axisConfigs.cbegin(), m_axisConfigs.cend(),
                           [role](const MachineAxisRuntimeConfig& config) {
                               return config.axis.role == role;
                           });
    };

    const bool hasCartesian = hasRole(MachineAxisRole::LinearX)
        && hasRole(MachineAxisRole::LinearY)
        && hasRole(MachineAxisRole::LinearZ);
    QList<MachiningMode> modes;
    if (hasCartesian) {
        modes.append(MachiningMode::Planar3Axis);
    }
    if (hasCartesian && (hasRole(MachineAxisRole::WorkpieceRotary)
        || (hasRole(MachineAxisRole::TableTilt) && hasRole(MachineAxisRole::TableSpin)))) {
        modes.append(MachiningMode::RotaryTube4Axis);
    }
    if (hasCartesian && hasRole(MachineAxisRole::TableTilt)
        && hasRole(MachineAxisRole::TableSpin))
        modes.append(MachiningMode::SimultaneousTable5Axis);
    const double beamMagnitude = std::sqrt(
        m_headToolGeometry.zeroBeamX * m_headToolGeometry.zeroBeamX
        + m_headToolGeometry.zeroBeamY * m_headToolGeometry.zeroBeamY
        + m_headToolGeometry.zeroBeamZ * m_headToolGeometry.zeroBeamZ);
    if (hasCartesian && hasRole(MachineAxisRole::HeadTiltPrimary)
        && hasRole(MachineAxisRole::HeadTiltSecondary)
        && m_headToolGeometry.focusLength > 0.0 && beamMagnitude > 1e-9)
        modes.append(MachiningMode::SimultaneousHead5Axis);
    return modes;
}

MachiningMode MachineConfigurationService::defaultMachiningMode() const
{
    const QList<MachiningMode> modes = supportedMachiningModes();
    // A newly created project starts with the highest coordinated mode that
    // the currently selected physical machine can actually support.
    // 中文翻译：新项目默认使用当前物理机床实际支持的最高联动加工模式。
    for (MachiningMode preferred : {MachiningMode::SimultaneousHead5Axis,
                                    MachiningMode::SimultaneousTable5Axis,
                                    MachiningMode::RotaryTube4Axis,
                                    MachiningMode::Planar3Axis}) {
        if (modes.contains(preferred))
            return preferred;
    }
    return MachiningMode::Planar3Axis;
}

void MachineConfigurationService::setWorkpieceSetupTransform(const WorkpieceSetupTransform& setup)
{
    const auto close = [](double lhs, double rhs) { return std::abs(lhs - rhs) <= 1e-9; };
    if (close(m_workpieceSetup.x, setup.x) && close(m_workpieceSetup.y, setup.y)
        && close(m_workpieceSetup.z, setup.z)
        && close(m_workpieceSetup.rotationXDeg, setup.rotationXDeg)
        && close(m_workpieceSetup.rotationYDeg, setup.rotationYDeg)
        && close(m_workpieceSetup.rotationZDeg, setup.rotationZDeg))
        return;
    m_workpieceSetup = setup;
    notifyChanged();
}

MachineModeDefinition MachineConfigurationService::modeDefinition(MachiningMode mode) const
{
    MachineModeDefinition definition;
    definition.mode = mode;
    definition.solverId = machiningModeName(mode);
    definition.solverVersion = machiningModeSolverVersion(mode);

    auto appendRole = [this, &definition](MachineAxisRole role) {
        for (const MachineAxisRuntimeConfig& config : m_axisConfigs) {
            if (config.axis.role == role)
                return definition.interpolatedAxes.append(config.axis.name, role);
        }
        return false;
    };
    appendRole(MachineAxisRole::LinearX);
    appendRole(MachineAxisRole::LinearY);
    appendRole(MachineAxisRole::LinearZ);

    switch (mode) {
    case MachiningMode::Planar3Axis:
        for (const MachineAxisRuntimeConfig& config : m_axisConfigs)
            if (config.axis.motionType == MachineAxisDef::Rotary)
                definition.lockedAxisTargets.insert(config.axis.name, 0.0);
        break;
    case MachiningMode::RotaryTube4Axis:
        if (!appendRole(MachineAxisRole::WorkpieceRotary)) {
            appendRole(MachineAxisRole::TableSpin);
            for (const MachineAxisRuntimeConfig& config : m_axisConfigs)
                if (config.axis.role == MachineAxisRole::TableTilt)
                    definition.lockedAxisTargets.insert(config.axis.name, 90.0);
        }
        break;
    case MachiningMode::SimultaneousTable5Axis:
        appendRole(MachineAxisRole::TableTilt);
        appendRole(MachineAxisRole::TableSpin);
        break;
    case MachiningMode::SimultaneousHead5Axis:
        appendRole(MachineAxisRole::HeadTiltPrimary);
        appendRole(MachineAxisRole::HeadTiltSecondary);
        break;
    }
    if (m_lockedTargetOverrides.contains(mode)) {
        // A persisted override may only adjust an axis that this physical
        // topology already locks.  In particular, a stale AC-table A=90
        // record must never turn XYZA's participating workpiece A axis into
        // a locked axis.
        // 中文翻译：持久化覆盖项只能调整当前物理拓扑本来就需要锁定的轴；旧 AC 转台的 A=90 不能把 XYZA 的参与 A 轴变成锁定轴。
        const QMap<QString, double>& overrides = m_lockedTargetOverrides.value(mode);
        for (auto it = definition.lockedAxisTargets.begin(); it != definition.lockedAxisTargets.end(); ++it) {
            const auto overrideIt = overrides.constFind(it.key());
            if (overrideIt != overrides.cend())
                it.value() = overrideIt.value();
        }
    }
    return definition;
}

bool MachineConfigurationService::supportsMachiningMode(MachiningMode mode) const
{
    return supportedMachiningModes().contains(mode) && modeDefinition(mode).isValid();
}

bool MachineConfigurationService::validateConfiguration(QString* errorMessage) const
{
    QHash<QString, bool> names;
    QHash<int, QString> controllerOwners;
    QHash<MachineAxisRole, QString> roleOwners;
    for (const MachineAxisRuntimeConfig& config : m_axisConfigs) {
        const QString name = config.axis.name.trimmed().toUpper();
        if (name.isEmpty() || names.contains(name)) {
            if (errorMessage) *errorMessage = QStringLiteral("Machine axis names must be non-empty and unique");
            return false;
        }
        names.insert(name, true);
        if (config.controllerIndex < 0) {
            if (errorMessage) *errorMessage = QStringLiteral("Machine axis %1 has an invalid controller index").arg(name);
            return false;
        }
        if (controllerOwners.contains(config.controllerIndex)) {
            if (errorMessage) *errorMessage = QStringLiteral("Controller axis index %1 is assigned more than once").arg(config.controllerIndex);
            return false;
        }
        controllerOwners.insert(config.controllerIndex, name);
        if (config.axis.role != MachineAxisRole::Unspecified) {
            if (roleOwners.contains(config.axis.role)) {
                if (errorMessage) *errorMessage = QStringLiteral("Machine axis role %1 is assigned more than once")
                    .arg(machineAxisRoleName(config.axis.role));
                return false;
            }
            roleOwners.insert(config.axis.role, name);
        } else {
            if (errorMessage) *errorMessage = QStringLiteral("Machine axis %1 has no semantic role").arg(name);
            return false;
        }
        if (!std::isfinite(config.axis.minVal) || !std::isfinite(config.axis.maxVal)
            || config.axis.minVal > config.axis.maxVal) {
            if (errorMessage) *errorMessage = QStringLiteral("Machine axis %1 has invalid soft limits").arg(name);
            return false;
        }
        if (!std::isfinite(config.resolution) || config.resolution <= 0.0
            || !std::isfinite(config.motionSpeed) || config.motionSpeed <= 0.0
            || !std::isfinite(config.highSpeed) || config.highSpeed <= 0.0
            || !std::isfinite(config.acceleration) || config.acceleration <= 0.0) {
            if (errorMessage) *errorMessage = QStringLiteral("Machine axis %1 has invalid motion parameters").arg(name);
            return false;
        }
        if (!std::isfinite(config.jerk) || config.jerk <= 0.0) {
            if (errorMessage) *errorMessage = QStringLiteral("Machine axis %1 jerk must be greater than zero").arg(name);
            return false;
        }
    }

    QHash<QString, QString> parentOf;
    for (const MachineAxisRuntimeConfig& config : m_axisConfigs)
        parentOf.insert(config.axis.name, config.axis.parentAxis);
    for (const MachineAxisRuntimeConfig& config : m_axisConfigs) {
        if (!config.axis.parentAxis.isEmpty() && config.axis.parentAxis != QStringLiteral("BASE")
            && !parentOf.contains(config.axis.parentAxis)) {
            if (errorMessage) *errorMessage = QStringLiteral("Machine axis %1 references missing parent %2")
                .arg(config.axis.name, config.axis.parentAxis);
            return false;
        }
        QHash<QString, bool> visited;
        QString current = config.axis.name;
        while (!current.isEmpty() && current != QStringLiteral("BASE")) {
            if (visited.contains(current)) {
                if (errorMessage) *errorMessage = QStringLiteral("Machine axis parent chain contains a cycle at %1").arg(current);
                return false;
            }
            visited.insert(current, true);
            current = parentOf.value(current);
        }
    }
    for (MachiningMode mode : supportedMachiningModes()) {
        const MachineModeDefinition definition = modeDefinition(mode);
        if (definition.interpolatedAxes.count > MachineAxisLayout::kMaxAxes
            || !definition.isValid(errorMessage)) return false;
        for (auto it = definition.lockedAxisTargets.cbegin();
             it != definition.lockedAxisTargets.cend(); ++it) {
            const auto axisIt = std::find_if(m_axisConfigs.cbegin(), m_axisConfigs.cend(),
                [&it](const MachineAxisRuntimeConfig& config) { return config.axis.name == it.key(); });
            if (axisIt == m_axisConfigs.cend() || it.value() < axisIt->axis.minVal
                || it.value() > axisIt->axis.maxVal) {
                if (errorMessage) *errorMessage = QStringLiteral("Locked target for axis %1 is outside its limits").arg(it.key());
                return false;
            }
        }
    }
    return !supportedMachiningModes().isEmpty();
}

bool MachineConfigurationService::validateCandidateConfiguration(
    const QString& presetName,
    const QList<MachineAxisDef>& axes,
    const HeadToolGeometry& headGeometry,
    QString* errorMessage) const
{
    MachineConfigurationService candidate;
    candidate.m_presetName = presetName.trimmed().isEmpty()
        ? QStringLiteral("VERTICAL_AC_TABLE") : presetName.trimmed();
    candidate.m_axisConfigs = mergedAxisConfigurations(axes);
    candidate.m_headToolGeometry = headGeometry;
    candidate.m_workpieceSetup = m_workpieceSetup;
    candidate.m_lockedTargetOverrides = candidate.m_presetName.compare(
        m_presetName, Qt::CaseInsensitive) == 0 ? m_lockedTargetOverrides
                                                : QHash<MachiningMode, QMap<QString, double>>{};
    return candidate.validateConfiguration(errorMessage);
}

QString MachineConfigurationService::configurationFingerprint() const
{
    QByteArray payload = m_presetName.toUtf8();
    payload += "|workpieceSetup|";
    payload += QByteArray::number(m_workpieceSetup.x, 'g', 17);
    payload += '|';
    payload += QByteArray::number(m_workpieceSetup.y, 'g', 17);
    payload += '|';
    payload += QByteArray::number(m_workpieceSetup.z, 'g', 17);
    payload += '|';
    payload += QByteArray::number(m_workpieceSetup.rotationXDeg, 'g', 17);
    payload += '|';
    payload += QByteArray::number(m_workpieceSetup.rotationYDeg, 'g', 17);
    payload += '|';
    payload += QByteArray::number(m_workpieceSetup.rotationZDeg, 'g', 17);
    for (const MachineAxisRuntimeConfig& config : m_axisConfigs) {
        const MachineAxisDef& axis = config.axis;
        payload += '|';
        payload += axis.name.toUtf8();
        payload += QByteArray::number(static_cast<int>(axis.motionType));
        payload += machineAxisRoleName(axis.role).toUtf8();
        payload += axis.parentAxis.toUtf8();
        payload += QByteArray::number(axis.direction.X(), 'g', 17);
        payload += QByteArray::number(axis.direction.Y(), 'g', 17);
        payload += QByteArray::number(axis.direction.Z(), 'g', 17);
        payload += QByteArray::number(axis.origin.X(), 'g', 17);
        payload += QByteArray::number(axis.origin.Y(), 'g', 17);
        payload += QByteArray::number(axis.origin.Z(), 'g', 17);
        payload += QByteArray::number(axis.minVal, 'g', 17);
        payload += QByteArray::number(axis.maxVal, 'g', 17);
        payload += QByteArray::number(config.controllerIndex);
        payload += QByteArray::number(config.homeIndex);
        payload += QByteArray::number(config.resolution, 'g', 17);
        payload += QByteArray::number(config.motionSpeed, 'g', 17);
        payload += QByteArray::number(config.acceleration, 'g', 17);
        payload += QByteArray::number(config.jerk, 'g', 17);
        payload += QByteArray::number(config.pipeDiameter, 'g', 17);
    }
    payload += QByteArray::number(m_headToolGeometry.zeroBeamX, 'g', 17);
    payload += QByteArray::number(m_headToolGeometry.zeroBeamY, 'g', 17);
    payload += QByteArray::number(m_headToolGeometry.zeroBeamZ, 'g', 17);
    payload += QByteArray::number(m_headToolGeometry.focusLength, 'g', 17);
    payload += QByteArray::number(m_headToolGeometry.installationOffsetX, 'g', 17);
    payload += QByteArray::number(m_headToolGeometry.installationOffsetY, 'g', 17);
    payload += QByteArray::number(m_headToolGeometry.installationOffsetZ, 'g', 17);
    return QString::fromLatin1(QCryptographicHash::hash(payload, QCryptographicHash::Sha256).toHex());
}

void MachineConfigurationService::setHeadToolGeometry(const HeadToolGeometry& geometry)
{
    m_headToolGeometry = geometry;
    notifyChanged();
}

void MachineConfigurationService::syncFromKinematics(const MachineKinematics* kinematics)
{
    if (!kinematics)
        return;

    const QString nextPreset = kinematics->configType().trimmed().isEmpty()
        ? m_presetName
        : kinematics->configType().trimmed();
    setMachineAxisDefinitions(nextPreset, kinematics->axes());
}

QVector<MachineAxisRuntimeConfig> MachineConfigurationService::defaultAxisConfigsForPreset(const QString& presetName)
{
    MachineKinematics kinematics;
    kinematics.loadPreset(presetName);

    // 控制器编号按"标准 CNC 习惯"分配：
    //   X=0, Y=1, Z=2, 然后旋转轴：A/B → 3，C → 4。
    // MachineKinematics::loadPreset 出于"动力学父子链路"考虑会把先转的轴前置（例如
    // VERTICAL_AC_TABLE 里链路是 BASE→Y→X→Z，Y 先于 X），但物理控制器序号与运动学
    // 父子无关，不能按枚举顺序简单赋 0/1/2 否则会出现 X↔Y 互换。
    auto controllerIndexFor = [](const QString& axisName) -> int {
        const QString n = axisName.trimmed().toUpper();
        if (n == QStringLiteral("X")) return 0;
        if (n == QStringLiteral("Y")) return 1;
        if (n == QStringLiteral("Z")) return 2;
        if (n == QStringLiteral("A") || n == QStringLiteral("B")) return 3;
        if (n == QStringLiteral("C")) return 4;
        return -1;
    };

    QVector<MachineAxisRuntimeConfig> configs;
    int fallbackIndex = 0;
    for (const MachineAxisDef& axis : kinematics.axes()) {
        if (axis.name == QStringLiteral("BASE"))
            continue;
        int idx = controllerIndexFor(axis.name);
        if (idx < 0)
            idx = fallbackIndex;  // 非常规轴名兜底
        ++fallbackIndex;
        configs.append(defaultRuntimeConfig(axis, idx));
    }
    return configs;
}

MachineAxisRuntimeConfig MachineConfigurationService::defaultRuntimeConfig(const MachineAxisDef& axis,
                                                                          int index)
{
    MachineAxisRuntimeConfig config;
    config.axis = axis;
    config.controllerIndex = index;
    config.homeIndex = index;
    config.resolution = 2000.0;
    config.motionSpeed = 10.0;
    config.acceleration = 200.0;
    config.jerk = kDefaultAxisJerk;
    if (axis.motionType == MachineAxisDef::Rotary) {
        config.lowSpeed = 1.0;
        config.mediumSpeed = 10.0;
        config.highSpeed = 30.0;
    } else {
        config.lowSpeed = 1.0;
        config.mediumSpeed = 10.0;
        config.highSpeed = 50.0;
    }
    return config;
}

MachineAxisDef MachineConfigurationService::defaultAxisDefinition(const QString& name,
                                                                  MachineAxisDef::MotionType motionType,
                                                                  const QString& parentAxis)
{
    MachineAxisDef axis;
    axis.name = name.trimmed().toUpper();
    axis.motionType = motionType;
    axis.parentAxis = parentAxis.trimmed().isEmpty() ? QStringLiteral("BASE") : parentAxis.trimmed().toUpper();
    if (motionType == MachineAxisDef::Rotary) {
        if (axis.name == QStringLiteral("A"))
            axis.direction = gp_Dir(1, 0, 0);
        else if (axis.name == QStringLiteral("B"))
            axis.direction = gp_Dir(0, 1, 0);
        else
            axis.direction = gp_Dir(0, 0, 1);
        axis.minVal = -360.0;
        axis.maxVal = 360.0;
    } else {
        if (axis.name == QStringLiteral("X"))
            axis.direction = gp_Dir(1, 0, 0);
        else if (axis.name == QStringLiteral("Y"))
            axis.direction = gp_Dir(0, 1, 0);
        else
            axis.direction = gp_Dir(0, 0, 1);
        axis.minVal = -300.0;
        axis.maxVal = 300.0;
    }
    return axis;
}

MachineAxisRole MachineConfigurationService::defaultRoleForAxis(
    const QString& presetName,
    const QString& axisName,
    MachineAxisDef::MotionType motionType)
{
    const QString preset = presetName.trimmed().toUpper();
    const QString name = axisName.trimmed().toUpper();
    if (motionType == MachineAxisDef::Linear) {
        if (name == QStringLiteral("X")) return MachineAxisRole::LinearX;
        if (name == QStringLiteral("Y")) return MachineAxisRole::LinearY;
        if (name == QStringLiteral("Z")) return MachineAxisRole::LinearZ;
        return MachineAxisRole::Unspecified;
    }
    if (preset == QStringLiteral("XYZA")) return MachineAxisRole::WorkpieceRotary;
    if (preset.contains(QStringLiteral("HEAD"))) {
        return name == QStringLiteral("A")
            ? MachineAxisRole::HeadTiltPrimary
            : MachineAxisRole::HeadTiltSecondary;
    }
    if (name == QStringLiteral("C")) return MachineAxisRole::TableSpin;
    return MachineAxisRole::TableTilt;
}

void MachineConfigurationService::setPresetDefaults(const QString& presetName)
{
    m_presetName = presetName.trimmed().isEmpty() ? QStringLiteral("VERTICAL_AC_TABLE") : presetName.trimmed();
    m_axisConfigs = defaultAxisConfigsForPreset(m_presetName);
    m_lockedTargetOverrides.clear();
    if (m_presetName.compare(QStringLiteral("XYZA"), Qt::CaseInsensitive) == 0)
        m_configuredDefaultMode = MachiningMode::RotaryTube4Axis;
    else if (m_presetName.contains(QStringLiteral("TABLE"), Qt::CaseInsensitive))
        m_configuredDefaultMode = MachiningMode::SimultaneousTable5Axis;
    else
        m_configuredDefaultMode = MachiningMode::Planar3Axis;
}

void MachineConfigurationService::notifyChanged()
{
    LCNC_INFO(LogCode::Generic,
              "machine.config: preset='{}' axes={} algorithm='{}'",
              m_presetName.toStdString(),
              m_axisConfigs.size(),
              toolpathAlgorithmText().toStdString());
    saveDefault();
    emit machineConfigurationChanged();
}

void MachineConfigurationService::readFrom(const toml::value& root)
{
    using namespace lcnc::toml_io;
    m_presetName = get_qstring(root, "machinePreset", QStringLiteral("VERTICAL_AC_TABLE"));
    setPresetDefaults(m_presetName);

    if (root.is_table() && root.contains("headTcp") && root.at("headTcp").is_table()) {
        const toml::value& tcp = root.at("headTcp");
        m_headToolGeometry.zeroBeamX = get_double(tcp, "zeroBeamX", 0.0);
        m_headToolGeometry.zeroBeamY = get_double(tcp, "zeroBeamY", 0.0);
        m_headToolGeometry.zeroBeamZ = get_double(tcp, "zeroBeamZ", -1.0);
        m_headToolGeometry.focusLength = get_double(tcp, "focusLength", 0.0);
        m_headToolGeometry.installationOffsetX = get_double(tcp, "installationOffsetX", 0.0);
        m_headToolGeometry.installationOffsetY = get_double(tcp, "installationOffsetY", 0.0);
        m_headToolGeometry.installationOffsetZ = get_double(tcp, "installationOffsetZ", 0.0);
    }
    if (root.is_table() && root.contains("workpieceSetup") && root.at("workpieceSetup").is_table()) {
        const toml::value& setup = root.at("workpieceSetup");
        m_workpieceSetup.x = get_double(setup, "x", 0.0);
        m_workpieceSetup.y = get_double(setup, "y", 0.0);
        m_workpieceSetup.z = get_double(setup, "z", 0.0);
        m_workpieceSetup.rotationXDeg = get_double(setup, "rotationXDeg", 0.0);
        m_workpieceSetup.rotationYDeg = get_double(setup, "rotationYDeg", 0.0);
        m_workpieceSetup.rotationZDeg = get_double(setup, "rotationZDeg", 0.0);
    }
    if (root.is_table()) {
        m_configuredDefaultMode = machiningModeFromName(
            get_qstring(root, "defaultMachiningMode", machiningModeName(m_configuredDefaultMode)),
            m_configuredDefaultMode);
        if (root.contains("modeDefinitions") && root.at("modeDefinitions").is_array()) {
            for (const toml::value& modeValue : root.at("modeDefinitions").as_array()) {
                if (!modeValue.is_table() || !modeValue.contains("mode")) continue;
                const MachiningMode mode = machiningModeFromName(get_qstring(modeValue, "mode", QString()));
                QMap<QString, double> targets;
                if (modeValue.contains("lockedAxes") && modeValue.at("lockedAxes").is_array()) {
                    for (const toml::value& target : modeValue.at("lockedAxes").as_array()) {
                        if (!target.is_table()) continue;
                        const QString name = get_qstring(target, "name", QString()).trimmed().toUpper();
                        if (!name.isEmpty()) targets.insert(name, get_double(target, "target", 0.0));
                    }
                }
                m_lockedTargetOverrides.insert(mode, targets);
            }
        }
    }

    if (!root.is_table() || !root.contains("axes") || !root.at("axes").is_array())
        return;

    QVector<MachineAxisRuntimeConfig> loaded;
    for (const toml::value& item : root.at("axes").as_array()) {
        if (!item.is_table())
            continue;
        const QString name = get_qstring(item, "name", QString()).trimmed().toUpper();
        if (name.isEmpty() || name == QStringLiteral("BASE"))
            continue;
        const auto motionType = motionTypeFromName(get_qstring(item, "motionType", QStringLiteral("Linear")));
        MachineAxisRuntimeConfig config;
        config.axis = defaultAxisDefinition(name, motionType, get_qstring(item, "parent", QStringLiteral("BASE")));
        config.axis.role = machineAxisRoleFromName(
            get_qstring(item, "role", machineAxisRoleName(defaultRoleForAxis(m_presetName, name, motionType))));
        config.axis.minVal = get_double(item, "min", config.axis.minVal);
        config.axis.maxVal = get_double(item, "max", config.axis.maxVal);
        config.controllerIndex = get_int(item, "controllerIndex", loaded.size());
        config.homeIndex = get_int(item, "homeIndex", config.controllerIndex);
        config.resolution = get_double(item, "resolution", config.resolution);
        config.motionSpeed = get_double(item, "motionSpeed", config.motionSpeed);
        config.lowSpeed = get_double(item, "lowSpeed", config.lowSpeed);
        config.mediumSpeed = get_double(item, "mediumSpeed", config.mediumSpeed);
        config.highSpeed = get_double(item, "highSpeed", config.highSpeed);
        config.acceleration = get_double(item, "acceleration", config.acceleration);
        config.jerk = get_double(item, "jerk", config.jerk);
        config.pipeDiameter = get_double(item, "pipeDiameter", config.pipeDiameter);
        sanitizeAxisMotionParameters(config);
        config.axis.direction = gp_Dir(get_double(item, "directionX", config.axis.direction.X()),
                           get_double(item, "directionY", config.axis.direction.Y()),
                           get_double(item, "directionZ", config.axis.direction.Z()));
        config.axis.origin = gp_Pnt(get_double(item, "originX", config.axis.origin.X()),
                        get_double(item, "originY", config.axis.origin.Y()),
                        get_double(item, "originZ", config.axis.origin.Z()));
        loaded.append(config);
    }
    if (!loaded.isEmpty())
        // XYZA is a continuous workpiece rotary machine.  Older machine.toml
        // files were written while its A axis inherited an AC-table tilt
        // limit, so upgrade that persisted axis semantics on every load.
        // 中文翻译：XYZA 的 A 为连续工件回转轴；旧配置曾错误继承 AC 转台倾斜限位，加载时统一升级。
        for (MachineAxisRuntimeConfig& config : loaded) {
            if (m_presetName.compare(QStringLiteral("XYZA"), Qt::CaseInsensitive) == 0
                && config.axis.role == MachineAxisRole::WorkpieceRotary
                && config.axis.motionType == MachineAxisDef::Rotary) {
                config.axis.minVal = -kContinuousRotaryPlanningLimitDeg;
                config.axis.maxVal = kContinuousRotaryPlanningLimitDeg;
            }
        }
    if (!loaded.isEmpty())
        m_axisConfigs = loaded;
}

void MachineConfigurationService::writeTo(toml::value& root) const
{
    using namespace lcnc::toml_io;
    root["machinePreset"] = qs(m_presetName);
    root["toolpathAlgorithm"] = qs(toolpathAlgorithmText());
    root["defaultMachiningMode"] = qs(machiningModeName(defaultMachiningMode()));
    toml::value workpieceSetup(toml::table{});
    workpieceSetup["x"] = m_workpieceSetup.x;
    workpieceSetup["y"] = m_workpieceSetup.y;
    workpieceSetup["z"] = m_workpieceSetup.z;
    workpieceSetup["rotationXDeg"] = m_workpieceSetup.rotationXDeg;
    workpieceSetup["rotationYDeg"] = m_workpieceSetup.rotationYDeg;
    workpieceSetup["rotationZDeg"] = m_workpieceSetup.rotationZDeg;
    root["workpieceSetup"] = workpieceSetup;

    toml::array axes;
    for (const MachineAxisRuntimeConfig& config : m_axisConfigs) {
        toml::value item(toml::table{});
        item["name"] = qs(config.axis.name);
        item["motionType"] = qs(motionTypeName(config.axis.motionType));
        item["role"] = qs(machineAxisRoleName(config.axis.role));
        item["parent"] = qs(config.axis.parentAxis);
        item["directionX"] = config.axis.direction.X();
        item["directionY"] = config.axis.direction.Y();
        item["directionZ"] = config.axis.direction.Z();
        item["originX"] = config.axis.origin.X();
        item["originY"] = config.axis.origin.Y();
        item["originZ"] = config.axis.origin.Z();
        item["controllerIndex"] = config.controllerIndex;
        item["homeIndex"] = config.homeIndex;
        item["resolution"] = config.resolution;
        item["motionSpeed"] = config.motionSpeed;
        item["min"] = config.axis.minVal;
        item["max"] = config.axis.maxVal;
        item["lowSpeed"] = config.lowSpeed;
        item["mediumSpeed"] = config.mediumSpeed;
        item["highSpeed"] = config.highSpeed;
        item["acceleration"] = config.acceleration;
        item["jerk"] = config.jerk;
        item["pipeDiameter"] = config.pipeDiameter;
        axes.emplace_back(item);
    }
    root["axes"] = axes;
    toml::array modeDefinitions;
    for (MachiningMode mode : supportedMachiningModes()) {
        const MachineModeDefinition definition = modeDefinition(mode);
        toml::value modeValue(toml::table{});
        modeValue["mode"] = qs(machiningModeName(mode));
        modeValue["solverId"] = qs(definition.solverId);
        modeValue["solverVersion"] = definition.solverVersion;
        toml::array lockedAxes;
        for (auto it = definition.lockedAxisTargets.cbegin();
             it != definition.lockedAxisTargets.cend(); ++it) {
            toml::value target(toml::table{});
            target["name"] = qs(it.key());
            target["target"] = it.value();
            lockedAxes.push_back(target);
        }
        modeValue["lockedAxes"] = lockedAxes;
        modeDefinitions.push_back(modeValue);
    }
    root["modeDefinitions"] = modeDefinitions;
    toml::value headTcp(toml::table{});
    headTcp["zeroBeamX"] = m_headToolGeometry.zeroBeamX;
    headTcp["zeroBeamY"] = m_headToolGeometry.zeroBeamY;
    headTcp["zeroBeamZ"] = m_headToolGeometry.zeroBeamZ;
    headTcp["focusLength"] = m_headToolGeometry.focusLength;
    headTcp["installationOffsetX"] = m_headToolGeometry.installationOffsetX;
    headTcp["installationOffsetY"] = m_headToolGeometry.installationOffsetY;
    headTcp["installationOffsetZ"] = m_headToolGeometry.installationOffsetZ;
    root["headTcp"] = headTcp;
}

} // namespace lcnc
