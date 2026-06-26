#include "core/kinematics/machine_configuration_service.h"

#include "core/logging/logger.h"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>

#include <algorithm>
#include <cmath>

namespace lcnc {
namespace {

QString motionTypeName(MachineAxisDef::MotionType type)
{
    return type == MachineAxisDef::Rotary ? QStringLiteral("Rotary") : QStringLiteral("Linear");
}

MachineAxisDef::MotionType motionTypeFromName(const QString& name)
{
    const QString normalized = name.trimmed().toLower();
    return normalized == QStringLiteral("rotary") || normalized == QStringLiteral("旋转")
        ? MachineAxisDef::Rotary
        : MachineAxisDef::Linear;
}

bool sameAxisConfig(const MachineAxisRuntimeConfig& lhs, const MachineAxisRuntimeConfig& rhs)
{
    return lhs.axis.name == rhs.axis.name
        && lhs.axis.motionType == rhs.axis.motionType
        && lhs.axis.parentAxis == rhs.axis.parentAxis
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
        && std::abs(lhs.lowSpeed - rhs.lowSpeed) < 1e-9
        && std::abs(lhs.mediumSpeed - rhs.mediumSpeed) < 1e-9
        && std::abs(lhs.highSpeed - rhs.highSpeed) < 1e-9
        && std::abs(lhs.acceleration - rhs.acceleration) < 1e-9
        && std::abs(lhs.jerk - rhs.jerk) < 1e-9;
}

void copyHardwareConfig(MachineAxisRuntimeConfig& target, const MachineAxisRuntimeConfig& source)
{
    target.controllerIndex = source.controllerIndex;
    target.homeIndex = source.homeIndex;
    target.axis.minVal = source.axis.minVal;
    target.axis.maxVal = source.axis.maxVal;
    target.lowSpeed = source.lowSpeed;
    target.mediumSpeed = source.mediumSpeed;
    target.highSpeed = source.highSpeed;
    target.acceleration = source.acceleration;
    target.jerk = source.jerk;
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
        config.axis.name = axis.name;
        config.axis.motionType = axis.motionType;
        config.axis.direction = axis.direction;
        config.axis.origin = axis.origin;
        config.axis.parentAxis = axis.parentAxis.isEmpty() ? QStringLiteral("BASE") : axis.parentAxis;
        nextConfigs.append(config);
        ++index;
    }

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
    int rotaryCount = 0;
    for (const MachineAxisRuntimeConfig& config : m_axisConfigs) {
        if (config.axis.motionType == MachineAxisDef::Rotary)
            ++rotaryCount;
    }
    if (rotaryCount <= 0)
        return MachineToolpathAlgorithm::ThreeAxis;
    const QString preset = m_presetName.toUpper();
    if (preset.contains(QStringLiteral("HEAD")))
        return MachineToolpathAlgorithm::FiveAxisHead;
    return MachineToolpathAlgorithm::FiveAxisTable;
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
    config.acceleration = 200.0;
    config.jerk = 0.0;
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

void MachineConfigurationService::setPresetDefaults(const QString& presetName)
{
    m_presetName = presetName.trimmed().isEmpty() ? QStringLiteral("VERTICAL_AC_TABLE") : presetName.trimmed();
    m_axisConfigs = defaultAxisConfigsForPreset(m_presetName);
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
        config.axis.minVal = get_double(item, "min", config.axis.minVal);
        config.axis.maxVal = get_double(item, "max", config.axis.maxVal);
        config.controllerIndex = get_int(item, "controllerIndex", loaded.size());
        config.homeIndex = get_int(item, "homeIndex", config.controllerIndex);
        config.lowSpeed = get_double(item, "lowSpeed", config.lowSpeed);
        config.mediumSpeed = get_double(item, "mediumSpeed", config.mediumSpeed);
        config.highSpeed = get_double(item, "highSpeed", config.highSpeed);
        config.acceleration = get_double(item, "acceleration", config.acceleration);
        config.jerk = get_double(item, "jerk", config.jerk);
        config.axis.direction = gp_Dir(get_double(item, "directionX", config.axis.direction.X()),
                           get_double(item, "directionY", config.axis.direction.Y()),
                           get_double(item, "directionZ", config.axis.direction.Z()));
        config.axis.origin = gp_Pnt(get_double(item, "originX", config.axis.origin.X()),
                        get_double(item, "originY", config.axis.origin.Y()),
                        get_double(item, "originZ", config.axis.origin.Z()));
        loaded.append(config);
    }
    if (!loaded.isEmpty())
        m_axisConfigs = loaded;
}

void MachineConfigurationService::writeTo(toml::value& root) const
{
    using namespace lcnc::toml_io;
    root["machinePreset"] = qs(m_presetName);
    root["toolpathAlgorithm"] = qs(toolpathAlgorithmText());

    toml::array axes;
    for (const MachineAxisRuntimeConfig& config : m_axisConfigs) {
        toml::value item(toml::table{});
        item["name"] = qs(config.axis.name);
        item["motionType"] = qs(motionTypeName(config.axis.motionType));
        item["parent"] = qs(config.axis.parentAxis);
        item["directionX"] = config.axis.direction.X();
        item["directionY"] = config.axis.direction.Y();
        item["directionZ"] = config.axis.direction.Z();
        item["originX"] = config.axis.origin.X();
        item["originY"] = config.axis.origin.Y();
        item["originZ"] = config.axis.origin.Z();
        item["controllerIndex"] = config.controllerIndex;
        item["homeIndex"] = config.homeIndex;
        item["min"] = config.axis.minVal;
        item["max"] = config.axis.maxVal;
        item["lowSpeed"] = config.lowSpeed;
        item["mediumSpeed"] = config.mediumSpeed;
        item["highSpeed"] = config.highSpeed;
        item["acceleration"] = config.acceleration;
        item["jerk"] = config.jerk;
        axes.emplace_back(item);
    }
    root["axes"] = axes;
}

} // namespace lcnc