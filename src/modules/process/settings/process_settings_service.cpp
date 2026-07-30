#include "modules/process/settings/process_settings_service.h"

#include "core/kernel/kernel.h"
#include "core/logging/logger.h"
#include "modules/process/Setting/BuiltinIODefs.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QSaveFile>
#include <QObject>
#include <QUuid>

#include <algorithm>

namespace lcnc::process {
using toml::table;
using toml::value;
namespace {

QString ioBucketName(ProcessIoBucket bucket)
{
    switch (bucket) {
    case ProcessIoBucket::DigitalInput: return QStringLiteral("DigitalIN");
    case ProcessIoBucket::DigitalOutput: return QStringLiteral("DigitalOUT");
    case ProcessIoBucket::AnalogInput: return QStringLiteral("AnalogIN");
    case ProcessIoBucket::AnalogOutput: return QStringLiteral("AnalogOUT");
    }
    return {};
}

bool isDigital(ProcessIoBucket bucket)
{
    return bucket == ProcessIoBucket::DigitalInput || bucket == ProcessIoBucket::DigitalOutput;
}

bool sameToml(const toml::value& lhs, const toml::value& rhs)
{
    return toml::format(lhs) == toml::format(rhs);
}

table& ensureTable(toml::value& node)
{
    if (!node.is_table()) node = table{};
    return node.as_table();
}

table& ensureChildTable(table& parent, const std::string& key)
{
    return ensureTable(parent[key]);
}

QVariant variantFromToml(const toml::value& value, const QVariant& fallback)
{
    if (value.is_boolean()) return value.as_boolean();
    if (value.is_integer()) return static_cast<qlonglong>(value.as_integer());
    if (value.is_floating()) return value.as_floating();
    if (value.is_string()) return QString::fromStdString(value.as_string());
    return fallback;
}

toml::value tomlFromVariant(const QVariant& value, ParameterValueType type)
{
    if (type == ParameterValueType::Bool) return value.toBool();
    if (type == ParameterValueType::Int) return value.toInt();
    if (type == ParameterValueType::Double) return value.toDouble();
    return value.toString().toStdString();
}

} // namespace

ProcessSettingsService::ProcessSettingsService()
    : m_registry(*this)
{
}

QString ProcessSettingsService::rootDir() const
{
    return QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("config/process"));
}

QString ProcessSettingsService::sectionName(ProcessConfigArea area, const QString& tableName) const
{
    switch (area) {
    case ProcessConfigArea::Devices:
        if (tableName == QStringLiteral("MotionControl")) return QStringLiteral("MotionControl");
        if (tableName.startsWith(QStringLiteral("Camera"))) return QStringLiteral("Camera");
        if (tableName == QStringLiteral("Internet")) return QStringLiteral("Internet");
        return QStringLiteral("Laser");
    case ProcessConfigArea::DigitalIo: return QStringLiteral("Digital");
    case ProcessConfigArea::AnalogIo: return QStringLiteral("Analog");
    case ProcessConfigArea::Operations:
        if (tableName.startsWith(QStringLiteral("Water")) || tableName == QStringLiteral("Pump")) return QStringLiteral("Water");
        if (tableName == QStringLiteral("Cutting")) return QStringLiteral("Monitor");
        if (tableName == QStringLiteral("LoadingPos") || tableName == QStringLiteral("BlankingPos")) return QStringLiteral("LoadingPos");
        return QStringLiteral("Gas");
    case ProcessConfigArea::Workflow: return QStringLiteral("Special");
    case ProcessConfigArea::Tools: return QStringLiteral("Tool");
    }
    return {};
}

toml::table& ProcessSettingsService::sectionRef(ProcessConfigArea area, const QString& tableName)
{
    table& root = ensureTable(m_draft);
    table& settings = ensureChildTable(root, "Setting");
    return ensureChildTable(settings, sectionName(area, tableName).toStdString());
}

const toml::table& ProcessSettingsService::sectionRef(ProcessConfigArea area, const QString& tableName) const
{
    static const toml::table kEmpty;
    if (!m_draft.is_table() || !m_draft.contains("Setting") || !m_draft.at("Setting").is_table()) return kEmpty;
    const auto& root = m_draft.at("Setting").as_table();
    const auto it = root.find(sectionName(area, tableName).toStdString());
    return it != root.end() && it->second.is_table() ? it->second.as_table() : kEmpty;
}

bool ProcessSettingsService::writeTomlAtomically(const QString& filePath, const toml::value& value, QString* error) const
{
    QDir().mkpath(QFileInfo(filePath).absolutePath());
    QSaveFile file(filePath);
    // 中文翻译：无法写入 %1: %2
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) { if (error) *error = QObject::tr("Unable to write to %1: %2").arg(filePath, file.errorString()); return false; }
    const QByteArray content = QByteArray::fromStdString(toml::format(value));
    // 中文翻译：无法提交 %1: %2
    if (file.write(content) != content.size() || !file.commit()) { if (error) *error = QObject::tr("Unable to submit %1: %2").arg(filePath, file.errorString()); return false; }
    return true;
}

void ProcessSettingsService::seedBuiltinIo()
{
    const auto seed = [this](ProcessIoBucket bucket, const BuiltinIODefList& defs) {
        table& section = sectionRef(isDigital(bucket) ? ProcessConfigArea::DigitalIo : ProcessConfigArea::AnalogIo);
        table& channels = ensureChildTable(section, ioBucketName(bucket).toStdString());
        for (int i = 0; i < defs.count; ++i) {
            const BuiltinIODef& def = defs.items[i];
            if (channels.count(def.tomlKey)) continue;
            table row;
            row["name"] = std::string(def.nameZh);
            row["index"] = std::string(def.defaultIndex);
            row["active"] = def.defaultActive;
            row["enabled"] = def.defaultEnabled;
            row["showInMain"] = def.defaultShowInMain;
            row["builtin"] = true;
            channels[def.tomlKey] = row;
        }
    };
    seed(ProcessIoBucket::DigitalInput, builtinDigitalIN());
    seed(ProcessIoBucket::DigitalOutput, builtinDigitalOUT());
    seed(ProcessIoBucket::AnalogInput, builtinAnalogIN());
    seed(ProcessIoBucket::AnalogOutput, builtinAnalogOUT());
}

void ProcessSettingsService::seedDefaults()
{
    m_draft = toml::value(toml::table{});
#if defined(LCNC_PROCESS_HAS_ACS) && LCNC_PROCESS_HAS_ACS
    constexpr const char* defaultController = "SimulatorCMHP";
#elif defined(LCNC_PROCESS_HAS_GTN) && LCNC_PROCESS_HAS_GTN
    constexpr const char* defaultController = "GTN";
#else
    constexpr const char* defaultController = "Simulator";
#endif
    ensureChildTable(sectionRef(ProcessConfigArea::Devices, QStringLiteral("MotionControl")), "MotionControl")["sType"] = defaultController;
    table& laser = ensureChildTable(sectionRef(ProcessConfigArea::Devices, QStringLiteral("Laser")), "Laser");
    laser["sType"] = "Simulator";
    laser["fResolution"] = 0.0;
    ensureChildTable(sectionRef(ProcessConfigArea::Operations, QStringLiteral("Gas")), "Gas")["bBlow"] = false;
    ensureChildTable(sectionRef(ProcessConfigArea::Operations, QStringLiteral("Water")), "Water")["bWater"] = false;
    table& toolSection = sectionRef(ProcessConfigArea::Tools);
    table& toolIndex = ensureChildTable(toolSection, "ToolIndex");
    toolIndex["sToolIndex"] = "Default";
    toolIndex["sTool_0"] = "Default";
    table tool;
    tool["fLineVel"] = 10.0; tool["fCutAcc"] = 100.0; tool["fCutJerk"] = 1000.0;
    tool["fIdelAcc"] = 100.0; tool["fIdelJerk"] = 1000.0;
    tool["fCuttingHeight"] = 0.0; tool["fIdleHeight"] = 0.0;
    tool["fEnergy"] = 20.0; tool["fFrequency"] = 30.0; tool["fPluse"] = 20.0;
    tool["fBeforeOpenLaser"] = 0.0; tool["fAfterCloseLaser"] = 0.0;
    toolSection["Default"] = tool;
    // Keep every domain structurally valid even when it has no fields yet.
    sectionRef(ProcessConfigArea::Devices, QStringLiteral("Internet"));
    sectionRef(ProcessConfigArea::Devices, QStringLiteral("Camera"));
    sectionRef(ProcessConfigArea::Operations, QStringLiteral("Cutting"));
    sectionRef(ProcessConfigArea::Operations, QStringLiteral("LoadingPos"));
    sectionRef(ProcessConfigArea::Workflow, QStringLiteral("Special"));
    seedBuiltinIo();
}

bool ProcessSettingsService::loadDomain(const QString& fileName, QString* error)
{
    const QString path = QDir(rootDir()).filePath(fileName);
    if (!QFileInfo::exists(path)) return true;
    try {
        const toml::value parsed = toml::parse(path.toStdString());
        if (!parsed.is_table() || !parsed.contains("Setting") || !parsed.at("Setting").is_table()) {
            // 中文翻译：配置文件 %1 的 schema 无效。
            if (error) *error = QObject::tr("The schema for configuration file %1 is invalid.").arg(path);
            return false;
        }
        const auto& parsedSettings = parsed.at("Setting").as_table();
        for (const auto& item : parsedSettings) {
            if (!item.second.is_table()) {
                // 中文翻译：配置文件 %1 中的域 %2 不是表。
                if (error) *error = QObject::tr("Field %2 in configuration file %1 is not a table.").arg(path, QString::fromStdString(item.first));
                return false;
            }
        }
        // Commit only after the entire file has passed schema validation.
        for (const auto& item : parsedSettings) {
            m_draft["Setting"][item.first] = item.second;
        }
        return true;
    } catch (const std::exception& ex) {
        LCNC_ERR(lcnc::LogCode::SettingsParseFailed,
                 "process.settings: failed to read '{}': {}",
                 path.toStdString(), ex.what());
        // 中文翻译：无法读取 %1: %2
        if (error) *error = QObject::tr("Unable to read %1: %2").arg(path, QString::fromLocal8Bit(ex.what()));
        return false;
    }
}

bool ProcessSettingsService::initialize()
{
    seedDefaults();
    QDir().mkpath(QDir(rootDir()).filePath(QStringLiteral("tools")));
    QString error;
    const auto backupInvalidFile = [this](const QString& fileName, QString* backupError) {
        const QString path = QDir(rootDir()).filePath(fileName);
        const QString stamp = QDateTime::currentDateTimeUtc().toString(QStringLiteral("yyyyMMdd-HHmmsszzz"));
        const QString backup = path + QStringLiteral(".invalid-") + stamp;
        if (!QFile::copy(path, backup)) {
            if (backupError)
                // 中文翻译：无法备份损坏配置 %1 到 %2。
                *backupError = QObject::tr("Unable to back up corrupted configurations %1 to %2.").arg(path, backup);
            return false;
        }
        LCNC_WARN(lcnc::LogCode::SettingsParseFailed,
                  "process.settings: invalid '{}' preserved as '{}'; rebuilding defaults",
                  path.toStdString(), backup.toStdString());
        return true;
    };
    const auto loadOrRepairDomain = [this, &error, &backupInvalidFile](
                                        const QString& fileName,
                                        const QStringList& sections) {
        const QString path = QDir(rootDir()).filePath(fileName);
        if (!QFileInfo::exists(path))
            return writeDomain(fileName, sections, &error);
        if (loadDomain(fileName, &error))
            return true;
        if (!backupInvalidFile(fileName, &error))
            return false;
        error.clear();
        return writeDomain(fileName, sections, &error);
    };

    const bool ok = loadOrRepairDomain(
                        QStringLiteral("devices.toml"),
                        {QStringLiteral("MotionControl"), QStringLiteral("Laser"),
                         QStringLiteral("Internet"), QStringLiteral("Camera")})
        && loadOrRepairDomain(
            QStringLiteral("io.toml"),
            {QStringLiteral("Digital"), QStringLiteral("Analog")})
        && loadOrRepairDomain(
            QStringLiteral("operations.toml"),
            {QStringLiteral("Gas"), QStringLiteral("Water"),
             QStringLiteral("Monitor"), QStringLiteral("LoadingPos")})
        && loadOrRepairDomain(
            QStringLiteral("workflow.toml"),
            {QStringLiteral("Special")});
    if (!ok) {
        LCNC_ERR(lcnc::LogCode::SettingsParseFailed,
                 "process.settings: initialization failed: {}",
                 error.toStdString());
        return false;
    }
    const QString indexPath = QDir(rootDir()).filePath(QStringLiteral("tools/index.toml"));
    if (!QFileInfo::exists(indexPath) && !writeTools(&error, nullptr)) {
        LCNC_ERR(lcnc::LogCode::SettingsParseFailed,
                 "process.settings: failed to create tool defaults: {}",
                 error.toStdString());
        return false;
    }
    if (QFileInfo::exists(indexPath)) {
        if (!loadDomain(QStringLiteral("tools/index.toml"), &error)) {
            if (!backupInvalidFile(QStringLiteral("tools/index.toml"), &error)
                || !writeTools(&error, nullptr)) {
                LCNC_ERR(lcnc::LogCode::SettingsParseFailed,
                         "process.settings: failed to repair tool index: {}",
                         error.toStdString());
                return false;
            }
        }
        try {
            const toml::value index = toml::parse(indexPath.toStdString());
            if (index.contains("tools") && index.at("tools").is_array()) {
                for (const auto& item : index.at("tools").as_array()) {
                    if (!item.is_table() || !item.contains("id") || !item.at("id").is_string()
                        || !item.contains("name") || !item.at("name").is_string()) continue;
                    const QString id = QString::fromStdString(item.at("id").as_string());
                    const QString name = QString::fromStdString(item.at("name").as_string());
                    const QString toolPath = QDir(rootDir()).filePath(QStringLiteral("tools/%1.toml").arg(id));
                    if (!QFileInfo::exists(toolPath)) continue;
                    const toml::value tool = toml::parse(toolPath.toStdString());
                    if (!tool.contains("Setting") || !tool.at("Setting").is_table()) continue;
                    const auto& settings = tool.at("Setting").as_table();
                    const auto toolIt = settings.find("Tool");
                    if (toolIt == settings.end() || !toolIt->second.is_table()) continue;
                    const auto named = toolIt->second.as_table().find(name.toStdString());
                    if (named == toolIt->second.as_table().end()) continue;
                    sectionRef(ProcessConfigArea::Tools)[name.toStdString()] = named->second;
                    m_toolIds.insert(name, id);
                }
            }
        } catch (const std::exception& ex) {
            LCNC_ERR(lcnc::LogCode::SettingsParseFailed,
                     "process.settings: tool index load failed: {}",
                     ex.what());
            return false;
        }
    }
    seedBuiltinIo();
    // No legacy file is inspected.  First run persists only the new default store.
    m_committed.value = m_draft;
    beginEdit();
    return true;
}

void ProcessSettingsService::beginEdit()
{
    m_draft = m_committed.value;
    if (auto machine = Kernel::current().services().getService<MachineConfigurationService>()) m_axisDraft = machine->axisConfigurations();
    m_axisDirty = false;
}

void ProcessSettingsService::refreshDraft() { beginEdit(); }
bool ProcessSettingsService::cancelEdit() { beginEdit(); return true; }

QStringList ProcessSettingsService::toolNames() const
{
    QStringList result;
    const table& tools = sectionRef(ProcessConfigArea::Tools);
    const auto indexIt = tools.find("ToolIndex");
    if (indexIt == tools.end() || !indexIt->second.is_table()) return result;
    const table& index = indexIt->second.as_table();
    for (int i = 0;; ++i) {
        const auto it = index.find("sTool_" + std::to_string(i));
        if (it == index.end() || !it->second.is_string()) break;
        const QString name = QString::fromStdString(it->second.as_string()).trimmed();
        if (!name.isEmpty() && !result.contains(name)) result.append(name);
    }
    return result;
}

QStringList ProcessSettingsService::toolDisplayNames() const { return toolNames(); }
QString ProcessSettingsService::toolId(const QString& name) const { if (!m_toolIds.contains(name)) m_toolIds.insert(name, QUuid::createUuid().toString(QUuid::WithoutBraces)); return m_toolIds.value(name); }

QVector<ParameterObjectDescriptor> ProcessSettingsService::objects() const { return m_registry.buildObjects(); }

toml::table ProcessSettingsService::rawTable(ProcessConfigArea area, const QString& tableName) const
{
    const table& section = sectionRef(area, tableName);
    if (tableName.isEmpty()) return section;
    const auto it = section.find(tableName.toStdString());
    return it != section.end() && it->second.is_table() ? it->second.as_table() : table{};
}

QVariant ProcessSettingsService::rawValue(ProcessConfigArea area, const QString& tableName, const QString& key, const QVariant& fallback) const
{
    const table values = rawTable(area, tableName);
    const auto it = values.find(key.toStdString());
    return it == values.end() ? fallback : variantFromToml(it->second, fallback);
}

toml::table ProcessSettingsService::axisRuntimeTable(const QString& axisName) const
{
    for (const auto& axis : m_axisDraft) {
        if (axis.axis.name.compare(axisName, Qt::CaseInsensitive) != 0) continue;
        table result;
        result["iIndex"] = axis.controllerIndex;
        result["iHomeIndex"] = axis.homeIndex;
        result["fResolution"] = axis.resolution;
        result["bRotation"] = axis.axis.motionType == MachineAxisDef::Rotary;
        result["fVel"] = axis.motionSpeed;
        result["fAcc"] = axis.acceleration;
        result["fJerk"] = axis.jerk;
        result["fLeftLimit"] = axis.axis.minVal;
        result["fRightLimit"] = axis.axis.maxVal;
        result["fPipeDiameter"] = axis.pipeDiameter;
        return result;
    }
    return {};
}

QVariant ProcessSettingsService::machineAxisValue(const QString& axisName, const QString& key) const
{
    for (const auto& axis : m_axisDraft) if (axis.axis.name.compare(axisName, Qt::CaseInsensitive) == 0) {
        if (key == "controllerIndex") return axis.controllerIndex; if (key == "homeIndex") return axis.homeIndex;
        if (key == "resolution") return axis.resolution; if (key == "motionSpeed") return axis.motionSpeed;
        if (key == "min") return axis.axis.minVal; if (key == "max") return axis.axis.maxVal;
        if (key == "lowSpeed") return axis.lowSpeed; if (key == "mediumSpeed") return axis.mediumSpeed; if (key == "highSpeed") return axis.highSpeed;
        if (key == "acceleration") return axis.acceleration; if (key == "jerk") return axis.jerk; if (key == "pipeDiameter") return axis.pipeDiameter;
    }
    return {};
}

bool ProcessSettingsService::setMachineAxisValue(const QString& axisName, const QString& key, const QVariant& value, QString* error)
{
    for (auto& axis : m_axisDraft) if (axis.axis.name.compare(axisName, Qt::CaseInsensitive) == 0) {
        if (key == "controllerIndex") axis.controllerIndex = value.toInt(); else if (key == "homeIndex") axis.homeIndex = value.toInt();
        else if (key == "resolution") axis.resolution = value.toDouble(); else if (key == "motionSpeed") axis.motionSpeed = value.toDouble();
        else if (key == "min") axis.axis.minVal = value.toDouble(); else if (key == "max") axis.axis.maxVal = value.toDouble();
        else if (key == "lowSpeed") axis.lowSpeed = value.toDouble(); else if (key == "mediumSpeed") axis.mediumSpeed = value.toDouble(); else if (key == "highSpeed") axis.highSpeed = value.toDouble();
        else if (key == "acceleration") axis.acceleration = value.toDouble(); else if (key == "jerk") axis.jerk = value.toDouble(); else if (key == "pipeDiameter") axis.pipeDiameter = value.toDouble();
        // 中文翻译：未知轴参数：%1
        else { if (error) *error = QObject::tr("Unknown axis parameter: %1").arg(key); return false; }
        // 中文翻译：轴 %1 的负限位不能大于正限位。
        if (axis.axis.minVal > axis.axis.maxVal) { if (error) *error = QObject::tr("The negative limit of axis %1 cannot be greater than the positive limit.").arg(axisName); return false; }
        m_axisDirty = true; return true;
    }
    // 中文翻译：找不到轴：%1
    if (error) *error = QObject::tr("Axis not found: %1").arg(axisName); return false;
}

QVariant ProcessSettingsService::fieldValue(const ParameterDescriptor& field, const QString& objectId) const
{
    if (field.machineAxisField) return machineAxisValue(objectId.section(':', 1), field.key);
    return rawValue(field.area, field.tableName, field.key, field.defaultValue);
}

bool ProcessSettingsService::setFieldValue(const ParameterDescriptor& field, const QString& objectId, const QVariant& value, QString* error)
{
    // 中文翻译：该参数为只读。
    if (field.readOnly) { if (error) *error = QObject::tr("This parameter is read-only."); return false; }
    if (field.machineAxisField) return setMachineAxisValue(objectId.section(':', 1), field.key, value, error);
    // 中文翻译：%1 超出允许范围。
    if ((field.type == ParameterValueType::Int || field.type == ParameterValueType::Double) && (value.toDouble() < field.minimum || value.toDouble() > field.maximum)) { if (error) *error = QObject::tr("%1 is outside the allowed range.").arg(field.title); return false; }
    ensureChildTable(sectionRef(field.area, field.tableName), field.tableName.toStdString())[field.key.toStdString()] = tomlFromVariant(value, field.type);
    return true;
}

QVector<ProcessIoChannel> ProcessSettingsService::ioChannels(ProcessIoBucket bucket) const
{
    QVector<ProcessIoChannel> result;
    const table& section = sectionRef(isDigital(bucket) ? ProcessConfigArea::DigitalIo : ProcessConfigArea::AnalogIo);
    const auto group = section.find(ioBucketName(bucket).toStdString());
    if (group == section.end() || !group->second.is_table()) return result;
    for (const auto& item : group->second.as_table()) if (item.second.is_table()) {
        const table& row = item.second.as_table(); ProcessIoChannel channel; channel.id = QString::fromStdString(item.first.data());
        channel.name = row.count("name") && row.at("name").is_string() ? QString::fromStdString(row.at("name").as_string()) : channel.id;
        channel.hardwareIndex = row.count("index") && row.at("index").is_string() ? QString::fromStdString(row.at("index").as_string()) : QString();
        channel.activeHigh = !row.count("active") || !row.at("active").is_boolean() || row.at("active").as_boolean();
        channel.enabled = !row.count("enabled") || !row.at("enabled").is_boolean() || row.at("enabled").as_boolean();
        channel.showInMain = row.count("showInMain") && row.at("showInMain").is_boolean() && row.at("showInMain").as_boolean();
        channel.builtin = row.count("builtin") && row.at("builtin").is_boolean() && row.at("builtin").as_boolean();
        result.append(channel);
    }
    return result;
}

QStringList ProcessSettingsService::ioDisplayNames(ProcessIoBucket bucket) const
{
    QStringList names;
    for (const auto& channel : ioChannels(bucket))
        if (channel.enabled) names.append(QStringLiteral("%1 (%2)").arg(channel.name, channel.id));
    return names;
}

bool ProcessSettingsService::setIoChannel(ProcessIoBucket bucket, const QString& id, const ProcessIoChannel& channel, QString* error)
{
    // 中文翻译：I/O 名称不能为空。
    if (id.isEmpty() || channel.name.trimmed().isEmpty()) { if (error) *error = QObject::tr("I/O name cannot be empty."); return false; }
    // 中文翻译：同类 I/O 名称不能重复。
    for (const auto& other : ioChannels(bucket)) if (other.id != id && other.name.compare(channel.name.trimmed(), Qt::CaseInsensitive) == 0) { if (error) *error = QObject::tr("I/O names of the same type cannot be repeated."); return false; }
    table row; row["name"] = channel.name.trimmed().toStdString(); row["index"] = channel.hardwareIndex.trimmed().toStdString(); row["enabled"] = channel.enabled; row["builtin"] = channel.builtin;
    if (isDigital(bucket)) row["active"] = channel.activeHigh; if (bucket == ProcessIoBucket::DigitalOutput) row["showInMain"] = channel.showInMain;
    ensureChildTable(sectionRef(isDigital(bucket) ? ProcessConfigArea::DigitalIo : ProcessConfigArea::AnalogIo),
                     ioBucketName(bucket).toStdString())[id.toStdString()] = row;
    return true;
}

bool ProcessSettingsService::addIoChannel(ProcessIoBucket bucket, QString* createdId, QString* error)
{
    // 中文翻译：新%1
    ProcessIoChannel channel; channel.id = QStringLiteral("custom-%1").arg(QUuid::createUuid().toString(QUuid::WithoutBraces)); channel.name = QObject::tr("New %1").arg(ioBucketName(bucket)); channel.hardwareIndex = isDigital(bucket) ? QStringLiteral("0.0") : QStringLiteral("0");
    if (!setIoChannel(bucket, channel.id, channel, error)) return false; if (createdId) *createdId = channel.id; return true;
}

QStringList ProcessSettingsService::ioReferenceLocations(const QString& id) const { Q_UNUSED(id); return {}; }

bool ProcessSettingsService::removeIoChannel(ProcessIoBucket bucket, const QString& id, QString* error)
{
    const auto items = ioChannels(bucket); const auto it = std::find_if(items.cbegin(), items.cend(), [&id](const ProcessIoChannel& item) { return item.id == id; });
    // 中文翻译：I/O 通道不存在。
    if (it == items.cend()) { if (error) *error = QObject::tr("I/O channel does not exist."); return false; }
    // 中文翻译：内置 I/O 通道不能删除，可将其禁用。
    if (it->builtin) { if (error) *error = QObject::tr("Built-in I/O channels cannot be deleted, they can be disabled."); return false; }
    // 中文翻译：I/O 通道正在被引用：%1
    const auto refs = ioReferenceLocations(id); if (!refs.isEmpty()) { if (error) *error = QObject::tr("I/O channel is being referenced: %1").arg(refs.join(QStringLiteral("、"))); return false; }
    ensureChildTable(sectionRef(isDigital(bucket) ? ProcessConfigArea::DigitalIo : ProcessConfigArea::AnalogIo),
                     ioBucketName(bucket).toStdString()).erase(id.toStdString()); return true;
}

bool ProcessSettingsService::createTool(const QString& name, QString* error)
{
    // 中文翻译：工具名称为空或重复。
    const QString clean = name.trimmed(); if (clean.isEmpty() || toolNames().contains(clean)) { if (error) *error = QObject::tr("Tool name is empty or duplicate."); return false; }
    table tool; tool["fLineVel"] = 10.0; tool["fCutAcc"] = 100.0; tool["fCutJerk"] = 1000.0; tool["fCuttingHeight"] = 0.0; tool["fIdleHeight"] = 0.0; tool["fEnergy"] = 20.0;
    for (const auto& axis : m_axisDraft) tool[("f" + axis.axis.name + "Vel").toStdString()] = 10.0;
    table& all = sectionRef(ProcessConfigArea::Tools); all[clean.toStdString()] = tool; table& index = ensureChildTable(all, "ToolIndex"); index["sTool_" + std::to_string(toolNames().size())] = clean.toStdString(); if (!index.count("sToolIndex")) index["sToolIndex"] = clean.toStdString(); return true;
}

// 中文翻译：源工具不存在。
bool ProcessSettingsService::copyTool(const QString& source, const QString& target, QString* error) { const table all = sectionRef(ProcessConfigArea::Tools); const auto it = all.find(source.toStdString()); if (it == all.end()) { if (error) *error = QObject::tr("The source tool does not exist."); return false; } if (!createTool(target, error)) return false; sectionRef(ProcessConfigArea::Tools)[target.trimmed().toStdString()] = it->second; return true; }
bool ProcessSettingsService::renameTool(const QString& source, const QString& target, QString* error) { if (source == target) return true; if (!copyTool(source, target, error)) return false; return deleteTool(source, error); }
// 中文翻译：至少保留一个工具。
bool ProcessSettingsService::deleteTool(const QString& name, QString* error) { auto names = toolNames(); if (names.size() <= 1 || !names.removeOne(name)) { if (error) *error = QObject::tr("Keep at least one tool."); return false; } table& all = sectionRef(ProcessConfigArea::Tools); all.erase(name.toStdString()); table idx; idx["sToolIndex"] = names.first().toStdString(); for (int i = 0; i < names.size(); ++i) idx["sTool_" + std::to_string(i)] = names[i].toStdString(); all["ToolIndex"] = idx; return true; }

ProcessSettingsChangeSet ProcessSettingsService::changesSinceCommitted() const
{
    ProcessSettingsChangeSet result; if (m_axisDirty) result.domains.append(QStringLiteral("machine"));
    const auto changed = [this](const QString& section) { const auto& oldRoot = m_committed.value.at("Setting").as_table(); const auto& nowRoot = m_draft.at("Setting").as_table(); const auto oldIt = oldRoot.find(section.toStdString()); const auto nowIt = nowRoot.find(section.toStdString()); return oldIt == oldRoot.end() || nowIt == nowRoot.end() || !sameToml(oldIt->second, nowIt->second); };
    if (changed("MotionControl") || changed("Laser") || changed("Internet") || changed("Camera")) result.domains.append(QStringLiteral("devices"));
    if (changed("Digital") || changed("Analog")) result.domains.append(QStringLiteral("io"));
    if (changed("Gas") || changed("Water") || changed("Monitor") || changed("LoadingPos")) result.domains.append(QStringLiteral("operations"));
    if (changed("Special")) result.domains.append(QStringLiteral("workflow")); if (changed("Tool")) result.domains.append(QStringLiteral("tools")); return result;
}

bool ProcessSettingsService::hasChanges() const { return !changesSinceCommitted().empty(); }
// 中文翻译：至少需要保留一个工具。
bool ProcessSettingsService::validate(QString* error) const { if (toolNames().isEmpty()) { if (error) *error = QObject::tr("At least one tool needs to be kept."); return false; } return true; }

bool ProcessSettingsService::writeDomain(const QString& fileName, const QStringList& sections, QString* error) const
{
    toml::value root(toml::table{}); root["schemaVersion"] = 2; for (const auto& section : sections) { const auto it = m_draft.at("Setting").as_table().find(section.toStdString()); if (it != m_draft.at("Setting").as_table().end()) root["Setting"][section.toStdString()] = it->second; }
    return writeTomlAtomically(QDir(rootDir()).filePath(fileName), root, error);
}

bool ProcessSettingsService::writeTools(QString* error, ProcessSettingsChangeSet*) const
{
    toml::value index(toml::table{}); index["schemaVersion"] = 2; index["Setting"]["Tool"]["ToolIndex"] = sectionRef(ProcessConfigArea::Tools).at("ToolIndex"); toml::array entries;
    for (const auto& name : toolNames()) { const QString id = toolId(name); toml::value entry(toml::table{}); entry["id"] = id.toStdString(); entry["name"] = name.toStdString(); entries.push_back(entry); toml::value tool(toml::table{}); tool["schemaVersion"] = 2; tool["Setting"]["Tool"][name.toStdString()] = sectionRef(ProcessConfigArea::Tools).at(name.toStdString()); if (!writeTomlAtomically(QDir(rootDir()).filePath(QStringLiteral("tools/%1.toml").arg(id)), tool, error)) return false; }
    index["tools"] = entries; return writeTomlAtomically(QDir(rootDir()).filePath(QStringLiteral("tools/index.toml")), index, error);
}

ProcessSettingsCommitResult ProcessSettingsService::commit()
{
    ProcessSettingsCommitResult result; result.changes = changesSinceCommitted(); if (result.changes.empty()) { result.success = true; return result; } if (!validate(&result.error)) return result;
    QString error; const auto has = [&result](const QString& name) { return result.changes.domains.contains(name); };
    if ((has("devices") && !writeDomain(QStringLiteral("devices.toml"), {"MotionControl", "Laser", "Internet", "Camera"}, &error)) || (has("io") && !writeDomain(QStringLiteral("io.toml"), {"Digital", "Analog"}, &error)) || (has("operations") && !writeDomain(QStringLiteral("operations.toml"), {"Gas", "Water", "Monitor", "LoadingPos"}, &error)) || (has("workflow") && !writeDomain(QStringLiteral("workflow.toml"), {"Special"}, &error)) || (has("tools") && !writeTools(&error, &result.changes))) { result.error = error; return result; }
    // 中文翻译：机台配置服务不可用。
    if (has("machine")) { auto machine = Kernel::current().services().getService<MachineConfigurationService>(); if (!machine) { result.error = QObject::tr("The machine configuration service is unavailable."); return result; } machine->setAxisHardwareConfigurations(m_axisDraft); }
    m_committed.value = m_draft; beginEdit(); result.success = true; return result;
}

} // namespace lcnc::process
