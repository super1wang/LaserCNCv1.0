#include "modules/cam/settings/cam_config.h"

#include "core/algorithms/cam/laser_toolpath.h"
#include "core/logging/logger.h"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>

#include <algorithm>
#include <cmath>

namespace {

constexpr double kEps    = 1e-9;
constexpr double kEps2   = 1e-12;

bool nearlyEqual(double a, double b)        { return qAbs(a - b) <= kEps; }
bool samePoint(const gp_Pnt& a, const gp_Pnt& b) { return a.SquareDistance(b) <= kEps2; }

QString normalizedCollisionSourceId(QString value)
{
    value = value.trimmed();
    if (value.startsWith(QStringLiteral("axis:"), Qt::CaseInsensitive))
        return QStringLiteral("axis:") + value.mid(5).trimmed().toUpper();
    return value.toLower();
}

QString renderQualityToString(lcnc::RenderQualityPreset q)
{
    switch (q) {
    case lcnc::RenderQualityPreset::High:   return QStringLiteral("high");
    case lcnc::RenderQualityPreset::Medium: return QStringLiteral("medium");
    case lcnc::RenderQualityPreset::Low:    return QStringLiteral("low");
    case lcnc::RenderQualityPreset::Custom: return QStringLiteral("custom");
    }
    return QStringLiteral("medium");
}

lcnc::RenderQualityPreset renderQualityFromString(const QString& v)
{
    const QString s = v.trimmed().toLower();
    if (s == QStringLiteral("high")) return lcnc::RenderQualityPreset::High;
    if (s == QStringLiteral("low"))  return lcnc::RenderQualityPreset::Low;
    if (s == QStringLiteral("custom")) return lcnc::RenderQualityPreset::Custom;
    return lcnc::RenderQualityPreset::Medium;
}

QString autoSortAxisToString(lcnc::cam::AutoSortAxis axis)
{
    switch (axis) {
    case lcnc::cam::AutoSortAxis::XPos: return QStringLiteral("X+");
    case lcnc::cam::AutoSortAxis::XNeg: return QStringLiteral("X-");
    case lcnc::cam::AutoSortAxis::YPos: return QStringLiteral("Y+");
    case lcnc::cam::AutoSortAxis::YNeg: return QStringLiteral("Y-");
    case lcnc::cam::AutoSortAxis::ZPos: return QStringLiteral("Z+");
    case lcnc::cam::AutoSortAxis::ZNeg: return QStringLiteral("Z-");
    }
    return QStringLiteral("X+");
}

lcnc::cam::AutoSortAxis autoSortAxisFromString(const QString& value)
{
    const QString axis = value.trimmed().toUpper();
    if (axis == QStringLiteral("X-")) return lcnc::cam::AutoSortAxis::XNeg;
    if (axis == QStringLiteral("Y+")) return lcnc::cam::AutoSortAxis::YPos;
    if (axis == QStringLiteral("Y-")) return lcnc::cam::AutoSortAxis::YNeg;
    if (axis == QStringLiteral("Z+")) return lcnc::cam::AutoSortAxis::ZPos;
    if (axis == QStringLiteral("Z-")) return lcnc::cam::AutoSortAxis::ZNeg;
    return lcnc::cam::AutoSortAxis::XPos;
}

QString collisionVerificationModeToString(
    lcnc::cam::CollisionVerificationMode mode)
{
    switch (mode) {
    case lcnc::cam::CollisionVerificationMode::Disabled:
        return QStringLiteral("disabled");
    case lcnc::cam::CollisionVerificationMode::Optional:
        return QStringLiteral("optional");
    case lcnc::cam::CollisionVerificationMode::Required:
        return QStringLiteral("required");
    }
    return QStringLiteral("disabled");
}

bool collisionVerificationModeFromString(
    const QString& value, lcnc::cam::CollisionVerificationMode* mode)
{
    if (!mode)
        return false;
    const QString normalized = value.trimmed().toLower();
    if (normalized == QStringLiteral("optional")) {
        *mode = lcnc::cam::CollisionVerificationMode::Optional;
        return true;
    }
    if (normalized == QStringLiteral("required")) {
        *mode = lcnc::cam::CollisionVerificationMode::Required;
        return true;
    }
    if (normalized == QStringLiteral("disabled")) {
        *mode = lcnc::cam::CollisionVerificationMode::Disabled;
        return true;
    }
    return false;
}

// ── TOML <-> gp_Pnt ─────────────────────────────────────────────────────────
toml::value pointToToml(const gp_Pnt& p)
{
    toml::array a;
    a.emplace_back(p.X());
    a.emplace_back(p.Y());
    a.emplace_back(p.Z());
    return toml::value(a);
}

bool pointFromToml(const toml::value& v, gp_Pnt* out)
{
    if (!out || !v.is_array()) return false;
    const auto& a = v.as_array();
    if (a.size() != 3) return false;
    auto coord = [](const toml::value& x) -> double {
        if (x.is_floating()) return x.as_floating();
        if (x.is_integer())  return static_cast<double>(x.as_integer());
        return 0.0;
    };
    *out = gp_Pnt(coord(a[0]), coord(a[1]), coord(a[2]));
    return true;
}

} // namespace

// ── Path helpers ────────────────────────────────────────────────────────────
QString CamConfig::configDirectoryPath()
{
    return QDir(QCoreApplication::applicationDirPath())
        .filePath(QStringLiteral("config"));
}

QString CamConfig::tomlFilePath()
{
    return QDir(configDirectoryPath()).filePath(QStringLiteral("cam.toml"));
}

QString CamConfig::machineKey(const QString& machinePath)
{
    if (machinePath.isEmpty()) return QStringLiteral("__virtual_collision_environment__");
    return QFileInfo(machinePath).absoluteFilePath().replace('\\', '/').toLower();
}

// ── Load / Save ─────────────────────────────────────────────────────────────
bool CamConfig::loadDefault()
{
    LCNC_DEBUG(lcnc::LogCode::SettingsLoaded, "CamConfig::loadDefault begin");

    return load(tomlFilePath());
}

bool CamConfig::saveDefault() const
{
    return save(tomlFilePath());
}

// ── TOML serialization ──────────────────────────────────────────────────────
void CamConfig::readFrom(const toml::value& root)
{
    using namespace lcnc::toml_io;

    m_machineModelPath     = get_qstring(root, "machineModelPath",     QString());
    m_autoLoadMachineModel = get_bool(root, "autoLoadMachineModel", true);
    m_machinePreset        = get_qstring(root, "machinePreset",        QString());
    m_autoInstallWorkpiece = get_bool(root, "autoInstallWorkpiece", true);
    const QString presetText = get_qstring(
        root,
        "machineRenderQualityPreset",
        get_qstring(root, "machineRenderQuality", QStringLiteral("medium")));
    m_machineRenderQualityPreset = renderQualityFromString(presetText);

    if (root.contains("cutterCollision") && root.at("cutterCollision").is_table()) {
        const auto& collision = root.at("cutterCollision");
        const int mode = static_cast<int>(get_int(collision, "proxyMode", 0));
        m_cutterCollisionProxyMode = mode == static_cast<int>(CutterCollisionProxyMode::ModelFile)
            ? CutterCollisionProxyMode::ModelFile : CutterCollisionProxyMode::SimulatedCone;
        m_cutterNozzleModelPath = get_qstring(collision, "modelPath", QString());
        m_simulatedConeLengthMm = std::max(0.1, get_double(collision, "coneLengthMm", 20.0));
        m_simulatedConeTipRadiusMm = std::max(0.0, get_double(collision, "coneTipRadiusMm", 0.2));
        m_simulatedConeBaseRadiusMm = std::max(m_simulatedConeTipRadiusMm,
            get_double(collision, "coneBaseRadiusMm", 5.0));
        m_cutterCollisionClearanceMm = std::max(0.0,
            get_double(collision, "clearanceMm", 0.5));
        m_maximumRapidSafetyOffsetMm = std::max(0.1,
            get_double(collision, "maximumSafetyOffsetMm", 100.0));
        m_blockMachiningOnCollisionWarning = get_bool(collision, "blockMachiningOnWarning", true);
    }

    if (root.contains("toolpath") && root.at("toolpath").is_table()) {
        const auto& tp = root.at("toolpath");
        m_leadInLength          = get_double(tp, "leadInLength",          m_leadInLength);
        m_deflection            = get_double(tp, "deflection",            m_deflection);
        m_cuttingOffsetMm       = get_double(tp, "cuttingOffsetMm",       m_cuttingOffsetMm);
        m_rapidOffsetMm         = std::max(0.0, get_double(tp, "rapidOffsetMm", m_rapidOffsetMm));
        if (m_rapidOffsetMm <= m_cuttingOffsetMm)
            m_rapidOffsetMm = std::max(5.0, m_cuttingOffsetMm + 0.001);
        m_smoothAngle           = get_double(tp, "smoothAngle",           m_smoothAngle);
        m_useFaceClassification = get_bool  (tp, "useFaceClassification", m_useFaceClassification);
        m_extractionStrategy = static_cast<int>(extractionStrategyFromPersistedValue(
            get_int(tp, "extractionStrategy", m_extractionStrategy)));
        m_showNormals           = get_bool  (tp, "showNormals",           m_showNormals);
        m_normalSampleStep      = get_double(tp, "normalSampleStep",      m_normalSampleStep);
        m_autoSortAxis = autoSortAxisFromString(
            get_qstring(tp, "autoSortAxis", autoSortAxisToString(m_autoSortAxis)));
    }

    m_trajectoryOptimizationMode = QStringLiteral("Off");
    m_enableDofReduction = false;
    m_enableLaserZHold = false;
    if (root.contains("trajectory")) {
        const auto& trajectory = root.at("trajectory");
        if (!trajectory.is_table()) m_trajectoryOptimizationMode = QStringLiteral("Invalid");
        else {
            m_trajectoryOptimizationMode = get_qstring(trajectory, "optimizationMode", QStringLiteral("Off"));
            m_enableDofReduction = get_bool(trajectory, "enableDofReduction", false);
            m_enableLaserZHold = get_bool(trajectory, "enableLaserZHold", false);
            if ((trajectory.contains("optimizationMode") && !trajectory.at("optimizationMode").is_string())
                || (trajectory.contains("enableDofReduction") && !trajectory.at("enableDofReduction").is_boolean())
                || (trajectory.contains("enableLaserZHold") && !trajectory.at("enableLaserZHold").is_boolean()))
                m_trajectoryOptimizationMode = QStringLiteral("Invalid");
        }
    }
    m_machineProfiles.clear();
    if (root.contains("machineProfile") && root.at("machineProfile").is_array()) {
        for (const auto& mp : root.at("machineProfile").as_array()) {
            if (!mp.is_table()) continue;
            const QString path = get_qstring(mp, "path", QString());
            if (path.isEmpty()) continue;

            MachineProfile profile;
            gp_Pnt p;
            if (mp.contains("cutterHeadModelPosition")    && pointFromToml(mp.at("cutterHeadModelPosition"),    &p)) { profile.hasCutterHeadModel       = true; profile.cutterHeadModelPosition       = p; }
            if (mp.contains("cutterHeadPhysicalPosition") && pointFromToml(mp.at("cutterHeadPhysicalPosition"), &p)) { profile.hasCutterHeadPhysical    = true; profile.cutterHeadPhysicalPosition    = p; }
            if (mp.contains("workpieceInstallPosition")   && pointFromToml(mp.at("workpieceInstallPosition"),   &p)) { profile.hasWorkpieceInstallPosition = true; profile.workpieceInstallPosition   = p; }

            // Legacy collision source arrays are read only for compatible
            // round-trip. Runtime roles are derived from fixed machine
            // topology plus the current workpiece and ignore these values.
            if (mp.contains("collisionVerificationMode")) {
                profile.invalidCollisionVerificationMode =
                    get_qstring(mp, "collisionVerificationMode", QString());
                profile.collisionVerificationModeValid =
                    mp.at("collisionVerificationMode").is_string()
                    && collisionVerificationModeFromString(
                        profile.invalidCollisionVerificationMode,
                        &profile.collisionVerificationMode);
                if (!profile.collisionVerificationModeValid) {
                    // An explicit malformed policy is safety-critical. Keep it
                    // invalid and fail closed instead of falling back to the
                    // legacy false/default value.
                    profile.collisionVerificationMode =
                        lcnc::cam::CollisionVerificationMode::Required;
                } else {
                    profile.invalidCollisionVerificationMode.clear();
                }
            } else {
                profile.collisionVerificationMode =
                    get_bool(mp, "collisionDetectionEnabled", false)
                    ? lcnc::cam::CollisionVerificationMode::Required
                    : lcnc::cam::CollisionVerificationMode::Disabled;
            }
            const auto readSources = [&mp](const char* name, const QString& fallback) {
                QSet<QString> result;
                const bool hasPersistedArray = mp.contains(name) && mp.at(name).is_array();
                if (hasPersistedArray) {
                    for (const auto& item : mp.at(name).as_array()) {
                        if (!item.is_string()) continue;
                        const QString value = normalizedCollisionSourceId(
                            QString::fromStdString(item.as_string()));
                        if (!value.isEmpty()) result.insert(value);
                    }
                }
                // An explicitly persisted empty array is meaningful: the
                // configuration remains incomplete until the operator selects
                // a source.  Apply defaults only to profiles written before
                // source-level collision configuration existed.
                // 中文翻译：显式空数组表示尚未选择碰撞源；仅旧配置缺少字段时使用默认值。
                if (!hasPersistedArray) result.insert(fallback);
                return result;
            };
            profile.activeCollisionSources = readSources("activeCollisionSources", QStringLiteral("cutter"));
            profile.passiveCollisionSources = readSources("passiveCollisionSources", QStringLiteral("workpiece"));

            if (mp.contains("axisOrigins") && mp.at("axisOrigins").is_array()) {
                for (const auto& ao : mp.at("axisOrigins").as_array()) {
                    if (!ao.is_table()) continue;
                    const QString name = get_qstring(ao, "name", QString());
                    gp_Pnt origin;
                    if (!name.isEmpty() && ao.contains("origin") && pointFromToml(ao.at("origin"), &origin))
                        profile.axisOrigins.insert(name, origin);
                }
            }
            m_machineProfiles.insert(machineKey(path), profile);
        }
    }
}

void CamConfig::writeTo(toml::value& root) const
{
    using namespace lcnc::toml_io;

    root["machineModelPath"]     = qs(m_machineModelPath);
    root["autoLoadMachineModel"] = m_autoLoadMachineModel;
    root["machinePreset"]        = qs(m_machinePreset);
    root["autoInstallWorkpiece"] = m_autoInstallWorkpiece;
    root["machineRenderQualityPreset"] = qs(renderQualityToString(m_machineRenderQualityPreset));

    toml::value collision(toml::table{});
    collision["proxyMode"] = static_cast<int>(m_cutterCollisionProxyMode);
    collision["modelPath"] = qs(m_cutterNozzleModelPath);
    collision["coneLengthMm"] = m_simulatedConeLengthMm;
    collision["coneTipRadiusMm"] = m_simulatedConeTipRadiusMm;
    collision["coneBaseRadiusMm"] = m_simulatedConeBaseRadiusMm;
    collision["clearanceMm"] = m_cutterCollisionClearanceMm;
    collision["maximumSafetyOffsetMm"] = m_maximumRapidSafetyOffsetMm;
    collision["blockMachiningOnWarning"] = m_blockMachiningOnCollisionWarning;
    root["cutterCollision"] = collision;

    toml::value tp(toml::table{});
    tp["leadInLength"]          = m_leadInLength;
    tp["deflection"]            = m_deflection;
    tp["cuttingOffsetMm"]       = m_cuttingOffsetMm;
    tp["rapidOffsetMm"]         = m_rapidOffsetMm;
    tp["smoothAngle"]           = m_smoothAngle;
    tp["useFaceClassification"] = m_useFaceClassification;
    tp["extractionStrategy"] = m_extractionStrategy;
    tp["showNormals"]           = m_showNormals;
    tp["normalSampleStep"]      = m_normalSampleStep;
    tp["autoSortAxis"]          = qs(autoSortAxisToString(m_autoSortAxis));
    root["toolpath"] = tp;
    root["trajectory"] = toml::table{
        {"optimizationMode", qs(m_trajectoryOptimizationMode)},
        {"enableDofReduction", m_enableDofReduction},
        {"enableLaserZHold", m_enableLaserZHold}};

    toml::array profiles;
    for (auto it = m_machineProfiles.cbegin(); it != m_machineProfiles.cend(); ++it) {
        toml::value mp(toml::table{});
        mp["path"] = qs(it.key());
        if (it.value().hasCutterHeadModel)       mp["cutterHeadModelPosition"]    = pointToToml(it.value().cutterHeadModelPosition);
        if (it.value().hasCutterHeadPhysical)    mp["cutterHeadPhysicalPosition"] = pointToToml(it.value().cutterHeadPhysicalPosition);
        mp["collisionVerificationMode"] = qs(
            it.value().collisionVerificationModeValid
                ? collisionVerificationModeToString(
                    it.value().collisionVerificationMode)
                : it.value().invalidCollisionVerificationMode);
        toml::array activeSources;
        for (const QString& source : it.value().activeCollisionSources)
            activeSources.emplace_back(qs(source));
        mp["activeCollisionSources"] = activeSources;
        toml::array passiveSources;
        for (const QString& source : it.value().passiveCollisionSources)
            passiveSources.emplace_back(qs(source));
        mp["passiveCollisionSources"] = passiveSources;

        toml::array axes;
        for (auto ax = it.value().axisOrigins.cbegin(); ax != it.value().axisOrigins.cend(); ++ax) {
            toml::value entry(toml::table{});
            entry["name"]   = qs(ax.key());
            entry["origin"] = pointToToml(ax.value());
            axes.emplace_back(entry);
        }
        if (!axes.empty()) mp["axisOrigins"] = axes;
        profiles.emplace_back(mp);
    }
    if (!profiles.empty()) root["machineProfile"] = profiles;
}

// ── Setters with auto-save ──────────────────────────────────────────────────
void CamConfig::setMachineModelPath(const QString& path)
{
    const QString normalized = path.trimmed().isEmpty()
        ? QString()
        : QFileInfo(path).absoluteFilePath();
    if (m_machineModelPath == normalized) return;
    m_machineModelPath = normalized;
    saveDefault();
}

void CamConfig::setAutoLoadMachineModel(bool enabled)
{
    if (m_autoLoadMachineModel == enabled) return;
    m_autoLoadMachineModel = enabled;
    saveDefault();
}

void CamConfig::setMachinePreset(const QString& preset)
{
    if (m_machinePreset == preset) return;
    m_machinePreset = preset;
    saveDefault();
}

void CamConfig::setMachineRenderQualityPreset(lcnc::RenderQualityPreset quality)
{
    if (m_machineRenderQualityPreset == quality) return;
    m_machineRenderQualityPreset = quality;
    saveDefault();
}

void CamConfig::setAutoInstallWorkpiece(bool enabled)
{
    if (m_autoInstallWorkpiece == enabled) return;
    m_autoInstallWorkpiece = enabled;
    saveDefault();
}

void CamConfig::setLeadInLength(double mm)
{
    if (nearlyEqual(m_leadInLength, mm)) return;
    m_leadInLength = mm;
    saveDefault();
}

void CamConfig::setDeflection(double mm)
{
    if (nearlyEqual(m_deflection, mm)) return;
    m_deflection = mm;
    saveDefault();
}

void CamConfig::setSmoothAngle(double deg)
{
    if (nearlyEqual(m_smoothAngle, deg)) return;
    m_smoothAngle = deg;
    saveDefault();
}

void CamConfig::setUseFaceClassification(bool enabled)
{
    if (m_useFaceClassification == enabled) return;
    m_useFaceClassification = enabled;
    saveDefault();
}

void CamConfig::setExtractionStrategy(int strategy)
{
    strategy = static_cast<int>(extractionStrategyFromPersistedValue(strategy));
    if (m_extractionStrategy == strategy) return;
    m_extractionStrategy = strategy;
    saveDefault();
}

void CamConfig::setShowNormals(bool enabled)
{
    if (m_showNormals == enabled) return;
    m_showNormals = enabled;
    saveDefault();
}

void CamConfig::setNormalSampleStep(double mm)
{
    if (nearlyEqual(m_normalSampleStep, mm)) return;
    m_normalSampleStep = mm;
    saveDefault();
}

// ── Profile lookup ──────────────────────────────────────────────────────────
CamConfig::MachineProfile* CamConfig::mutableProfileForMachine(const QString& machinePath)
{
    return &m_machineProfiles[machineKey(machinePath)];
}

const CamConfig::MachineProfile* CamConfig::profileForMachine(const QString& machinePath) const
{
    const QString key = machineKey(machinePath);
    const auto it = m_machineProfiles.constFind(key);
    if (it == m_machineProfiles.cend()) return nullptr;
    return &it.value();
}

bool CamConfig::axisOriginForMachine(const QString& machinePath,
                                     const QString& axisName,
                                     gp_Pnt* outOrigin) const
{
    const auto* profile = profileForMachine(machinePath);
    if (!profile || !outOrigin) return false;
    const auto it = profile->axisOrigins.constFind(axisName);
    if (it == profile->axisOrigins.cend()) return false;
    *outOrigin = it.value();
    return true;
}

void CamConfig::setAxisOriginForMachine(const QString& machinePath,
                                        const QString& axisName,
                                        const gp_Pnt& origin)
{
    if (machinePath.isEmpty() || axisName.isEmpty()) return;
    auto* profile = mutableProfileForMachine(machinePath);
    const auto it = profile->axisOrigins.constFind(axisName);
    if (it != profile->axisOrigins.cend() && samePoint(it.value(), origin)) return;
    profile->axisOrigins.insert(axisName, origin);
    saveDefault();
}

bool CamConfig::cutterHeadModelPositionForMachine(const QString& machinePath,
                                                  gp_Pnt* outPosition) const
{
    const auto* profile = profileForMachine(machinePath);
    if (!profile || !profile->hasCutterHeadModel || !outPosition) return false;
    *outPosition = profile->cutterHeadModelPosition;
    return true;
}

void CamConfig::setCutterHeadModelPositionForMachine(const QString& machinePath,
                                                     const gp_Pnt& position)
{
    if (machinePath.isEmpty()) return;
    auto* profile = mutableProfileForMachine(machinePath);
    if (profile->hasCutterHeadModel && samePoint(profile->cutterHeadModelPosition, position)) return;
    profile->hasCutterHeadModel       = true;
    profile->cutterHeadModelPosition  = position;
    saveDefault();
}

bool CamConfig::cutterHeadPhysicalPositionForMachine(const QString& machinePath,
                                                     gp_Pnt* outPosition) const
{
    const auto* profile = profileForMachine(machinePath);
    if (!profile || !profile->hasCutterHeadPhysical || !outPosition) return false;
    *outPosition = profile->cutterHeadPhysicalPosition;
    return true;
}

void CamConfig::setCutterHeadPhysicalPositionForMachine(const QString& machinePath,
                                                        const gp_Pnt& position)
{
    if (machinePath.isEmpty()) return;
    auto* profile = mutableProfileForMachine(machinePath);
    if (profile->hasCutterHeadPhysical && samePoint(profile->cutterHeadPhysicalPosition, position)) return;
    profile->hasCutterHeadPhysical       = true;
    profile->cutterHeadPhysicalPosition  = position;
    saveDefault();
}

bool CamConfig::workpieceInstallPositionForMachine(const QString& machinePath,
                                                   gp_Pnt* outPosition) const
{
    const auto* profile = profileForMachine(machinePath);
    if (!profile || !profile->hasWorkpieceInstallPosition || !outPosition) return false;
    *outPosition = profile->workpieceInstallPosition;
    return true;
}

void CamConfig::setCuttingOffsetMm(double mm)
{
    if (!std::isfinite(mm) || nearlyEqual(m_cuttingOffsetMm, mm)) return;
    m_cuttingOffsetMm = mm;
    if (m_rapidOffsetMm <= m_cuttingOffsetMm)
        m_rapidOffsetMm = m_cuttingOffsetMm + 0.001;
    saveDefault();
}

void CamConfig::setAutoSortAxis(lcnc::cam::AutoSortAxis axis)
{
    if (m_autoSortAxis == axis) return;
    m_autoSortAxis = axis;
    saveDefault();
}

void CamConfig::setRapidOffsetMm(double mm)
{
    if (!std::isfinite(mm) || mm < 0.0 || mm <= m_cuttingOffsetMm
        || nearlyEqual(m_rapidOffsetMm, mm)) return;
    m_rapidOffsetMm = mm;
    saveDefault();
}

void CamConfig::setCutterCollisionProxyMode(CutterCollisionProxyMode mode)
{
    if (m_cutterCollisionProxyMode == mode) return;
    m_cutterCollisionProxyMode = mode;
    saveDefault();
}

void CamConfig::setCutterNozzleModelPath(const QString& path)
{
    const QString normalized = path.trimmed().isEmpty()
        ? QString() : QFileInfo(path).absoluteFilePath();
    if (m_cutterNozzleModelPath == normalized) return;
    m_cutterNozzleModelPath = normalized;
    saveDefault();
}

void CamConfig::setSimulatedConeLengthMm(double value)
{
    value = std::max(0.1, value);
    if (nearlyEqual(m_simulatedConeLengthMm, value)) return;
    m_simulatedConeLengthMm = value;
    saveDefault();
}

void CamConfig::setSimulatedConeTipRadiusMm(double value)
{
    value = std::max(0.0, value);
    if (nearlyEqual(m_simulatedConeTipRadiusMm, value)) return;
    m_simulatedConeTipRadiusMm = value;
    if (m_simulatedConeBaseRadiusMm < value) m_simulatedConeBaseRadiusMm = value;
    saveDefault();
}

void CamConfig::setSimulatedConeBaseRadiusMm(double value)
{
    value = std::max(m_simulatedConeTipRadiusMm, value);
    if (nearlyEqual(m_simulatedConeBaseRadiusMm, value)) return;
    m_simulatedConeBaseRadiusMm = value;
    saveDefault();
}

void CamConfig::setCutterCollisionClearanceMm(double value)
{
    value = std::max(0.0, value);
    if (nearlyEqual(m_cutterCollisionClearanceMm, value)) return;
    m_cutterCollisionClearanceMm = value;
    saveDefault();
}

void CamConfig::setMaximumRapidSafetyOffsetMm(double value)
{
    value = std::max(0.1, value);
    if (nearlyEqual(m_maximumRapidSafetyOffsetMm, value)) return;
    m_maximumRapidSafetyOffsetMm = value;
    saveDefault();
}

void CamConfig::setBlockMachiningOnCollisionWarning(bool enabled)
{
    if (m_blockMachiningOnCollisionWarning == enabled) return;
    m_blockMachiningOnCollisionWarning = enabled;
    saveDefault();
}

bool CamConfig::collisionDetectionEnabledForMachine(const QString& machinePath) const
{
    return collisionVerificationModeForMachine(machinePath)
        != lcnc::cam::CollisionVerificationMode::Disabled;
}

void CamConfig::setCollisionDetectionEnabledForMachine(const QString& machinePath, bool enabled)
{
    setCollisionVerificationModeForMachine(
        machinePath, enabled ? lcnc::cam::CollisionVerificationMode::Required
                             : lcnc::cam::CollisionVerificationMode::Disabled);
}

lcnc::cam::CollisionVerificationMode
CamConfig::collisionVerificationModeForMachine(const QString& machinePath) const
{
    const auto* profile = profileForMachine(machinePath);
    return profile ? profile->collisionVerificationMode
                   : lcnc::cam::CollisionVerificationMode::Disabled;
}

bool CamConfig::collisionVerificationModeValidForMachine(
    const QString& machinePath) const
{
    const auto* profile = profileForMachine(machinePath);
    return !profile || profile->collisionVerificationModeValid;
}

void CamConfig::setCollisionVerificationModeForMachine(
    const QString& machinePath, lcnc::cam::CollisionVerificationMode mode)
{
    auto* profile = mutableProfileForMachine(machinePath);
    if (profile->collisionVerificationMode == mode
        && profile->collisionVerificationModeValid) return;
    profile->collisionVerificationMode = mode;
    profile->collisionVerificationModeValid = true;
    profile->invalidCollisionVerificationMode.clear();
    saveDefault();
}

QSet<QString> CamConfig::activeCollisionSourcesForMachine(const QString& machinePath) const
{
    const auto* profile = profileForMachine(machinePath);
    return profile ? profile->activeCollisionSources : QSet<QString>{QStringLiteral("cutter")};
}

QSet<QString> CamConfig::passiveCollisionSourcesForMachine(const QString& machinePath) const
{
    const auto* profile = profileForMachine(machinePath);
    return profile ? profile->passiveCollisionSources : QSet<QString>{QStringLiteral("workpiece")};
}

void CamConfig::setCollisionSourcesForMachine(const QString& machinePath,
                                              const QSet<QString>& active,
                                              const QSet<QString>& passive)
{
    auto* profile = mutableProfileForMachine(machinePath);
    QSet<QString> normalizedActive;
    QSet<QString> normalizedPassive;
    for (const QString& source : active) normalizedActive.insert(normalizedCollisionSourceId(source));
    for (const QString& source : passive) normalizedPassive.insert(normalizedCollisionSourceId(source));
    if (profile->activeCollisionSources == normalizedActive
        && profile->passiveCollisionSources == normalizedPassive) return;
    profile->activeCollisionSources = normalizedActive;
    profile->passiveCollisionSources = normalizedPassive;
    saveDefault();
}

void CamConfig::copyMachineProfile(const QString& sourceMachinePath,
                                   const QString& targetMachinePath)
{
    const QString sourceKey = machineKey(sourceMachinePath);
    const QString targetKey = machineKey(targetMachinePath);
    if (sourceKey == targetKey)
        return;
    const auto source = m_machineProfiles.constFind(sourceKey);
    if (source == m_machineProfiles.cend())
        return;
    m_machineProfiles.insert(targetKey, source.value());
    saveDefault();
}

void CamConfig::clearLegacyWorkpieceInstallPositionForMachine(const QString& machinePath)
{
    if (machinePath.isEmpty())
        return;
    auto* profile = mutableProfileForMachine(machinePath);
    if (!profile || !profile->hasWorkpieceInstallPosition)
        return;
    profile->hasWorkpieceInstallPosition = false;
    profile->workpieceInstallPosition = gp_Pnt();
    saveDefault();
}
