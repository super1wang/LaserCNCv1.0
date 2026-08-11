#include "modules/cam/settings/cam_config.h"

#include "core/algorithms/cam/laser_toolpath.h"
#include "core/logging/logger.h"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>

namespace {

constexpr double kEps    = 1e-9;
constexpr double kEps2   = 1e-12;

bool nearlyEqual(double a, double b)        { return qAbs(a - b) <= kEps; }
bool samePoint(const gp_Pnt& a, const gp_Pnt& b) { return a.SquareDistance(b) <= kEps2; }

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
    if (machinePath.isEmpty()) return QString();
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

    if (root.contains("toolpath") && root.at("toolpath").is_table()) {
        const auto& tp = root.at("toolpath");
        m_leadInLength          = get_double(tp, "leadInLength",          m_leadInLength);
        m_deflection            = get_double(tp, "deflection",            m_deflection);
        m_smoothAngle           = get_double(tp, "smoothAngle",           m_smoothAngle);
        m_useFaceClassification = get_bool  (tp, "useFaceClassification", m_useFaceClassification);
        m_extractionStrategy = static_cast<int>(extractionStrategyFromPersistedValue(
            get_int(tp, "extractionStrategy", m_extractionStrategy)));
        m_showNormals           = get_bool  (tp, "showNormals",           m_showNormals);
        m_normalSampleStep      = get_double(tp, "normalSampleStep",      m_normalSampleStep);
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

            if (mp.contains("acAngleOffsetA") && mp.at("acAngleOffsetA").is_floating()) {
                profile.hasAcAngleOffset = true;
                profile.acAngleOffsetA   = mp.at("acAngleOffsetA").as_floating();
            }
            if (mp.contains("acAngleOffsetC") && mp.at("acAngleOffsetC").is_floating()) {
                profile.hasAcAngleOffset = true;
                profile.acAngleOffsetC   = mp.at("acAngleOffsetC").as_floating();
            }
            if (mp.contains("physicalAcCenter") && pointFromToml(mp.at("physicalAcCenter"), &p)) {
                profile.hasPhysicalAcCenter = true;
                profile.physicalAcCenter    = p;
            }

            if (mp.contains("axisOrigins") && mp.at("axisOrigins").is_array()) {
                for (const auto& ao : mp.at("axisOrigins").as_array()) {
                    if (!ao.is_table()) continue;
                    const QString name = get_qstring(ao, "name", QString());
                    gp_Pnt origin;
                    if (!name.isEmpty() && ao.contains("origin") && pointFromToml(ao.at("origin"), &origin))
                        profile.axisOrigins.insert(name, origin);
                }
            }
            m_machineProfiles.insert(path, profile);
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

    toml::value tp(toml::table{});
    tp["leadInLength"]          = m_leadInLength;
    tp["deflection"]            = m_deflection;
    tp["smoothAngle"]           = m_smoothAngle;
    tp["useFaceClassification"] = m_useFaceClassification;
    tp["extractionStrategy"] = m_extractionStrategy;
    tp["showNormals"]           = m_showNormals;
    tp["normalSampleStep"]      = m_normalSampleStep;
    root["toolpath"] = tp;

    toml::array profiles;
    for (auto it = m_machineProfiles.cbegin(); it != m_machineProfiles.cend(); ++it) {
        toml::value mp(toml::table{});
        mp["path"] = qs(it.key());
        if (it.value().hasCutterHeadModel)       mp["cutterHeadModelPosition"]    = pointToToml(it.value().cutterHeadModelPosition);
        if (it.value().hasCutterHeadPhysical)    mp["cutterHeadPhysicalPosition"] = pointToToml(it.value().cutterHeadPhysicalPosition);
        if (it.value().hasAcAngleOffset) {
            mp["acAngleOffsetA"] = it.value().acAngleOffsetA;
            mp["acAngleOffsetC"] = it.value().acAngleOffsetC;
        }
        if (it.value().hasPhysicalAcCenter) {
            mp["physicalAcCenter"] = pointToToml(it.value().physicalAcCenter);
        }

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

bool CamConfig::acAngleOffsetForMachine(const QString& machinePath,
                                        double* outA,
                                        double* outC) const
{
    const auto* profile = profileForMachine(machinePath);
    if (!profile || !profile->hasAcAngleOffset) return false;
    if (outA) *outA = profile->acAngleOffsetA;
    if (outC) *outC = profile->acAngleOffsetC;
    return true;
}

void CamConfig::setAcAngleOffsetForMachine(const QString& machinePath,
                                           double aAngle,
                                           double cAngle)
{
    if (machinePath.isEmpty()) return;
    auto* profile = mutableProfileForMachine(machinePath);
    if (profile->hasAcAngleOffset
        && nearlyEqual(profile->acAngleOffsetA, aAngle)
        && nearlyEqual(profile->acAngleOffsetC, cAngle))
        return;
    profile->hasAcAngleOffset = true;
    profile->acAngleOffsetA   = aAngle;
    profile->acAngleOffsetC   = cAngle;
    saveDefault();
}

bool CamConfig::physicalAcCenterForMachine(const QString& machinePath,
                                           gp_Pnt* outCenter) const
{
    const auto* profile = profileForMachine(machinePath);
    if (!profile || !profile->hasPhysicalAcCenter) return false;
    if (outCenter) *outCenter = profile->physicalAcCenter;
    return true;
}

void CamConfig::setPhysicalAcCenterForMachine(const QString& machinePath,
                                              const gp_Pnt& center)
{
    if (machinePath.isEmpty()) return;
    auto* profile = mutableProfileForMachine(machinePath);
    if (profile->hasPhysicalAcCenter && samePoint(profile->physicalAcCenter, center))
        return;
    profile->hasPhysicalAcCenter = true;
    profile->physicalAcCenter    = center;
    saveDefault();
}
