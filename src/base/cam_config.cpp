#include "base/cam_config.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

namespace {

QJsonArray pointToJson(const gp_Pnt& point)
{
    return QJsonArray{point.X(), point.Y(), point.Z()};
}

bool pointFromJson(const QJsonValue& value, gp_Pnt* outPoint)
{
    if (!outPoint || !value.isArray())
        return false;

    const QJsonArray array = value.toArray();
    if (array.size() != 3)
        return false;

    *outPoint = gp_Pnt(array.at(0).toDouble(),
                       array.at(1).toDouble(),
                       array.at(2).toDouble());
    return true;
}

QString renderQualityToString(MachineRenderQuality quality)
{
    switch (quality) {
    case MachineRenderQuality::High:
        return QStringLiteral("high");
    case MachineRenderQuality::Medium:
        return QStringLiteral("medium");
    case MachineRenderQuality::Low:
        return QStringLiteral("low");
    }

    return QStringLiteral("medium");
}

MachineRenderQuality renderQualityFromString(const QString& value)
{
    const QString normalized = value.trimmed().toLower();
    if (normalized == QStringLiteral("high"))
        return MachineRenderQuality::High;
    if (normalized == QStringLiteral("low"))
        return MachineRenderQuality::Low;
    return MachineRenderQuality::Medium;
}

bool nearlyEqual(double lhs, double rhs)
{
    return qAbs(lhs - rhs) <= 1e-9;
}

bool samePoint(const gp_Pnt& lhs, const gp_Pnt& rhs)
{
    return lhs.SquareDistance(rhs) <= 1e-12;
}

} // namespace

CamConfig& CamConfig::instance()
{
    static CamConfig config;
    return config;
}

void CamConfig::load()
{
    m_loaded = true;
    m_machineModelPath.clear();
    m_machinePreset.clear();
    m_machineRenderQuality = MachineRenderQuality::Medium;
    m_leadInLength = 5.0;
    m_normalAngle = 0.0;
    m_deflection = 0.1;
    m_smoothAngle = 5.0;
    m_useFaceClassification = true;
    m_showNormals = false;
    m_normalSampleStep = 2.0;
    m_machineProfiles.clear();

    QFile file(configFilePath());
    if (!file.exists())
        return;

    if (!file.open(QIODevice::ReadOnly))
        return;

    const QJsonDocument document = QJsonDocument::fromJson(file.readAll());
    if (!document.isObject())
        return;

    const QJsonObject root = document.object();
    m_machineModelPath = root.value(QStringLiteral("machineModelPath")).toString();
    m_machinePreset = root.value(QStringLiteral("machinePreset")).toString();
    m_machineRenderQuality = renderQualityFromString(
        root.value(QStringLiteral("machineRenderQuality")).toString());

    const QJsonObject toolpath = root.value(QStringLiteral("toolpath")).toObject();
    if (!toolpath.isEmpty()) {
        m_leadInLength = toolpath.value(QStringLiteral("leadInLength")).toDouble(m_leadInLength);
        m_normalAngle = toolpath.value(QStringLiteral("normalAngle")).toDouble(m_normalAngle);
        m_deflection = toolpath.value(QStringLiteral("deflection")).toDouble(m_deflection);
        m_smoothAngle = toolpath.value(QStringLiteral("smoothAngle")).toDouble(m_smoothAngle);
        m_useFaceClassification = toolpath.value(QStringLiteral("useFaceClassification")).toBool(m_useFaceClassification);
        m_showNormals = toolpath.value(QStringLiteral("showNormals")).toBool(m_showNormals);
        m_normalSampleStep = toolpath.value(QStringLiteral("normalSampleStep")).toDouble(m_normalSampleStep);
    }

    const QJsonObject profiles = root.value(QStringLiteral("machineProfiles")).toObject();
    for (auto it = profiles.begin(); it != profiles.end(); ++it) {
        if (!it.value().isObject())
            continue;

        MachineProfile profile;
        const QJsonObject profileObject = it.value().toObject();

        const QJsonObject axisOrigins = profileObject.value(QStringLiteral("axisOrigins")).toObject();
        for (auto axisIt = axisOrigins.begin(); axisIt != axisOrigins.end(); ++axisIt) {
            gp_Pnt origin;
            if (pointFromJson(axisIt.value(), &origin))
                profile.axisOrigins.insert(axisIt.key(), origin);
        }

        gp_Pnt point;
        if (pointFromJson(profileObject.value(QStringLiteral("cutterHeadModelPosition")), &point)) {
            profile.hasCutterHeadModel = true;
            profile.cutterHeadModelPosition = point;
        }
        if (pointFromJson(profileObject.value(QStringLiteral("cutterHeadPhysicalPosition")), &point)) {
            profile.hasCutterHeadPhysical = true;
            profile.cutterHeadPhysicalPosition = point;
        }
        if (pointFromJson(profileObject.value(QStringLiteral("workpieceInstallPosition")), &point)) {
            profile.hasWorkpieceInstallPosition = true;
            profile.workpieceInstallPosition = point;
        }

        m_machineProfiles.insert(it.key(), profile);
    }
}

void CamConfig::save()
{
    ensureLoaded();

    QDir dir(configDirectoryPath());
    if (!dir.exists())
        dir.mkpath(QStringLiteral("."));

    QJsonObject root;
    root.insert(QStringLiteral("machineModelPath"), m_machineModelPath);
    root.insert(QStringLiteral("machinePreset"), m_machinePreset);
    root.insert(QStringLiteral("machineRenderQuality"), renderQualityToString(m_machineRenderQuality));

    QJsonObject toolpath;
    toolpath.insert(QStringLiteral("leadInLength"), m_leadInLength);
    toolpath.insert(QStringLiteral("normalAngle"), m_normalAngle);
    toolpath.insert(QStringLiteral("deflection"), m_deflection);
    toolpath.insert(QStringLiteral("smoothAngle"), m_smoothAngle);
    toolpath.insert(QStringLiteral("useFaceClassification"), m_useFaceClassification);
    toolpath.insert(QStringLiteral("showNormals"), m_showNormals);
    toolpath.insert(QStringLiteral("normalSampleStep"), m_normalSampleStep);
    root.insert(QStringLiteral("toolpath"), toolpath);

    QJsonObject profiles;
    for (auto it = m_machineProfiles.cbegin(); it != m_machineProfiles.cend(); ++it) {
        QJsonObject profileObject;
        QJsonObject axisOrigins;
        for (auto axisIt = it.value().axisOrigins.cbegin(); axisIt != it.value().axisOrigins.cend(); ++axisIt)
            axisOrigins.insert(axisIt.key(), pointToJson(axisIt.value()));

        profileObject.insert(QStringLiteral("axisOrigins"), axisOrigins);
        if (it.value().hasCutterHeadModel) {
            profileObject.insert(QStringLiteral("cutterHeadModelPosition"),
                                 pointToJson(it.value().cutterHeadModelPosition));
        }
        if (it.value().hasCutterHeadPhysical) {
            profileObject.insert(QStringLiteral("cutterHeadPhysicalPosition"),
                                 pointToJson(it.value().cutterHeadPhysicalPosition));
        }
        if (it.value().hasWorkpieceInstallPosition) {
            profileObject.insert(QStringLiteral("workpieceInstallPosition"),
                                 pointToJson(it.value().workpieceInstallPosition));
        }
        profiles.insert(it.key(), profileObject);
    }
    root.insert(QStringLiteral("machineProfiles"), profiles);

    QFile file(configFilePath());
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate))
        return;

    file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
}

QString CamConfig::machineModelPath() const
{
    ensureLoaded();
    return m_machineModelPath;
}

void CamConfig::setMachineModelPath(const QString& path)
{
    ensureLoaded();

    const QString normalized = path.trimmed().isEmpty()
        ? QString()
        : QFileInfo(path).absoluteFilePath();
    if (m_machineModelPath == normalized)
        return;

    m_machineModelPath = normalized;
    save();
}

QString CamConfig::machinePreset() const
{
    ensureLoaded();
    return m_machinePreset;
}

void CamConfig::setMachinePreset(const QString& preset)
{
    ensureLoaded();
    if (m_machinePreset == preset)
        return;

    m_machinePreset = preset;
    save();
}

MachineRenderQuality CamConfig::machineRenderQuality() const
{
    ensureLoaded();
    return m_machineRenderQuality;
}

void CamConfig::setMachineRenderQuality(MachineRenderQuality quality)
{
    ensureLoaded();
    if (m_machineRenderQuality == quality)
        return;

    m_machineRenderQuality = quality;
    save();
}

double CamConfig::leadInLength() const
{
    ensureLoaded();
    return m_leadInLength;
}

void CamConfig::setLeadInLength(double mm)
{
    ensureLoaded();
    if (nearlyEqual(m_leadInLength, mm))
        return;

    m_leadInLength = mm;
    save();
}

double CamConfig::normalAngle() const
{
    ensureLoaded();
    return m_normalAngle;
}

void CamConfig::setNormalAngle(double deg)
{
    ensureLoaded();
    if (nearlyEqual(m_normalAngle, deg))
        return;

    m_normalAngle = deg;
    save();
}

double CamConfig::deflection() const
{
    ensureLoaded();
    return m_deflection;
}

void CamConfig::setDeflection(double mm)
{
    ensureLoaded();
    if (nearlyEqual(m_deflection, mm))
        return;

    m_deflection = mm;
    save();
}

double CamConfig::smoothAngle() const
{
    ensureLoaded();
    return m_smoothAngle;
}

void CamConfig::setSmoothAngle(double deg)
{
    ensureLoaded();
    if (nearlyEqual(m_smoothAngle, deg))
        return;

    m_smoothAngle = deg;
    save();
}

bool CamConfig::useFaceClassification() const
{
    ensureLoaded();
    return m_useFaceClassification;
}

void CamConfig::setUseFaceClassification(bool enabled)
{
    ensureLoaded();
    if (m_useFaceClassification == enabled)
        return;

    m_useFaceClassification = enabled;
    save();
}

bool CamConfig::showNormals() const
{
    ensureLoaded();
    return m_showNormals;
}

void CamConfig::setShowNormals(bool enabled)
{
    ensureLoaded();
    if (m_showNormals == enabled)
        return;

    m_showNormals = enabled;
    save();
}

double CamConfig::normalSampleStep() const
{
    ensureLoaded();
    return m_normalSampleStep;
}

void CamConfig::setNormalSampleStep(double mm)
{
    ensureLoaded();
    if (nearlyEqual(m_normalSampleStep, mm))
        return;

    m_normalSampleStep = mm;
    save();
}

bool CamConfig::axisOriginForMachine(const QString& machinePath,
                                     const QString& axisName,
                                     gp_Pnt* outOrigin) const
{
    ensureLoaded();
    const MachineProfile* profile = profileForMachine(machinePath);
    if (!profile || !outOrigin)
        return false;

    const auto it = profile->axisOrigins.constFind(axisName);
    if (it == profile->axisOrigins.cend())
        return false;

    *outOrigin = it.value();
    return true;
}

void CamConfig::setAxisOriginForMachine(const QString& machinePath,
                                        const QString& axisName,
                                        const gp_Pnt& origin)
{
    ensureLoaded();
    if (machinePath.isEmpty() || axisName.isEmpty())
        return;

    MachineProfile* profile = mutableProfileForMachine(machinePath);
    const auto it = profile->axisOrigins.constFind(axisName);
    if (it != profile->axisOrigins.cend() && samePoint(it.value(), origin))
        return;

    profile->axisOrigins.insert(axisName, origin);
    save();
}

bool CamConfig::cutterHeadModelPositionForMachine(const QString& machinePath,
                                                  gp_Pnt* outPosition) const
{
    ensureLoaded();
    const MachineProfile* profile = profileForMachine(machinePath);
    if (!profile || !profile->hasCutterHeadModel || !outPosition)
        return false;

    *outPosition = profile->cutterHeadModelPosition;
    return true;
}

void CamConfig::setCutterHeadModelPositionForMachine(const QString& machinePath,
                                                     const gp_Pnt& position)
{
    ensureLoaded();
    if (machinePath.isEmpty())
        return;

    MachineProfile* profile = mutableProfileForMachine(machinePath);
    if (profile->hasCutterHeadModel && samePoint(profile->cutterHeadModelPosition, position))
        return;

    profile->hasCutterHeadModel = true;
    profile->cutterHeadModelPosition = position;
    save();
}

bool CamConfig::cutterHeadPhysicalPositionForMachine(const QString& machinePath,
                                                     gp_Pnt* outPosition) const
{
    ensureLoaded();
    const MachineProfile* profile = profileForMachine(machinePath);
    if (!profile || !profile->hasCutterHeadPhysical || !outPosition)
        return false;

    *outPosition = profile->cutterHeadPhysicalPosition;
    return true;
}

void CamConfig::setCutterHeadPhysicalPositionForMachine(const QString& machinePath,
                                                        const gp_Pnt& position)
{
    ensureLoaded();
    if (machinePath.isEmpty())
        return;

    MachineProfile* profile = mutableProfileForMachine(machinePath);
    if (profile->hasCutterHeadPhysical && samePoint(profile->cutterHeadPhysicalPosition, position))
        return;

    profile->hasCutterHeadPhysical = true;
    profile->cutterHeadPhysicalPosition = position;
    save();
}

bool CamConfig::workpieceInstallPositionForMachine(const QString& machinePath,
                                                   gp_Pnt* outPosition) const
{
    ensureLoaded();
    const MachineProfile* profile = profileForMachine(machinePath);
    if (!profile || !profile->hasWorkpieceInstallPosition || !outPosition)
        return false;

    *outPosition = profile->workpieceInstallPosition;
    return true;
}

void CamConfig::setWorkpieceInstallPositionForMachine(const QString& machinePath,
                                                      const gp_Pnt& position)
{
    ensureLoaded();
    if (machinePath.isEmpty())
        return;

    MachineProfile* profile = mutableProfileForMachine(machinePath);
    if (profile->hasWorkpieceInstallPosition && samePoint(profile->workpieceInstallPosition, position))
        return;

    profile->hasWorkpieceInstallPosition = true;
    profile->workpieceInstallPosition = position;
    save();
}

void CamConfig::ensureLoaded() const
{
    if (!m_loaded)
        const_cast<CamConfig*>(this)->load();
}

QString CamConfig::configDirectoryPath() const
{
    return QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("config"));
}

QString CamConfig::configFilePath() const
{
    return QDir(configDirectoryPath()).filePath(QStringLiteral("CamConfig.json"));
}

QString CamConfig::machineKey(const QString& machinePath) const
{
    if (machinePath.isEmpty())
        return QString();

    return QFileInfo(machinePath).absoluteFilePath().replace('\\', '/').toLower();
}

CamConfig::MachineProfile* CamConfig::mutableProfileForMachine(const QString& machinePath)
{
    return &m_machineProfiles[machineKey(machinePath)];
}

const CamConfig::MachineProfile* CamConfig::profileForMachine(const QString& machinePath) const
{
    const QString key = machineKey(machinePath);
    const auto it = m_machineProfiles.constFind(key);
    if (it == m_machineProfiles.cend())
        return nullptr;
    return &it.value();
}