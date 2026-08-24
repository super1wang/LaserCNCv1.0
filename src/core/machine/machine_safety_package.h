#pragma once

#include <QByteArray>
#include <QString>

namespace lcnc {

struct MachineSafetyPackageManifest
{
    static constexpr int kCurrentFormatVersion = 1;

    QString schema{QStringLiteral("lcnc.machine-safety-package")};
    int formatVersion{kCurrentFormatVersion};
    QString createdUtc;
    QString softwareVersion;
    QString modelPath{QStringLiteral("machine/machine.step")};
    QString safetyIndexPath{QStringLiteral("safety/machine.lmsi")};
    QByteArray modelSha256;
    QByteArray safetyIndexSha256;
    QByteArray indexContentSha256;
    QByteArray safetyConfigurationSha256;
    QByteArray runtimeConfigurationSha256;
    QByteArray packageKeySha256;
};

struct MachineSafetyPackageLoadResult
{
    MachineSafetyPackageManifest manifest;
    QString packagePath;
    QString modelPath;
    QString safetyIndexPath;
};

/**
 * @brief Reader/writer for the immutable LaserCNC machine safety package.
 *
 * A .lmsp archive contains one machine model and exactly one .lmsi file.
 * The manifest binds both resources by SHA-256 and also binds the index's
 * configuration/content fingerprints. Any modification is therefore rejected
 * before the package can become execution eligible.
 */
class MachineSafetyPackage final
{
public:
    static bool isPackagePath(const QString& path);
    static QString normalizedPackagePath(const QString& path);

    static bool create(const QString& machineModelPath,
                       const QString& safetyIndexPath,
                       const QString& packagePath,
                       const QByteArray& runtimeConfigurationSha256 = {},
                       QString* errorMessage = nullptr);

    static bool extractAndValidate(const QString& packagePath,
                                   const QString& targetDirectory,
                                   MachineSafetyPackageLoadResult* result,
                                   QString* errorMessage = nullptr);
};

} // namespace lcnc
