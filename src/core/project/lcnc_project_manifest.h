#pragma once

#include "core/project/project_save_options.h"
#include "core/settings/toml_config.h"

#include <QString>

namespace lcnc {

/**
 * @brief TOML manifest for a .lcnc project directory package.
 */
class LcncProjectManifest : public TomlConfig
{
public:
    static constexpr int kCurrentFormatVersion = 4;

    QString schema{QStringLiteral("lcnc.project")};
    int formatVersion{kCurrentFormatVersion};
    QString projectName;
    QString documentName;
    QString sourceFilePath;
    QString createdUtc;
    QString savedUtc;
    QString softwareVersion;
    QString machineConfigurationFingerprint;
    QString configurationSchemaVersion;
    QString toolpathAlgorithmVersion;
    QString toolSnapshotPath{QStringLiteral("tools.toml")};
    QString projectXcafPath{QStringLiteral("project.xbf")};   ///< Legacy v1 migration input only.
    QString workpieceXcafPath{QStringLiteral("workpiece.xbf")}; ///< v4 unified workpiece + CAM entities.
    QString camCacheDirectory{QStringLiteral("cam/cache")};
    ProjectSaveOptions saveOptions;

    /// Returns true when the manifest can be consumed by this build.  Legacy
    /// formats are intentionally reserved for the offline migration utility.
    bool validate(QString* errorMsg = nullptr, bool allowLegacyFormat = false) const;

protected:
    void readFrom(const toml::value& root) override;
    void writeTo(toml::value& root) const override;
    const char* configName() const override { return "LcncProjectManifest"; }
};

} // namespace lcnc
