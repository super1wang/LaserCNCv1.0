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
    static constexpr int kCurrentFormatVersion = 2;

    QString schema{QStringLiteral("lcnc.project")};
    int formatVersion{kCurrentFormatVersion};
    QString projectName;
    QString documentName;
    QString sourceFilePath;
    QString createdUtc;
    QString savedUtc;
    QString projectXcafPath{QStringLiteral("project.xbf")};   ///< v1 legacy; v2 uses workpieceXcafPath
    QString workpieceXcafPath{QStringLiteral("workpiece.xbf")}; ///< v2 workpiece only
    QString camCacheDirectory{QStringLiteral("cam/cache")};
    ProjectSaveOptions saveOptions;

    /// Returns true when the manifest can be consumed by this build.
    bool validate(QString* errorMsg = nullptr) const;

protected:
    void readFrom(const toml::value& root) override;
    void writeTo(toml::value& root) const override;
    const char* configName() const override { return "LcncProjectManifest"; }
};

} // namespace lcnc