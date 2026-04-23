#include "core/settings/toml_config.h"

#include "core/logging/logger.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QTextStream>

#include <fstream>
#include <sstream>

namespace lcnc {

bool TomlConfig::load(const QString& path)
{
    m_filePath = path;

    QFileInfo info(path);
    if (!info.exists()) {
        // Missing file is fine: subclass keeps its built-in defaults.
        LCNC_INFO(LogCode::SettingsLoaded,
                  "{} file missing at '{}', using defaults",
                  configName(), path.toStdString());
        return true;
    }

    try {
        const auto root = toml::parse(path.toStdString());
        readFrom(root);
        LCNC_INFO(LogCode::SettingsLoaded,
                  "{} loaded from '{}'",
                  configName(), path.toStdString());
        return true;
    } catch (const toml::syntax_error& e) {
        LCNC_ERR(LogCode::SettingsParseFailed,
                 "{} parse error in '{}': {}",
                 configName(), path.toStdString(), e.what());
        return false;
    } catch (const std::exception& e) {
        LCNC_ERR(LogCode::SettingsParseFailed,
                 "{} load failed for '{}': {}",
                 configName(), path.toStdString(), e.what());
        return false;
    }
}

bool TomlConfig::save(const QString& path) const
{
    m_filePath = path;

    QFileInfo info(path);
    QDir().mkpath(info.absolutePath());

    try {
        toml::value root(toml::table{});
        writeTo(root);

        std::ofstream out(path.toStdString(), std::ios::binary | std::ios::trunc);
        if (!out.is_open()) {
            LCNC_ERR(LogCode::SettingsSaveFailed,
                     "{} could not open '{}' for write",
                     configName(), path.toStdString());
            return false;
        }
        out << toml::format(root);
        return true;
    } catch (const std::exception& e) {
        LCNC_ERR(LogCode::SettingsSaveFailed,
                 "{} write failed for '{}': {}",
                 configName(), path.toStdString(), e.what());
        return false;
    }
}

} // namespace lcnc
