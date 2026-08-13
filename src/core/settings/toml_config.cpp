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

    // Do not hand a Windows QString path to toml11's filename overload.
    // Loading the bytes through QFile keeps the Qt/UTF-16 path boundary in one
    // place, and parsing an in-memory stream avoids a second CRT file handle.
    // This is also important at startup: a malformed/unavailable path must be
    // reported as a recoverable settings failure rather than aborting module
    // construction inside QFileInfo/std::ifstream.
    QFile file(path);
    if (!file.exists()) {
        // Missing file is fine: subclass keeps its built-in defaults.
        LCNC_INFO(LogCode::SettingsLoaded,
                  "{} file missing at '{}', using defaults",
                  configName(), path.toStdString());
        return true;
    }

    if (!file.open(QIODevice::ReadOnly)) {
        LCNC_ERR(LogCode::SettingsParseFailed,
                 "{} could not open '{}': {}",
                 configName(), path.toStdString(), file.errorString().toStdString());
        return false;
    }

    const QByteArray bytes = file.readAll();
    if (file.error() != QFileDevice::NoError) {
        LCNC_ERR(LogCode::SettingsParseFailed,
                 "{} could not read '{}': {}",
                 configName(), path.toStdString(), file.errorString().toStdString());
        return false;
    }

    try {
        std::istringstream input(std::string(bytes.constData(),
                                             static_cast<std::size_t>(bytes.size())));
        const auto root = toml::parse(input, path.toUtf8().toStdString());
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
