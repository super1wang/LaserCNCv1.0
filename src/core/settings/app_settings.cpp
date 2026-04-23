#include "core/settings/app_settings.h"

#include <QCoreApplication>
#include <QDir>

namespace lcnc {

namespace {
QString defaultPath()
{
    const QString exeDir = QCoreApplication::applicationDirPath();
    return QDir(exeDir).absoluteFilePath(QStringLiteral("config/mainwindow.toml"));
}
AppSettings* g_instance = nullptr;
}

AppSettings::AppSettings()
{
    Q_ASSERT_X(!g_instance, "AppSettings",
               "second AppSettings instance — must be Kernel-owned only");
    g_instance = this;
}

AppSettings::~AppSettings()
{
    if (g_instance == this) g_instance = nullptr;
}

bool AppSettings::loadDefault() { return load(defaultPath()); }
bool AppSettings::saveDefault() const { return save(defaultPath()); }

bool AppSettings::isModuleDisabled(const QString& moduleId) const
{
    return disabledModules.contains(moduleId);
}

void AppSettings::readFrom(const toml::value& root)
{
    using namespace toml_io;

    if (root.contains("general") && root.at("general").is_table()) {
        const auto& g = root.at("general");
        theme       = get_qstring(g, "theme",    theme);
        language    = get_qstring(g, "language", language);
        recentLimit = get_int(g,    "recent_limit", recentLimit);
    }

    if (root.contains("window") && root.at("window").is_table()) {
        const auto& w = root.at("window");
        windowX         = get_int(w,    "x",         windowX);
        windowY         = get_int(w,    "y",         windowY);
        windowWidth     = get_int(w,    "width",     windowWidth);
        windowHeight    = get_int(w,    "height",    windowHeight);
        windowMaximized = get_bool(w,   "maximized", windowMaximized);
    }

    recentFiles.clear();
    if (root.contains("recent") && root.at("recent").is_table()) {
        const auto& r = root.at("recent");
        if (r.contains("files") && r.at("files").is_array()) {
            for (const auto& v : r.at("files").as_array()) {
                if (v.is_string())
                    recentFiles << QString::fromStdString(v.as_string());
            }
        }
    }

    disabledModules.clear();
    if (root.contains("modules") && root.at("modules").is_table()) {
        const auto& m = root.at("modules");
        if (m.contains("disabled") && m.at("disabled").is_array()) {
            for (const auto& v : m.at("disabled").as_array()) {
                if (v.is_string())
                    disabledModules << QString::fromStdString(v.as_string());
            }
        }
    }
}

void AppSettings::writeTo(toml::value& root) const
{
    using namespace toml_io;

    toml::value general(toml::table{});
    general["theme"]        = qs(theme);
    general["language"]     = qs(language);
    general["recent_limit"] = recentLimit;
    root["general"] = general;

    toml::value window(toml::table{});
    window["x"]         = windowX;
    window["y"]         = windowY;
    window["width"]     = windowWidth;
    window["height"]    = windowHeight;
    window["maximized"] = windowMaximized;
    root["window"] = window;

    toml::array files;
    for (const auto& f : recentFiles)
        files.emplace_back(qs(f));
    toml::value recent(toml::table{});
    recent["files"] = files;
    root["recent"] = recent;

    toml::array disabled;
    for (const auto& m : disabledModules)
        disabled.emplace_back(qs(m));
    toml::value modules(toml::table{});
    modules["disabled"] = disabled;
    root["modules"] = modules;
}

} // namespace lcnc
