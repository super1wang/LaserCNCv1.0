#pragma once

#include "core/settings/toml_config.h"

#include <QString>
#include <QStringList>

namespace lcnc {

/**
 * @brief Application shell settings (mainwindow.toml).
 *
 * Theme, recent-files list and main-window geometry. Anything that is
 * not specific to CAM/Process logic.
 */
class AppSettings : public TomlConfig
{
public:
    /// Construct directly. Normally only created once by lcnc::Kernel
    /// during registerCoreServices(); ctor sets the global instance.
    AppSettings();
    ~AppSettings() override;

    /// Process-wide accessor (asserts Kernel has constructed it).

    /// Convenience: load from <exeDir>/config/mainwindow.toml.
    bool loadDefault();

    /// Convenience: save back to the path used by loadDefault().
    bool saveDefault() const;

    // ── Fields ──────────────────────────────────────────────────────
    QString     theme       = QStringLiteral("light");   ///< "light" | "dark"
    QString     language    = QStringLiteral("zh_CN");
    QStringList recentFiles;                             ///< most-recent first
    int         recentLimit = 10;

    // [window]
    int  windowX        = -1;     ///< -1 means "use system default"
    int  windowY        = -1;
    int  windowWidth    = 1600;
    int  windowHeight   = 1000;
    bool windowMaximized = true;

    // [modules]
    /// 微内核模块开关：出现在本列表中的模块 id（如 "process"）不会被
    /// main.cpp 加入 Kernel。默认为空（全部启用）。
    /// 例：
    /// @code
    /// [modules]
    /// disabled = ["process"]
    /// @endcode
    QStringList disabledModules;

    /// 快捷查询：该模块是否被禁用（大小写敏感）。
    bool isModuleDisabled(const QString& moduleId) const;

protected:
    void readFrom(const toml::value& root) override;
    void writeTo(toml::value& root)  const override;
    const char* configName() const override { return "AppSettings"; }
};

} // namespace lcnc
