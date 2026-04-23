#pragma once

#include <QString>
#include <toml.hpp>

namespace lcnc {

/**
 * @brief Base helper for TOML-backed configuration files.
 *
 * Subclasses override @ref readFrom and @ref writeTo to map the in-memory
 * fields to a @c toml::value. The base class handles file IO and error
 * reporting through the lcnc::Logger facade.
 *
 * Usage:
 * @code
 *   AppSettings s;
 *   s.load(QStringLiteral("config/mainwindow.toml"));
 *   s.theme = QStringLiteral("dark");
 *   s.save(QStringLiteral("config/mainwindow.toml"));
 * @endcode
 */
class TomlConfig
{
public:
    virtual ~TomlConfig() = default;

    /// Returns the path the config was last loaded from / saved to.
    QString filePath() const { return m_filePath; }

    /**
     * @brief Load and parse the file. Missing file => use defaults (returns true).
     * @return false only on hard parse errors (file exists but is malformed).
     */
    bool load(const QString& path);

    /// Serialize current state. Creates parent directory if missing.
    bool save(const QString& path) const;

protected:
    /// Subclass: pull values out of @p root into your members.
    virtual void readFrom(const toml::value& root) = 0;

    /// Subclass: write current member values into @p root.
    virtual void writeTo(toml::value& root) const = 0;

    /// Subclass label used in log messages (e.g. "AppSettings").
    virtual const char* configName() const = 0;

private:
    mutable QString m_filePath;
};

// ─────────────────────────────────────────────────────────────────────────────
// Small TOML helpers (header-only) used by every concrete settings class.
// They tolerate missing keys and type mismatches by falling back to the default.
// ─────────────────────────────────────────────────────────────────────────────

namespace toml_io {

inline bool get_bool(const toml::value& tbl, const std::string& key, bool def)
{
    if (!tbl.is_table() || !tbl.contains(key)) return def;
    const auto& v = tbl.at(key);
    return v.is_boolean() ? v.as_boolean() : def;
}

inline int get_int(const toml::value& tbl, const std::string& key, int def)
{
    if (!tbl.is_table() || !tbl.contains(key)) return def;
    const auto& v = tbl.at(key);
    return v.is_integer() ? static_cast<int>(v.as_integer()) : def;
}

inline double get_double(const toml::value& tbl, const std::string& key, double def)
{
    if (!tbl.is_table() || !tbl.contains(key)) return def;
    const auto& v = tbl.at(key);
    if (v.is_floating()) return v.as_floating();
    if (v.is_integer())  return static_cast<double>(v.as_integer());
    return def;
}

inline QString get_qstring(const toml::value& tbl, const std::string& key, const QString& def)
{
    if (!tbl.is_table() || !tbl.contains(key)) return def;
    const auto& v = tbl.at(key);
    return v.is_string() ? QString::fromStdString(v.as_string()) : def;
}

inline std::string qs(const QString& s) { return s.toStdString(); }

} // namespace toml_io
} // namespace lcnc
