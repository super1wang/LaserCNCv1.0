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

QString colorToHex(const QColor& c)
{
    return c.name(QColor::HexRgb);
}

QColor colorFromHex(const QString& s, const QColor& def)
{
    const QColor c(s);
    return c.isValid() ? c : def;
}

QString presetToString(RenderQualityPreset preset)
{
    switch (preset) {
    case RenderQualityPreset::Low: return QStringLiteral("low");
    case RenderQualityPreset::High: return QStringLiteral("high");
    case RenderQualityPreset::Custom: return QStringLiteral("custom");
    case RenderQualityPreset::Medium: return QStringLiteral("medium");
    }
    return QStringLiteral("medium");
}

RenderQualityPreset presetFromString(const QString& text)
{
    const QString s = text.toLower();
    if (s == QStringLiteral("low")) return RenderQualityPreset::Low;
    if (s == QStringLiteral("high")) return RenderQualityPreset::High;
    if (s == QStringLiteral("custom")) return RenderQualityPreset::Custom;
    return RenderQualityPreset::Medium;
}

QString methodToString(RenderMethod method)
{
    return method == RenderMethod::RayTracing
        ? QStringLiteral("raytracing")
        : QStringLiteral("rasterization");
}

RenderMethod methodFromString(const QString& text)
{
    return text.compare(QStringLiteral("raytracing"), Qt::CaseInsensitive) == 0
        ? RenderMethod::RayTracing
        : RenderMethod::Rasterization;
}

QString displayModeToString(StartupDisplayMode mode)
{
    return mode == StartupDisplayMode::Wireframe
        ? QStringLiteral("wireframe")
        : QStringLiteral("shaded");
}

StartupDisplayMode displayModeFromString(const QString& text)
{
    return text.compare(QStringLiteral("wireframe"), Qt::CaseInsensitive) == 0
        ? StartupDisplayMode::Wireframe
        : StartupDisplayMode::Shaded;
}

void applyPresetDefaults(RenderProfileSettings& profile, bool cam)
{
    switch (profile.qualityPreset) {
    case RenderQualityPreset::Low:
        profile.renderMethod = RenderMethod::Rasterization;
        profile.antiAliasing = false;
        profile.msaaSamples = 0;
        profile.shadows = false;
        profile.reflections = false;
        profile.adaptiveSampling = false;
        profile.frustumCulling = true;
        profile.backFaceCulling = cam;
        profile.deviationCoefficient = cam ? 0.14 : 0.10;
        profile.deviationAngle = cam ? 0.75 : 0.65;
        profile.edgeWidth = 0.6;
        profile.targetFps = 60;
        break;
    case RenderQualityPreset::High:
        profile.antiAliasing = true;
        profile.msaaSamples = cam ? 4 : 8;
        profile.shadows = !cam;
        profile.reflections = !cam;
        profile.frustumCulling = true;
        profile.backFaceCulling = false;
        profile.deviationCoefficient = cam ? 0.025 : 0.01;
        profile.deviationAngle = cam ? 0.20 : 0.12;
        profile.edgeWidth = 1.1;
        profile.targetFps = cam ? 45 : 30;
        break;
    case RenderQualityPreset::Medium:
        profile.renderMethod = RenderMethod::Rasterization;
        profile.antiAliasing = true;
        profile.msaaSamples = 2;
        profile.shadows = false;
        profile.reflections = false;
        profile.adaptiveSampling = false;
        profile.frustumCulling = true;
        profile.backFaceCulling = cam;
        profile.deviationCoefficient = cam ? 0.06 : 0.05;
        profile.deviationAngle = cam ? 0.40 : 0.35;
        profile.edgeWidth = 0.8;
        profile.targetFps = 60;
        break;
    case RenderQualityPreset::Custom:
        break;
    }
}

void readProfile(const toml::value& table, RenderProfileSettings& profile)
{
    using namespace toml_io;
    profile.defaultDisplayMode = displayModeFromString(
        get_qstring(table, "default_display_mode", displayModeToString(profile.defaultDisplayMode)));
    profile.qualityPreset = presetFromString(
        get_qstring(table, "quality", presetToString(profile.qualityPreset)));
    profile.renderMethod = methodFromString(
        get_qstring(table, "render_method", methodToString(profile.renderMethod)));
    profile.material = get_qstring(table, "material", profile.material);
    profile.antiAliasing = get_bool(table, "anti_aliasing", profile.antiAliasing);
    profile.msaaSamples = get_int(table, "msaa_samples", profile.msaaSamples);
    profile.shadows = get_bool(table, "shadows", profile.shadows);
    profile.reflections = get_bool(table, "reflections", profile.reflections);
    profile.adaptiveSampling = get_bool(table, "adaptive_sampling", profile.adaptiveSampling);
    profile.frustumCulling = get_bool(table, "frustum_culling", profile.frustumCulling);
    profile.backFaceCulling = get_bool(table, "back_face_culling", profile.backFaceCulling);
    profile.geometryMerge = get_bool(table, "geometry_merge", profile.geometryMerge);
    profile.proxyGeometry = get_bool(table, "proxy_geometry", profile.proxyGeometry);
    profile.lowLodWhileMoving = get_bool(table, "low_lod_while_moving", profile.lowLodWhileMoving);
    profile.disableHeavyEffectsDuringSimulation = get_bool(
        table, "disable_heavy_effects_during_simulation", profile.disableHeavyEffectsDuringSimulation);
    profile.ambientLight = get_double(table, "ambient_light", profile.ambientLight);
    profile.deviationCoefficient = get_double(table, "deviation_coefficient", profile.deviationCoefficient);
    profile.deviationAngle = get_double(table, "deviation_angle", profile.deviationAngle);
    profile.edgeWidth = get_double(table, "edge_width", profile.edgeWidth);
    profile.renderResolutionScale = get_double(table, "render_resolution_scale", profile.renderResolutionScale);
    profile.raytracingDepth = get_int(table, "raytracing_depth", profile.raytracingDepth);
    profile.rayTracingTileSize = get_int(table, "raytracing_tile_size", profile.rayTracingTileSize);
    profile.rayTracingTileCount = get_int(table, "raytracing_tile_count", profile.rayTracingTileCount);
    profile.targetFps = get_int(table, "target_fps", profile.targetFps);
}

void writeProfile(toml::value& table, const RenderProfileSettings& profile)
{
    using namespace toml_io;
    table["default_display_mode"] = qs(displayModeToString(profile.defaultDisplayMode));
    table["quality"] = qs(presetToString(profile.qualityPreset));
    table["render_method"] = qs(methodToString(profile.renderMethod));
    table["material"] = qs(profile.material);
    table["anti_aliasing"] = profile.antiAliasing;
    table["msaa_samples"] = profile.msaaSamples;
    table["shadows"] = profile.shadows;
    table["reflections"] = profile.reflections;
    table["adaptive_sampling"] = profile.adaptiveSampling;
    table["frustum_culling"] = profile.frustumCulling;
    table["back_face_culling"] = profile.backFaceCulling;
    table["geometry_merge"] = profile.geometryMerge;
    table["proxy_geometry"] = profile.proxyGeometry;
    table["low_lod_while_moving"] = profile.lowLodWhileMoving;
    table["disable_heavy_effects_during_simulation"] = profile.disableHeavyEffectsDuringSimulation;
    table["ambient_light"] = profile.ambientLight;
    table["deviation_coefficient"] = profile.deviationCoefficient;
    table["deviation_angle"] = profile.deviationAngle;
    table["edge_width"] = profile.edgeWidth;
    table["render_resolution_scale"] = profile.renderResolutionScale;
    table["raytracing_depth"] = profile.raytracingDepth;
    table["raytracing_tile_size"] = profile.rayTracingTileSize;
    table["raytracing_tile_count"] = profile.rayTracingTileCount;
    table["target_fps"] = profile.targetFps;
}
} // namespace

QHash<QString, QColor> defaultMachineAxisColors()
{
    return {
        {QStringLiteral("BASE"), QColor::fromRgbF(0.62, 0.64, 0.68)},
        {QStringLiteral("X"),    QColor::fromRgbF(0.90, 0.27, 0.18)},
        {QStringLiteral("Y"),    QColor::fromRgbF(0.14, 0.66, 0.28)},
        {QStringLiteral("Z"),    QColor::fromRgbF(0.18, 0.48, 0.94)},
        {QStringLiteral("A"),    QColor::fromRgbF(0.93, 0.60, 0.08)},
        {QStringLiteral("B"),    QColor::fromRgbF(0.10, 0.70, 0.70)},
        {QStringLiteral("C"),    QColor::fromRgbF(0.76, 0.23, 0.79)},
    };
}

AppSettings::AppSettings()
{
    Q_ASSERT_X(!g_instance, "AppSettings",
               "second AppSettings instance — must be Kernel-owned only");
    g_instance = this;
    colors.machineAxisColors = defaultMachineAxisColors();
    cadViewRendering.qualityPreset = RenderQualityPreset::Medium;
    camViewRendering.qualityPreset = RenderQualityPreset::Medium;
    camViewRendering.backFaceCulling = true;
    camViewRendering.lowLodWhileMoving = true;
    camViewRendering.disableHeavyEffectsDuringSimulation = true;
    applyPresetDefaults(cadViewRendering, false);
    applyPresetDefaults(camViewRendering, true);
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
        unitSystem  = get_qstring(g, "units",    unitSystem);
        recentLimit = get_int(g,    "recent_limit", recentLimit);
    }

    if (root.contains("window") && root.at("window").is_table()) {
        const auto& w = root.at("window");
        windowX         = get_int(w,  "x",         windowX);
        windowY         = get_int(w,  "y",         windowY);
        windowWidth     = get_int(w,  "width",     windowWidth);
        windowHeight    = get_int(w,  "height",    windowHeight);
        windowMaximized = get_bool(w, "maximized", windowMaximized);
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

    if (root.contains("rendering_cad") && root.at("rendering_cad").is_table())
        readProfile(root.at("rendering_cad"), cadViewRendering);
    if (root.contains("rendering_cam") && root.at("rendering_cam").is_table())
        readProfile(root.at("rendering_cam"), camViewRendering);

    // 兼容上一版 [rendering] 字段。
    if (root.contains("rendering") && root.at("rendering").is_table()) {
        const auto& r = root.at("rendering");
        cadViewRendering.defaultDisplayMode = static_cast<StartupDisplayMode>(get_int(r, "display_mode", 1));
        camViewRendering.defaultDisplayMode = cadViewRendering.defaultDisplayMode;
        cadViewRendering.qualityPreset = static_cast<RenderQualityPreset>(get_int(r, "quality_level", 1));
        camViewRendering.qualityPreset = cadViewRendering.qualityPreset;
        colors.workpieceColor = colorFromHex(get_qstring(r, "file_color", colorToHex(colors.workpieceColor)), colors.workpieceColor);
        colors.camBackgroundColor = colorFromHex(get_qstring(r, "machine_default_color", colorToHex(colors.camBackgroundColor)), colors.camBackgroundColor);
        if (r.contains("axis_colors") && r.at("axis_colors").is_table()) {
            const auto& ac = r.at("axis_colors");
            for (const auto& kv : ac.as_table()) {
                if (kv.second.is_string()) {
                    colors.machineAxisColors.insert(QString::fromStdString(kv.first),
                        colorFromHex(QString::fromStdString(kv.second.as_string()), QColor()));
                }
            }
        }
    }

    if (root.contains("colors") && root.at("colors").is_table()) {
        const auto& c = root.at("colors");
        colors.workpieceColor = colorFromHex(get_qstring(c, "workpiece", colorToHex(colors.workpieceColor)), colors.workpieceColor);
        colors.cadBackgroundColor = colorFromHex(get_qstring(c, "cad_background", colorToHex(colors.cadBackgroundColor)), colors.cadBackgroundColor);
        colors.camBackgroundColor = colorFromHex(get_qstring(c, "cam_background", colorToHex(colors.camBackgroundColor)), colors.camBackgroundColor);
        colors.selectionColor = colorFromHex(get_qstring(c, "selection", colorToHex(colors.selectionColor)), colors.selectionColor);
        colors.hoverColor = colorFromHex(get_qstring(c, "hover", colorToHex(colors.hoverColor)), colors.hoverColor);
        colors.treeSelectionColor = colorFromHex(get_qstring(c, "tree_selection", colorToHex(colors.treeSelectionColor)), colors.treeSelectionColor);
        colors.highlightDisplayMode = get_int(c, "highlight_display_mode", colors.highlightDisplayMode);
        colors.highlightLineWidth = get_double(c, "highlight_line_width", colors.highlightLineWidth);
        if (c.contains("axis_colors") && c.at("axis_colors").is_table()) {
            const auto& ac = c.at("axis_colors");
            for (const auto& kv : ac.as_table()) {
                if (kv.second.is_string()) {
                    colors.machineAxisColors.insert(QString::fromStdString(kv.first),
                        colorFromHex(QString::fromStdString(kv.second.as_string()), QColor()));
                }
            }
        }
    }

    // 兼容上一版 [highlight] 字段。
    if (root.contains("highlight") && root.at("highlight").is_table()) {
        const auto& h = root.at("highlight");
        colors.selectionColor = colorFromHex(get_qstring(h, "selection_color", colorToHex(colors.selectionColor)), colors.selectionColor);
        colors.hoverColor = colorFromHex(get_qstring(h, "hover_color", colorToHex(colors.hoverColor)), colors.hoverColor);
        colors.treeSelectionColor = colorFromHex(get_qstring(h, "tree_color", colorToHex(colors.treeSelectionColor)), colors.treeSelectionColor);
        colors.highlightDisplayMode = get_int(h, "display_mode", colors.highlightDisplayMode);
        colors.highlightLineWidth = get_double(h, "line_width", colors.highlightLineWidth);
    }
}

void AppSettings::writeTo(toml::value& root) const
{
    using namespace toml_io;

    toml::value general(toml::table{});
    general["theme"] = qs(theme);
    general["language"] = qs(language);
    general["units"] = qs(unitSystem);
    general["recent_limit"] = recentLimit;
    root["general"] = general;

    toml::value window(toml::table{});
    window["x"] = windowX;
    window["y"] = windowY;
    window["width"] = windowWidth;
    window["height"] = windowHeight;
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

    toml::value cad(toml::table{});
    writeProfile(cad, cadViewRendering);
    root["rendering_cad"] = cad;

    toml::value cam(toml::table{});
    writeProfile(cam, camViewRendering);
    root["rendering_cam"] = cam;

    toml::value colorTable(toml::table{});
    colorTable["workpiece"] = qs(colorToHex(colors.workpieceColor));
    colorTable["cad_background"] = qs(colorToHex(colors.cadBackgroundColor));
    colorTable["cam_background"] = qs(colorToHex(colors.camBackgroundColor));
    colorTable["selection"] = qs(colorToHex(colors.selectionColor));
    colorTable["hover"] = qs(colorToHex(colors.hoverColor));
    colorTable["tree_selection"] = qs(colorToHex(colors.treeSelectionColor));
    colorTable["highlight_display_mode"] = colors.highlightDisplayMode;
    colorTable["highlight_line_width"] = colors.highlightLineWidth;
    toml::value axisColors(toml::table{});
    for (auto it = colors.machineAxisColors.cbegin(); it != colors.machineAxisColors.cend(); ++it)
        axisColors[qs(it.key())] = qs(colorToHex(it.value()));
    colorTable["axis_colors"] = axisColors;
    root["colors"] = colorTable;
}

} // namespace lcnc
