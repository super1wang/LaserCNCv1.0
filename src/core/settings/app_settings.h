#pragma once

#include "core/settings/toml_config.h"

#include <QColor>
#include <QHash>
#include <QString>
#include <QStringList>

namespace lcnc {

/**
 * @brief 渲染质量预设级别。
 */
enum class RenderQualityPreset {
    Low = 0,
    Medium = 1,
    High = 2,
    Custom = 3
};

/**
 * @brief OCC 渲染方法。
 */
enum class RenderMethod {
    Rasterization = 0,
    RayTracing = 1
};

/**
 * @brief 默认显示模式，仅用于程序启动或新 view 创建。
 */
enum class StartupDisplayMode {
    Wireframe = 0,
    Shaded = 1
};

/**
 * @brief 打开新文件时的项目工作区策略。
 */
enum class DocumentOpenMode {
    SingleDocument = 0,
    MultiDocument = 1
};

/**
 * @brief 单个视图族（CAD View / CAM View）的渲染参数配置。
 *
 * 此结构只保存可序列化的轻量值，不包含 OCC 类型，便于放在 core/settings。
 */
struct RenderProfileSettings {
    StartupDisplayMode defaultDisplayMode = StartupDisplayMode::Shaded;
    RenderQualityPreset qualityPreset = RenderQualityPreset::Medium;
    RenderMethod renderMethod = RenderMethod::Rasterization;
    QString material = QStringLiteral("plastic");

    bool antiAliasing = true;
    int msaaSamples = 2;                 ///< 0/2/4/8，只有 antiAliasing=true 时应用
    bool shadows = false;
    bool reflections = false;
    bool adaptiveSampling = false;
    bool frustumCulling = true;
    bool backFaceCulling = false;
    bool geometryMerge = false;          ///< 当前仅持久化；重几何合并由 CAM 压缩路径处理
    bool proxyGeometry = false;          ///< 当前仅持久化；用于后续机台代理模型策略
    bool lowLodWhileMoving = true;
    bool disableHeavyEffectsDuringSimulation = true;

    double ambientLight = 0.30;
    double deviationCoefficient = 0.05;
    double deviationAngle = 0.35;
    double edgeWidth = 0.8;
    double renderResolutionScale = 1.0;
    int raytracingDepth = 2;
    int rayTracingTileSize = 32;
    int rayTracingTileCount = 64;
    int targetFps = 60;
};

/**
 * @brief 模型、背景、分轴与选择高亮颜色配置。
 */
struct ColorSettings {
    QColor workpieceColor = QColor(200, 200, 210);
    QColor backgroundColor = QColor(42, 48, 58);
    QColor cadBackgroundColor = QColor(42, 48, 58);
    QColor camBackgroundColor = QColor(42, 48, 58);
    QColor selectionColor = QColor(255, 0, 0);
    QColor hoverColor = QColor(255, 165, 0);
    QColor treeSelectionColor = QColor(42, 111, 219);
    int highlightDisplayMode = -1;
    double highlightLineWidth = 2.0;
    QHash<QString, QColor> machineAxisColors;
};

struct ViewStateSettings {
    int displayMode = 1;              ///< AIS_Shaded by default
    bool faceBoundary = false;        ///< shaded-with-edges
    bool worldAxesVisible = false;
    bool rotaryAxisGuidesVisible = true;
    bool cutterHeadGuideVisible = true;
    bool machineModelVisible = false;
};

/**
 * @brief Application shell settings (mainwindow.toml).
 *
 * Theme, recent-files list, main-window geometry and view rendering defaults.
 */
class AppSettings : public TomlConfig
{
public:
    /// Construct directly. Normally only created once by lcnc::Kernel during registerCoreServices().
    AppSettings();
    ~AppSettings() override;

    /// Convenience: load from <exeDir>/config/mainwindow.toml.
    bool loadDefault();

    /// Convenience: save back to the path used by loadDefault().
    bool saveDefault() const;

    QString     theme       = QStringLiteral("light");   ///< "light" | "dark"
    QString     language    = QStringLiteral("zh_CN");
    QString     unitSystem  = QStringLiteral("mm");      ///< "mm" | "inch"
    DocumentOpenMode documentOpenMode = DocumentOpenMode::MultiDocument;
    QStringList recentFiles;                             ///< most-recent first
    int         recentLimit = 10;

    int  windowX        = -1;     ///< -1 means "use system default"
    int  windowY        = -1;
    int  windowWidth    = 1600;
    int  windowHeight   = 1000;
    bool windowMaximized = true;

    /// 微内核模块开关：出现在本列表中的模块 id（如 "process"）不会被 main.cpp 加入 Kernel。
    QStringList disabledModules;

    /// CAD 文件视图渲染默认参数。
    RenderProfileSettings cadViewRendering;
    /// CAM/机台视图渲染默认参数。
    RenderProfileSettings camViewRendering;
    /// 模型、背景和高亮颜色配置。
    ColorSettings colors;
    /// Ribbon 视图页的运行时显示状态，启动时恢复。
    ViewStateSettings viewState;

    /// 快捷查询：该模块是否被禁用（大小写敏感）。
    bool isModuleDisabled(const QString& moduleId) const;

protected:
    void readFrom(const toml::value& root) override;
    void writeTo(toml::value& root)  const override;
    const char* configName() const override { return "AppSettings"; }
};

/// 返回内置机台分轴默认颜色。
QHash<QString, QColor> defaultMachineAxisColors();

} // namespace lcnc
