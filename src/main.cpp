#include <QApplication>
#include <QCoreApplication>
#include <QDir>
#include <QSurfaceFormat>

#include "app/main_window.h"
#include "core/document/lcnc_application.h"
#include "core/kernel/kernel.h"
#include "core/logging/logger.h"
#include "core/settings/app_settings.h"
#include "modules/cad/cad_module.h"
#include "modules/cam/cam_module.h"
#include "modules/process/process_module.h"

int main(int argc, char* argv[])
{
    // High DPI / fractional scaling support
    QApplication::setHighDpiScaleFactorRoundingPolicy(
        Qt::HighDpiScaleFactorRoundingPolicy::PassThrough);

    // OpenGL surface format for OCC
    QSurfaceFormat fmt;
    fmt.setDepthBufferSize(24);
    fmt.setStencilBufferSize(8);
    fmt.setSamples(4);
    fmt.setVersion(4, 5);
    fmt.setProfile(QSurfaceFormat::CoreProfile);
    QSurfaceFormat::setDefaultFormat(fmt);

    QApplication app(argc, argv);
    app.setApplicationName("LaserCNC");
    app.setApplicationVersion("1.0.0");
    app.setApplicationDisplayName("五轴激光加工CAM软件");
    app.setOrganizationName("LaserCNC");

    // ── Core infrastructure ──────────────────────────────────────────────
    const QString exeDir = QCoreApplication::applicationDirPath();
    lcnc::Logger::init(QDir(exeDir).absoluteFilePath(QStringLiteral("logs")));

    LCNC_INFO(lcnc::LogCode::Generic,
              "{} {} starting up",
              app.applicationName().toStdString(),
              app.applicationVersion().toStdString());

    // ── Kernel：注册核心服务 + 加载业务模块 ────────────────────────────
    //    Kernel 直接持有 LcncApplication / GuiApplication / TaskManager /
    //    AppSettings 实例，不再有 XxxClass::instance() 懒加载单例。
    //    模块依赖：cad ← cam ← process（CAM 共享 LcncApplication 的机台
    //    文档；Process 使用 CAM 的轴定义）。Kernel 内部用 Kahn 拓扑排序，
    //    保证依赖在前。
    //    [modules].disabled = [...] 可在 mainwindow.toml 中关闭某些模块；
    //    被关闭的模块连同依赖它的下游模块都不会加入 Kernel。
    lcnc::Kernel kernel;
    kernel.registerCoreServices();
    kernel.appSettings()->loadDefault();   // mainwindow.toml

    auto* settings = kernel.appSettings();
    auto tryAdd = [&](const QString& id,
                      const QStringList& deps,
                      std::unique_ptr<lcnc::IModule> mod) {
        if (settings->isModuleDisabled(id)) {
            LCNC_INFO(lcnc::LogCode::Generic,
                      "Module '{}' disabled by config; skipped",
                      id.toStdString());
            return;
        }
        for (const auto& d : deps) {
            if (settings->isModuleDisabled(d)) {
                LCNC_WARN(lcnc::LogCode::InternalUnexpectedState,
                          "Module '{}' skipped: depends on disabled '{}'",
                          id.toStdString(), d.toStdString());
                return;
            }
        }
        kernel.addModule(std::move(mod));
    };

    tryAdd("cad",     {},          std::make_unique<CadModule>());
    tryAdd("cam",     {"cad"},     std::make_unique<CamModule>());
    tryAdd("process", {"cam"},     std::make_unique<ProcessModule>());

    if (!kernel.bootstrap()) {
        LCNC_CRIT(lcnc::LogCode::InternalUnexpectedState,
                  "Kernel bootstrap failed; aborting startup");
        return -1;
    }
    LCNC_INFO(lcnc::LogCode::Generic,
              "Kernel ready: services={}",
              kernel.services().size());

    MainWindow mainWin;
    mainWin.show();

    const int rc = app.exec();

    // 注意：Kernel 在 MainWindow 之后销毁，但模块在销毁前先 stop。
    kernel.appSettings()->saveDefault();
    kernel.shutdown();

    LCNC_INFO(lcnc::LogCode::Generic, "Shutdown rc={}", rc);
    lcnc::Logger::shutdown();
    return rc;
}
