#include <QApplication>
#include <QCoreApplication>
#include <QDir>
#include <QSurfaceFormat>

#include "app/main_window.h"
#include "core/kernel/kernel.h"
#include "core/logging/logger.h"
#include "core/project/lcnc_project_manager.h"
#include "core/settings/app_settings.h"
#include "view/gui_application.h"
#include "modules/cad/cad_module.h"
#include "modules/cam/cam_module.h"
#include "modules/process/process_module.h"

namespace {

QSurfaceFormat makeOccSurfaceFormat()
{
    QSurfaceFormat format;
    format.setRenderableType(QSurfaceFormat::OpenGL);
    format.setProfile(QSurfaceFormat::CoreProfile);
    format.setVersion(3, 3);
    format.setDepthBufferSize(24);
    format.setStencilBufferSize(8);
    format.setSamples(4);
    format.setSwapBehavior(QSurfaceFormat::DoubleBuffer);
    format.setSwapInterval(1);
    return format;
}

} // namespace

int main(int argc, char* argv[])
{
    // High DPI / fractional scaling support
    QApplication::setHighDpiScaleFactorRoundingPolicy(
        Qt::HighDpiScaleFactorRoundingPolicy::PassThrough);

    // Qt/OpenGL defaults must be fixed before QApplication creates any native window.
    QSurfaceFormat::setDefaultFormat(makeOccSurfaceFormat());

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

    const QSurfaceFormat glFormat = QSurfaceFormat::defaultFormat();
    LCNC_INFO(lcnc::LogCode::Generic,
              "Qt OpenGL default format: version={}.{} profile={} samples={} depth={} stencil={} swapInterval={}",
              glFormat.majorVersion(),
              glFormat.minorVersion(),
              static_cast<int>(glFormat.profile()),
              glFormat.samples(),
              glFormat.depthBufferSize(),
              glFormat.stencilBufferSize(),
              glFormat.swapInterval());

    // ── Kernel：注册核心服务 + 加载业务模块 ────────────────────────────
    //    Kernel 直接持有 ProjectManager / TaskManager / AppSettings 实例，
    //    GuiApplication 由 main 创建并注入，不再有 XxxClass::instance() 懒加载单例。
    //    模块依赖：cad ← cam ← process（Process 使用 CAM 的轴定义）。Kernel 内部用 Kahn 拓扑排序，
    //    保证依赖在前。
    //    [modules].disabled = [...] 可在 mainwindow.toml 中关闭某些模块；
    //    被关闭的模块连同依赖它的下游模块都不会加入 Kernel。
    lcnc::Kernel kernel;
    kernel.registerCoreServices();
    kernel.appSettings()->loadDefault();   // mainwindow.toml
    if (auto* project = kernel.projectManager())
        project->setDocumentOpenMode(kernel.appSettings()->documentOpenMode);

    // GuiApplication 不在 core/Kernel 内创建（避免 core 反向依赖 view），
    // 改在此处由 main 拥有并注入 Kernel。须在模块 init 之前完成，
    // 因为 CAM 在 init() 里会访问 guiApp() 获取 workspace 视图。
    auto guiAppOwner = std::make_unique<GuiApplication>();
    kernel.setGuiApp(guiAppOwner.get());

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

    // 作用域销毁顺序：mainWin → guiAppOwner → kernel（声明顺序的反序），
    // 满足 "UI → GuiApplication → ProjectManager/TaskManager" 的依赖
    // 反向释放，无需在此处手动 reset。

    LCNC_INFO(lcnc::LogCode::Generic, "Shutdown rc={}", rc);
    lcnc::Logger::shutdown();
    return rc;
}
