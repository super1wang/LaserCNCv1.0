#include <QApplication>
#include <QCoreApplication>
#include <QDir>
#include <QSurfaceFormat>
#include <QTranslator>

#include "app/main_window.h"
#include "core/kernel/kernel.h"
#include "core/logging/logger.h"
#include "core/project/lcnc_project_manager.h"
#include "core/settings/app_settings.h"
#include "view/gui_application.h"
#include "modules/cad/cad_module.h"
#include "modules/cam/cam_module.h"
#include "modules/process/process_module.h"
#include "modules/process/System/MessageModule.h"

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

QString industrialStyleSheet()
{
    // Keep the CAD viewport unstyled: OCC owns its rendering surface.  The
    // surrounding chrome is deliberately dark, low-glare and high-contrast so
    // it remains readable beside a bright laser/toolpath preview.
    return QStringLiteral(R"QSS(
        QWidget { font-family: "Segoe UI", "Microsoft YaHei UI"; font-size: 12px; color: #D8E1E8; }
        QMainWindow, QDialog { background: #1B232C; }
        QMenuBar { background: #202A34; border-bottom: 1px solid #364654; padding: 2px 8px; }
        QMenuBar::item { padding: 6px 12px; background: transparent; }
        QMenuBar::item:selected { background: #2D4254; color: #F4FAFF; }
        QMenu { background: #24303B; border: 1px solid #405260; padding: 4px; }
        QMenu::item { padding: 6px 26px 6px 22px; border-radius: 3px; }
        QMenu::item:selected { background: #176B86; }
        SARibbonBar { background: #18232D; border-bottom: 1px solid #405260; }
        SARibbonTabBar { background: #1B2731; border: 0; }
        SARibbonTabBar::tab { color: #AFC1CC; background: transparent; border: 0; padding: 3px 15px 8px; margin: 0 2px; }
        SARibbonTabBar::tab:hover { background: #2A3D4B; color: #E8F6FB; }
        SARibbonTabBar::tab:selected { background: #176B86; color: #FFFFFF; border-bottom: 2px solid #64D8F2; }
        SARibbonStackedWidget, SARibbonCategory { background: #22303A; border-top: 1px solid #405260; }
        SARibbonPanel { background: transparent; border-right: 1px solid #41535F; }
        SARibbonPanelLabel { color: #7ED7EB; font-weight: 600; }
        SARibbonToolButton, QToolButton { color: #D7E4EA; background: transparent; border: 1px solid transparent; border-radius: 3px; padding: 3px; }
        SARibbonToolButton:hover, QToolButton:hover { background: #324956; border-color: #5E8594; }
        SARibbonToolButton:pressed, QToolButton:pressed { background: #176B86; }
        SARibbonSeparatorWidget { background: #41535F; }
        QTabWidget::pane { border: 1px solid #405260; background: #202B35; }
        QTabBar::tab { background: #273440; border: 1px solid #405260; border-bottom: 0; color: #9EAFBC; padding: 6px 12px; margin-right: 2px; }
        QTabBar::tab:selected { background: #202B35; color: #5DD6F5; border-top: 2px solid #20B8D7; }
        QGroupBox { border: 1px solid #405260; border-radius: 4px; margin-top: 10px; padding: 8px 6px 6px 6px; font-weight: 600; color: #B9D8E4; background: #202B35; }
        QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; left: 9px; padding: 0 5px; color: #6BD4EF; }
        QPushButton { background: #2A3945; border: 1px solid #506571; border-radius: 3px; min-height: 25px; padding: 3px 8px; color: #E6EEF2; }
        QPushButton:hover { background: #354B5A; border-color: #70CAE1; }
        QPushButton:pressed { background: #1D6A82; }
        QPushButton:checked { background: #176B86; border-color: #63D5F3; color: #FFFFFF; }
        QPushButton[role="run"] { background: #087A5B; border-color: #22B98A; font-weight: 700; }
        QPushButton[role="pause"] { background: #886B32; border-color: #D7AE54; font-weight: 700; }
        QPushButton[role="resume"] { background: #166C99; border-color: #4ABCE7; font-weight: 700; }
        QPushButton[role="stop"] { background: #A52B2B; border-color: #F05B5B; font-weight: 700; }
        QPushButton[jogDirection="negative"], QPushButton[jogDirection="positive"] { min-height: 30px; min-width: 48px; padding: 2px; font-size: 14px; font-weight: 700; }
        QPushButton[jogDirection="negative"] { background: #263A48; border-color: #557788; }
        QPushButton[jogDirection="positive"] { background: #1E5260; border-color: #4DBFCB; }
        QLineEdit, QDoubleSpinBox, QComboBox, QTextEdit { background: #16212A; border: 1px solid #4A606D; border-radius: 3px; padding: 3px 6px; selection-background-color: #197A98; }
        QDoubleSpinBox:focus, QComboBox:focus { border-color: #55C9E5; }
        QProgressBar { border: 1px solid #4A606D; border-radius: 3px; text-align: center; color: #E7F3F7; background: #142029; min-height: 12px; }
        QProgressBar::chunk { background: #1397A8; border-radius: 2px; }
        QTreeWidget, QTreeView, QTableView { background: #202B35; alternate-background-color: #25333F; border: 1px solid #405260; }
        QTreeView::item { color: #D5E0E7; background: transparent; padding: 2px 3px; }
        QTreeView::item:selected { color: #FFFFFF; background: #176B86; }
        QHeaderView::section { background: #2A3945; border: 0; border-right: 1px solid #405260; border-bottom: 1px solid #405260; padding: 5px; color: #B9D8E4; }
        QScrollBar:vertical { background: #18222B; width: 10px; margin: 0; }
        QScrollBar::handle:vertical { background: #4B6471; min-height: 26px; border-radius: 4px; }
        QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
        QScrollArea, QScrollArea > QWidget > QWidget { background: #202B35; }
    )QSS");
}

bool installApplicationTranslator(QApplication& app, const QString& language)
{
    // 翻译资源内置于可执行文件，避免运行目录缺失 qm 文件导致语言设置失效。
    if (!language.startsWith(QStringLiteral("zh"), Qt::CaseInsensitive))
        return true;

    auto* translator = new QTranslator(&app);
    if (!translator->load(QStringLiteral(":/i18n/lasercnc_zh_CN.qm"))) {
        LCNC_ERR(lcnc::LogCode::SettingsLoaded,
                 "Unable to load application translation for language '{}'",
                 language.toStdString());
        delete translator;
        return false;
    }
    app.installTranslator(translator);
    return true;
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
    app.setStyleSheet(industrialStyleSheet());
    app.setApplicationName(QStringLiteral("LaserCNC"));
    app.setApplicationVersion(QStringLiteral("1.0.0"));
    app.setOrganizationName(QStringLiteral("LaserCNC"));

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
    int rc = -1;
    try {
    {
    lcnc::Kernel kernel;
    kernel.registerCoreServices();
    kernel.appSettings()->loadDefault();   // mainwindow.toml
    installApplicationTranslator(app, kernel.appSettings()->language);
    // 中文翻译：五轴激光加工 CAM
    app.setApplicationDisplayName(
        QCoreApplication::translate("Application", "Five-Axis Laser Machining CAM"));
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
        LCNC_ERR(lcnc::LogCode::InternalUnexpectedState,
                 "Kernel bootstrap failed; aborting startup");
        rc = 1;
    } else {
        LCNC_INFO(lcnc::LogCode::Generic,
                  "Kernel ready: services={}",
                  kernel.services().size());

        MainWindow mainWin;
        mainWin.show();

        rc = app.exec();

        // 注意：Kernel 在 MainWindow 之后销毁，但模块在销毁前先 stop。
        kernel.appSettings()->saveDefault();
        kernel.shutdown();
    }

    // 作用域销毁顺序：mainWin → guiAppOwner → kernel（声明顺序的反序），
    // 满足 "UI → GuiApplication → ProjectManager/TaskManager" 的依赖
    // 反向释放，无需在此处手动 reset。
    }
    } catch (const std::exception& e) {
        rc = 1;
        LCNC_ERR(lcnc::LogCode::InternalUnexpectedState,
                 "Unhandled std::exception in main: {}", e.what());
    } catch (...) {
        rc = 1;
        LCNC_ERR(lcnc::LogCode::InternalUnexpectedState,
                 "Unhandled unknown exception in main");
    }

    // 旧消息桥接线程可能由任一 Process 路径惰性创建；必须在 QApplication
    // 和日志系统仍存活时显式停止，不能依赖进程退出阶段的静态析构顺序。
    MessageModule::shutdown();

    // Logger 必须晚于所有可能写日志的模块、任务和 GUI 对象关闭。
    LCNC_INFO(lcnc::LogCode::Generic, "Shutdown complete rc={}", rc);
    lcnc::Logger::shutdown();
    return rc;
}
