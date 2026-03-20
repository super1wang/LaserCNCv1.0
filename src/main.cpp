#include <QApplication>
#include <QSurfaceFormat>

#include "app/mainwindow.h"
#include "base/lcnc_application.h"

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

    // Initialize LCNC application core
    LcncApplication::instance();

    MainWindow mainWin;
    mainWin.show();

    return app.exec();
}
