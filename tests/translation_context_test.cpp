#include <QCoreApplication>
#include <QTextStream>
#include <QTranslator>

namespace {

int fail(const QString& message)
{
    QTextStream(stderr) << message << '\n';
    return 1;
}

struct TranslationExpectation {
    const char* context;
    const char* source;
    const char* expected;
};

} // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);
    QTranslator translator;
    if (!translator.load(QStringLiteral(":/i18n/lasercnc_zh_CN.qm")))
        return fail(QStringLiteral("Unable to load the embedded Chinese translation resource"));
    app.installTranslator(&translator);

    // These strings cover the visible workflows that previously regressed when
    // lupdate replaced namespace-qualified Q_OBJECT contexts with bare names.
    const TranslationExpectation expectations[] = {
        {"lcnc::cad::ui::WidgetCadTaskPanel", "Basic modeling", "基础建模"},
        {"lcnc::cad::ui::WidgetCadTaskPanel", "Create a new sketch", "新建草图"},
        {"lcnc::app::StartGuideWidget", "start", "开始"},
        {"lcnc::app::StartGuideWidget", "Recently opened projects and STEP files", "最近打开的工程和 STEP 文件"},
        {"lcnc::process::CmdNewProcess", "Create new process", "新建流程"},
        {"lcnc::process::CmdLoadProcess", "Loading process", "加载流程"},
        {"lcnc::process::CmdSaveProcess", "Save process", "保存流程"},
        {"lcnc::process::CmdRunStart", "run", "运行"},
        {"lcnc::process::NormalCuttingManager", "The motion controller is not connected during processing", "加工过程中运动控制器未连接"},
        {"lcnc::process::NormalCuttingManager", "Machining cannot start: %1", "无法开始加工：%1"},
        {"WidgetLaserControl", "Laser", "激光"},
        {"WidgetLaserControl", "Blow", "吹气"},
        {"lcnc::DialogOptions", "Application Options", "应用程序选项"},
        {"lcnc::DialogOptions", "Cutter head parameters", "切割头参数"},
        {"lcnc::DialogOptions", "Cutter head appearance", "切割头外观"},
        {"lcnc::DialogOptions", "Linear X", "X 直线轴"},
        {"lcnc::DialogOptions", "Table tilt", "转台倾斜轴"},
        {"lcnc::DialogOptions", "Primary head tilt", "第一摆头轴"},
        {"lcnc::cam::ui::DialogAxisCalibrationWizard", "Machine coordinate system calibration wizard", "机台坐标系标定向导"},
    };

    for (const TranslationExpectation& expectation : expectations) {
        const QString actual = QCoreApplication::translate(expectation.context, expectation.source);
        const QString expected = QString::fromUtf8(expectation.expected);
        if (actual != expected) {
            return fail(QStringLiteral("Translation mismatch [%1] %2: expected '%3', got '%4'")
                            .arg(QString::fromLatin1(expectation.context),
                                 QString::fromLatin1(expectation.source), expected, actual));
        }
    }

    return 0;
}
