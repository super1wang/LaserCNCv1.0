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
        {"QObject", "Allow configuration-derived RTCP machining", "允许使用构型派生参数进行 RTCP 加工"},
        {"lcnc::cam::ui::DialogConfigurationDerivedRtcp", "Generate, activate, and enable RTCP machining", "生成、激活并启用 RTCP 加工"},
        {"lcnc::cam::ui::DialogConfigurationDerivedRtcp", "I confirmed the rotary centers and TCP in MCS. I understand that machining uses configured speeds and may enable the laser and gas according to the tool settings.", "我已确认旋转中心和 TCP 的 MCS 坐标，并理解加工使用配置速度，且可能按刀具设置开启激光和气体。"},
        {"MainWindow", "Configuration-derived calibration %1 is active. GTN Group and RTCP machining are enabled. Reconnect the controller before running. Configured tool and axis limits apply; laser and gas follow the normal machining sequence.", "构型派生标定 %1 已激活。已启用 GTN Group 和 RTCP 加工。运行前请重新连接控制器。使用已配置的刀具和轴限制，激光及气体按正常加工时序控制。"},
        {"lcnc::cad::ui::WidgetCadTaskPanel", "Basic modeling", "基础建模"},
        {"lcnc::cad::ui::WidgetCadTaskPanel", "Create a new sketch", "新建草图"},
        {"lcnc::app::StartGuideWidget", "start", "开始"},
        {"lcnc::app::StartGuideWidget", "Recently opened projects and STEP files", "最近打开的工程和 STEP 文件"},
        {"lcnc::process::CmdNewProcess", "Create new process", "新建流程"},
        {"lcnc::process::CmdLoadProcess", "Loading process", "加载流程"},
        {"lcnc::process::CmdSaveProcess", "Save process", "保存流程"},
        {"lcnc::process::CmdRunStart", "run", "运行"},
        {"GTNMotionControl", "GTN call failed: operation %1, API %2, axis %3, result %4", "GTN 调用失败：操作 %1，API %2，轴 %3，返回值 %4"},
        {"QObject", "Safe stop could not be confirmed; the controller connection is preserved", "无法确认安全停止；保留控制器连接。"},
        {"QObject", "Cannot safely disconnect the previous controller or create the selected controller; connection canceled", "无法安全断开旧控制器或创建所选控制器；已取消连接。"},
        {"GTNMotionControl", "GTN axis %1 (physical %2) feedback validation failed: profile %3, feedback %4, deviation %5 %6, tolerance %7 %6. Further machining is blocked; Stop/Reset preserves fault evidence. Check direction and feedback scaling.", "GTN %1 轴（物理轴 %2）位置反馈校验失败：规划 %3，反馈 %4，偏差 %5 %6，容差 %7 %6。已阻止继续加工；停止/复位保留故障证据，请检查方向和反馈比例。"},
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
