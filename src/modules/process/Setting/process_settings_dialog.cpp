#include "modules/process/Setting/process_settings_dialog.h"

#include "core/logging/logger.h"
#include "modules/process/communication/ui/communication_settings_page.h"
#include "modules/process/settings/process_settings.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFormLayout>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSet>
#include <QSizePolicy>
#include <QSpinBox>
#include <QStackedWidget>
#include <QTextEdit>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QUiLoader>
#include <QXmlStreamReader>
#include <QHBoxLayout>
#include <QVBoxLayout>

namespace lcnc::process {

namespace {

struct LegacyUiPage {
    const char* pageId;
    const char* title;
    const char* resourcePath;
};

struct LegacyUiField {
    QString objectName;
    QString className;
    QString label;
    QStringList options;
};

class LegacyUiLoader : public QUiLoader
{
public:
    using QUiLoader::QUiLoader;

protected:
    QWidget* createWidget(const QString& className, QWidget* parent, const QString& name) override
    {
        if (className == QStringLiteral("CSwitchWidget")) {
            auto* widget = new QCheckBox(parent);
            widget->setObjectName(name);
            return widget;
        }
        return QUiLoader::createWidget(className, parent, name);
    }
};

const LegacyUiPage* legacyUiPages(int* count)
{
    static const LegacyUiPage pages[] = {
        {"Setting", QT_TR_NOOP("Setting 全量"), ":/process/setting/Setting.ui"},
        {"Setting_Tool", QT_TR_NOOP("Tool"), ":/process/setting/Setting_Tool.ui"},
        {"Setting_Laser", QT_TR_NOOP("Laser"), ":/process/setting/Setting_Laser.ui"},
        {"Setting_MotionControl", QT_TR_NOOP("Motion Control"), ":/process/setting/Setting_MotionControl.ui"},
        {"Setting_Axis", QT_TR_NOOP("Axis"), ":/process/setting/Setting_Axis.ui"},
        {"Setting_Analog", QT_TR_NOOP("Analog"), ":/process/setting/Setting_Analog.ui"},
        {"Setting_Digital", QT_TR_NOOP("Digital"), ":/process/setting/Setting_Digital.ui"},
        {"Setting_IOIndex", QT_TR_NOOP("IO Index"), ":/process/setting/Setting_IOIndex.ui"},
        {"Setting_Gas", QT_TR_NOOP("Gas"), ":/process/setting/Setting_Gas.ui"},
        {"Setting_Water", QT_TR_NOOP("Water"), ":/process/setting/Setting_Water.ui"},
        {"Setting_Monitor", QT_TR_NOOP("Monitor"), ":/process/setting/Setting_Monitor.ui"},
        {"Setting_LoadingPos", QT_TR_NOOP("Loading Pos"), ":/process/setting/Setting_LoadingPos.ui"},
        {"Setting_Camera", QT_TR_NOOP("Camera"), ":/process/setting/Setting_Camera.ui"},
        {"Setting_Internet", QT_TR_NOOP("Internet"), ":/process/setting/Setting_Internet.ui"},
        {"qg_dlgsetting", QT_TR_NOOP("Process Setting Shell"), ":/process/setting/qg_dlgsetting.ui"},
        {"qg_dlgpbasicsetting", QT_TR_NOOP("Process Basic"), ":/process/setting/qg_dlgpbasicsetting.ui"},
        {"qg_dlgtbasicsetting", QT_TR_NOOP("Technology Basic"), ":/process/setting/qg_dlgtbasicsetting.ui"},
        {"qg_dlgmotionsetting", QT_TR_NOOP("Motion Process"), ":/process/setting/qg_dlgmotionsetting.ui"},
        {"qg_dlgcuttingprocesssetting", QT_TR_NOOP("Cutting Process"), ":/process/setting/qg_dlgcuttingprocesssetting.ui"},
        {"qg_dlgautomationsetting", QT_TR_NOOP("Automation"), ":/process/setting/qg_dlgautomationsetting.ui"},
        {"qg_dlgsensorsetting", QT_TR_NOOP("Sensor"), ":/process/setting/qg_dlgsensorsetting.ui"},
        {"qg_dlgsignalsourcesetting", QT_TR_NOOP("Signal Source"), ":/process/setting/qg_dlgsignalsourcesetting.ui"},
        {"qg_dlgjsonsetting", QT_TR_NOOP("JSON Signal"), ":/process/setting/qg_dlgjsonsetting.ui"},
        {"qg_dlgsmcsetting", QT_TR_NOOP("SMC"), ":/process/setting/qg_dlgsmcsetting.ui"},
        {"qg_dlgtcpsetting", QT_TR_NOOP("TCP"), ":/process/setting/qg_dlgtcpsetting.ui"},
    };
    if (count)
        *count = static_cast<int>(sizeof(pages) / sizeof(pages[0]));
    return pages;
}

bool isLegacyEditorClass(const QString& className)
{
    return className == QStringLiteral("QLineEdit")
        || className == QStringLiteral("QComboBox")
        || className == QStringLiteral("QCheckBox")
        || className == QStringLiteral("QSpinBox")
        || className == QStringLiteral("QDoubleSpinBox")
        || className == QStringLiteral("QTextEdit")
        || className == QStringLiteral("QPlainTextEdit");
}

QString visibleNodePathForPage(const QString& pageId)
{
    if (pageId == QStringLiteral("Setting_MotionControl"))
        return QStringLiteral("Settings/外设/运动控制器");
    if (pageId == QStringLiteral("Setting_Axis"))
        return QStringLiteral("Settings/外设/运动轴");
    if (pageId == QStringLiteral("Setting_IOIndex")
        || pageId == QStringLiteral("Setting_Digital")
        || pageId == QStringLiteral("Setting_Analog"))
        return QStringLiteral("Settings/外设/I/O索引");
    if (pageId == QStringLiteral("Setting_Laser"))
        return QStringLiteral("Settings/外设/激光器");
    if (pageId == QStringLiteral("Setting_Tool"))
        return QStringLiteral("Settings/加工设置/工具");
    if (pageId == QStringLiteral("Setting_Gas"))
        return QStringLiteral("Settings/加工设置/吹气");
    if (pageId == QStringLiteral("Setting_Water"))
        return QStringLiteral("Settings/加工设置/湿切");
    if (pageId == QStringLiteral("Setting_Monitor"))
        return QStringLiteral("Settings/加工设置/监控");
    if (pageId == QStringLiteral("Setting_LoadingPos"))
        return QStringLiteral("Settings/加工设置/上料位");
    return QString();
}

bool isVisibleSettingsPage(const QString& pageId)
{
    return !visibleNodePathForPage(pageId).isEmpty();
}

QString legacyEditorClassName(QWidget* editor)
{
    if (qobject_cast<QLineEdit*>(editor))
        return QStringLiteral("QLineEdit");
    if (qobject_cast<QComboBox*>(editor))
        return QStringLiteral("QComboBox");
    if (qobject_cast<QCheckBox*>(editor))
        return QStringLiteral("QCheckBox");
    if (qobject_cast<QSpinBox*>(editor))
        return QStringLiteral("QSpinBox");
    if (qobject_cast<QDoubleSpinBox*>(editor))
        return QStringLiteral("QDoubleSpinBox");
    if (qobject_cast<QTextEdit*>(editor))
        return QStringLiteral("QTextEdit");
    if (qobject_cast<QPlainTextEdit*>(editor))
        return QStringLiteral("QPlainTextEdit");
    return QString();
}

bool isLegacyEditorWidget(QWidget* editor)
{
    return !legacyEditorClassName(editor).isEmpty() && !editor->objectName().trimmed().isEmpty();
}

void registerLegacyEditor(QMap<QString, QWidget*>& editors, const QString& pageId, QWidget* editor)
{
    if (!isLegacyEditorWidget(editor))
        return;
    const QString key = QStringLiteral("%1.%2").arg(pageId, editor->objectName().trimmed());
    editor->setProperty("uiSettingKey", key);
    editor->setProperty("uiSettingPage", pageId);
    editor->setProperty("uiSettingField", editor->objectName().trimmed());
    editor->setProperty("uiSettingClass", legacyEditorClassName(editor));
    editor->setProperty("legacySettingKey", key);
    editor->setProperty("legacySettingClass", legacyEditorClassName(editor));
    editors.insert(key, editor);
}

void registerLegacyEditors(QMap<QString, QWidget*>& editors, const QString& pageId, QWidget* root)
{
    if (!root)
        return;
    registerLegacyEditor(editors, pageId, root);
    const auto children = root->findChildren<QWidget*>();
    for (QWidget* child : children)
        registerLegacyEditor(editors, pageId, child);
}

void hideEmbeddedSettingDialogButtons(QWidget* root)
{
    if (!root)
        return;
    const QStringList buttonNames = {
        QStringLiteral("pushButton_Setting_Apply"),
        QStringLiteral("pushButton_Setting_OK"),
        QStringLiteral("pushButton_Setting_Cancel"),
        QStringLiteral("pushButton_Setting_ExportConfig"),
        QStringLiteral("pushButton_Setting_ImportConfig"),
    };
    for (const QString& name : buttonNames) {
        if (QWidget* button = root->findChild<QWidget*>(name))
            button->hide();
    }
    for (QDialogButtonBox* buttons : root->findChildren<QDialogButtonBox*>())
        buttons->hide();
}

QWidget* loadLegacyUiWidget(const QString& resourcePath, QWidget* parent)
{
    QFile file(resourcePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        LCNC_WARN(lcnc::LogCode::Generic,
                  "process.settings.ui: open failed resource='{}'",
                  resourcePath.toStdString());
        return nullptr;
    }

    LegacyUiLoader loader;
    QWidget* widget = loader.load(&file, parent);
    if (!widget) {
        LCNC_WARN(lcnc::LogCode::Generic,
                  "process.settings.ui: load failed resource='{}' error='{}'",
                  resourcePath.toStdString(),
                  loader.errorString().toStdString());
        return nullptr;
    }
    widget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    return widget;
}

QString legacyFieldLabel(QString objectName)
{
    static const QStringList prefixes = {
        QStringLiteral("lineEdit_"),
        QStringLiteral("comboBox_"),
        QStringLiteral("checkBox_"),
        QStringLiteral("spinBox_"),
        QStringLiteral("doubleSpinBox_"),
        QStringLiteral("textEdit_"),
        QStringLiteral("plainTextEdit_"),
    };
    for (const QString& prefix : prefixes) {
        if (objectName.startsWith(prefix)) {
            objectName = objectName.mid(prefix.size());
            break;
        }
    }
    return objectName.replace(QLatin1Char('_'), QStringLiteral(" / "));
}

QList<LegacyUiField> parseLegacyUiFields(const QString& resourcePath)
{
    QFile file(resourcePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        return {};

    QList<LegacyUiField> fields;
    QSet<QString> seen;
    QXmlStreamReader xml(&file);
    while (!xml.atEnd()) {
        xml.readNext();
        if (!xml.isStartElement() || xml.name() != QStringLiteral("widget"))
            continue;

        const QString className = xml.attributes().value(QStringLiteral("class")).toString();
        const QString objectName = xml.attributes().value(QStringLiteral("name")).toString();
        if (!isLegacyEditorClass(className) || objectName.isEmpty() || seen.contains(objectName))
            continue;

        LegacyUiField field;
        field.className = className;
        field.objectName = objectName;
        field.label = legacyFieldLabel(objectName);
        seen.insert(objectName);

        int depth = 1;
        while (!xml.atEnd() && depth > 0) {
            xml.readNext();
            if (xml.isStartElement()) {
                if (xml.name() == QStringLiteral("widget")) {
                    ++depth;
                } else if (className == QStringLiteral("QComboBox")
                           && xml.name() == QStringLiteral("string")) {
                    const QString option = xml.readElementText().trimmed();
                    if (!option.isEmpty() && !field.options.contains(option))
                        field.options.append(option);
                }
            } else if (xml.isEndElement() && xml.name() == QStringLiteral("widget")) {
                --depth;
            }
        }

        fields.append(field);
    }
    return fields;
}

QDoubleSpinBox* makeDoubleSpin(QWidget* parent, double maxValue, const QString& suffix)
{
    auto* spin = new QDoubleSpinBox(parent);
    spin->setRange(0.0, maxValue);
    spin->setDecimals(3);
    spin->setSuffix(suffix);
    spin->setKeyboardTracking(false);
    return spin;
}

QSpinBox* makeIntSpin(QWidget* parent, int maxValue, const QString& suffix)
{
    auto* spin = new QSpinBox(parent);
    spin->setRange(0, maxValue);
    spin->setSuffix(suffix);
    spin->setKeyboardTracking(false);
    return spin;
}

} // namespace

ProcessSettingsDialog::ProcessSettingsDialog(lcnc::ProcessSettings& settings,
                                             const QStringList& motionControllers,
                                             const QStringList& laserDevices,
                                             InitialPage initialPage,
                                             QWidget* parent)
    : QDialog(parent)
    , m_settings(settings)
    , m_motionControllers(motionControllers)
    , m_laserDevices(laserDevices)
{
    setWindowTitle(tr("加工参数"));
    resize(720, 420);

    auto* layout = new QVBoxLayout(this);
    auto* body = new QHBoxLayout();

    m_pageTree = new QTreeWidget(this);
    m_pageTree->setHeaderHidden(true);
    m_pageTree->setMinimumWidth(190);
    m_pageTree->setMaximumWidth(240);

    m_pages = new QStackedWidget(this);
    const int processPage = m_pages->addWidget(buildProcessPage());
    const int motionPage = m_pages->addWidget(buildMotionPage());
    const int laserPage = m_pages->addWidget(buildLaserPage());
    const int axisPage = m_pages->addWidget(buildAxisPage());
    const int toolPage = m_pages->addWidget(buildToolPage());
    const int ioPage = m_pages->addWidget(buildIoPage());
    const int gasPage = m_pages->addWidget(buildGasPage());
    const int waterPage = m_pages->addWidget(buildWaterPage());
    const int monitorPage = m_pages->addWidget(buildMonitorPage());
    const int loadingPage = m_pages->addWidget(buildLoadingPage());
    const int cameraPage = m_pages->addWidget(buildCameraPage());
    const int internetPage = m_pages->addWidget(buildInternetPage());
    const int communicationPage = m_pages->addWidget(buildCommunicationPage());

    m_legacyPageIndexes.insert(QStringLiteral("_process"), processPage);
    m_legacyPageIndexes.insert(QStringLiteral("_motion"), motionPage);
    m_legacyPageIndexes.insert(QStringLiteral("_laser"), laserPage);
    m_legacyPageIndexes.insert(QStringLiteral("_axis"), axisPage);
    m_legacyPageIndexes.insert(QStringLiteral("_tool"), toolPage);
    m_legacyPageIndexes.insert(QStringLiteral("_io"), ioPage);
    m_legacyPageIndexes.insert(QStringLiteral("_gas"), gasPage);
    m_legacyPageIndexes.insert(QStringLiteral("_water"), waterPage);
    m_legacyPageIndexes.insert(QStringLiteral("_monitor"), monitorPage);
    m_legacyPageIndexes.insert(QStringLiteral("_loading"), loadingPage);
    m_legacyPageIndexes.insert(QStringLiteral("_camera"), cameraPage);
    m_legacyPageIndexes.insert(QStringLiteral("_internet"), internetPage);
    m_legacyPageIndexes.insert(QStringLiteral("_communication"), communicationPage);

    loadLegacySettingsPages();
    buildSettingsTree();
    m_pageTree->expandAll();

    connect(m_pageTree, &QTreeWidget::itemClicked,
            this, &ProcessSettingsDialog::switchPage);

    body->addWidget(m_pageTree);
    body->addWidget(m_pages, 1);
    layout->addLayout(body, 1);

    auto* buttons = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Apply | QDialogButtonBox::Cancel,
        this);
    connect(buttons, &QDialogButtonBox::accepted, this, [this] {
        applyToSettings();
        accept();
    });
    connect(buttons->button(QDialogButtonBox::Apply), &QPushButton::clicked,
            this, [this] { applyToSettings(); });
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    layout->addWidget(buttons);

    loadFromSettings();

    switch (initialPage) {
    case InitialPage::Motion:
        m_pages->setCurrentIndex(pageIndexFor(QStringLiteral("Setting_MotionControl"), motionPage));
        break;
    case InitialPage::Laser:
        m_pages->setCurrentIndex(pageIndexFor(QStringLiteral("Setting_Laser"), laserPage));
        break;
    case InitialPage::Process:
    default:
        m_pages->setCurrentIndex(pageIndexFor(QStringLiteral("Setting_Tool"), toolPage));
        break;
    }
}

QTreeWidgetItem* ProcessSettingsDialog::addGroupNode(QTreeWidgetItem* parent, const QString& text)
{
    auto* item = parent
        ? new QTreeWidgetItem(parent)
        : new QTreeWidgetItem(m_pageTree);
    item->setText(0, text);
    item->setData(0, Qt::UserRole, -1);
    return item;
}

QTreeWidgetItem* ProcessSettingsDialog::addPageNode(QTreeWidgetItem* parent,
                                                    const QString& text,
                                                    int pageIndex,
                                                    const QString& pageId)
{
    auto* item = parent
        ? new QTreeWidgetItem(parent)
        : new QTreeWidgetItem(m_pageTree);
    item->setText(0, text);
    item->setData(0, Qt::UserRole, pageIndex);
    item->setData(0, Qt::UserRole + 1, pageId);
    return item;
}

int ProcessSettingsDialog::pageIndexFor(const QString& pageId, int fallbackPageIndex) const
{
    return m_legacyPageIndexes.value(pageId, fallbackPageIndex);
}

void ProcessSettingsDialog::switchPage(QTreeWidgetItem* item, int column)
{
    if (!item || column != 0)
        return;
    const int pageIndex = item->data(0, Qt::UserRole).toInt();
    if (pageIndex >= 0 && pageIndex < m_pages->count())
        m_pages->setCurrentIndex(pageIndex);
}

QWidget* ProcessSettingsDialog::buildProcessPage()
{
    auto* page = new QWidget(this);
    auto* form = new QFormLayout(page);

    m_simulationModeCheck = new QCheckBox(tr("仿真模式"), page);
    form->addRow(tr("运行模式"), m_simulationModeCheck);

    m_endpointEdit = new QLineEdit(page);
    m_endpointEdit->setPlaceholderText(QStringLiteral("tcp://127.0.0.1:5000"));
    form->addRow(tr("控制器地址"), m_endpointEdit);

    return page;
}

QWidget* ProcessSettingsDialog::buildMotionPage()
{
    auto* page = new QWidget(this);
    auto* form = new QFormLayout(page);

    m_motionControllerCombo = new QComboBox(page);
    m_motionControllerCombo->addItems(m_motionControllers);
    form->addRow(tr("运动控制器"), m_motionControllerCombo);

    return page;
}

QWidget* ProcessSettingsDialog::buildLaserPage()
{
    auto* page = new QWidget(this);
    auto* form = new QFormLayout(page);

    m_laserDeviceCombo = new QComboBox(page);
    m_laserDeviceCombo->addItems(m_laserDevices);
    form->addRow(tr("激光器"), m_laserDeviceCombo);

    m_laserEnergySpin = makeDoubleSpin(page, 1000000.0, QStringLiteral(" uJ"));
    form->addRow(tr("能量"), m_laserEnergySpin);

    m_laserFrequencySpin = makeDoubleSpin(page, 1000000.0, QStringLiteral(" Hz"));
    form->addRow(tr("频率"), m_laserFrequencySpin);

    m_laserPulseWidthSpin = makeDoubleSpin(page, 1000000.0, QStringLiteral(" ns"));
    form->addRow(tr("脉宽"), m_laserPulseWidthSpin);

    return page;
}

QWidget* ProcessSettingsDialog::buildAxisPage()
{
    auto* page = new QWidget(this);
    auto* form = new QFormLayout(page);

    m_axisTravelXSpin = makeDoubleSpin(page, 1000000.0, QStringLiteral(" mm"));
    form->addRow(tr("X 行程"), m_axisTravelXSpin);

    m_axisTravelYSpin = makeDoubleSpin(page, 1000000.0, QStringLiteral(" mm"));
    form->addRow(tr("Y 行程"), m_axisTravelYSpin);

    m_axisTravelZSpin = makeDoubleSpin(page, 1000000.0, QStringLiteral(" mm"));
    form->addRow(tr("Z 行程"), m_axisTravelZSpin);

    m_axisMaxVelocitySpin = makeDoubleSpin(page, 1000000.0, QStringLiteral(" mm/s"));
    form->addRow(tr("最大速度"), m_axisMaxVelocitySpin);

    m_axisAccelerationSpin = makeDoubleSpin(page, 1000000.0, QStringLiteral(" mm/s2"));
    form->addRow(tr("加速度"), m_axisAccelerationSpin);

    return page;
}

QWidget* ProcessSettingsDialog::buildToolPage()
{
    auto* page = new QWidget(this);
    auto* form = new QFormLayout(page);

    m_toolFeedRateSpin = makeDoubleSpin(page, 1000000.0, QStringLiteral(" mm/s"));
    form->addRow(tr("进给速度"), m_toolFeedRateSpin);

    m_toolKerfWidthSpin = makeDoubleSpin(page, 1000000.0, QStringLiteral(" mm"));
    form->addRow(tr("割缝宽度"), m_toolKerfWidthSpin);

    m_pierceDelaySpin = makeIntSpin(page, 3600000, QStringLiteral(" ms"));
    form->addRow(tr("穿孔延时"), m_pierceDelaySpin);

    return page;
}

QWidget* ProcessSettingsDialog::buildIoPage()
{
    auto* page = new QWidget(this);
    auto* form = new QFormLayout(page);

    m_ioDefaultChannelEdit = new QLineEdit(page);
    form->addRow(tr("默认通道"), m_ioDefaultChannelEdit);

    m_ioDefaultValueCheck = new QCheckBox(tr("输出高电平"), page);
    form->addRow(tr("默认输出"), m_ioDefaultValueCheck);

    return page;
}

QWidget* ProcessSettingsDialog::buildGasPage()
{
    auto* page = new QWidget(this);
    auto* form = new QFormLayout(page);

    m_assistGasEdit = new QLineEdit(page);
    form->addRow(tr("辅助气体"), m_assistGasEdit);

    m_gasPressureSpin = makeDoubleSpin(page, 1000.0, QStringLiteral(" bar"));
    form->addRow(tr("气压"), m_gasPressureSpin);

    return page;
}

QWidget* ProcessSettingsDialog::buildWaterPage()
{
    auto* page = new QWidget(this);
    auto* form = new QFormLayout(page);

    m_waterCoolingCheck = new QCheckBox(tr("启用水冷"), page);
    form->addRow(tr("水冷"), m_waterCoolingCheck);

    m_waterMinFlowSpin = makeDoubleSpin(page, 1000000.0, QStringLiteral(" L/min"));
    form->addRow(tr("最小流量"), m_waterMinFlowSpin);

    return page;
}

QWidget* ProcessSettingsDialog::buildMonitorPage()
{
    auto* page = new QWidget(this);
    auto* form = new QFormLayout(page);

    m_monitorEnabledCheck = new QCheckBox(tr("启用监控"), page);
    form->addRow(tr("监控"), m_monitorEnabledCheck);

    m_monitorIntervalSpin = makeIntSpin(page, 3600000, QStringLiteral(" ms"));
    form->addRow(tr("采样周期"), m_monitorIntervalSpin);

    return page;
}

QWidget* ProcessSettingsDialog::buildLoadingPage()
{
    auto* page = new QWidget(this);
    auto* form = new QFormLayout(page);

    m_loadingXSpin = makeDoubleSpin(page, 1000000.0, QStringLiteral(" mm"));
    form->addRow(tr("X 位置"), m_loadingXSpin);

    m_loadingYSpin = makeDoubleSpin(page, 1000000.0, QStringLiteral(" mm"));
    form->addRow(tr("Y 位置"), m_loadingYSpin);

    m_loadingZSpin = makeDoubleSpin(page, 1000000.0, QStringLiteral(" mm"));
    form->addRow(tr("Z 位置"), m_loadingZSpin);

    return page;
}

QWidget* ProcessSettingsDialog::buildCameraPage()
{
    auto* page = new QWidget(this);
    auto* form = new QFormLayout(page);

    m_cameraNameEdit = new QLineEdit(page);
    form->addRow(tr("相机"), m_cameraNameEdit);

    m_cameraExposureSpin = makeIntSpin(page, 3600000, QStringLiteral(" ms"));
    form->addRow(tr("曝光"), m_cameraExposureSpin);

    return page;
}

QWidget* ProcessSettingsDialog::buildInternetPage()
{
    auto* page = new QWidget(this);
    auto* form = new QFormLayout(page);

    m_internetHostEdit = new QLineEdit(page);
    form->addRow(tr("Host"), m_internetHostEdit);

    m_internetPortSpin = makeIntSpin(page, 65535, QString());
    form->addRow(tr("Port"), m_internetPortSpin);

    return page;
}

QWidget* ProcessSettingsDialog::buildCommunicationPage()
{
    m_communicationPage = new CommunicationSettingsPage(this);
    return m_communicationPage;
}

QWidget* ProcessSettingsDialog::buildLegacySettingsPage(const QString& pageId,
                                                        const QString& title,
                                                        const QString& resourcePath)
{
    auto* scroll = new QScrollArea(this);
    scroll->setWidgetResizable(true);

    if (QWidget* loadedUi = loadLegacyUiWidget(resourcePath, scroll)) {
        const int editorCountBefore = m_legacyEditors.size();
        hideEmbeddedSettingDialogButtons(loadedUi);
        registerLegacyEditors(m_legacyEditors, pageId, loadedUi);
        LCNC_INFO(lcnc::LogCode::Generic,
                  "process.settings.ui: loaded page='{}' editors={}",
                  pageId.toStdString(),
                  m_legacyEditors.size() - editorCountBefore);
        scroll->setWidget(loadedUi);
        return scroll;
    }

    auto* page = new QWidget(scroll);
    auto* form = new QFormLayout(page);
    form->setFieldGrowthPolicy(QFormLayout::ExpandingFieldsGrow);

    const QList<LegacyUiField> fields = parseLegacyUiFields(resourcePath);
    if (fields.isEmpty()) {
        auto* empty = new QLineEdit(page);
        empty->setReadOnly(true);
        empty->setText(tr("未找到可迁移字段：%1").arg(resourcePath));
        form->addRow(title, empty);
    }

    for (const LegacyUiField& field : fields) {
        QWidget* editor = nullptr;
        if (field.className == QStringLiteral("QCheckBox")) {
            auto* check = new QCheckBox(page);
            editor = check;
        } else if (field.className == QStringLiteral("QComboBox")) {
            auto* combo = new QComboBox(page);
            combo->setEditable(true);
            combo->addItems(field.options);
            editor = combo;
        } else if (field.className == QStringLiteral("QSpinBox")) {
            auto* spin = makeIntSpin(page, 100000000, QString());
            editor = spin;
        } else if (field.className == QStringLiteral("QDoubleSpinBox")) {
            auto* spin = makeDoubleSpin(page, 100000000.0, QString());
            spin->setMinimum(-100000000.0);
            editor = spin;
        } else {
            auto* edit = new QLineEdit(page);
            edit->setPlaceholderText(field.objectName);
            editor = edit;
        }

        const QString key = QStringLiteral("%1.%2").arg(pageId, field.objectName);
        editor->setProperty("legacySettingKey", key);
        editor->setProperty("legacySettingClass", field.className);
        m_legacyEditors.insert(key, editor);
        form->addRow(field.label, editor);
    }

    scroll->setWidget(page);
    return scroll;
}

void ProcessSettingsDialog::loadLegacySettingsPages()
{
    m_fieldRegistry.clear();
    int count = 0;
    const LegacyUiPage* pages = legacyUiPages(&count);
    for (int index = 0; index < count; ++index) {
        const QString pageId = QString::fromLatin1(pages[index].pageId);
        const QString title = tr(pages[index].title);
        const QString resourcePath = QString::fromLatin1(pages[index].resourcePath);
        const QString visibleNodePath = visibleNodePathForPage(pageId);
        QString registryError;
        if (!m_fieldRegistry.registerUiFile(pageId,
                                            resourcePath,
                                            visibleNodePath,
                                            isVisibleSettingsPage(pageId),
                                            &registryError)) {
            LCNC_WARN(lcnc::LogCode::Generic,
                      "process.settings.ui: field registry failed page='{}' error='{}'",
                      pageId.toStdString(),
                      registryError.toStdString());
        }
        const int pageIndex = m_pages->addWidget(buildLegacySettingsPage(
            pageId,
            title,
            resourcePath));
        m_legacyPageIndexes.insert(pageId, pageIndex);
    }
    LCNC_INFO(lcnc::LogCode::Generic,
              "process.settings.ui: legacy pages loaded pages={} fields={}",
              count,
              m_fieldRegistry.fieldCount());
}

void ProcessSettingsDialog::buildSettingsTree()
{
    m_pageTree->clear();

    auto* root = addGroupNode(nullptr, tr("Settings"));
    auto* deviceRoot = addGroupNode(root, tr("外设"));
    addPageNode(deviceRoot,
                tr("运动控制器"),
                pageIndexFor(QStringLiteral("Setting_MotionControl"), m_legacyPageIndexes.value(QStringLiteral("_motion"), 0)),
                QStringLiteral("Setting_MotionControl"));
    addPageNode(deviceRoot,
                tr("运动轴"),
                pageIndexFor(QStringLiteral("Setting_Axis"), m_legacyPageIndexes.value(QStringLiteral("_axis"), 0)),
                QStringLiteral("Setting_Axis"));
    addPageNode(deviceRoot,
                tr("I/O索引"),
                pageIndexFor(QStringLiteral("Setting_IOIndex"), m_legacyPageIndexes.value(QStringLiteral("_io"), 0)),
                QStringLiteral("Setting_IOIndex"));
    addPageNode(deviceRoot,
                tr("激光器"),
                pageIndexFor(QStringLiteral("Setting_Laser"), m_legacyPageIndexes.value(QStringLiteral("_laser"), 0)),
                QStringLiteral("Setting_Laser"));

    auto* processRoot = addGroupNode(root, tr("加工设置"));
    auto* toolRoot = addPageNode(processRoot,
                                 tr("工具"),
                                 pageIndexFor(QStringLiteral("Setting_Tool"), m_legacyPageIndexes.value(QStringLiteral("_tool"), 0)),
                                 QStringLiteral("Setting_Tool"));
    addPageNode(toolRoot,
                tr("运动&激光"),
                pageIndexFor(QStringLiteral("Setting_Tool"), m_legacyPageIndexes.value(QStringLiteral("_tool"), 0)),
                QStringLiteral("Setting_Tool"));
    addPageNode(toolRoot,
                tr("基础参数"),
                pageIndexFor(QStringLiteral("Setting_Tool"), m_legacyPageIndexes.value(QStringLiteral("_tool"), 0)),
                QStringLiteral("Setting_Tool"));
    addPageNode(toolRoot,
                tr("随动"),
                pageIndexFor(QStringLiteral("Setting_Tool"), m_legacyPageIndexes.value(QStringLiteral("_tool"), 0)),
                QStringLiteral("Setting_Tool"));
    addPageNode(processRoot,
                tr("吹气"),
                pageIndexFor(QStringLiteral("Setting_Gas"), m_legacyPageIndexes.value(QStringLiteral("_gas"), 0)),
                QStringLiteral("Setting_Gas"));
    addPageNode(processRoot,
                tr("湿切"),
                pageIndexFor(QStringLiteral("Setting_Water"), m_legacyPageIndexes.value(QStringLiteral("_water"), 0)),
                QStringLiteral("Setting_Water"));
    addPageNode(processRoot,
                tr("监控"),
                pageIndexFor(QStringLiteral("Setting_Monitor"), m_legacyPageIndexes.value(QStringLiteral("_monitor"), 0)),
                QStringLiteral("Setting_Monitor"));
    addPageNode(processRoot,
                tr("上料位"),
                pageIndexFor(QStringLiteral("Setting_LoadingPos"), m_legacyPageIndexes.value(QStringLiteral("_loading"), 0)),
                QStringLiteral("Setting_LoadingPos"));
}

void ProcessSettingsDialog::loadLegacySettings()
{
    for (auto it = m_legacyEditors.cbegin(); it != m_legacyEditors.cend(); ++it) {
        QWidget* editor = it.value();
        if (!m_settings.uiSettingValues().contains(it.key()))
            continue;
        const QString className = editor->property("legacySettingClass").toString();
        const QString value = m_settings.uiSettingValue(it.key());
        if (auto* check = qobject_cast<QCheckBox*>(editor)) {
            check->setChecked(value == QStringLiteral("true") || value == QStringLiteral("1"));
        } else if (auto* combo = qobject_cast<QComboBox*>(editor)) {
            if (!value.isEmpty() && combo->findText(value) < 0)
                combo->addItem(value);
            combo->setCurrentText(value);
        } else if (auto* spin = qobject_cast<QSpinBox*>(editor)) {
            spin->setValue(value.toInt());
        } else if (auto* doubleSpin = qobject_cast<QDoubleSpinBox*>(editor)) {
            doubleSpin->setValue(value.toDouble());
        } else if (auto* edit = qobject_cast<QLineEdit*>(editor)) {
            edit->setText(value);
        } else if (auto* textEdit = qobject_cast<QTextEdit*>(editor)) {
            textEdit->setPlainText(value);
        } else if (auto* plainTextEdit = qobject_cast<QPlainTextEdit*>(editor)) {
            plainTextEdit->setPlainText(value);
        }
        Q_UNUSED(className);
    }
}

void ProcessSettingsDialog::applyLegacySettings(QMap<QString, QString>& values) const
{
    for (auto it = m_legacyEditors.cbegin(); it != m_legacyEditors.cend(); ++it) {
        QString value;
        if (auto* check = qobject_cast<QCheckBox*>(it.value()))
            value = check->isChecked() ? QStringLiteral("true") : QStringLiteral("false");
        else if (auto* combo = qobject_cast<QComboBox*>(it.value()))
            value = combo->currentText().trimmed();
        else if (auto* spin = qobject_cast<QSpinBox*>(it.value()))
            value = QString::number(spin->value());
        else if (auto* doubleSpin = qobject_cast<QDoubleSpinBox*>(it.value()))
            value = QString::number(doubleSpin->value(), 'g', 15);
        else if (auto* edit = qobject_cast<QLineEdit*>(it.value()))
            value = edit->text().trimmed();
        else if (auto* textEdit = qobject_cast<QTextEdit*>(it.value()))
            value = textEdit->toPlainText().trimmed();
        else if (auto* plainTextEdit = qobject_cast<QPlainTextEdit*>(it.value()))
            value = plainTextEdit->toPlainText().trimmed();
        values.insert(it.key(), value);
    }
}

void ProcessSettingsDialog::applyKnownLegacyTypedSettings(const QMap<QString, QString>& values)
{
    auto firstValue = [&values](std::initializer_list<const char*> keys) {
        for (const char* key : keys) {
            const QString value = values.value(QString::fromLatin1(key)).trimmed();
            if (!value.isEmpty())
                return value;
        }
        return QString();
    };

    auto applyDouble = [&](std::initializer_list<const char*> keys, auto setter) {
        const QString valueText = firstValue(keys);
        if (valueText.isEmpty())
            return false;
        bool ok = false;
        const double value = valueText.toDouble(&ok);
        if (!ok)
            return false;
        setter(value);
        return true;
    };

    auto applyInt = [&](std::initializer_list<const char*> keys, auto setter) {
        const QString valueText = firstValue(keys);
        if (valueText.isEmpty())
            return false;
        bool ok = false;
        const int value = valueText.toInt(&ok);
        if (!ok)
            return false;
        setter(value);
        return true;
    };

    auto applyString = [&](std::initializer_list<const char*> keys, auto setter) {
        const QString value = firstValue(keys);
        if (value.isEmpty())
            return false;
        setter(value);
        return true;
    };

    int mappedCount = 0;
    mappedCount += applyDouble({"Setting_Tool.lineEdit_Laser_fEnergy", "Setting_Laser.lineEdit_Laser_fEnergy"},
                               [this](double value) { m_settings.setLaserEnergy(value); });
    mappedCount += applyDouble({"Setting_Tool.lineEdit_Laser_fFrequency", "Setting_Laser.lineEdit_Laser_fFrequency"},
                               [this](double value) { m_settings.setLaserFrequency(value); });
    mappedCount += applyDouble({"Setting_Tool.lineEdit_Laser_fPluse", "Setting_Laser.lineEdit_Laser_fPulseWidth"},
                               [this](double value) { m_settings.setLaserPulseWidth(value); });
    mappedCount += applyDouble({"Setting_Tool.lineEdit_Cutting_fLineVel"},
                               [this](double value) { m_settings.setToolFeedRate(value); });
    mappedCount += applyDouble({"Setting_MotionControl.lineEdit_AxisSetting_fVel"},
                               [this](double value) { m_settings.setAxisMaxVelocity(value); });
    mappedCount += applyDouble({"Setting_MotionControl.lineEdit_AxisSetting_fAcc"},
                               [this](double value) { m_settings.setAxisAcceleration(value); });
    mappedCount += applyDouble({"Setting_Gas.lineEdit_Gas_fPressure"},
                               [this](double value) { m_settings.setGasPressure(value); });
    mappedCount += applyDouble({"Setting_LoadingPos.lineEdit_LoadingPos_fLoadingPosX", "qg_dlgpbasicsetting.lineEdit_X_Position"},
                               [this](double value) { m_settings.setLoadingPositionX(value); });
    mappedCount += applyDouble({"Setting_LoadingPos.lineEdit_LoadingPos_fLoadingPosY", "qg_dlgpbasicsetting.lineEdit_Y_Position"},
                               [this](double value) { m_settings.setLoadingPositionY(value); });
    mappedCount += applyDouble({"Setting_LoadingPos.lineEdit_LoadingPos_fLoadingPosZ", "qg_dlgpbasicsetting.lineEdit_Z_Position"},
                               [this](double value) { m_settings.setLoadingPositionZ(value); });
    mappedCount += applyInt({"Setting_Tool.lineEdit_Laser_iDelay"},
                            [this](int value) { m_settings.setPierceDelayMs(value); });
    mappedCount += applyString({"Setting_IOIndex.lineEdit_DigitalOUT_aLaser"},
                               [this](const QString& value) { m_settings.setIoDefaultChannel(value); });

    LCNC_INFO(lcnc::LogCode::Generic,
              "process.settings.ui: mapped legacy fields to typed settings count={}",
              mappedCount);
}

void ProcessSettingsDialog::loadFromSettings()
{
    m_simulationModeCheck->setChecked(m_settings.simulationMode());
    m_endpointEdit->setText(m_settings.controllerEndpoint());
    m_motionControllerCombo->setCurrentText(m_settings.motionControllerName());
    m_laserDeviceCombo->setCurrentText(m_settings.laserDeviceName());
    m_laserEnergySpin->setValue(m_settings.laserEnergy());
    m_laserFrequencySpin->setValue(m_settings.laserFrequency());
    m_laserPulseWidthSpin->setValue(m_settings.laserPulseWidth());
    m_axisTravelXSpin->setValue(m_settings.axisTravelX());
    m_axisTravelYSpin->setValue(m_settings.axisTravelY());
    m_axisTravelZSpin->setValue(m_settings.axisTravelZ());
    m_axisMaxVelocitySpin->setValue(m_settings.axisMaxVelocity());
    m_axisAccelerationSpin->setValue(m_settings.axisAcceleration());
    m_toolFeedRateSpin->setValue(m_settings.toolFeedRate());
    m_toolKerfWidthSpin->setValue(m_settings.toolKerfWidth());
    m_pierceDelaySpin->setValue(m_settings.pierceDelayMs());
    m_ioDefaultChannelEdit->setText(m_settings.ioDefaultChannel());
    m_ioDefaultValueCheck->setChecked(m_settings.ioDefaultValue());
    m_assistGasEdit->setText(m_settings.assistGas());
    m_gasPressureSpin->setValue(m_settings.gasPressure());
    m_waterCoolingCheck->setChecked(m_settings.waterCoolingEnabled());
    m_waterMinFlowSpin->setValue(m_settings.waterMinFlow());
    m_monitorEnabledCheck->setChecked(m_settings.monitorEnabled());
    m_monitorIntervalSpin->setValue(m_settings.monitorIntervalMs());
    m_loadingXSpin->setValue(m_settings.loadingPositionX());
    m_loadingYSpin->setValue(m_settings.loadingPositionY());
    m_loadingZSpin->setValue(m_settings.loadingPositionZ());
    m_cameraNameEdit->setText(m_settings.cameraName());
    m_cameraExposureSpin->setValue(m_settings.cameraExposureMs());
    m_internetHostEdit->setText(m_settings.internetHost());
    m_internetPortSpin->setValue(m_settings.internetPort());
    if (m_communicationPage)
        m_communicationPage->loadFromSettings(m_settings);
    loadLegacySettings();
}

void ProcessSettingsDialog::applyToSettings()
{
    m_settings.setSimulationMode(m_simulationModeCheck->isChecked());
    m_settings.setControllerEndpoint(m_endpointEdit->text().trimmed());
    m_settings.setMotionControllerName(m_motionControllerCombo->currentText());
    m_settings.setLaserDeviceName(m_laserDeviceCombo->currentText());
    m_settings.setLaserEnergy(m_laserEnergySpin->value());
    m_settings.setLaserFrequency(m_laserFrequencySpin->value());
    m_settings.setLaserPulseWidth(m_laserPulseWidthSpin->value());
    m_settings.setAxisTravelX(m_axisTravelXSpin->value());
    m_settings.setAxisTravelY(m_axisTravelYSpin->value());
    m_settings.setAxisTravelZ(m_axisTravelZSpin->value());
    m_settings.setAxisMaxVelocity(m_axisMaxVelocitySpin->value());
    m_settings.setAxisAcceleration(m_axisAccelerationSpin->value());
    m_settings.setToolFeedRate(m_toolFeedRateSpin->value());
    m_settings.setToolKerfWidth(m_toolKerfWidthSpin->value());
    m_settings.setPierceDelayMs(m_pierceDelaySpin->value());
    m_settings.setIoDefaultChannel(m_ioDefaultChannelEdit->text());
    m_settings.setIoDefaultValue(m_ioDefaultValueCheck->isChecked());
    m_settings.setAssistGas(m_assistGasEdit->text());
    m_settings.setGasPressure(m_gasPressureSpin->value());
    m_settings.setWaterCoolingEnabled(m_waterCoolingCheck->isChecked());
    m_settings.setWaterMinFlow(m_waterMinFlowSpin->value());
    m_settings.setMonitorEnabled(m_monitorEnabledCheck->isChecked());
    m_settings.setMonitorIntervalMs(m_monitorIntervalSpin->value());
    m_settings.setLoadingPositionX(m_loadingXSpin->value());
    m_settings.setLoadingPositionY(m_loadingYSpin->value());
    m_settings.setLoadingPositionZ(m_loadingZSpin->value());
    m_settings.setCameraName(m_cameraNameEdit->text());
    m_settings.setCameraExposureMs(m_cameraExposureSpin->value());
    m_settings.setInternetHost(m_internetHostEdit->text());
    m_settings.setInternetPort(m_internetPortSpin->value());
    if (m_communicationPage)
        m_communicationPage->applyToSettings(m_settings);
    QMap<QString, QString> uiValues = m_settings.uiSettingValues();
    applyLegacySettings(uiValues);
    m_settings.setUiSettingValues(uiValues);
    applyKnownLegacyTypedSettings(uiValues);
}

} // namespace lcnc::process
