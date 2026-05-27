#include "modules/process/Setting/process_settings_dialog.h"

#include "modules/process/settings/process_settings.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QStackedWidget>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QHBoxLayout>
#include <QVBoxLayout>

namespace lcnc::process {

namespace {

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

        auto* processRoot = addPageNode(nullptr, tr("Process"), processPage);
        addPageNode(processRoot, tr("运行"), processPage);
        addPageNode(processRoot, tr("Axis"), axisPage);
        addPageNode(processRoot, tr("Tool"), toolPage);
        auto* deviceRoot = addPageNode(nullptr, tr("External"), motionPage);
        addPageNode(deviceRoot, tr("Motion Controller"), motionPage);
        addPageNode(deviceRoot, tr("Laser"), laserPage);
        addPageNode(deviceRoot, tr("IO"), ioPage);
        addPageNode(deviceRoot, tr("Gas"), gasPage);
        addPageNode(deviceRoot, tr("Water"), waterPage);
        addPageNode(deviceRoot, tr("Monitor"), monitorPage);
        addPageNode(deviceRoot, tr("LoadingPos"), loadingPage);
        addPageNode(deviceRoot, tr("Camera"), cameraPage);
        addPageNode(deviceRoot, tr("Internet"), internetPage);
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
        m_pages->setCurrentIndex(motionPage);
        break;
    case InitialPage::Laser:
        m_pages->setCurrentIndex(laserPage);
        break;
    case InitialPage::Process:
    default:
        m_pages->setCurrentIndex(processPage);
        break;
    }
}

QTreeWidgetItem* ProcessSettingsDialog::addPageNode(QTreeWidgetItem* parent, const QString& text, int pageIndex)
{
    auto* item = parent
        ? new QTreeWidgetItem(parent)
        : new QTreeWidgetItem(m_pageTree);
    item->setText(0, text);
    item->setData(0, Qt::UserRole, pageIndex);
    return item;
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
}

} // namespace lcnc::process
