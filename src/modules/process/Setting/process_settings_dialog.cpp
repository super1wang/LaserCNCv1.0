#include "modules/process/Setting/process_settings_dialog.h"

#include "modules/process/settings/process_settings.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLineEdit>
#include <QPushButton>
#include <QTabWidget>
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
    resize(460, 320);

    auto* layout = new QVBoxLayout(this);
    m_tabs = new QTabWidget(this);
    m_tabs->addTab(buildProcessPage(), tr("加工"));
    m_tabs->addTab(buildMotionPage(), tr("运动"));
    m_tabs->addTab(buildLaserPage(), tr("激光"));
    layout->addWidget(m_tabs);

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
        m_tabs->setCurrentIndex(1);
        break;
    case InitialPage::Laser:
        m_tabs->setCurrentIndex(2);
        break;
    case InitialPage::Process:
    default:
        m_tabs->setCurrentIndex(0);
        break;
    }
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

void ProcessSettingsDialog::loadFromSettings()
{
    m_simulationModeCheck->setChecked(m_settings.simulationMode());
    m_endpointEdit->setText(m_settings.controllerEndpoint());
    m_motionControllerCombo->setCurrentText(m_settings.motionControllerName());
    m_laserDeviceCombo->setCurrentText(m_settings.laserDeviceName());
    m_laserEnergySpin->setValue(m_settings.laserEnergy());
    m_laserFrequencySpin->setValue(m_settings.laserFrequency());
    m_laserPulseWidthSpin->setValue(m_settings.laserPulseWidth());
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
}

} // namespace lcnc::process
