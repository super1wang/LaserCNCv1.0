#pragma once

#include <QDialog>
#include <QStringList>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLineEdit;
class QTabWidget;

namespace lcnc { class ProcessSettings; }

namespace lcnc::process {

class ProcessSettingsDialog : public QDialog
{
public:
    enum class InitialPage {
        Process,
        Motion,
        Laser,
    };

    explicit ProcessSettingsDialog(lcnc::ProcessSettings& settings,
                                   const QStringList& motionControllers,
                                   const QStringList& laserDevices,
                                   InitialPage initialPage,
                                   QWidget* parent = nullptr);

private:
    QWidget* buildProcessPage();
    QWidget* buildMotionPage();
    QWidget* buildLaserPage();
    void loadFromSettings();
    void applyToSettings();

    lcnc::ProcessSettings& m_settings;
    QStringList m_motionControllers;
    QStringList m_laserDevices;
    QTabWidget* m_tabs{nullptr};
    QCheckBox* m_simulationModeCheck{nullptr};
    QLineEdit* m_endpointEdit{nullptr};
    QComboBox* m_motionControllerCombo{nullptr};
    QComboBox* m_laserDeviceCombo{nullptr};
    QDoubleSpinBox* m_laserEnergySpin{nullptr};
    QDoubleSpinBox* m_laserFrequencySpin{nullptr};
    QDoubleSpinBox* m_laserPulseWidthSpin{nullptr};
};

} // namespace lcnc::process
