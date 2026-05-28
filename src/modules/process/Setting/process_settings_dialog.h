#pragma once

#include <QDialog>
#include <QMap>
#include <QStringList>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLineEdit;
class QSpinBox;
class QStackedWidget;
class QTreeWidget;
class QTreeWidgetItem;

namespace lcnc { class ProcessSettings; }

namespace lcnc::process {

class CommunicationSettingsPage;

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
    QWidget* buildAxisPage();
    QWidget* buildToolPage();
    QWidget* buildIoPage();
    QWidget* buildGasPage();
    QWidget* buildWaterPage();
    QWidget* buildMonitorPage();
    QWidget* buildLoadingPage();
    QWidget* buildCameraPage();
    QWidget* buildInternetPage();
    QWidget* buildCommunicationPage();
    QWidget* buildLegacySettingsPage(const QString& pageId, const QString& title, const QString& resourcePath);
    void addLegacySettingsPages(QTreeWidgetItem* parent);
    void loadLegacySettings();
    void applyLegacySettings(QMap<QString, QString>& values) const;
    QTreeWidgetItem* addPageNode(QTreeWidgetItem* parent, const QString& text, int pageIndex);
    void switchPage(QTreeWidgetItem* item, int column);
    void loadFromSettings();
    void applyToSettings();

    lcnc::ProcessSettings& m_settings;
    QStringList m_motionControllers;
    QStringList m_laserDevices;
    QTreeWidget* m_pageTree{nullptr};
    QStackedWidget* m_pages{nullptr};
    QCheckBox* m_simulationModeCheck{nullptr};
    QLineEdit* m_endpointEdit{nullptr};
    QComboBox* m_motionControllerCombo{nullptr};
    QComboBox* m_laserDeviceCombo{nullptr};
    QDoubleSpinBox* m_laserEnergySpin{nullptr};
    QDoubleSpinBox* m_laserFrequencySpin{nullptr};
    QDoubleSpinBox* m_laserPulseWidthSpin{nullptr};
    QDoubleSpinBox* m_axisTravelXSpin{nullptr};
    QDoubleSpinBox* m_axisTravelYSpin{nullptr};
    QDoubleSpinBox* m_axisTravelZSpin{nullptr};
    QDoubleSpinBox* m_axisMaxVelocitySpin{nullptr};
    QDoubleSpinBox* m_axisAccelerationSpin{nullptr};
    QDoubleSpinBox* m_toolFeedRateSpin{nullptr};
    QDoubleSpinBox* m_toolKerfWidthSpin{nullptr};
    QSpinBox* m_pierceDelaySpin{nullptr};
    QLineEdit* m_ioDefaultChannelEdit{nullptr};
    QCheckBox* m_ioDefaultValueCheck{nullptr};
    QLineEdit* m_assistGasEdit{nullptr};
    QDoubleSpinBox* m_gasPressureSpin{nullptr};
    QCheckBox* m_waterCoolingCheck{nullptr};
    QDoubleSpinBox* m_waterMinFlowSpin{nullptr};
    QCheckBox* m_monitorEnabledCheck{nullptr};
    QSpinBox* m_monitorIntervalSpin{nullptr};
    QDoubleSpinBox* m_loadingXSpin{nullptr};
    QDoubleSpinBox* m_loadingYSpin{nullptr};
    QDoubleSpinBox* m_loadingZSpin{nullptr};
    QLineEdit* m_cameraNameEdit{nullptr};
    QSpinBox* m_cameraExposureSpin{nullptr};
    QLineEdit* m_internetHostEdit{nullptr};
    QSpinBox* m_internetPortSpin{nullptr};
    CommunicationSettingsPage* m_communicationPage{nullptr};
    QMap<QString, QWidget*> m_legacyEditors;
};

} // namespace lcnc::process
