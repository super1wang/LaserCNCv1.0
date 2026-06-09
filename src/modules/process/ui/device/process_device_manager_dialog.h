#pragma once

#include "modules/process/device/process_device_manager.h"
#include "modules/process/settings/process_settings.h"
#include "core/kinematics/machine_configuration_service.h"

#include <QDialog>
#include <QList>
#include <QMap>
#include <QStringList>
#include <QVector>

class QComboBox;
class QLabel;
class QListWidget;
class QPushButton;
class QStackedWidget;
class QTableWidget;
class QTimer;
class QVBoxLayout;
class QWidget;

namespace lcnc::process {

struct ProcessIoConsoleItem
{
    QString key;
    QString name;
    QString ioIndex;
    bool analog{false};
    bool output{false};
};

struct ProcessIoStateSnapshot
{
    QMap<QString, bool> digitalValues;
    QMap<QString, double> analogValues;
    QStringList errors;
};

/**
 * @brief Standalone Process peripheral manager and debug dialog.
 */
class ProcessDeviceManagerDialog : public QDialog
{
    Q_OBJECT
public:
    explicit ProcessDeviceManagerDialog(ProcessDeviceManager& manager,
                                        lcnc::ProcessSettings& settings,
                                        ProcessDeviceKind initialKind = ProcessDeviceKind::MotionController,
                                        QWidget* parent = nullptr);

signals:
    void settingsApplied();

private slots:
    void refreshPages();
    void applySettings();
    void switchCurrentPage(int row);

private:
    void buildUi();
    QWidget* buildMotionControllerPage();
    QWidget* buildLaserPage();
    QWidget* buildLegacyUiPage(const QString& pageId, const QString& resourcePath);
    QWidget* buildStatusPage(ProcessDeviceKind kind);
    QWidget* buildAxisConfigurationPage();
    QWidget* buildCustomIoPage(bool analog);
    QWidget* buildMotionDebugPage();
    QWidget* buildIoDebugPage();
    QWidget* buildLaserDebugPage();
    void registerLegacyEditors(const QString& pageId, QWidget* root);
    void loadEditorValues();
    void collectEditorValues(QMap<QString, QString>& values) const;
    void appendDeviceStatus(QVBoxLayout* layout, ProcessDeviceKind kind);
    ProcessDeviceSession roleSession(ProcessDeviceKind kind, bool* found = nullptr) const;
    void populateAxisConfigTable();
    QVector<lcnc::MachineAxisRuntimeConfig> readAxisConfigTable() const;
    void populateCustomIoTable(QTableWidget* table, const QVector<lcnc::ProcessIoTableEntry>& entries) const;
    QVector<lcnc::ProcessIoTableEntry> readCustomIoTable(QTableWidget* table) const;
    QList<ProcessIoConsoleItem> ioConsoleItems() const;
    void refreshIoStatesAsync();
    void applyIoStateSnapshot(const ProcessIoStateSnapshot& snapshot);
    void setDigitalIndicator(QPushButton* indicator, bool high, bool valid = true);
    void showError(const QString& message);

    ProcessDeviceManager& m_manager;
    lcnc::ProcessSettings& m_settings;
    QListWidget* m_roleList{nullptr};
    QStackedWidget* m_pages{nullptr};
    QComboBox* m_motionModelCombo{nullptr};
    QComboBox* m_laserModelCombo{nullptr};
    QTableWidget* m_axisConfigTable{nullptr};
    QTableWidget* m_customDigitalIoTable{nullptr};
    QTableWidget* m_customAnalogIoTable{nullptr};
    QTimer* m_ioRefreshTimer{nullptr};
    bool m_ioRefreshInFlight{false};
    QMap<QString, QWidget*> m_legacyEditors;
    QMap<QString, QPushButton*> m_digitalIndicators;
    QMap<QString, QLabel*> m_analogValueLabels;
    lcnc::MachineConfigurationService* m_machineConfig{nullptr};
};

} // namespace lcnc::process