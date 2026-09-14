#pragma once

#include "core/kinematics/machine_calibration_record.h"

#include <QDialog>
#include <QMap>
#include <functional>

class QCheckBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QTabWidget;
class QTableWidget;

namespace lcnc {
class MachineConfigurationService;
namespace kinematics { class MachineCalibrationService; }
}

namespace lcnc::cam::ui {

/// Guided physical calibration for the two rotary-axis lines used by the GTN
/// five-axis Group model.  It consumes real controller feedback and measured
/// MCS reference points; CAD face picking is intentionally a separate wizard.
class DialogPhysicalKinematicsCalibrationWizard final : public QDialog
{
    Q_OBJECT
public:
    enum class TargetAxis { Primary, Slave };
    Q_ENUM(TargetAxis)
    DialogPhysicalKinematicsCalibrationWizard(
        lcnc::MachineConfigurationService* machine,
        lcnc::kinematics::MachineCalibrationService* calibration,
        QWidget* parent = nullptr);
    void applyCurrentAxisFeedback(TargetAxis target,
                                  const QMap<QString, double>& positions);
    void setActivationAdmission(std::function<bool()> admission);
    void setMachiningInteractionLocked(bool locked);

signals:
    void currentAxisFeedbackRequested(
        lcnc::cam::ui::DialogPhysicalKinematicsCalibrationWizard::TargetAxis target);

private slots:
    void importPrimaryCsv();
    void importSlaveCsv();
    void generatePrimaryTemplate();
    void generateSlaveTemplate();
    void capturePrimaryFeedback();
    void captureSlaveFeedback();
    void computeCandidate();
    void saveCandidate();
    void saveAndActivate();

private:
    void buildUi();
    void invalidateCandidate();
    QTableWidget* createSampleTable(QWidget* parent);
    void generateTemplate(QTableWidget* table, int targetAxisSlot);
    void importCsv(QTableWidget* table);
    bool collectSamples(QTableWidget* table, const QString& axisName,
                        int targetAxisSlot,
                        QVector<lcnc::kinematics::CalibrationSample>* samples,
                        QString* error) const;
    bool buildCandidate(lcnc::kinematics::MachineCalibrationRecord* record,
                        QString* error);
    bool persist(bool activate);
    void setResultText(const QString& text, bool error = false);

    lcnc::MachineConfigurationService* m_machine{nullptr};
    lcnc::kinematics::MachineCalibrationService* m_calibration{nullptr};
    QString m_primaryAxisName;
    QString m_slaveAxisName;
    QString m_linearAxisNames[3];
    QTableWidget* m_primaryTable{nullptr};
    QTableWidget* m_slaveTable{nullptr};
    QLabel* m_result{nullptr};
    QLineEdit* m_operator{nullptr};
    QLineEdit* m_device{nullptr};
    QLineEdit* m_toolId{nullptr};
    QDoubleSpinBox* m_toolPoint[3]{};
    QCheckBox* m_machineVerified{nullptr};
    QDoubleSpinBox* m_fixedTcpRms{nullptr};
    QDoubleSpinBox* m_fixedTcpMax{nullptr};
    QPushButton* m_save{nullptr};
    QPushButton* m_saveActivate{nullptr};
    lcnc::kinematics::MachineCalibrationRecord m_candidate;
    bool m_candidateReady{false};
    bool m_interactionLocked{false};
    std::function<bool()> m_activationAdmission;
};

} // namespace lcnc::cam::ui
