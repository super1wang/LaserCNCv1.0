#pragma once

#include <QDialog>

#include <array>

class QCheckBox;
class QDoubleSpinBox;
class QLabel;
class QPushButton;

namespace lcnc {
class MachineConfigurationService;
namespace kinematics { class MachineCalibrationService; }
}

namespace lcnc::cam::ui {

/// Creates an immutable configuration-derived RTCP record from the active machine topology.
/// It never claims physical calibration accuracy. Execution uses normal
/// machining parameters after the operator explicitly enables this source.
class DialogConfigurationDerivedRtcp final : public QDialog
{
    Q_OBJECT
public:
    DialogConfigurationDerivedRtcp(
        lcnc::MachineConfigurationService* machine,
        lcnc::kinematics::MachineCalibrationService* calibration,
        QWidget* parent = nullptr);

    QString savedCalibrationId() const { return m_savedCalibrationId; }

private slots:
    void generateAndActivate();
    void setTcpToMcsOrigin();

private:
    lcnc::MachineConfigurationService* m_machine{nullptr};
    lcnc::kinematics::MachineCalibrationService* m_calibration{nullptr};
    std::array<QDoubleSpinBox*, 3> m_tcp{};
    QLabel* m_status{nullptr};
    QCheckBox* m_confirmation{nullptr};
    QPushButton* m_generate{nullptr};
    QString m_savedCalibrationId;
};

} // namespace lcnc::cam::ui
