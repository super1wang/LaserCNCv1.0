#include "modules/cam/ui/machine/dialog_physical_kinematics_calibration_wizard.h"
#include "core/kinematics/machine_calibration_service.h"
#include "core/kinematics/machine_configuration_service.h"

#include <QApplication>
#include <QCheckBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QMessageBox>
#include <QLabel>
#include <QPushButton>
#include <QTableWidget>
#include <QTemporaryDir>
#include <QTimer>

#include <cmath>
#include <iostream>

int main(int argc, char** argv)
{
    qputenv("QT_QPA_PLATFORM", "offscreen");
    QApplication app(argc, argv);
    QTemporaryDir directory;
    if (!directory.isValid()) return 1;
    lcnc::MachineConfigurationService machine;
    machine.applyPreset(QStringLiteral("VERTICAL_AC_TABLE"));
    auto axes = machine.axisConfigurations();
    for (int i = 0; i < axes.size(); ++i) axes[i].controllerIndex = i + 1;
    machine.setAxisConfigurations(axes);
    lcnc::kinematics::MachineCalibrationService calibration(directory.path());
    lcnc::cam::ui::DialogPhysicalKinematicsCalibrationWizard wizard(&machine, &calibration);
    const QList<QTableWidget*> tables{
        wizard.findChild<QTableWidget*>(QStringLiteral("primarySamples")),
        wizard.findChild<QTableWidget*>(QStringLiteral("slaveSamples"))};
    const auto values = wizard.findChildren<QDoubleSpinBox*>();
    auto* verified = wizard.findChild<QCheckBox*>();
    QPushButton* save = nullptr;
    for (auto* button : wizard.findChildren<QPushButton*>())
        if (button->text() == QStringLiteral("Save candidate")) save = button;
    if (!tables[0] || !tables[1] || values.size() != 5 || !verified || !save) return 2;
    for (auto* table : tables) {
        const int rotaryColumn = table == tables.front() ? 4 : 5;
        for (int row = 0; row < table->rowCount(); ++row) {
            const double angle = table->item(row, rotaryColumn)->text().toDouble()
                * 3.14159265358979323846 / 180.0;
            table->item(row, 6)->setText(QString::number(50.0 * std::cos(angle), 'g', 17));
            table->item(row, 7)->setText(QString::number(50.0 * std::sin(angle), 'g', 17));
            table->item(row, 8)->setText(QStringLiteral("0"));
        }
    }
    const auto compute = [&] {
        QMetaObject::invokeMethod(&wizard, "computeCandidate", Qt::DirectConnection);
        if (!save->isEnabled())
            for (auto* label : wizard.findChildren<QLabel*>())
                std::cerr << label->text().toStdString() << '\n';
        return save->isEnabled();
    };
    verified->setChecked(true);
    if (!compute()) return 3;
    verified->setChecked(false);
    if (save->isEnabled() || !compute()) return 4;
    values.front()->setValue(12.0);
    if (save->isEnabled() || !compute()) return 5;
    tables.front()->item(0, 6)->setText(QStringLiteral("invalid"));
    if (save->isEnabled() || compute()) return 6;
    const double angle = tables.front()->item(0, 4)->text().toDouble()
        * 3.14159265358979323846 / 180.0;
    tables.front()->item(0, 6)->setText(QString::number(50.0 * std::cos(angle), 'g', 17));
    if (!compute()) return 7;
    const auto files = [&] { return QDir(directory.path()).entryList({QStringLiteral("*.json")}, QDir::Files); };
    // Without an app-owned admission callback, direct slot calls fail closed.
    QMetaObject::invokeMethod(&wizard, "saveAndActivate", Qt::DirectConnection);
    if (!files().isEmpty()) return 8;
    bool admitted = true;
    wizard.setActivationAdmission([&] { return admitted; });
    admitted = false; // connection/operation changed after computing
    QMetaObject::invokeMethod(&wizard, "saveAndActivate", Qt::DirectConnection);
    if (!files().isEmpty()) return 9;
    wizard.setMachiningInteractionLocked(true);
    QMetaObject::invokeMethod(&wizard, "saveCandidate", Qt::DirectConnection);
    if (wizard.isEnabled() || !files().isEmpty()) return 10;
    wizard.setMachiningInteractionLocked(false);
    // Save recomputes visible inputs even when no explicit Compute follows.
    QTimer::singleShot(0, [] {
        for (auto* widget : QApplication::topLevelWidgets())
            if (auto* box = qobject_cast<QMessageBox*>(widget)) box->accept();
    });
    QMetaObject::invokeMethod(&wizard, "saveCandidate", Qt::DirectConnection);
    const auto saved = files();
    if (saved.size() != 1) return 11;
    lcnc::kinematics::MachineCalibrationRecord record;
    QString error;
    if (!calibration.record(saved.front().chopped(5), &record, &error)
        || record.verification.state != lcnc::kinematics::CalibrationVerificationState::Computed
        || record.tool.toolLocationPointMcs[0] != 12.0) return 12;
    std::cout << "calibration wizard invalidation and write admission passed\n";
    return 0;
}
