#pragma once

#include "core/kernel/i_service.h"
#include "core/kinematics/machine_calibration_record.h"

#include <QObject>

namespace lcnc {

class MachineConfigurationService;

namespace kinematics {

class MachineCalibrationService final : public QObject, public IService
{
    Q_OBJECT
public:
    explicit MachineCalibrationService(QString storageRoot = {},
                                       QObject* parent = nullptr);

    QString storageRoot() const;
    QString activeCalibrationId() const;
    QString activeMachineConfigurationFingerprint() const;
    bool activeRecord(MachineCalibrationRecord* record,
                      QString* error = nullptr) const;
    bool record(const QString& calibrationId,
                MachineCalibrationRecord* record,
                QString* error = nullptr) const;
    bool saveCandidate(MachineCalibrationRecord record,
                       QString* savedId = nullptr,
                       QString* error = nullptr) const;
    bool createConfigurationDerivedCandidate(
        const MachineConfigurationService& machine,
        const std::array<double, 3>& toolLocationPointMcs,
        QString* savedId = nullptr,
        QString* error = nullptr) const;
    bool activate(const QString& calibrationId,
                  MachineConfigurationService* machine,
                  QString* error = nullptr);

    static QString computeSamplesSha256(const QVector<CalibrationSample>& samples);
    static QString computeCalibrationFingerprint(const MachineCalibrationRecord& record);

signals:
    void activeCalibrationChanged(const QString& calibrationId);

private:
    QString recordPath(const QString& calibrationId) const;
    QString activePointerPath() const;

    QString m_storageRoot;
};

} // namespace kinematics
} // namespace lcnc
