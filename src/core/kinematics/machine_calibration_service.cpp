#include "core/kinematics/machine_calibration_service.h"

#include "core/kinematics/controller_kinematics_snapshot.h"
#include "core/kinematics/machine_configuration_service.h"
#include "core/logging/logger.h"

#include <QCoreApplication>
#include <QCryptographicHash>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>
#include <QUuid>

#include <Standard_Failure.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <cmath>
#include <limits>

namespace lcnc::kinematics {
namespace {

QJsonArray vectorArray(const std::array<double, 3>& values)
{
    return {values[0], values[1], values[2]};
}

QJsonArray axesArray(const std::array<double, 5>& values)
{
    QJsonArray array;
    for (double value : values) array.append(value);
    return array;
}

bool readVector3(const QJsonValue& value, std::array<double, 3>* output)
{
    const QJsonArray array = value.toArray();
    if (!output || array.size() != 3)
        return false;
    for (int index = 0; index < 3; ++index) {
        (*output)[index] = array[index].toDouble(std::numeric_limits<double>::quiet_NaN());
        if (!std::isfinite((*output)[index])) return false;
    }
    return true;
}

bool readAxes5(const QJsonValue& value, std::array<double, 5>* output)
{
    const QJsonArray array = value.toArray();
    if (!output || array.size() != 5)
        return false;
    for (int index = 0; index < 5; ++index) {
        (*output)[index] = array[index].toDouble(std::numeric_limits<double>::quiet_NaN());
        if (!std::isfinite((*output)[index])) return false;
    }
    return true;
}

QJsonObject fitObject(const AxisLineFit& fit)
{
    QJsonArray rejected;
    for (const QString& id : fit.suggestedRejectedSampleIds) rejected.append(id);
    return {{QStringLiteral("axisName"), fit.axisName},
            {QStringLiteral("pointMcs"), vectorArray(fit.pointMcs)},
            {QStringLiteral("unitVectorMcs"), vectorArray(fit.unitVectorMcs)},
            {QStringLiteral("sampleCount"), fit.sampleCount},
            {QStringLiteral("angularCoverageDeg"), fit.angularCoverageDeg},
            {QStringLiteral("fittedRadiusMm"), fit.fittedRadiusMm},
            {QStringLiteral("rmsResidualMm"), fit.rmsResidualMm},
            {QStringLiteral("maxResidualMm"), fit.maxResidualMm},
            {QStringLiteral("conditionMetric"), fit.conditionMetric},
            {QStringLiteral("suggestedRejectedSampleIds"), rejected}};
}

bool readFit(const QJsonValue& value, AxisLineFit* fit)
{
    if (!fit || !value.isObject()) return false;
    const QJsonObject object = value.toObject();
    fit->axisName = object.value(QStringLiteral("axisName")).toString();
    if (fit->axisName.isEmpty()
        || !readVector3(object.value(QStringLiteral("pointMcs")), &fit->pointMcs)
        || !readVector3(object.value(QStringLiteral("unitVectorMcs")), &fit->unitVectorMcs)) {
        return false;
    }
    fit->sampleCount = object.value(QStringLiteral("sampleCount")).toInt();
    fit->angularCoverageDeg = object.value(QStringLiteral("angularCoverageDeg")).toDouble();
    fit->fittedRadiusMm = object.value(QStringLiteral("fittedRadiusMm")).toDouble();
    fit->rmsResidualMm = object.value(QStringLiteral("rmsResidualMm")).toDouble();
    fit->maxResidualMm = object.value(QStringLiteral("maxResidualMm")).toDouble();
    fit->conditionMetric = object.value(QStringLiteral("conditionMetric")).toDouble();
    fit->suggestedRejectedSampleIds.clear();
    for (const QJsonValue& id : object.value(QStringLiteral("suggestedRejectedSampleIds")).toArray())
        fit->suggestedRejectedSampleIds.append(id.toString());
    return true;
}

QJsonObject sampleObject(const CalibrationSample& sample)
{
    return {{QStringLiteral("sampleId"), sample.sampleId},
            {QStringLiteral("targetAxisName"), sample.targetAxisName},
            {QStringLiteral("targetAxisSlot"), sample.targetAxisSlot},
            {QStringLiteral("actualAxes"), axesArray(sample.actualAxes)},
            {QStringLiteral("measuredReferencePointMcs"), vectorArray(sample.measuredReferencePointMcs)},
            {QStringLiteral("measurementSource"), sample.measurementSource},
            {QStringLiteral("timestampUtc"), sample.timestampUtc},
            {QStringLiteral("accepted"), sample.accepted}};
}

bool readSample(const QJsonValue& value, CalibrationSample* sample)
{
    if (!sample || !value.isObject()) return false;
    const QJsonObject object = value.toObject();
    sample->sampleId = object.value(QStringLiteral("sampleId")).toString();
    sample->targetAxisName = object.value(QStringLiteral("targetAxisName")).toString();
    sample->targetAxisSlot = object.value(QStringLiteral("targetAxisSlot")).toInt(-1);
    sample->measurementSource = object.value(QStringLiteral("measurementSource")).toString();
    sample->timestampUtc = object.value(QStringLiteral("timestampUtc")).toString();
    sample->accepted = object.value(QStringLiteral("accepted")).toBool(true);
    return !sample->sampleId.isEmpty() && !sample->targetAxisName.isEmpty()
        && readAxes5(object.value(QStringLiteral("actualAxes")), &sample->actualAxes)
        && readVector3(object.value(QStringLiteral("measuredReferencePointMcs")),
                       &sample->measuredReferencePointMcs);
}

QJsonObject recordObject(const MachineCalibrationRecord& record,
                         bool includeFingerprint)
{
    QJsonArray samples;
    for (const CalibrationSample& sample : record.samples)
        samples.append(sampleObject(sample));
    QJsonObject object{
        {QStringLiteral("schemaVersion"), record.schemaVersion},
        {QStringLiteral("calibrationId"), record.calibrationId},
        {QStringLiteral("machineIdentity"), record.machineIdentity},
        {QStringLiteral("nominalConfigurationFingerprint"), record.nominalConfigurationFingerprint},
        {QStringLiteral("controllerModelType"), record.controllerModelType},
        {QStringLiteral("primaryAxis"), fitObject(record.primaryAxis)},
        {QStringLiteral("slaveAxis"), fitObject(record.slaveAxis)},
        {QStringLiteral("samples"), samples},
        {QStringLiteral("rawSamplesSha256"), record.rawSamplesSha256},
        {QStringLiteral("createdAtUtc"), record.createdAtUtc},
        {QStringLiteral("operatorName"), record.operatorName},
        {QStringLiteral("measurementDevice"), record.measurementDevice},
        {QStringLiteral("softwareVersion"), record.softwareVersion}};
    object.insert(QStringLiteral("tool"), QJsonObject{
        {QStringLiteral("toolId"), record.tool.toolId},
        {QStringLiteral("toolLocationPointMcs"), vectorArray(record.tool.toolLocationPointMcs)},
        {QStringLiteral("fingerprint"), record.tool.fingerprint}});
    object.insert(QStringLiteral("verification"), QJsonObject{
        {QStringLiteral("state"), calibrationVerificationStateName(record.verification.state)},
        {QStringLiteral("controllerTransformMaxAxisError"), record.verification.controllerTransformMaxAxisError},
        {QStringLiteral("fixedTcpRmsErrorMm"), record.verification.fixedTcpRmsErrorMm},
        {QStringLiteral("fixedTcpMaxErrorMm"), record.verification.fixedTcpMaxErrorMm},
        {QStringLiteral("verifiedAtUtc"), record.verification.verifiedAtUtc},
        {QStringLiteral("notes"), record.verification.notes}});
    if (includeFingerprint)
        object.insert(QStringLiteral("calibrationFingerprint"), record.calibrationFingerprint);
    return object;
}

bool parseRecord(const QByteArray& bytes, MachineCalibrationRecord* record,
                 QString* error)
{
    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(bytes, &parseError);
    if (!record || parseError.error != QJsonParseError::NoError || !document.isObject()) {
        if (error) *error = QStringLiteral("Invalid calibration JSON: %1").arg(parseError.errorString());
        return false;
    }
    const QJsonObject object = document.object();
    MachineCalibrationRecord parsed;
    parsed.schemaVersion = object.value(QStringLiteral("schemaVersion")).toInt();
    parsed.calibrationId = object.value(QStringLiteral("calibrationId")).toString();
    parsed.machineIdentity = object.value(QStringLiteral("machineIdentity")).toString();
    parsed.nominalConfigurationFingerprint = object.value(QStringLiteral("nominalConfigurationFingerprint")).toString();
    parsed.controllerModelType = object.value(QStringLiteral("controllerModelType")).toString();
    parsed.rawSamplesSha256 = object.value(QStringLiteral("rawSamplesSha256")).toString();
    parsed.calibrationFingerprint = object.value(QStringLiteral("calibrationFingerprint")).toString();
    parsed.createdAtUtc = object.value(QStringLiteral("createdAtUtc")).toString();
    parsed.operatorName = object.value(QStringLiteral("operatorName")).toString();
    parsed.measurementDevice = object.value(QStringLiteral("measurementDevice")).toString();
    parsed.softwareVersion = object.value(QStringLiteral("softwareVersion")).toString();
    if ((parsed.schemaVersion != 1
         && parsed.schemaVersion != MachineCalibrationRecord::kCurrentSchemaVersion)
        || parsed.calibrationId.isEmpty()
        || !readFit(object.value(QStringLiteral("primaryAxis")), &parsed.primaryAxis)
        || !readFit(object.value(QStringLiteral("slaveAxis")), &parsed.slaveAxis)) {
        if (error) *error = QStringLiteral("Calibration record is incomplete or unsupported");
        return false;
    }
    for (const QJsonValue& value : object.value(QStringLiteral("samples")).toArray()) {
        CalibrationSample sample;
        if (!readSample(value, &sample)) {
            if (error) *error = QStringLiteral("Calibration record contains an invalid sample");
            return false;
        }
        parsed.samples.append(sample);
    }
    const QJsonObject tool = object.value(QStringLiteral("tool")).toObject();
    parsed.tool.toolId = tool.value(QStringLiteral("toolId")).toString();
    parsed.tool.fingerprint = tool.value(QStringLiteral("fingerprint")).toString();
    if (!readVector3(tool.value(QStringLiteral("toolLocationPointMcs")),
                     &parsed.tool.toolLocationPointMcs)) {
        if (error) *error = QStringLiteral("Calibration record contains an invalid tool point");
        return false;
    }
    const QJsonObject verification = object.value(QStringLiteral("verification")).toObject();
    if (!calibrationVerificationStateFromName(
            verification.value(QStringLiteral("state")).toString(),
            &parsed.verification.state)) {
        if (error) *error = QStringLiteral("Calibration verification state is invalid");
        return false;
    }
    parsed.verification.controllerTransformMaxAxisError = verification.value(QStringLiteral("controllerTransformMaxAxisError")).toDouble();
    parsed.verification.fixedTcpRmsErrorMm = verification.value(QStringLiteral("fixedTcpRmsErrorMm")).toDouble();
    parsed.verification.fixedTcpMaxErrorMm = verification.value(QStringLiteral("fixedTcpMaxErrorMm")).toDouble();
    parsed.verification.verifiedAtUtc = verification.value(QStringLiteral("verifiedAtUtc")).toString();
    parsed.verification.notes = verification.value(QStringLiteral("notes")).toString();
    if (MachineCalibrationService::computeSamplesSha256(parsed.samples) != parsed.rawSamplesSha256
        || MachineCalibrationService::computeCalibrationFingerprint(parsed) != parsed.calibrationFingerprint) {
        if (error) *error = QStringLiteral("Calibration record hash verification failed");
        return false;
    }
    *record = parsed;
    return true;
}

bool writeJsonAtomically(const QString& path, const QJsonObject& object,
                         QString* error)
{
    QDir().mkpath(QFileInfo(path).absolutePath());
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly)) {
        if (error) *error = file.errorString();
        return false;
    }
    if (file.write(QJsonDocument(object).toJson(QJsonDocument::Indented)) < 0
        || !file.commit()) {
        if (error) *error = file.errorString();
        return false;
    }
    return true;
}

} // namespace

QString calibrationVerificationStateName(CalibrationVerificationState state)
{
    switch (state) {
    case CalibrationVerificationState::Draft: return QStringLiteral("Draft");
    case CalibrationVerificationState::ConfigurationDerived: return QStringLiteral("ConfigurationDerived");
    case CalibrationVerificationState::Computed: return QStringLiteral("Computed");
    case CalibrationVerificationState::ControllerVerified: return QStringLiteral("ControllerVerified");
    case CalibrationVerificationState::MachineVerified: return QStringLiteral("MachineVerified");
    }
    return QStringLiteral("Draft");
}

bool calibrationVerificationStateFromName(const QString& name,
                                           CalibrationVerificationState* state)
{
    if (!state) return false;
    if (name == QStringLiteral("Draft")) *state = CalibrationVerificationState::Draft;
    else if (name == QStringLiteral("ConfigurationDerived"))
        *state = CalibrationVerificationState::ConfigurationDerived;
    else if (name == QStringLiteral("Computed")) *state = CalibrationVerificationState::Computed;
    else if (name == QStringLiteral("ControllerVerified")) *state = CalibrationVerificationState::ControllerVerified;
    else if (name == QStringLiteral("MachineVerified")) *state = CalibrationVerificationState::MachineVerified;
    else return false;
    return true;
}

MachineCalibrationService::MachineCalibrationService(QString storageRoot,
                                                     QObject* parent)
    : QObject(parent)
    , m_storageRoot(std::move(storageRoot))
{
    if (m_storageRoot.trimmed().isEmpty())
        m_storageRoot = QDir(QCoreApplication::applicationDirPath())
                            .filePath(QStringLiteral("config/calibrations"));
}

QString MachineCalibrationService::storageRoot() const
{
    return QDir::cleanPath(QDir(m_storageRoot).absolutePath());
}

QString MachineCalibrationService::recordPath(const QString& calibrationId) const
{
    return QDir(storageRoot()).filePath(calibrationId + QStringLiteral(".json"));
}

QString MachineCalibrationService::activePointerPath() const
{
    return QDir(storageRoot()).filePath(QStringLiteral("active.json"));
}

QString MachineCalibrationService::activeCalibrationId() const
{
    QFile file(activePointerPath());
    if (!file.open(QIODevice::ReadOnly)) return {};
    const QJsonDocument document = QJsonDocument::fromJson(file.readAll());
    return document.object().value(QStringLiteral("activeCalibrationId")).toString();
}

QString MachineCalibrationService::activeMachineConfigurationFingerprint() const
{
    QFile file(activePointerPath());
    if (!file.open(QIODevice::ReadOnly)) return {};
    const QJsonDocument document = QJsonDocument::fromJson(file.readAll());
    return document.object()
        .value(QStringLiteral("machineConfigurationFingerprint")).toString();
}

bool MachineCalibrationService::activeRecord(MachineCalibrationRecord* value,
                                             QString* error) const
{
    QFile pointerFile(activePointerPath());
    if (!pointerFile.open(QIODevice::ReadOnly)) {
        if (error) *error = QStringLiteral("No active machine calibration");
        return false;
    }
    const QJsonObject pointer = QJsonDocument::fromJson(pointerFile.readAll()).object();
    if (pointer.value(QStringLiteral("schemaVersion")).toInt() != 1) {
        if (error) *error = QStringLiteral("The active calibration pointer is invalid");
        return false;
    }
    const QString id = pointer.value(QStringLiteral("activeCalibrationId")).toString();
    if (id.isEmpty()) {
        if (error) *error = QStringLiteral("No active machine calibration");
        return false;
    }
    MachineCalibrationRecord loaded;
    if (!record(id, &loaded, error))
        return false;
    if (pointer.value(QStringLiteral("calibrationFingerprint")).toString()
        != loaded.calibrationFingerprint) {
        if (error) *error = QStringLiteral("The active calibration pointer fingerprint is invalid");
        return false;
    }
    if (value) *value = loaded;
    return true;
}

bool MachineCalibrationService::record(const QString& calibrationId,
                                       MachineCalibrationRecord* value,
                                       QString* error) const
{
    QFile file(recordPath(calibrationId));
    if (!file.open(QIODevice::ReadOnly)) {
        if (error) *error = file.errorString();
        return false;
    }
    return parseRecord(file.readAll(), value, error);
}

QString MachineCalibrationService::computeSamplesSha256(
    const QVector<CalibrationSample>& samples)
{
    QJsonArray array;
    for (const CalibrationSample& sample : samples) array.append(sampleObject(sample));
    return QString::fromLatin1(QCryptographicHash::hash(
        QJsonDocument(array).toJson(QJsonDocument::Compact),
        QCryptographicHash::Sha256).toHex());
}

QString MachineCalibrationService::computeCalibrationFingerprint(
    const MachineCalibrationRecord& record)
{
    return QString::fromLatin1(QCryptographicHash::hash(
        QJsonDocument(recordObject(record, false)).toJson(QJsonDocument::Compact),
        QCryptographicHash::Sha256).toHex());
}

bool MachineCalibrationService::saveCandidate(MachineCalibrationRecord value,
                                              QString* savedId,
                                              QString* error) const
{
    if (value.schemaVersion != MachineCalibrationRecord::kCurrentSchemaVersion) {
        if (error) *error = QStringLiteral("New calibration records must use right-handed machine-world MCS (schema 2)");
        return false;
    }
    const bool configurationDerived = value.verification.state
        == CalibrationVerificationState::ConfigurationDerived;
    if (value.primaryAxis.axisName.isEmpty() || value.slaveAxis.axisName.isEmpty()
        || (!configurationDerived && value.samples.isEmpty())) {
        if (error) *error = QStringLiteral("Calibration candidate is incomplete");
        return false;
    }
    if (value.calibrationId.isEmpty())
        value.calibrationId = QUuid::createUuid().toString(QUuid::WithoutBraces);
    if (value.createdAtUtc.isEmpty())
        value.createdAtUtc = QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);
    value.rawSamplesSha256 = computeSamplesSha256(value.samples);
    value.calibrationFingerprint = computeCalibrationFingerprint(value);
    const QString path = recordPath(value.calibrationId);
    if (QFileInfo::exists(path)) {
        if (error) *error = QStringLiteral("Calibration record already exists and is immutable");
        return false;
    }
    if (!writeJsonAtomically(path, recordObject(value, true), error))
        return false;
    if (savedId) *savedId = value.calibrationId;
    return true;
}

bool MachineCalibrationService::createConfigurationDerivedCandidate(
    const MachineConfigurationService& machine,
    const std::array<double, 3>& toolLocationPointMcs,
    QString* savedId,
    QString* error) const
{
    ControllerKinematicsSnapshot snapshot;
    if (!buildControllerKinematicsSnapshot(
            machine, nullptr, ControllerCalibrationRequirement::None,
            &snapshot, error)) {
        return false;
    }
    for (double value : toolLocationPointMcs) {
        if (!std::isfinite(value)) {
            if (error) *error = QStringLiteral("The coarse TCP contains a non-finite value");
            return false;
        }
    }

    MachineCalibrationRecord record;
    record.machineIdentity = machine.presetName();
    record.nominalConfigurationFingerprint = machine.configurationFingerprint();
    record.controllerModelType = QString::number(snapshot.modelType);
    record.primaryAxis.axisName = snapshot.primaryAxisName;
    record.primaryAxis.pointMcs = snapshot.primaryAxisPointMcs;
    record.primaryAxis.unitVectorMcs = snapshot.axisVectorsMcs[3];
    record.slaveAxis.axisName = snapshot.slaveAxisName;
    record.slaveAxis.pointMcs = snapshot.slaveAxisPointMcs;
    record.slaveAxis.unitVectorMcs = snapshot.axisVectorsMcs[4];
    record.tool.toolId = QStringLiteral("configuration-derived-trial-tool");
    record.tool.toolLocationPointMcs = toolLocationPointMcs;
    QByteArray toolPayload = record.tool.toolId.toUtf8();
    for (double value : toolLocationPointMcs)
        toolPayload += QByteArray::number(value, 'g', 17);
    record.tool.fingerprint = QString::fromLatin1(QCryptographicHash::hash(
        toolPayload, QCryptographicHash::Sha256).toHex());
    record.verification.state = CalibrationVerificationState::ConfigurationDerived;
    record.verification.notes = QStringLiteral(
        "Generated from the active machine configuration; not a precision machine-verified calibration");
    record.measurementDevice = QStringLiteral("machine-configuration");
    record.softwareVersion = QCoreApplication::applicationVersion();
    return saveCandidate(record, savedId, error);
}

bool MachineCalibrationService::activate(const QString& calibrationId,
                                         MachineConfigurationService* machine,
                                         QString* error)
{
    if (!machine) {
        if (error) *error = QStringLiteral("Machine configuration service is unavailable");
        return false;
    }
    MachineCalibrationRecord value;
    if (!record(calibrationId, &value, error))
        return false;
    if (value.schemaVersion != MachineCalibrationRecord::kCurrentSchemaVersion) {
        if (error) *error = QStringLiteral(
            "Legacy calibration cannot be activated; regenerate or remeasure it in right-handed machine-world MCS");
        return false;
    }
    QVector<MachineAxisRuntimeConfig> configs = machine->axisConfigurations();
    bool primaryFound = false;
    bool slaveFound = false;
    for (MachineAxisRuntimeConfig& config : configs) {
        const AxisLineFit* fit = nullptr;
        if (config.axis.name.compare(value.primaryAxis.axisName, Qt::CaseInsensitive) == 0) {
            fit = &value.primaryAxis;
            primaryFound = true;
        } else if (config.axis.name.compare(value.slaveAxis.axisName, Qt::CaseInsensitive) == 0) {
            fit = &value.slaveAxis;
            slaveFound = true;
        }
        if (!fit) continue;
        // V2 MCS is already right-handed machine world. Only persisted origins
        // use axis coordinates; directions must not be projected a second time.
        // 中文翻译：V2 MCS 已是右手机床世界系；仅中心转为配置轴坐标，方向不得二次转换。
        if (!machine->worldToAxisCoordinates(
                gp_Pnt(fit->pointMcs[0], fit->pointMcs[1], fit->pointMcs[2]),
                &config.axis.origin)) {
            if (error) *error = QStringLiteral(
                "Cannot convert the calibrated MCS rotary center to controller-axis coordinates");
            return false;
        }
        try {
            config.axis.direction = gp_Dir(
                fit->unitVectorMcs[0], fit->unitVectorMcs[1], fit->unitVectorMcs[2]);
        } catch (const Standard_Failure&) {
            LCNC_ERR(lcnc::LogCode::Generic,
                     "machine.calibration: operation=activate reason=invalid_axis_vector");
            if (error) *error = QStringLiteral("Calibrated axis vector is invalid");
            return false;
        }
    }
    if (!primaryFound || !slaveFound) {
        if (error) *error = QStringLiteral("Calibration axes do not match the active machine configuration");
        return false;
    }
    QString validationError;
    const QVector<MachineAxisRuntimeConfig> oldConfigs = machine->axisConfigurations();
    machine->setAxisConfigurations(configs);
    if (!machine->validateConfiguration(&validationError) || !machine->saveDefault()) {
        machine->setAxisConfigurations(oldConfigs);
        if (error) *error = validationError.isEmpty()
            ? QStringLiteral("Cannot save the calibrated machine configuration")
            : validationError;
        return false;
    }
    const QJsonObject pointer{
        {QStringLiteral("schemaVersion"), 1},
        {QStringLiteral("activeCalibrationId"), calibrationId},
        {QStringLiteral("calibrationFingerprint"), value.calibrationFingerprint},
        {QStringLiteral("machineConfigurationFingerprint"),
         machine->configurationFingerprint()},
        {QStringLiteral("activatedAtUtc"), QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs)}};
    if (!writeJsonAtomically(activePointerPath(), pointer, error)) {
        machine->setAxisConfigurations(oldConfigs);
        machine->saveDefault();
        return false;
    }
    LCNC_INFO(lcnc::LogCode::Generic,
              "machine.calibration: operation=activate id={} fingerprint={} state={} result=success",
              calibrationId.toStdString(), value.calibrationFingerprint.toStdString(),
              calibrationVerificationStateName(value.verification.state).toStdString());
    emit activeCalibrationChanged(calibrationId);
    return true;
}

} // namespace lcnc::kinematics
