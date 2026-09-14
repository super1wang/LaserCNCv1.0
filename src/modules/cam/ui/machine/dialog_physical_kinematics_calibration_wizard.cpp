#include "modules/cam/ui/machine/dialog_physical_kinematics_calibration_wizard.h"

#include "core/algorithms/kinematics/axis_line_calibration_solver.h"
#include "core/kinematics/controller_kinematics_snapshot.h"
#include "core/kinematics/machine_calibration_service.h"
#include "core/kinematics/machine_configuration_service.h"
#include "core/logging/logger.h"

#include <QCheckBox>
#include <QCoreApplication>
#include <QCryptographicHash>
#include <QDateTime>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QTabWidget>
#include <QTableWidget>
#include <QTextStream>
#include <QVBoxLayout>

#include <cmath>
#include <iterator>

namespace lcnc::cam::ui {
namespace {

constexpr int kColumnSampleId = 0;
constexpr int kColumnAxisX = 1;
constexpr int kColumnReferenceX = 6;

QString cellText(const QTableWidget* table, int row, int column)
{
    const QTableWidgetItem* item = table ? table->item(row, column) : nullptr;
    return item ? item->text().trimmed() : QString{};
}

void setCell(QTableWidget* table, int row, int column, const QString& value)
{
    auto* item = table->item(row, column);
    if (!item) {
        item = new QTableWidgetItem;
        table->setItem(row, column, item);
    }
    item->setText(value);
}

QString formatFit(const lcnc::kinematics::AxisLineFit& fit)
{
    QString summary = QObject::tr(
        "%1: center=(%2, %3, %4), vector=(%5, %6, %7), radius=%8 mm, "
        "coverage=%9°, RMS=%10 mm, max=%11 mm")
        .arg(fit.axisName)
        .arg(fit.pointMcs[0], 0, 'f', 6).arg(fit.pointMcs[1], 0, 'f', 6)
        .arg(fit.pointMcs[2], 0, 'f', 6)
        .arg(fit.unitVectorMcs[0], 0, 'f', 9)
        .arg(fit.unitVectorMcs[1], 0, 'f', 9)
        .arg(fit.unitVectorMcs[2], 0, 'f', 9)
        .arg(fit.fittedRadiusMm, 0, 'f', 6)
        .arg(fit.angularCoverageDeg, 0, 'f', 3)
        .arg(fit.rmsResidualMm, 0, 'f', 6)
        .arg(fit.maxResidualMm, 0, 'f', 6);
    if (!fit.suggestedRejectedSampleIds.isEmpty()) {
        // 中文翻译：；建议复核的异常样本=[%1]（不会自动排除）
        summary += QObject::tr(
            "; suggested outliers=[%1] (not excluded automatically)")
                       .arg(fit.suggestedRejectedSampleIds.join(QStringLiteral(", ")));
    }
    return summary;
}

} // namespace

DialogPhysicalKinematicsCalibrationWizard::DialogPhysicalKinematicsCalibrationWizard(
    lcnc::MachineConfigurationService* machine,
    lcnc::kinematics::MachineCalibrationService* calibration,
    QWidget* parent)
    : QDialog(parent), m_machine(machine), m_calibration(calibration)
{
    setAttribute(Qt::WA_DeleteOnClose);
    buildUi();
}

QTableWidget* DialogPhysicalKinematicsCalibrationWizard::createSampleTable(QWidget* parent)
{
    auto* table = new QTableWidget(0, 9, parent);
    // 中文翻译：样本；X；Y；Z；R1；R2；参考点 X；参考点 Y；参考点 Z。
    table->setHorizontalHeaderLabels({tr("Sample"), tr("X"), tr("Y"), tr("Z"),
                                      tr("R1"), tr("R2"), tr("Reference X"),
                                      tr("Reference Y"), tr("Reference Z")});
    table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    table->verticalHeader()->setVisible(false);
    table->setAlternatingRowColors(true);
    return table;
}

void DialogPhysicalKinematicsCalibrationWizard::buildUi()
{
    setWindowTitle(tr("Physical five-axis calibration wizard"));
    resize(1080, 720);
    auto* root = new QVBoxLayout(this);
    auto* intro = new QLabel(
        // 中文翻译：本向导使用真实轴反馈与机床坐标系中的实测参考点拟合两条旋转轴线。STEP 面拾取只用于理论配置，不能替代物理标定。
        tr("This wizard fits both rotary-axis lines from real axis feedback and measured reference points in MCS. STEP face picking configures nominal geometry and does not replace physical calibration."), this);
    intro->setWordWrap(true);
    root->addWidget(intro);
    auto* coordinateHint = new QLabel(
        // 中文翻译：参考点与刀尖用 Z 向上右手机床世界坐标填写；实际轴反馈保留控制器原值，不得取反。
        tr("Enter reference and tool points in right-handed, Z-up machine-world MCS. Keep actual axis feedback in native controller coordinates without sign changes."), this);
    coordinateHint->setWordWrap(true);
    root->addWidget(coordinateHint);

    if (m_machine) {
        for (const auto& axis : m_machine->axisConfigurations()) {
            if (axis.axis.role == lcnc::MachineAxisRole::LinearX)
                m_linearAxisNames[0] = axis.axis.name;
            else if (axis.axis.role == lcnc::MachineAxisRole::LinearY)
                m_linearAxisNames[1] = axis.axis.name;
            else if (axis.axis.role == lcnc::MachineAxisRole::LinearZ)
                m_linearAxisNames[2] = axis.axis.name;
            else if (axis.axis.role == lcnc::MachineAxisRole::TableTilt)
                m_primaryAxisName = axis.axis.name;
            else if (axis.axis.role == lcnc::MachineAxisRole::TableSpin)
                m_slaveAxisName = axis.axis.name;
        }
    }

    auto* tabs = new QTabWidget(this);
    const auto addAxisTab = [this, tabs](const QString& axisName, bool primary) {
        auto* page = new QWidget(tabs);
        auto* layout = new QVBoxLayout(page);
        auto* hint = new QLabel(
            tr("Keep the other rotary axis fixed. Move %1 through at least six poses spanning 180° or more. For every pose enter actual X/Y/Z/R1/R2 feedback and the measured MCS coordinates of the same reference point.")
                .arg(axisName), page);
        hint->setWordWrap(true);
        layout->addWidget(hint);
        auto* actions = new QHBoxLayout;
        auto* generate = new QPushButton(tr("Generate pose template"), page);
        auto* import = new QPushButton(tr("Import CSV..."), page);
        auto* capture = new QPushButton(tr("Fill selected row from current feedback"), page);
        actions->addWidget(generate);
        actions->addWidget(import);
        actions->addWidget(capture);
        actions->addStretch();
        layout->addLayout(actions);
        QTableWidget* table = createSampleTable(page);
        table->setObjectName(primary ? QStringLiteral("primarySamples") : QStringLiteral("slaveSamples"));
        layout->addWidget(table);
        if (primary) {
            m_primaryTable = table;
            connect(generate, &QPushButton::clicked, this,
                    &DialogPhysicalKinematicsCalibrationWizard::generatePrimaryTemplate);
            connect(import, &QPushButton::clicked, this,
                    &DialogPhysicalKinematicsCalibrationWizard::importPrimaryCsv);
            connect(capture, &QPushButton::clicked, this,
                    &DialogPhysicalKinematicsCalibrationWizard::capturePrimaryFeedback);
        } else {
            m_slaveTable = table;
            connect(generate, &QPushButton::clicked, this,
                    &DialogPhysicalKinematicsCalibrationWizard::generateSlaveTemplate);
            connect(import, &QPushButton::clicked, this,
                    &DialogPhysicalKinematicsCalibrationWizard::importSlaveCsv);
            connect(capture, &QPushButton::clicked, this,
                    &DialogPhysicalKinematicsCalibrationWizard::captureSlaveFeedback);
        }
        tabs->addTab(page, tr("Axis %1").arg(axisName));
    };
    addAxisTab(m_primaryAxisName.isEmpty() ? tr("Primary") : m_primaryAxisName, true);
    addAxisTab(m_slaveAxisName.isEmpty() ? tr("Slave") : m_slaveAxisName, false);
    root->addWidget(tabs, 1);

    auto* metadata = new QGroupBox(tr("Calibration record and verification"), this);
    auto* form = new QFormLayout(metadata);
    m_operator = new QLineEdit(metadata);
    m_device = new QLineEdit(metadata);
    m_toolId = new QLineEdit(QStringLiteral("default-tool"), metadata);
    auto* toolPoint = new QWidget(metadata);
    auto* toolPointLayout = new QHBoxLayout(toolPoint);
    toolPointLayout->setContentsMargins(0, 0, 0, 0);
    const auto head = m_machine ? m_machine->headToolGeometry() : lcnc::HeadToolGeometry{};
    const double defaults[3] = {head.installationOffsetX, head.installationOffsetY,
                                head.installationOffsetZ};
    for (int index = 0; index < 3; ++index) {
        m_toolPoint[index] = new QDoubleSpinBox(toolPoint);
        m_toolPoint[index]->setRange(-1000000.0, 1000000.0);
        m_toolPoint[index]->setDecimals(6);
        m_toolPoint[index]->setValue(defaults[index]);
        m_toolPoint[index]->setSuffix(QStringLiteral(" mm"));
        toolPointLayout->addWidget(m_toolPoint[index]);
    }
    m_machineVerified = new QCheckBox(
        tr("Fixed-TCP physical verification completed (RMS <= 0.10 mm, max <= 0.20 mm)"), metadata);
    m_fixedTcpRms = new QDoubleSpinBox(metadata);
    m_fixedTcpMax = new QDoubleSpinBox(metadata);
    for (QDoubleSpinBox* value : {m_fixedTcpRms, m_fixedTcpMax}) {
        value->setRange(0.0, 1000.0);
        value->setDecimals(6);
        value->setSuffix(QStringLiteral(" mm"));
    }
    form->addRow(tr("Operator:"), m_operator);
    form->addRow(tr("Measurement device:"), m_device);
    form->addRow(tr("Tool ID:"), m_toolId);
    form->addRow(tr("Tool location point in MCS:"), toolPoint);
    form->addRow(m_machineVerified);
    form->addRow(tr("Fixed-TCP RMS:"), m_fixedTcpRms);
    form->addRow(tr("Fixed-TCP maximum:"), m_fixedTcpMax);
    root->addWidget(metadata);

    m_result = new QLabel(this);
    m_result->setWordWrap(true);
    m_result->setTextInteractionFlags(Qt::TextSelectableByMouse);
    root->addWidget(m_result);
    auto* actions = new QHBoxLayout;
    auto* compute = new QPushButton(tr("Compute and validate"), this);
    m_save = new QPushButton(tr("Save candidate"), this);
    m_saveActivate = new QPushButton(tr("Save and activate"), this);
    m_save->setEnabled(false);
    m_saveActivate->setEnabled(false);
    actions->addWidget(compute);
    actions->addStretch();
    actions->addWidget(m_save);
    actions->addWidget(m_saveActivate);
    auto* closeButtons = new QDialogButtonBox(QDialogButtonBox::Close, this);
    actions->addWidget(closeButtons);
    root->addLayout(actions);
    connect(compute, &QPushButton::clicked, this,
            &DialogPhysicalKinematicsCalibrationWizard::computeCandidate);
    connect(m_save, &QPushButton::clicked, this,
            &DialogPhysicalKinematicsCalibrationWizard::saveCandidate);
    connect(m_saveActivate, &QPushButton::clicked, this,
            &DialogPhysicalKinematicsCalibrationWizard::saveAndActivate);
    connect(closeButtons, &QDialogButtonBox::rejected, this, &QDialog::reject);

    for (auto* table : {m_primaryTable, m_slaveTable})
        connect(table, &QTableWidget::itemChanged, this,
                &DialogPhysicalKinematicsCalibrationWizard::invalidateCandidate);
    for (auto* edit : {m_operator, m_device, m_toolId})
        connect(edit, &QLineEdit::textChanged, this,
                &DialogPhysicalKinematicsCalibrationWizard::invalidateCandidate);
    for (auto* value : {m_toolPoint[0], m_toolPoint[1], m_toolPoint[2],
                       m_fixedTcpRms, m_fixedTcpMax})
        connect(value, &QDoubleSpinBox::valueChanged, this,
                &DialogPhysicalKinematicsCalibrationWizard::invalidateCandidate);
    connect(m_machineVerified, &QCheckBox::toggled, this,
            &DialogPhysicalKinematicsCalibrationWizard::invalidateCandidate);
    if (m_machine)
        connect(m_machine, &lcnc::MachineConfigurationService::machineConfigurationChanged,
                this, &DialogPhysicalKinematicsCalibrationWizard::invalidateCandidate);

    generateTemplate(m_primaryTable, 3);
    generateTemplate(m_slaveTable, 4);
    if (!m_machine || !m_calibration || m_primaryAxisName.isEmpty() || m_slaveAxisName.isEmpty()) {
        compute->setEnabled(false);
        setResultText(tr("The active configuration does not provide one TableTilt and one TableSpin axis."), true);
    }
}

void DialogPhysicalKinematicsCalibrationWizard::invalidateCandidate()
{
    m_candidateReady = false;
    m_candidate = {};
    m_save->setEnabled(false);
    m_saveActivate->setEnabled(false);
}

void DialogPhysicalKinematicsCalibrationWizard::setActivationAdmission(
    std::function<bool()> admission)
{
    m_activationAdmission = std::move(admission);
}

void DialogPhysicalKinematicsCalibrationWizard::setMachiningInteractionLocked(bool locked)
{
    m_interactionLocked = locked;
    invalidateCandidate();
    setEnabled(!locked);
}

void DialogPhysicalKinematicsCalibrationWizard::generateTemplate(
    QTableWidget* table, int targetAxisSlot)
{
    if (!table) return;
    static const double angles[] = {-135.0, -90.0, -45.0, 0.0,
                                    45.0, 90.0, 135.0, 179.0};
    table->setRowCount(static_cast<int>(std::size(angles)));
    for (int row = 0; row < table->rowCount(); ++row) {
        setCell(table, row, kColumnSampleId, QStringLiteral("S%1").arg(row + 1, 2, 10, QLatin1Char('0')));
        for (int axis = 0; axis < 5; ++axis)
            setCell(table, row, kColumnAxisX + axis,
                    QString::number(axis == targetAxisSlot ? angles[row] : 0.0, 'f', 6));
        for (int component = 0; component < 3; ++component)
            setCell(table, row, kColumnReferenceX + component, QString{});
    }
    m_candidateReady = false;
    m_save->setEnabled(false);
    m_saveActivate->setEnabled(false);
}

void DialogPhysicalKinematicsCalibrationWizard::generatePrimaryTemplate()
{
    generateTemplate(m_primaryTable, 3);
}

void DialogPhysicalKinematicsCalibrationWizard::generateSlaveTemplate()
{
    generateTemplate(m_slaveTable, 4);
}

void DialogPhysicalKinematicsCalibrationWizard::capturePrimaryFeedback()
{
    emit currentAxisFeedbackRequested(TargetAxis::Primary);
}

void DialogPhysicalKinematicsCalibrationWizard::captureSlaveFeedback()
{
    emit currentAxisFeedbackRequested(TargetAxis::Slave);
}

void DialogPhysicalKinematicsCalibrationWizard::applyCurrentAxisFeedback(
    TargetAxis target, const QMap<QString, double>& positions)
{
    QTableWidget* table = target == TargetAxis::Primary ? m_primaryTable : m_slaveTable;
    if (!table) return;
    int row = table->currentRow();
    if (row < 0) {
        row = 0;
        for (int candidate = 0; candidate < table->rowCount(); ++candidate) {
            if (cellText(table, candidate, kColumnReferenceX).isEmpty()) {
                row = candidate;
                break;
            }
        }
    }
    const QString names[5]{m_linearAxisNames[0], m_linearAxisNames[1],
                           m_linearAxisNames[2], m_primaryAxisName, m_slaveAxisName};
    for (int axis = 0; axis < 5; ++axis) {
        if (names[axis].isEmpty() || !positions.contains(names[axis])) {
            setResultText(tr("Current controller feedback does not contain axis %1")
                              .arg(names[axis]), true);
            return;
        }
    }
    for (int axis = 0; axis < 5; ++axis)
        setCell(table, row, kColumnAxisX + axis,
                QString::number(positions.value(names[axis]), 'f', 6));
    table->selectRow(row);
    setResultText(tr("Filled row %1 with current actual axis feedback. Enter the measured MCS reference point before computing.")
                      .arg(row + 1));
    m_candidateReady = false;
    m_save->setEnabled(false);
    m_saveActivate->setEnabled(false);
}

void DialogPhysicalKinematicsCalibrationWizard::importPrimaryCsv()
{
    importCsv(m_primaryTable);
}

void DialogPhysicalKinematicsCalibrationWizard::importSlaveCsv()
{
    importCsv(m_slaveTable);
}

void DialogPhysicalKinematicsCalibrationWizard::importCsv(QTableWidget* table)
{
    const QString path = QFileDialog::getOpenFileName(
        this, tr("Import calibration samples"), {}, tr("CSV files (*.csv);;All files (*)"));
    if (path.isEmpty()) return;
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QMessageBox::critical(this, tr("Import failed"), file.errorString());
        return;
    }
    QTextStream stream(&file);
    QVector<QStringList> rows;
    while (!stream.atEnd()) {
        const QString line = stream.readLine().trimmed();
        if (line.isEmpty()) continue;
        rows.append(line.split(QLatin1Char(',')));
    }
    if (rows.isEmpty()) return;
    bool header = rows.front().value(1).trimmed().compare(QStringLiteral("X"), Qt::CaseInsensitive) == 0;
    const int first = header ? 1 : 0;
    table->setRowCount(qMax(0, rows.size() - first));
    for (int row = first; row < rows.size(); ++row) {
        const QStringList values = rows[row];
        const bool hasId = values.size() >= 9;
        setCell(table, row - first, 0, hasId ? values.value(0).trimmed()
                                             : QStringLiteral("S%1").arg(row - first + 1));
        for (int column = 1; column < 9; ++column)
            setCell(table, row - first, column,
                    values.value(hasId ? column : column - 1).trimmed());
    }
    m_candidateReady = false;
    m_save->setEnabled(false);
    m_saveActivate->setEnabled(false);
    setResultText(tr("Imported %1 sample rows from %2").arg(table->rowCount()).arg(path));
}

bool DialogPhysicalKinematicsCalibrationWizard::collectSamples(
    QTableWidget* table, const QString& axisName, int targetAxisSlot,
    QVector<lcnc::kinematics::CalibrationSample>* samples, QString* error) const
{
    if (!table || !samples) return false;
    for (int row = 0; row < table->rowCount(); ++row) {
        bool blank = true;
        for (int column = 1; column < 9; ++column)
            blank = blank && cellText(table, row, column).isEmpty();
        if (blank) continue;
        lcnc::kinematics::CalibrationSample sample;
        sample.sampleId = cellText(table, row, 0);
        if (sample.sampleId.isEmpty()) sample.sampleId = QStringLiteral("S%1").arg(row + 1);
        sample.sampleId = axisName + QLatin1Char('-') + sample.sampleId;
        sample.targetAxisName = axisName;
        sample.targetAxisSlot = targetAxisSlot;
        for (int axis = 0; axis < 5; ++axis) {
            bool ok = false;
            sample.actualAxes[axis] = cellText(table, row, kColumnAxisX + axis).toDouble(&ok);
            if (!ok || !std::isfinite(sample.actualAxes[axis])) {
                if (error) *error = tr("Row %1 of axis %2 has invalid actual-axis feedback").arg(row + 1).arg(axisName);
                return false;
            }
        }
        for (int component = 0; component < 3; ++component) {
            bool ok = false;
            sample.measuredReferencePointMcs[component] =
                cellText(table, row, kColumnReferenceX + component).toDouble(&ok);
            if (!ok || !std::isfinite(sample.measuredReferencePointMcs[component])) {
                if (error) *error = tr("Row %1 of axis %2 has an invalid measured MCS point").arg(row + 1).arg(axisName);
                return false;
            }
        }
        sample.measurementSource = m_device->text().trimmed();
        sample.timestampUtc = QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);
        samples->append(sample);
    }
    return true;
}

bool DialogPhysicalKinematicsCalibrationWizard::buildCandidate(
    lcnc::kinematics::MachineCalibrationRecord* record, QString* error)
{
    if (!record || !m_machine || !m_calibration) return false;
    QVector<lcnc::kinematics::CalibrationSample> samples;
    if (!collectSamples(m_primaryTable, m_primaryAxisName, 3, &samples, error)
        || !collectSamples(m_slaveTable, m_slaveAxisName, 4, &samples, error))
        return false;
    lcnc::kinematics::AxisLineCalibrationOptions options;
    options.minimumSamples = 6;
    options.minimumAngularCoverageDeg = 180.0;
    options.maximumRmsResidualMm = 0.20;
    options.maximumResidualMm = 0.50;
    options.maximumFixedRotaryDriftDeg = 0.10;
    const auto primary = lcnc::kinematics::fitRotaryAxisLine(samples, m_primaryAxisName, options);
    const auto slave = lcnc::kinematics::fitRotaryAxisLine(samples, m_slaveAxisName, options);
    if (!primary.success || !slave.success) {
        if (error) *error = !primary.success ? primary.error : slave.error;
        return false;
    }
    lcnc::kinematics::ControllerKinematicsSnapshot controller;
    QString controllerError;
    if (!lcnc::kinematics::buildControllerKinematicsSnapshot(
            *m_machine, nullptr,
            lcnc::kinematics::ControllerCalibrationRequirement::None,
            &controller, &controllerError)) {
        if (error) *error = controllerError;
        return false;
    }
    lcnc::kinematics::MachineCalibrationRecord value;
    value.machineIdentity = m_machine->presetName();
    value.nominalConfigurationFingerprint = m_machine->configurationFingerprint();
    value.controllerModelType = QString::number(controller.modelType);
    value.primaryAxis = primary.fit;
    value.slaveAxis = slave.fit;
    value.samples = samples;
    value.operatorName = m_operator->text().trimmed();
    value.measurementDevice = m_device->text().trimmed();
    value.softwareVersion = QCoreApplication::applicationVersion();
    value.tool.toolId = m_toolId->text().trimmed();
    QByteArray toolPayload = value.tool.toolId.toUtf8();
    for (int component = 0; component < 3; ++component) {
        value.tool.toolLocationPointMcs[component] = m_toolPoint[component]->value();
        toolPayload += QByteArray::number(value.tool.toolLocationPointMcs[component], 'g', 17);
    }
    value.tool.fingerprint = QString::fromLatin1(QCryptographicHash::hash(
        toolPayload, QCryptographicHash::Sha256).toHex());
    value.verification.fixedTcpRmsErrorMm = m_fixedTcpRms->value();
    value.verification.fixedTcpMaxErrorMm = m_fixedTcpMax->value();
    const bool verified = m_machineVerified->isChecked()
        && value.verification.fixedTcpRmsErrorMm <= 0.10
        && value.verification.fixedTcpMaxErrorMm <= 0.20;
    value.verification.state = verified
        ? lcnc::kinematics::CalibrationVerificationState::MachineVerified
        : lcnc::kinematics::CalibrationVerificationState::Computed;
    if (verified)
        value.verification.verifiedAtUtc = QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);
    value.verification.notes = verified
        ? tr("Operator-attested fixed-TCP verification passed the configured acceptance limits")
        : tr("Axis-line solution computed; fixed-TCP physical verification remains open");
    *record = value;
    return true;
}

void DialogPhysicalKinematicsCalibrationWizard::computeCandidate()
{
    if (m_interactionLocked)
        return;
    QString error;
    lcnc::kinematics::MachineCalibrationRecord candidate;
    if (!buildCandidate(&candidate, &error)) {
        m_candidateReady = false;
        m_save->setEnabled(false);
        m_saveActivate->setEnabled(false);
        setResultText(error, true);
        return;
    }
    m_candidate = candidate;
    m_candidateReady = true;
    m_save->setEnabled(true);
    m_saveActivate->setEnabled(true);
    setResultText(formatFit(candidate.primaryAxis) + QStringLiteral("\n")
                  + formatFit(candidate.slaveAxis) + QStringLiteral("\n")
                  + tr("Verification state: %1. RTCP remains blocked unless the state is MachineVerified.")
                        .arg(lcnc::kinematics::calibrationVerificationStateName(
                            candidate.verification.state)));
}

bool DialogPhysicalKinematicsCalibrationWizard::persist(bool activate)
{
    // Recheck at the write boundary: a previously opened modeless window
    // must not change the active model after machining/connection starts.
    if (m_interactionLocked || (activate
        && (!m_activationAdmission || !m_activationAdmission()))) {
        // 中文翻译：激活标定前请停止加工并断开控制器；设备操作必须已结束。
        setResultText(tr("Stop machining and disconnect the controller before activating calibration; device operations must be finished."), true);
        return false;
    }
    // Always rebuild from the visible inputs, including verification state.
    computeCandidate();
    if (!m_candidateReady) return false;
    QString savedId;
    QString error;
    const auto savedVerificationState = m_candidate.verification.state;
    if (!m_calibration->saveCandidate(m_candidate, &savedId, &error)) {
        setResultText(error, true);
        return false;
    }
    if (activate && !m_calibration->activate(savedId, m_machine, &error)) {
        setResultText(tr("Candidate %1 was saved but could not be activated: %2")
                          .arg(savedId, error), true);
        return false;
    }
    LCNC_INFO(lcnc::LogCode::Generic,
              "machine.calibration: operation=wizard_save id={} activate={} state={} result=success",
              savedId.toStdString(), activate,
              lcnc::kinematics::calibrationVerificationStateName(
                  savedVerificationState).toStdString());
    QMessageBox::information(this, tr("Calibration saved"),
        activate ? tr("Calibration %1 was saved and activated.").arg(savedId)
                 : tr("Calibration candidate %1 was saved.").arg(savedId));
    m_candidateReady = false;
    m_save->setEnabled(false);
    m_saveActivate->setEnabled(false);
    return true;
}

void DialogPhysicalKinematicsCalibrationWizard::saveCandidate()
{
    persist(false);
}

void DialogPhysicalKinematicsCalibrationWizard::saveAndActivate()
{
    persist(true);
}

void DialogPhysicalKinematicsCalibrationWizard::setResultText(
    const QString& text, bool error)
{
    m_result->setStyleSheet(error ? QStringLiteral("color:#c62828;")
                                  : QStringLiteral("color:#2e7d32;"));
    m_result->setText(text);
}

} // namespace lcnc::cam::ui
