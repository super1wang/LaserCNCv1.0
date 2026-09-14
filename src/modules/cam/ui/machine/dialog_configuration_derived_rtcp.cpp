#include "modules/cam/ui/machine/dialog_configuration_derived_rtcp.h"

#include "core/kinematics/controller_kinematics_snapshot.h"
#include "core/kinematics/machine_calibration_service.h"
#include "core/kinematics/machine_configuration_service.h"

#include <QCheckBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

namespace lcnc::cam::ui {
namespace {

QString pointText(const std::array<double, 3>& point)
{
    return QStringLiteral("[%1, %2, %3] mm")
        .arg(point[0], 0, 'f', 6)
        .arg(point[1], 0, 'f', 6)
        .arg(point[2], 0, 'f', 6);
}

QString vectorText(const std::array<double, 3>& vector)
{
    return QStringLiteral("[%1, %2, %3]")
        .arg(vector[0], 0, 'f', 9)
        .arg(vector[1], 0, 'f', 9)
        .arg(vector[2], 0, 'f', 9);
}

} // namespace

DialogConfigurationDerivedRtcp::DialogConfigurationDerivedRtcp(
    lcnc::MachineConfigurationService* machine,
    lcnc::kinematics::MachineCalibrationService* calibration,
    QWidget* parent)
    : QDialog(parent), m_machine(machine), m_calibration(calibration)
{
    // 中文翻译：生成构型派生 RTCP 参数
    setWindowTitle(tr("Generate configuration-derived RTCP parameters"));
    setAttribute(Qt::WA_DeleteOnClose, false);
    resize(680, 420);

    auto* root = new QVBoxLayout(this);
    auto* warning = new QLabel(
        // 中文翻译：这不是精密标定。软件将直接使用当前构型中的两条旋转轴线、方向、轴号和当量生成 RTCP 记录。
        tr("This is not a precision calibration. The two rotary-axis lines, directions, controller indices, and scales are copied directly from the active machine configuration into an RTCP record."),
        this);
    warning->setWordWrap(true);
    warning->setStyleSheet(QStringLiteral("color:#a06000; font-weight:600;"));
    root->addWidget(warning);

    auto* model = new QFormLayout;
    lcnc::kinematics::ControllerKinematicsSnapshot snapshot;
    QString snapshotError;
    const bool validSnapshot = m_machine
        && lcnc::kinematics::buildControllerKinematicsSnapshot(
            *m_machine, nullptr,
            lcnc::kinematics::ControllerCalibrationRequirement::None,
            &snapshot, &snapshotError);
    if (validSnapshot) {
        // 中文翻译：第一旋转轴线；第二旋转轴线
        model->addRow(tr("Primary rotary-axis line:"), new QLabel(
            snapshot.primaryAxisName + QStringLiteral("  P=")
                + pointText(snapshot.primaryAxisPointMcs) + QStringLiteral("  V=")
                + vectorText(snapshot.axisVectorsMcs[3]), this));
        model->addRow(tr("Secondary rotary-axis line:"), new QLabel(
            snapshot.slaveAxisName + QStringLiteral("  P=")
                + pointText(snapshot.slaveAxisPointMcs) + QStringLiteral("  V=")
                + vectorText(snapshot.axisVectorsMcs[4]), this));
    }
    root->addLayout(model);

    auto* tcpHint = new QLabel(
        // 中文翻译：输入机床零位时刀尖/焦点在 MCS 中的 XYZ。如果 MCS 原点就建在该点，请填 0,0,0。不要填焦距或旋转中心。
        tr("Enter the XYZ coordinates of the tool tip / laser focus in MCS at the machine zero pose. If the MCS origin was established at that point, enter 0, 0, 0. Do not enter the focal length or a rotary center."),
        this);
    tcpHint->setWordWrap(true);
    root->addWidget(tcpHint);
    auto* coordinateHint = new QLabel(
        // 中文翻译：MCS 是与 CAD/CAM 一致的 Z 向上右手机床世界系，不是轴反馈坐标。旧版标定必须重新生成。
        tr("MCS is the right-handed, Z-up machine-world frame shared with CAD/CAM, not axis feedback coordinates. Legacy calibration records must be regenerated."), this);
    coordinateHint->setWordWrap(true);
    root->addWidget(coordinateHint);

    auto* tcpRow = new QWidget(this);
    auto* tcpLayout = new QHBoxLayout(tcpRow);
    tcpLayout->setContentsMargins(0, 0, 0, 0);
    const auto head = m_machine ? m_machine->headToolGeometry() : lcnc::HeadToolGeometry{};
    const double defaults[3] = {head.installationOffsetX, head.installationOffsetY,
                                head.installationOffsetZ};
    const QString labels[3] = {QStringLiteral("X"), QStringLiteral("Y"), QStringLiteral("Z")};
    for (int index = 0; index < 3; ++index) {
        tcpLayout->addWidget(new QLabel(labels[index], tcpRow));
        m_tcp[index] = new QDoubleSpinBox(tcpRow);
        m_tcp[index]->setRange(-1000000.0, 1000000.0);
        m_tcp[index]->setDecimals(6);
        m_tcp[index]->setValue(defaults[index]);
        m_tcp[index]->setSuffix(QStringLiteral(" mm"));
        tcpLayout->addWidget(m_tcp[index]);
    }
    auto* zero = new QPushButton(
        // 中文翻译：设为 MCS 原点
        tr("Set to MCS origin"), tcpRow);
    tcpLayout->addWidget(zero);
    root->addWidget(tcpRow);
    connect(zero, &QPushButton::clicked, this,
            &DialogConfigurationDerivedRtcp::setTcpToMcsOrigin);

    m_confirmation = new QCheckBox(
        // 中文翻译：我已确认旋转中心和 TCP 的 MCS 坐标，并理解加工使用配置速度，且可能按刀具设置开启激光和气体。
        tr("I confirmed the rotary centers and TCP in MCS. I understand that machining uses configured speeds and may enable the laser and gas according to the tool settings."),
        this);
    root->addWidget(m_confirmation);

    m_status = new QLabel(this);
    m_status->setWordWrap(true);
    if (!validSnapshot) {
        m_status->setText(snapshotError);
        m_status->setStyleSheet(QStringLiteral("color:#b00020;"));
    }
    root->addWidget(m_status);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Cancel, this);
    m_generate = buttons->addButton(
        // 中文翻译：生成、激活并启用 RTCP 加工
        tr("Generate, activate, and enable RTCP machining"),
        QDialogButtonBox::AcceptRole);
    m_generate->setEnabled(validSnapshot && m_calibration && m_confirmation->isChecked());
    root->addWidget(buttons);
    connect(m_confirmation, &QCheckBox::toggled, this, [this, validSnapshot](bool checked) {
        m_generate->setEnabled(validSnapshot && m_calibration && checked);
    });
    connect(m_generate, &QPushButton::clicked, this,
            &DialogConfigurationDerivedRtcp::generateAndActivate);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
}

void DialogConfigurationDerivedRtcp::setTcpToMcsOrigin()
{
    for (QDoubleSpinBox* value : m_tcp)
        value->setValue(0.0);
}

void DialogConfigurationDerivedRtcp::generateAndActivate()
{
    if (!m_machine || !m_calibration || !m_confirmation->isChecked())
        return;
    const std::array<double, 3> toolPoint{
        m_tcp[0]->value(), m_tcp[1]->value(), m_tcp[2]->value()};
    QString id;
    QString error;
    if (!m_calibration->createConfigurationDerivedCandidate(
            *m_machine, toolPoint, &id, &error)
        || !m_calibration->activate(id, m_machine, &error)) {
        m_status->setText(error);
        m_status->setStyleSheet(QStringLiteral("color:#b00020;"));
        return;
    }
    m_savedCalibrationId = id;
    accept();
}

} // namespace lcnc::cam::ui
