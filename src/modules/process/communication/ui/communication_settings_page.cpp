#include "modules/process/communication/ui/communication_settings_page.h"

#include "modules/process/communication/communication_manager.h"
#include "modules/process/settings/process_settings.h"

#include <QComboBox>
#include <QAbstractItemView>
#include <QFormLayout>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QTableView>
#include <QVBoxLayout>

namespace lcnc::process {

CommunicationSettingsPage::CommunicationSettingsPage(QWidget* parent)
    : QWidget(parent)
    , m_manager(new CommunicationManager(this))
{
    auto* layout = new QVBoxLayout(this);
    auto* form = new QFormLayout();

    m_deviceIdEdit = new QLineEdit(this);
    form->addRow(tr("Device"), m_deviceIdEdit);

    m_protocolCombo = new QComboBox(this);
    m_protocolCombo->addItems({ QStringLiteral("Mock"), QStringLiteral("TCP"), QStringLiteral("HTTP"), QStringLiteral("Serial") });
    form->addRow(tr("Protocol"), m_protocolCombo);

    m_hostEdit = new QLineEdit(this);
    form->addRow(tr("Host"), m_hostEdit);

    m_portSpin = new QSpinBox(this);
    m_portSpin->setRange(0, 65535);
    form->addRow(tr("Port"), m_portSpin);

    m_pathEdit = new QLineEdit(this);
    form->addRow(tr("HTTP Path"), m_pathEdit);

    m_serialPortEdit = new QLineEdit(this);
    form->addRow(tr("Serial Port"), m_serialPortEdit);

    m_baudRateSpin = new QSpinBox(this);
    m_baudRateSpin->setRange(1200, 4000000);
    form->addRow(tr("Baud"), m_baudRateSpin);

    m_timeoutSpin = new QSpinBox(this);
    m_timeoutSpin->setRange(100, 600000);
    m_timeoutSpin->setSuffix(QStringLiteral(" ms"));
    form->addRow(tr("Timeout"), m_timeoutSpin);

    m_stateLabel = new QLabel(communicationStateToString(CommunicationState::Disconnected), this);
    form->addRow(tr("State"), m_stateLabel);
    layout->addLayout(form);

    auto* commandRow = new QHBoxLayout();
    m_connectButton = new QPushButton(tr("Connect"), this);
    m_disconnectButton = new QPushButton(tr("Disconnect"), this);
    commandRow->addWidget(m_connectButton);
    commandRow->addWidget(m_disconnectButton);
    commandRow->addStretch(1);
    layout->addLayout(commandRow);

    m_payloadEdit = new QPlainTextEdit(this);
    m_payloadEdit->setMaximumHeight(76);
    m_payloadEdit->setPlaceholderText(QStringLiteral("payload"));
    layout->addWidget(m_payloadEdit);

    auto* sendRow = new QHBoxLayout();
    m_sendButton = new QPushButton(tr("Send"), this);
    auto* clearButton = new QPushButton(tr("Clear Log"), this);
    sendRow->addWidget(m_sendButton);
    sendRow->addWidget(clearButton);
    sendRow->addStretch(1);
    layout->addLayout(sendRow);

    m_logView = new QTableView(this);
    m_logView->setModel(m_manager->logModel());
    m_logView->horizontalHeader()->setStretchLastSection(true);
    m_logView->verticalHeader()->setVisible(false);
    m_logView->setSelectionBehavior(QAbstractItemView::SelectRows);
    layout->addWidget(m_logView, 1);

    connect(m_protocolCombo, &QComboBox::currentTextChanged, this, &CommunicationSettingsPage::updateProtocolFields);
    connect(m_connectButton, &QPushButton::clicked, this, &CommunicationSettingsPage::connectCurrentEndpoint);
    connect(m_disconnectButton, &QPushButton::clicked, this, &CommunicationSettingsPage::disconnectCurrentEndpoint);
    connect(m_sendButton, &QPushButton::clicked, this, &CommunicationSettingsPage::sendPayload);
    connect(clearButton, &QPushButton::clicked, m_manager->logModel(), &CommunicationLogModel::clear);
    connect(m_manager, &CommunicationManager::deviceStateChanged, this,
            [this](const QString&, CommunicationState state) { setStateText(state); });

    loadFromSettings(lcnc::ProcessSettings{});
}

void CommunicationSettingsPage::loadFromSettings(const lcnc::ProcessSettings& settings)
{
    m_deviceIdEdit->setText(settings.communicationDeviceId());
    m_protocolCombo->setCurrentText(settings.communicationProtocol());
    m_hostEdit->setText(settings.communicationHost());
    m_portSpin->setValue(settings.communicationPort());
    m_pathEdit->setText(settings.communicationPath());
    m_serialPortEdit->setText(settings.communicationSerialPort());
    m_baudRateSpin->setValue(settings.communicationBaudRate());
    m_timeoutSpin->setValue(settings.communicationTimeoutMs());
    updateProtocolFields();
}

void CommunicationSettingsPage::applyToSettings(lcnc::ProcessSettings& settings) const
{
    const CommunicationEndpoint endpoint = endpointFromControls();
    settings.setCommunicationDeviceId(endpoint.normalizedDeviceId());
    settings.setCommunicationProtocol(communicationProtocolToString(endpoint.protocol));
    settings.setCommunicationHost(endpoint.host);
    settings.setCommunicationPort(endpoint.port);
    settings.setCommunicationPath(endpoint.path);
    settings.setCommunicationSerialPort(endpoint.serialPortName);
    settings.setCommunicationBaudRate(endpoint.baudRate);
    settings.setCommunicationTimeoutMs(endpoint.timeoutMs);
}

void CommunicationSettingsPage::connectCurrentEndpoint()
{
    const CommunicationEndpoint endpoint = endpointFromControls();
    QString errorMessage;
    if (!m_manager->configureEndpoint(endpoint, &errorMessage) || !m_manager->connectDevice(endpoint.normalizedDeviceId(), &errorMessage))
        m_stateLabel->setText(errorMessage);
}

void CommunicationSettingsPage::disconnectCurrentEndpoint()
{
    const CommunicationEndpoint endpoint = endpointFromControls();
    m_manager->disconnectDevice(endpoint.normalizedDeviceId());
}

void CommunicationSettingsPage::sendPayload()
{
    const CommunicationEndpoint endpoint = endpointFromControls();
    QString errorMessage;
    if (!m_manager->sendData(endpoint.normalizedDeviceId(), m_payloadEdit->toPlainText().toUtf8(), &errorMessage))
        m_stateLabel->setText(errorMessage);
}

void CommunicationSettingsPage::updateProtocolFields()
{
    bool ok = false;
    const CommunicationProtocol protocol = communicationProtocolFromString(m_protocolCombo->currentText(), &ok);
    Q_UNUSED(ok)
    const bool tcpLike = protocol == CommunicationProtocol::Tcp || protocol == CommunicationProtocol::Http;
    const bool http = protocol == CommunicationProtocol::Http;
    const bool serial = protocol == CommunicationProtocol::Serial;
    m_hostEdit->setEnabled(tcpLike);
    m_portSpin->setEnabled(tcpLike);
    m_pathEdit->setEnabled(http);
    m_serialPortEdit->setEnabled(serial);
    m_baudRateSpin->setEnabled(serial);
}

CommunicationEndpoint CommunicationSettingsPage::endpointFromControls() const
{
    CommunicationEndpoint endpoint;
    endpoint.deviceId = m_deviceIdEdit->text().trimmed();
    endpoint.protocol = communicationProtocolFromString(m_protocolCombo->currentText());
    endpoint.host = m_hostEdit->text().trimmed();
    endpoint.port = m_portSpin->value();
    endpoint.path = m_pathEdit->text().trimmed();
    endpoint.serialPortName = m_serialPortEdit->text().trimmed();
    endpoint.baudRate = m_baudRateSpin->value();
    endpoint.timeoutMs = m_timeoutSpin->value();
    return endpoint;
}

void CommunicationSettingsPage::setStateText(CommunicationState state)
{
    m_stateLabel->setText(communicationStateToString(state));
}

} // namespace lcnc::process