#include "modules/process/communication/protocols/serial_channel.h"

#include <QSerialPort>

namespace lcnc::process {

SerialChannel::SerialChannel(QObject* parent)
    : CommunicationChannelBase(parent)
    , m_port(new QSerialPort(this))
{
    connect(m_port, &QSerialPort::readyRead, this, [this] {
        const QByteArray payload = m_port->readAll();
        if (!payload.isEmpty())
            reportReceived(payload);
    });
    connect(m_port, &QSerialPort::errorOccurred, this, [this](QSerialPort::SerialPortError error) {
        if (error != QSerialPort::NoError)
            reportError(m_port->errorString());
    });
}

SerialChannel::~SerialChannel() = default;

bool SerialChannel::connectChannel(QString* errorMessage)
{
    const QString portName = m_endpoint.serialPortName.trimmed();
    if (portName.isEmpty()) {
        const QString message = tr("Serial endpoint requires a port name");
        if (errorMessage)
            *errorMessage = message;
        reportError(message);
        return false;
    }

    setState(CommunicationState::Connecting);
    if (m_port->isOpen())
        m_port->close();
    m_port->setPortName(portName);
    m_port->setBaudRate(m_endpoint.baudRate > 0 ? m_endpoint.baudRate : 115200);
    m_port->setDataBits(QSerialPort::Data8);
    m_port->setParity(QSerialPort::NoParity);
    m_port->setStopBits(QSerialPort::OneStop);
    m_port->setFlowControl(QSerialPort::NoFlowControl);

    if (!m_port->open(QIODevice::ReadWrite)) {
        const QString message = m_port->errorString();
        if (errorMessage)
            *errorMessage = message;
        reportError(message);
        return false;
    }

    setState(CommunicationState::Connected);
    return true;
}

void SerialChannel::disconnectChannel()
{
    if (m_port->isOpen())
        m_port->close();
    setState(CommunicationState::Disconnected);
}

bool SerialChannel::sendData(const QByteArray& payload, QString* errorMessage)
{
    if (state() != CommunicationState::Connected || !m_port->isOpen()) {
        const QString message = tr("Serial channel is not connected");
        if (errorMessage)
            *errorMessage = message;
        reportError(message);
        return false;
    }

    const qint64 written = m_port->write(payload);
    if (written < 0) {
        const QString message = m_port->errorString();
        if (errorMessage)
            *errorMessage = message;
        reportError(message);
        return false;
    }
    reportSent(payload);
    return true;
}

} // namespace lcnc::process