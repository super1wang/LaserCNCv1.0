#include "UDPClient.h"
#include <QCoreApplication>
#include <QThread>
#include <QMutexLocker>
#include <QNetworkInterface>
#include "boost/thread.hpp"
#include "LogModule.h"

UDPClient::UDPClient(QObject* parent)
	: QObject(parent)
	, m_bConnected(false)
	, m_qstrRemoteHost("127.0.0.1")
	, m_iRemotePort(8080)
	, m_iLocalPort(8081)
	, m_iTimeOut(1000)
	, m_bMulticast(false)
{
	m_pSocket = new QUdpSocket(this);
}

UDPClient::~UDPClient()
{
	Disconnect();
	m_pSocket->deleteLater();
}

ErrorCode UDPClient::SetUDPTable(const table& tableUDP, bool& bConnectChange)
{
	if (!IsConnected() ||
		IsUDPChange(tableUDP.at("sRemoteHost").as_string(),
			tableUDP.at("iRemotePort").as_integer(),
			tableUDP.at("iLocalPort").as_integer(),
			tableUDP.at("iTimeout").as_integer(),
			tableUDP.at("bMulticast").as_boolean()))
	{
		Disconnect();
		SetUDPSetting(tableUDP.at("sRemoteHost").as_string(),
			tableUDP.at("iRemotePort").as_integer(),
			tableUDP.at("iLocalPort").as_integer(),
			tableUDP.at("iTimeout").as_integer(),
			tableUDP.at("bMulticast").as_boolean());
		if (!Connect())
			return ErrorCode::ERROR_NONE;
		bConnectChange = true;
	}
	return ErrorCode::ERROR_NONE;
}

void UDPClient::slotReadData()
{
	QMutexLocker locker(&m_mutex);
	while (m_pSocket->hasPendingDatagrams()) {
		QByteArray datagram;
		datagram.resize(m_pSocket->pendingDatagramSize());
		QHostAddress sender;
		quint16 senderPort;
		m_pSocket->readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);
		m_qbDataBuffer.append(datagram);
	}
}

bool UDPClient::Connect()
{
	if (!m_bConnected) {
		bool bindResult = false;

		if (m_bMulticast) {
			bindResult = m_pSocket->bind(QHostAddress::AnyIPv4, m_iLocalPort);
			if (bindResult) {
				m_pSocket->joinMulticastGroup(QHostAddress(m_qstrRemoteHost));
			}
		}
		else {
			bindResult = m_pSocket->bind(QHostAddress::Any, m_iLocalPort);
		}

		if (bindResult) {
			m_bConnected = true;
			connect(m_pSocket, &QUdpSocket::readyRead, this, &UDPClient::slotReadData);
		}
		else {
			m_bConnected = false;
			LOG_SYS_ERROR("UDP bind failed on port: " + std::to_string(m_iLocalPort));
		}
	}
	return m_bConnected;
}

void UDPClient::Disconnect()
{
	if (m_pSocket->isOpen()) {
		if (m_bMulticast) {
			m_pSocket->leaveMulticastGroup(QHostAddress(m_qstrRemoteHost));
		}
		m_pSocket->close();
	}
	m_bConnected = false;
}

bool UDPClient::IsConnected()
{
	return m_bConnected;
}

bool UDPClient::IsAvailableData(const QByteArray& data)
{
	return data.size();
}

void UDPClient::ClearBuffer()
{
	QMutexLocker locker(&m_mutex);
	m_qbDataBuffer.clear();
}

QByteArray UDPClient::ReadData()
{
	QMutexLocker locker(&m_mutex);
	QByteArray data = m_qbDataBuffer;
	m_qbDataBuffer.clear();
	return data;
}

QByteArray UDPClient::ReadRawData()
{
	LOG_SYS_INFO("UDP: ReadRawData called");
	if (!m_bConnected || !m_pSocket->isOpen()) {
		LOG_SYS_ERROR("UDP: not connected or socket not open");
		return QByteArray();
	}

	if (QThread::currentThread() != m_pSocket->thread()) {
		struct ReadRawDataResult {
			QByteArray data;
			int timeout;
		} result = { QByteArray(), m_iTimeOut };

		QMetaObject::invokeMethod(
			this,
			[this, &result]() {
				QMutexLocker locker(&m_mutex);
				if (m_pSocket->isOpen()) {
					int remainingTime = result.timeout;
					const int pollInterval = 50;

					while (remainingTime > 0) {
						if (m_pSocket->waitForReadyRead(pollInterval)) {
							while (m_pSocket->hasPendingDatagrams()) {
								QByteArray datagram;
								datagram.resize(m_pSocket->pendingDatagramSize());
								QHostAddress sender;
								quint16 senderPort;
								m_pSocket->readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);
								result.data.append(datagram);
							}
							if (!result.data.isEmpty()) {
								break;
							}
						}
						remainingTime -= pollInterval;
					}
				}
			},
			Qt::BlockingQueuedConnection
		);

		return result.data;
	}

	QMutexLocker locker(&m_mutex);
	if (!m_pSocket->isOpen()) {
		LOG_SYS_ERROR("UDP socket not open");
		return QByteArray();
	}

	QByteArray data;
	int remainingTime = m_iTimeOut;
	const int pollInterval = 50;

	while (remainingTime > 0) {
		if (m_pSocket->waitForReadyRead(pollInterval)) {
			while (m_pSocket->hasPendingDatagrams()) {
				QByteArray datagram;
				datagram.resize(m_pSocket->pendingDatagramSize());
				QHostAddress sender;
				quint16 senderPort;
				m_pSocket->readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);
				data.append(datagram);
			}
			if (!data.isEmpty()) {
				break;
			}
		}
		remainingTime -= pollInterval;
	}

	return data;
}

bool UDPClient::WriteData(const QByteArray& data)
{
	LOG_SYS_INFO("UDP: WriteData called, data size: " + std::to_string(data.size()) + " bytes");
	if (!m_bConnected || !m_pSocket->isOpen()) {
		LOG_SYS_ERROR("UDP: not connected or socket not open");
		return false;
	}

	if (QThread::currentThread() != m_pSocket->thread()) {
		struct WriteDataResult {
			bool success;
			QByteArray data;
			int timeout;
		} result = { false, data, m_iTimeOut };

		QMetaObject::invokeMethod(
			this,
			[this, &result]() {
				QMutexLocker locker(&m_mutex);
				QHostAddress targetHost(m_qstrRemoteHost);
				qint64 bytesWritten = m_pSocket->writeDatagram(result.data, targetHost, m_iRemotePort);
				if (bytesWritten == result.data.size()) {
					if (QCoreApplication::instance()) {
						result.success = m_pSocket->waitForBytesWritten(result.timeout);
					}
					else {
						int sendTime = qMax(10, qMin(100, result.timeout));
						boost::this_thread::sleep_for(boost::chrono::milliseconds(sendTime));
						result.success = true;
					}
				}
			},
			Qt::BlockingQueuedConnection
		);

		return result.success;
	}

	QMutexLocker locker(&m_mutex);
	QHostAddress targetHost(m_qstrRemoteHost);
	qint64 bytesWritten = m_pSocket->writeDatagram(data, targetHost, m_iRemotePort);
	if (bytesWritten != data.size()) {
		LOG_SYS_ERROR("UDP writeDatagram failed");
		return false;
	}

	if (QCoreApplication::instance()) {
		if (!m_pSocket->waitForBytesWritten(m_iTimeOut)) {
			LOG_SYS_ERROR("UDP waitForBytesWritten timeout");
			return false;
		}
	}
	else {
		int sendTime = qMax(10, qMin(100, m_iTimeOut));
		boost::this_thread::sleep_for(boost::chrono::milliseconds(sendTime));
	}

	return true;
}

bool UDPClient::OnceData(const QByteArray& send)
{
	QByteArray recvData;
	return OnceData(send, recvData);
}

bool UDPClient::OnceData(const QByteArray& send, QByteArray& recv)
{
	if (!IsConnected()) {
		LOG_SYS_ERROR("UDP not connected");
		return false;
	}

	bool isSameThread = (QThread::currentThread() == m_pSocket->thread());

	if (!isSameThread) {
		struct OnceDataResult {
			bool success;
			QByteArray sendData;
			QByteArray recvData;
		} result = { false, send, QByteArray() };

		QMetaObject::invokeMethod(this, [this, &result]() {
			if (!DoOnceData(result.sendData, result.recvData, result.success)) {
				result.success = false;
			}
		}, Qt::BlockingQueuedConnection);

		recv = result.recvData;
		return result.success;
	}

	QByteArray rawData;
	bool success = false;
	return DoOnceData(send, recv, success) ? success : false;
}

bool UDPClient::DoOnceData(const QByteArray& send, QByteArray& recv, bool& success)
{
	QMutexLocker locker(&m_mutex);
	if (!IsConnected()) {
		LOG_SYS_ERROR("ONCE UDP not connected");
		success = false;
		return false;
	}

	disconnect(m_pSocket, &QUdpSocket::readyRead, this, &UDPClient::slotReadData);
	m_qbDataBuffer.clear();

	if (QCoreApplication::instance()) {
		QThread::msleep(5);
	}
	else {
		boost::this_thread::sleep_for(boost::chrono::milliseconds(5));
	}

	QHostAddress targetHost(m_qstrRemoteHost);
	qint64 bytesWritten = m_pSocket->writeDatagram(send, targetHost, m_iRemotePort);
	if (bytesWritten != send.size()) {
		QObject::connect(m_pSocket, &QUdpSocket::readyRead, this, &UDPClient::slotReadData, Qt::UniqueConnection);
		if (QCoreApplication::instance()) {
			QThread::msleep(20);
		}
		else {
			boost::this_thread::sleep_for(boost::chrono::milliseconds(20));
		}
		LOG_SYS_ERROR("ONCE UDP write failed");
		success = false;
		return true;
	}

	if (QCoreApplication::instance()) {
		QThread::msleep(5);
	}
	else {
		boost::this_thread::sleep_for(boost::chrono::milliseconds(5));
	}

	QByteArray rawData;
	int remainingTime = m_iTimeOut;
	const int pollInterval = 50;

	while (remainingTime > 0) {
		if (m_pSocket->waitForReadyRead(pollInterval)) {
			while (m_pSocket->hasPendingDatagrams()) {
				QByteArray datagram;
				datagram.resize(m_pSocket->pendingDatagramSize());
				QHostAddress sender;
				quint16 senderPort;
				m_pSocket->readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);
				rawData.append(datagram);
			}
			if (!rawData.isEmpty()) {
				break;
			}
		}
		remainingTime -= pollInterval;
	}

	if (rawData.isEmpty()) {
		QObject::connect(m_pSocket, &QUdpSocket::readyRead, this, &UDPClient::slotReadData, Qt::UniqueConnection);
		LOG_SYS_ERROR("ONCE UDP read failed");
		success = false;
		return true;
	}

	if (IsAvailableData(rawData)) {
		QObject::connect(m_pSocket, &QUdpSocket::readyRead, this, &UDPClient::slotReadData, Qt::UniqueConnection);
		recv = rawData;
		success = true;
		return true;
	}

	QObject::connect(m_pSocket, &QUdpSocket::readyRead, this, &UDPClient::slotReadData, Qt::UniqueConnection);
	LOG_SYS_ERROR("ONCE UDP data validation failed");
	success = false;
	return true;
}

bool UDPClient::IsUDPChange(string strRemoteHost, int iRemotePort, int iLocalPort, int iTimeOut, bool bMulticast)
{
	if (m_qstrRemoteHost != QString::fromStdString(strRemoteHost))
		return true;

	if (m_iRemotePort != iRemotePort)
		return true;

	if (m_iLocalPort != iLocalPort)
		return true;

	if (m_iTimeOut != iTimeOut)
		return true;

	if (m_bMulticast != bMulticast)
		return true;

	return false;
}

void UDPClient::SetUDPSetting(string strRemoteHost, int iRemotePort, int iLocalPort, int iTimeOut, bool bMulticast)
{
	m_qstrRemoteHost = QString::fromStdString(strRemoteHost);
	m_iRemotePort	 = iRemotePort;
	m_iLocalPort	 = iLocalPort;
	m_iTimeOut		 = iTimeOut;
	m_bMulticast	 = bMulticast;
}
