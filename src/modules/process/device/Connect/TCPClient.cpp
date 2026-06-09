#include "TCPClient.h"
#include <QCoreApplication>
#include <QThread>
#include <QMutexLocker>
#include "boost/thread.hpp"
#include "LogModule.h"

TCPClient::TCPClient(QObject* parent)
	: QObject(parent)
	, m_bConnected(false)
	, m_qstrRemoteHost("127.0.0.1")
	, m_iRemotePort(8080)
	, m_iTimeOut(1000)
{
	m_pSocket = new QTcpSocket(this);
}

TCPClient::~TCPClient()
{
	Disconnect();
	m_pSocket->deleteLater();
}

ErrorCode TCPClient::SetTCPTable(const table& tableTCP, bool& bConnectChange)
{
	if (!IsConnected() || bConnectChange ||
		IsTCPChange(tableTCP.at("sRemoteHost").as_string(),
			tableTCP.at("iRemotePort").as_integer(),
			tableTCP.at("iTimeout").as_integer()))
	{
		Disconnect();
		SetTCPSetting(tableTCP.at("sRemoteHost").as_string(),
			tableTCP.at("iRemotePort").as_integer(),
			tableTCP.at("iTimeout").as_integer());
		if (!Connect())
			return ErrorCode::ERROR_NONE;
		bConnectChange = true;
	}
	return ErrorCode::ERROR_NONE;
}

void TCPClient::slotReadData()
{
	QMutexLocker locker(&m_mutex);
	if (m_pSocket->bytesAvailable() > 0) {
		QByteArray newData = m_pSocket->readAll();

		// 保存响应数据并通知等待线程
		{
			QMutexLocker responseLocker(&m_responseMutex);
			m_responseData.append(newData);
			m_responseCondition.wakeOne();
		}

		LOG_SYS_INFO("TCP: received data, size: " + std::to_string(newData.size()) + " bytes");
	}
}

void TCPClient::slotDisconnected()
{
	QMutexLocker locker(&m_mutex);
	m_bConnected = false;
}

bool TCPClient::Connect()
{
	if (!m_bConnected) {
		// 先断开之前的连接
		Disconnect();
		
		m_pSocket->connectToHost(m_qstrRemoteHost, m_iRemotePort);
		if (m_pSocket->waitForConnected(m_iTimeOut)) {
			m_bConnected = true;
			// 使用 QueuedConnection 确保跨线程时正确处理
			connect(m_pSocket, &QTcpSocket::readyRead, this, &TCPClient::slotReadData, Qt::QueuedConnection);
			connect(m_pSocket, &QTcpSocket::disconnected, this, &TCPClient::slotDisconnected, Qt::QueuedConnection);
			LOG_SYS_INFO("TCP connected to " + m_qstrRemoteHost.toStdString() + ":" + std::to_string(m_iRemotePort));
		}
		else {
			m_bConnected = false;
			LOG_SYS_ERROR("TCP connect failed to " + m_qstrRemoteHost.toStdString() + ":" + std::to_string(m_iRemotePort) + ", error: " + m_pSocket->errorString().toStdString());
		}
	}
	return m_bConnected;
}

void TCPClient::Disconnect()
{
	// 断开信号槽连接
	disconnect(m_pSocket, &QTcpSocket::readyRead, this, &TCPClient::slotReadData);
	disconnect(m_pSocket, &QTcpSocket::disconnected, this, &TCPClient::slotDisconnected);
	
	if (m_pSocket->isOpen()) {
		m_pSocket->close();
	}
	m_bConnected = false;
	
	// 清空响应数据
	QMutexLocker responseLocker(&m_responseMutex);
	m_responseData.clear();
	
	LOG_SYS_INFO("TCP disconnected");
}

bool TCPClient::IsConnected()
{
	return m_bConnected && m_pSocket->state() == QAbstractSocket::ConnectedState;
}

bool TCPClient::IsAvailableData(const QByteArray& data)
{
	return data.size();
}

bool TCPClient::SendCommand(const QByteArray& command, QByteArray& response)
{
	QString strCommand = QString::fromUtf8(command);
	QString strResponse;
	bool success = false;
	
	LOG_SYS_INFO("TCP: SendCommand called, data size: " + std::to_string(command.size()) + " bytes");
	if (!IsConnected()) {
		LOG_SYS_ERROR("TCP: not connected or socket not open");
		emit signalCommunication(strCommand, "Failed: Not connected", false);
		return false;
	}

	// 发送命令并等待响应
	if (QThread::currentThread() != m_pSocket->thread()) {
		// 跨线程调用，使用阻塞式连接获取响应
		struct SendCommandResult {
			bool success;
			QByteArray response;
			QByteArray command;
			int timeout;
		} result = { false, QByteArray(), command, m_iTimeOut * 2 };

		QMetaObject::invokeMethod(
			this,
			[this, &result]() {
				QMutexLocker locker(&m_mutex);
				if (m_pSocket->isOpen() && m_pSocket->isWritable()) {
					// 发送命令
					qint64 bytesWritten = m_pSocket->write(result.command);
					if (bytesWritten == result.command.size()) {
						// 等待数据发送完成
						if (QCoreApplication::instance()) {
							if (!m_pSocket->waitForBytesWritten(result.timeout / 2)) {
								LOG_SYS_ERROR("TCP: waitForBytesWritten timeout");
								result.success = false;
								return;
							}
						}
						else {
							int sendTime = (result.command.size() * 1000) / 1024;
							sendTime = qMax(10, qMin(sendTime + 100, result.timeout / 2));
							boost::this_thread::sleep_for(boost::chrono::milliseconds(sendTime));
						}

						// 直接等待响应数据
						if (QCoreApplication::instance()) {
							if (m_pSocket->waitForReadyRead(result.timeout / 2)) {
								result.response = m_pSocket->readAll();
								result.success = true;
							}
							else {
								LOG_SYS_ERROR("TCP: waitForReadyRead timeout");
								result.success = false;
							}
						}
						else {
							// 在非Qt事件循环环境中，使用轮询方式读取数据
							int remainingTime = result.timeout / 2;
							const int pollInterval = 50;
							while (remainingTime > 0) {
								if (m_pSocket->bytesAvailable() > 0) {
									result.response = m_pSocket->readAll();
									result.success = true;
									break;
								}
								boost::this_thread::sleep_for(boost::chrono::milliseconds(pollInterval));
								remainingTime -= pollInterval;
							}
							if (remainingTime <= 0) {
								LOG_SYS_ERROR("TCP: read timeout in non-Qt environment");
								result.success = false;
							}
						}
					}
					else {
						LOG_SYS_ERROR("TCP: write failed");
						result.success = false;
					}
				}
				else {
					LOG_SYS_ERROR("TCP: socket not writable");
					result.success = false;
				}
			},
			Qt::BlockingQueuedConnection
		);

		if (!result.success) {
			LOG_SYS_ERROR("TCP: SendCommand cross-thread call failed");
			emit signalCommunication(strCommand, "Failed: Cross-thread call failed", false);
			return false;
		}

		response = result.response;
		strResponse = QString::fromUtf8(response);
		success = true;
	}
	else {
		// 同线程调用，直接使用阻塞式读取
		QMutexLocker locker(&m_mutex);
		if (!m_pSocket->isOpen() || !m_pSocket->isWritable()) {
			LOG_SYS_ERROR("TCP: socket not writable");
			return false;
		}

		// 发送命令
		qint64 bytesWritten = m_pSocket->write(command);
		if (bytesWritten != command.size()) {
			LOG_SYS_ERROR("TCP: write failed");
			emit signalCommunication(strCommand, "Failed: Write failed", false);
			return false;
		}

		// 等待数据发送完成
		if (QCoreApplication::instance()) {
			if (!m_pSocket->waitForBytesWritten(m_iTimeOut)) {
				LOG_SYS_ERROR("TCP: waitForBytesWritten timeout");
				emit signalCommunication(strCommand, "Failed: waitForBytesWritten timeout", false);
				return false;
			}
		}
		else {
			int sendTime = (command.size() * 1000) / 1024;
			sendTime = qMax(10, qMin(sendTime + 100, m_iTimeOut));
			boost::this_thread::sleep_for(boost::chrono::milliseconds(sendTime));
		}

		// 等待响应数据
		if (QCoreApplication::instance()) {
			if (m_pSocket->waitForReadyRead(m_iTimeOut)) {
				response = m_pSocket->readAll();
			}
			else {
				LOG_SYS_ERROR("TCP: waitForReadyRead timeout");
				emit signalCommunication(strCommand, "Failed: waitForReadyRead timeout", false);
				return false;
			}
		}
		else {
			// 在非Qt事件循环环境中，使用轮询方式读取数据
			int remainingTime = m_iTimeOut;
			const int pollInterval = 50;
			while (remainingTime > 0) {
				if (m_pSocket->bytesAvailable() > 0) {
					response = m_pSocket->readAll();
					break;
				}
				boost::this_thread::sleep_for(boost::chrono::milliseconds(pollInterval));
				remainingTime -= pollInterval;
			}
			if (remainingTime <= 0) {
				LOG_SYS_ERROR("TCP: read timeout in non-Qt environment");
				emit signalCommunication(strCommand, "Failed: Read timeout in non-Qt environment", false);
				return false;
			}
		}
	}

	if (response.isEmpty()) {
		LOG_SYS_ERROR("TCP: empty response received");
		emit signalCommunication(strCommand, "Failed: Empty response received", false);
		return false;
	}

	strResponse = QString::fromUtf8(response);
	success = true;
	emit signalCommunication(strCommand, strResponse, true);
	LOG_SYS_INFO("TCP: SendCommand completed successfully, response size: " + std::to_string(response.size()) + " bytes");
	return true;
}

void TCPClient::ClearBuffer()
{
	QMutexLocker locker(&m_mutex);
	m_qbDataBuffer.clear();
}

QByteArray TCPClient::ReadData()
{
	QMutexLocker locker(&m_mutex);
	QByteArray data = m_qbDataBuffer;
	m_qbDataBuffer.clear();
	return data;
}

QByteArray TCPClient::ReadRawData()
{
	LOG_SYS_INFO("TCP: ReadRawData called");
	if (!m_bConnected || !m_pSocket->isOpen()) {
		LOG_SYS_ERROR("TCP: not connected or socket not open");
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
							QByteArray newData = m_pSocket->readAll();
							if (!newData.isEmpty()) {
								result.data = newData;
								break;
							}
						}
						remainingTime -= pollInterval;
					}
				}
			},
			Qt::BlockingQueuedConnection
		);

		if (!result.data.isEmpty()) {
			LOG_SYS_INFO("TCP: ReadRawData cross-thread call successful, data size: " + std::to_string(result.data.size()) + " bytes");
		}
		else {
			LOG_SYS_WARN("TCP: ReadRawData cross-thread call returned empty data");
		}
		return result.data;
	}

	QMutexLocker locker(&m_mutex);
	if (!m_pSocket->isOpen()) {
		LOG_SYS_ERROR("TCP socket not open");
		return QByteArray();
	}

	QByteArray data;
	int remainingTime = m_iTimeOut;
	const int pollInterval = 50;

	while (remainingTime > 0) {
		if (m_pSocket->waitForReadyRead(pollInterval)) {
			QByteArray newData = m_pSocket->readAll();
			if (!newData.isEmpty()) {
				data = newData;
				break;
			}
		}
		remainingTime -= pollInterval;
	}

	if (!data.isEmpty()) {
		LOG_SYS_INFO("TCP: ReadRawData successful, data size: " + std::to_string(data.size()) + " bytes");
	}
	else {
		LOG_SYS_WARN("TCP: ReadRawData returned empty data");
	}
	return data;
}

bool TCPClient::WriteData(const QByteArray& data)
{
	LOG_SYS_INFO("TCP: WriteData called, data size: " + std::to_string(data.size()) + " bytes");
	if (!m_bConnected || !m_pSocket->isOpen()) {
		LOG_SYS_ERROR("TCP: not connected or socket not open");
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
				if (m_pSocket->isOpen() && m_pSocket->isWritable()) {
					qint64 bytesWritten = m_pSocket->write(result.data);
					if (bytesWritten == result.data.size()) {
						if (QCoreApplication::instance()) {
							result.success = m_pSocket->waitForBytesWritten(result.timeout);
						}
						else {
							int sendTime = (result.data.size() * 1000) / 1024;
							sendTime = qMax(10, qMin(sendTime + 100, result.timeout));
							boost::this_thread::sleep_for(boost::chrono::milliseconds(sendTime));
							result.success = true;
						}
					}
				}
			},
			Qt::BlockingQueuedConnection
		);

		if (result.success) {
			LOG_SYS_INFO("TCP: WriteData cross-thread call successful");
		}
		else {
			LOG_SYS_WARN("TCP: WriteData cross-thread call failed");
		}
		return result.success;
	}

	QMutexLocker locker(&m_mutex);
	if (m_pSocket->isOpen() && m_pSocket->isWritable()) {
		qint64 bytesWritten = m_pSocket->write(data);
		if (bytesWritten != data.size()) {
			LOG_SYS_ERROR("TCP write failed");
			return false;
		}

		if (QCoreApplication::instance()) {
			if (!m_pSocket->waitForBytesWritten(m_iTimeOut)) {
				LOG_SYS_ERROR("TCP waitForBytesWritten timeout");
				return false;
			}
		}
		else {
			int sendTime = (data.size() * 1000) / 1024;
			sendTime = qMax(10, qMin(sendTime + 100, m_iTimeOut));
			boost::this_thread::sleep_for(boost::chrono::milliseconds(sendTime));
		}

		LOG_SYS_INFO("TCP: WriteData completed successfully");
		return true;
	}

	LOG_SYS_ERROR("TCP socket not writable");
	return false;
}

bool TCPClient::OnceData(const QByteArray& send)
{
	QByteArray recvData;
	return OnceData(send, recvData);
}

bool TCPClient::OnceData(const QByteArray& send, QByteArray& recv)
{
	if (!IsConnected()) {
		LOG_SYS_ERROR("TCP not connected");
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

bool TCPClient::DoOnceData(const QByteArray& send, QByteArray& recv, bool& success)
{
	QMutexLocker locker(&m_mutex);
	if (!IsConnected()) {
		LOG_SYS_ERROR("ONCE TCP not connected");
		success = false;
		return false;
	}

	disconnect(m_pSocket, &QTcpSocket::readyRead, this, &TCPClient::slotReadData);
	m_qbDataBuffer.clear();

	if (QCoreApplication::instance()) {
		QThread::msleep(5);
	}
	else {
		boost::this_thread::sleep_for(boost::chrono::milliseconds(5));
	}

	if (!m_pSocket->isOpen() || !m_pSocket->isWritable()) {
		QObject::connect(m_pSocket, &QTcpSocket::readyRead, this, &TCPClient::slotReadData, Qt::UniqueConnection);
		LOG_SYS_ERROR("ONCE TCP socket not writable");
		success = false;
		return true;
	}

	qint64 bytesWritten = m_pSocket->write(send);
	if (bytesWritten != send.size()) {
		QObject::connect(m_pSocket, &QTcpSocket::readyRead, this, &TCPClient::slotReadData, Qt::UniqueConnection);
		if (QCoreApplication::instance()) {
			QThread::msleep(20);
		}
		else {
			boost::this_thread::sleep_for(boost::chrono::milliseconds(20));
		}
		LOG_SYS_ERROR("ONCE TCP write failed");
		success = false;
		return true;
	}

	if (QCoreApplication::instance()) {
		if (!m_pSocket->waitForBytesWritten(m_iTimeOut)) {
			QObject::connect(m_pSocket, &QTcpSocket::readyRead, this, &TCPClient::slotReadData, Qt::UniqueConnection);
			QThread::msleep(20);
			LOG_SYS_ERROR("ONCE TCP waitForBytesWritten timeout");
			success = false;
			return true;
		}
		QThread::msleep(5);
	}
	else {
		int sendTime = (send.size() * 1000) / 1024;
		sendTime = qMax(10, qMin(sendTime + 100, m_iTimeOut));
		boost::this_thread::sleep_for(boost::chrono::milliseconds(sendTime));
		boost::this_thread::sleep_for(boost::chrono::milliseconds(5));
	}

	QByteArray rawData;
	int remainingTime = m_iTimeOut;
	const int pollInterval = 50;

	while (remainingTime > 0) {
		if (m_pSocket->waitForReadyRead(pollInterval)) {
			rawData = m_pSocket->readAll();
			if (!rawData.isEmpty()) {
				break;
			}
		}
		remainingTime -= pollInterval;
	}

	if (rawData.isEmpty()) {
		QObject::connect(m_pSocket, &QTcpSocket::readyRead, this, &TCPClient::slotReadData, Qt::UniqueConnection);
		LOG_SYS_ERROR("ONCE TCP read failed");
		success = false;
		return true;
	}

	if (IsAvailableData(rawData)) {
		QObject::connect(m_pSocket, &QTcpSocket::readyRead, this, &TCPClient::slotReadData, Qt::UniqueConnection);
		recv = rawData;
		success = true;
		return true;
	}

	QObject::connect(m_pSocket, &QTcpSocket::readyRead, this, &TCPClient::slotReadData, Qt::UniqueConnection);
	LOG_SYS_ERROR("ONCE TCP data validation failed");
	success = false;
	return true;
}

bool TCPClient::IsTCPChange(string strRemoteHost, int iRemotePort, int iTimeOut)
{
	if (m_qstrRemoteHost != QString::fromStdString(strRemoteHost))
		return true;

	if (m_iRemotePort != iRemotePort)
		return true;

	if (m_iTimeOut != iTimeOut)
		return true;

	return false;
}

void TCPClient::SetTCPSetting(string strRemoteHost, int iRemotePort, int iTimeOut)
{
	m_qstrRemoteHost = QString::fromStdString(strRemoteHost);
	m_iRemotePort = iRemotePort;
	m_iTimeOut = iTimeOut;
}
