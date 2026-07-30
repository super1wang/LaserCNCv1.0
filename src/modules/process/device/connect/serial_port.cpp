#include "serial_port.h"
#include <QSerialPortInfo>
#include <QMutexLocker>
#include <QCoreApplication>
#include <QThread>
#include "boost/thread.hpp"
#include "core/logging/logger.h"


SerialPort::SerialPort(QObject* parent)
	: QObject(parent)
	, m_bConnected(false)
	, m_dwBaudRate(9600)
	, m_iDataBits(8)
	, m_iParity(0)
	, m_iStopBits(1)
{
	port = new QSerialPort();
	port->moveToThread(&m_ioThread);
	m_ioThread.setObjectName(QStringLiteral("ProcessSerialIo"));
	m_ioThread.start();
}

SerialPort::~SerialPort()
{
	Disconnect();
	if (port) {
		if (QThread::currentThread() == port->thread()) {
			delete port;
			port = nullptr;
		} else {
			QMetaObject::invokeMethod(port, [this] {
				delete port;
				port = nullptr;
			}, Qt::BlockingQueuedConnection);
		}
	}
	m_ioThread.quit();
	m_ioThread.wait();
}

ErrorCode SerialPort::SetComTable(const table& tableCom, bool& bConnectChange)
{
	if (!IsConnected() ||
		IsComChange(tableCom.at("sPort").as_string(),
			tableCom.at("sBaudRate").as_string(),
			tableCom.at("sDataBits").as_string(),
			tableCom.at("sParity").as_string(),
			tableCom.at("sStopBits").as_string()))
	{
		Disconnect();
		SetComSetting(tableCom.at("sPort").as_string(),
			tableCom.at("sBaudRate").as_string(),
			tableCom.at("sDataBits").as_string(),
			tableCom.at("sParity").as_string(),
			tableCom.at("sStopBits").as_string());
		if (!Connect())
			return ErrorCode::ERROR_SERIALPORT_CONNECTIONFAILED;
		bConnectChange = true;
	}
	return ErrorCode::ERROR_NONE;
}

bool SerialPort::Connect()
{
	if (m_bConnected.load())
		return true;

	bool connected = false;
	auto connectPort = [this, &connected] {
		port->setPortName(m_qstrPort);
		port->setBaudRate(m_dwBaudRate);
		port->setDataBits(static_cast<QSerialPort::DataBits>(m_iDataBits));
		port->setStopBits(static_cast<QSerialPort::StopBits>(m_iStopBits));
		port->setParity(static_cast<QSerialPort::Parity>(m_iParity));
		port->setFlowControl(QSerialPort::NoFlowControl);
		connected = port->open(QIODevice::ReadWrite);
		m_bConnected.store(connected);
		if (connected) {
			QObject::connect(port, &QSerialPort::readyRead, port,
			                 [this] { slotReadData(); });
		}
	};
	if (QThread::currentThread() == port->thread())
		connectPort();
	else
		QMetaObject::invokeMethod(port, connectPort, Qt::BlockingQueuedConnection);
	return connected;
}

void SerialPort::Disconnect()
{
	if (!port) {
		m_bConnected.store(false);
		return;
	}
	auto disconnectPort = [this] {
		QObject::disconnect(port, nullptr, port, nullptr);
		if (port->isOpen())
			port->close();
		m_bConnected.store(false);
	};
	if (QThread::currentThread() == port->thread())
		disconnectPort();
	else
		QMetaObject::invokeMethod(port, disconnectPort, Qt::BlockingQueuedConnection);
}

bool SerialPort::IsConnected()
{
	return m_bConnected.load();
}

bool SerialPort::IsAvailableData(const QByteArray&)
{
	return true;
}

void SerialPort::ClearBuffer()
{
	auto clearPort = [this] {
		QMutexLocker locker(&m_mutex);
		m_qbDataBuffer.clear();
		port->clear();
	};
	if (QThread::currentThread() == port->thread())
		clearPort();
	else
		QMetaObject::invokeMethod(port, clearPort, Qt::BlockingQueuedConnection);
}

void SerialPort::slotReadData()
{
	QMutexLocker locker(&m_mutex);
	if (port->bytesAvailable() > 0)
	{
		QByteArray newData = port->readAll();
		m_qbDataBuffer.append(newData);
	}
}

QByteArray SerialPort::ReadData()
{
	QMutexLocker locker(&m_mutex);
	QByteArray data = m_qbDataBuffer;
	m_qbDataBuffer.clear();
	return data;
}

bool SerialPort::WriteData(const QByteArray& data, int iTimeOut)
{
    // 检查是否在QSerialPort所在的线程中
    if (QThread::currentThread() != port->thread()) {
        // 不在同一线程，需要使用线程安全的方式调用
        struct WriteDataResult {
            bool success;
            QByteArray data;
            int timeout;
        } result = {false, data, iTimeOut};

        // 使用Qt的invokeMethod在正确的线程中执行实际的写操作
        QMetaObject::invokeMethod(
            port,
            [this, &result]() {
                QMutexLocker locker(&m_mutex);
                if (port->isOpen() && port->isWritable()) {
                    qint64 bytesWritten = port->write(result.data);
                    if (bytesWritten == result.data.size()) {
                        // 使用waitForBytesWritten确保数据发送完成
                        if (QCoreApplication::instance()) {
                            result.success = port->waitForBytesWritten(result.timeout);
                        } else {
                            // 如果没有事件循环，使用基于波特率的动态延迟
                            int bytesPerSecond = m_dwBaudRate / 10;
                            int sendTime = (result.data.size() * 1000) / bytesPerSecond;
                            sendTime = qMax(10, qMin(sendTime + 100, result.timeout));
                            boost::this_thread::sleep_for(boost::chrono::milliseconds(sendTime));
                            result.success = true;
                        }
                    }
                }
            },
            Qt::BlockingQueuedConnection
        );

        return result.success;
    }

    // 已经在正确的线程中，直接执行
    QMutexLocker locker(&m_mutex);
    if (port->isOpen() && port->isWritable()) {
        qint64 bytesWritten = port->write(data);
        if (bytesWritten != data.size()) {
            LCNC_ERR(lcnc::LogCode::Generic, "{}", "bytesWritten != data.size()");
            return false;
        }

        if (QCoreApplication::instance()) {
            if (!port->waitForBytesWritten(iTimeOut)) {
                LCNC_ERR(lcnc::LogCode::Generic, "{}", "waitForBytesWritten timeout");
                return false;
            }
        } else {
            int bytesPerSecond = m_dwBaudRate / 10;
            int sendTime = (data.size() * 1000) / bytesPerSecond;
            sendTime = qMax(10, qMin(sendTime + 100, iTimeOut));
            boost::this_thread::sleep_for(boost::chrono::milliseconds(sendTime));
        }

        return true;
    }

    LCNC_ERR(lcnc::LogCode::Generic, "{}", "port->isWritable()");
    return false;
}

//bool SerialPort::OnceData(const string& send, int iTimeOut)
//{
//    string recvData;
//	if (!OnceData(send, recvData, iTimeOut))
//		return false;
//
//	return true;
//}
//
//bool SerialPort::OnceData(const string& send, string& recv, int iTimeOut)
//{
//	QByteArray sendData = QByteArray::fromStdString(send);
//	QByteArray recvData;
//	if (!OnceData(sendData, recvData, iTimeOut))
//		return false;
//
//	recv = recvData.toStdString();
//	return true;
//}

bool SerialPort::OnceData(const QByteArray& send, int iTimeOut)
{
    QByteArray recvData;
    return OnceData(send, recvData, iTimeOut);
}

bool SerialPort::OnceData(const QByteArray& send, QByteArray& recv, int iTimeOut)
{
    // 检查是否在QSerialPort所在的线程中
    if (QThread::currentThread() != port->thread()) {
        // 不在同一线程，使用线程安全的方式调用
        struct OnceDataResult {
            bool success;
            QByteArray sendData;
            QByteArray recvData;
            int timeout;
        } result = {false, send, QByteArray(), iTimeOut};

        // 使用Qt的invokeMethod在正确的线程中执行实际的操作
        QMetaObject::invokeMethod(port, [this, &result]() {
                QMutexLocker locker(&m_mutex);
                if (!IsConnected()) {
                    result.success = false;
                    LCNC_ERR(lcnc::LogCode::Generic, "{}", "ONCE Q not connected");
                    return;
                }

                // 保存当前的readyRead连接状态
                const bool isConnected = QObject::disconnect(port, nullptr, port, nullptr);

                // 清空所有缓冲区
				port->clear(QSerialPort::Input | QSerialPort::Output);
				m_qbDataBuffer.clear();
    
                // 添加短暂等待，让硬件有时间处理缓冲区清空
                if (QCoreApplication::instance()) {
                    QThread::msleep(15);
                } else {
                    boost::this_thread::sleep_for(boost::chrono::milliseconds(15));
                }

                // 发送数据
                qint64 bytesWritten = port->write(result.sendData);
                if (bytesWritten != result.sendData.size()) {
                    // 恢复信号连接
                    if (isConnected) {
                        QObject::connect(port, &QSerialPort::readyRead, port, [this] { slotReadData(); });
                    }
                    // 添加50ms延迟帮助设备恢复
                    if (QCoreApplication::instance()) {
                        QThread::msleep(20);
                    } else {
                        boost::this_thread::sleep_for(boost::chrono::milliseconds(20));
                    }
                    result.success = false;
                    LCNC_ERR(lcnc::LogCode::Generic, "{}", "ONCE Q write failed");
                    return;
                }

                // 等待数据发送完成
                if (QCoreApplication::instance()) {
                    if (!port->waitForBytesWritten(result.timeout)) {
                        if (isConnected) {
                            QObject::connect(port, &QSerialPort::readyRead, port, [this] { slotReadData(); });
                        }
                        // 添加50ms延迟帮助设备恢复
                        QThread::msleep(20);
                        result.success = false;
                        LCNC_ERR(lcnc::LogCode::Generic, "{}", "ONCE Q waitForBytesWritten timeout");
                        return;
                    }
                    // 添加短暂延迟，让设备有时间准备响应
                    QThread::msleep(20);
                } else {
                    // 基于波特率的动态延迟
                    int bytesPerSecond = m_dwBaudRate / 10;
                    int sendTime = (result.sendData.size() * 1000) / bytesPerSecond;
                    sendTime = qMax(10, qMin(sendTime + 100, result.timeout));
                    boost::this_thread::sleep_for(boost::chrono::milliseconds(sendTime));
                    // 添加短暂延迟，让设备有时间准备响应
                    boost::this_thread::sleep_for(boost::chrono::milliseconds(20));
                }

                // 等待并读取响应
                result.recvData.clear();
                int remainingTime = result.timeout;
                const int pollInterval = 50;

                while (remainingTime > 0)
                {
                    if (port->waitForReadyRead(pollInterval)) {
                        QByteArray newData = port->readAll();
                        result.recvData.append(newData);
                        if (IsAvailableData(result.recvData)) {
                            // 恢复信号连接
                            if (isConnected) {
                                QObject::connect(port, &QSerialPort::readyRead, port, [this] { slotReadData(); });
                            }
                            result.success = true;
                            return;
                        }
                    }
                    remainingTime -= pollInterval;
                }

                // 超时，恢复信号连接
                if (isConnected) {
                    QObject::connect(port, &QSerialPort::readyRead, port, [this] { slotReadData(); });
                }
                result.success = false;
                LCNC_ERR(lcnc::LogCode::Generic, "{}", "ONCE Q waitForReadyRead timeout");
            },
            Qt::BlockingQueuedConnection
        );

        // 将结果返回给调用者
        recv = result.recvData;
        return result.success;
    }

    // 已经在正确的线程中，直接执行
    QMutexLocker locker(&m_mutex);
    if (!IsConnected()) {
        LCNC_ERR(lcnc::LogCode::Generic, "{}", "ONCE B not connected");
        return false;
    }

    // 保存当前的readyRead连接状态
    const bool isConnected = QObject::disconnect(port, nullptr, port, nullptr);

    // 清空所有缓冲区
    port->clear(QSerialPort::Input | QSerialPort::Output);
    m_qbDataBuffer.clear();

    // 发送数据
    qint64 bytesWritten = port->write(send);
    if (bytesWritten != send.size()) {
        // 恢复信号连接
        if (isConnected) {
            QObject::connect(port, &QSerialPort::readyRead, port, [this] { slotReadData(); });
        }
        // 添加50ms延迟帮助设备恢复
        if (QCoreApplication::instance()) {
            QThread::msleep(50);
        } else {
            boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
        }
        LCNC_ERR(lcnc::LogCode::Generic, "{}", "ONCE B write failed");
        return false;
    }

    // 等待数据发送完成
    if (QCoreApplication::instance()) {
        if (!port->waitForBytesWritten(iTimeOut)) {
            if (isConnected) {
                QObject::connect(port, &QSerialPort::readyRead, port, [this] { slotReadData(); });
            }
            // 添加50ms延迟帮助设备恢复
            QThread::msleep(50);
            LCNC_ERR(lcnc::LogCode::Generic, "{}", "ONCE B waitForBytesWritten timeout");
            return false;
        }
        // 添加短暂延迟，让设备有时间准备响应
        QThread::msleep(5);
    } else {
        int bytesPerSecond = m_dwBaudRate / 10;
        int sendTime = (send.size() * 1000) / bytesPerSecond;
        sendTime = qMax(10, qMin(sendTime + 100, iTimeOut));
        boost::this_thread::sleep_for(boost::chrono::milliseconds(sendTime));
        // 添加短暂延迟，让设备有时间准备响应
        boost::this_thread::sleep_for(boost::chrono::milliseconds(5));
    }

    // 等待并读取响应
    recv.clear();
    int remainingTime = iTimeOut;
    const int pollInterval = 50;

    while (remainingTime > 0)
    {
        if (port->waitForReadyRead(pollInterval)) {
            QByteArray newData = port->readAll();
            recv.append(newData);
            if (IsAvailableData(recv)) {
                // 恢复信号连接
                if (isConnected) {
                    QObject::connect(port, &QSerialPort::readyRead, port, [this] { slotReadData(); });
                }
                return true;
            }
        }
        remainingTime -= pollInterval;
    }

    // 恢复信号连接
    if (isConnected) {
        QObject::connect(port, &QSerialPort::readyRead, port, [this] { slotReadData(); });
    }

    LCNC_ERR(lcnc::LogCode::Generic, "{}", "ONCE B waitForReadyRead timeout");
    return false;
}

const QStringList& SerialPort::GetComList() const
{
	static QStringList qsl_ComList;
	qsl_ComList.clear();
	QList<QSerialPortInfo> serialPorts = QSerialPortInfo::availablePorts();
	for (const QSerialPortInfo& portInfo : serialPorts)
	{
		QString qstr = portInfo.portName();
		QSerialPort testPort;
		testPort.setPortName(qstr);
		if (testPort.open(QIODevice::ReadWrite))
		{
			testPort.close();
			qsl_ComList.append(qstr);
		}
	}
	return qsl_ComList;
}

void SerialPort::SetComSetting(string strCom, string strBaudRate, string strDataBits, string strParity, string strStopBits)
{
	m_qstrPort = QString::fromStdString(strCom);
	m_dwBaudRate = static_cast<DWORD>(std::stoul(strBaudRate));
	m_iDataBits = std::stoi(strDataBits);

	if (strParity == "NONE")
		m_iParity = 0;
	else if (strParity == "ODD")
		m_iParity = 3;
	else if (strParity == "EVEN")
		m_iParity = 2;
	else if (strParity == "MARK")
		m_iParity = 5;
	else if (strParity == "SPACE")
		m_iParity = 4;

	if (strStopBits != "1.5")
		m_iStopBits = std::stoi(strStopBits);
	else
		m_iStopBits = 3;
}

bool SerialPort::IsComChange(string strCom, string strBaudRate, string strDataBits, string strParity, string strStopBits)
{
	if (m_qstrPort != QString::fromStdString(strCom))
		return true;

	if (m_dwBaudRate != static_cast<DWORD>(std::stoul(strBaudRate)))
		return true;

	if (m_iDataBits != std::stoi(strDataBits))
		return true;

	switch (m_iParity)
	{
	case 0:
		if (strParity != "NONE") return true;
		break;
	case 3:
		if (strParity != "ODD") return true;
		break;
	case 2:
		if (strParity != "EVEN") return true;
		break;
	case 5:
		if (strParity != "MARK") return true;
		break;
	case 4:
		if (strParity != "SPACE") return true;
		break;
	}

	if (m_iStopBits == 3)
	{
		if (strStopBits != "1.5") return true;
	}
	else
	{
		if (m_iStopBits != std::stoi(strStopBits)) return true;
	}

	return false;
}

void SerialPort::ADD_CRC_MODBUS(QByteArray& data, bool bByteOrder)
{
	uint16_t crc = 0xFFFF;
	for (int i = 0; i < data.length(); ++i) {
		crc ^= static_cast<uint8_t>(data[i]);
		for (int j = 0; j < 8; ++j) {
			if (crc & 0x0001) {
				crc = (crc >> 1) ^ 0xA001;
			}
			else {
				crc = crc >> 1;
			}
		}
	}

	if (bByteOrder)
	{
		data.append(crc & 0xFF);
		data.append((crc >> 8) & 0xFF);
	}
	else
	{
		data.append((crc >> 8) & 0xFF);
		data.append(crc & 0xFF);
	}
}

