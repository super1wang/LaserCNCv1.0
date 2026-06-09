#pragma once

#include <QObject>
#include <QUDPSocket>
#include <QString>
#include <QMutex>
#include "windows.h"
#include <string>
#include "MessageCode.h"
#include "toml.hpp"

using std::string;
using toml::table;

class UDPClient : public QObject
{
	Q_OBJECT

public:
	UDPClient(QObject* parent = nullptr);
	virtual ~UDPClient();

public slots:
	void                slotReadData();

public:
	virtual ErrorCode   SetUDPTable(const table& tableUDP, bool& bConnectChange);
	virtual bool        Connect();
	virtual void        Disconnect();
	virtual bool        IsConnected();
	virtual bool        IsAvailableData(const QByteArray& data);

	void                ClearBuffer();
	bool                OnceData(const QByteArray& send);
	bool                OnceData(const QByteArray& send, QByteArray& recv);
	QByteArray          ReadRawData();
	bool                WriteData(const QByteArray& data);

	bool                IsUDPChange(string strRemoteHost, int iRemotePort, int iLocalPort, int iTimeOut, bool bMulticast = false);

	// 设置接口参数
	void                SetUDPSetting(string strRemoteHost, int iRemotePort, int iLocalPort, int iTimeOut, bool bMulticast = false);
	void                SetRemoteHost(const QString& strHost)	{ m_qstrRemoteHost		= strHost;		}
	void                SetRemotePort(int iPort)				{ m_iRemotePort			= iPort;		}
	void                SetLocalPort(int iPort)					{ m_iLocalPort			= iPort;		}
	void                SetTimeout(int iTimeout)				{ m_iTimeOut			= iTimeout;		}
	void                SetMulticast(bool bMulticast)			{ m_bMulticast			= bMulticast;	}

	// 获取端口参数
	QString             GetRemoteHost() const	{ return m_qstrRemoteHost;	}
	int					GetRemotePort() const	{ return m_iRemotePort;		}
	int					GetLocalPort() const	{ return m_iLocalPort;		}
	int                 GetTimeout() const		{ return m_iTimeOut;		}
	bool                IsMulticast() const		{ return m_bMulticast;		}

protected:
	QByteArray          ReadData();
	bool                DoOnceData(const QByteArray& send, QByteArray& recv, bool& success);

protected:
	bool                m_bConnected;
	QString             m_qstrRemoteHost;
	quint16             m_iRemotePort;
	quint16             m_iLocalPort;
	int                 m_iTimeOut;
	bool                m_bMulticast;
	QByteArray          m_qbDataBuffer;

	QMutex              m_mutex;

private:
	QUdpSocket*			m_pSocket;
};
