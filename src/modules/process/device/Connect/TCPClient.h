#pragma once

#include <QObject>
#include <QTCPSocket>
#include <QString>
#include <QMutex>
#include <QWaitCondition>
#include "windows.h"
#include <string>
#include "MessageCode.h"
#include "toml.hpp"

using std::string;
using toml::table;

typedef std::function<void(const QByteArray&)> ResponseCallback;

class TCPClient : public QObject
{
	Q_OBJECT

public:
	TCPClient(QObject* parent = nullptr);
	virtual ~TCPClient();

public slots:
	void                slotReadData();
	void                slotDisconnected();

public:
	virtual ErrorCode   SetTCPTable(const table& tableTCP, bool& bConnectChange);
	virtual bool        Connect();
	virtual void        Disconnect();
	virtual bool        IsConnected();
	virtual bool        IsAvailableData(const QByteArray& data);
	
	bool SendCommand(const QByteArray& command, QByteArray& response);

	void                ClearBuffer();
	bool                OnceData(const QByteArray& send);
	bool                OnceData(const QByteArray& send, QByteArray& recv);
	QByteArray          ReadRawData();
	bool                WriteData(const QByteArray& data);

	bool                IsTCPChange(string strRemoteHost, int iRemotePort,int iTimeOut);

	// 设置接口参数
	void                SetTCPSetting(string strRemoteHost, int iRemotePort, int iTimeOut);
	void                SetRemoteHost(const QString& strHost)	{ m_qstrRemoteHost	= strHost;	}
	void                SetRemotePort(int iPort)				{ m_iRemotePort		= iPort;	}
	void                SetLocalPort(int iPort)					{ m_iLocalPort		= iPort;	}
	void                SetTimeout(int iTimeOut)				{ m_iTimeOut		= iTimeOut; }

	// 获取端口参数
	QString             GetRemoteHost() const	{ return m_qstrRemoteHost;	}
	int					GetRemotePort() const	{ return m_iRemotePort;		}
	int					GetLocalPort() const	{ return m_iLocalPort;		}
	int                 GetTimeout() const		{ return m_iTimeOut;		}

signals:
	void signalCommunication(const QString& command, const QString& response, bool success);

protected:
	QByteArray          ReadData();
	bool                DoOnceData(const QByteArray& send, QByteArray& recv, bool& success);

protected:
	bool                m_bConnected;
	QString             m_qstrRemoteHost;
	quint16             m_iRemotePort;
	quint16             m_iLocalPort;
	int                 m_iTimeOut;
	QByteArray          m_qbDataBuffer;

	QMutex              m_mutex;

private:
	QTcpSocket*         m_pSocket;
	ResponseCallback m_responseCallback;
	QByteArray m_responseData;
	QMutex m_responseMutex;
	QWaitCondition m_responseCondition;
};
