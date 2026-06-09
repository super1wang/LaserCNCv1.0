#pragma once
#include <QObject>
#include <QSerialPort>
#include <QString>
#include <QVector>
#include <QMutex>
#include "windows.h"
#include <string>
#include "MessageCode.h"
#include "toml.hpp"

using std::vector;
using std::wstring;
using std::string;
using toml::table;

class SerialPort : public QObject
{
	Q_OBJECT

public:
	SerialPort(QObject* parent = nullptr);
	virtual ~SerialPort();

public slots:
	void                slotReadData();                             // 处理接收到的数据

public:
	virtual ErrorCode   SetComTable(const table& tableCom, bool& bConnectChange);
	virtual bool        Connect();
	virtual void        Disconnect();
	virtual bool        IsConnected();
	virtual bool		IsAvailableData(const QByteArray& data);	// 返回数据校验方式，基类不校验

	void                ClearBuffer();												// 清空缓存
	bool				OnceData(const QByteArray& send, int iTimeOut = 1000);						// 一次通讯
	bool				OnceData(const QByteArray& send, QByteArray& recv, int iTimeOut = 1000);	// 一次通讯
	//bool				OnceData(const string& send, int iTimeOut = 1000);							// 一次通讯
	//bool				OnceData(const string& send, string& recv, int iTimeOut = 1000);			// 一次通讯

	bool                IsComChange(string strCom, string strBaudRate, string strDataBits, string strParity, string strStopBits); // 对比参数是否改变

	// 设置端口参数
	void                SetComSetting(string strCom, string strBaudRate, string strDataBits, string strParity, string strStopBits);
	void                SetPortName(const QString& wstrPort)	{ m_qstrPort = wstrPort;	 };
	void                SetBaudRate(DWORD dwBaudRate)			{ m_dwBaudRate = dwBaudRate; };
	void                SetDataBits(int iByteSize)				{ m_iDataBits = iByteSize;	 };
	void                SetParity(int iParity)					{ m_iParity = iParity;		 };
	void                SetStopBits(int iStopBits)				{ m_iStopBits = iStopBits;	 };

	// 获取端口参数
	const QStringList&	GetComList()	const;
	QString             GetPortName()	const	{ return m_qstrPort;	};
	DWORD               GetBaudRate()	const	{ return m_dwBaudRate;	};
	int                 GetDataBits()	const	{ return m_iDataBits;	};
	int                 GetParity()		const	{ return m_iParity;		};
	int                 GetStopBits()	const	{ return m_iStopBits;	};

protected:
	QByteArray          ReadData();													// 读取缓存区
	bool                WriteData(const QByteArray& data, int iTimeOut = 3000);     // 发送数据

public:
	// 辅助函数
	void                ADD_CRC_MODBUS(QByteArray& data, bool bByteOrder = true); // 添加校验位 CRC/MODBUS


protected:
	bool                m_bConnected;
	QString             m_qstrPort;
	DWORD               m_dwBaudRate;
	int                 m_iDataBits;
	int                 m_iParity;
	int                 m_iStopBits;
	QByteArray          m_qbDataBuffer;

	QMutex              m_mutex;

private:
	QSerialPort*		port;
};