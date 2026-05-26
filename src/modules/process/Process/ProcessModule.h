#ifndef PROCESSMODULE_H
#define PROCESSMODULE_H

#include "Service.h"
#include "DataType.h"
#include "ProcessModule/Process_TreeView.h"
#include <QList>

class QC_ApplicationWindow;
class RS_Document;
class RS_Entity;


using std::map;
using std::vector;

#define PROCESSMODULE ProcessModule::instance()

class ProcessModule : public QObject
{
	Q_OBJECT
public:
	ProcessModule(QC_ApplicationWindow* parent);
	// 函数宏指针
	static ProcessModule*	instance(ProcessModule* pProcessModule = nullptr);

	void					SetTreeView(ProcessTreeView* model);
	void					SetService(Service* pService);

public:
	// 流程的读写
	bool					SaveProcessValue(toml::value& valueProcess);
	bool					LoadProcessValue(const toml::value& valueProcess);

	// 流程的操控
	void					Init();
	void					Start(RunMode eModel = RunMode::SignMode);
	void					Pause();
	void					Continue();
	void					Stop();
	void					BackToIdle();

	// 上料位
	void					LoadingPos();

private:
	// 流程逻辑的实现
	void					SetRunMode(RunMode eModel);	// 设置运行模式
	void					GetRunItemList();			// 获取待运行的节点
	bool					CheckRunItemList();			// 检测运行节点中Group是否存在
	void					ProcessThread		(const vector<Item>& ItemList);						// 流程线程
	bool					RunItemList			(const vector<Item>& ItemList, bool bMain = false);	// 递归函数，运行逻辑
	bool					RunGroupBackground	(const vector<Item>& ItemList);						// 支线程函数，运行逻辑
	bool					StateListen			(const Item& item);									// 暂停、停止输入监听, 返回0正常，返回1结束
	void					MonitorThread();			// 监控线程，同时更新气压值
	void					PumpThread();				// 回水泵线程

	void					InitProcess();				// 初始化流程
	void					TestProcess();				// 测试流程
	void					ContinueProcess();			// 流程继续处理
	void					PauseProcess();				// 流程暂停处理
	void					StopProcess();				// 流程停止处理
	void					RestoreState();				// 恢复节点状态

	void					LoadingPosProcess();		// 上料位流程

	bool					WaitTime(int iWaitTime);	// 等待时间，单位毫秒，内置暂停、停止检测，停止等待返回false 

	// 节点运行封装
	void					ItemWait			(const map<QString, QString>& maps);
	void					ItemAxis			(const map<QString, QString>& maps);
	void					ItemAxesMove		(const map<QString, QString>& maps);
	void					ItemIO				(const map<QString, QString>& maps);
	void					ItemCommands		(const map<QString, QString>& maps);
	void					ItemMeasurement		(const map<QString, QString>& maps);
	void					ItemCalculation		(const map<QString, QString>& maps);
	void					ItemRunGroupCheck	(const map<QString, QString>& maps);
	void					ItemFeeding			(const map<QString, QString>& maps);
	void					ItemCutting			(const map<QString, QString>& maps);
	void					ItemOverCutting		(const map<QString, QString>& maps);
	void					ItemEnergySwitch	(const map<QString, QString>& maps);
	void					ItemCamera			(const map<QString, QString>& maps);
	void					ItemMarkAcquire		(const map<QString, QString>& maps);
	void					ItemAlignment		(const map<QString, QString>& maps);
	void					ItemAutoFocus		(const map<QString, QString>& maps);
	void					ClearAlignmentSequenceCache(RS_Document* document = nullptr);


	// 封装函数
	bool					StartBlow();				// 切割前吹气
	bool					AxisMoveToPos(Axis eAxis, double dPos, string strSpeedMode);		// 单轴绝对位置运动封装

signals:
	void					SignalUpdateTreeView();

private:
	// 变量
	int								m_iNestingNumber;			// 嵌套计数器
	RunMode							m_eRunMode;					// 特殊运行模式

	// 线程
	boost::thread*					m_pInitThread;				// 初始化线程
	boost::thread					m_ProcessThread;			// 流程线程
	boost::thread					m_ItemThread;				// 节点线程
	boost::thread					m_GroupThread;				// 并行组线程
	boost::thread					m_MonitorThread;			// 监控线程
	boost::thread					m_PumpThread;				// 回水泵线程

	boost::thread*					m_pLoadingPosThread;		// 上料位进程

	// 容器
	vector<Item>					vec_RunItemList;			// 运行节点容器
	map<QString, vector<Item>>		map_Group;

	ProcessTreeView*				m_pTreeView;
	Service*						m_pService;
	QList<RS_Entity*>				m_alignmentSequenceCache;
	QList<RS_Entity*>				m_alignmentSavedSequence;
	QList<RS_Entity*>				m_alignmentSavedSelectedSequence;
	bool							m_alignmentHasTemporaryCuttingScope = false;

	// 类指针
	static ProcessModule*			uniqueInstance;

public:
	bool							m_bProcessTest;				// 测试流程标志
	double							m_dPressure;				// 气压

private:
	QC_ApplicationWindow*			main_window;
};

#endif // PROCESSMODEL_H

#ifndef EXTERNATOMIC_H
#define EXTERNATOMIC_H

#include <boost/atomic.hpp>

// 状态
extern	boost::atomic<enum::SystemStatus>	g_eState;					// 系统状态
extern	boost::atomic<enum::ErrorCode>		g_eError;					// 错误代码

// 切割计数
extern	boost::atomic<int>					g_iCuttedOpening;			// 已切开口数
extern	boost::atomic<int>					g_iTotalOpening;			// 总开口数
extern	boost::atomic<int>					g_iCurrentFeeding;			// 当前进给数
extern	boost::atomic<int>					g_iTotalFeeding;			// 总进给数
extern	boost::atomic<int>					g_iCurrentNumber;			// 本次加工数
extern	boost::atomic<int>					g_iTotalNumber;				// 累计加工数

// 标志位
extern	boost::atomic<bool>					g_bInit;					// 初始化标志
extern	boost::atomic<bool>					g_bRunning;					// 流程运行标志
extern	boost::atomic<bool>					g_bFinish;					// 节点线程标志
extern	boost::atomic<bool>					g_bPause;					// 暂停标志
extern	boost::atomic<bool>					g_bPausing;					// 暂停中标志
extern	boost::atomic<bool>					g_bStop;					// 停止标志
extern	boost::atomic<bool>					g_bGroupRuning;				// 后台运行标志
extern	boost::atomic<bool>					g_bGroupError;				// 后台运行错误标志
#endif // EXTERNATOMIC_H