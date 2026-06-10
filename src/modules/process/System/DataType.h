#pragma once
#include <map>
#include <vector>
#include <string>
#include <QString>
#include "magic_enum.hpp"
#include "MessageCode.h"

// enum-string 互转枚举原始值范围，该值直接于该库源文件（magic_enum.hpp）中修改
//#define	MAGIC_ENUM_RANGE_MIN 0;
//#define	MAGIC_ENUM_RANGE_MAX 63;

// 轴系组，临时定义，后改为使用注册表读写
#define AXISGROUP		188

#define	MaxNestingNumber 1000		// 最大嵌套数
#define PRECISION 1e-10
#define HIGH_PRECISION 1e-11
#define LOW_PRECISION 1e-8
#define LOW_LOW_PRECISION 1e-7
#define LOW_LOW_LOW_PRECISION 1e-6
#define ENGI_PRECISION 1e-4
#define ONE_MICRO 1e-3
#define PI 3.14159265358979323846

#define ONE_MICRON 1E-3
#define FILE_PRECISION 1E-5
//#ifdef MAXIMUM
#define MAXIMUM 1E32

using std::vector;
using std::map;
using std::string;
using magic_enum::enum_cast;	//string -> enum		enum_cast<>().value()
using magic_enum::enum_name;	//enum -> string		enum_name().data()
using magic_enum::enum_names;	//enum -> auto			

// 用户权限
enum class PermissionLevel
{
	Developers		= 9,
	Factory			= 7,
	Simulate		= 6,
	Administrator	= 4,
	Technician		= 2,
	Operator		= 1,
	None			= 0
};

enum class Axis
{
	X  = 0,	Y  = 1,	Z  = 2,	A  = 3, 
	X1 = 4,	Y1 = 5,	Z1 = 6,	A1 = 7

// 机台轴系标识符说明，用轴所在位启用形式转二进制，再转十进制进行存储与判断条件
//				X	Y	Z	A	X1	Y1	Z1	A1	二进制		十进制
// X Y Z		1	1	1	0	0	0	0	0	11100000	224
// X A Z		1	0	1	1	0	0	0	0	10110000	176
// X A Z X1		1	0	1	1	1	0	0	0	10111000	184
// X A Z Y1		1	0	1	1	0	1	0	0	10110100	180
// X A Z Z1		1	0	1	1	0	0	1	0	10110010	178
// X A Z A1		1	0	1	1	0	0	0	1	10110001	177
// X A Z X1 Y1	1	0	1	1	1	1	0	0	10111100	188
// X A Z A1  Y1	1	0	1	1	0	1	0	1	10110101	181
// X Y Z A  A1	1	1	1	1	0	0	0	1	11110001	241
};

// 系统状态
enum class SystemStatus
{
	// 仅UnInit
	UnInit,		Initializing,
	Idle,
	Paused,		Pausing,
	Processing,	LaserProcessing,	// 激光加工中为加工中的子状态
	Error							// 报错时，提示错误代码
};

// 运行模式
enum class RunMode
{
	SignMode	= 0,	// 标准模式，正常切割
	CuttingTest = 1,	// 切割测试，不出光不吹气
	ProcessTest = 2,	// 流程测试，测试流程节点逻辑
};

// 总设置页
enum class SettingSection
{	
	// 添加新枚举需要在对应分类同步添加

	// 外设 Peripheral.toml
	MotionControl,
	Digital,
	Analog,
	Laser,
	Internet,
	Special,

	// 工艺 config.toml
	Tool,
	Axis,
	Gas,
	Water,
	Monitor,
	LoadingPos,
	Camera
};

enum class Peripheral
{
	// 外设 Peripheral.toml
	MotionControl,
	Digital,
	Analog,
	Laser,
	Internet,
	Special
};

enum class Technology
{
	// 工艺 config.toml
	Tool,
	Axis,
	Gas,
	Water,
	Monitor,
	LoadingPos,
	Camera
};

// 节点类型
enum class ItemType
{
	// 节点按执行速度进行区分，瞬时节点在ProcessThread中执行，非瞬时节点于ItemThread
	// 瞬时节点
	Start,		Stop,
	If,			Loop,			Group,			RunGroup,
	IO,			Camera,			Monitor,		//相机模块由后续修改， 监控节点暂时移除
	Calculation,Compare,
	Base,	// 原始节点，此处拿来当枚举中间值区分

	// 非瞬时节点
	Wait,		Commands,		Feeding,		RunGroupCheck,
	Axis,		AxesMove,		Measurement,	MarkAcquire,	Alignment,
	AutoFocus,	EnergySwitch,	Cutting,		OverCutting
};

// 节点状态
enum class ItemState
{
	// 特殊
	StateSave,  //禁止用于Set！该值用于SwitchState的默认值，用于设置成保存的状态
	// 非切割
	Disable,	Enable,		Editing,	Unavailable,
	// 切割
	Run,		Stop,		Pause,		Unuse,			Unrun
};

struct Item
{
	int						iParentIndex;
	int						iChildrenIndex;
	ItemType				Type;
	map<QString, QString>	maps;
};

// 数字量IN
enum class DigitalIN
{
	Start, Stop, InterLock,	RemnantsMonitor, SafetyLightCurtain,
	PressureMonitor,	WaterLeakageMonitor,	WaterTankMonitor,
	IN1,	IN2,	IN3,	IN4,	IN5,	IN6,	IN7,	IN8,
	IN9,	IN10,	IN11,	IN12,	IN13,	IN14,	IN15,	IN16,
	IN17,	IN18,	IN19,	IN20,	IN21,	IN22,	IN23,	IN24,
	IN25,	IN26,	IN27,	IN28,	IN29,	IN30,	IN31,	IN32,
	IN33,	IN34,	IN35,	IN36,	IN37,	IN38,	IN39,	IN40,
	IN41,	IN42,	IN43,	IN44,	IN45,	IN46,	IN47,	IN48,
	IN49,	IN50,	IN51,	IN52,	IN53,	IN54,	IN55,	IN56,
	IN57,	IN58,	IN59,	IN60,	IN61,	IN62,	IN63,	IN64
};

// 数字量OUT
enum class DigitalOUT
{
	Laser,	Blow,	Chuck,	Pliers,	Water,	Pump,
	RedLight,	YellowLight,	GreenLight, Buzzer,
	Blow2,
	OUT1,	OUT2,	OUT3,	OUT4,	OUT5,	OUT6,	OUT7,	OUT8,
	OUT9,	OUT10,	OUT11,	OUT12,	OUT13,	OUT14,	OUT15,	OUT16,
	OUT17,	OUT18,	OUT19,	OUT20,	OUT21,	OUT22,	OUT23,	OUT24,
	OUT25,	OUT26,	OUT27,	OUT28,	OUT29,	OUT30,	OUT31,	OUT32,
	OUT33,	OUT34,	OUT35,	OUT36,	OUT37,	OUT38,	OUT39,	OUT40,
	OUT41,	OUT42,	OUT43,	OUT44,	OUT45,	OUT46,	OUT47,	OUT48,
	OUT49,	OUT50,	OUT51,	OUT52,	OUT53,	OUT54,	OUT55,	OUT56,
	OUT57,	OUT58,	OUT59,	OUT60,	OUT61,	OUT62,	OUT63,	OUT64
};

// 模拟量IN
enum class AnalogIN
{
	WaterLevel,	WaterPressure,	Pressure,
	IN1,	IN2,	IN3,	IN4,	IN5,	IN6,	IN7,	IN8,
	IN9,	IN10,	IN11,	IN12,	IN13,	IN14,	IN15,	IN16,
	IN17,	IN18,	IN19,	IN20,	IN21,	IN22,	IN23,	IN24,
	IN25,	IN26,	IN27,	IN28,	IN29,	IN30,	IN31,	IN32,
	IN33,	IN34,	IN35,	IN36,	IN37,	IN38,	IN39,	IN40,
	IN41,	IN42,	IN43,	IN44,	IN45,	IN46,	IN47,	IN48,
	IN49,	IN50,	IN51,	IN52,	IN53,	IN54,	IN55,	IN56,
	IN57,	IN58,	IN59,	IN60,	IN61,	IN62,	IN63,	IN64
};

// 模拟量OUT
enum class AnalogOUT
{
	Laser,	Pressure,
	OUT1,	OUT2,	OUT3,	OUT4,	OUT5,	OUT6,	OUT7,	OUT8,
	OUT9,	OUT10,	OUT11,	OUT12,	OUT13,	OUT14,	OUT15,	OUT16,
	OUT17,	OUT18,	OUT19,	OUT20,	OUT21,	OUT22,	OUT23,	OUT24,
	OUT25,	OUT26,	OUT27,	OUT28,	OUT29,	OUT30,	OUT31,	OUT32,
	OUT33,	OUT34,	OUT35,	OUT36,	OUT37,	OUT38,	OUT39,	OUT40,
	OUT41,	OUT42,	OUT43,	OUT44,	OUT45,	OUT46,	OUT47,	OUT48,
	OUT49,	OUT50,	OUT51,	OUT52,	OUT53,	OUT54,	OUT55,	OUT56,
	OUT57,	OUT58,	OUT59,	OUT60,	OUT61,	OUT62,	OUT63,	OUT64
};

#include <QStringList>
class DT
{
// 渲染优化
private:
	static bool		RenderOpt;
public:
	static void		setRenderOpt(bool bUse) { RenderOpt = bUse; };
	static bool		IsRenderOptUse()		{ return RenderOpt; };

// 模拟模式
private:
	static bool		SimulatMode;
public:
	static void		setSimulatMode(bool bUse) { SimulatMode = bUse; };
	static bool		IsSimulatMode() { return SimulatMode; };

// 相机使能
private:
	static bool		UseCamera;
public:
	static void		setUseCamera(bool bUse) { UseCamera = bUse; };
	static bool		IsUseCamera() { return UseCamera; };

// 客户标识
private:
	static string	CustomerID;
public:
	static void		setCustomerID(string strID)	{ CustomerID = strID; }
	static string	getCustomerID()				{ return CustomerID; }
	
// 权限标识
private:
	static PermissionLevel	Permission;
public:
	static void				setPermission(PermissionLevel permission)	{ Permission = permission; }
	static PermissionLevel	getPermission()								{ return Permission; }


// 轴系组
private:
	static int			AxisGroup;
	static QStringList	DirectionXList;
	static QStringList	DirectionYList;
public:
	static void			setAxisGroup(int iAxisgroup)	
	{ 
		AxisGroup = iAxisgroup ; 
		if (DT::IsAxisUse(Axis::X))		{ DirectionXList.append("X");	}
		if (DT::IsAxisUse(Axis::X1))	{ DirectionXList.append("X1");	}
		if (DT::IsAxisUse(Axis::Y))		{ DirectionYList.append("Y");	}
		if (DT::IsAxisUse(Axis::A))		{ DirectionYList.append("A");	}
		if (DT::IsAxisUse(Axis::Y1))	{ DirectionYList.append("Y1");	}
		if (DT::IsAxisUse(Axis::A1))	{ DirectionYList.append("A1");	}
	}
	static int					getAxisGroup()					{ return AxisGroup; }
	static bool					IsAxisUse(Axis eAxis)			{ return (AxisGroup & (1 << (7 - (int)eAxis))) != 0; }
	static const QStringList&	getDirectionX()					{ return DirectionXList; };
	static const QStringList&	getDirectionY()					{ return DirectionYList; };


// IO索引
private:
	static QStringList DigitalINList;
	static QStringList DigitalOUTList;
	static QStringList AnalogINList;
	static QStringList AnalogOUTList;
public:
	static bool IsDigitalINList	 (const QString& qstr)		{ return DigitalINList.count(qstr);  };
	static bool IsDigitalOUTList (const QString& qstr)		{ return DigitalOUTList.count(qstr); };
	static bool IsAnalogINList	 (const QString& qstr)		{ return AnalogINList.count(qstr);	 };
	static bool IsAnalogOUTList  (const QString& qstr)		{ return AnalogOUTList.count(qstr);  };

	static void setDigitalINList (const QStringList& list)	{ DigitalINList  = list; };
	static void setDigitalOUTList(const QStringList& list)	{ DigitalOUTList = list; };
	static void setAnalogINList  (const QStringList& list)	{ AnalogINList	 = list; };
	static void setAnalogOUTList (const QStringList& list)	{ AnalogOUTList  = list; };

	static const QStringList& getDigitalINList()			{ return DigitalINList;	 };
	static const QStringList& getDigitalOUTList()			{ return DigitalOUTList; };
	static const QStringList& getAnalogINList()				{ return AnalogINList;	 };
	static const QStringList& getAnalogOUTList()			{ return AnalogOUTList;  };

// 相机指令组
private:
	static QStringList	CameraCommands;
public:
	static void					setCameraCommands(const QStringList& list) { CameraCommands = list; };
	static const QStringList&	getCameraCommands() { return CameraCommands; };
};

inline bool				DT::RenderOpt		= false;

inline bool				DT::SimulatMode		= false;

inline bool				DT::UseCamera		= false;

inline string			DT::CustomerID		= "Standard";

inline PermissionLevel	DT::Permission		= PermissionLevel::Operator;

inline int				DT::AxisGroup = 0;
inline QStringList		DT::DirectionXList  = {};
inline QStringList		DT::DirectionYList  = {};

inline QStringList		DT::DigitalINList	= {};
inline QStringList		DT::DigitalOUTList	= {};
inline QStringList		DT::AnalogINList	= {};
inline QStringList		DT::AnalogOUTList	= {};

inline QStringList		DT::CameraCommands	= {};
