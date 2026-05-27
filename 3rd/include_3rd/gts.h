#pragma once

#ifdef __linux__
// Linux平台下GT_API的定义
#define GT_API extern "C" short

/*
Windows平台下，部分函数使用了long数据类型的函数参数
在linux平台下，那些函数的long类型参数改为int32_t类型
Linux平台使用系统提供的stdint.h头文件来定义int32_t
*/
#include <stdint.h>

typedef int64_t __int64;

#else

#define GT_API extern "C" short __stdcall
#include "windows.h"

typedef signed char                    int8_t;
typedef short                          int16_t;
typedef int                            int32_t;

typedef unsigned char                  uint8_t;
typedef unsigned short                 uint16_t;
typedef unsigned int                   uint32_t;

typedef long long                       int64_t;
typedef unsigned long long              uint64_t;

#endif

/*------------------------------------------------------------------------*/
/*------------------------------------------------------------------------*/
/*                        标准发布功能函数		                          */
/*------------------------------------------------------------------------*/
/*------------------------------------------------------------------------*/

/*-----------------------------------------------------------*/
/* GVN高速核和通用核宏定义                                    */
/*-----------------------------------------------------------*/
#define GP_CORE                  (1)
#define HS_CORE                  (2)

/*-----------------------------------------------------------*/
/* Channel of Command                                        */
/*-----------------------------------------------------------*/
#define CHANNEL_HOST                   (0)
#define CHANNEL_UART                   (1)
#define CHANNEL_SIM                    (2)
#define CHANNEL_ETHER                  (3)
#define CHANNEL_RS232                  (4)
#define CHANNEL_PCIE                   (5)
#define CHANNEL_RINGNET                (6)

/*-----------------------------------------------------------*/
/* Parameter of Command                                        */
/*-----------------------------------------------------------*/
#define DSP_SPORT_2_0                  (2) // Sport2.0
#define DSP_POWER_OFF_RECOVER      (5) // Sport2.0,主卡断电，模块通用输出和使能保持

/*-----------------------------------------------------------*/
/* Error Code                                                */
/*-----------------------------------------------------------*/
#define CMD_SUCCESS                     (0)

#define CMD_ERROR_READ_LEN              (-2)     /* 读取数据长度错误 */
#define CMD_ERROR_READ_CHECKSUM         (-3)     /* 读取数据校验和错误 */

#define CMD_ERROR_WRITE_BLOCK           (-4)     /* 写入数据块错误 */
#define CMD_ERROR_READ_BLOCK            (-5)     /* 读取数据块错误 */

#define CMD_ERROR_OPEN                  (-6)     /* 打开设备错误 */
#define CMD_ERROR_CLOSE                 (-6)     /* 关闭设备错误 */
#define CMD_ERROR_DSP_BUSY              (-7)     /* DSP忙 */

#define CMD_LOCK_ERROR                  (-8)     /* 多线程资源忙 */
#define CMD_DMA_ERROR                   (-9)     /* DMA传输错误 */
#define CMD_COMM_ERROR                  (-10)    /* pcie通讯失败 */
#define CMD_LOAD_RINGNET_DLL_ERROR      (-11)    /* 等环网库加载失败 */
#define CMD_RINGNET_STIME_ERROR         (-12)    /* 等环网库加载失败 */

#define CMD_RINGNET_ENC0_ERROR          (-13)    /* core1编码器初始化失败 */

#define CMD_RINGNET_ENC1_ERROR          (-14)    /* core2编码器初始化失败 */

#define CMD_LOAD_RINGNET_ERROR          (-17)    /* 等环网库API加载失败 */
#define CMD_MCVERSION_MATCH_ERROR       (-15)    /* 等环网匹配失败，需要更新库 */
#define CMD_LOCK_NULL                   (-20)    /* 多线程保护打开句柄失败 */
#define CMD_MCVERSION_MATCH_WARNING     (15)     /* 运控版本匹配失败，某些功能不具备 */
#define CMD_DSPVERSION_MATCH_WARNING    (16)     /* 运控DSP版本较老,不具备版本匹配功能 */

#define CMD_FILE_MATCH_WARNING          (17)     /* 配置文件格式或日期不匹配 */

#define CMD_ERROR_EXECUTE               (1)
#define CMD_ERROR_VERSION_NOT_MATCH     (3)
#define CMD_ERROR_PARAMETER             (7)
#define CMD_ERROR_UNKNOWN               (8)      /* unspported command */

#define MC_NONE                         (-1)

#define MC_LIMIT_POSITIVE               (0)
#define MC_LIMIT_NEGATIVE               (1)
#define MC_ALARM                        (2)
#define MC_HOME                         (3)
#define MC_GPI                          (4)
#define MC_ARRIVE                       (5)
#define MC_EGPI0                        (6)
// #define MC_EGPI1                     (7)
// #define MC_EGPI2                     (8)
#define MC_MPG                          (9)

#define MC_ENABLE                       (10)
#define MC_CLEAR                        (11)
#define MC_GPO                          (12)
// #define MC_EGPO0                     (13)
// #define MC_EGPO1                     (14)
#define MC_AU_ADC                       (17)
#define MC_HSO                          (18)
#define MC_AU_DAC                       (19)


#define MC_DAC                          (20)
#define MC_STEP                         (21)
#define MC_PULSE                        (22)
#define MC_ENCODER                      (23)
#define MC_ADC                          (24)

#define MC_AU_ENCODER                   (26)

#define MC_ABS_ENCODER                  (29)

#define MC_AXIS                         (30)
#define MC_PROFILE                      (31)
#define MC_CONTROL                      (32)
#define MC_ACTVEL                       (33)
#define MC_PRF_VEL                      (34)
#define MC_PRF_POS                      (35)
#define MC_CRD                          (36)
#define MC_DR_FOLLOW_ERROR              (37) // 驱动器侧的跟随误差
#define MC_COMBINE_AXES                 (38)
#define MC_SERVO_READY                  (39) //伺服使能完成信号
#define MC_TRIGGER                      (40)

#define MC_AU_TRIGGER                   (44)
#define MC_SERVO_READY_TO_SWITCH_ON     (49) //伺服准备就绪信号
#define MC_TERMINAL                     (50)                                   // 从站模块类型资源
#define MC_NET_PORT                     (51)
#define MC_AU_ENCODER_EX                (52)
#define MC_MPG_ENCODER                  (53)
#define MC_SERIAL_NUMBER                (55)                                   // 模块的序列号资源标识
#define MC_REMOTE_AU_ENCODER            (55)
#define MC_SLOTS_NUMBER                 (56)                                   // 模块支持的槽数信息资源标识
#define MC_SUB_TERMINAL                 (57)                                   // 从站模块子板类型资源
#define MC_EXT_MODULE                   (60)
#define MC_EXT_DI                       (61)
#define MC_EXT_DO                       (62)
#define MC_EXT_AI                       (63)
#define MC_EXT_AO                       (64)
#define MC_EHMI_DI                      (65)
#define MC_EHMI_DO                      (66)
#define MC_LOCAL_TERMINAL_NO_RESOURCE   (67)
#define MC_AXIS_CONNECT                 (68)

#define MC_SCAN_CRD                     (70)
#define MC_LASER                        (71)
#define MC_LASER_AO                     (72)

#define MC_POS_COMPARE                  (80)

#define MC_WATCH_VAR                    (200)
#define MC_WATCH_EVENT                  (201)
#define MC_LIMIT_POSITIVE_RAW           (250)
#define MC_LIMIT_NEGATIVE_RAW           (251)
#define MC_ALARM_RAW                    (252)
#define MC_HOME_RAW                     (253)
#define MC_GPI_RAW                      (254)
#define MC_ARRIVE_RAW                   (255)
#define MC_MPG_RAW                      (259)

#define MC_GROUP                        (500)
#define MC_GROUP_PROFILE                (501)
#define MC_GROUP_SYNTHESIS              (505)
#define MC_GROUP_RTCP                   (510)

#define PROFILE_MODE_NONE               (-1)
#define PROFILE_MODE_TRAP               (0)
#define PROFILE_MODE_JOG                (1)
#define PROFILE_MODE_PT                 (2)
#define PROFILE_MODE_GEAR               (3)
#define PROFILE_MODE_FOLLOW             (4)
#define PROFILE_MODE_CRD                (5)
#define PROFILE_MODE_PVT                (6)
#define PROFILE_MODE_CAM                (7)

#define PROFILE_MODE_FOLLOW_EX          (10)
#define PROFILE_MODE_MOVE_ABSOLUTE      (20)
#define PROFILE_MODE_MOVE_VELOCITY      (30)

#define PROFILE_MODE_MOVE_CONTINUOUS               (40)
#define PROFILE_MODE_MOVE_JOG                      (41)
#define PROFILE_MODE_MOVE_SYNCHRONIZATION          (42)
#define PROFILE_MODE_MOVE_POS                      (48)

#define PROFILE_MODE_GROUP_MOVE_NONE               (100)
#define PROFILE_MODE_GROUP_MOVE_CONTINUOUS         (140)
#define PROFILE_MODE_GROUP_MOVE_JOG                (141)
#define PROFILE_MODE_GROUP_MOVE_SYNCHRONIZATION    (142)
#define PROFILE_MODE_GROUP_MOVE_POS                (148)
#define PROFILE_MODE_GROUP_MOVE_PATH               (150)

#define LIMIT_MODE_EXTERMAL          (0)
#define LIMIT_MODE_SOFT              (1)
#define LIMIT_MODE_ALL              (-1)

#define LIMIT_TYPE_POS              (0)
#define LIMIT_TYPE_NEG              (1)
#define LIMIT_TYPE_ALL              (-1)

typedef struct Version
{
    short year;
    short month;
    short day;
    short version;
    short user;
    short reserve1;
    short reserve2;
    short chip;
} TVersion;

#define CORE_MODE_TIMER                 (0)
#define CORE_MODE_SYNCH                 (1)
#define CORE_MODE_EXTERNAL              (2)

#define CORE_TASK_DEFAULT               (0)
#define CORE_TASK_DLM                   (1)

#define SKIP_MODULE_SCAN                (0x001)
#define SKIP_MODULE_POS_COMPARE         (0x002)
#define SKIP_MODULE_CRD                 (0x004)

#define SKIP_MODULE_PLC                 (0x010)
#define SKIP_MODULE_DLM                 (0x020)

#define SKIP_MODULE_AXIS_CALCULATE      (0x100)


#define SKIP_MODULE_WATCH               (0x800)

typedef enum TimeElapse
{
    TIME_ELAPSE_PROFILE = 1000,

    TIME_ELAPSE_INTERRUPT_ERROR_COUNT = 1100,

    TIME_ELAPSE_HOST_COMMAND_EXECUTE = 1220,
    TIME_ELAPSE_ETHER_COMMAND_EXECUTE,

    TIME_ELAPSE_PROFILE_CALCULATE = 6000,
    TIME_ELAPSE_BEFOR_PROFILE_CALCULATE = 6001,

    TIME_ELAPSE_GROUP_CALCULATE = 9100,

    TIME_ELAPSE_FORWARD_KINEMATIC = 9101,
    TIME_ELAPSE_INVERSE_KINEMATIC = 9102,

    TIME_ELAPSE_SCAN = 18000,

    TIME_ELAPSE_AXIS_CHECK = 20000,
    TIME_ELAPSE_AXIS_CALCULATE,
    TIME_ELAPSE_AXIS_FILTER,

    TIME_ELAPSE_ENCODER = 30000,

    TIME_ELAPSE_DI = 31000,

    TIME_ELAPSE_DO = 32000,

    TIME_ELAPSE_AI = 33000,

    TIME_ELAPSE_AO = 34000,

    TIME_ELAPSE_TRIGGER = 38000,

    TIME_ELAPSE_CONTROL = 40000,

    TIME_ELAPSE_WATCH = 52000,

    TIME_ELAPSE_TERMINAL = 53000,

    TIME_ELAPSE_TERMINAL_OPERATION = 53001,

    TIME_ELAPSE_READ_ADC = 54000,

} ETimeElapse;

typedef struct Pid
{
    double kp;
    double ki;
    double kd;
    double kvff;
    double kaff;
    long   integralLimit;
    long   derivativeLimit;
    short  limit;
} TPid;

#define RESET_TYPE_NONE			(-1)
#define RESET_TYPE_IO			(0)

typedef struct ResetIo
{
    short mode;		// mode为1表示保持，为0表示复位到初始状态
}TResetIo;

typedef union ResetModePrmUnion
{
    TResetIo resetIo;
    double data[7];
}TResetModePrmUnion;

typedef struct AxisCircularSafetyZone
{
    short axisIndex[2];
    short reserve1[2];
    double radius;
    double center[2];
}TAxisCircularSafetyZone;

typedef struct ProfileReferenceSafetyZone
{
    short refType;           // 安全区参考类型：MC_ENCODER/MC_AU_ENCODER/MC_AU_ENCODER_EX/MC_MPG_ENCODER
    short refIndex;          // 安全区参考类型对应的索引号
    short reserve1[2];
    double refRatio;         // 参考比例，refRatio = (轴1mm对应的脉冲数)/(参考类型1mm对应的脉冲数)
    double limitPositive;    // 安全区正向范围，单位：脉冲
    double limitNegative;    // 安全区负向范围，单位：脉冲
}TProfileReferenceSafetyZone;

typedef struct LimitInfo
{
    short hwLmtPositiveEnable;         // 正硬限位使能状态，0：关闭，1：打开
    short hwLmtNegativeEnable;         // 负硬限位使能状态，0：关闭，1：打开
    short swLmtPositiveEnable;         // 正软限位使能状态，0：关闭，1：打开
    short swLmtNegativeEnable;         // 负软限位使能状态，0：关闭，1：打开

    short hwLmtPositiveStatus;         // 正硬限位触发状态，0：未触发，1：触发
    short hwLmtNegativeStatus;         // 负硬限位触发状态，0：未触发，1：触发
    short swLmtPositiveStatus;         // 正软限位触发状态，0：未触发，1：触发
    short swLmtNegativeStatus;         // 负软限位触发状态，0：未触发，1：触发
}TLimitInfo;

/*-----------------------------------------------------------*/
/* Basic function                                            */
/*-----------------------------------------------------------*/
GT_API GT_SetCardNo(short index);
GT_API GT_GetCardNo(short* pIndex);
GT_API GT_Open(short channel = 0, short param = 1);
GT_API GT_Close(void);
GT_API GT_SetCore(short core);
GT_API GT_GetCore(short* pCore);
GT_API GT_GetVersion(char** pVersion);
GT_API GT_GetVersionEx(short type, TVersion* pVersion);
GT_API GT_Reset();
GT_API GT_GetClock(unsigned long* pClock, unsigned long* pLoop = NULL);
GT_API GT_GetClockHighPrecision(unsigned long* pClock);
GT_API GT_ClearTime(ETimeElapse item);
GT_API GT_GetTime(ETimeElapse item, unsigned long* pTime, unsigned long* pTimeMax, unsigned long* pValue = NULL);
GT_API GT_GetSts(short axis, long* pSts, short count = 1, unsigned long* pClock = NULL);
GT_API GT_ClrSts(short axis, short count = 1);
GT_API GT_AxisOn(short axis);
GT_API GT_AxisOff(short axis);
GT_API GT_MultiAxisOn(unsigned long mask);
GT_API GT_MultiAxisOff(unsigned long mask);
GT_API GT_SetAxisOnDelayTime(unsigned short ms);
GT_API GT_GetAxisOnDelayTime(unsigned short* pMs);
GT_API GT_Stop(long mask, long option);
GT_API GT_SetPrfPos(short profile, long prfPos);
GT_API GT_SynchAxisPos(long mask);
GT_API GT_ZeroPos(short axis, short count = 1);
GT_API GT_GetLimitStatus(short axis, short* pLimitPositive, short* pLimitNegative);
GT_API GT_SetSoftLimitMode(short axis, short mode);
GT_API GT_GetSoftLimitMode(short axis, short* pMode);
GT_API GT_SetSoftLimit(short axis, long positive, long negative);
GT_API GT_GetSoftLimit(short axis, long* pPositive, long* pNegative);
GT_API GT_SetAxisBand(short axis, long band, long time);
GT_API GT_GetAxisBand(short axis, long* pBand, long* pTime);
GT_API GT_GetPrfPos(short profile, double* pValue, short count = 1, unsigned long* pClock = NULL);
GT_API GT_GetPrfVel(short profile, double* pValue, short count = 1, unsigned long* pClock = NULL);
GT_API GT_GetPrfAcc(short profile, double* pValue, short count = 1, unsigned long* pClock = NULL);
GT_API GT_GetPrfMode(short profile, long* pValue, short count = 1, unsigned long* pClock = NULL);
GT_API GT_GetAxisPrfPos(short axis, double* pValue, short count = 1, unsigned long* pClock = NULL);
GT_API GT_GetAxisPrfPosCompensate(short axis, double* pValue, short count = 1, unsigned long* pClock = NULL);
GT_API GT_GetAxisPrfVel(short axis, double* pValue, short count = 1, unsigned long* pClock = NULL);
GT_API GT_GetAxisPrfAcc(short axis, double* pValue, short count = 1, unsigned long* pClock = NULL);
GT_API GT_GetAxisEncPos(short axis, double* pValue, short count = 1, unsigned long* pClock = NULL);
GT_API GT_GetAxisEncVel(short axis, double* pValue, short count = 1, unsigned long* pClock = NULL);
GT_API GT_GetAxisEncAcc(short axis, double* pValue, short count = 1, unsigned long* pClock = NULL);
GT_API GT_GetAxisError(short axis, double* pValue, short count = 1, unsigned long* pClock = NULL);
GT_API GT_SetControlFilter(short control, short index);
GT_API GT_GetControlFilter(short control, short* pIndex);
GT_API GT_SetControlSuperimposed(short control, short superimposedType, short superimposedIndex);
GT_API GT_GetControlSuperimposed(short control, short* pSuperimposedType, short* pSuperimposedIndex);
GT_API GT_SetPid(short control, short index, TPid* pPid);
GT_API GT_GetPid(short control, short index, TPid* pPid);
GT_API GT_SetKvffFilter(short control, short index, short kvffFilterExp, double accMax);
GT_API GT_GetKvffFilter(short control, short index, short* pKvffFilterExp, double* pAccMax);
GT_API GT_Delay(unsigned short ms);
GT_API GT_DelayHighPrecision(unsigned short profile);



GT_API GTN_Open(short channel = 5, short param = 2);
GT_API GTN_Close(void);

GT_API GTN_OpenRingNet(short channel = 5, short param = 2, char* pFile = NULL, short index = 1, long count = 1);
GT_API GTN_RingNetInit(short mode, char* pFile, short index);

GT_API GTN_GetChannel(short* pChannel);
GT_API GTN_GetVersion(short core, char** pVersion);
GT_API GTN_GetVersionEx(short core, short type, TVersion* pVersion);
GT_API GTN_SetVersion(short core, short type, TVersion* pVersion);
GT_API GTN_Reset(short core);
GT_API GTN_SetResetMode(short core, short type, TResetModePrmUnion* pResetPrm);
GT_API GTN_GetClock(short core, unsigned long* pClock, unsigned long* pLoop = NULL);
GT_API GTN_GetClockHighPrecision(short core, unsigned long* pClock);
GT_API GTN_ClearTime(short core, ETimeElapse item);
GT_API GTN_GetTime(short core, ETimeElapse item, unsigned long* pTime, unsigned long* pTimeMax, unsigned long* pValue = NULL);
GT_API GTN_SetCoreMode(short core, short mode);
GT_API GTN_GetCoreMode(short core, short* pMode);
GT_API GTN_SetCoreShare(short core, short type, short index, short count);
GT_API GTN_GetCoreShare(short core, short type, short* pIndex, short* pCount);
GT_API GTN_SetCoreTask(short core, short task);
GT_API GTN_GetCoreTask(short core, short* pTask);
GT_API GTN_GetResMax(short core, short type, short* pCount);
GT_API GTN_SetResCount(short core, short type, short count);
GT_API GTN_GetResCount(short core, short type, short* pCount);
GT_API GTN_SetDeviceShareMax(short core, short count);
GT_API GTN_GetDeviceShareMax(short core, short* pCount);
GT_API GTN_GetChipTemperature(double* pTempValue, double* pMinValue, double* pMaxValue);
GT_API GTN_SetTemperatureEnable(short core, short enable);
GT_API GTN_GetTemperature(short core, double* pTemperature);

GT_API GTN_GetSts(short core, short axis, long* pSts, short count = 1, unsigned long* pClock = NULL);
GT_API GTN_GetStsEx(short core, short axis, long* pSts, short count = 1, unsigned long* pClock = NULL);
GT_API GTN_GetPrfSts(short core, short profile, long* pSts, short count = 1, unsigned long* pClock = NULL);
GT_API GTN_GetDriverSts(short core, short axis, long* pSts, short count = 1, unsigned long* pClock = NULL);
GT_API GTN_GetDriverPosErr(short core, short station, short axis, long* posError);
GT_API GTN_SetDryRunVel(short core, short crd, double vel, double velG0);
GT_API GTN_GetDryRunVel(short core, short crd, double* pVel, double* pVelG0);
GT_API GTN_SetDryRunMode(short core, short crd, short mode);
GT_API GTN_GetDryRunMode(short core, short crd, short* pMode);
GT_API GTN_SetDrvPrfVel(short core, short axis, long* pPrfVel, short count);
GT_API GTN_GetDrvAtlVel(short core, short axis, long* pPrfVel, short count);
GT_API GTN_ClrSts(short core, short axis, short count = 1);
GT_API GTN_AxisOn(short core, short axis);
GT_API GTN_AxisOff(short core, short axis);
GT_API GTN_MultiAxisOn(short core, unsigned long mask);
GT_API GTN_MultiAxisOff(short core, unsigned long mask);
GT_API GTN_SetAxisOnDelayTime(short core, unsigned short delayTime);
GT_API GTN_GetAxisOnDelayTime(short core, unsigned short* pDelayTime);
GT_API GTN_Stop(short core, long mask, long option);
GT_API GTN_StopPro(short core, short* pAxis, short* pOption, short count);               // 核内轴数超过32轴使用该指令
GT_API GTN_SetPrfPos(short core, short profile, long prfPos);
GT_API GTN_SetPrfPosEx(short core, short profile, double pos);
GT_API GTN_SynchAxisPos(short core, long mask);
GT_API GTN_SynchAxisPosPro(short core, short* pAxis, short count);                       // 核内轴数超过32轴使用该指令
GT_API GTN_ZeroPos(short core, short axis, short count = 1);
GT_API GTN_GetLimitStatus(short core, short axis, short* pLimitPositive, short* pLimitNegative);
GT_API GTN_SetSoftLimitMode(short core, short axis, short mode);
GT_API GTN_GetSoftLimitMode(short core, short axis, short* pMode);
GT_API GTN_SetSoftLimit(short core, short axis, long positive, long negative);
GT_API GTN_GetSoftLimit(short core, short axis, long* pPositive, long* pNegative);
GT_API GTN_SetSoftLimitEx(short core, short axis, double positive, double negative);
GT_API GTN_GetSoftLimitEx(short core, short axis, double* pPositive, double* pNegative);
GT_API GTN_GetLimitInfo(short core, short axis, TLimitInfo* pInfo);
GT_API GTN_SetSoftLimitTrimMode(short core, short enable);
GT_API GTN_SetAxisBand(short core, short axis, long band, long time);
GT_API GTN_GetAxisBand(short core, short axis, long* pBand, long* pTime);
GT_API GTN_GetPrfPos(short core, short profile, double* pValue, short count = 1, unsigned long* pClock = NULL);
GT_API GTN_GetPrfVel(short core, short profile, double* pValue, short count = 1, unsigned long* pClock = NULL);
GT_API GTN_GetPrfAcc(short core, short profile, double* pValue, short count = 1, unsigned long* pClock = NULL);
GT_API GTN_GetPrfMode(short core, short profile, long* pValue, short count = 1, unsigned long* pClock = NULL);
GT_API GTN_GetAxisPrfPos(short core, short axis, double* pValue, short count = 1, unsigned long* pClock = NULL);
GT_API GTN_GetAxisPrfPosCompensate(short core, short axis, double* pValue, short count = 1, unsigned long* pClock = NULL);
GT_API GTN_GetAxisPrfVel(short core, short axis, double* pValue, short count = 1, unsigned long* pClock = NULL);
GT_API GTN_GetAxisPrfAcc(short core, short axis, double* pValue, short count = 1, unsigned long* pClock = NULL);
GT_API GTN_GetAxisEncPos(short core, short axis, double* pValue, short count = 1, unsigned long* pClock = NULL);
GT_API GTN_GetAxisEncVel(short core, short axis, double* pValue, short count = 1, unsigned long* pClock = NULL);
GT_API GTN_GetAxisEncAcc(short core, short axis, double* pValue, short count = 1, unsigned long* pClock = NULL);
GT_API GTN_GetAxisError(short core, short axis, double* pValue, short count = 1, unsigned long* pClock = NULL);
GT_API GTN_SetControlFilter(short core, short control, short index);
GT_API GTN_GetControlFilter(short core, short control, short* pIndex);
GT_API GTN_SetControlSuperimposed(short core, short control, short superimposedType, short superimposedIndex);
GT_API GTN_GetControlSuperimposed(short core, short control, short* pSuperimposedType, short* pSuperimposedIndex);
GT_API GTN_SetPid(short core, short control, short index, TPid* pPid);
GT_API GTN_GetPid(short core, short control, short index, TPid* pPid);
GT_API GTN_SetKvffFilter(short core, short control, short index, short kvffFilterExp, double accMax);
GT_API GTN_GetKvffFilter(short core, short control, short index, short* pKvffFilterExp, double* pAccMax);
GT_API GTN_Delay(short core, unsigned short ms);
GT_API GTN_DelayHighPrecision(short core, unsigned short profile);
GT_API GTN_SetLimit(short core, short axis, short type, void* pPrm);
GT_API GTN_GetLimit(short core, short axis, short type, void* pPrm);

typedef struct
{
    int16_t mode;
    int16_t reserve1[3];                // 保留，必须填0
    double adjustMinError;
    double reserve2[8];                // 保留，必须填0
} TPulseCloseWorkModePrm;

/**
* @brief 修改脉冲闭环控制模式及参数
* @param axis 脉冲闭环轴号
* @param pPulseModePrm->mode 工作模式，0，默认模式，1：有最小阈值
* @param pPulseModePrm->adjustMinErr 中断时间获取过程中的信息,0: 正常从FLASH中获取到,7: 获取到的中断事件不合理,其他: 读取FLASH出错
* @return 指令返回值
*/
GT_API GTN_SetPulseCloseWorkMode(short core,short axis,TPulseCloseWorkModePrm *pPulseModePrm);

/**
 * @brief 设置速度倍率模式
 * @param core 核号
 * @param mode 模式参数
 * @return 17056：模式参数错误
*/
GT_API GTN_SetFeedOverrideMode(short core,short mode);

/**
 * @brief 读取速度倍率模式
 * @param core 核号
 * @param pMode 模式参数
 * @return
 */
GT_API GTN_GetFeedOverrideMode(short core,short *pMode);

#define STEP_DIR                        (0)
#define STEP_PULSE                      (1)
#define STEP_ORTHOGONAL                 (2)

GT_API GT_LoadConfig(char* pFile);
GT_API GT_AlarmOff(short axis);
GT_API GT_AlarmOn(short axis);
GT_API GT_LmtsOn(short axis, short limitType = -1);
GT_API GT_LmtsOff(short axis, short limitType = -1);
GT_API GT_StepDir(short step);
GT_API GT_StepPulse(short step);
GT_API GT_StepOrthogonal(short step);
GT_API GT_SetMtrBias(short dac, short bias);
GT_API GT_GetMtrBias(short dac, short* pBias);
GT_API GT_SetMtrLmt(short dac, short limit);
GT_API GT_GetMtrLmt(short dac, short* pLimit);
GT_API GT_EncOn(short encoder);
GT_API GT_EncOff(short encoder);
GT_API GT_SetPosErr(short control, long error);
GT_API GT_GetPosErr(short control, long* pError);
GT_API GT_SetStopDec(short profile, double decSmoothStop, double decAbruptStop);
GT_API GT_GetStopDec(short profile, double* pDecSmoothStop, double* pDecAbruptStop);
GT_API GT_CtrlMode(short axis, short mode);
GT_API GT_SetStopIo(short axis, short stopType, short inputType, short inputIndex);
GT_API GT_SetAdcFilterPrm(short adc, double k);
GT_API GT_GetAdcFilterPrm(short adc, double* pk);
GT_API GT_SetAxisPrfVelFilter(short axis, short filterNumExp);
GT_API GT_GetAxisPrfVelFilter(short axis, short* pFilterNumExp);
GT_API GT_SetAxisEncVelFilter(short axis, short filterNumExp);
GT_API GT_GetAxisEncVelFilter(short axis, short* pFilterNumExp);
GT_API GT_SetProfileScale(short i, long alpha, long beta);
GT_API GT_GetProfileScale(short i, long* pAlhpa, long* pBeta);
GT_API GT_SetEncoderScale(short i, long alpha, long beta);
GT_API GT_GetEncoderScale(short i, long* pAlhpa, long* pBeta);

GT_API GTN_LoadConfig(short core, char* pFile);
GT_API GTN_AlarmOn(short core, short axis);
GT_API GTN_AlarmOff(short core, short axis);
GT_API GTN_ClearAlarm(short core, short axis, short count, unsigned short delayTime);


#define LIMIT_MODE_EXTERMAL     				(0)
#define LIMIT_MODE_SOFT         				(1)
#define LIMIT_MODE_ALL          				(-1)

GT_API GTN_LmtsOn(short core, short axis, short limitType = -1);
GT_API GTN_LmtsOnEx(short core, short axis, short limitType = -1, short limitMode = -1);
GT_API GTN_LmtsOff(short core, short axis, short limitType = -1);
GT_API GTN_LmtsOffEx(short core, short axis, short limitType = -1, short limitMode = -1);
GT_API GTN_StepDir(short core, short step);
GT_API GTN_StepPulse(short core, short step);
GT_API GTN_StepOrthogonal(short core, short step);
GT_API GTN_SetMtrBias(short core, short dac, short bias);
GT_API GTN_GetMtrBias(short core, short dac, short* pBias);
GT_API GTN_SetMtrLmt(short core, short dac, short limit);
GT_API GTN_GetMtrLmt(short core, short dac, short* pLimit);
GT_API GTN_SetSense(short core, short dataType, short dataIndex, short value);
GT_API GTN_GetSense(short core, short dataType, short dataIndex, short* pValue);
GT_API GTN_EncOn(short core, short encoder);
GT_API GTN_EncOff(short core, short encoder);
GT_API GTN_SetPosErr(short core, short control, long error);
GT_API GTN_GetPosErr(short core, short control, long* pError);
GT_API GTN_SetAxisPosErrLimit(short core, short axis, long errorLimit);
GT_API GTN_GetAxisPosErrLimit(short core, short axis, long* pErrorLimit);
GT_API GTN_SetStopDec(short core, short profile, double decSmoothStop, double decAbruptStop);
GT_API GTN_GetStopDec(short core, short profile, double* pDecSmoothStop, double* pDecAbruptStop);
GT_API GTN_CtrlMode(short core, short axis, short mode);
GT_API GTN_SetStopIo(short core, short axis, short stopType, short inputType, short inputIndex);
GT_API GTN_SetProbeCaptureStopAxis(short core, short encoder, short stopAxis, short stopType, short enable);
GT_API GTN_SetAxisPrfVelFilter(short core, short axis, short filterNumExp);
GT_API GTN_GetAxisPrfVelFilter(short core, short axis, short* pFilterNumExp);
GT_API GTN_SetAxisEncVelFilter(short core, short axis, short filterNumExp);
GT_API GTN_GetAxisEncVelFilter(short core, short axis, short* pFilterNumExp);
GT_API GTN_SetAxisFilterArrivalBand(short core, short axis, double band, long time);
GT_API GTN_GetAxisFilterArrivalBand(short core, short axis, double* pBand, long* pTime);
GT_API GTN_SetProfileScale(short core, short i, long alpha, long beta);
GT_API GTN_GetProfileScale(short core, short i, long* pAlhpa, long* pBeta);
GT_API GTN_SetEncoderScale(short core, short i, long alpha, long beta);
GT_API GTN_GetEncoderScale(short core, short i, long* pAlhpa, long* pBeta);
GT_API GTN_SetAuEncoderScale(short core, short i, long alpha, long beta);
GT_API GTN_GetAuEncoderScale(short core, short i, long* pAlhpa, long* pBeta);
GT_API GTN_SetEncoderMapRelation(short core, short masterEncType, short masterEncIndex, short mapEncType, short mapEncIndex);
GT_API GTN_GetEncoderMapRelation(short core, short masterEncType, short masterEncIndex, short* pMapEncType, short* pMapEncIndex);
GT_API GTN_SetProfilePpr(short core, short profile, double ppr);
GT_API GTN_GetProfilePpr(short core, short profile, double* pPpr);

/*-----------------------------------------------------------*/
/* Capture and Triggr                                        */
/*-----------------------------------------------------------*/
#define CAPTURE_HOME                    (1)
#define CAPTURE_INDEX                   (2)
#define CAPTURE_PROBE                   (3)
#define CAPTURE_HSIO0                   (6)
#define CAPTURE_HSIO1                   (7)
#define CAPTURE_POSCOMPARE              (6)

typedef struct Trigger
{
    short encoder;
    short probeType;
    short probeIndex;
    short sense;
    long  offset;
    unsigned long loop;
    short windowOnly;
    long firstPosition;
    long lastPosition;
}TTrigger;

typedef struct TriggerEx
{
    short latchType;
    short latchIndex;
    short probeType;
    short probeIndex;
    short sense;
    long  offset;
    unsigned long loop;
    short windowOnly;
    long firstPosition;
    long lastPosition;
}TTriggerEx;

typedef struct TriggerAlign
{
    short encoder;
    short probeType;
    short probeIndex;
    short sense;
    long offset;
    unsigned long loop;
    short windowOnly;
    short pad2;
    long firstPosition;
    long lastPosition;
}TTriggerAlign;

typedef struct TriggerStatus
{
    short execute;
    short done;
    long position;
}TTriggerStatus;

typedef struct TriggerStatusEx
{
    short execute;
    short done;
    long position;
    unsigned long clock;
    unsigned long loopCount;
}TTriggerStatusEx;


GT_API GT_SetTrigger(short index, TTrigger* pTrigger);
GT_API GT_GetTrigger(short index, TTrigger* pTrigger);
GT_API GT_GetTriggerStatus(short index, TTriggerStatus* pTriggerStatus, short count = 1);
GT_API GT_ClearTriggerStatus(short index);
GT_API GT_SetCaptureMode(short encoder, short mode);
GT_API GT_GetCaptureMode(short encoder, short* pMode, short count = 1);
GT_API GT_GetCaptureStatus(short encoder, short* pStatus, long* pValue, short count = 1, unsigned long* pClock = NULL);
GT_API GT_SetCaptureSense(short encoder, short mode, short sense);
GT_API GT_ClearCaptureStatus(short encoder);
GT_API GT_SetCaptureRepeat(short encoder, short count);
GT_API GT_GetCaptureRepeatStatus(short encoder, short* pCount);
GT_API GT_GetCaptureRepeatPos(short encoder, long* pValue, short startNum, short count);
GT_API GT_SetCaptureRepeatFifoMode(short encoder, short mode);
GT_API GT_GetCaptureRepeatFifoMode(short encoder, short* pMode);


typedef struct TriggerPrm
{
    short latchType;
    short latchIndex;
    short probeType;
    short probeIndex;
    short sense;
    short loopType;
    long  offset;
    unsigned long loop;
    short windowOnly;
    short pad1;
    long firstPosition;
    long lastPosition;
    short fifoMode;
    short pad2[3];
    double pad3;
}TTriggerPrm;
typedef struct LatchValueInfo
{
    short fifoFull;
    short pad1[3];
    double pad2[2];
}TLatchValueInfo;
#define TRIGGER_RESULT_FIFO_MODE_STATIC  0
#define TRIGGER_RESULT_FIFO_MODE_LOOP    1

typedef struct TriggerLatchPrm
{
    unsigned long latchIndex;//捕获触发的锁存序号。
    long latchValue;//锁存数值，和Trigger捕获锁存的类型有关
    double   userValue;//用户设置的数据，和TTriggerPrm的参数pad3对应
    double   reserve;//保留参数，必须为0
}TTriggerLatchPrm;

#define LATCH_USER_VAR_DOUBLE           (100)
#define LATCH_USER_VAR_LONG              (101)

GT_API GTN_SetAuTrigger(short core, short index, TTriggerEx* pTrigger);
GT_API GTN_GetAuTrigger(short core, short index, TTriggerEx* pTrigger);
GT_API GTN_DisableAuTrigger(short core, short index);
GT_API GTN_ClearAuTriggerStatus(short core, short index);
GT_API GTN_GetAuTriggerStatus(short core, short index, TTriggerStatusEx* pTriggerStatusEx, short count = 1);
GT_API GTN_SetTriggerPrm(short core, short index, TTriggerPrm* pTriggerPrm);
GT_API GTN_GetTriggerPrm(short core, short index, TTriggerPrm* pTriggerPrm);
GT_API GTN_GetTriggerLatchValue(short core, short index, long count, long* pValue, long* pCount, TLatchValueInfo* pInfo);
GT_API GTN_GetTriggerLatchValuePro(short core, short index, long count, TTriggerLatchPrm* pLatchPrm, long* pCount, TLatchValueInfo* pInfo);
GT_API GTN_SetTrigger(short core, short index, TTrigger* pTrigger);
GT_API GTN_GetTrigger(short core, short index, TTrigger* pTrigger);
GT_API GTN_SetTriggerEx(short core, short index, TTriggerEx* pTrigger);
GT_API GTN_GetTriggerEx(short core, short index, TTriggerEx* pTrigger);
GT_API GTN_GetTriggerStatus(short core, short index, TTriggerStatus* pTriggerStatus, short count = 1);
GT_API GTN_GetTriggerStatusEx(short core, short index, TTriggerStatusEx* pTriggerStatusEx, short count = 1);
GT_API GTN_ClearTriggerStatus(short core, short index);
GT_API GTN_DisableTrigger(short core, short index);
#define TRIGGER_DELTA_MODE_DEFAULT            (0)

#define TRIGGER_DELTA_CHECKPOINT_MODE_AUTO    (0)
#define TRIGGER_DELTA_CHECKPOINT_MODE_MANUAL  (1)

typedef struct Checkpoint
{
    short mode;
    long offset;
    short fifoIndex;
    unsigned long crossCount;
    short fifoDataCount;
    short dataReady;
    long data;
    short dataIndex;
} TCheckpoint;

typedef struct TriggerDeltaPrm
{
    short mode;
    short dir;
    short triggerIndex[2];
} TTriggerDeltaPrm;

typedef struct TriggerDelta
{
    short enable;
    short checkpointCount;
    short fifoDataCount;
    short lostCount;
}TTriggerDeltaInfo;

GT_API GT_ClearTriggerDelta(short index, short mode = 0);
GT_API GT_AddTriggerDeltaCheckpoint(short index, short mode, long offset, short fifo = 0, short* pIndex = NULL);
GT_API GT_ReadTriggerDeltaCheckpointData(short index, short checkpointIndex, long* pBuf, short count, short* pReadCount);
GT_API GT_WriteTriggerDeltaCheckpointData(short index, short checkpointIndex, long* pBuf, short count, short* pWriteCount);
GT_API GT_SetTriggerDeltaPrm(short index, TTriggerDeltaPrm* pPrm);
GT_API GT_GetTriggerDeltaPrm(short index, TTriggerDeltaPrm* pPrm);
GT_API GT_GetTriggerDeltaCheckpoint(short index, short checkpointIndex, TCheckpoint* pCheckpoint);
GT_API GT_GetTriggerDeltaInfo(short index, TTriggerDeltaInfo* pTriggerDelta);
GT_API GT_TriggerDeltaOn(short index);
GT_API GT_TriggerDeltaOff(short index);

GT_API GTN_ClearTriggerDelta(short core, short index, short mode = 0);
GT_API GTN_AddTriggerDeltaCheckpoint(short core, short index, short mode, long offset, short fifo = 0, short* pIndex = NULL);
GT_API GTN_ReadTriggerDeltaCheckpointData(short core, short index, short checkpointIndex, long* pBuf, short count, short* pReadCount);
GT_API GTN_WriteTriggerDeltaCheckpointData(short core, short index, short checkpointIndex, long* pBuf, short count, short* pWriteCount);
GT_API GTN_SetTriggerDeltaPrm(short core, short index, TTriggerDeltaPrm* pPrm);
GT_API GTN_GetTriggerDeltaPrm(short core, short index, TTriggerDeltaPrm* pPrm);
GT_API GTN_GetTriggerDeltaCheckpoint(short core, short index, short checkpointIndex, TCheckpoint* pCheckpoint);
GT_API GTN_GetTriggerDeltaInfo(short core, short index, TTriggerDeltaInfo* pTriggerDelta);
GT_API GTN_TriggerDeltaOn(short core, short index);
GT_API GTN_TriggerDeltaOff(short core, short index);

GT_API GT_LinkCaptureOffset(short encoder, short source);
GT_API GT_SetCaptureOffset(short encoder, long* pOffset, short count, long loop);
GT_API GT_GetCaptureOffset(short encoder, long* pOffset, short* pCount, long* pLoop);
GT_API GT_GetCaptureOffsetStatus(short encoder, short* pCount, long* pLoop, long* pCapturePos);

GT_API GT_SetCaptureEncoder(short trigger, short encoder);
GT_API GT_GetCaptureWidth(short trigger, short* pWidth, short count);

GT_API GTN_SetCaptureMode(short core, short encoder, short mode);
GT_API GTN_GetCaptureMode(short core, short encoder, short* pMode, short count = 1);
GT_API GTN_GetCaptureStatus(short core, short encoder, short* pStatus, long* pValue, short count, unsigned long* pClock);
GT_API GTN_SetCaptureSense(short core, short encoder, short mode, short sense);
GT_API GTN_ClearCaptureStatus(short core, short encoder);
GT_API GTN_SetCaptureRepeat(short core, short encoder, short count);
GT_API GTN_GetCaptureRepeatStatus(short core, short encoder, short* pCount);
GT_API GTN_GetCaptureRepeatPos(short core, short encoder, long* pValue, short startNum, short count);
GT_API GTN_SetCaptureRepeatFifoMode(short core, short encoder, short mode);
GT_API GTN_GetCaptureRepeatFifoMode(short core, short encoder, short* pMode);
GT_API GTN_LinkCaptureOffset(short core, short encoder, short source);
GT_API GTN_SetCaptureOffset(short core, short encoder, long* pOffset, short count, long loop);
GT_API GTN_GetCaptureOffset(short core, short encoder, long* pOffset, short* pCount, long* pLoop);
GT_API GTN_GetCaptureOffsetStatus(short core, short encoder, short* pCount, long* pLoop, long* pCapturePos);

GT_API GTN_AutoCaptureOn(short core, short encoder);
GT_API GTN_AutoCaptureOff(short core, short encoder);

typedef struct ToolPosition
{
    long position1;                    // 刀径检测捕获位置1，单位：脉冲
    long position2;                    // 刀径检测捕获位置2，单位：脉冲
}TToolPosition;

/**
 * @brief 读取Trigger捕获到的刀径位置
 * @param core 核号，索引从1开始
 * @param triggerIndex 需要读取捕获到刀径位置的Trigger起始索引，索引从1开始
 * @param pToolPos 刀径位置结构体数组指针，数组大小为count
 * @param count 需要读取捕获到刀径位置的Trigger数量，必须大于0
 * @return 17051：count数量超限，取值范围：[1,119]
 *         17052：捕获类型不支持，仅支持Trigger和AuTrigger
 *         17053：捕获起始索引超限，取值范围与控制器类型相关
*/
GT_API GTN_GetTriggerToolPosition(short core, short triggerIndex, TToolPosition* pToolPos, short count = 1);

/**
 * @brief 读取AuTrigger捕获到的刀径位置
 * @param core 核号，索引从1开始
 * @param auTriggerIndex 需要读取捕获到刀径位置的AuTrigger起始索引，索引从1开始
 * @param pToolPos 刀径位置结构体数组指针，数组大小为count
 * @param count 需要读取捕获到刀径位置的AuTrigger数量，必须大于0
 * @return 17051：count数量超限，取值范围：[1,119]
 *         17052：捕获类型不支持，仅支持Trigger和AuTrigger
 *         17053：捕获起始索引超限，取值范围与控制器类型相关
*/
GT_API GTN_GetAuTriggerToolPosition(short core, short auTriggerIndex, TToolPosition* pToolPos, short count = 1);

/*-----------------------------------------------------------*/
/* Basic Motion                                              */
/*-----------------------------------------------------------*/
typedef struct TrapPrm
{
    double acc;
    double dec;
    double velStart;
    short  smoothTime;
} TTrapPrm;

GT_API GT_Update(long mask);
GT_API GT_SetPos(short profile, long pos);
GT_API GT_GetPos(short profile, long* pPos);
GT_API GT_SetVel(short profile, double vel);
GT_API GT_GetVel(short profile, double* pVel);

GT_API GT_PrfTrap(short profile);
GT_API GT_SetTrapPrm(short profile, TTrapPrm* pPrm);
GT_API GT_GetTrapPrm(short profile, TTrapPrm* pPrm);
typedef struct TTrapTime
{
    double totalTime;
    double remainderTime;
    double pad[2];
} TTrapTime;
GT_API GTN_Update(short core, long mask);
GT_API GTN_UpdatePro(short core, short* pAxis, short count);                             // 核内轴数超过32轴使用该指令
GT_API GTN_SetPos(short core, short profile, long pos);
GT_API GTN_GetPos(short core, short profile, long* pPos);
GT_API GTN_SetVel(short core, short profile, double vel);
GT_API GTN_GetVel(short core, short profile, double* pVel);
GT_API GTN_SetPosArray(short core, short profile, double* pPos, short count);
GT_API GTN_GetPosArray(short core, short profile, double* pPos, short count);

GT_API GTN_PrfTrap(short core, short profile);
GT_API GTN_SetTrapPrm(short core, short profile, TTrapPrm* pPrm);
GT_API GTN_GetTrapPrm(short core, short profile, TTrapPrm* pPrm);
GT_API GTN_GetTrapTime(short core, short profile, TTrapTime* pTime);
GT_API GTN_GetTrapSts(short core, short profile, short* prfsts);
GT_API GTN_ClearTrapSts(short core, short profile);

GT_API GTN_SetTrapOverride(short core, short profile, double velRatio);
GT_API GTN_GetTrapOverride(short core, short profile, double* pVelRatio);
typedef struct JogPrm
{
    double acc;
    double dec;
    double smooth;
} TJogPrm;

GT_API GT_PrfJog(short profile);
GT_API GT_SetJogPrm(short profile, TJogPrm* pPrm);
GT_API GT_GetJogPrm(short profile, TJogPrm* pPrm);

GT_API GTN_PrfJog(short core, short profile);
GT_API GTN_SetJogPrm(short core, short profile, TJogPrm* pPrm);
GT_API GTN_GetJogPrm(short core, short profile, TJogPrm* pPrm);

#define PT_MODE_STATIC                  (0)
#define PT_MODE_DYNAMIC                 (1)

#define PT_SEGMENT_NORMAL               (0)
#define PT_SEGMENT_EVEN                 (1)
#define PT_SEGMENT_STOP                 (2)

typedef struct PtInfo
{
    double prfPos;
    long loop;
    short mode;
    short fifoUse;
    short fifoPlace;
    short segmentNumber;
    unsigned long segmentReceive[2];
    unsigned long segmentExecute[2];
    unsigned long bufferReceive[2];
    unsigned long bufferExecute[2];
} TPtInfo;

GT_API GT_PrfPt(short profile, short mode = PT_MODE_STATIC);
GT_API GT_SetPtLoop(short profile, long loop);
GT_API GT_GetPtLoop(short profile, long* pLoop);
GT_API GT_PtSpace(short profile, short* pSpace, short fifo = 0);
GT_API GT_PtSpaceEx(short profile, short* pSpace, short* pListSpace, short fifo = 0);
GT_API GT_PtData(short profile, double pos, long time, short type = PT_SEGMENT_NORMAL, short fifo = 0);
GT_API GT_PtClear(short profile, short fifo = 0);
GT_API GT_PtStart(long mask, long option = 0);
GT_API GT_SetPtMemory(short profile, short memory);
GT_API GT_GetPtMemory(short profile, short* pMemory);
GT_API GT_SetPtPrecisionMode(short profile, short precisionMode);
GT_API GT_GetPtPrecisionMode(short profile, short* pPrecisionMode);
GT_API GT_GetPtInfo(short profile, TPtInfo* pPtInfo);
GT_API GT_SetPtLink(short profile, short fifo, short list);
GT_API GT_GetPtLink(short profile, short fifo, short* pList);
GT_API GT_PtDoBit(short profile, short doType, short index, short value, short fifo = 0);
GT_API GT_PtAo(short profile, short aoType, short index, double value, short fifo = 0);
//GT_API GTN_PosCurrFeedForward(short core,short profile,double pos,long time,short torque,short type,short fifo=0);

GT_API GTN_PrfPt(short core, short profile, short mode = PT_MODE_STATIC);
GT_API GTN_SetPtLoop(short core, short profile, long loop);
GT_API GTN_GetPtLoop(short core, short profile, long* pLoop);
GT_API GTN_PtSpace(short core, short profile, short* pSpace, short fifo = 0);
GT_API GTN_PtSpaceEx(short core, short profile, short* pSpace, short* pListSpace, short fifo = 0);
GT_API GTN_PtData(short core, short profile, double pos, long time, short type = PT_SEGMENT_NORMAL, short fifo = 0);
GT_API GTN_PtDataPro(short core, short profile, double pos, double time, short type = PT_SEGMENT_NORMAL, short fifo = 0);
GT_API GTN_PtClear(short core, short profile, short fifo = 0);
GT_API GTN_PtStart(short core, long mask, long option = 0);
GT_API GTN_PtStartPro(short core, short* pAxis, short* pOption, short count);            // 核内轴数超过32轴使用该指令
GT_API GTN_SetPtMemory(short core, short profile, short memory);
GT_API GTN_GetPtMemory(short core, short profile, short* pMemory);
GT_API GTN_SetPtPrecisionMode(short core, short profile, short precisionMode);
GT_API GTN_GetPtPrecisionMode(short core, short profile, short* pPrecisionMode);
GT_API GTN_GetPtInfo(short core, short profile, TPtInfo* pPtInfo);
GT_API GTN_SetPtLink(short core, short profile, short fifo, short list);
GT_API GTN_GetPtLink(short core, short profile, short fifo, short* pList);
GT_API GTN_PtDoBit(short core, short profile, short doType, short index, short value, short fifo = 0);
GT_API GTN_PtAo(short core, short profile, short aoType, short index, double value, short fifo = 0);
GT_API GTN_PtSpaceDmaType(short core, short profile, short mode);
GT_API GTN_PtSpaceArray(short core, short profile, short* pSpace, short fifo, short count);
GT_API GTN_GetPtRemainder(short core, short profile, long* pRemainder);
GT_API GTN_PtDataArray(short core, short profile, double* pos, long time, short* type, short fifo, short count);
GT_API GTN_PtSmoothBegin(short core, short profile, long deltaTime, long degree, short fifo);
GT_API GTN_PtSmoothEnd(short core, short profile, short fifo);


typedef struct TPvtTableMovePrm
{
    short tableId;
    long distance;
    double vm;
    double am;
    double jm;
    double time;
} TPvtTableMovePrm;

GT_API GT_PrfPvt(short profile);
GT_API GT_SetPvtLoop(short profile, long loop);
GT_API GT_GetPvtLoop(short profile, long* pLoopCount, long* pLoop);
GT_API GT_PvtStatus(short profile, short* pTableId, double* pTime, short count = 1);
GT_API GT_PvtStart(long mask);
GT_API GTN_PvtStartPro(short core, short* pAxis, short count);                           // 核内轴数超过32轴使用该指令
GT_API GT_PvtTableSelect(short profile, short tableId);

GT_API GT_PvtTable(short tableId, long count, double* pTime, double* pPos, double* pVel);
GT_API GT_PvtTableEx(short tableId, long count, double* pTime, double* pPos, double* pVelBegin, double* pVelEnd);
GT_API GT_PvtTableComplete(short tableId, long count, double* pTime, double* pPos, double* pA, double* pB, double* pC, double velBegin = 0, double velEnd = 0);
GT_API GT_PvtTablePercent(short tableId, long count, double* pTime, double* pPos, double* pPercent, double velBegin = 0);
GT_API GT_PvtPercentCalculate(long n, double* pTime, double* pPos, double* pPercent, double velBegin, double* pVel);
GT_API GT_PvtTableContinuous(short tableId, long count, double* pPos, double* pVel, double* pPercent, double* pVelMax, double* pAcc, double* pDec, double timeBegin);
GT_API GT_PvtTableContinuousEx(short tableId, long n, double* pPos, double* pVel, double* pAccPercent, double* pDecPercent, double* pVelMax, double* pAcc, double* pDec, double timeBegin);
GT_API GT_PvtContinuousCalculate(long n, double* pPos, double* pVel, double* pPercent, double* pVelMax, double* pAcc, double* pDec, double* pTime);

GT_API GT_PvtTableMove(short tableId, long distance, double vm, double am, double jm, double* pTime = NULL);
GT_API GT_PvtTableMove2(short tableId, long distance, double vm, double am, double jm, double* pTime = NULL);
GT_API GT_PvtTableMovePercent(short tableId, long distance, double vm,
    double acc, double pa1, double pa2,
    double dec, double pd1, double pd2,
    double* pVel = NULL, double* pAcc = NULL, double* pDec = NULL, double* pTime = NULL);
GT_API GT_PvtTableMovePercentEx(short tableId, long distance, double vm,
    double acc, double pa1, double pa2, double ma,
    double dec, double pd1, double pd2, double md,
    double* pVel = NULL, double* pAcc = NULL, double* pDec = NULL, double* pTime = NULL);

GT_API GT_PvtTableMoveTogether(short tableCount, TPvtTableMovePrm* pPvtTableMovePrm);

GT_API GTN_PrfPvt(short core, short profile);
GT_API GTN_SetPvtLoop(short core, short profile, long loop);
GT_API GTN_GetPvtLoop(short core, short profile, long* pLoopCount, long* pLoop);
GT_API GTN_PvtStatus(short core, short profile, short* pTableId, double* pTime, short count = 1);
GT_API GTN_PvtStart(short core, long mask);
GT_API GTN_PvtTableSelect(short core, short profile, short tableId);

GT_API GTN_PvtTable(short core, short tableId, long count, double* pTime, double* pPos, double* pVel);
GT_API GTN_PvtTableEx(short core, short tableId, long count, double* pTime, double* pPos, double* pVelBegin, double* pVelEnd);
GT_API GTN_PvtTableComplete(short core, short tableId, long count, double* pTime, double* pPos, double* pA, double* pB, double* pC, double velBegin = 0, double velEnd = 0);
GT_API GTN_PvtTablePercent(short core, short tableId, long count, double* pTime, double* pPos, double* pPercent, double velBegin = 0);
GT_API GTN_PvtPercentCalculate(short core, long count, double* pTime, double* pPos, double* pPercent, double velBegin, double* pVel);
GT_API GTN_PvtTableContinuous(short core, short tableId, long count, double* pPos, double* pVel, double* pPercent, double* pVelMax, double* pAcc, double* pDec, double timeBegin);
GT_API GTN_PvtContinuousCalculate(short core, long count, double* pPos, double* pVel, double* pPercent, double* pVelMax, double* pAcc, double* pDec, double* pTime);

GT_API GTN_PvtTableMove(short core, short tableId, long distance, double vm, double am, double jm, double* pTime = NULL);
GT_API GTN_PvtTableMoveCalculate(short core, long distance, double vm, double am, double* pVm, double* pJm);
GT_API GTN_PvtTableMove2(short core, short tableId, long distance, double vm, double am, double jm, double* pTime = NULL);
GT_API GTN_PvtTableMove2Calculate(short core, long distance, double vm, double am, double* pVm, double* pJm);
GT_API GTN_PvtTableMoveAbsoluteJerk(short core, short profile, short tableId, double* pPos, double* pVelMax, double* pAccMax, double* pJerkMax, double* pTime, short count);
GT_API GTN_PvtTableMovePercent(short core, short tableId, long distance, double vm,
    double acc, double pa1, double pa2,
    double dec, double pd1, double pd2,
    double* pVel = NULL, double* pAcc = NULL, double* pDec = NULL, double* pTime = NULL);
GT_API GTN_PvtTableMovePercentEx(short core, short tableId, long distance, double vm,
    double acc, double pa1, double pa2, double ma,
    double dec, double pd1, double pd2, double md,
    double* pVel = NULL, double* pAcc = NULL, double* pDec = NULL, double* pTime = NULL);

GT_API GTN_PvtTableMoveTogether(short core, short tableCount, TPvtTableMovePrm* pPvtTableMovePrm);
GT_API GTN_SetPvtTableMoveAccList(short core, short tableId, short count, long* pDistance, double* pAccMax);
GT_API GTN_SetPvtTableLoop(short core, short profile, short tableId, long loop, short nextTableId);
GT_API GTN_GetPvtTableLoop(short core, short profile, short tableId, long* pLoopCount, long* pLoop, short* pNextTableId);
GT_API GTN_PvtTableMoveAbsolute(short core, short profile, double* pDistance, double* pVm,
    double* pAcc, double* pPa1, double* pPa2,
    double* pDec, double* pPd1, double* pPd2,
    double* pVelM, double* pAccM, double* pDecM, double* pTime, short count);


#define GEAR_MASTER_ENCODER             (1)
#define GEAR_MASTER_PROFILE             (2)
#define GEAR_MASTER_AXIS                (3)
#define GEAR_MASTER_AU_ENCODER          (4)
#define GEAR_MASTER_MPG_ENCODER         (5)
#define GEAR_MASTER_MPG                 (9)

#define GEAR_MASTER_ENCODER_OTHER       (101)
#define GEAR_MASTER_AXIS_OTHER          (103)
#define GEAR_MASTER_AU_ENCODER_OTHER    (104)
#define GEAR_MASTER_MPG_ENCODER_OTHER   (105)

#define GEAR_EVENT_START                (1)
#define GEAR_EVENT_PASS                 (2)
#define GEAR_EVENT_AREA                 (5)

GT_API GT_PrfGear(short profile, short dir = 0);
GT_API GT_SetGearMaster(short profile, short masterIndex, short masterType = GEAR_MASTER_PROFILE, short masterItem = 0);
GT_API GT_GetGearMaster(short profile, short* pMasterIndex, short* pMasterType = NULL, short* pMasterItem = NULL);
GT_API GT_SetGearRatio(short profile, long masterEven, long slaveEven, long masterSlope = 0);
GT_API GT_GetGearRatio(short profile, long* pMasterEven, long* pSlaveEven, long* pMasterSlope = NULL);
GT_API GT_GearStart(long mask);
GT_API GT_SetGearEvent(short profile, short event, long startPara0, long startPara1);
GT_API GT_GetGearEvent(short profile, short* pEvent, long* pStartPara0, long* pStartPara1);

GT_API GTN_PrfGear(short core, short profile, short dir = 0);
GT_API GTN_SetGearMaster(short core, short profile, short masterIndex, short masterType = GEAR_MASTER_PROFILE, short masterItem = 0);
GT_API GTN_GetGearMaster(short core, short profile, short* pMasterIndex, short* pMasterType = NULL, short* pMasterItem = NULL);
GT_API GTN_SetGearRatio(short core, short profile, long masterEven, long slaveEven, long masterSlope = 0);
GT_API GTN_GetGearRatio(short core, short profile, long* pMasterEven, long* pSlaveEven, long* pMasterSlope = NULL);
GT_API GTN_GearStart(short core, long mask);
GT_API GTN_GearStartPro(short core, short* pAxis, short count);                          // 核内轴数超过32轴使用该指令
GT_API GTN_SetGearEvent(short core, short profile, short event, long startPara0, long startPara1);
GT_API GTN_GetGearEvent(short core, short profile, short* pEvent, long* pStartPara0, long* pStartPara1);

typedef struct GearInParameter
{
    unsigned short masterIndex;        // GearIn模式跟随的主轴索引
    short masterValueSource;           // 跟随源
    short pad1;                        // 保留值，必须填0
    short pad2;                        // 保留值，必须填0
    long ratioNumerator;               // 齿轮比的分子
    unsigned long ratioDenominator;    // 齿轮比的分母
    double acceleration;               // 离合区的加速度
    double deceleration;               // 离合区的减速度
    double jerk;                       // 离合区的加加速度
    double pad3;                       // 保留值，必须填0
}TGearInParameter;

typedef struct GearInInfo
{
    short state;
    unsigned short masterIndex;        // GearIn模式跟随的主轴索引
    unsigned short masterValueSource;  // 跟随源
    short pad1[3];                     // 保留值，必须填0

    long ratioNumerator;               // 齿轮比的分子
    unsigned long ratioDenominator;    // 齿轮比的分母
    long slaveEven;                    // 除以公约数后的齿轮比的分子
    unsigned long masterEven;          // 除以公约数后的齿轮比的分母
    double acceleration;               // 离合区的加速度
    double deceleration;               // 离合区的减速度
    double jerk;                       // 离合区的加加速度
    double pad2;                       // 保留值，必须填0
}TGearInInfo;

GT_API GTN_SetGearInParameter(short core, short slaveIndex, TGearInParameter* pGearInPrm);
GT_API GTN_GearInStart(short core, unsigned long* pStartMask, unsigned short maskCount);
GT_API GTN_GetGearInInfo(short core, unsigned short slaveIndex, TGearInInfo* pGearInInfo);

#define FOLLOW_SWITCH_SEGMENT			(1)
#define FOLLOW_SWITCH_TABLE				(2)

#define FOLLOW_MASTER_ENCODER           (1)
#define FOLLOW_MASTER_PROFILE           (2)
#define FOLLOW_MASTER_AXIS              (3)
#define FOLLOW_MASTER_AU_ENCODER        (4)

#define FOLLOW_MASTER_ENCODER_OTHER     (101)
#define FOLLOW_MASTER_AXIS_OTHER        (103)

#define FOLLOW_EVENT_START              (1)
#define FOLLOW_EVENT_PASS               (2)

#define FOLLOW_SEGMENT_NORMAL           (0)
#define FOLLOW_SEGMENT_EVEN             (1)
#define FOLLOW_SEGMENT_STOP             (2)
#define FOLLOW_SEGMENT_CONTINUE         (3)

GT_API GT_PrfFollow(short profile, short dir = 0);
GT_API GT_SetFollowMaster(short profile, short masterIndex, short masterType = FOLLOW_MASTER_PROFILE, short masterItem = 0);
GT_API GT_GetFollowMaster(short profile, short* pMasterIndex, short* pMasterType = NULL, short* pMasterItem = NULL);
GT_API GT_SetFollowLoop(short profile, long loop);
GT_API GT_GetFollowLoop(short profile, long* pLoop);
GT_API GT_SetFollowEvent(short profile, short event, short masterDir, long pos = 0);
GT_API GT_GetFollowEvent(short profile, short* pEvent, short* pMasterDir, long* pPos = NULL);
GT_API GT_FollowSpace(short profile, short* pSpace, short fifo = 0);
GT_API GT_FollowData(short profile, long masterSegment, double slaveSegment, short type = FOLLOW_SEGMENT_NORMAL, short fifo = 0);
GT_API GT_FollowClear(short profile, short fifo = 0);
GT_API GT_FollowStart(long mask, long option = 0);
GT_API GT_FollowSwitch(long mask);
GT_API GT_SetFollowMemory(short profile, short memory);
GT_API GT_GetFollowMemory(short profile, short* pMemory);
GT_API GT_GetFollowStatus(short profile, short* pFifoNum, short* pSwitchStatus);
GT_API GT_SetFollowPhasing(short profile, short profilePhasing);
GT_API GT_GetFollowPhasing(short profile, short* pProfilePhasing);

GT_API GT_PrfFollowEx(short profile, short dir = 0);
GT_API GT_SetFollowMasterEx(short profile, short masterIndex, short masterType = FOLLOW_MASTER_PROFILE, short masterItem = 0);
GT_API GT_GetFollowMasterEx(short profile, short* pMasterIndex, short* pMasterType = NULL, short* pMasterItem = NULL);
GT_API GT_SetFollowLoopEx(short profile, long loop);
GT_API GT_GetFollowLoopEx(short profile, long* pLoop);
GT_API GT_SetFollowEventEx(short profile, short event, short masterDir, long pos = 0);
GT_API GT_GetFollowEventEx(short profile, short* pEvent, short* pMasterDir, long* pPos = NULL);
GT_API GT_FollowSpaceEx(short profile, short* pSpace, short fifo = 0);
GT_API GT_FollowDataPercentEx(short profile, double masterSegment, double slaveSegment, short type = FOLLOW_SEGMENT_NORMAL, short percent = 0, short fifo = 0);
GT_API GT_FollowClearEx(short profile, short fifo = 0);
GT_API GT_FollowStartEx(long mask, long option = 0);
GT_API GT_FollowSwitchEx(long mask);
GT_API GT_SetFollowMemoryEx(short profile, short memory);
GT_API GT_GetFollowMemoryEx(short profile, short* pMemory);
GT_API GT_GetFollowStatusEx(short profile, short* pFifoNum, short* pSwitchStatus);
GT_API GT_SetFollowPhasingEx(short profile, short profilePhasing);
GT_API GT_GetFollowPhasingEx(short profile, short* pProfilePhasing);
GT_API GT_FollowSwitchNowEx(short profile, short method, short buffer = 0, short fifo = 0);
GT_API GT_FollowDataPercent2Ex(short profile, double masterSegment, double slaveSegment, double velBeginRatio, double velEndRatio, short percent = 100, short* pPercent1 = NULL, short fifo = 0);
GT_API GT_GetFollowDataPercent2Ex(double masterPos, double v1, double v2, double p, double p1, double* pSlavePos);
GT_API GT_FollowDoBitEx(short profile, short doType, short index, short value, short fifo = 0);
GT_API GT_FollowDelayEx(short profile, unsigned long delayTime, short fifo = 0);
GT_API GT_FollowDiBitEx(short profile, short diType, short index, short value, unsigned long time = 0, short fifo = 0);

typedef struct FollowInfo
{
    short fifoNum;
    short switchStatus;
    short segNum;
    short pad1;
    double masterPosDelta;
    double slavePosDelta;
    double virtualMasterError;
    double pad2;
    double pad3;
} TFollowInfo;

GT_API GTN_PrfFollow(short core, short profile, short dir = 0);
GT_API GTN_SetFollowMaster(short core, short profile, short masterIndex, short masterType = FOLLOW_MASTER_PROFILE, short masterItem = 0);
GT_API GTN_GetFollowMaster(short core, short profile, short* pMasterIndex, short* pMasterType = NULL, short* pMasterItem = NULL);
GT_API GTN_SetFollowLoop(short core, short profile, long loop);
GT_API GTN_GetFollowLoop(short core, short profile, long* pLoop);
GT_API GTN_SetFollowEvent(short core, short profile, short event, short masterDir, long pos = 0);
GT_API GTN_GetFollowEvent(short core, short profile, short* pEvent, short* pMasterDir, long* pPos = NULL);
GT_API GTN_FollowSpace(short core, short profile, short* pSpace, short fifo = 0);
GT_API GTN_FollowData(short core, short profile, long masterSegment, double slaveSegment, short type = FOLLOW_SEGMENT_NORMAL, short fifo = 0);
GT_API GTN_FollowClear(short core, short profile, short fifo = 0);
GT_API GTN_FollowStart(short core, long mask, long option = 0);
GT_API GTN_FollowStartPro(short core, short* pAxis, short* pOption, short count);        // 核内轴数超过32轴使用该指令
GT_API GTN_FollowSwitch(short core, long mask);
GT_API GTN_FollowSwitchPro(short core, short* pAxis, short count);                       // 核内轴数超过32轴使用该指令
GT_API GTN_SetFollowMemory(short core, short profile, short memory);
GT_API GTN_GetFollowMemory(short core, short profile, short* pMemory);
GT_API GTN_GetFollowStatus(short core, short profile, short* pFifoNum, short* pSwitchStatus);
GT_API GTN_SetFollowPhasing(short core, short profile, short profilePhasing);
GT_API GTN_GetFollowPhasing(short core, short profile, short* pProfilePhasing);
GT_API GTN_GetFollowInfo(short core, short profile, TFollowInfo* pFollowInfo);
GT_API GTN_SetFollowVirtualSeg(short core, short profile, short seg, short axis, short fifo);
GT_API GTN_GetFollowVirtualSeg(short core, short profile, short* pSegment, short* pAxis, short fifo);
GT_API GTN_GetFollowVirtualErr(short core, short profile, double* pVirtualErr);
GT_API GTN_ClearFollowVirtualErr(short core, short profile);

GT_API GTN_PrfFollowEx(short core, short profile, short dir = 0);
GT_API GTN_SetFollowMasterEx(short core, short profile, short masterIndex, short masterType = FOLLOW_MASTER_PROFILE, short masterItem = 0);
GT_API GTN_GetFollowMasterEx(short core, short profile, short* pMasterIndex, short* pMasterType = NULL, short* pMasterItem = NULL);
GT_API GTN_SetFollowLoopEx(short core, short profile, long loop);
GT_API GTN_GetFollowLoopEx(short core, short profile, long* pLoop);
GT_API GTN_SetFollowEventEx(short core, short profile, short event, short masterDir, long pos = 0);
GT_API GTN_GetFollowEventEx(short core, short profile, short* pEvent, short* pMasterDir, long* pPos = NULL);
GT_API GTN_FollowSpaceEx(short core, short profile, short* pSpace, short fifo = 0);
GT_API GTN_FollowDataPercentEx(short core, short profile, double masterSegment, double slaveSegment, short type = FOLLOW_SEGMENT_NORMAL, short percent = 0, short fifo = 0);
GT_API GTN_FollowClearEx(short core, short profile, short fifo = 0);
GT_API GTN_FollowStartEx(short core, long mask, long option = 0);
GT_API GTN_FollowStartExPro(short core, short* pAxis, short* pOption, short count);      // 核内轴数超过32轴使用该指令
GT_API GTN_FollowSwitchEx(short core, long mask);
GT_API GTN_FollowSwitchExPro(short core, short* pAxis, short count);                     // 核内轴数超过32轴使用该指令
GT_API GTN_SetFollowMemoryEx(short core, short profile, short memory);
GT_API GTN_GetFollowMemoryEx(short core, short profile, short* pMemory);
GT_API GTN_GetFollowStatusEx(short core, short profile, short* pFifoNum, short* pSwitchStatus);
GT_API GTN_SetFollowPhasingEx(short core, short profile, short profilePhasing);
GT_API GTN_GetFollowPhasingEx(short core, short profile, short* pProfilePhasing);
GT_API GTN_FollowSwitchNowEx(short core, short profile, short method, short buffer = 0, short fifo = 0);
GT_API GTN_FollowDataPercent2Ex(short core, short profile, double masterSegment, double slaveSegment, double velBeginRatio, double velEndRatio, short percent = 100, short* pPercent1 = NULL, short fifo = 0);
GT_API GTN_GetFollowDataPercent2Ex(short core, double masterPos, double v1, double v2, double p, double p1, double* pSlavePos);
GT_API GTN_FollowDoBitEx(short core, short profile, short doType, short index, short value, short fifo = 0);
GT_API GTN_FollowDelayEx(short core, short profile, unsigned long delayTime, short fifo = 0);
GT_API GTN_FollowDiBitEx(short core, short profile, short diType, short index, short value, unsigned long time = 0, short fifo = 0);
/**
 * @brief Follow缓冲区操作扩展模块，执行该指令前，需要先初始化扩展模块，并将扩展模块的控制权切换至控制器，如：sRtn = GTN_ExtModuleInit(CORE_1,100);
 * @param core 核号
 * @param profile Follow跟随轴号
 * @param doIndex 操作扩展模块DO的起始索引
 * @param value 操作扩展模块DO的值，按bit位操作
 * @param mask 操作扩展模块DO的值的控制掩码，按bit位操作，bit为1时才输出响应value的bit的值
 * @param fifo Follow缓冲区号，Follow缓冲区一共有两个，
 * @return：
            7：参数错误 / 扩展模块未初始化
            1：执行错误，跟随轴未设置FollowEx规划模式 / 非运动指令段数超过16段
*/
GT_API GTN_FollowExtDoEx(short core,short profile,short doIndex,int32_t value,int32_t mask, short fifo);
GT_API GTN_SetAxisFollowErrorMode(short core, short axis, short followErrorMode);
GT_API GTN_GetAxisFollowErrorMode(short core, short axis, short* pFollowErrorMode);

GT_API GTN_SetFollowRegist(short core, short profile, short segment, short fifo);
GT_API GTN_GetFollowRegist(short core, short profile, short* pSegment, short fifo);
GT_API GTN_GetFollowRegistCount(short core, short profile, unsigned long* pCount);

GT_API GT_SetFollowVirtualSeg(short profile, short segment, short axis, short fifo);
GT_API GT_GetFollowVirtualSeg(short profile, short* pSegment, short* pAxis, short fifo);

GT_API GT_GetFollowVirtualErr(short profile, double* pVirtualErr);
GT_API GT_ClearFollowVirtualErr(short profile);

typedef struct MoveAbsolutePrm
{
    long pos;
    double vel;
    double acc;
    double dec;
    short percent;
} TMoveAbsolutePrm;

typedef struct MoveAbsolutePrmEx
{
    long pos;
    double vel;
    double acc;
    double dec;
    short percent;
    double velStart;
    double velEnd;
    short accStartPercent;
    short decEndPercent;
} TMoveAbsolutePrmEx;

GT_API GT_MoveAbsolute(short profile, TMoveAbsolutePrm* pPrm);
GT_API GT_GetMoveAbsolute(short profile, TMoveAbsolutePrm* pPrm);

GT_API GTN_MoveAbsolute(short core, short profile, TMoveAbsolutePrm* pPrm);
GT_API GTN_GetMoveAbsolute(short core, short profile, TMoveAbsolutePrm* pPrm);
GT_API GTN_MoveAbsoluteEx(short core, short profile, TMoveAbsolutePrmEx* pPrm);
GT_API GTN_GetMoveAbsoluteEx(short core, short profile, TMoveAbsolutePrmEx* pPrm);
typedef struct MoveVelocityPrm
{
    double vel;
    double acc;
    double dec;
    double jerkBegin;
    double jerkEnd;
    short direction;
} TMoveVelocityPrm;
GT_API GTN_MoveVelocity(short core, short profile, TMoveVelocityPrm* pPrm);
GT_API GTN_GetMoveVelocity(short core, short profile, TMoveVelocityPrm* pPrm);

#define LISTINFO_RESERVE2_USERTAG                           (0)
typedef struct ListInfo
{
    short list;
    short reserve1[2];
    short modal;
    long segNum;
    long reserve2[3];
    double reserve3[4];
}TListInfo;
/*-----------------------------------------------------------*/
/* Move                                                      */
/*-----------------------------------------------------------*/
#define VEL_PROFILE_MODE_TRAP               (1)
#define VEL_PROFILE_MODE_SMOOTH             (40)
#define VEL_PROFILE_MODE_SMOOTH_EX          (41)
#define VEL_PROFILE_MODE_JERK               (46)
#define VEL_PROFILE_MODE_FOLLOW             (50) // group的速度规划为跟随模式

typedef struct VelProfileModeSmooth
{
    double accTime;				// 加速度变化时间
    double k;					// 形态
    double reserve[18];
} TVelProfileModeSmooth;

typedef struct VelProfileModeJerk
{
    double value;				// 加加速度
    double beginPercent;
    double endPercent;
    double reserve[17];
} TVelProfileModeJerk;

typedef struct
{
    short mode;                        // group跟随模式：cam模式
    short reserve1[7];
    double reserve2[18];
} TVelProfileModeFollow;

typedef union VelProfileModeParameter
{
    TVelProfileModeSmooth smooth;
    TVelProfileModeJerk jerk;
    TVelProfileModeFollow follow;
    double data[20];
} TVelProfileModeParameter;

typedef struct VelProfileMode
{
    short mode;			// 模式
    short reserve[3];
    TVelProfileModeParameter parameter;
} TVelProfileMode;

typedef struct MoveContinuousAbsolutePrm
{
    double pos;
    double vel;
    double velEnd;
    double acc;
    double dec;
    short  direction;
    short overrideSelect;
    short reserve;
    short velProfileMode;
    TVelProfileModeParameter velProfile;
} TMoveContinuousAbsolutePrm;
typedef struct MoveContinuousRelativePrm
{
    double distance;
    double vel;
    double velEnd;
    double acc;
    double dec;
    short  direction;
    short overrideSelect;
    short reserve;
    short velProfileMode;
    TVelProfileModeParameter velProfile;
} TMoveContinuousRelativePrm;

typedef struct TaskMoveContinuousAbsolute
{
	short profile;
	short reserve[3];
	short group;
	short cmdCoord;
	short oriMode;
	short configIndex;
	TMoveContinuousAbsolutePrm prm;
} TTaskMoveContinuousAbsolute;
GT_API GTN_MoveContinuousAbsolute(short core, short profile, TMoveContinuousAbsolutePrm* pPrm, TListInfo* pListInfo = NULL, short group = 0);
GT_API GTN_MoveContinuousRelative(short core, short profile, TMoveContinuousRelativePrm* pPrm, TListInfo* pListInfo = NULL, short group = 0);

#define MOVE_JOG_MODE_GENERAL    0  //JOG模式
typedef struct JogParameter
{
    short overrideSelect;
    short reserve1[3];
    double vel;
    double acc;
    double dec;
    short reserve[44];
}TJogParameter;
typedef union MoveJogPrmUnion
 {
    TJogParameter jog;
    short reserve[60];
}TMoveJogPrmUnion;
typedef struct MoveJogPrm
{
    short mode;
    short reserve[3];
    TMoveJogPrmUnion data;
}TMoveJogPrm;
GT_API GTN_MoveJog(short core, short index, TMoveJogPrm* pPrm, TListInfo* pListInfo = NULL, short group = 0);
GT_API GTN_MoveTrap(short core, short profile, double pos, double vel, TTrapPrm* pPrm, TListInfo* pList = NULL);
/*-----------------------------------------------------------*/
/* Command Array			                                       */
/*-----------------------------------------------------------*/
GT_API GTN_SetVelArray(short core, short profile, double* pVel, short count);
GT_API GTN_SetTrapPrmArray(short core, short profile, TTrapPrm* pPrm, short count);
GT_API GTN_ClearTriggerStatusArray(short core, short trigger, short count);
/*-----------------------------------------------------------*/
/* MovePos                                                   */
/*-----------------------------------------------------------*/
typedef struct MovePosPercent
{
    short value;
}TMovePosPercent;

typedef struct MovePosSmoothTime
{
    short value;
}TMovePosSmoothTime;

typedef union MovePosUnion
{
    TMovePosPercent percent;
    TMovePosSmoothTime smoothTime;
    double value[2];
}TMovePosUnion;

typedef struct MovePosParameter
{
    double pos;
    double vel;
    double acc;
    double dec;
    short direction;
    short overrideSelect;
    short pad;
    short mode;
    TMovePosUnion data;
}TMovePosParameter;

typedef struct MovePosTwoSegmentParameter
{
    double pos[2];
    double vel[2];
    double acc;
    double dec;
    short direction;
    short overrideSelect;
    short pad;
    short mode;
    TMovePosUnion data;
}TMovePosTwoSegmentParameter;

typedef struct MultiMovePosParameter
{
    short profile;
    short pad1[3];
    double pos;
    double vel;
    double acc;
    double dec;
    short direction;
    short overrideSelect;
    short pad2;
    short mode;
    TMovePosUnion data;
}TMultiMovePosParameter;

typedef struct MultiMovePosTwoSegmentParameter
{
    short profile;
    short pad1[3];
    double pos[2];
    double vel[2];
    double acc;
    double dec;
    short direction;
    short overrideSelect;
    short pad2;
    short mode;
    TMovePosUnion data;
}TMultiMovePosTwoSegmentParameter;

GT_API GTN_MovePos(short core, short profile, TMovePosParameter* pMovePos, TListInfo* pListInfo = NULL, short group = 0);
GT_API GTN_MovePosTwoSegment(short core, short profile, TMovePosTwoSegmentParameter* pMovePos, TListInfo* pListInfo = NULL, short group = 0);
GT_API GTN_MultiMovePos(short core, TMultiMovePosParameter* pMovePos, short count = 1, TListInfo* pListInfo = NULL, short group = 0);
GT_API GTN_MultiMovePosTwoSegment(short core, TMultiMovePosTwoSegmentParameter* pMovePos, short count = 1, TListInfo* pListInfo = NULL, short group = 0);
#define MOTION_RESTRICT_PROFILE_MAX						(8)

#define MOTION_TIME_RESTRICT_MODE_DIRECT				(1)
#define MOTION_TIME_RESTRICT_MODE_ATTENTION_PROFILE		(2)
typedef struct MotionTimeRestrictDirect
{
    short condition[MOTION_RESTRICT_PROFILE_MAX];
    double time[MOTION_RESTRICT_PROFILE_MAX];
}TMotionTimeRestrictDirect;

typedef struct MotionTimeRestrictAttentionProfile
{
    short condition;
    short attentionProfileCount;
    short attentionProfile[MOTION_RESTRICT_PROFILE_MAX];
    short reserve1[6];
    double settlingTime;
}TMotionTimeRestrictAttentionProfile;

typedef union MotionTimeRestrictUnion
{
    TMotionTimeRestrictDirect direct;
    TMotionTimeRestrictAttentionProfile attentionProfile;
    double data[32];
}TMotionTimeRestrictUnion;

typedef struct MotionTimeRestrict
{
    short profileCount;
    short profile[MOTION_RESTRICT_PROFILE_MAX];
    short pad1[7];
    double profilePos[MOTION_RESTRICT_PROFILE_MAX];

    short mode;
    short pad2[3];
    TMotionTimeRestrictUnion parameter;
}TMotionTimeRestrict;
GT_API GTN_SetMotionTimeRestrict(short core, short restrictIndex, TMotionTimeRestrict* pRestrict, TListInfo* pListInfo = NULL);
GT_API GTN_GetMotionTimeRestrict(short core, short restrictIndex, TMotionTimeRestrict* pRestrict);

#define MOVE_SYNCHRONIZATION_MODE_GEAR    0  //gear模式(位置同步模式)
#define MOVE_SYNCHRONIZATION_MODE_MPG     1  //mpg同步模式(位置同步模式和速度同步模式1切换)
#define MOVE_SYNCHRONIZATION_MODE_FOLLOW  2  //follow同步模式(速度跟随模式(速度同步模式2))
#define MOVE_SYNCHRONIZATION_MODE_GEAR_POS  3  //位置模式(位置同步模式)
typedef struct ProfileItem
{
    short type;     // 类型
    short index;    //索引
    short subIndex; // 二级索引
    short reserve[5];
}TProfileItem;

typedef struct GearParameter
{
	double masterEven;          // 传动比主轴
    double slaveEven;           // 传动比从轴
	double slope;               // 离合区位移
	short  slopeType;           // 离合区位移描述的是主轴还是从轴
	short  dir;
	short  startMode;
	short  startPrmType;        // 启动参数中描述的信息时主轴还是从轴
	double startPrm[6];

	double deltaSlaveEven;      // 主从比的变化量
	double deltaSlope;          // 主从比变化时的离合区位移
	short  deltaSlopeType;      // 主从比变化时的离合区位移描述的是主轴还是从轴
	short  deltaLoop;           // 主从比变化次数，0：无限循环
	short  reserve2[10];
}TGearParameter;

#define MOVE_SYNCH_MPG_RESERVE_MASTER_VEL_LIMIT     (0) // 设置主轴的最大限速，单位pusle/ms
typedef struct MpgParameter
{
    double masterEven;          //传动比主轴，单位是pulse
    double slaveEven;            //传动比从轴，单位是mm
    double masterFilterTime;// 主轴滤波时间
    double sampleTime;    // 主轴采样时间
    double stopWaitTime;   // 停止等待时间
    double acceleration;    // 从轴运动加速度
    double deceleration;    // 从轴运动减速度
    double maxVel;        // 从轴运动最大速度
    short reserve[28];
}TMpgParameter;

typedef struct FollowParameter
{
    double masterEven;          //传动比主轴，单位是pulse
    double slaveEven;            //传动比从轴，单位是mm
    short reserve[52];
}TFollowParameter;

typedef struct GearPosParameter
{
    double pos;            // 从轴终点位置
    double masterLength;   // 主轴位移
    short reserve[52];
}TGearPosParameter;

typedef union MoveSynchronizationUnion
{
    TGearParameter gear;
    TMpgParameter mpg;
    TFollowParameter follow;
    TGearPosParameter gearPos;
    short reserve[60];
}TMoveSynchronizationUnion;

typedef struct MoveSynchronizationPrm
{
    short mode;          // 同步运动模式
    short enable;        // 是否使能同步运动功能
    short softDirCheck;  //0:关闭方向趋势判断功能 1：开启方向趋势判断功能
    short reserve;
    TMoveSynchronizationUnion prm;
}TMoveSynchronizationPrm;

typedef struct MoveSynchronizationInfo
{
    short info1[8];
    long info2[8];
    double info3[8];
}TMoveSynchronizationInfo;

GT_API GTN_MoveSynchronization(short core, TProfileItem slave, TProfileItem master, TMoveSynchronizationPrm* pPrm, TListInfo* pListInfo = NULL);
GT_API GTN_GetMoveSynchronizationInfo(short core, TProfileItem slave, TMoveSynchronizationInfo* pInfo);

//////////////////////////////////////////////////////////////////////////
// Cam功能相关指令
//---------------------------------------------------------
// Start Mode
//---------------------------------------------------------
#define MC_START_MODE_ABSOLUTE                      (0)
#define MC_START_MODE_RELATIVE                      (1)

#define MC_START_MODE_CLUTCH_TRAP_POS               (63)
#define MC_START_MODE_CLUTCH_TRAP_NEG               (64)
#define MC_START_MODE_CLUTCH_TRAP_CURRENT           (65)

//---------------------------------------------------------
// TableTrapPoint Type
//---------------------------------------------------------
#define CAM_POINT_TYPE_CLUTCH                      (-1)
#define CAM_POINT_TYPE_TRAP                        (20)
#define CAM_POINT_TYPE_TRAP_EVEN                   (21)

//---------------------------------------------------------
// CamOut Mode
//---------------------------------------------------------
#define CAM_OUT_MODE_STOP                           (0)
#define CAM_OUT_MODE_CONTINUOUS                     (1)

#define CAM_OUT_MODE_TO_POS                         (3)

//---------------------------------------------------------
// Ramp Type
//---------------------------------------------------------
#define RAMP_TYPE_TRAP                              (0)
#define RAMP_TYPE_JERK                              (2)


typedef struct CamTableTrapPoint
{
    short type;
    short pad1[3];

    double masterPos;                  // 主轴位置

    double slavePos;                   // 从轴位置
    double slaveVelRatio;              // 从轴和主轴速度比值

    double slaveVelRatioMax;           // 从轴和主轴速度比值最大值
    double slaveAccRatio;              // 从轴和主轴加速度比值
    double slaveDecRatio;              // 从轴和主轴减速度比值

    double percentAcc;                 // 加速段S曲线百分比
    double percentDec;                 // 减速段S曲线百分比

    double pad2[4];
} TCamTableTrapPoint;

typedef struct CamTableTrap
{
    unsigned long camTableId;          // 凸轮表标识
    unsigned long pad1;

    unsigned short periodic;           // 凸轮表周期执行
    unsigned short masterAbsolute;
    unsigned short slaveAbsolute;
    unsigned short pad2;

    double masterPosBegin;             // 凸轮表起点主轴位置
    double slavePosBegin;              // 凸轮表起点从轴位置
    double slaveVelRatioBegin;         // 凸轮表起点从轴和主轴的速度比值

    TCamTableTrapPoint* pPoint;        // 凸轮表数据首地址
    unsigned long pointCount;          // 凸轮表数据点数量
    unsigned long pad3;
} TCamTableTrap;

typedef struct CamTableTrapPrm
{
    unsigned short periodic;           // 凸轮表周期执行
    unsigned short masterAbsolute;
    unsigned short slaveAbsolute;
    unsigned short pad2;

    double masterPosBegin;             // 凸轮表起点主轴位置
    double slavePosBegin;              // 凸轮表起点从轴位置
    double slaveVelRatioBegin;         // 凸轮表起点从轴和主轴的速度比值

    short slavePosCalcMode;            // 从轴位置计算模式，0：设置数据点时给入，1：内部自动计算
    unsigned long camTableSpace;       // 凸轮表的占用空间数量
} TCamTableTrapPrm;

typedef struct CamInParameter
{
    unsigned short masterIndex;        // 主轴索引
    unsigned short masterValueSource;  // 主轴数据源

    unsigned short slaveIndex;         // 从轴索引
    unsigned short slaveType;          // 从轴类型，MC_PROFILE：单轴；MC_GROUP：group


    unsigned long camTableId;

    unsigned short pad1;               // 必须为0
    unsigned short bufferMode;
    unsigned short bufferCommandType;

    unsigned short startMode;          // 0:没有离合区

    unsigned short masterModulo;       // 0：线性轴，主轴位置在凸轮表内时才跟随
                                       // 1：旋转轴，主轴位置对凸轮表长度求余数，随时可以跟随

    unsigned short slaveModulo;        // 0：线性轴，直接使用从轴位置
                                       // 1：旋转轴，从轴位置对凸轮表长度求余数
    double masterModuloBegin;          // 主轴求余起点位置，目前只能为0
    double masterModuloLength;         // 主轴单圈长度

    double slaveModuloBegin;           // 从轴求余起点位置，目前只能为0
    double slaveModuloLength;          // 从轴单圈长度

    double masterOffset;
    double slaveOffset;

    double masterScaling;
    double slaveScaling;

    double masterStartDistance;        // startMode=poly5时，表示离合区长度，应大于0
    double masterSyncPosition;         // startMode=poly5时，表示同步的主轴位置

    double velRatioMax;                // 速度比最大值
    double accRatioMax;                // 加速度比值最大值
    double decRatioMax;                // 减速度比值最大值

    double pad2[8];                    // 必须为0

}TCamInParameter;

typedef struct CamOutParameter
{
    unsigned short slaveIndex;         // 从轴索引
    unsigned short slaveType;          // 从轴类型

    unsigned short mode;               // 0：减速到0；1：保持当前速度；3：停到指定位置
    unsigned short rampType;

    unsigned short pad1[3];
    unsigned short segCount;           // 当从轴为group，mode=3时有效，设置camout对应的插补段数

    double pos;                        // 目标位置，mode=3时有效
    double vel;                        // 目标速度，mode=3时有效
    double acc;                        // 加速度，  mode=3时有效
    double dec;                        // 减速度，  mode=1、3时有效
    double jerk;                       // 暂未实现，必须为0

    double velEnd;                     // 终点速度，mode=3时有效

    double pad2[8];                    // 保留参数，必须为0
} TCamOutParameter;

typedef struct CamStatus
{
    unsigned short execute;            // 执行状态
    unsigned short inSync;             // 同步状态
    unsigned short done;               // 完成状态
    unsigned short command;            // 当前正在执行的指令，1：CMD_CAM_IN；2：CMD_CAM_OUT

    unsigned long pad1;                // 保留参数
    unsigned long loopCount;           // 循环次数

    double masterPos;                  // 主轴位置
    double masterVel;                  // 主轴速度，是点速度，等于位置差分除以周期
    double masterAcc;                  // 主轴加速度，是点加速度
    double masterJerk;                 // 主轴加加速度，是点加加速度

    double slavePos;                   // 从轴位置
    double slaveVel;                   // 从轴速度，是点速度，等于位置差分除以周期
    double slaveAcc;                   // 从轴加速度，是点加速度
    double slaveJerk;                  // 从轴加加速度，是点加加速度

    double masterPosModulo;            // 主轴单圈位置
    double slavePosModulo;             // 从轴单圈位置

    double pad2[4];                    // 保留参数
} TCamStatus;


/**
 * @brief 清除所有凸轮表的信息
 * @param core 核号
 * @param pListInfo 指令流描述信息
 * @return
*/
GT_API GTN_CamTableClear(short core,TListInfo *pListInfo=NULL);

/**
 * @brief 加载凸轮表
 * @param core 核号
 * @param pCamTableTrap 凸轮表信息
 * @param pListInfo 指令流描述信息
 * @return
*/
GT_API GTN_CamTableLoadTrap(short core,const TCamTableTrap* pCamTableTrap,TListInfo *pListInfo=NULL);

/**
 * @brief 设置凸轮表的描述参数
 * @param core 核号
 * @param camTableId 凸轮表的标识
 * @param pPrm 凸轮表的描述参数
 * @param pListInfo 指令流描述信息
 * @return
*/
GT_API GTN_SetCamTableTrapPrm(short core,unsigned long camTableId,const TCamTableTrapPrm *pPrm,TListInfo *pListInfo=NULL);

/**
 * @brief 设置凸轮表的数据信息
 * @param core 核号
 * @param camTableId 凸轮表的标识
 * @param pPoint 凸轮表的数据段信息
 * @param pListInfo 指令流描述信息
 * @return
*/
GT_API GTN_CamTableTrapPoint(short core,unsigned long camTableId,const TCamTableTrapPoint *pPoint,TListInfo *pListInfo=NULL);

/**
 * @brief 凸轮表数据结束
 * @param core 核号
 * @param camTableId 凸轮表的标识
 * @param pListInfo 指令流描述信息
 * @return
*/
GT_API GTN_CamTableTrapPointEnd(short core,unsigned long camTableId,TListInfo *pListInfo=NULL);

/**
 * @brief 启动凸轮跟随
 * @param core 核号
 * @param pPrm 凸轮跟随的参数
 * @param pListInfo 指令流描述信息
 * @return
*/
GT_API GTN_CamIn(short core,TCamInParameter *pPrm,TListInfo *pListInfo=NULL);

/**
 * @brief 设置退出凸轮跟随的参数
 * @param core 核号
 * @param pPrm 退出凸轮跟随的参数
 * @param pListInfo 指令流描述信息
 * @return
*/
GT_API GTN_CamOut(short core,TCamOutParameter *pPrm,TListInfo *pListInfo=NULL);

/**
 * @brief 设置cam模块急停参数
 * @param core 核号
 * @param pPrm 急停参数
 * @param pListInfo 指令流描述信息
 * @return
*/
GT_API GTN_SetCamErrorStopPrm(short core,TCamOutParameter *pPrm,TListInfo *pListInfo=NULL);

/**
 * @brief 获取凸轮跟随状态
 * @param core 核号
 * @param slaveIndex 从轴索引，从1开始
 * @param slaveType 从轴类型
 * @param pStatus 凸轮跟随状态
 * @return
*/
GT_API GTN_GetCamStatus(short core,unsigned short slaveIndex,unsigned short slaveType,TCamStatus *pStatus);

typedef struct TransformOrthogonal
{
    short source;
    short enable;
    short x;
    short y;
    double theta;		// degree
} TTransformOrthogonal;

GT_API GT_SetTransformOrthogonal(short index, TTransformOrthogonal* pOrthogonal);
GT_API GT_GetTransformOrthogonal(short index, TTransformOrthogonal* pOrthogonal);
GT_API GT_GetTransformOrthogonalPosition(short index, double* pPositionX, double* pPositionY);

GT_API GTN_SetTransformOrthogonal(short core, short index, TTransformOrthogonal* pOrthogonal);
GT_API GTN_GetTransformOrthogonal(short core, short index, TTransformOrthogonal* pOrthogonal);
GT_API GTN_GetTransformOrthogonalPosition(short core, short index, double* pPositionX, double* pPositionY);

typedef struct TransformPerpendicularity
{
    short source;
    short enable;
    short x;
    short y;
    short z;
    double alpha;		// Z轴和XY平面的夹角
    double beta;		// Z轴在XY平面的投影和X轴的夹角
    double gama;		// XY轴之间的夹角
} TTransformPerpendicularity;

GT_API GTN_SetTransformPerpendicularity(short core, short index, TTransformPerpendicularity* pPerpendicularity);
GT_API GTN_GetTransformPerpendicularity(short core, short index, TTransformPerpendicularity* pPerpendicularity);
GT_API GTN_GetTransformPerpendicularityPosition(short core, short index, double* pPositionX, double* pPositionY, double* pPositionZ);

/*-----------------------------------------------------------*/
/* Comp                                                      */
/*-----------------------------------------------------------*/

#define COMP_TYPE_COORD_SYNC	(0)

typedef struct CoordTransformPrm
{
    double translateion[2];	//工件坐标系相对原始坐标系的平移
    double theta;	//工件坐标系相对原始坐标系的旋转
}TCoordTransformPrm;

typedef struct CoordSyncCompPrm
{
    short enable;	//是否使能坐标系转换功能
    short refType;	//参考类型，可以设置为MC_CRD、MC_GROUP、MC_PROFILE等
    short index;	//如果参考类型为MC_CRD，写入坐标系号
    short refIndex[2];	//参考轴号，指定坐标系中的某两个轴，或者profile的两个轴
    double offset[2];	//补偿轴X2、Y2零点相对参考坐标系X1、Y1零点的偏移
    TCoordTransformPrm refTrans;	//参考工件坐标系相对原坐标系的平移和旋转
    TCoordTransformPrm syncTrans;	//同步工件坐标系相对原坐标系的平移和旋转
    short reserve1[4];
    double reserve2[4];
}TCoordSyncCompPrm;

typedef struct PrfComp
{
    short type;	//补偿类型，例如同步补偿等
    short index;	//一级索引，例如同步补偿支持最多允许四套坐标系，该索引表示第几套同步补偿
    short subIndex;	//二级索引，例如第一套同步补偿的第一个轴
    short reserve1[5];
    double reserve2[4];
}TPrfComp;

GT_API GTN_PrfComp(short core, short profile, TPrfComp* pComp);
GT_API GTN_PrfCompEnable(short core, short profile, short enable, short enableType = 0);
GT_API GTN_BufPrfCompEnable(short core, short crd, short fifo, short profile, short enable, short enableType = 0);
GT_API GTN_BufPrfCompEnableEx(short core, short crd, short fifo, short profile, short enable, short enableType = 0);
GT_API GTN_SetCoordSyncCompPrm(short core, short index, TCoordSyncCompPrm* pPrm);
GT_API GTN_GetCoordSyncCompPrm(short core, short index, TCoordSyncCompPrm* pPrm);
GT_API GTN_GetCoordSyncCompValue(short core, short index, double x, double y, double* pCompX, double* pCompY);

/*-----------------------------------------------------------*/
/* Home                                                      */
/*-----------------------------------------------------------*/
#define HOME_STAGE_IDLE                           (0)
#define HOME_STAGE_START                          (1)
#define HOME_STAGE_ON_HOME_LIMIT_ESCAPE           (2)
#define HOME_STAGE_ON_HOME_ESCAPE                 (3)

#define HOME_STAGE_SEARCH_LIMIT                   (10)
#define HOME_STAGE_SEARCH_LIMIT_STOP              (11)

#define HOME_STAGE_SEARCH_LIMIT_ESCAPE            (13)

#define HOME_STAGE_SEARCH_LIMIT_RETURN            (15)
#define HOME_STAGE_SEARCH_LIMIT_RETURN_STOP       (16)

#define HOME_STAGE_SEARCH_HOME                    (20)

#define HOME_STAGE_SERCH_HOME_STOP                (22)
#define HOME_STAGE_SEARCH_HOME_OPPOSITE           (23)
#define HOME_STAGE_SEARCH_HOME_AGAIN              (24)
#define HOME_STAGE_SEARCH_HOME_RETURN             (25)

#define HOME_STAGE_SEARCH_INDEX                   (30)

#define HOME_STAGE_SEARCH_GPI                     (40)

#define HOME_STAGE_SEARCH_GPI_RETURN              (45)

#define HOME_STAGE_GO_HOME                        (80)

#define HOME_STAGE_END                            (100)

#define HOME_ERROR_NONE                           (0)
#define HOME_ERROR_NOT_TRAP_MODE                  (1)
#define HOME_ERROR_DISABLE                        (2)
#define HOME_ERROR_ALARM                          (3)
#define HOME_ERROR_STOP                           (4)
#define HOME_ERROR_STAGE                          (5)
#define HOME_ERROR_HOME_MODE                      (6)
#define HOME_ERROR_SET_CAPTURE_HOME               (7)
#define HOME_ERROR_NO_HOME                        (8)
#define HOME_ERROR_SET_CAPTURE_INDEX              (9)
#define HOME_ERROR_NO_INDEX                       (10)
#define HOME_ERROR_LIMIT                          (11)
#define HOME_ERROR_ESCAPE                         (12)

#define HOME_MODE_LIMIT                           (10)
#define HOME_MODE_LIMIT_HOME                      (11)
#define HOME_MODE_LIMIT_INDEX                     (12)
#define HOME_MODE_LIMIT_HOME_INDEX                (13)
#define HOME_MODE_LIMITPOS_INDEX		          (14)
#define HOME_MODE_HOME                            (20)

#define HOME_MODE_HOME_INDEX                      (22)

#define HOME_MODE_HOME_LEVEL                      (24)                               // 使用HOME电平回零(适用于没有HOME捕获的情况下)

#define HOME_MODE_INDEX                           (30)
#define HOME_MODE_FORCED_HOME                     (40)
#define HOME_MODE_FORCED_HOME_INDEX               (41)
#define HOME_MODE_DRIVER_HOME                     (42)
typedef struct HomePrm
{
    short mode;
    short moveDir;
    short indexDir;
    short edge;
    short triggerIndex;
    short pad1[3];
    double velHigh;
    double velLow;
    double acc;
    double dec;
    short smoothTime;
    short pad2[3];
    long homeOffset;
    long searchHomeDistance;
    long searchIndexDistance;
    long escapeStep;
    long pad3[2];
} THomePrm;

typedef struct HomeStatus
{
    short run;
    short stage;
    short error;
    short pad1;
    long capturePos;
    long targetPos;
} THomeStatus;

GT_API GT_GoHome(short axis, THomePrm* pHomePrm);
GT_API GT_GetHomePrm(short profile, THomePrm* pHomePrm);
GT_API GT_GetHomeStatus(short profile, THomeStatus* pHomeStatus);

GT_API GTN_GoHome(short core, short axis, THomePrm* pHomePrm);
GT_API GTN_GetHomePrm(short core, short axis, THomePrm* pHomePrm);
GT_API GTN_GetHomeStatus(short core, short axis, THomeStatus* pHomeStatus);

GT_API GT_HandwheelInit(short mode = 0);
GT_API GT_SetHandwheelStopDec(short slave, double decSmoothStop, double decAbruptStop);
GT_API GT_StartHandwheel(short slave, short master, short masterEven, short slaveEven, short intervalTime, double acc, double dec, double vel, short stopWaitTime);
GT_API GT_EndHandwheel(short slave);

GT_API GTN_HandwheelInit(short core, short mode = 0);
GT_API GTN_SetHandwheelStopDec(short core, short slave, double decSmoothStop, double decAbruptStop);
GT_API GTN_StartHandwheel(short core, short slave, short master, short masterEven, short slaveEven, short intervalTime, double acc, double dec, double vel, short stopWaitTime);
GT_API GTN_EndHandwheel(short core, short slave);

/*-----------------------------------------------------------*/
/* PLC                                                       */
/*-----------------------------------------------------------*/
#define PLC_THREAD_MAX					(32)
#define PLC_PAGE_MAX					(32)
#define PLC_LOCAL_VAR_MAX				(1024)
#define PLC_ACCESS_VAR_COUNT_MAX        (8)

#define PLC_TIMER_TT					(0)
#define PLC_TIMER_TF					(1)
#define PLC_TIMER_TTF					(2)

#define PLC_COUNTER_EQ					(0)
#define PLC_COUNTER_LE					(1)
#define PLC_COUNTER_GE					(2)

#define PLC_COUNTER_EDGE_UP				(0)
#define PLC_COUNTER_EDGE_DOWN			(1)
#define PLC_COUNTER_EDGE_UP_DOWN		(2)

#define PLC_FLANK_UP					(0)
#define PLC_FLANK_DOWN					(1)
#define PLC_FLANK_UP_DOWN				(2)

typedef enum PlcBind
{
    PLC_BIND_NONE,
    PLC_BIND_DI,
    PLC_BIND_DO,
    PLC_BIND_TIMER,
    PLC_BIND_COUNTER,
    PLC_BIND_FLANK,
    PLC_BIND_SRFF,
} EPlcBind;

typedef struct VarInfo
{
    short id;
    short dataType;
    char  name[32];
} TVarInfo;

typedef struct BindDi
{
    short diType;
    short index;
    short reverse;
} TBindDi;

typedef struct BindDo
{
    short doType;
    short index;
    short reverse;
} TBindDo;

typedef struct BindTimer
{
    short timerType;
    long delay;
    short inputVarId;
} TBindTimer;

typedef struct BindCounter
{
    short counterType;
    short edge;
    long init;
    long target;
    long begin;
    long end;
    short dir;
    long unit;
    short inputVarId;
    short resetVarId;
} TBindCounter;

typedef struct BindFlank
{
    short flankType;
    short inputVarId;
} TBindFlank;

typedef struct BindSrff
{
    short setVarId;
    short resetVarId;
} TBindSrff;

typedef struct CompileInfo
{
    char* pFileName;
    short* pLineNo;
    char* pMessage;
} TCompileInfo;

typedef struct ThreadSts
{
    short  run;
    short  error;
    double result;
    short  line;
} TThreadSts;

GT_API GT_Compile(char* pFileName, TCompileInfo* pWrongInfo);
GT_API GT_Download(char* pFileName);
GT_API GT_GetFunId(char* pFunName, short* pFunId);
GT_API GT_GetVarId(char* pFunName, char* pVarName, TVarInfo* pVarInfo);
GT_API GT_Bind(short thread, short funId, short page);
GT_API GT_RunThread(short thread);
GT_API GT_RunThreadPeriod(short thread, short ms, short priority = 4);
GT_API GT_StopThread(short thread);
GT_API GT_PauseThread(short thread);
GT_API GT_GetThreadSts(short thread, TThreadSts* pThreadSts);
GT_API GT_GetThreadTime(short thread, short* pPeriod, double* pExecuteTime, double* pExecuteTimeMax);
GT_API GT_SetVarValue(short page, TVarInfo* pVarInfo, double* pValue, short count = 1);
GT_API GT_GetVarValue(short page, TVarInfo* pVarInfo, double* pValue, short count = 1);

GT_API GT_UnbindVar(short thread);
GT_API GT_BindDi(short thread, TVarInfo* pVarInfo, TBindDi* pBindDi);
GT_API GT_BindDo(short thread, TVarInfo* pVarInfo, TBindDo* pBindDo);
GT_API GT_BindTimer(short thread, TVarInfo* pVarInfo, TBindTimer* pBindTimer);
GT_API GT_BindCounter(short thread, TVarInfo* pVarInfo, TBindCounter* pBindCounter);
GT_API GT_BindFlank(short thread, TVarInfo* pVarInfo, TBindFlank* pBindFlank);
GT_API GT_BindSrff(short thread, TVarInfo* pVarInfo, TBindSrff* pBindSrff);

GT_API GT_GetBindDi(TVarInfo* pVarInfo, TBindDi* pBindDi);
GT_API GT_GetBindDo(TVarInfo* pVarInfo, TBindDo* pBindDo);
GT_API GT_GetBindTimer(TVarInfo* pVarInfo, TBindTimer* pBindTimer, long* pCount);
GT_API GT_GetBindCounter(TVarInfo* pVarInfo, TBindCounter* pBindCounter, long* pUnitCount, long* pCount);
GT_API GT_GetBindFlank(TVarInfo* pVarInfo, TBindFlank* pBindFlank);
GT_API GT_GetBindSrff(TVarInfo* pVarInfo, TBindSrff* pBindSrff);

GT_API GTN_Compile(char* pFileName, TCompileInfo* pWrongInfo);
GT_API GTN_Download(short core, char* pFileName);
GT_API GTN_GetFunId(char* pFunName, short* pFunId);
GT_API GTN_GetVarId(char* pFunName, char* pVarName, TVarInfo* pVarInfo);
GT_API GTN_Bind(short core, short thread, short funId, short page);
GT_API GTN_RunThread(short core, short thread);
GT_API GTN_RunThreadPeriod(short core, short thread, short ms, short priority = 4);
GT_API GTN_StepThread(short core, short thread);
GT_API GTN_StopThread(short core, short thread);
GT_API GTN_PauseThread(short core, short thread);
GT_API GTN_GetThreadSts(short core, short thread, TThreadSts* pThreadSts);
GT_API GTN_GetThreadTime(short core, short thread, short* pPeriod, double* pExecuteTime, double* pExecuteTimeMax);
GT_API GTN_RunThreadToBreakpoint(short core, short thread, short line);
GT_API GTN_SetVarValue(short core, short page, TVarInfo* pVarInfo, double* pValue, short count = 1);
GT_API GTN_GetVarValue(short core, short page, TVarInfo* pVarInfo, double* pValue, short count = 1);

GT_API GTN_UnbindVar(short core, short thread);
GT_API GTN_BindDi(short core, short thread, TVarInfo* pVarInfo, TBindDi* pBindDi);
GT_API GTN_BindDo(short core, short thread, TVarInfo* pVarInfo, TBindDo* pBindDo);
GT_API GTN_BindTimer(short core, short thread, TVarInfo* pVarInfo, TBindTimer* pBindTimer);
GT_API GTN_BindCounter(short core, short thread, TVarInfo* pVarInfo, TBindCounter* pBindCounter);
GT_API GTN_BindFlank(short core, short thread, TVarInfo* pVarInfo, TBindFlank* pBindFlank);
GT_API GTN_BindSrff(short core, short thread, TVarInfo* pVarInfo, TBindSrff* pBindSrff);

GT_API GTN_GetBindDi(short core, TVarInfo* pVarInfo, TBindDi* pBindDi);
GT_API GTN_GetBindDo(short core, TVarInfo* pVarInfo, TBindDo* pBindDo);
GT_API GTN_GetBindTimer(short core, TVarInfo* pVarInfo, TBindTimer* pBindTimer, long* pCount);
GT_API GTN_GetBindCounter(short core, TVarInfo* pVarInfo, TBindCounter* pBindCounter, long* pUnitCount, long* pCount);
GT_API GTN_GetBindFlank(short core, TVarInfo* pVarInfo, TBindFlank* pBindFlank);
GT_API GTN_GetBindSrff(short core, TVarInfo* pVarInfo, TBindSrff* pBindSrff);

GT_API GTN_GetBindDiCount(short core, short thread, short* pCount);
GT_API GTN_GetBindDoCount(short core, short thread, short* pCount);
GT_API GTN_GetBindTimerCount(short core, short thread, short* pCount);
GT_API GTN_GetBindCounterCount(short core, short thread, short* pCount);
GT_API GTN_GetBindFlankCount(short core, short thread, short* pCount);
GT_API GTN_GetBindSrffCount(short core, short thread, short* pCount);

GT_API GTN_GetBindDiInfo(short core, short thread, short index, short* pVar, TBindDi* pBindDi);
GT_API GTN_GetBindDoInfo(short core, short thread, short index, short* pVar, TBindDo* pBindDo);
GT_API GTN_GetBindTimerInfo(short core, short thread, short index, short* pVar, TBindTimer* pBindTimer);
GT_API GTN_GetBindCounterInfo(short core, short thread, short index, short* pVar, TBindCounter* pBindCounter);
GT_API GTN_GetBindFlankInfo(short core, short thread, short index, short* pVar, TBindFlank* pBindFlank);
GT_API GTN_GetBindSrffInfo(short core, short thread, short index, short* pVar, TBindSrff* pBindSrff);

typedef struct ThreadStatus
{
    short link;
    unsigned long  address;
    short size;
    unsigned long  page;
    short delay;
    short priority;
    short ptr;
    short status;
    short error;
    short result[4];
    short resultType;
    short breakpoint;
    short period;
    short count;
    short function;
} TThreadStatus;
GT_API GTN_GetThread(short core, short thread, TThreadStatus* pThread);

GT_API GTN_ClearPlc(short core);
GT_API GTN_LoadPlc(short core, short id, short returnType);
GT_API GTN_LoadPlcCommand(short core, short id, short count, short* pData);

GT_API GT_ClearPlc(void);
GT_API GT_LoadPlc(short id, short returnType);
GT_API GT_LoadPlcCommand(short id, short count, short* pData);
GT_API GT_StepThread(short thread);
GT_API GT_RunThreadToBreakpoint(short thread, short line);
GT_API GT_GetThread(short thread, TThreadStatus* pThread);


/*-----------------------------------------------------------*/
/* Interpolation                                             */
/*-----------------------------------------------------------*/
#define INTERPOLATION_AXIS_MAX                   (8)

#define CRD_OPERATION_DATA_EXT_MAX               (2)


#define CRD_BUFFER_MODE_DYNAMIC_DEFAULT          (0)
#define CRD_BUFFER_MODE_DYNAMIC_KEEP             (1)

#define CRD_BUFFER_MODE_STATIC_INPUT             (11)
#define CRD_BUFFER_MODE_STATIC_READY             (12)
#define CRD_BUFFER_MODE_STATIC_START             (13)

#define INTERPOLATION_CIRCLE_PLAT_XY             (0)
#define INTERPOLATION_CIRCLE_PLAT_YZ             (1)
#define INTERPOLATION_CIRCLE_PLAT_ZX             (2)

#define INTERPOLATION_HELIX_CIRCLE_XY_LINE_Z     (0)
#define INTERPOLATION_HELIX_CIRCLE_YZ_LINE_X     (1)
#define INTERPOLATION_HELIX_CIRCLE_ZX_LINE_Y     (2)

#define INTERPOLATION_CIRCLE_DIR_CW              (0)
#define INTERPOLATION_CIRCLE_DIR_CCW             (1)

#define CRD_AXIS_X                               (1) // 插补坐标系x轴
#define CRD_AXIS_Y                               (2) // 插补坐标系y轴
#define CRD_AXIS_Z                               (3) // 插补坐标系z轴
#define CRD_AXIS_A                               (4) // 插补坐标系a轴
#define CRD_AXIS_C                               (5) // 插补坐标系c轴
#define CRD_AXIS_U                               (6) // 插补坐标系u轴
#define CRD_AXIS_V                               (7) // 插补坐标系v轴
#define CRD_AXIS_W                               (8) // 插补坐标系w轴

typedef struct CrdPrm
{
    short dimension;
    short profile[8];
    double synVelMax;
    double synAccMax;
    short evenTime;
    short setOriginFlag;
    long originPos[8];
}TCrdPrm;

typedef struct CrdBufOperation
{
    short flag;
    unsigned short delay;
    short doType;
    unsigned short doMask;
    unsigned short doValue;
    unsigned short dataExt[CRD_OPERATION_DATA_EXT_MAX];
}TCrdBufOperation;

typedef struct CrdData
{
    short motionType;
    short circlePlat;
    long pos[INTERPOLATION_AXIS_MAX];
    double radius;
    short circleDir;
    double center[3];
    double vel;
    double acc;
    short velEndZero;
    TCrdBufOperation operation;

    double cos[INTERPOLATION_AXIS_MAX];
    double velEnd;
    double velEndAdjust;
    double r;
}TCrdData;

typedef struct CrdTime
{
    double time;
    long segmentUsed;
    long segmentHead;
    long segmentTail;
} TCrdTime;

typedef struct BufFollowMaster
{
    short crdAxis;
    short masterIndex;
    short masterType;
} TBufFollowMaster;

typedef struct BufFollowEventCross
{
    long masterPos;
    long pad;
} TBufFollowEventCross;

typedef struct BufFollowEventTrigger
{
    short triggerIndex;
    long triggerOffset;
    long pad;
} TBufFollowEventTrigger;

typedef struct CrdFollowPrm
{
    double velRatioMax;
    double accRatioMax;
    long masterLead;
    long masterEven;
    long slaveEven;
    short dir;
    short smoothPercent;
    short synchAlign;
} TCrdFollowPrm;

typedef struct CrdFollowStatus
{
    short stage;
    double slavePos;
    double slaveVel;
    long masterFrameWidth;
    long masterFrameIndex;
    unsigned long loopCount;
} TCrdFollowStatus;

GT_API GT_SetCrdPrm(short crd, TCrdPrm* pCrdPrm);
GT_API GT_GetCrdPrm(short crd, TCrdPrm* pCrdPrm);
GT_API GT_CrdSpace(short crd, long* pSpace, short fifo = 0);
GT_API GT_CrdData(short crd, TCrdData* pCrdData, short fifo = 0);

GT_API GT_LnXY(short crd, long x, long y, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GT_LnXYOverride2(short crd, long x, long y, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GT_LnXYWN(short crd, long x, long y, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);
GT_API GT_LnXYOverride2WN(short crd, long x, long y, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);

GT_API GT_LnXYG0(short crd, long x, long y, double synVel, double synAcc, short fifo = 0);
GT_API GT_LnXYG0Override2(short crd, long x, long y, double synVel, double synAcc, short fifo = 0);
GT_API GT_LnXYG0WN(short crd, long x, long y, double synVel, double synAcc, long segNum = 0, short fifo = 0);
GT_API GT_LnXYG0Override2WN(short crd, long x, long y, double synVel, double synAcc, long segNum = 0, short fifo = 0);


GT_API GT_LnXYZ(short crd, long x, long y, long z, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GT_LnXYZOverride2(short crd, long x, long y, long z, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GT_LnXYZWN(short crd, long x, long y, long z, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);
GT_API GT_LnXYZOverride2WN(short crd, long x, long y, long z, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);

GT_API GT_LnXYZG0(short crd, long x, long y, long z, double synVel, double synAcc, short fifo = 0);
GT_API GT_LnXYZG0Override2(short crd, long x, long y, long z, double synVel, double synAcc, short fifo = 0);
GT_API GT_LnXYZG0WN(short crd, long x, long y, long z, double synVel, double synAcc, long segNum = 0, short fifo = 0);
GT_API GT_LnXYZG0Override2WN(short crd, long x, long y, long z, double synVel, double synAcc, long segNum = 0, short fifo = 0);

GT_API GT_LnXYZA(short crd, long x, long y, long z, long a, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GT_LnXYZAOverride2(short crd, long x, long y, long z, long a, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GT_LnXYZAWN(short crd, long x, long y, long z, long a, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);
GT_API GT_LnXYZAOverride2WN(short crd, long x, long y, long z, long a, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);

GT_API GT_LnXYZAG0(short crd, long x, long y, long z, long a, double synVel, double synAcc, short fifo = 0);
GT_API GT_LnXYZAG0Override2(short crd, long x, long y, long z, long a, double synVel, double synAcc, short fifo = 0);
GT_API GT_LnXYZAG0WN(short crd, long x, long y, long z, long a, double synVel, double synAcc, long segNum = 0, short fifo = 0);
GT_API GT_LnXYZAG0Override2WN(short crd, long x, long y, long z, long a, double synVel, double synAcc, long segNum = 0, short fifo = 0);

GT_API GT_LnXYZACUVW(short crd, long* pPos, short posMask, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GT_LnXYZACUVWWN(short crd, long* pPos, short posMask, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);
GT_API GT_LnXYZACUVWOverride2(short crd, long* pPos, short posMask, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GT_LnXYZACUVWOverride2WN(short crd, long* pPos, short posMask, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);

GT_API GT_ArcXYR(short crd, long x, long y, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GT_ArcXYROverride2(short crd, long x, long y, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GT_ArcXYRWN(short crd, long x, long y, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);
GT_API GT_ArcXYROverride2WN(short crd, long x, long y, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);

GT_API GT_ArcXYC(short crd, long x, long y, double xCenter, double yCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GT_ArcXYCOverride2(short crd, long x, long y, double xCenter, double yCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GT_ArcXYCWN(short crd, long x, long y, double xCenter, double yCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);
GT_API GT_ArcXYCOverride2WN(short crd, long x, long y, double xCenter, double yCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);

GT_API GT_ArcYZR(short crd, long y, long z, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GT_ArcYZROverride2(short crd, long y, long z, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GT_ArcYZRWN(short crd, long y, long z, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);
GT_API GT_ArcYZROverride2WN(short crd, long y, long z, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);

GT_API GT_ArcYZC(short crd, long y, long z, double yCenter, double zCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GT_ArcYZCOverride2(short crd, long y, long z, double yCenter, double zCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GT_ArcYZCWN(short crd, long y, long z, double yCenter, double zCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);
GT_API GT_ArcYZCOverride2WN(short crd, long y, long z, double yCenter, double zCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);

GT_API GT_ArcZXR(short crd, long z, long x, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GT_ArcZXROverride2(short crd, long z, long x, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GT_ArcZXRWN(short crd, long z, long x, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);
GT_API GT_ArcZXROverride2WN(short crd, long z, long x, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);

GT_API GT_ArcZXC(short crd, long z, long x, double zCenter, double xCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GT_ArcZXCOverride2(short crd, long z, long x, double zCenter, double xCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GT_ArcZXCWN(short crd, long z, long x, double zCenter, double xCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);
GT_API GT_ArcZXCOverride2WN(short crd, long z, long x, double zCenter, double xCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);


GT_API GT_ArcXYZ(short crd, long x, long y, long z, double interX, double interY, double interZ, double synVel, double synAcc, double velEnd, short fifo = 0);
GT_API GT_ArcXYZWN(short crd, long x, long y, long z, double interX, double interY, double interZ, double synVel, double synAcc, double velEnd, long segNum, short fifo = 0);
GT_API GT_ArcXYZOverride2WN(short crd, long x, long y, long z, double interX, double interY, double interZ, double synVel, double synAcc, double velEnd, long segNum, short fifo = 0);


GT_API GT_HelixXYRZ(short crd, long x, long y, long z, double radius, short circleDir, double synVel, double synAcc, double velEnd, short fifo = 0);
GT_API GT_HelixXYRZOverride2(short crd, long x, long y, long z, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GT_HelixXYRZWN(short crd, long x, long y, long z, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);
GT_API GT_HelixXYRZOverride2WN(short crd, long x, long y, long z, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);

GT_API GT_HelixXYCZ(short crd, long x, long y, long z, double xCenter, double yCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GT_HelixXYCZOverride2(short crd, long x, long y, long z, double xCenter, double yCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GT_HelixXYCZWN(short crd, long x, long y, long z, double xCenter, double yCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);
GT_API GT_HelixXYCZOverride2WN(short crd, long x, long y, long z, double xCenter, double yCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);

GT_API GT_HelixYZRX(short crd, long x, long y, long z, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GT_HelixYZRXOverride2(short crd, long x, long y, long z, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GT_HelixYZRXWN(short crd, long x, long y, long z, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);
GT_API GT_HelixYZRXOverride2WN(short crd, long x, long y, long z, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);

GT_API GT_HelixYZCX(short crd, long x, long y, long z, double yCenter, double zCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GT_HelixYZCXOverride2(short crd, long x, long y, long z, double yCenter, double zCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GT_HelixYZCXWN(short crd, long x, long y, long z, double yCenter, double zCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);
GT_API GT_HelixYZCXOverride2WN(short crd, long x, long y, long z, double yCenter, double zCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);

GT_API GT_HelixZXRY(short crd, long x, long y, long z, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GT_HelixZXRYOverride2(short crd, long x, long y, long z, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GT_HelixZXRYWN(short crd, long x, long y, long z, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);
GT_API GT_HelixZXRYOverride2WN(short crd, long x, long y, long z, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);

GT_API GT_HelixZXCY(short crd, long x, long y, long z, double zCenter, double xCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GT_HelixZXCYOverride2(short crd, long x, long y, long z, double zCenter, double xCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GT_HelixZXCYWN(short crd, long x, long y, long z, double zCenter, double xCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);
GT_API GT_HelixZXCYOverride2WN(short crd, long x, long y, long z, double zCenter, double xCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);

GT_API GT_BufIO(short crd, unsigned short doType, unsigned short doMask, unsigned short doValue, short fifo = 0);
GT_API GT_BufDelay(short crd, unsigned short delayTime, short fifo = 0);
GT_API GT_BufDA(short crd, short chn, short daValue, short fifo = 0);
GT_API GT_BufLmtsOn(short crd, short axis, short limitType, short fifo = 0);
GT_API GT_BufLmtsOff(short crd, short axis, short limitType, short fifo = 0);
GT_API GT_BufSetStopIo(short crd, short axis, short stopType, short inputType, short inputIndex, short fifo = 0);
GT_API GT_BufMove(short crd, short moveAxis, long pos, double vel, double acc, short modal, short fifo = 0);
GT_API GT_BufGear(short crd, short gearAxis, long pos, short fifo = 0);
GT_API GT_BufGearPercent(short crd, short gearAxis, long pos, short accPercent, short decPercent, short fifo = 0);
GT_API GT_BufStopMotion(short crd, short fifo = 0);
GT_API GT_BufSetVarValue(short crd, short pageId, TVarInfo* pVarInfo, double value, short fifo = 0);
GT_API GT_BufJumpNextSeg(short crd, short axis, short limitType, short fifo = 0);
GT_API GT_BufSynchPrfPos(short crd, short encoder, short profile, short fifo = 0);
GT_API GT_BufVirtualToActual(short crd, short fifo = 0);
GT_API GT_CrdStart(short mask, short option);
GT_API GT_CrdStartStep(short mask, short option);
GT_API GT_CrdStepMode(short mask, short option);
GT_API GT_SetOverride(short crd, double synVelRatio);
GT_API GT_SetOverride2(short crd, double synVelRatio);
GT_API GT_InitLookAhead(short crd, short fifo, double T, double accMax, short n, TCrdData* pLookAheadBuf);
GT_API GT_GetLookAheadSpace(short crd, long* pSpace, short fifo = 0);
GT_API GT_GetLookAheadSegCount(short crd, long* pSegCount, short fifo = 0);
GT_API GT_CrdClear(short crd, short fifo);
GT_API GT_CrdStatus(short crd, short* pRun, long* pSegment, short fifo = 0);
GT_API GT_SetUserSegNum(short crd, long segNum, short fifo = 0);
GT_API GT_GetUserSegNum(short crd, long* pSegment, short fifo = 0);
GT_API GT_GetUserSegNumWN(short crd, long* pSegment, short fifo = 0);
GT_API GT_GetRemainderSegNum(short crd, long* pSegment, short fifo = 0);
GT_API GT_SetCrdStopDec(short crd, double decSmoothStop, double decAbruptStop);
GT_API GT_GetCrdStopDec(short crd, double* pDecSmoothStop, double* pDecAbruptStop);
GT_API GT_SetCrdLmtStopMode(short crd, short lmtStopMode);
GT_API GT_GetCrdLmtStopMode(short crd, short* pLmtStopMode);
GT_API GT_GetUserTargetVel(short crd, double* pTargetVel);
GT_API GT_GetSegTargetPos(short crd, long* pTargetPos);
GT_API GT_GetCrdPos(short crd, double* pPos);
GT_API GT_GetCrdVel(short crd, double* pSynVel);
GT_API GT_BufLaserOn(short crd, short fifo = 0, short channel = 0);
GT_API GT_BufLaserOff(short crd, short fifo = 0, short channel = 0);
GT_API GT_BufLaserPrfCmd(short crd, double laserPower, short fifo = 0, short channel = 0);

GT_API GT_SetG0Mode(short crd, short mode);
GT_API GT_GetG0Mode(short crd, short* pMode);

GT_API GT_SetCrdMapBase(short crd, short base);
GT_API GT_GetCrdMapBase(short crd, short* pBase);
GT_API GT_SetCrdBufferMode(short crd, short bufferMode, short fifo = 0);
GT_API GT_GetCrdBufferMode(short crd, short* pBufferMode, short fifo = 0);
GT_API GT_GetCrdSegmentTime(short crd, long segmentIndex, double* pSegmentTime, long* pSegmentNumber, short fifo = 0);
GT_API GT_GetCrdTime(short crd, TCrdTime* pTime, short fifo = 0);

GT_API GT_BufFollowMaster(short crd, TBufFollowMaster* pBufFollowMaster, short fifo = 0);
GT_API GT_BufFollowEventCross(short crd, TBufFollowEventCross* pEventCross, short fifo = 0);
GT_API GT_BufFollowEventTrigger(short crd, TBufFollowEventTrigger* pEventTrigger, short fifo = 0);
GT_API GT_BufFollowStart(short crd, long masterSegment, long slaveSegment, long masterFrameWidth, short fifo = 0);
GT_API GT_BufFollowNext(short crd, long width, short fifo = 0);
GT_API GT_BufFollowReturn(short crd, double vel, double acc, short smoothPercent, short fifo = 0);
GT_API GT_BufLaserFollowMode(short crd, short source, short fifo, short channel, double startPower = 0);
GT_API GT_BufLaserFollowRatio(short crd, double ratio, double minPower, double maxPower, short fifo, short channel);
GT_API GT_BufLaserFollowOff(short crd, short fifo, short channel);
GT_API GT_BufLaserFollowSpline(short crd, short tableId, double minPower, double maxPower, short fifo, short channel);
GT_API GT_BufLaserFollowTable(short crd, short tableId, double minPower, double maxPower, short fifo, short channel);

// 插补用户指令停止模式设置
#define BUF_CMD_TYPE_ALL               (-1) // 所有支持设置停止模式的Buf指令，该类型仅支持设置为Default和Continue模式
#define BUF_CMD_TYPE_BUF_MOVE          (0)  // BufMove模态指令，该类型仅支持设置为Default和Continue模式
#define BUF_CMD_TYPE_BUF_GEAR          (1)  // BufGear指令，该类型仅支持设置为Default和Continue模式
#define BUF_CMD_TYPE_BUF_IO            (2)  // BufIo指令，该类型支持设置为Default、Continue和Hold模式
// 注意：在Continue和Hold模式下，只有指令GT_SetBufIoHoldValue设置doMask非0时才生效

#define BUF_CMD_STOP_MODE_DEFAULT      (0)  // 默认模式
#define BUF_CMD_STOP_MODE_CONTINUE     (1)  // 暂停恢复模式。注意：BufIo暂停时输出指令GT_SetBufIoHoldValue设置的do值，恢复运动时输出暂停前状态，doMask非0时生效
#define BUF_CMD_STOP_MODE_HOLD         (2)  // 暂停输出保持值，不恢复暂停前状态。仅BufIo支持，暂停插补运动时，输出指令GT_SetBufIoHoldValue设置的do值，不恢复，doMask非0时生效

// 设置插补运动暂停或异常停止时，do保持输出特定值
// crd：插补坐标系号，当前指令配置仅在对应坐标系生效
// doType：需要设置暂停保持输出的do类型，仅支持：MC_ENABLE、MC_CLEAR和MC_GPO
// doMask：暂停时保持输出特定值的do索引掩码，同时也表示恢复运动时需要恢复原有输出值的do索引掩码，按位指示，0：表示无效，1：表示有效
// doValue：暂停时保持输出的特定值，按位指示
// 注意：当doMask=0时，即使BufIO设置为continue或者hold模式，暂停保持输出和运动恢复输出都不生效。
GT_API GTN_SetBufIoHoldValue(short core, short crd, short doType, unsigned short doMask, unsigned short doValue);
GT_API GTN_GetBufIoHoldValue(short core, short crd, short doType, unsigned short* pDoMask, unsigned short* pDoValue);

// 设置插补用户段指令停止模式
// crd：插补坐标系号，当前指令配置仅在对应坐标系生效
// bufCmdType：插补用户段指令类型，仅支持：BUF_CMD_TYPE_ALL、BUF_CMD_TYPE_BUF_MOVE、BUF_CMD_TYPE_BUF_GEAR和BUF_CMD_TYPE_BUF_IO
// mode：停止模式
GT_API GTN_SetBufCmdStopMode(short core, short crd, short bufCmdType, short mode);
GT_API GTN_GetBufCmdStopMode(short core, short crd, short bufCmdType, short* pMode);

/**
 * @brief 设置插补打印log信息使能标志
 * @param core 核号，索引从1开始
 * @param crd 插补坐标系号，索引从1开始
 * @param enable 使能标志，0-不打印，1-打印
 * @return 非零，设置失败
*/
GT_API GTN_SetCrdPrintLogEnable(short core,short crd,short enable);

/**
 * @brief 获取插补打印log信息使能标志
 * @param core 核号，索引从1开始
 * @param crd 插补坐标系号，索引从1开始
 * @param pEnable 使能标志，0-不打印，1-打印
 * @return 非零，获取失败
*/
GT_API GTN_GetCrdPrintLogEnable(short core,short crd,short *pEnable);

GT_API GTN_SetCrdPrm(short core, short crd, TCrdPrm* pCrdPrm);
GT_API GTN_GetCrdPrm(short core, short crd, TCrdPrm* pCrdPrm);
GT_API GTN_CrdSpace(short core, short crd, long* pSpace, short fifo = 0);
GT_API GTN_CrdData(short core, short crd, TCrdData* pCrdData, short fifo = 0);
GT_API GTN_SetCrdLaserLevelDelay(short core,short crd,double highLevelDelay,double lowLevelDelay,short channel);

GT_API GTN_LnXY(short core, short crd, long x, long y, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GTN_LnXYOverride2(short core, short crd, long x, long y, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GTN_LnXYWN(short core, short crd, long x, long y, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);
GT_API GTN_LnXYOverride2WN(short core, short crd, long x, long y, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);

GT_API GTN_LnXYG0(short core, short crd, long x, long y, double synVel, double synAcc, short fifo = 0);
GT_API GTN_LnXYG0Override2(short core, short crd, long x, long y, double synVel, double synAcc, short fifo = 0);
GT_API GTN_LnXYG0WN(short core, short crd, long x, long y, double synVel, double synAcc, long segNum = 0, short fifo = 0);
GT_API GTN_LnXYG0Override2WN(short core, short crd, long x, long y, double synVel, double synAcc, long segNum = 0, short fifo = 0);

GT_API GTN_LnXYZ(short core, short crd, long x, long y, long z, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GTN_LnXYZOverride2(short core, short crd, long x, long y, long z, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GTN_LnXYZWN(short core, short crd, long x, long y, long z, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);
GT_API GTN_LnXYZOverride2WN(short core, short crd, long x, long y, long z, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);

GT_API GTN_LnXYZG0(short core, short crd, long x, long y, long z, double synVel, double synAcc, short fifo = 0);
GT_API GTN_LnXYZG0Override2(short core, short crd, long x, long y, long z, double synVel, double synAcc, short fifo = 0);
GT_API GTN_LnXYZG0WN(short core, short crd, long x, long y, long z, double synVel, double synAcc, long segNum = 0, short fifo = 0);
GT_API GTN_LnXYZG0Override2WN(short core, short crd, long x, long y, long z, double synVel, double synAcc, long segNum = 0, short fifo = 0);

GT_API GTN_LnXYZA(short core, short crd, long x, long y, long z, long a, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GTN_LnXYZAOverride2(short core, short crd, long x, long y, long z, long a, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GTN_LnXYZAWN(short core, short crd, long x, long y, long z, long a, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);
GT_API GTN_LnXYZAOverride2WN(short core, short crd, long x, long y, long z, long a, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);

GT_API GTN_LnXYZAG0(short core, short crd, long x, long y, long z, long a, double synVel, double synAcc, short fifo = 0);
GT_API GTN_LnXYZAG0Override2(short core, short crd, long x, long y, long z, long a, double synVel, double synAcc, short fifo = 0);
GT_API GTN_LnXYZAG0WN(short core, short crd, long x, long y, long z, long a, double synVel, double synAcc, long segNum = 0, short fifo = 0);
GT_API GTN_LnXYZAG0Override2WN(short core, short crd, long x, long y, long z, long a, double synVel, double synAcc, long segNum = 0, short fifo = 0);

GT_API GTN_LnXYZACUVW(short core, short crd, long* pPos, short posMask, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GTN_LnXYZACUVWOverride2(short core, short crd, long* pPos, short posMask, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GTN_LnXYZACUVWWN(short core, short crd, long* pPos, short posMask, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);
GT_API GTN_LnXYZACUVWOverride2WN(short core, short crd, long* pPos, short posMask, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);

GT_API GTN_LnXYZAC(short core, short crd, long* pPos, short posMask, double synVel, double synAcc, double velEnd, short fifo);
GT_API GTN_LnXYZACOverride2(short core, short crd, long* pPos, short posMask, double synVel, double synAcc, double velEnd, short fifo);
GT_API GTN_LnXYZACWN(short core, short crd, long* pPos, short posMask, double synVel, double synAcc, double velEnd, long segNum, short fifo);
GT_API GTN_LnXYZACOverride2WN(short core, short crd, long* pPos, short posMask, double synVel, double synAcc, double velEnd, long segNum, short fifo);

GT_API GTN_LnXYZACG0(short core, short crd, long* pPos, short posMask, double synVel, double synAcc, short fifo);
GT_API GTN_LnXYZACG0Override2(short core, short crd, long* pPos, short posMask, double synVel, double synAcc, short fifo);
GT_API GTN_LnXYZACG0WN(short core, short crd, long* pPos, short posMask, double synVel, double synAcc, long segNum, short fifo);
GT_API GTN_LnXYZACG0Override2WN(short core, short crd, long* pPos, short posMask, double synVel, double synAcc, long segNum, short fifo);
GT_API GTN_ArcXYR(short core, short crd, long x, long y, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GTN_ArcXYROverride2(short core, short crd, long x, long y, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GTN_ArcXYRWN(short core, short crd, long x, long y, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);
GT_API GTN_ArcXYROverride2WN(short core, short crd, long x, long y, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);

GT_API GTN_ArcXYC(short core, short crd, long x, long y, double xCenter, double yCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GTN_ArcXYCOverride2(short core, short crd, long x, long y, double xCenter, double yCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GTN_ArcXYCWN(short core, short crd, long x, long y, double xCenter, double yCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);
GT_API GTN_ArcXYCOverride2WN(short core, short crd, long x, long y, double xCenter, double yCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);

GT_API GTN_ArcYZR(short core, short crd, long y, long z, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GTN_ArcYZROverride2(short core, short crd, long y, long z, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GTN_ArcYZRWN(short core, short crd, long y, long z, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);
GT_API GTN_ArcYZROverride2WN(short core, short crd, long y, long z, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);

GT_API GTN_ArcYZC(short core, short crd, long y, long z, double yCenter, double zCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GTN_ArcYZCOverride2(short core, short crd, long y, long z, double yCenter, double zCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GTN_ArcYZCWN(short core, short crd, long y, long z, double yCenter, double zCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);
GT_API GTN_ArcYZCOverride2WN(short core, short crd, long y, long z, double yCenter, double zCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);

GT_API GTN_ArcZXR(short core, short crd, long z, long x, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GTN_ArcZXROverride2(short core, short crd, long z, long x, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GTN_ArcZXRWN(short core, short crd, long z, long x, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);
GT_API GTN_ArcZXROverride2WN(short core, short crd, long z, long x, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);

GT_API GTN_ArcZXC(short core, short crd, long z, long x, double zCenter, double xCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GTN_ArcZXCOverride2(short core, short crd, long z, long x, double zCenter, double xCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GTN_ArcZXCWN(short core, short crd, long z, long x, double zCenter, double xCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);
GT_API GTN_ArcZXCOverride2WN(short core, short crd, long z, long x, double zCenter, double xCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);

GT_API GTN_ArcXYZ(short core, short crd, long x, long y, long z, double interX, double interY, double interZ, double synVel, double synAcc, double velEnd, short fifo = 0);
GT_API GTN_ArcXYZWN(short core, short crd, long x, long y, long z, double interX, double interY, double interZ, double synVel, double synAcc, double velEnd, long segNum, short fifo = 0);
GT_API GTN_ArcXYZOverride2WN(short core, short crd, long x, long y, long z, double interX, double interY, double interZ, double synVel, double synAcc, double velEnd, long segNum, short fifo = 0);

GT_API GTN_HelixXYRZ(short core, short crd, long x, long y, long z, double radius, short circleDir, double synVel, double synAcc, double velEnd, short fifo = 0);
GT_API GTN_HelixXYRZOverride2(short core, short crd, long x, long y, long z, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GTN_HelixXYRZWN(short core, short crd, long x, long y, long z, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);
GT_API GTN_HelixXYRZOverride2WN(short core, short crd, long x, long y, long z, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);

GT_API GTN_HelixXYCZ(short core, short crd, long x, long y, long z, double xCenter, double yCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GTN_HelixXYCZOverride2(short core, short crd, long x, long y, long z, double xCenter, double yCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GTN_HelixXYCZWN(short core, short crd, long x, long y, long z, double xCenter, double yCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);
GT_API GTN_HelixXYCZOverride2WN(short core, short crd, long x, long y, long z, double xCenter, double yCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);

GT_API GTN_HelixYZRX(short core, short crd, long x, long y, long z, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GTN_HelixYZRXOverride2(short core, short crd, long x, long y, long z, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GTN_HelixYZRXWN(short core, short crd, long x, long y, long z, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);
GT_API GTN_HelixYZRXOverride2WN(short core, short crd, long x, long y, long z, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);

GT_API GTN_HelixYZCX(short core, short crd, long x, long y, long z, double yCenter, double zCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GTN_HelixYZCXOverride2(short core, short crd, long x, long y, long z, double yCenter, double zCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GTN_HelixYZCXWN(short core, short crd, long x, long y, long z, double yCenter, double zCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);
GT_API GTN_HelixYZCXOverride2WN(short core, short crd, long x, long y, long z, double yCenter, double zCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);

GT_API GTN_HelixZXRY(short core, short crd, long x, long y, long z, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GTN_HelixZXRYOverride2(short core, short crd, long x, long y, long z, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GTN_HelixZXRYWN(short core, short crd, long x, long y, long z, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);
GT_API GTN_HelixZXRYOverride2WN(short core, short crd, long x, long y, long z, double radius, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);

GT_API GTN_HelixZXCY(short core, short crd, long x, long y, long z, double zCenter, double xCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GTN_HelixZXCYOverride2(short core, short crd, long x, long y, long z, double zCenter, double xCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, short fifo = 0);
GT_API GTN_HelixZXCYWN(short core, short crd, long x, long y, long z, double zCenter, double xCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);
GT_API GTN_HelixZXCYOverride2WN(short core, short crd, long x, long y, long z, double zCenter, double xCenter, short circleDir, double synVel, double synAcc, double velEnd = 0, long segNum = 0, short fifo = 0);

GT_API GTN_HelixXYRMultiZ(short core, short crd, long* pPos, double radius, short circleDir, double synVel, double synAcc, double velEnd, long segNum, short override2, short fifo);
GT_API GTN_HelixYZRMultiX(short core, short crd, long* pPos, double radius, short circleDir, double synVel, double synAcc, double velEnd, long segNum, short override2, short fifo);
GT_API GTN_HelixZXRMultiY(short core, short crd, long* pPos, double radius, short circleDir, double synVel, double synAcc, double velEnd, long segNum, short override2, short fifo);

GT_API GTN_HelixXYCMultiZ(short core, short crd, long* pPos, double xCenter, double yCenter, short circleDir, double synVel, double synAcc, double velEnd, long segNum, short override2, short fifo);
GT_API GTN_HelixYZCMultiX(short core, short crd, long* pPos, double yCenter, double zCenter, short circleDir, double synVel, double synAcc, double velEnd, long segNum, short override2, short fifo);
GT_API GTN_HelixZXCMultiY(short core, short crd, long* pPos, double zCenter, double xCenter, short circleDir, double synVel, double synAcc, double velEnd, long segNum, short override2, short fifo);

GT_API GTN_HelixXYRMultiZEx(short core, short crd, double* pPos, double radius, short circleDir, double synVel, double synAcc, long segNum, short override2, short fifo);
GT_API GTN_HelixYZRMultiXEx(short core, short crd, double* pPos, double radius, short circleDir, double synVel, double synAcc, long segNum, short override2, short fifo);
GT_API GTN_HelixZXRMultiYEx(short core, short crd, double* pPos, double radius, short circleDir, double synVel, double synAcc, long segNum, short override2, short fifo);

GT_API GTN_HelixXYCMultiZEx(short core, short crd, double* pPos, double xCenter, double yCenter, short circleDir, double synVel, double synAcc, long segNum, short override2, short fifo);
GT_API GTN_HelixYZCMultiXEx(short core, short crd, double* pPos, double yCenter, double zCenter, short circleDir, double synVel, double synAcc, long segNum, short override2, short fifo);
GT_API GTN_HelixZXCMultiYEx(short core, short crd, double* pPos, double zCenter, double xCenter, short circleDir, double synVel, double synAcc, long segNum, short override2, short fifo);
GT_API GTN_BufIO(short core, short crd, unsigned short doType, unsigned short doMask, unsigned short doValue, short fifo = 0);
GT_API GTN_BufDelay(short core, short crd, unsigned short delayTime, short fifo = 0);
GT_API GTN_BufDA(short core, short crd, short chn, short daValue, short fifo = 0);
GT_API GTN_BufAuDA(short core, short crd, short chn, short daValue, short fifo = 0);
GT_API GTN_BufLmtsOn(short core, short crd, short axis, short limitType, short fifo = 0);
GT_API GTN_BufLmtsOff(short core, short crd, short axis, short limitType, short fifo = 0);
GT_API GTN_BufSetStopIo(short core, short crd, short axis, short stopType, short inputType, short inputIndex, short fifo = 0);
GT_API GTN_BufMove(short core, short crd, short moveAxis, long pos, double vel, double acc, short modal, short fifo = 0);
GT_API GTN_BufGear(short core, short crd, short gearAxis, long pos, short fifo = 0);
GT_API GTN_BufGearPercent(short core, short crd, short gearAxis, long pos, short accPercent, short decPercent, short fifo = 0);
GT_API GTN_BufStop(short core, short crd, long mask, long option, short fifo);
GT_API GTN_BufStopEx(short core, short crd, long mask, long option, short fifo);
GT_API GTN_BufMoveJog(short core, short crd, short moveAxis, double vel, double acc, short modal, short fifo);
GT_API GTN_BufMoveJogEx(short core, short crd, short moveAxis, double vel, double acc, short modal, short fifo);
GT_API GTN_BufStopMotion(short core, short crd, short fifo = 0);
GT_API GTN_BufSetVarValue(short core, short crd, short pageId, TVarInfo* pVarInfo, double value, short fifo = 0);
GT_API GTN_BufJumpNextSeg(short core, short crd, short axis, short limitType, short fifo = 0);
GT_API GTN_BufSynchPrfPos(short core, short crd, short encoder, short profile, short fifo = 0);
GT_API GTN_BufVirtualToActual(short core, short crd, short fifo = 0);
GT_API GTN_BufEnableDoBitPulse(short core, short crd, short doType, short doIndex, unsigned short highLevelTime, unsigned short lowLevelTime, long pulseNum, short firstLevel, short fifo);
GT_API GTN_BufDisableDoBitPulse(short core, short crd, short doType, short doIndex, short fifo);
GT_API GTN_BufTrend(short core, short crd, unsigned long trendSegNum, double trendDistance, double trendVelEnd, short fifo);
//插补缓存区扩展模块指令
GT_API GTN_BufExtIO(short core, short crd, unsigned short doIndex, unsigned short doMask, unsigned short doValue, short fifo);
GT_API GTN_BufExtDA(short core, short crd, short chn, short daValue, short fifo);
GT_API GTN_BufExtAoEx(short core, short crd, short aoIndex, double aoValue, short fifo);
GT_API GTN_BufExtDoBitEx(short core, short crd, unsigned short doIndex, unsigned short doValue, short fifo);
GT_API GTN_BufExtDoBit(short core, short crd, unsigned short doIndex, unsigned short doValue, short fifo);
GT_API GTN_BufExtAo(short core, short crd, short aoIndex, double aoValue, short fifo);
GT_API GTN_BufGearListBegin(short core, short crd, short gearAxis, double pos, short fifo);
GT_API GTN_BufGearListAdd(short core, short crd, float k, float percent, short fifo);
GT_API GTN_BufGearListEnd(short core, short crd, short fifo);
typedef struct BufMoveAbsPrm
{
    double pos;
    double vel;
    double acc;
    double velMax;
    double reserve1[4];
    short smoothTime;
    short enableRatio;
    short modal;
    short fifo;
    short dir;
    short reserve2[3];
    long segNum;
    long reserve3[3];
}TBufMoveAbsPrm;
GT_API GTN_BufMoveAbsoluteEx(short core, short crd, short moveAxis, TBufMoveAbsPrm* pPrm);
typedef struct CrdBlend
{
    short time;
    short aheadOfTime;
    short shape;
    short reverseSmooth;
    double velDelta;
} TCrdBlend;
GT_API GTN_BufSetCrdBlend(short core, short crd, TCrdBlend* pCrdBlend, short fifo);
GT_API GTN_BufCrdBlendOn(short core, short crd, short fifo);
GT_API GTN_BufCrdBlendOff(short core, short crd, short fifo);

/**
 * @brief 插补缓冲区设置轴的力矩，注意：只能在轴运动停止时设置力矩才生效
 * @param core 核号，索引从1开始
 * @param crd 插补坐标系号，索引从1开始
 * @param axis 轴号
 * @param prfTorque 设置力矩
 * @param fifo 插补坐标系缓冲区号，取值范围：[0,1]
 * @return
*/
GT_API GTN_BufSetPrfTorque(short core,short crd,short axis,short prfTorque,short fifo);

/**
 * @brief 插补前瞻缓冲区设置轴的力矩，注意：只能在轴运动停止时设置力矩才生效
 * @param core 核号，索引从1开始
 * @param crd 插补坐标系号，索引从1开始
 * @param axis 轴号
 * @param prfTorque 设置力矩
 * @param fifo 插补坐标系缓冲区号，取值范围：[0,1]
 * @return
*/
GT_API GTN_BufSetPrfTorqueEx(short core,short crd,short axis,short prfTorque,short fifo);

GT_API GTN_CrdStart(short core, short mask, short option);
GT_API GTN_CrdStartStep(short core, short mask, short option);
GT_API GTN_CrdStepMode(short core, short mask, short option);
GT_API GTN_SetOverride(short core, short crd, double synVelRatio);
GT_API GTN_SetOverride2(short core, short crd, double synVelRatio);
GT_API GTN_InitLookAhead(short core, short crd, short fifo, double T, double accMax, short n, TCrdData* pLookAheadBuf);
/**
 * @brief 设置老前瞻最小段长，在初始化老前瞻之后调用
 * @param core 核号
 * @param crd 坐标系号
 * @param fifo fifo号
 * @param minSegmentLength 最小段长，如果用户下压的数据段长小于此值，则会返回102，并且会把该段数据从内部fifo中剔除,单位：pulse
 * @return 返回值，1：未初始化老前瞻，0：成功
*/
GT_API GTN_SetLookAheadMinSegmentLength(short core, short crd, short fifo, double minSegmentLength);

GT_API GTN_GetLookAheadSpace(short core, short crd, long* pSpace, short fifo = 0);
GT_API GTN_GetLookAheadSegCount(short core, short crd, long* pSegCount, short fifo = 0);
GT_API GTN_CrdClear(short core, short crd, short fifo);
GT_API GTN_CrdStatus(short core, short crd, short* pRun, long* pSegment, short fifo = 0);
GT_API GTN_SetUserSegNum(short core, short crd, long segNum, short fifo = 0);
GT_API GTN_GetUserSegNum(short core, short crd, long* pSegment, short fifo = 0);
GT_API GTN_GetUserSegNumWN(short core, short crd, long* pSegment, short fifo = 0);
GT_API GTN_GetRemainderSegNum(short core, short crd, long* pSegment, short fifo = 0);
GT_API GTN_GetRemainderSegNumEx(short core, short crd, long* pSegment, short fifo);
GT_API GTN_SetCrdStopDec(short core, short crd, double decSmoothStop, double decAbruptStop);
GT_API GTN_GetCrdStopDec(short core, short crd, double* pDecSmoothStop, double* pDecAbruptStop);
GT_API GTN_SetCrdLmtStopMode(short core, short crd, short lmtStopMode);
GT_API GTN_GetCrdLmtStopMode(short core, short crd, short* pLmtStopMode);
GT_API GTN_GetUserTargetVel(short core, short crd, double* pTargetVel);
GT_API GTN_GetSegTargetPos(short core, short crd, long* pTargetPos);
GT_API GTN_GetCrdPos(short core, short crd, double* pPos);
GT_API GTN_GetCrdVel(short core, short crd, double* pSynVel);
GT_API GTN_SetCrdSingleMaxVel(short core, short crd, double* pMaxVel);
GT_API GTN_GetCrdSingleMaxVel(short core, short crd, double* pMaxVel);
#define DIMENSION_MAX                   (8)                 // 每个坐标系的最大维数
typedef struct CrdSegSyncInfo
{
    long segNumUserSet;
    long segNumInCmd;
    double synPrfVel;
    double prfPosCrd[DIMENSION_MAX];
}TCrdSegSyncInfo;
GT_API GTN_GetCrdSegSyncInfo(short core, short crd, short fifo, TCrdSegSyncInfo* pSegInfo);

typedef struct CrdStatusEx
{
    short runEmpty;
    long segUseCount;
    long segReceiveCount;
    short pad1[2];
    long pad2[2];
    double pad3[2];
}TCrdStatusEx;
GT_API GTN_CrdStatusEx(short core, short crd, TCrdStatusEx* pCrdStatus, short fifo);
GT_API GT_CrdStatusEx(short crd, TCrdStatusEx* pCrdStatus, short fifo);
typedef struct CrdSegmentLength
{
    double receive;
    double travel;
    double left;
} TCrdSegmentLength;
GT_API GTN_GetCrdSegmentLength(short core, short crd, TCrdSegmentLength* pSegmentLength, short fifo);
GT_API GTN_SetArcAllowError(short core, short crd, double error);
GT_API GTN_SetArcAllowErrorLa(short core, short crd, double error);
/*-----------------------------------------------------------*/
/* Laser Follow	                                            */
/*-----------------------------------------------------------*/
typedef struct LaserFollowPrm
{
    short laserFollowMode;
    double maxFrq;
    double minFrq;
    double maxDuty;
    double minDuty;
    short pad1[7];
    double pad2[8];
}TLaserFollowPrm;

typedef struct LaserOnOffCount
{
    unsigned long onCount;
    unsigned long offCount;
    unsigned long onCountInFpga;
    unsigned long offCountInFpga;
    unsigned long pad[4];
}TLaserOnOffCount;

typedef struct LaserFollowSpline2Prm
{
    double ratio;                                          // 系数
    double laserPowerFollowVelMax;                         // mm/s
    double laserPowerFollowFreqMax;                        // KHz
    double laserFollowPulseMax;                            // us
    double pad1[4];
}TLaserFollowSpline2Prm;

GT_API GTN_SetLaserFollowMode(short core, TLaserFollowPrm* pPrm, short channel);
GT_API GTN_LaserFollowOff(short core, short crd, short fifo, short channel);
GT_API GTN_SetLaserFollowTable(short core, short tableId, long n, double* pVel, double* pPower, short channel);
GT_API GTN_GetLaserFollowTable(short core, short tableId, long n, double* pVel, double* pPower, long* pCount, short channel);
GT_API GTN_SetLaserFollowSpline(short core, short tableId, long n, double* pX, double* pY, double beginValue, double endValue, short channel);
GT_API GTN_GetLaserFollowSpline(short core, short tableId, long n, double* pX, double* pY, double* pA, double* pB, double* pC, long* pCount, short channel);
GT_API GTN_GetLaserOnOffCount(short core, short channel, TLaserOnOffCount* pLaserCount);
GT_API GTN_ClearLaserOnOffCount(short core, short channel);
GT_API GT_LaserFollowOff(short crd, short fifo, short channel);
GT_API GTN_BufLaserOn(short core, short crd, short fifo = 0, short channel = 0);
GT_API GTN_BufLaserOff(short core, short crd, short fifo = 0, short channel = 0);
GT_API GTN_BufLaserPrfCmd(short core, short crd, double laserPower, short fifo = 0, short channel = 0);

GT_API GTN_BufLaserFollowMode(short core, short crd, short source, short fifo, short channel, double startPower);
GT_API GTN_BufLaserFollowRatio(short core, short crd, double ratio, double minPower, double maxPower, short fifo, short channel);
GT_API GTN_BufLaserFollowOff(short core, short crd, short fifo, short channel);
GT_API GTN_BufLaserFollowSpline(short core, short crd, short tableId, double minPower, double maxPower, short fifo, short channel);
GT_API GTN_BufLaserFollowTable(short core, short crd, short tableId, double minPower, double maxPower, short fifo, short channel);

GT_API GTN_BufFollowMaster(short core, short crd, TBufFollowMaster* pBufFollowMaster, short fifo = 0);
GT_API GTN_BufFollowEventCross(short core, short crd, TBufFollowEventCross* pEventCross, short fifo = 0);
GT_API GTN_BufFollowEventTrigger(short core, short crd, TBufFollowEventTrigger* pEventTrigger, short fifo = 0);
GT_API GTN_BufFollowStart(short core, short crd, long masterSegment, long slaveSegment, long masterFrameWidth, short fifo = 0);
GT_API GTN_BufFollowNext(short core, short crd, long width, short fifo = 0);
GT_API GTN_BufFollowReturn(short core, short crd, double vel, double acc, short smoothPercent, short fifo = 0);

GT_API GTN_BufLaserOnEx(short core, short crd, short fifo = 0, short channel = 0);
GT_API GTN_BufLaserOffEx(short core, short crd, short fifo = 0, short channel = 0);
GT_API GTN_BufLaserFollowModeEx(short core, short crd, short source, short fifo, short channel, double startPower = 0);
GT_API GTN_BufLaserFollowTableEx(short core, short crd, short tableId, double minPower, double maxPower, short fifo, short channel);
GT_API GTN_BufLaserFollowOffEx(short core, short crd, short fifo, short channel);
GT_API GTN_BufLaserFollowSpline2Ex(short core,short crd,TLaserFollowSpline2Prm *pPrm,short fifo,short channel);
GT_API GTN_BufLaserPrfCmdEx(short core, short crd, double laserPower, short fifo = 0, short channel = 0);
GT_API GTN_BufSetPulseWidthEx(short core, short crd, unsigned short width, short fifo = 0, short channel = 0);
GT_API GTN_BufLaserFollowRatioEx(short core, short crd, double ratio, double minPower, double maxPower, short fifo, short channel);

/**
 * @brief 前瞻缓冲区设置控制权
 * @param core 核号
 * @param crd 插补坐标系号
 * @param station 逻辑站号
 * @param dataType 硬件输出口类型，MC_HSO/MC_GPO
 * @param index 硬件输出口索引
 * @param permit 硬件输出口控制权
 * @param fifo 插补坐标系缓存区号
 * @return 0表示成功，非0表示失败
*/
GT_API GTN_BufSetTerminalPermitEx(short core,short crd,short station,short dataType,short index,short permit,short fifo=0);

typedef struct LaserFollowDuoTablePrm
{
    short frqTableId;
    short dutyTableId;
    double pad1[8];
    short pad2[8];
}TLaserFollowDuoTablePrm;
GT_API GTN_BufLaserFollowDuoTableEx(short core, short crd, TLaserFollowDuoTablePrm* pPrm, short fifo, short channel);

/*-----------------------------------------------------------*/
/* 激光能量波形控制输出                                      */
/*-----------------------------------------------------------*/

typedef struct LaserPowerWave
{
    short tableId;
    short channel;
    short enable;
    short loopCount;
    long intervalTime;
    short pad[2];
}TLaserPowerWave;

typedef struct LaserPowerWaveStatus
{
    short tableId;
    short enable;
    short loopIndex;
    short pad;
    double power;
}TLaserPowerWaveStatus;

/**
 * @brief 下压激光能量波形控制表
 * @param core 核号
 * @param tableId 表ID
 * @param pTime 波形控制时间轴
 * @param pPower 时间轴对应的激光能量，两个时间点之间的波形能量线性过渡变化
 * @param count 波形能量控制表中数据的个数，最大50个点
 * @param mode 保留参数，必须设置为0
 * @return 错误码
*/
GT_API GTN_SetLaserPowerWaveTable(short core, short tableId, long* pTime, double* pPower, short count, short mode);

/**
 * @brief 立即指令启动激光能量波形输出
 * @param core 核号
 * @param pLaserPowerWave 激光能量波形参数，主要选择波形表ID，激光通道，设置波形循环次数，循环间隔
 * @return 错误码
*/
GT_API GTN_SetLaserPowerWaveEnable(short core, TLaserPowerWave* pLaserPowerWave);

/**
 * @brief 插补缓冲区启动激光能量波形输出
 * @param core 核号
 * @param crd 坐标系号
 * @param pLaserPowerWave 激光能量波形参数，主要选择波形表ID，激光通道，设置波形循环次数，循环间隔
 * @param fifo 坐标系fifo号
 * @return
*/
GT_API GTN_BufSetLaserPowerWaveEnable(short core, short crd, TLaserPowerWave* pLaserPowerWave, short fifo);

/**
 * @brief 插补前瞻缓冲区启动激光能量波形输出
 * @param core 核号
 * @param crd 坐标系号
 * @param pLaserPowerWave 激光能量波形参数，主要选择波形表ID，激光通道，设置波形循环次数，循环间隔
 * @param fifo 坐标系fifo号
 * @return 错误码
*/
GT_API GTN_BufSetLaserPowerWaveEnableEx(short core, short crd, TLaserPowerWave* pLaserPowerWave, short fifo);

/**
 * @brief 立即指令读取激光能量波形输出状态
 * @param core 核号
 * @param laserChannel 激光通道号
 * @param pLaserPowerWaveStatus 波形状态输出参数
 * @return 错误码
*/
GT_API GTN_GetLaserPowerWaveStatus(short core, short laserChannel, TLaserPowerWaveStatus* pLaserPowerWaveStatus);

/*-----------------------------------------------------------*/
/* Crd					                                              */
/*-----------------------------------------------------------*/
GT_API GTN_SetG0Mode(short core, short crd, short mode);
GT_API GTN_GetG0Mode(short core, short crd, short* pMode);

GT_API GTN_SetCrdMapBase(short core, short crd, short base);
GT_API GTN_GetCrdMapBase(short core, short crd, short* pBase);
GT_API GTN_SetCrdBufferMode(short core, short crd, short bufferMode, short fifo = 0);
GT_API GTN_GetCrdBufferMode(short core, short crd, short* pBufferMode, short fifo = 0);
GT_API GTN_GetCrdSegmentTime(short core, short crd, long segmentIndex, double* pSegmentTime, long* pSegmentNumber, short fifo = 0);
GT_API GTN_GetCrdTime(short core, short crd, TCrdTime* pTime, short fifo = 0);
GT_API GTN_SetCrdFollowLoop(short core, short crd, unsigned long loop);
GT_API GTN_GetCrdFollowLoop(short core, short crd, unsigned long* pLoop);
GT_API GTN_SetCrdFollowPrm(short core, short crd, TCrdFollowPrm* pPrm);
GT_API GTN_GetCrdFollowPrm(short core, short crd, TCrdFollowPrm* pPrm);
GT_API GTN_GetCrdFollowStatus(short core, short crd, TCrdFollowStatus* pStatus);

GT_API GTN_SetCrdBlend(short core, short crd, TCrdBlend* pCrdBlend);
GT_API GTN_GetCrdBlend(short core, short crd, TCrdBlend* pCrdBlend);
GT_API GTN_CrdBlendOn(short core, short crd);
GT_API GTN_CrdBlendOff(short core, short crd);
GT_API GTN_CrdEnableVariableCircle(short core, short crd, short fifo);
GT_API GTN_CrdDisableVariableCircle(short core, short crd, short fifo);
GT_API GT_GetCmdCount(short crd, short* pResult, short fifo);

GT_API GTN_SetCrdContourErrorControl(short core, short crd, short enable, double percent);
GT_API GTN_GetCrdContourErrorControl(short core, short crd, short* pEnable, double* pPercent);

//////////////////////////////////////////////////////////////////////////
//Smooth
//////////////////////////////////////////////////////////////////////////
GT_API GT_SetCrdJerk(short crd, double jerkMax);
GT_API GT_GetCrdJerk(short crd, double* pJerkMax);
GT_API GT_SetCrdJerkTime(short crd, double jerkTime, double coef);
GT_API GT_GetCrdJerkTime(short crd, double* pJerkTime, double* pCoef);

GT_API GTN_SetCrdJerkMode(short core, short crd,double jerkMax, short mode);
GT_API GTN_GetCrdJerkMode(short core, short crd,double *pJerkMax, short *pMode);
GT_API GTN_SetCrdJerk(short core, short crd, double jerkMax);
GT_API GTN_GetCrdJerk(short core, short crd, double* pJerkMax);
GT_API GTN_SetCrdJerkTime(short core, short crd, double jerkTime, double coef);
GT_API GTN_GetCrdJerkTime(short core, short crd, double* pJerkTime, double* pCoef);
GT_API GTN_SetCrdJerkEx(short core, short crd, double jerkMax, double acc);
GT_API GTN_GetCrdJerkEx(short core, short crd, double* pJerkMax, double* pAcc);
GT_API GTN_SetCrdAccTime(short core, short crd, short time, short k);
GT_API GTN_GetCrdAccTime(short core, short crd, short* pTime, short* pK);
typedef struct CrdSmooth
{
    short percent;
    short accStartPercent;
    short decEndPercent;
    double reserve;
} TCrdSmooth;
GT_API GT_SetCrdSmooth(short crd, TCrdSmooth* pCrdSmooth);
GT_API GT_GetCrdSmooth(short crd, TCrdSmooth* pCrdSmooth);
GT_API GT_SetCrdSmoothTime(short crd, short smoothType, double* pPrm);
GT_API GT_GetCrdSmoothTime(short crd, short* pSmoothType, double* pPrm);
GT_API GTN_SetCrdSmooth(short core, short crd, TCrdSmooth* pCrdSmooth);
GT_API GTN_GetCrdSmooth(short core, short crd, TCrdSmooth* pCrdSmooth);
GT_API GTN_SetCrdSmoothTime(short core, short crd, short smoothType, double* pPrm);
GT_API GTN_GetCrdSmoothTime(short core, short crd, short* pSmoothType, double* pPrm);
GT_API GTN_SetAxisPrfVelSmooth(short core, short index, short type, short k);
GT_API GTN_GetAxisPrfVelSmooth(short core, short index, short* pType, short* pK);
GT_API GTN_SetAxisPrfVelSmoothEx(short core, short index, double w, double t, double n);
GT_API GTN_GetAxisPrfVelSmoothEx(short core, short index, double* pW, double* pT, double* pN);
GT_API GTN_SetMotionSmoothWorkMode(short core, short type, short index, short mode);
GT_API GTN_GetMotionSmoothWorkMode(short core, short type, short index, short* pMode);

GT_API GT_SetAxisMotionSmooth(short axis, double time, double k);
GT_API GT_GetAxisMotionSmooth(short axis, double* pTime, double* pK);
GT_API GTN_SetAxisMotionSmooth(short core, short axis, double time, double k);
GT_API GTN_GetAxisMotionSmooth(short core, short axis, double* pTime, double* pK);
typedef struct PathOptimizePrm
{
    short optimizeMode;
    short blendingType;
    short pad[2];
    double tolerance;
    double blendingPrm;
    double blendingMinAngle;
    double blendingMaxAngle;
} TPathOptimizePrm;
GT_API GTN_SetPathOptimizePrmLa(short core, short crd, short enable, short mode, void* pPrm);   //轨迹优化
typedef struct MotionSmooth
{
    short mode;                  //平滑模式
    short reserve[3];            //保留值
    double prm[6];               //参数
}TMotionSmooth;
GT_API GTN_SetMotionSmooth(short core, short index, TMotionSmooth* pSmooth, short count);
GT_API GTN_GetMotionSmooth(short core, short index, TMotionSmooth* pSmooth, short count, short* pCountReturn);
//设置本核的硬件平滑资源数量，另外一个核为总数量减去本核数量
GT_API GTN_SetMotionSmoothFpgaResCount(short core, short count);
GT_API GTN_GetMotionSmoothFpgaResCount(short core, short* pCount);
// Input shaping
GT_API GTN_SetAxisInputShaping(short core, short axis, short enable, short count, double k);
GT_API GTN_SetAxisInputShapingPro(short core, short axis, short enable, double freq, double z);
GT_API GT_SetAxisInputShaping(short axis, short enable, short count, double k);
GT_API GTN_SetPathShaping(short core, short axis, short enable, double t1, double t2);
GT_API GTN_GetPathShaping(short core, short axis, short* pEnable, double* pT1, double* pT2);
GT_API GTN_SetPathShapingPara(short core, short axis, double coef, short level);
/*-----------------------------------------------------------*/
/* Compensate                                                */
/*-----------------------------------------------------------*/
GT_API GT_SetBacklash(short axis, long value, double changeValue, long dir);
GT_API GT_GetBacklash(short axis, long* pValue, double* pChangeValue, long* pDir);
GT_API GT_SetLeadScrewComp(short axis, short n, long startPos, long lenPos, long* pPositive, long* pNegative);
GT_API GT_EnableLeadScrewComp(short axis, short mode);
GT_API GT_SetLeadScrewCrossComp(short axis, short n, long startPos, long lenPos, long* pPositive, long* pNegative, short link);
GT_API GT_EnableLeadScrewCrossComp(short axis, short mode);
GT_API GT_GetCompensate(short axis, double* pPitchError, double* pCrossError, double* pBacklashError, double* pEncPos, double* pPrfPos);

typedef struct BacklashComp
{
    short enable;
    long compValue;
    long compTime;
    short dir;
}TBacklashComp;

typedef struct LeadScrewCompStatus
{
    short run;
    short pad1[3];
    long  pad2[2];
    double pad3[2];
}TLeadScrewCompStatus;
GT_API GTN_SetBacklash(short core, short axis, long value, double changeValue, long dir);
GT_API GTN_GetBacklash(short core, short axis, long* pValue, double* pChangeValue, long* pDir);
GT_API GTN_SetBacklashComp(short core, short axis, TBacklashComp* pBacklash);
GT_API GTN_GetBacklashComp(short core, short axis, TBacklashComp* pBacklash);
GT_API GTN_SetLeadScrewComp(short core, short axis, short n, long startPos, long lenPos, long* pPositive, long* pNegative);
GT_API GTN_EnableLeadScrewComp(short core, short axis, short mode);
GT_API GTN_SetLeadScrewCrossComp(short core, short axis, short n, long startPos, long lenPos, long* pPositive, long* pNegative, short link);
GT_API GTN_EnableLeadScrewCrossComp(short core, short axis, short mode);

typedef struct
{
    int16_t axis;
    int16_t link;
    int16_t pad;
    int16_t n;
    double startPos;
    double lenPos;
}TAxisLeadScrewCrossCompPrm;

typedef struct
{
    double compPos;
    double compNeg;
}TLeadScrewCrossCompData;

typedef struct
{
    int16_t axis;
    int16_t pad[2];
    int16_t compDataCount;
    TLeadScrewCrossCompData leadScrewCrossCompData[64];
}TAxisLeadScrewCrossCompData;

typedef struct
{
    int16_t axis;
    int16_t enable;
    int16_t pad[2];
}TEnableAxisLeadScrewCrossComp;

/**
 * @brief 设置轴的交叉补偿参数
 * @param core 核号，索引从1开始
 * @param pAxisLeadScrewCrossCompPrm 交叉补偿参数
 * @return 0：    指令执行成功
 *         7：    （1）补偿表个数小于2；（2）补偿距离数小于0；
 *         17050：（1）轴号超出参数范围；（2）交叉补偿参考的轴号超出参数范围
 *         11059：当前轴的交叉误差补偿已使能，无法重新配置参数，需要先调用GTN_EnableAxisLeadScrewCrossComp关闭交叉补偿后再配置
 *         17100：保留参数未设置为0
 *         17051：补偿表的个数超过了1021
 *         17505：补偿距离参数和补偿表个数计算出来的内部参数有误
*/
GT_API GTN_SetAxisLeadScrewCrossCompPrm(short core, TAxisLeadScrewCrossCompPrm *pAxisLeadScrewCrossCompPrm);

/**
 * @brief 下压交叉误差补偿参数，该指令必须在GTN_SetAxisLeadScrewCrossCompPrm或者GTN_EnableAxisLeadScrewCrossComp指令之后调用
 * @param core 核号，索引从1开始
 * @param pAxisLeadScrewCrossCompData 各个轴的交叉补偿表，一次下压多个轴时，该参数为数组，数组大小为count
 * @param count 表示pAxisLeadScrewCrossCompData有多少个数组元素
 * @return 0：    指令执行成功
 *         7：    （1）某些轴号超出范围；（2）某些轴当前下压的补偿数据个数不大于0
*/
GT_API GTN_SetMultiAxisLeadScrewCrossCompData(short core, TAxisLeadScrewCrossCompData *pAxisLeadScrewCrossCompData,int16_t count);

/**
 * @brief 使能或者关闭交叉误差补偿
 * @param core 核号，索引从1开始
 * @param pEnableAxisLeadScrewCrossComp 使能或者关闭交叉补偿参数
 * @return 0：    指令执行成功
 *         17050：（1）轴号超出参数范围；（2）交叉补偿参考的轴号超出参数范围
 *         17054：使能参数错误，取值范围：[0,1]
 *         17100：保留参数未设置为0
 *         11050：规划轴正在运动
 *         11056：未调用GTN_SetAxisLeadScrewCrossCompPrm指令将交叉补偿表修改为动态下压模式
 *         11051：使能交叉补偿时，检测到未调用GTN_SetAxisLeadScrewCrossCompPrm配置补偿表个数
*/
GT_API GTN_EnableAxisLeadScrewCrossComp(short core, TEnableAxisLeadScrewCrossComp *pEnableAxisLeadScrewCrossComp);

GT_API GTN_SetLeadScrewLink(short core, short axis, short link);
GT_API GTN_GetLeadScrewLink(short core, short axis, short* pLink);
GT_API GTN_GetLeadScrewCompStatus(short core, short axis, TLeadScrewCompStatus* pSts);
GT_API GTN_GetCompensate(short core, short axis, double* pPitchError, double* pCrossError, double* pBacklashError, double* pEncPos, double* pPrfPos);
GT_API GTN_SetLeadScrewCompMode(short core, short axis, short cycleMode, short revCompCycle);
GT_API GTN_GetLeadScrewCompMode(short core, short axis, short* pCycleMode, short* pRevCompCycle);
GT_API GTN_SetFriction(short core, short axis, short gain, double compTime);
GT_API GTN_GetFriction(short core, short axis, short* pGain, double* pCompTime);
GT_API GTN_EnableFriction(short core, short axis);
GT_API GTN_DisableFriction(short core, short axis);
GT_API GTN_SetZeroVelThreshold(short core, short axis, double zeroVelThreshold);
GT_API GTN_GetZeroVelThreshold(short core, short axis, double* pZeroVelThreshold);


typedef struct LeadScrewPrm
{
    short n;
    long startPos;
    long lenPos;
    long* pCompPos;
    long* pCompNeg;
} TLeadScrewPrm;

GT_API GT_SetLeadScrewTable(short axis, TLeadScrewPrm* pPrm);
GT_API GT_EnableLeadScrewTable(short axis, long error = 0);
GT_API GT_DisableLeadScrewTable(short axis);
GT_API GT_GetLeadScrewTablePrfPosCount(long encPos, TLeadScrewPrm* pPrm, short* pCountPositive, short* pCountNegative);
GT_API GT_GetLeadScrewTablePrfPosPositive(long encPos, TLeadScrewPrm* pPrm, short index, long* pPrfPosPositive);
GT_API GT_GetLeadScrewTablePrfPosNegative(long encPos, TLeadScrewPrm* pPrm, short index, long* pPrfPosNegative);

GT_API GTN_SetLeadScrewTable(short core, short axis, TLeadScrewPrm* pPrm);
GT_API GTN_EnableLeadScrewTable(short core, short axis, long error = 0);
GT_API GTN_DisableLeadScrewTable(short core, short axis);
GT_API GTN_GetLeadScrewTablePrfPosCount(short core, long encPos, TLeadScrewPrm* pPrm, short* pCountPositive, short* pCountNegative);
GT_API GTN_GetLeadScrewTablePrfPosPositive(short core, long encPos, TLeadScrewPrm* pPrm, short index, long* pPrfPosPositive);
GT_API GTN_GetLeadScrewTablePrfPosNegative(short core, long encPos, TLeadScrewPrm* pPrm, short index, long* pPrfPosNegative);
GT_API GTN_GetPrfPosBeforeLeadScrewComp(short core,short axis,short dir,double endPos,double *pOriginPos);
typedef struct Compensate2DTable
{
    short count[2];
    long posBegin[2];
    long step[2];
} TCompensate2DTable;

typedef struct Compensate2D
{
    short enable;
    short tableIndex;
    short axisType[2];
    short axisIndex[2];
} TCompensate2D;

GT_API GT_SetCompensate2DTable(short tableIndex, TCompensate2DTable* pTable, long* pData, short extend);
GT_API GT_GetCompensate2DTable(short tableIndex, TCompensate2DTable* pTable, short* pExtend);
GT_API GT_SetCompensate2DTableRotationAngle(short tableIndex, short enable, double rotationAngle);
GT_API GT_GetCompensate2DTableRotationAngle(short tableIndex, short* pEnable, double* pRotationAngle);
GT_API GT_SetCompensate2D(short axis, TCompensate2D* pComp2d);
GT_API GT_GetCompensate2D(short axis, TCompensate2D* pComp2d);
GT_API GT_GetCompensate2DValue(short axis, double* pValue);

GT_API GTN_SetCompensate2DTable(short core, short tableIndex, TCompensate2DTable* pTable, long* pData, short extend);
GT_API GTN_GetCompensate2DTable(short core, short tableIndex, TCompensate2DTable* pTable, short* pExtend);
GT_API GTN_SetCompensate2DTableRotationAngle(short core, short tableIndex, short enable, double rotationAngle);
GT_API GTN_GetCompensate2DTableRotationAngle(short core, short tableIndex, short* pEnable, double* pRotationAngle);
GT_API GTN_SetCompensate2D(short core, short axis, TCompensate2D* pComp2d);
GT_API GTN_GetCompensate2D(short core, short axis, TCompensate2D* pComp2d);
GT_API GTN_GetCompensate2DValue(short core, short axis, double* pValue);
GT_API GTN_SetCompensate2DMode(short core, short axis, short mode);
GT_API GTN_GetCompensate2DMode(short core, short axis, short* pMode);

GT_API GTN_SetCompensateLimit(short core, short compensateIndex, short compensateType, double minLimit, double maxLimit);
GT_API GTN_GetCompensateLimit(short core, short compensateIndex, short compensateType, double* pMinLimit, double* pMaxLimit);

/*-----------------------------------------------------------*/
/* IO and Encoder                                            */
/*-----------------------------------------------------------*/
GT_API GT_SetDo(short doType, long value);
GT_API GT_SetDoBit(short doType, short doIndex, short value);
GT_API GT_GetDo(short doType, long* pValue);
GT_API GT_SetDoBitReverse(short doType, short doIndex, short value, short reverseTime);
GT_API GT_GetDi(short diType, long* pValue);
GT_API GT_GetDiReverseCount(short diType, short diIndex, unsigned long* pReverseCount, short count = 1);
GT_API GT_SetDiReverseCount(short diType, short diIndex, unsigned long* pReverseCount, short count = 1);
GT_API GT_GetDiRaw(short diType, long* pValue);
GT_API GT_GetDiEx(short diType, long* pValue, short count);

GT_API GT_SetDac(short dac, short* pValue, short count = 1);
GT_API GT_GetDac(short dac, short* pValue, short count = 1, unsigned long* pClock = NULL);
GT_API GT_GetAdc(short adc, double* pValue, short count = 1, unsigned long* pClock = NULL);
GT_API GT_GetAdcValue(short adc, short* pValue, short count = 1, unsigned long* pClock = NULL);
GT_API GT_GetAuAdc(short adc, double* pValue, short count = 1, unsigned long* pClock = NULL);
GT_API GT_GetAuAdcValue(short adc, short* pValue, short count = 1, unsigned long* pClock = NULL);

GT_API GT_SetEncPos(short encoder, long encPos);
GT_API GT_GetEncPos(short encoder, double* pValue, short count = 1, unsigned long* pClock = NULL);
GT_API GT_GetEncPosPre(short encoder, double* pValue, short count = 1, unsigned long* pClock = NULL);
GT_API GT_GetEncVel(short encoder, double* pValue, short count = 1, unsigned long* pClock = NULL);

GT_API GT_SetPlsPos(short encoder, long encPos);
GT_API GT_GetPlsPos(short pulse, double* pValue, short count = 1, unsigned long* pClock = NULL);
GT_API GT_GetPlsVel(short pulse, double* pValue, short count = 1, unsigned long* pClock = NULL);

GT_API GT_SetAuEncPos(short encoder, long encPos);
GT_API GT_GetAuEncPos(short encoder, double* pValue, short count = 1, unsigned long* pClock = NULL);
GT_API GT_GetAuEncVel(short encoder, double* pValue, short count = 1, unsigned long* pClock = NULL);

typedef struct DoBit
{
    short pad1[2];
    long reverseTime;
    long pad2[3];
    double pad3[2];
}TDoBit;

GT_API GTN_SetDo(short core, short doType, long value);
GT_API GTN_SetDoEx(short core, short doType, long* pValue, short count);
GT_API GTN_SetDoBit(short core, short doType, short doIndex, short value);
GT_API GTN_SetDoMultiBit(short core, short doType, long* pValue, long* pMask, short count = 1);
GT_API GTN_GetDo(short core, short doType, long* pValue);
GT_API GTN_GetDoEx(short core, short doType, long* pValue, short count);
GT_API GTN_SetDoBitReverse(short core, short doType, short doIndex, short value, short reverseTime);
GT_API GTN_SetDoBitReverseEx(short core, short doType, short doIndex, short value, TDoBit* pDoBit);
GT_API GTN_EnableDoBitPulse(short core, short doType, short doIndex, unsigned short highLevelTime, unsigned short lowLevelTime, long pulseNum, short firstLevel);
GT_API GTN_DisableDoBitPulse(short core, short doType, short doIndex);
GT_API GTN_GetDi(short core, short diType, long* pValue);
GT_API GTN_GetDiBit(short core, short diType, short diIndex, short* pValue);
GT_API GTN_GetDiReverseCount(short core, short diType, short diIndex, unsigned long* pReverseCount, short count = 1);
GT_API GTN_SetDiReverseCount(short core, short diType, short diIndex, unsigned long* pReverseCount, short count = 1);
GT_API GTN_GetDiRaw(short core, short diType, long* pValue);
GT_API GTN_GetDiEx(short core, short diType, long* pValue, short count);

GT_API GTN_SetGtmDoBit(short core, short station, short slot, short doIndex, short* pValue, short count = 1);
GT_API GTN_GetGtmDoBit(short core, short station, short slot, short doIndex, short* pValue, short count = 1);
GT_API GTN_GetGtmDiBit(short core, short station, short slot, short diIndex, short* pValue, short count = 1);
GT_API GTN_SetDacScale(short core, short dac, long scale);
GT_API GTN_GetDacScale(short core, short dac, long* pScale);
GT_API GTN_SetDac(short core, short dac, short* pValue, short count = 1);
GT_API GTN_GetDac(short core, short dac, short* pValue, short count = 1, unsigned long* pClock = NULL);
GT_API GTN_SetAuDac(short core, short dac, short* pValue, short count = 1);
GT_API GTN_GetAuDac(short core, short dac, short* pValue, short count = 1, unsigned long* pClock = NULL);
GT_API GTN_GetAdc(short core, short adc, double* pValue, short count = 1, unsigned long* pClock = NULL);
GT_API GTN_GetAdcValue(short core, short adc, short* pValue, short count = 1, unsigned long* pClock = NULL);
GT_API GTN_GetAuAdc(short core, short adc, double* pValue, short count = 1, unsigned long* pClock = NULL);
GT_API GTN_GetAuAdcValue(short core, short adc, short* pValue, short count = 1, unsigned long* pClock = NULL);
GT_API GTN_SetAdcFilter(short core, short adc, short filterTime);
GT_API GTN_SetAdcFilterPrm(short core, short adc, double k);
GT_API GTN_GetAdcFilterPrm(short core, short adc, double* pk);
GT_API GTN_SetAuAdcFilterPrm(short core, short adc, double k);
GT_API GTN_GetAuAdcFilterPrm(short core, short adc, double* pk);
GT_API GTN_SetAdcBias(short core, short adc, short bias);
GT_API GTN_GetAdcBias(short core, short adc, short* pBias);
GT_API GTN_SetAuAdcBias(short core, short auAdc, short bias);
GT_API GTN_GetAuAdcBias(short core, short auAdc, short* pBias);
#define DIGITAL_OUTPUT_MODE_NORMAL                    (0)
#define DIGITAL_OUTPUT_MODE_REVERSE_TIME              (10)

typedef struct DoReverseParameter
{
    double time;
    double reserve[9];
}TDoReverseParameter;

typedef union DigitalOutputMode
{
    TDoReverseParameter doReverse;
    double data[10];
}TDigitalOutputMode;

typedef struct AtParameter
{
    double distance;
    double delayTime;
    double reserve[8];
}TAtParameter;

typedef struct PsoParameter
{
    double distance;
    double reserve[9];
}TPsoParameter;

typedef struct TsoParameter
{
    double time;
    double reserve[9];
}TTsoParameter;

typedef struct WriteDigitalOutputMode
{
    TAtParameter atPrm;
    TPsoParameter psoPrm;
    TTsoParameter tsoPrm;
    TDoReverseParameter doReverse;
    double data[10];
}TWriteDigitalOutputMode;
typedef struct DigitalOutput
 {
    short mode;              //Do输出模式
    short doType;
    short doIndex;
    short doCount;
    short* pValue;
    short reverse1[2];
    long  reserve2[2];
    TWriteDigitalOutputMode prm;
}TDigitalOutput;

typedef struct DigitalOutputBit
{
    short mode;              //Do输出模式
    short doType;
    short doIndex;
    short doValue;
	short reverseTime;
	short reverse1[3];
	long  reserve2[2];
	TWriteDigitalOutputMode prm;
}TDigitalOutputBit;
typedef struct Delay
{
    double delayTime;
    short reserve1[4];
    long reserve2[2];
    double reserve3[2];
}TDelay;
typedef struct DigitalOutputProByMoveDistance
{
    short type;					// type=0:距离起点distance后Do输出；type=1:距离终点distance时Do输出
    short motionType;			// 保留，必须为0，目前只支持插补运动指令，
    long delayTime;				// 保留，必须为0

    double distance;			// 运动distance后Do输出，单位mm
}TDigitalOutputProByMoveDistance;
typedef struct DigitalOutputProByMoveTime
{
    short type;					// type=0:开始运动后延时delayTime后Do输出；type1:运动结束前提前delayTimeDo输出
    short motionType;			// 保留，必须为0，目前只支持插补运动指令，
    short reserve1[2];			// 保留，必须为0
    double delayTime;			// 时间,单位ms
}TDigitalOutputProByMoveTime;
typedef struct DigitalOutputProDelay
{
    double time;				// 延时输出的时间，单位ms
}TDigitalOutputProDelay;
typedef union WriteDigitalOutputProPrmUnion
{
    double data[30];
    TDigitalOutputProDelay delay;					// 纯延时输出
    TDigitalOutputProByMoveTime moveTime;			// 开始运动后，根据运动时间输出。
    TDigitalOutputProByMoveDistance moveDistance;  // 开始运动后，根据运动距离输出。
}TWriteDigitalOutputProPrmUnion;
typedef struct DigitalOutputPro
{
    // Do输出模式:
    // 模式1：延时输出;
    // 模式2：执行下一条插补指令时，按照设定的距离段前延迟输出或者段末提前输出;
    // 模式3：执行下一条插补指令时，按照设定的时间段前延迟输出或者段末提前输出;
    short mode;              // Do输出模式
    short doType;		     // Do类型
    short doIndex;			 // Do索引
    short doCount;			 // Do输出个数
    short* pValue;			 // Do输出值，
    short reserve1[2];		 // 保留，必须为0
    long  reserve2[2];		 // 保留，必须为0
    TWriteDigitalOutputProPrmUnion prm;
}TDigitalOutputPro;
typedef struct AnalogOutput
{
    short aoType;
    short aoIndex;
    short count;
    short reserve1;
    double* pValue;
    long reserve2[3];
}TAnalogOutput;
typedef struct DigitalInput
{
    short diType;
    short diIndex;
    short diCount;
    short* pValue;
    short reverse1[3];
    long  reserve2[2];
}TDigitalInput;

typedef struct StopIoPrm
{
    short inputType;
    short inputIndex;
    short mode;
    short modePrm;
    short reserve1[4];
    double resuerve2[4];
}TStopIoPrm;

GT_API GTN_ReadDigitalInput(short core, short diType, short diIndex, short* pDi, short diCount = 1);
GT_API GTN_WaitDigitalInput(short core, TDigitalInput* pDi, TListInfo* pListInfo);
GT_API GTN_WriteDigitalOutput(short core, TDigitalOutput* pDo, TListInfo* pListInfo = NULL);
GT_API GTN_WriteDigitalOutputBit(short core, TDigitalOutputBit* pDoBit, TListInfo* pListInfo = NULL);
GT_API GTN_ReadDigitalOutput(short core, short doType, short doIndex, short* pValue, short doCount = 1);
GT_API GTN_ReadAnalogInput(short core, short adcType, short adcIndex, double* pValue, short adcCount);
GT_API GTN_ReadAnalogOutput(short core,short aoType,short aoIndex,double *pValue,short aoCount);
GT_API GTN_WriteAnalogOutput(short core, TAnalogOutput* pAo, TListInfo* pListInfo);
GT_API GTN_SetDelay(short core, TDelay* pDelay, TListInfo* pListInfo);

typedef struct
{
    short linkType;
    short linkIndex;
    short reserve1[6];
    double linkRatio;
    double reserve2[4];
}TAoLinkPrm;
GT_API GTN_SetAoLinkPrm(short core,short aoType,short aoIndex,short link,TAoLinkPrm *pAolinkPrm,TListInfo *pListInfo = NULL);
GT_API GTN_GetAoLinkPrm(short core,short aoType,short aoIndex,short *pLink,TAoLinkPrm *pAolinkPrm);

GT_API GTN_SetAuMtrBias(short core, short dac, short bias);
GT_API GTN_GetAuMtrBias(short core, short dac, short* pBias);
GT_API GTN_SetAuMtrLmt(short core, short dac, short limit);
GT_API GTN_GetAuMtrLmt(short core, short dac, short* pLimit);
GT_API GTN_SetEncPrm(short core, short encoder, long lineNum, short type);
GT_API GTN_GetEncPrm(short core, short encoder, long* pLineNum, short* pType);
GT_API GTN_GetEncLineNum(short core, short encoder, long* pLineNum, short count, unsigned long* pClock);
GT_API GTN_GetEncType(short core, short encoder, short* pType, short count, unsigned long* pClock);
GT_API GTN_SetGratingScale(short core, short index, short* pScale, short count = 1);
GT_API GTN_GetGratingScale(short core, short index, short* pScale, short count = 1);
GT_API GTN_SetEncPos(short core, short encoder, long encPos);
GT_API GTN_SetEncPosEx(short core, short encoder, double pos);
GT_API GTN_GetEncPos(short core, short encoder, double* pValue, short count = 1, unsigned long* pClock = NULL);
GT_API GTN_GetEncPosPre(short core, short encoder, double* pValue, short count = 1, unsigned long* pClock = NULL);
GT_API GTN_GetEncPosRegist(short core, short encoder, double* pValue, short count, unsigned long* pClock);
GT_API GTN_GetEncVel(short core, short encoder, double* pValue, short count = 1, unsigned long* pClock = NULL);

GT_API GTN_EncSns(short core, unsigned short sense);
GT_API GTN_LmtSns(short core, unsigned short sense);
GT_API GTN_GpiSns(short core, unsigned short sense);
GT_API GTN_SetPlsPos(short core, short encoder, long encPos);
GT_API GTN_GetPlsPos(short core, short pulse, double* pValue, short count = 1, unsigned long* pClock = NULL);
GT_API GTN_GetPlsVel(short core, short pulse, double* pValue, short count = 1, unsigned long* pClock = NULL);

GT_API GTN_SetAuEncPos(short core, short encoder, long encPos);
GT_API GTN_GetAuEncPos(short core, short encoder, double* pValue, short count = 1, unsigned long* pClock = NULL);
GT_API GTN_GetAuEncVel(short core, short encoder, double* pValue, short count = 1, unsigned long* pClock = NULL);

GT_API GTN_GetAbsEncPos(short core, short encoder, long* pValue, short mode = 0, short param = 0);
GT_API GTN_GetAbsEncPosEx(short core, short encoder, short mode, __int64* pValue);

typedef struct EncoderSource
{
    short type;
    short index;
}TEncoderSource;
GT_API GTN_SetEncoderSource(short core, short encoder, TEncoderSource* pEncoderSource);
GT_API GTN_GetEncoderSource(short core, short encoder, TEncoderSource* pEncoderSource);

typedef struct SignalDetect
{
    short probeType;			// 捕获类型
    short probeIndex;			// 捕获索引
    short latchType;			// 锁存类型
    short latchIndex;			// 锁存索引

    short sense;				// 捕获电平
    short pad[3];
    double filterWidth;			// 捕获最小宽度
}TSignalDetect;

typedef struct SignalDetectStatus
{
    short status;				// 完成标志,0，未触发，1计数中，2计数完成
    short pad[3];
    double latchWidth;			// 锁存脉宽值
}TSignalDetectStatus;

GT_API GTN_SetSignalTimeFilter(short core, short type, short index, double filterWidth);
GT_API GTN_GetSignalTimeFilter(short core, short type, short index, double* pFilterWidth);
GT_API GTN_ClearSignalDetect(short core, short index);
GT_API GTN_SetSignalDetect(short core, short index, short enable, TSignalDetect* pPrm);
GT_API GTN_GetSignalDetect(short core, short index, short* pEnable, TSignalDetect* pPrm);
GT_API GTN_GetSignalDetectStatus(short core, short index, TSignalDetectStatus* pPrm);
/*-----------------------------------------------------------*/
/* ExtModule                                                 */
/*-----------------------------------------------------------*/
GT_API GTN_ExtModuleInit(short core, short method = 1);
GT_API GTN_SetILinkManuMode(short core, short station);
GT_API GTN_GetILinkManuId(short core, short station, short autoId, short* manuId);
GT_API GTN_GetILinkAutoId(short core, short station, short manuId, short* autoId);
GT_API GTN_SetExtDoBit(short core, short doIndex, short value);
GT_API GTN_GetExtDoBit(short core, short doIndex, short* pValue);
GT_API GTN_SetExtDo(short core, short doIndex, long value, long mask);
GT_API GTN_GetExtDo(short core, short doIndex, long* pValue);
GT_API GTN_GetExtDi(short core, short diIndex, long* pValue);
GT_API GTN_GetExtDiBit(short core, short diIndex, short* pValue);
GT_API GTN_SetExtAoValue(short core, short index, short* pValue, short count = 1);
GT_API GTN_GetExtAiValue(short core, short index, short* pValue, short count = 1);
GT_API GTN_SetExtAo(short core, short index, double* pValue, short count = 1);
GT_API GTN_GetExtAo(short core, short index, double* pValue, short count = 1);
GT_API GTN_GetExtAi(short core, short index, double* pValue, short count = 1);

GT_API GTN_SetILinkDo(short core, short station, short module, unsigned short data, unsigned short mask);
GT_API GTN_GetILinkDi(short core, short station, short module, unsigned short* data);
GT_API GTN_SetILinkAo(short core, short station, short module, short channel, double data);
GT_API GTN_GetILinkAi(short core, short station, short module, short channel, double* data);

GT_API GTN_SetEHMIType(short core, long length);
GT_API GTN_InitialEHMIIO(short core, char* pCfgFile, char cfgType);
GT_API GTN_InitialEHMIPrm(short core, short inputCount, short outputCount);
GT_API GTN_GetEHMIDo(short core, unsigned char* data, unsigned short offset, unsigned short count);
GT_API GTN_SetEHMIDo(short core, unsigned char* data, unsigned short offset, unsigned short count);
GT_API GTN_GetEHMIDi(short core, unsigned char* data, unsigned short offset, unsigned short count);
/*-----------------------------------------------------------*/
/* Config of Ext-Module                                      */
/*-----------------------------------------------------------*/
typedef struct ExtModuleStatus
{
    short active;
    short checkError;
    short linkError;
    short packageErrorCount;
    short pad[8];
} TExtModuleStatus;

typedef struct ExtModuleType
{
    short type;
    short input;
    short output;
} TExtModuleType;

typedef struct ExtIoMap
{
    short station;
    short module;
    short index;
} TExtIoMap;

GT_API GT_OpenExtMdl(char* pDllName);
GT_API GT_CloseExtMdl();
GT_API GT_LoadExtConfig(char* pFileName);
GT_API GT_ResetExtMdl();
GT_API GTN_OpenExtMdl(short core, char* pDllName);
GT_API GTN_CloseExtMdl(short core);
GT_API GTN_ResetExtMdl(short core);
GT_API GTN_LoadExtConfig(short core, char* pFileName);
GT_API GTN_SetExtIoValue(short core, short module, unsigned short value);
GT_API GTN_GetExtIoValue(short core, short module, unsigned short* pValue);
GT_API GTN_LoadExtModuleConfig(short core, char* pFile);
GT_API GTN_SaveExtModuleConfig(short core, char* pFile);
GT_API GTN_SaveRingNetConfig(short core,char *pFile);
GT_API GTN_ExtModuleOn(short core, short station);
GT_API GTN_ExtModuleOff(short core, short station);
GT_API GTN_GetExtModuleStatus(short core, short station, TExtModuleStatus* pStatus);
GT_API GTN_SetExtModuleId(short core, short station, short count, short* pId);
GT_API GTN_GetExtModuleId(short core, short station, short count, short* pId);
GT_API GTN_SetExtModuleReverse(short core, short station, short module, short inputCount, short* pInputReverse, short outputCount, short* pOutputReverse);
GT_API GTN_GetExtModuleReverse(short core, short station, short module, short inputCount, short* pInputReverse, short outputCount, short* pOutputReverse);
GT_API GTN_GetExtModuleCount(short core, short station, short* pCount);
GT_API GTN_GetExtModuleType(short core, short station, short module, TExtModuleType* pModuleType);
GT_API GTN_SetExtIoMap(short core, short type, short index, TExtIoMap* pMap);
GT_API GTN_GetExtIoMap(short core, short type, short index, TExtIoMap* pMap);
GT_API GTN_ClearExtIoMap(short core, short type);
GT_API GTN_SetExtAoRange(short core, short index, double max, double min);
GT_API GTN_GetExtAoRange(short core, short index, double* pMax, double* pMin);
GT_API GTN_SetExtAiRange(short core, short index, double max, double min);
GT_API GTN_GetExtAiRange(short core, short index, double* pMax, double* pMin);
/* Pos Compare                                               */
/*-----------------------------------------------------------*/
#define POS_COMPARE_MODE_FIFO                       (0)
#define POS_COMPARE_MODE_LINEAR                     (1)
#define POS_COMPARE_MODE_EQUIDISTANT                (2)
#define POS_COMPARE_MODE_EQUIDISTANT_BUFFER_PERMIT  (3)	// PSO等待到位触发模式
#define POS_COMPARE_MODE_PSO7					    (7) // PSO7模式
#define POS_COMPARE_MODE_LINEAR_ABS                    	   (8)
#define POS_COMPARE_MODE_FIFO_ABS                          (14)
#define POS_COMPARE_MODE_FIFO_ABS_STATIC                   (15)

#define POS_COMPARE_OUTPUT_PULSE                    (0)
#define POS_COMPARE_OUTPUT_LEVEL                    (1)
#define POS_COMPARE_OUTPUT_AUTO                     (2)

#define POS_COMPARE_SOURCE_ENCODER                  (0)
#define POS_COMPARE_SOURCE_PULSE                    (1)


typedef struct PosCompareMode
{
    short mode;
    short dimension;
    short sourceMode;
    short sourceX;
    short sourceY;
    short outputMode;
    short outputCounter;
    unsigned short outputPulseWidth;
    unsigned short errorBand;
} TPosCompareMode;

typedef struct PosCompareLinear
{
    unsigned long count;
    unsigned short hso;
    unsigned short gpo;

    long startPos;
    long interval;
} TPosCompareLinear;

typedef struct PosCompareLinear2D
{
    unsigned long count;
    unsigned short hso;
    unsigned short gpo;

    long startPosX;
    long startPosY;
    long intervalX;
    long intervalY;
} TPosCompareLinear2D;


typedef struct PosCompareData
{
    long pos;
    unsigned short hso;
    unsigned short gpo;
    unsigned long segmentNumber;
} TPosCompareData;

typedef struct PosCompareData2D
{
    long posX;
    long posY;
    unsigned short hso;
    unsigned short gpo;
    unsigned long segmentNumber;
} TPosCompareData2D;

typedef struct PosCompareStatus
{
    unsigned short mode;
    unsigned short run;
    unsigned short space;
    unsigned long pulseCount;
    unsigned short hso;
    unsigned short gpo;
    unsigned long segmentNumber;
} TPosCompareStatus;

typedef struct PosCompareStatusEx
{
    unsigned short mode;
    unsigned short run;
    uint32_t space;
    uint32_t pulseCount;
    unsigned short hso;
    unsigned short gpo;
    uint32_t segmentNumber;
    uint32_t reserve1[3];
    double   reserve2[2];
} TPosCompareStatusEx;

typedef struct PosCompareInfo
{
    unsigned short config;
    unsigned short fifoEmpty;
    unsigned short head;
    unsigned short tail;
    unsigned long commandReceive;
    unsigned long commandSend;
    long posX;
    long posY;
} TPosCompareInfo;

typedef struct PosComparePsoPrm
{
    unsigned long count;
    unsigned short hso;
    unsigned short gpo;
    long startPosX;
    long startPosY;
    long syncPos;
    long time;
    short reserve[20];
} TPosComparePsoPrm;

typedef struct PosComparePsoPrmPro
{
    unsigned long count;
    unsigned short hso;
    unsigned short gpo;
    long startPosX;
    long startPosY;
    long syncPos;
    long time;
    short pulseWidth;
    short pad;
} TPosComparePsoPrmPro;


typedef struct PosCompareContinueMode
{
    short mode;
    unsigned short count;
    short highLevel;
    short lowLevel;
    short resver[20];
} TPosCompareContinueMode;

typedef struct PosCompareReferencePrm
{
    short	referenceX;
    short	referenceY;
    short	pad1[2];
    double	pad2[2];
} TPosCompareReferencePrm;

typedef struct HsoPulsePrm
{
    short mode;          // 0：不输出，1：按默认设置输出，2：按照当前指令配置信息输出
    short timeScale;     // 时间精度：0：1us，1:0.1us
    short pad1[2];
    double pulseWidth;
    double pad2[3];
}THsoPulsePrm;

typedef struct PosComparePulse
{
    short outputMode;
    short level;
    short reserve1[2];
    double highLevelTime;
    double lowLevelTime;
    double reserve2[4];
}TPosComparePulse;

typedef struct PosCompareMultiPulse
{
    short outputMode;
    short level;
    long count;
    double highLevelTime;
    double lowLevelTime;
    double reserve2[4];
}TPosCompareMultiPulse;

typedef struct PosComparePulseStatus
{
    long count;
    short reserve1[2];
    double reserve2[4];
}TPosComparePulseStatus;

typedef struct PosCompareMultiPsoPrm
{
    unsigned long count;
    unsigned short hso;
    unsigned short gpo;
    long startPosX;
    long startPosY;
    long time;
    short multiNumber;
    short pad1;
    long syncPosArray[256];
} TPosCompareMultiPsoPrm;

typedef struct PosCompareAdditionPrm
{
    short sourceMode;                  // 位置比较叠加轴源选择，-1：取消叠加，0：编码器，1：脉冲计数器
    short additionX;                   // 位置比较x轴的叠加轴索引
    short additionY;                   // 位置比较y轴的叠加轴索引
    short additionZ;                   // 位置比较z轴的叠加轴索引
}TPosCompareAdditionPrm;

GT_API GT_PosCompareStart(short core, short posCompareIndex);
GT_API GT_PosCompareStop(short core, short posCompareIndex);
GT_API GT_PosCompareClear(short core, short posCompareIndex);
GT_API GT_PosCompareStatus(short core, short posCompareIndex, TPosCompareStatus* pStatus);
GT_API GT_PosCompareData(short core, short posCompareIndex, TPosCompareData* pData);
GT_API GT_PosCompareData2D(short core, short posCompareIndex, TPosCompareData2D* pData);
GT_API GT_SetPosCompareMode(short core, short posCompareIndex, TPosCompareMode* pMode);
GT_API GT_GetPosCompareMode(short core, short posCompareIndex, TPosCompareMode* pMode);
GT_API GT_SetPosCompareLinear(short core, short posCompareIndex, TPosCompareLinear* pLinear);
GT_API GT_GetPosCompareLinear(short core, short posCompareIndex, TPosCompareLinear* pLinear);
GT_API GT_SetPosCompareLinear2D(short core, short posCompareIndex, TPosCompareLinear2D* pLinear);
GT_API GT_GetPosCompareLinear2D(short core, short posCompareIndex, TPosCompareLinear2D* pLinear);
GT_API GT_PosCompareInfo(short core, short posCompareIndex, TPosCompareInfo* pInfo);
GT_API GT_SetPosComparePsoPrm(short core, short posCompareIndex, TPosComparePsoPrm* pPrm);
GT_API GT_GetPosComparePsoPrm(short core, short posCompareIndex, TPosComparePsoPrm* pPrm);

GT_API GTN_PosCompareStart(short core, short posCompareIndex);
GT_API GTN_PosCompareStop(short core, short posCompareIndex);
GT_API GTN_PosCompareClear(short core, short posCompareIndex);
GT_API GTN_PosCompareStatus(short core, short posCompareIndex, TPosCompareStatus* pStatus);
GT_API GTN_PosCompareStatusEx(short core, short posCompareIndex, TPosCompareStatusEx* pStatus);
GT_API GTN_PosCompareData(short core, short posCompareIndex, TPosCompareData* pData);
GT_API GTN_PosCompareData2D(short core, short posCompareIndex, TPosCompareData2D* pData);
GT_API GTN_PosComparePulse(short core, short posCompareIndex, short outputMode, short level, unsigned short outputPulseWidth);
GT_API GTN_SetPosComparePulseCount(short core, short posCompareIndex, long count);
GT_API GTN_PosComparePulseEx(short core, short posCompareIndex, TPosComparePulse* pPosComparePulse);
GT_API GTN_PosCompareMultiPulse(short core, short posCompareIndex, TPosCompareMultiPulse* pPosComparePulse);
GT_API GTN_GetPosComparePulseStatus(short core, short posCompareIndex, TPosComparePulseStatus* pPosComparePulseStatus);
GT_API GTN_SetPosCompareMode(short core, short posCompareIndex, TPosCompareMode* pMode);
GT_API GTN_GetPosCompareMode(short core, short posCompareIndex, TPosCompareMode* pMode);
GT_API GTN_SetPosCompareContinueMode(short core, short posCompareIndex, TPosCompareContinueMode* pMode);
GT_API GTN_GetPosCompareContinueMode(short core, short posCompareIndex, TPosCompareContinueMode* pMode);
GT_API GTN_SetPosCompareLinear(short core, short posCompareIndex, TPosCompareLinear* pLinear);
GT_API GTN_GetPosCompareLinear(short core, short posCompareIndex, TPosCompareLinear* pLinear);
GT_API GTN_SetPosCompareLinearBuf(short core, short posCompareIndex, TPosCompareLinear* pLinear);
GT_API GTN_SetPosCompareLinear2D(short core, short posCompareIndex, TPosCompareLinear2D* pLinear);
GT_API GTN_GetPosCompareLinear2D(short core, short posCompareIndex, TPosCompareLinear2D* pLinear);
GT_API GTN_SetPosCompareLinear2DBuf(short core, short posCompareIndex, TPosCompareLinear2D* pLinear);
GT_API GTN_PosCompareInfo(short core, short posCompareIndex, TPosCompareInfo* pInfo);
GT_API GTN_PosCompareHsOn(short core, short posCompareIndex, short link, unsigned short threshold = 200);
GT_API GTN_PosCompareHsOff(short core, short posCompareIndex);
GT_API GTN_PosCompareSpace(short core, short posCompareIndex, unsigned short* pSpace);
GT_API GTN_SetPosComparePsoPrm(short core, short posCompareIndex, TPosComparePsoPrm* pPrm);
GT_API GTN_GetPosComparePsoPrm(short core, short posCompareIndex, TPosComparePsoPrm* pPrm);
/**
 * @brief 设置pso间距
 * @param core 核号
 * @param posCompareIndex pso索引
 * @param synchPos pso间距，精度：取至8位小数，单位：pulse
 * @return 0：指令执行成功
 *         1：位置比较已启动
 *         4：模块固件不支持
 *         7：参数错误
*/
GT_API GTN_SetPosComparePsoSynchPos(short core,short posCompareIndex,double synchPos);
GT_API GTN_SetPosCompareMultiPsoPrm(short core, short posCompareIndex, TPosCompareMultiPsoPrm* pPrm);
GT_API GTN_GetPosCompareMultiPsoPrm(short core, short posCompareIndex, TPosCompareMultiPsoPrm* pPrm);
GT_API GTN_SetPosComparePsoOffDistance(short core, short posCompareIndex, long distance);
GT_API GTN_SetPosCompareStartLevel(short core, short posCompareIndex, short type, short startLevel);
GT_API GTN_SetPosCompareReference(short core, short posCompareIndex, TPosCompareReferencePrm* pPrm);
GT_API GTN_GetPosCompareReference(short core, short posCompareIndex, TPosCompareReferencePrm* pPrm);
GT_API GTN_SetPosCompareAddition(short core, short posCompareIndex, TPosCompareAdditionPrm* pPrm);
GT_API GTN_GetPosCompareAddition(short core, short posCompareIndex, TPosCompareAdditionPrm* pPrm);
GT_API GTN_SetHsoPulsePrm(short core, short station, short hsoIndex, THsoPulsePrm* pPem, short hsoCount = 1);
GT_API GTN_GetHsoPulsePrm(short core, short station, short hsoIndex, THsoPulsePrm* pPem, short hsoCount = 1);
GT_API GTN_SetPosComparePulseDuty(short core, short posCompareIndex, double duty);
GT_API GTN_PosCompareVariablePsoData(short core, short posCompareIndex, TPosCompareData* pData);
GT_API GTN_BufPosCompareData2D(short core, short crd, short posCompareIndex, TPosCompareData2D* pData, short fifo);
GT_API GTN_BufPosCompareData2DEx(short core, short crd, short posCompareIndex, TPosCompareData2D* pData, short fifo);

#define POS_COMPARE_THRESHOLD_DEFAULT                      (0)     //位置进入用户设置的误差带范围后，根据最优算法找到最优点输出。
#define POS_COMPARE_THRESHOLD_ZERO	              (1)     //位置进入用户设置的误差带范围立即输出。
#define POS_COMPARE_THRESHOLD_FPGA	              (2)     //位置进入用户设置的误差带范围后，根据自适应算法找到最优点输出。
GT_API GTN_SetPosCompareThresholdMode(short core, short index, short mode);
GT_API GTN_GetPosCompareThresholdMode(short core, short index, short* pMode);
GT_API GTN_SetPosCompareThresholdValue(short core, short index, short thresholdValue);
GT_API GTN_GetPosCompareThresholdValue(short core, short index, short* pThresholdValue);
typedef struct PosCompareModeEx
{
    short			mode;							// 0：FIFO模式，1：Linear模式 2：等间距输出模式
    short			dimension;					//1：1D，2：2D
    short			sourceMode;					// 0：编码器； 1：脉冲计数器
    short			source[8];					//编码器比较源
    short			outputMode;					/*输出模式：0:脉冲 1:电平2：电平自
                                        动翻转(电平模式并且输出电平自动反转，不受posCompareData电平影响)*/
    short			outputCounter;				// 保留
    unsigned short	outputPulseWidth;	/*输出脉冲宽度,单位为1us，电平模式该参数无效*/
    unsigned short	errorBand;			// 二维位置比较输出误差带
    short			reserve1[2];
    double			reserve2[16];
} TPosCompareModeEx;

typedef struct PosComparePsoPrmEx
{
    unsigned long count;
    unsigned short hso;
    unsigned short gpo;
    long startPosX;
    long startPosY;
    long time;

    short pad1;
    short multiNumber;
    long scale[6];
    double syncPosArray[256];
} TPosComparePsoPrmEx;

GT_API GTN_SetPosCompareModeEx(short core, short posCompareIndex, TPosCompareModeEx* pMode);
GT_API GTN_GetPosCompareModeEx(short core, short posCompareIndex, TPosCompareModeEx* pMode);
GT_API GTN_SetPosComparePsoPrmEx(short core, short posCompareIndex, TPosComparePsoPrmEx* pPrm);
GT_API GTN_GetPosComparePsoPrmEx(short core, short posCompareIndex, TPosComparePsoPrmEx* pPrm);
GT_API GTN_ComparePulse(short core, short level, short outputType, short time);
GT_API GTN_CompareStop(short core);
GT_API GTN_CompareStatus(short core, short* pStatus, long* pCount);
GT_API GTN_CompareData(short core, short encoder, short source, short pulseType, short startLevel, short time, long* pBuf1, short count1, long* pBuf2, short count2);
GT_API GTN_CompareLinear(short core, short encoder, short channel, long startPos, long repeatTimes, long interval, short time, short source);
GT_API GTN_EnablePoscomparePsoPulseWidthFollow(short core, short index, short enable, double velMin, double velMax, double cmpPulseWidthMax);
/**
 * @brief 打开或者关闭DSP位置比较fifo空间扩大至30000段的功能，默认是1000段
 * @param core 核号，索引从1开始
 * @param posCompareIndex 位置比较索引，取值范围：[1,10]
 * @param enable 打开或者关闭DSP位置比较fifo空间扩大至30000段的功能，1：打开，0：关闭
 * @param fifoOperation fifo空间需要变化（即扩大或者恢复默认）且fifo中存在数据时的处理方式，0：自动清空fifo后，再扩展fifo空间，1：fifo中有数据时返回错误
 * @return 0：指令执行成功
           1：（1）位置比较已启动；（2）fifoOperation设置为1且fifo中有数据
*/
GT_API GTN_PosCompareDataFifoSizeExtend(short core,short posCompareIndex,short enable,short fifoOperation);
/*-----------------------------------------------------------*/
/* Config of Position Compare                                */
/*-----------------------------------------------------------*/
typedef struct PosCompareMap
{
    short module;
    short fifo;
} TPosCompareMap;

GT_API GTN_SetPosCompareMap(short core, short index, TPosCompareMap* pMap);
GT_API GTN_GetPosCompareMap(short core, short index, TPosCompareMap* pMap);
GT_API GTN_ClearPosCompareMap(short core);
#define POS_COMPARE_MODE_PSO_SYNC_EXTERNAL_SIGNAL	(1)
#define POS_COMPARE_MODE_PSO_SYNC_INTERNAL_SIGNAL	(2)

GT_API GTN_SetPosComparePsoSyncPrm(short core, short posCompareIndex, short psoSyncMode, double frq = 100);

#define COMPARE_SEND_DATA_MAX			(30)
#define COMPARE_DATA_MAX				(4096)
#define COMPARE_STEP_MAX				(0x1fff)
#define COMPARE_MAX_NUM					(0x3fffffff)

typedef struct PosCompareCompensateCoeff
{
    unsigned short velCompCoeff[3];
    short reserve1[2];
    unsigned short accCompCoeff[3];
    short reserve2[2];
}TPosCompareCompensateCoeff;
GT_API GTN_SetPosCompareCompensateCoeff(short core, short posCompareIndex, TPosCompareCompensateCoeff* pCoeff);
GT_API GTN_GetPosCompareCompensateCoeff(short core, short posCompareIndex, TPosCompareCompensateCoeff* pCoeff);

typedef struct PosCompareEnablePro
{
    short index;              // 位置比较索引号
    short enable;             // 开启关闭
    long  reserve[3];         // 保留参数
}TPosCompareEnablePro;

typedef struct PosComparePrmPro
{
    short index;
    short pulseWidth;
    short reserve[2];
    unsigned long count;
    long syncPos;
    long time;
} TPosComparePrmPro;

GT_API GTN_SetPosCompareEnablePro(short core, short group, TPosCompareEnablePro* pPrm, TListInfo* pListInfo);
GT_API GTN_SetPosComparePrmPro(short core, short group, TPosComparePrmPro* pPrm, TListInfo* pListInfo);
GT_API GTN_SetPosCompareData2DPro(short core, short index, TPosCompareData2D* pData, TListInfo* pListInfo);


/*-----------------------------------------------------------*/
/* Laser and Scan                                            */
/*-----------------------------------------------------------*/
#define FIFO_MODE_STATIC                  (0)
#define FIFO_MODE_DYNAMIC                 (1)

#define SCAN_STATUS_WAIT                  (0)
#define SCAN_STATUS_RUN	                  (1)
#define SCAN_STATUS_DONE                  (2)

typedef struct ScanInit
{
    int lookAheadNum;
    double time;
    double radiusRatio;
}TScanInit;

typedef struct ScanInfo
{
    unsigned long segmentNumber;
    unsigned short commandNumber;
    unsigned short prfVel;
    unsigned short fifoEmpty;
    unsigned short head;
    unsigned short tail;
    unsigned long commandReceive;
    unsigned long commandSend;
    unsigned long reserve[6];
} TScanInfo;

typedef struct ScanPosSuperposeParameter
{
    short enable;
    short superposeSrc;
    short superposeAxisX;
    short superposeAxisY;
    double xCoefficient;
    double yCoefficient;
    double xVelCoefficient;
    double yVelCoefficient;
}TScanPosSuperposeParameter;

typedef struct LaserInfo
{
    unsigned short hso;
    unsigned short powerMode;
    unsigned short power;
    unsigned short powerMax;
    unsigned short powerMin;
    unsigned short frequency;
    unsigned short pulseWidth;
} TLaserInfo;

typedef struct LaserPowerPrm
{
    short n;
    double startVel;
    double power;
}TLaserPowerPrm;

typedef struct LaserPowerTable
{
    short n;
    double startVel;
    double velStep;
    double* power;
}TLaserPowerTable;

typedef struct ScanCorrectionTableData
{
    short corrX[65][65];
    short corrY[65][65];
}TScanCorrectionTableData;


GT_API GT_LaserAo(short value, unsigned short laserChannel = 0);
GT_API GT_SetHSIOOpt(unsigned short value, short laserChannel = 0);
GT_API GT_GetHSIOOpt(unsigned short* pValue, short laserChannel = 0);
GT_API GT_SetLaserMode(unsigned short laserMode);
GT_API GT_LaserPowerMode(short laserPowerMode, double maxValue, double minValue, short laserChannel = 0, short delayMode = 0);
GT_API GT_LaserPrfCmd(double power, unsigned short laserChannel = 0);
GT_API GT_LaserOutFrq(double outFrq, unsigned short laserChannel = 0);
GT_API GT_SetPulseWidth(unsigned short width, unsigned short laserChannel = 0);
GT_API GT_SetLevelDelay(unsigned short highLevelDelay, unsigned short lowLevelDelay, unsigned short laserChannel = 0);
GT_API GT_LaserInfo(TLaserInfo* pLaserInfo, short laserChannel = 0);

GT_API GTN_ScanInit(short core, TScanInit* pScanInit = NULL, double jumpAcc = 0, double markAcc = 0, short scan = 1);
GT_API GTN_ScanCrdDataEnd(short core, short scan = 1);
GT_API GTN_SetScanLaserLink(short core, short link, short scan = 1);
GT_API GTN_GetScanLaserLink(short core, short* pLink, short scan = 1);
GT_API GTN_SetScanMode(short core, short mode, short scan = 1);
GT_API GTN_GetScanMode(short core, short* pMode, short scan = 1);
GT_API GTN_ClearScanStatus(short core, short scan = 1);
GT_API GTN_ScanGetCrdPos(short core, short* pPos, short scan = 1);
GT_API GTN_ScanJump(short core, short x, short y, double vel, short scan = 1);
GT_API GTN_ScanJumpPoint(short core, short x, short y, double vel, long motionDelayTime, long laserDelayTime, short scan = 1);
GT_API GTN_ScanTimeJump(short core, short x, short y, unsigned short time, short scan = 1);
GT_API GTN_ScanTimeJumpPoint(short core, short x, short y, unsigned short time, long motionDelayTime, long laserDelayTime, short scan = 1);
GT_API GTN_ScanMark(short core, short x, short y, double vel, short scan = 1);
GT_API GTN_ScanTimeMark(short core, short x, short y, unsigned short time, short scan = 1);
GT_API GTN_ScanBufLaserPrfCmd(short core, double laserPower, short scan = 1);
GT_API GTN_ScanBufIO(short core, unsigned short doType, unsigned short doMask, unsigned short doValue, short scan = 1);
GT_API GTN_ScanBufDelay(short core, long time, short scan = 1);
GT_API GTN_ScanBufDA(short core, unsigned short chn, short value, short scan = 1);
GT_API GTN_ScanBufAO(short core, unsigned short chn, double voltage, short scan = 1);
GT_API GTN_ScanBufLaserDelay(short core, short laserOnDelay, short laserOffDelay, short scan = 1);
GT_API GTN_ScanBufLaserDelayLong(short core, long laserOnDelay, long laserOffDelay, short scan = 1);
GT_API GTN_ScanBufLaserOutFrq(short core, double outFrq, short scan = 1);
GT_API GTN_ScanBufSetPulseWidth(short core, unsigned short width, short scan = 1);
GT_API GTN_ScanBufLaserOn(short core, short scan = 1);
GT_API GTN_ScanBufLaserOff(short core, short scan = 1);
GT_API GTN_ScanBufStop(short core, short scan = 1);
GT_API GTN_ScanLaserIntervalOnList(short core, long time, short scan = 1);
GT_API GTN_SetScanDelayTime(short core, unsigned short maxJumpDelay, unsigned short markDelay, unsigned short multiMarkDelayConst, short scan = 1);
GT_API GTN_SetScanDelayMode(short core, short multiMarkDelayMode, unsigned short multiMarkLaserOffDelay, unsigned short minJumpDelay, unsigned short jumpDelayLengthLimit, short scan = 1);
GT_API GTN_ScanStop(short core, short stopType, short scan = 1);
GT_API GTN_ScanCrdSpace(short core, short* pSpace, short scan = 1);
GT_API GTN_ScanCrdStart(short core, short scan = 1);
GT_API GTN_ScanCrdClear(short core, short scan = 1);
GT_API GTN_ScanCrdStatus(short core, short* pRun, short* pCmdId, short scan = 1);

GT_API GTN_SetScanPosSuperposeParameter(short core, short scan, TScanPosSuperposeParameter param);
GT_API GTN_GetScanPosSuperposeParameter(short core, short scan, TScanPosSuperposeParameter* pParam);
GT_API GTN_ScanSetCorrectionTable(short core, short scan, TScanCorrectionTableData* pParam);
GT_API GTN_ScanCorrectionOn(short core, short scan);
GT_API GTN_ScanCorrectionOff(short core, short scan);
GT_API GTN_ScanGenerateCorrectionTable(short core, double paraX, double paraY, short rangeX, short rangeY, TScanCorrectionTableData* pParam);
GT_API GTN_GetScanPosSuperposeEncPos(short core, short scan, double* pXPos, double* pYPos);


GT_API GTN_SetScanDaType(short core, short type, short scan);

GT_API GTN_ScanInfo(short core, TScanInfo* pScanInfo, short scan = 1);
GT_API GTN_GetScanExecuteTime(short core, short scan, double* pExecuteTime);
GT_API GTN_ClearScanExecuteTime(short core, short scan = 1);
GT_API GTN_ScanHsOn(short core, short scan = 1, short link = 1, unsigned short threshold = 200);
GT_API GTN_ScanHsOff(short core, short scan = 1);

/*
*   功能说明：使能FPK功能（首脉冲抑制功能）
*   core：         核号，取值范围：[1,32]
*   time1：        表示FPK信号的有效电平持续时间，取值范围：[0,65535]，单位：us
*   time2：        表示FPK信号开始输出和脉冲信号开始输出之间的间隔时间，即开光延时时间，取值范围：[0,65535]，单位：us
*   laserOffDelay：表示激光关闭延时时间，取值范围：[0,65535]，单位：us
*   channel：      需要使能FPK功能的激光通道号，取值范围：[0,9]
*/
GT_API GTN_EnaFPK(short core, unsigned short time1, unsigned short time2, unsigned short laserOffDelay, short channel = 0);

/*
*   功能说明：关闭FPK功能（首脉冲抑制功能），如果首脉冲保持时间大于0，需要将开关光延时清零
*   core：         核号，取值范围：[1,32]
*   channel：需要关闭FPK功能的激光通道号，取值范围：[0,9]
*/
GT_API GTN_DisFPK(short core, short channel = 0);

GT_API GTN_LaserOn(short core, short laserChannel = 0);
GT_API GTN_LaserOff(short core, short laserChannel = 0);
GT_API GTN_LaserOnStatus(short core, unsigned short* pValue, short laserChannel = 0);
GT_API GTN_LaserPowerMode(short core, short laserPowerMode, double maxValue, double minValue, short laserChannel = 0);
GT_API GTN_LaserPrfCmd(short core, double power, short laserChannel = 0);
GT_API GTN_LaserOutFrq(short core, double outFrq, short laserChannel = 0);
GT_API GTN_SetPulseWidth(short core, unsigned short width, short laserChannel = 0);
GT_API GTN_SetLevelDelay(short core, unsigned long highLevelDelay, unsigned long lowLevelDelay, short laserChannel = 0);
GT_API GTN_LaserInfo(short core, TLaserInfo* pLaserInfo, short laserChannel = 0);
GT_API GTN_WriteLaserPrfCmd(short core, double power, short channel);
GT_API GTN_SetWaitPulse(short core, unsigned short mode, double waitPulseFrq, double waitPulseDuty, short channel);

/**
 * @brief 设置激光开关光信号与模拟量信号绑定关系
 * @param core 核号
 * @param laserChannel 激光通道号，索引从0开始
 * @param link 绑定关系，0：不绑定，1：绑定
 * @return 17053：激光通道参数错误
 *         17055：核号参数错误，目前每张卡只有第一个核支持激光功能
 *         17500：link参数错误
 *         11091：网络中没有接带激光功能的从站
*/
GT_API GTN_SetLaserOnAndVoltageLink(short core,short laserChannel,short link);

/**
 * @brief 读取激光开关光信号与模拟量信号绑定关系
 * @param core 核号
 * @param laserChannel 激光通道号，索引从0开始
 * @param pLink 绑定关系，0：不绑定，1：绑定
 * @return 17053：激光通道参数错误
 *         17055：核号参数错误，目前每张卡只有第一个核支持激光功能
 *         11091：网络中没有接带激光功能的从站
*/
GT_API GTN_GetLaserOnAndVoltageLink(short core,short laserChannel,short *pLink);

typedef struct LaserStatus
 {
    short run;
    short mode;
    double power;
    double frequency;
    double pulseWidth;
    double pad1[9];
    short pad2[8];
} TLaserStatus;
GT_API GTN_GetLaserStatus(short core, short channel, TLaserStatus* pStatus);

GT_API GTN_ScanLaserOn(short core, short scan = 1);
GT_API GTN_ScanLaserOff(short core, short scan = 1);
GT_API GTN_ScanLaserOnStatus(short core, unsigned short* pValue, short scan = 1);
GT_API GTN_ScanSetLaserMode(short core, unsigned short laserMode, short scan = 1);
GT_API GTN_ScanLaserPowerMode(short core, short laserPowerMode, double maxValue, double minValue, short scan = 1);
GT_API GTN_ScanLaserPrfCmd(short core, double power, short scan = 1);
GT_API GTN_ScanLaserOutFrq(short core, double outFrq, short scan = 1);
GT_API GTN_ScanSetPulseWidth(short core, unsigned short width, short scan = 1);
GT_API GTN_ScanSetLevelDelay(short core, unsigned short highLevelDelay, unsigned short lowLevelDelay, short scan = 1);
GT_API GTN_ScanSetDa(short core, short chn, short value, short scan = 1);
GT_API GTN_ScanLaserInfo(short core, TLaserInfo* pLaserInfo, short scan = 1);

GT_API GTN_ScanSetMotionWithLaserControl(short core, short laserCtrlEnable, short scan = 1);
GT_API GTN_ScanGetMotionWithLaserControl(short core, short* pLaserCtrlEnable, short scan = 1);

GT_API GTN_SetHsoPwmLink(short core, short station, short hsoIndex);
GT_API GTN_GetHsoPwmLink(short core, short station, short* pHsoIndex);

GT_API GTN_ScanSynchOn(short core);
GT_API GTN_ScanSynchOff(short core);
GT_API GTN_SetScanSynchModePrm(short core, short masterScan, short count, short* synchScan, short synchSignalSource);
GT_API GTN_GetScanWorkMode(short core, short scan, short* pMode);
/*-----------------------------------------------------------*/
/* DLM                                                       */
/*-----------------------------------------------------------*/
#define DLM_FUNCTION_EVENT							(0)
#define DLM_FUNCTION_TIMER							(1)
#define DLM_FUNCTION_BACKGROUND						(2)
#define DLM_FUNCTION_COMMAND						(3)

#define DLM_FUNCTION_PROCEDURE						(7)

#define DLM_FUNCTION_PROFILE_EVENT					(8)
#define DLM_FUNCTION_PROFILE						(9)
#define DLM_FUNCTION_PROFILE_SUPERIMPOSED			(10)
#define DLM_FUNCTION_PROFILE_FILTER					(11)

#define DLM_FUNCTION_SERVO_EVENT					(16)
#define DLM_FUNCTION_SERVO							(17)
#define DLM_FUNCTION_SERVO_SUPERIMPOSED				(18)
#define DLM_FUNCTION_SERVO_FILTER					(19)

#define DLM_LOAD_MODE_NONE							(0)
#define DLM_LOAD_MODE_COMMAND						(1)
#define DLM_LOAD_MODE_BOOT							(2)
#define DLM_LOAD_MODE_RUN							(3)

typedef struct TDlmStatus
{
    long version;
    long date;
    short enable;
    long function;
} TDlmStatus;

typedef struct TDlmFunction
{
    short function;
    short enable;
    long value;
} TDlmFunction;

GT_API GT_LoadDlm(long vender, long module, char* fileName, short* pId);
GT_API GT_ProgramDlm(short id, short loadMode);
GT_API GT_GetDlmLoadMode(short id, short* pLoadMode);
GT_API GT_RunDlm(short id);
GT_API GT_StopDlm(short id);
GT_API GT_GetDlmStatus(short id, TDlmStatus* pStatus);
GT_API GT_SetDlmFunction(short id, TDlmFunction* pFunction);
GT_API GT_GetDlmFunction(short id, TDlmFunction* pFunction);

GT_API GT_DlmCommandInit(short code, long index);
GT_API GT_DlmCommandAdd16(short value);
GT_API GT_DlmCommandAdd32(long value);
GT_API GT_DlmCommandAddFloat(float value);
GT_API GT_DlmCommandAddDouble(double value);
GT_API GT_SendDlmCommand(short id, short* pReturnValue);
GT_API GT_DlmCommandGet16(short* pValue);
GT_API GT_DlmCommandGet32(long* pValue);
GT_API GT_DlmCommandGetFloat(float* pValue);
GT_API GT_DlmCommandGetDouble(double* pValue);

GT_API GTN_LoadDlm(short core, long vender, long module, char* fileName, short* pId);
GT_API GTN_ProgramDlm(short core, short id, short loadMode);
GT_API GTN_GetDlmLoadMode(short core, short id, short* pLoadMode);
GT_API GTN_RunDlm(short core, short id);
GT_API GTN_StopDlm(short core, short id);
GT_API GTN_GetDlmStatus(short core, short id, TDlmStatus* pStatus);
GT_API GTN_SetDlmFunction(short core, short id, TDlmFunction* pFunction);
GT_API GTN_GetDlmFunction(short core, short id, TDlmFunction* pFunction);

GT_API GTN_DlmCommandInit(short core, short code, long index);
GT_API GTN_DlmCommandAdd16(short core, short value);
GT_API GTN_DlmCommandAdd32(short core, long value);
GT_API GTN_DlmCommandAddFloat(short core, float value);
GT_API GTN_DlmCommandAddDouble(short core, double value);
GT_API GTN_SendDlmCommand(short core, short id, short* pReturnValue);
GT_API GTN_DlmCommandGet16(short core, short* pValue);
GT_API GTN_DlmCommandGet32(short core, long* pValue);
GT_API GTN_DlmCommandGetFloat(short core, float* pValue);
GT_API GTN_DlmCommandGetDouble(short core, double* pValue);
/*-----------------------------------------------------------*/
/* Event-Task                                                */
/*-----------------------------------------------------------*/
#define TASK_SET_DO_BIT                          (0x1101)
#define TASK_SET_DAC                             (0x1120)

#define TASK_STOP                                (0x1303)

#define TASK_TRIGGER_CALLBACK_FUNCTION           (0x119c)

#define TASK_UPDATE_POS                          (0x2002)
#define TASK_UPDATE_VEL                          (0x2004)
#define TASK_UPDATE_DISTANCE                     (0x2022)

#define TASK_PT_START                            (0x2306)
#define TASK_PVT_START                           (0x2346)
#define TASK_MOVE_ABSOLUTE                       (0x2500)

#define TASK_MOVE_CONTINUOUS_ABSOLUTE            (0x2560)

#define TASK_GEAR_START                          (0x3005)

#define TASK_FOLLOW_START                        (0x310A)
#define TASK_FOLLOW_SWITCH                       (0x310B)

#define TASK_CRD_START                           (0x4004)
#define TASK_CRD_OVERRIDE                        (0x4006)

#define TASK_CRD_STEP_MODE                       (0x4015)
#define TASK_SCAN_START                          (0x4102)

#define TASK_START_COMMAND_LIST                  (0x4825)

#define TASK_STOP_COMMAND_LIST                   (0x482B)

#define TASK_MOVE_ESCAPE                         (0x5000)

#define TASK_UPDATE_SERVO_GAIN                   (0x6000)
#define	TASK_SET_TRIGGER_PRM                     (0x6001)

#define TASK_SET_DO_BIT_MODE_NONE                (0)
#define TASK_SET_DO_BIT_MODE_TIME                (10)
#define TASK_SET_DO_BIT_MODE_DISTANCE            (20)

//保存运控变量任务，以及保存变量的最大个数
#define TASK_SAVE_MC_VAR_MAX                    (5)
#define TASK_SAVE_MC_VAR_EX_MAX                 (8)
#define TASK_SAVE_MC_VAR                        (50)
#define TASK_ADJUST_MOVE_JOG_ANGLE_VELOCITY     (51)
#define TASK_SAVE_MC_VAR_EX                     (52)

typedef struct WatchVar
{
    unsigned short type;
    unsigned short index;
    unsigned short id;
} TWatchVar;

typedef struct TaskSetDoBit
{
    short doType;
    short doIndex;
    short doValue;
    short mode;
    long parameter[8];
} TTaskSetDoBit;

typedef struct TaskSetDac
{
    short dac;
    short value;
} TTaskSetDac;

typedef struct TaskStop
{
    long mask;
    long option;
} TTaskStop;

typedef struct TaskFifoOperation
{
    short type;
    short index;
    short operation;
    short data[20];
} TTaskFifoOperation;

typedef struct TaskUpdatePos
 {
    short profile;
    long pos;
} TTaskUpdatePos;

typedef struct TaskUpdateDistance
{
    short profile;
    short triggerIndex;
    long distance;
} TTaskUpdateDistance;

typedef struct TaskUpdateVel
{
    short profile;
    double vel;
} TTaskUpdateVel;

typedef struct TaskPtStart
{
    long mask;
    long option;
} TTaskPtStart;

typedef struct TaskPvtStart
{
    long mask;
} TTaskPvtStart;

typedef struct TaskGearStart
{
    long mask;
} TTaskGearStart;

typedef struct TaskFollowStart
{
    long mask;
    long option;
} TTaskFollowStart;

typedef struct TaskFollowSwitch
{
    long mask;
} TTaskFollowSwitch;

typedef struct TaskMoveAbsolute
{
    short profile;
    long pos;
    double vel;
    double acc;
    double dec;
    short percent;
} TTaskMoveAbsolute;

typedef struct TaskCrdStart
{
    short mask;
    short option;
} TTaskCrdStart;

typedef struct TaskCrdOverride
{
    short crd;
    double synVelOverride;
} TTaskCrdOverride;

typedef struct TaskCrdStepMode
{
    short mask;
    short option;
} TTaskCrdStepMode;

typedef struct TaskScanStart
{
    short port;
    short index;
    short count;
} TTaskScanStart;

//启动指令流
typedef struct TaskStartCommandList
{
    short list;
    short reserve1[23];
} TTaskStartCommandList;

typedef struct TaskTriggerCallbackFunction
{
    short option;
    short reserve;
}TTaskTriggerCallbackFunction;

typedef struct TaskSaveMcVar
{
    short count;
    TWatchVar var[TASK_SAVE_MC_VAR_MAX];
}TTaskSaveMcVar;

// 根据参考源的线速度，调整MoveJog轴（旋转轴）的角速度，使参考源线速度保持恒定
#define TASK_ADJUST_JOG_VEL_RESERVE1_GROUP       (0) // 调整jog轴速度时需要同时调整插补速度的group号
#define TASK_ADJUST_JOG_VEL_RESERVE1_TARGET_TYPE (1) // 调整的目标：0：jog轴速度 1：dac电压
#define TASK_ADJUST_JOG_VEL_RESERVE2_ORIGIN_VEL  (0) // jog轴的原始速度
#define TASK_ADJUST_JOG_VEL_RESERVE2_DAC_RATIO   (1) // 速度和电压的比例关系，每伏对应的速度，单位：(度/s)/伏

typedef struct TaskAdjustMoveJogAngleVelocity
{
    short profile;                     // 当reserve1[1]为0时，表示需要调整角速度的MoveJog的规划器号，必须为MoveJog模式，当reserve1[1]为MC_DAC或者MC_AU_DAC时，表示需要调整的dac通道号
    short refType;                     // 参考源的类型
    short refIndex;                    // 参考源的索引，索引从1开始，没有对用户设置的数据做处理
    short refSubIndex;                 // 参考源的二级索引，不是所有参考源都有二级索引，索引从1开始
    short reserve1[4];

    double refPosition;                // 参考位置，用于与参考源单签位置相减，得到圆半径，单位：mm
    double refLinearVelocity;          // 参考源目标线速度，调整旋转轴角速度的目的是保持线速度维持这个值不变，单位：mm/s
    double minAngleVelocity;           // 角速度最小限制，低于最小值按照最小值调整，单位：度/s
    double maxAngleVelocity;           // 角速度最大限制，高于最大值按照最大值调整，单位：度/s
    double reserve2[4];
}TTaskAdjustMoveJogAngleVelocity;

typedef struct TaskSaveMcVarEx
{
	short fifo;
	short count;
	TWatchVar var[TASK_SAVE_MC_VAR_EX_MAX];
}TTaskSaveMcVarEx;
typedef struct Event
{
    unsigned long loop;
    TWatchVar var;
    unsigned short condition;
    double value;
} TEvent;

GT_API GT_ClearEvent(void);
GT_API GT_ClearTask(void);
GT_API GT_ClearEventTaskLink(void);
GT_API GT_AddEvent(TEvent* pEvent, short* pEventIndex);
GT_API GT_AddTask(short taskType, void* pTaskData, short* pTaskIndex);
GT_API GT_AddEventTaskLink(short eventIndex, short taskIndex, short* pLinkIndex);
GT_API GT_GetEventCount(short* pCount);
GT_API GT_GetEvent(short eventIndex, TEvent* pEvent);
GT_API GT_GetEventLoop(short eventIndex, unsigned long* pCount);
GT_API GT_GetTaskCount(short* pCount);
GT_API GT_GetTask(short taskIndex, short* pTaskType, void* pTaskData);
GT_API GT_GetEventTaskLinkCount(short* pCount);
GT_API GT_GetEventTaskLink(short linkIndex, short* pEventIndex, short* pTaskIndex);
GT_API GT_EventOn(short eventIndex, short count);
GT_API GT_EventOff(short eventIndex, short count);

GT_API GTN_ClearEvent(short core);
GT_API GTN_ClearTask(short core);
GT_API GTN_ClearEventTaskLink(short core);
GT_API GTN_DeleteEvent(short core, short eventIndex);
GT_API GTN_DeleteTask(short core, short taskIndex);
GT_API GTN_DeleteEventTaskLink(short core, short linkIndex);
GT_API GTN_AddEvent(short core, TEvent* pEvent, short* pEventIndex);
GT_API GTN_AddTask(short core, short taskType, void* pTaskData, short* pTaskIndex);
GT_API GTN_AddEventTaskLink(short core, short eventIndex, short taskIndex, short* pLinkIndex);
GT_API GTN_SetEvent(short core, TEvent* pEvent, short eventIndex);
GT_API GTN_SetTask(short core, short taskType, void* pTaskData, short taskIndex);
GT_API GTN_SetEventTaskLink(short core, short eventIndex, short taskIndex, short linkIndex);
GT_API GTN_GetEventCount(short core, short* pCount);
GT_API GTN_GetEvent(short core, short eventIndex, TEvent* pEvent);
GT_API GTN_GetEventLoop(short core, short eventIndex, unsigned long* pEventLoop);
GT_API GTN_GetTaskCount(short core, short* pCount);
GT_API GTN_GetTask(short core, short taskIndex, short* pTaskType, void* pTaskData);
GT_API GTN_GetEventTaskLinkCount(short core, short* pCount);
GT_API GTN_GetEventTaskLink(short core, short linkIndex, short* pEventIndex, short* pTaskIndex);
GT_API GTN_EventOn(short core, short eventIndex, short count);
GT_API GTN_EventOff(short core, short eventIndex, short count);
GT_API GTN_GetTaskSaveMcVarResult(short core, short taskIndex, TWatchVar* pVar, double* pValue, short count, short* pReadCount);
GT_API GTN_GetTaskSaveMcVarResultEx(short core,short index,TWatchVar *pVar,double *pValue,short count,short *pReadCount);
GT_API GTN_SetEventWaitTime(short core, double waitTime);


#define WAIT_TIMEOUT_MODE_INFINITY          (0)
#define WAIT_TIMEOUT_MODE_SKIP              (1)
#define WAIT_TIMEOUT_MODE_STOP              (2)

typedef struct WatchCondition
{
    TWatchVar var;
    unsigned short condition;
    double value;
} TWatchCondition;

typedef struct VarCalculate
{
    unsigned short operation;
    unsigned short varType;
    unsigned short result;
    unsigned short lhs;
    unsigned short rhs;
} TVarCalculate;

typedef struct VarCondition
{
    short varIndex;
    short reserve[3];
    TWatchCondition watchCondition;
} TVarCondition;

typedef struct WaitTimeout
{
    long time;				// 超时时间
    short mode;		        // 超时后的行为，0：无限等待，1：跳过当前等待操作继续执行指令流，2：停止指令流
    short reserve1;
    double reserve2[4];
} TWaitTimeout;

typedef struct ConditionTaskEnable
{
    short enable;
    short loop;
    short reserve[2];
} TConditionTaskEnable;

typedef struct CommandListStopParameter
{
    short halt;
    short reserve1[15];
} TCommandListStopParameter;

//停止指令流
typedef struct TaskStopCommandList
{
    short stopList;
    short stopMode;
    short triggerSource;
    short reserve[5];
    TCommandListStopParameter prm;
} TTaskStopCommandList;

GT_API GTN_SetVarBoolCondition(short core, short varIndex, TWatchCondition* pWatchCondition);
GT_API GTN_GetVarBoolCondition(short core, short varIndex, TWatchCondition* pWatchCondition);
GT_API GTN_LoadVarCalculate(short core, TVarCalculate* pVarCalculate, short count, TListInfo* pListInfo = NULL);
GT_API GTN_AddVarCalculate(short core, TVarCalculate* pVarCalculate, short* pIndex);
GT_API GTN_GetVarCalculate(short core, short index, TVarCalculate* pVarCalculate);
GT_API GTN_GetVarCalculateCount(short core, short* pCount);
GT_API GTN_SetVarCondition(short core, short varIndex, TWatchCondition* pWatchCondition, short conditionCount, short operation, TListInfo* pListInfo);
GT_API GTN_GetVarCondition(short core, short varIndex, TWatchCondition* pWatchCondition, short* pConditionCount, short* pOperation);
GT_API GTN_ClearVar(short core);

GT_API GTN_GetMcVar(short core, const TWatchVar* pVar, double* pValue);
GT_API GTN_SetMcVar(short core, const TWatchVar* pVar, double value, TListInfo* pListInfo);
GT_API GTN_GetMcVarEx(short core, const TWatchVar* pVar, double* pValue, short count);
GT_API GTN_SetMcVarEx(short core, const TWatchVar* pVar, double* pValue, short count, TListInfo* pListInfo);

GT_API GTN_WaitForCondition(short core, TWatchCondition* pWatchCondition, short conditionCount, short operation, TWaitTimeout* pTimeout, TListInfo* pListInfo = NULL);
GT_API GTN_GetWaitForCondition(short core, short list, TWatchCondition* pWatchCondition, short* pConditionCount, short* pOperation, TWaitTimeout* pTimeout, short* pConditionResult, short* pConditionDone);

GT_API GTN_SetConditionTask(short core, short conditionTaskindex, TWatchCondition* pWatchCondition, short conditionCount, short operation, short taskType, void* pTaskData, TConditionTaskEnable* pEnable, TListInfo* pListInfo);
GT_API GTN_GetConditionTask(short core, short conditionTaskIndex, TWatchCondition* pWatchCondition, short* pConditionCount, short* pOperation, short* pTaskType, void* pTaskData, TConditionTaskEnable* pEnable);
GT_API GTN_ConditionTaskEnable(short core, short conditionTaskIndex, TConditionTaskEnable* pEnable, TListInfo* pListInfo);
GT_API GTN_ConditionStopCommandList(short core, TWatchCondition* pWatchCondition, short conditionCount, short operation, TTaskStopCommandList* pTask, TListInfo* pListInfo);

GT_API GTN_GetAxisConditionTriggerPos(short core, short type, short axis, double* pValue, short count);

#define VAR_CALCULATE_OR					(1)
#define VAR_CALCULATE_AND					(3)
#define VAR_CALCULATE_NOT					(5)

#define VAR_CALCULATE_ADD					(11)
#define VAR_CALCULATE_SUB					(12)
#define VAR_CALCULATE_MUL					(13)
#define VAR_CALCULATE_DIV					(14)

typedef struct
{
   // 例如 result = leftOperands + rightOperands;
   TWatchVar leftOperands;                       // 左操作数变量信息
   TWatchVar rightOperands;                      // 右操作数变量信息
   TWatchVar result;                             // 计算结果存放的变量信息
   unsigned short operation;                     // 计算类型
   short reserve[6];                             // 保留值必须为0。
} TVariableCalculatePrm;

/**
 * @brief 设置进行基本运算的参数信息
 * @param core 核号
 * @param pPrm 进行运算变量参数
 * @param count 需要运算的数量,count为pPrm数组的大小。
 * @return 错误码
 0 执行成功。
 8 不支持该指令。
 11501 内部执行错误，读取左操作数的值错误。
 11502 内部执行错误，读取左操作数的值错误。
 11503 内部执行错误，将计算结果设置到result时出错SetVarValue。
 11504 内部执行错误，根据左操作数/右操作数进行运算时出错。
 17501 leftOperands左操作数leftOperands参数错误。
 17502 rightOperands右操作数参数错误。
 17503 result参数错误。
 17504 result变量信息中的type参数错误。
 17505 operation参数错误。
 17745 指针参数pPrm错误，指针不能为NULL。
 17751 count超过可以设的最大范围,最大数量为4，count为pPrm的数组大小。
 */
GT_API GTN_SetVariableCalculate(short core,TVariableCalculatePrm *pPrm,short count,TListInfo *pListInfo=NULL);

/*--------- -------------------------------------------------*/
/* Group                                                     */
/*-----------------------------------------------------------*/

#define COORD_SYSTEM_PCS				(0)
#define COORD_SYSTEM_MCS				(1)
#define COORD_SYSTEM_ACS				(2)
#define COORD_SYSTEM_TCS				(3)
#define COORD_SYSTEM_VCS				(4)
#define COORD_SYSTEM_FCS				(5)
#define COORD_SYSTEM_MVCS				(10)
#define COORD_SYSTEM_TCS_USER_DEFINE    (20)
#define COORD_SYSTEM_PCS_USER_DEFINE    (30)  // 用户定义工件坐标系，可用于实现倾斜面加工的应用

#define COORD_SYSTEM_POLAR              (40)
#define COORD_SYSTEM_CYNLINDER          (41)


#define ORI_MODE_NONE				    (0)
#define ORI_MODE_EULER				    (1)
#define ORI_MODE_QUAD				    (2)
#define ORI_MODE_VECTOR				    (3)
#define ORI_MODE_ROTATE_AXIS_POS	    (4)
#define ORI_MODE_SPACE_ROTATE           (5)

#define VEL_MODE_DEFAULT                (0)
#define VEL_MODE_PERCENT                (1)

#define ORI_PROFILE_MODE_MINOR          (0) // 插补指令起点到终点的姿态变化方向为劣弧
#define ORI_PROFILE_MODE_MAJOR          (1) // 插补指令起点到终点的姿态变化方向为优弧
#define ORI_PROFILE_MODE_COMMAND_DIR    (2) // 插补指令起点到终点的姿态变化方向根据插补指令设置的方向决定

#define GROUP_PROGRAM_COORD_SYSTEM_TABLE         (0)       // 工作台坐标系模式
#define GROUP_PROGRAM_COORD_SYSTEM_PIECE         (1)       // 工件坐标系模式

#define GROUP_PCS_ROTATE_AXIS_POS_MODE_ORI       (0)       // PCS下的旋转轴位置描述的是相对PCS坐标系的姿态
#define GROUP_PCS_ROTATE_AXIS_POS_MODE_DIRECT    (1)       // PCS下的旋转轴位置和MCS保持一致

#define PCS_USER_DEFINE_COORD_TRANS_COUNT_MAX    (4)       // 用户坐标系变换最大叠加个数

#define COORD_TRANS_TYPE_EULER          (1)
#define COORD_TRANS_TYPE_QUAD           (2)
#define COORD_TRANS_TYPE_VECTOR	        (3)
#define COORD_TRANS_TYPE_ROTATE_AXIS_POS (4)
#define COORD_TRANS_TYPE_ACS_POS        (5)
#define COORD_TRANS_TYPE_EULER_FIX      (11)
#define COORD_TRANS_TYPE_TOOL_DIR       (20)               // 刀具轴方向
#define COORD_TRANS_TYPE_OFFSET         (30)               // 偏移模式
#define COORD_TRANS_TYPE_POINTS         (40)               // 三点模式
#define COORD_TRANS_TYPE_TWO_VECTORS    (50)               // 两个矢量模式
#define COORD_TRANS_TYPE_PROJECT_ANGLE  (60)               // 投影角模式

#define GROUP_INCLINED_PLANE_MODE_ORI_UNCHANGED     (0)    // 定姿态斜面加工
#define GROUP_INCLINED_PLANE_MODE_ORI_CHANGED       (1)    // 变姿态斜面加工

#define EULER_MODE_ZYX                  (0)
#define EULER_MODE_ZXZ	                (3)
#define EULER_MODE_XYZ	                (4)

#define ORI_PROFILE_MODE_MINOR          (0) // 插补指令起点到终点的姿态变化方向为劣弧
#define ORI_PROFILE_MODE_MAJOR          (1) // 插补指令起点到终点的姿态变化方向为优弧
#define ORI_PROFILE_MODE_COMMAND_DIR    (2) // 插补指令起点到终点的姿态变化方向根据插补指令设置的方向决定
#define VEL_MODE_DEFAULT                (0)
#define VEL_MODE_PERCENT                (1)

#define GROUP_PROGRAM_COORD_SYSTEM_TABLE         (0)       // 工作台坐标系模式
#define GROUP_PROGRAM_COORD_SYSTEM_PIECE         (1)       // 工件坐标系模式

#define GROUP_PCS_ROTATE_AXIS_POS_MODE_ORI       (0)       // PCS下的旋转轴位置描述的是相对PCS坐标系的姿态
#define GROUP_PCS_ROTATE_AXIS_POS_MODE_DIRECT    (1)       // PCS下的旋转轴位置和MCS保持一致
#define KIN_TYPE_ORTHOGONAL             (0)
#define KIN_TYPE_PARALLEL               (2)
#define KIN_TYPE_ORTHOGONAL_EXTEND      (6)
#define KIN_TYPE_ROBOT                  (10)
#define KIN_TYPE_FIVE_AXIS              (20)
#define KIN_TYPE_MULTI_AXIS             (30)
#define KIN_TYPE_CYLINDER               (40)

#define ROBOT_TYPE_SCARA                (0)
#define ROBOT_TYPE_SIX_REVOLUTE         (1)
#define ROBOT_TYPE_FOUR_REVOLUTE        (2)
#define ROBOT_TYPE_PALLETIZE            (3)
#define ROBOT_TYPE_POSITIONER           (4)    //变位机
#define ROBOT_TYPE_FIVE_REVOLUTE        (5)    //5R机器人
#define ROBOT_TYPE_XYZRR                (6)    //XYZRR
#define ROBOT_TYPE_SEVEN_REVOLUTE       (7)    //7R
#define ROBOT_TYPE_UR                   (10)   //UR机器人
#define ROBOT_TYPE_CYLINDER             (20)   //圆柱机器人
#define ROBOT_TYPE_SCARA_EXTEND         (30)   //scara扩展模型
#define CYLINDER_TYPE_NORMAL            (0)
#define ROBOT_TYPE_DELTA                (100)  //Delta机器人
#define DELTA_TYPE_BASIC                (0)    //简化模型，4个参数
#define DELTA_TYPE_COMMON               (1)    //通用模型，9个参数
#define DELTA_TYPE_COMMON_PRO           (2)    //通用模型，考虑了误差的模型，20个参数

#define SCARA_TYPE_RRPR                 (0)
#define SCARA_TYPE_PRRR                 (1)
#define SCARA_TYPE_RPRR                 (2)
#define SCARA_TYPE_RRRP                 (3)

#define PALLETIZE_TYPE_RPPR             (0)

// 变位机ROBOT_TYPE_POSITIONER子模式
#define POSITIONER_TYPE_SINGLE          (0)     //单轴变位机
#define POSITIONER_TYPE_DUAL            (10)    //双轴变位机

// 五轴机器人ROBOT_TYPE_XYZRR的子模式
#define XYZRR_TYPE_LIKE_SCARA           (0)     // XYZ+SCARA
#define XYZRR_TYPE_DT_C_ON_A            (100)   // 类似标准五轴双摆头机器人

// 平行轴子模式
#define PARALLEL_INDEPENDENT             (0)    // 平行轴之间为独立关系
#define PARALLEL_SUPERPOSITION           (1)    // 平行轴之间为叠加关系

// 五轴类型
#define  RW_C_ON_B                      (0) //双转台：B 为第一旋转轴，C 为第二旋转轴
#define  RW_B_ON_A                      (1) //双转台：A 为第一旋转轴，B 为第二旋转轴
#define  RW_A_ON_B                      (2) //双转台：B 为第一旋转轴，A 为第二旋转轴
#define  RW_C_ON_A                      (3) //双转台：A 为第一旋转轴，C 为第二旋转轴
#define  DT_B_ON_A                      (4) //双摆头：A 为第一旋转轴，B 为第二旋转轴
#define  DT_A_ON_B                      (5) //双摆头：B 为第一旋转轴，A 为第二旋转轴
#define  DT_A_ON_C                      (6) //双摆头：C 为第一旋转轴，A 为第二旋转轴
#define  DT_B_ON_C                      (7) //双摆头：C 为第一旋转轴，B 为第二旋转轴
#define  T_A_W_B                        (8) //转台摆头：A 为第一旋转轴，B 为第二旋转轴
#define  T_B_W_A                        (9) //转台摆头：B 为第一旋转轴，A 为第二旋转轴
#define  T_A_W_C                        (10) //转台摆头：A 为第一旋转轴，C 为第二旋转轴
#define  T_B_W_C                        (11) //转台摆头：B 为第一旋转轴，C 为第二旋转轴
#define  RW_A_ON_C                      (20) //双转台：C 为第一旋转轴，A 为第二旋转轴
#define  RW_B_ON_C                      (21) //双转台：C 为第一旋转轴，B 为第二旋转轴
typedef struct RobotKinematicParameter
{
    short type;
    short subType;          //子类型
    short dir[8];           //关节方向，0:与定义方向同向，1：与定义方向反向
    short reserve1[6];
    double prm[20];         //结构参数，杆长
    double offset[8];       //关节偏移，实际初始姿态与定义的初始姿态的偏移(实际初始姿态在定义坐标系下的位置)
                            //单位：旋转轴：角度，直线轴：毫米
    long reserve2[10];
    double reserve3[11];
}TRobotKinematicParameter;

typedef struct FiveAxisKinematicParameter
{
    short type;                          //机床类型
    short reserve1[3];                   //保留参数
    double primaryAxisPoint[3];          //第一旋转轴中心在MCS的坐标
    double slaveAxisPoint[3];            //第二旋转轴中心在MCS的坐标
    double toolLocationPoint[3];         //刀具坐标系中心在MCS的坐标
    short dirMode;                       //方向描述模式
    short reserve2[2];                   //保留参数
    short dir[5];                        //各轴方向
    double axisVector[5][3];             //各轴轴线方向
    long reserve3[10];
    double reserve4[16];
}TFiveAxisKinematicParameter;

typedef struct MultiAxisKinematicParameter
{
    short axisCount;                     //轴数
    short reserve1[3];                   //保留参数
    long reserve2[10];
    double reserve3[42];
}TMultiAxisKinematicParameter;

typedef struct ParallelParameter
{
    short subType;                       //0:平行轴为独立关系，1：平行轴为叠加关系
    short axisCount;                     //轴数
    short mode;
    short majorAxis[3];
    short reserve1[6];
    short parallelAxis[8];
    long reserve2[10];
    double reserve3[38];
}TParallelParameter;

// 三轴扩展模型参数
typedef struct OrthogonalExtendParameter
{
    short axisCount;                     // 轴数
    short reserve1[3];                   // 保留参数
    long reserve2[10];
    double reserve3[42];
}TOrthogonalExtendParameter;

typedef union KinematicParameter
{
    TOrthogonalExtendParameter orthogonalExtend;
    TRobotKinematicParameter robot;
    TFiveAxisKinematicParameter fiveAxis;
    TMultiAxisKinematicParameter multiAxis;
    TParallelParameter parallel;
    double data[48];
} TKinematicParameter;

typedef struct KinematicTransform
{
    short type;                    //结构类型
    short reserve[3];
    TKinematicParameter kinPrm;    //结构参数
}TKinematicTransform;

typedef struct GroupMotionConstraint
{
    double velMax;
    double accMax;
    double decMax;
    double jerkMax;
    double reserve[10];
} TGroupMotionConstraint;

typedef struct GroupOrientationConstraint
{
    double oriVelMax;
    double oriAccMax;
    double oriDecMax;
    double oriJerkMax;
    double reserve[10];
} TGroupOrientationConstraint;

typedef struct CartesianParameter
{
    double transX;
    double transY;
    double transZ;
    double rotAngle1;
    double rotAngle2;
    double rotAngle3;
}TCartesianParameter;

typedef struct CartesianTransformVelConstraint
{
    double vel;				        // 位置速度，单位mm/s
    double oriVel;                  // 姿态速度，单位度/s
    double reserve[18];
} TCartesianTransformVelConstraint;

typedef union CartesianTransformConstraintUnion
{
    TCartesianTransformVelConstraint velConstraint;
    double value[20];
} TCartesianTransformConstraintUnion;

typedef struct CartesianTransformConstraint
{
    short mode;			// 模式
    short reserve[3];
    TCartesianTransformConstraintUnion data;
} TCartesianTransformConstraint;

typedef struct CoordinateTransformAcsPos
{
    double prm[8];
    double reserve[12];
}TCoordinateTransformAcsPos;

typedef struct CoordinateTransformToolDir
{
    double transX;             // 相对PCS原点的X轴平移量
    double transY;             // 相对PCS原点的Y轴平移量
    double transZ;             // 相对PCS原点的Z轴平移量
    double rotAngle1;          // 刀具侧第一个旋转轴的旋转角
    double rotAngle2;          // 刀具侧第二个旋转轴的旋转角
    double alpha;              // 绕Z轴旋转的角度
    double reserve[14];
}TCoordinateTransformToolDir;

typedef struct CoordinateTransformOffset
{
    double prm[8];
    double reserve[12];
}TCoordinateTransformOffset;

// 根据平面内的三个点指定初始特征坐标系，再在初始特征坐标系上进行平移和旋转得到最终的特征坐标系
typedef struct CoordinateTransformPoints
{
    double point1[3];          // 第1点，初始特征坐标系的零点，坐标值为相对原始PCS的位置
    double point2[3];          // 第2点，初始特征坐标系X轴正方向上的点，坐标值为相对原始PCS的位置
    double point3[3];          // 第3点，坐标值为相对原始PCS的位置
    double offset[3];          // 最终特征坐标系的原点相对第1点的偏移量，偏移值为相对初始特征坐标系的位置
    double gama;               // 绕特征坐标系Z轴旋转的角度
    double reserve[7];
}TCoordinateTransformPoints;

// 根据两个矢量指定坐标系，通过Z矢量方向和X矢量方向的矢量积得到Y轴方向，再根据XY的右手法则得到Z方向
typedef struct CoordinateTransformTwoVectors
{
    double offset[3];          // 特征坐标系的原点相对原始PCS的位置
    double vector1[3];         // 特征坐标系X轴正向的矢量方向，单位为量纲1
    double vector2[3];         // 特征坐标系Z轴正向的矢量方向，单位为量纲1
    double reserve[11];
}TCoordinateTransformTwoVectors;

// 根据投影角指定坐标系
typedef struct CoordinateTransformProjectAngle
{
    double offset[3];          // 特征坐标系的原点相对原始PCS的位置
    double alpha;              // X轴绕原始坐标系的Y轴旋转的角度
    double beta;               // Y轴绕原始坐标系的X轴旋转的角度
    double gama;               // 绕特征坐标系Z轴旋转的角度
    double reserve[14];
}TCoordinateTransformProjectAngle;

typedef union CoordinateTransformUnion
{
    TCoordinateTransformAcsPos acsPos;
    TCartesianParameter euler;
    TCoordinateTransformToolDir toolDir;
    TCoordinateTransformOffset offset;
    TCoordinateTransformPoints points;
    TCoordinateTransformTwoVectors twoVectors;
    TCoordinateTransformProjectAngle projectAngle;
    double value[20];
} TCoordinateTransformUnion;

#define COORDINATE_TRANSFORM_RESERVE_SUB_MODE    (0)

typedef struct CoordinateTransform
{
    short mode;
    short reserve[3];
    TCoordinateTransformUnion data;
}TCoordinateTransform;

// 动态坐标系变换参数
typedef struct DynamicCoordinateTransformData
{
    short masterType;
    short masterIndex;
    short masterMoveType;
    short trackMode;
    double originPrfPos;
    TCartesianParameter masterOrigin;
    TCartesianParameter pcsToMasterTcs;
    TCartesianParameter dynamicPcsToSlaveMcs;            //设置指令不用设置该参数，用于读取实时的动态PCS信息
}TDynamicCoordinateTransformData;

typedef union DynamicCoordinateTransformUnion
{
    TDynamicCoordinateTransformData trans;
    double reserve[60];
}TDynamicCoordinateTransformUnion;

typedef struct DynamicCoordinateTransform
 {
    short mode;
    short pad[3];
    TDynamicCoordinateTransformUnion prm;
}TDynamicCoordinateTransform;

typedef struct DynamicCoordinateTransformMasterPos
{
    short masterType;                // 主轴类型
    short masterIndex;               // 主轴索引
    short reserve1[7];
    short commandPosCoord;           // 位置描述坐标系
    short commandOrientationMode;    // 位置描述姿态
    short commandConfigIndex;        // 位置构型解
    double pos[8];                   // 位置
    long  reserve2[8];
    double reserve3[8];
}TDynamicCoordinateTransformMasterPos;

// 动力学补偿参数
typedef struct GroupDynamicsCompensate
{
    short source;
    short reserve1[7];
    short enableFriction[8];
    short enableCompensate[8];
    long reserve2[8];
    double reserve3[8];
}TGroupDynamicsCompensate;

typedef struct GroupCoupleParameter
{
	short master;
	short slave;
	short reserve[2];
	double alpha;
	double beta;;
}TGroupCoupleParameter;

typedef struct GroupSoftLimit
{
	short positiveLimitEnable;
	short negativeLimitEnable;

	short positiveLimitMode;
	short negativeLimitMode;

	double positiveLimit;
	double negativeLimit;
} TGroupSoftLimit;

#define	GROUP_STATE_DISABLED			(0)
#define	GROUP_STATE_ERROR_STOP			(1)
#define	GROUP_STATE_STANDBY				(6)
#define	GROUP_STATE_HOMING				(10)
#define	GROUP_STATE_MOVING				(20)
#define	GROUP_STATE_STOPPING			(30)
#define GROUP_STATUS_RESERVE1_ERROR_STOP_STATE        (0)

typedef struct GroupStatus
{
	short run;
	short state;
	short stopInfo;
	short reserve1[13];
	long  reserve2[4];
} TGroupStatus;

typedef struct GroupMotionSmooth
{
	double time;				// 加速度变化时间
	double k;					// 形态
} TGroupMotionSmooth;

typedef union GroupMotionSmoothUnion
{
	TGroupMotionSmooth smooth;
	double value[20];
} TGroupMotionSmoothUnion;

typedef struct GroupMotionSmoothPrm
{
	short mode;
	short reserve[3];
	TGroupMotionSmoothUnion data;
} TGroupMotionSmoothPrm;

#define GROUP_CONTOUR_ERROR_CONTROL_RESERVE2_MAX_COMPENSATE_VALUE  (0)
typedef struct
{
    short feedback;
    short reserve1[3];
    double percent;
    double reserve2[4];
} TGroupContourErrorControlPrm;

typedef struct GroupMoveSegmentInfo
{
    long userTag;
    long segNum;
    double segmentTravel;
    double segmentLength;
    double totalTravel;
    short reserve1[8];
    double reserve2[8];
}TGroupMoveSegmentInfo;

#define GROUP_PATH_REF_AXIS_X          (0)       // X轴标识
#define GROUP_PATH_REF_AXIS_Y          (1)       // Y轴标识
#define GROUP_PATH_REF_AXIS_Z          (2)       // Z轴标识

typedef struct GroupPathRefAxis
{
    short enable;                       // 0: 不使能，1：使能
    short reserve[3];                   // 保留值
    short refAxis[8];                   // 轨迹参考轴
}TGroupPathRefAxis;

#define OVERRIDE_TYPE_VEL                        (0)
#define OVERRIDE_TYPE_ACC                        (1)

GT_API GTN_AddAxisToGroup(short core,short group,short profile,short identInGroup,TListInfo *pListInfo=NULL);
GT_API GTN_RemoveAxisFromGroup(short core,short group,short identInGroup,TListInfo *pListInfo=NULL);
GT_API GTN_UngroupAllAxes(short core,short group,TListInfo *pListInfo=NULL);
GT_API GTN_GroupEnable(short core,short group,TListInfo *pListInfo=NULL);
GT_API GTN_GroupDisable(short core,short group,TListInfo *pListInfo=NULL);
GT_API GTN_SetGroupVelProfileMode(short core,short group,TVelProfileMode *pSmooth,TListInfo *pListInfo=NULL);
GT_API GTN_GetGroupVelProfileMode(short core,short group,TVelProfileMode *pSmooth);
GT_API GTN_GroupStop(short core,short group,TListInfo *pListInfo=NULL);
GT_API GTN_ClearGroupStatus(short core,short group,TListInfo *pListInfo=NULL);
GT_API GTN_GetGroupStatus(short core,short group,TGroupStatus *pStatus);
GT_API GTN_GetGroupProfilePos(short core,short group,short index,double *pValue,short count=1,short coordSystem=COORD_SYSTEM_ACS,short oriMode=ORI_MODE_NONE);
GT_API GTN_GetGroupProfileVel(short core,short group,short index,double *pValue,short count=1,short coordSystem=COORD_SYSTEM_ACS);
GT_API GTN_GetGroupTargetPos(short core,short group,short index,double *pValue,short count=1,short coordSystem=COORD_SYSTEM_ACS,short oriMode=ORI_MODE_NONE);
GT_API GTN_GetGroupSyntheticVel(short core,short group,double *pValue,short coordSystem=COORD_SYSTEM_ACS);
GT_API GTN_GetGroupBreakPos(short core,short group,short index,double *pValue,short count=1,short coordSystem=COORD_SYSTEM_ACS,short oriMode=ORI_MODE_NONE);
GT_API GTN_GetGroupMoveSegmentInfo(short core,short group,TGroupMoveSegmentInfo *pInfo);
GT_API GTN_GetGroupKinematicConfigIndex(short core,short group,short *pConfigIndex);
GT_API GTN_GetGroupConditionTriggerPos(short core,short group,short index,double *pValue,short count,short coordSystem=COORD_SYSTEM_ACS,short oriMode=ORI_MODE_NONE);
GT_API GTN_SetGroupLinkCommandList(short core,short group,unsigned long linkListMask);
GT_API GTN_GetGroupLinkCommandList(short core,short group,unsigned long *pLinkListMask);
GT_API GTN_SetGroupMotionConstraint(short core, short group, TGroupMotionConstraint* pPrm, TListInfo* pListInfo = NULL);
GT_API GTN_GetGroupMotionConstraint(short core, short group, TGroupMotionConstraint* pPrm);
GT_API GTN_SetGroupOrientationConstraint(short core, short group, TGroupOrientationConstraint* pPrm, TListInfo* pListInfo = NULL);
GT_API GTN_GetGroupOrientationConstraint(short core, short group, TGroupOrientationConstraint* pPrm);
GT_API GTN_SetGroupCartesianTransform(short core, short group, short coordSystem, short enable, TCartesianParameter* pPrm, TListInfo* pListInfo = NULL);
GT_API GTN_GetGroupCartesianTransform(short core, short group, short coordSystem, short* pEnable, TCartesianParameter* pPrm);
GT_API GTN_SetGroupCoordinateTransform(short core, short group, short coordSystem, short enable, TCoordinateTransform* pPrm, TListInfo* pListInfo = NULL);
GT_API GTN_GetGroupCoordinateTransform(short core, short group, short coordSystem, short* pEnable, TCoordinateTransform* pPrm);
GT_API GTN_SetGroupCoordinateTransformPrm(short core, short group, short coordSystem, short enable, TCoordinateTransform* pPrm, short count = 1, TListInfo* pListInfo = NULL);
GT_API GTN_GetGroupCoordinateTransformPrm(short core, short group, short coordSystem, short* pEnable, TCoordinateTransform* pPrm, short* pCount);
GT_API GTN_SetGroupCartesianTransformConstraint(short core, short group, short coordSystem, short enable, TCartesianTransformConstraint* pConstraint, TListInfo* pListInfo = NULL);
GT_API GTN_GetGroupCartesianTransformConstraint(short core, short group, short coordSystem, short* pEnable, TCartesianTransformConstraint* pConstraint);



GT_API GTN_SetGroupDynamicCoordinateTransform(short core, short group, short enable, TDynamicCoordinateTransform* pTrans, TListInfo* pListInfo = NULL);
GT_API GTN_GetGroupDynamicCoordinateTransform(short core, short group, short* pEnable, TDynamicCoordinateTransform* pTrans);
GT_API GTN_SetGroupDynamicCoordinateParameter(short core, short group, short mode, void* pPrm, TListInfo* pListInfo = NULL);
GT_API GTN_GetGroupDynamicCoordinateParameter(short core, short group, short mode, void* pPrm);
GT_API GTN_SetGroupKinematicTransform(short core, short group, TKinematicTransform* pTransform, TListInfo* pListInfo = NULL);
GT_API GTN_GetGroupKinematicTransform(short core, short group, TKinematicTransform* pTransform);

GT_API GTN_SetGroupDynamicsParameter(short core, short group, short count, double* pPrm, TListInfo* pListInfo = NULL);
GT_API GTN_GetGroupDynamicsParameter(short core, short group, short* pCount, double* pPrm);
GT_API GTN_SetGroupDynamicsCompensate(short core,short group,short mode,TListInfo *pListInfo=NULL);
GT_API GTN_GetGroupDynamicsCompensate(short core,short group,short *pMode);
GT_API GTN_SetGroupDynamicsSource(short core,short group,short source,TListInfo *pListInfo=NULL);
GT_API GTN_GetGroupDynamicsSource(short core,short group,short *pSource);
GT_API GTN_SetGroupDynamicCoordinateMasterPos(short core, short group, short enable, TDynamicCoordinateTransformMasterPos* pMasterPos);

GT_API GTN_SetGroupAcsOriginPos(short core,short group,double originPos[],TListInfo *pListInfo=NULL);
GT_API GTN_GetGroupAcsOriginPos(short core,short group,short index,double originPos[],short count);
GT_API GTN_SetGroupAcsKinematicOffset(short core,short group,double offset[],TListInfo *pListInfo=NULL);
GT_API GTN_GetGroupAcsKinematicOffset(short core,short group,short index,double *pValue,short count);
GT_API GTN_SetGroupAcsVelLimit(short core,short group,short motionMode,short enable,double *pAcsVelLimit,TListInfo *pListInfo=NULL);
GT_API GTN_GetGroupAcsVelLimit(short core,short group,short motionMode,short *pEnable,double *pAcsVelLimit);
GT_API GTN_SetGroupAcsMoveMode(short core,short group,short index,short mode,TListInfo *pListInfo=NULL);
GT_API GTN_GetGroupAcsMoveMode(short core,short group,short index,short *pMode);
GT_API GTN_SetGroupCouple(short core,short group,short count,TGroupCoupleParameter *pGroupCouplePrm,TListInfo *pListInfo=NULL);
GT_API GTN_GetGroupCouple(short core,short group,short *pCount,TGroupCoupleParameter *pGroupCouplePrm);
GT_API GTN_GetGroupCouplePos(short core,short group,short index,double *pValue,short count=1);
GT_API GTN_SetGroupVelModifyByTaskMode(short core,short group,short mode,TListInfo *pListInfo=NULL);
GT_API GTN_GetGroupVelModifyByTaskMode(short core,short group,short *pMode);

GT_API GTN_SetGroupCommandPosDefine(short core,short group,short coordSystem,short orientationMode,short configIndex,TListInfo *pListInfo=NULL);
GT_API GTN_GetGroupCommandPosDefine(short core,short group,short *pCoordSystem,short *pOrientationMode,short *pConfigIndex);
GT_API GTN_SetGroupCommandVelDefine(short core,short group,short type,short mode,TListInfo *pListInfo=NULL);
GT_API GTN_GetGroupCommandVelDefine(short core,short group,short type,short *pMode);
GT_API GTN_SetGroupProfileCoordinateSystem(short core,short group,short coordSystem,TListInfo *pListInfo=NULL);
GT_API GTN_GetGroupProfileCoordinateSystem(short core,short group,short *pCoordSystem);
GT_API GTN_SetGroupOrientationProfileMode(short core,short group,short profileMode,TListInfo *pListInfo=NULL);
GT_API GTN_GetGroupOrientationProfileMode(short core,short group,short *pProfileMode);
GT_API GTN_SetGroupOrientationMotionRatio(short core,short group,double ratio,TListInfo *pListInfo=NULL);
GT_API GTN_GetGroupOrientationMotionRatio(short core,short group,double *pRatio);
GT_API GTN_SetGroupOrientationMotionMode(short core,short group,short mode,TListInfo *pListInfo=NULL);
GT_API GTN_GetGroupOrientationMotionMode(short core,short group,short *pMode);
GT_API GTN_SetGroupProgramCoordinateSystem(short core,short group,short mode,TListInfo *pListInfo=NULL);
GT_API GTN_GetGroupProgramCoordinateSystem(short core,short group,short *pMode);
GT_API GTN_SetGroupPcsRotateAxisPosMode(short core,short group,short mode,TListInfo *pListInfo=NULL);
GT_API GTN_GetGroupPcsRotateAxisPosMode(short core,short group,short *pMode);
GT_API GTN_SetGroupCommandVelRefAxis(short core,short group,long velAxisMask,TListInfo *pListInfo=NULL);
GT_API GTN_GetGroupCommandVelRefAxis(short core,short group,long *pVelAxisMask);
GT_API GTN_SetGroupCommandVelRefRatio(short core,short group,short ident,double *pRatio,short count=1,TListInfo *pListInfo=NULL);
GT_API GTN_GetGroupCommandVelRefRatio(short core,short group,short ident,double *pRatio,short count=1);
GT_API GTN_SetGroupPathRefAxis(short core,short group,TGroupPathRefAxis *pPathRefAxis,TListInfo *pListInfo=NULL);
GT_API GTN_SetGroupMaxOverride(short core,short group,short type,double override,TListInfo *pListInfo=NULL);
GT_API GTN_GetGroupMaxOverride(short core,short group,short type,double *pMaxOverride);
GT_API GTN_SetGroupSoftLimit(short core,short group,short coordSystem,short index,TGroupSoftLimit *pPrm,TListInfo *pListInfo=NULL);
GT_API GTN_GetGroupSoftLimit(short core,short group,short coordSystem,short index,TGroupSoftLimit *pPrm);
GT_API GTN_SetGroupMotionSmooth(short core,short group,TGroupMotionSmoothPrm *pPrm,TListInfo *pListInfo=NULL);
GT_API GTN_GetGroupMotionSmooth(short core,short group,TGroupMotionSmoothPrm *pPrm);
GT_API GTN_SetGroupContourErrorControl(short core,short group,short enable,TGroupContourErrorControlPrm *pPrm,TListInfo *pListInfo=NULL);
GT_API GTN_GetGroupContourErrorControl(short core,short group,short *pEnable,TGroupContourErrorControlPrm *pPrm);
GT_API GTN_SetGroupCircularParameter(short core,short group,short type,void *pData,TListInfo *pListInfo=NULL);
GT_API GTN_GetGroupCircularParameter(short core,short group,short type,void *pData);
GT_API GTN_SetGroupErrorStopRatio(short core,short group,double errorStopRatio);
GT_API GTN_GetGroupErrorStopRatio(short core,short group,double *pErrorStopRatio);

typedef struct CoordinateTransformParameter
{
    TKinematicTransform kinTrans;

    short enablePcsTrans;
    short enableTcsTrans;
    short reserve1[2];
    TCartesianParameter pcsTrans;
    TCartesianParameter tcsTrans;
    double acsKinOffset[8];

    short inputCoordSystem;
    short inputOriMode;
    short configIndex;
    short outputOriMode;
    double preAcsPos[8];

    short reserve2[4];
    double reserve3[8];
}TCoordinateTransformParameter;

typedef struct CoordinateTransformParameterEx
{
    TKinematicTransform kinTrans;

    short enablePcsTrans;
    short enableTcsTrans;
    short enableUserPcsTrans;
    short reserve1[5];
    TCoordinateTransform pcsTrans;
    TCoordinateTransform tcsTrans;
    TCoordinateTransform userPcsTrans;
    double acsKinOffset[8];

    short inputCoordSystem;
    short inputOriMode;
    short configIndex;
    short outputOriMode;
    double preAcsPos[8];

    short reserve2[4];
    double reserve3[8];
}TCoordinateTransformParameterEx;

#define GROUP_TRANSFORM_PRM_RESERVE2_ACS_POS_TYPE        (0)
#define GROUP_TRANSFORM_PRM_RESERVE2_PROGRAM_MODE        (2)
typedef struct GroupPosTransformPrm
{
    TKinematicTransform kinTrans;

    short enablePcsTrans;
    short enableTcsTrans;
    short enableUserPcsTrans;
    short userPcsTransCount;
    short inclinedPlaneMode;
    short inclinedPlaneConfigIndex;
    short reserve1[2];
    TCoordinateTransform pcsTrans;
    TCoordinateTransform tcsTrans;
    TCoordinateTransform userPcsTrans[4];
    double acsKinOffset[8];
    double preRotateAxisPos[2];

    short inputCoordSystem;
    short inputOriMode;
    short configIndex;
    short outputOriMode;
    double preAcsPos[8];

    short reserve2[4];
    double reserve3[8];
}TGroupPosTransformPrm;

typedef struct CoordinatePos
{
    double pcsPos[8];
    double mcsPos[8];
    double acsPos[8];
}TCoordinatePos;

GT_API GTN_GroupCoordinateTransform(short core, TCoordinateTransformParameter* pPrm, double* pInputPos, TCoordinatePos* pCoordPos, short* pConfigIndex);
GT_API GTN_GroupCoordinateTransformEx(short core, TCoordinateTransformParameterEx* pPrm, double* pInputPos, TCoordinatePos* pCoordPos, short* pConfigIndex);
GT_API GTN_UTL_GroupPosTransform(short core, TGroupPosTransformPrm* pPrm, double* pInputPos, TCoordinatePos* pCoordPos, short* pConfigIndex);

#define GROUP_TRANSFORM_INPUT_RESERVE1_ACS_POS_TYPE        (7)
typedef struct GroupPosTransformInput
{
	short coordSystem;
	short oriMode;
	short configIndex;
	short targetOriMode;
	short reserve1[8];
	double inputPos[8];
	double preAcsPos[8];
	double reserve2[24];
}TGroupPosTransformInput;

typedef struct GroupPosTransformOutput
{
	short configIndex;
	short singularity;
	short reserve1[10];
	double pcsPos[8];
	double mcsPos[8];
	double acsPos[8];
	double reserve2[16];
}TGroupPosTransformOutput;

GT_API GTN_GroupPosTransform(short core,short group,TGroupPosTransformInput *pInput,TGroupPosTransformOutput *pOutput);
typedef struct GroupStopParameter
{
    double deceleration;
    double jerk;
    short reserve1[4];
    double reserve2[5];
} TGroupStopParameter;

#define GROUP_INDENT_MAX                         (8)

// 插补模式
#define GROUP_PATH_MODE_GENERAL                  (0)     // 通用插补模式
#define GROUP_PATH_MODE_POLAR                    (1)     // 极插补模式
#define GROUP_PATH_MODE_CYNLINDER                (2)     // 圆柱插补模式

// 插补平面
#define GROUP_PATH_PLANE_XY                      (0)
#define GROUP_PATH_PLANE_YZ                      (1)
#define GROUP_PATH_PLANE_ZX                      (2)

typedef struct GroupPathPolar
{
    short plane;                       // 插补平面
    short reserve1[5];
    short rotateIdent;                 // 旋转轴在group中的轴号
    short setOriginFlag;               // 设置零点标识，0：极坐标系的零点位于工件坐标系的零点，1：极坐标系的零点通过originPos设置
    double origin[3];                  // 极坐标系的零点相对MCS也就是理论ACS的偏移量
    double rotateOrigin;               // 旋转轴零点，用于设置位于极坐标系X轴时旋转轴的理论ACS位置
	double reserve2[14];
} TGroupPathPolar;

typedef struct GroupPathCynlinder
{
    short plane;                       // 插补平面
    short reserve1[4];
    short linearIdent;                 // 直线轴在group中的轴号
    short rotateIdent;                 // 旋转轴在group中的轴号
    short setOriginFlag;               // 设置零点标识，0：圆柱坐标系的零点位于工件坐标系的零点，1：圆柱坐标系的零点通过originPos设置
    double r;                          // 圆柱半径
    double origin[3];                  // 圆柱坐标系的零点相对MCS也就是理论ACS的偏移量
    double rotateOrigin;               // 旋转轴零点，用于设置位于圆柱坐标系零点时旋转轴的理论ACS位置
	double reserve2[13];
} TGroupPathCynlinder;

typedef union GroupPathUnion
{
    TGroupPathPolar polar;
    TGroupPathCynlinder cynlinder;
    double value[20];
} TGroupPathUnion;

typedef struct GroupPathPrm
{
    short mode;
    short reserve[3];
    TGroupPathUnion data;
} TGroupPathPrm;


GT_API GTN_SetGroupStopParameter(short core, short group, short type, TGroupStopParameter* pPrm, TListInfo* pListInfo = NULL);
GT_API GTN_GetGroupStopParameter(short core, short group, short type, TGroupStopParameter* pPrm);
GT_API GTN_SetGroupPathMode(short core, short group, TGroupPathPrm* pPrm, TListInfo* pListInfo = NULL);
GT_API GTN_GetGroupPathMode(short core, short group, TGroupPathPrm* pPrm);

// GTN_SetGroupLookAheadFunc
#define LA_FUNC_MODE_DV_MAX_LIMIT      (1)       // 设置dvmax约束的相关参数
#define LA_FUNC_MODE_SOFTLIMIT_CHECK   (2)       // 软限位检测
#define LA_FUNC_MODE_DV_MAX_THRESHOLD  (3)       // 设置dvmax约束的阈值
#define LA_FUNC_MODE_GEOMETRY_LIMIT    (4)       // 设置几何约束的生效模式，0：工件坐标系，1：轴坐标系，2：曲率和夹角都不约束
#define LA_FUNC_MODE_CMD_VEL_LIMIT     (50)      // 根据用户压指令的速度来约束目标速度功能的相关参数
#define LA_FUNC_MODE_LA_PRM_TYPE       (99)      // 设置的前瞻参数类型，0：时间常数和曲率系数，1：误差和曲率系数

typedef struct GroupLookAheadParameter
{
    long lookAheadNum;					//前瞻段数
    short reserve1[6];
    long reserve2[4];
    double time;						//时间常数
    double radiusRatio;					//曲率限制调节参数
    double reserve3[7];
}TGroupLookAheadParameter;

typedef struct LookAheadErrorInfo
{
    long segNum;
    long userTag;
    double data[32];
}TLookAheadErrorInfo;

#define BLEND_MODE_NONE                  0       // 无轨迹过渡
#define BLEND_MODE_ARC                   1       // 圆弧过渡
#define BLEND_MODE_BIARC                 2       // 双圆狐过渡
#define BLEND_MODE_SPLINE                3       // 样条过渡

#define BLEND_PARA_TYPE_ERROR            0       // 过渡参数为误差值
#define BLEND_PARA_TYPE_RADIUS           1       // 过渡参数为半径
#define BLEND_PARA_TYPE_DISTANCE         2       // 过渡参数为与过渡点的距离

#define VEL_BLEND_MODE_NONE              0       // 不进行速度blending，以第一段的速度过渡，且不进行轨迹blending
#define VEL_BLEND_MODE_BUFFERER          1       // 减速到0，且不进行轨迹blending
#define VEL_BLEND_MODE_LOW               2       // 以两段中最低的速度过渡，根据轨迹blending参数进行轨迹过渡
#define VEL_BLEND_MODE_PREVIOUS          3       // 以第一段的速度过渡，根据轨迹blending参数进行轨迹过渡
#define VEL_BLEND_MODE_NEXT              4       // 以第二段的速度过渡，根据轨迹blending参数进行轨迹过渡
#define VEL_BLEND_MODE_HIGH              5       // 以两段中最高的速度过渡，根据轨迹blending参数进行轨迹过渡

typedef struct PathBlendingParameter
{
    short blendType;
    short prmType;
    short reserve1[2];
    double prm;
    double reserve2[8];
    double minAngle;
    double maxAngle;
} TPathBlendingParameter;

typedef struct VelBlendingParameter
{
    short blendType;                   // 速度blending的类型
    short prmType;                     // 参数类型
    short reserve[5];
    short percentMode;                 // 速度百分比模式，0：过渡速度为由blendType决定的速度，1：过渡速度为由blendType决定的速度*设置的百分比。
    double percent;                    // 速度百分比，在percentMode为1时生效，范围：[0,1]。
    double prm[17];
} TVelBlendingParameter;

typedef union BlendingParameter
{
    TPathBlendingParameter pathBlendingPrm;
    TVelBlendingParameter velBlendingPrm;
    double data[20];
} TBlendingParameter;

typedef struct GroupBlendingParameter
{
    short mode;			// 模式
    short reserve[3];
    TBlendingParameter prm;
} TGroupBlendingParameter;

#define GEOMETRY_LIMIT_MODE_WORK    (0)    //根据工件坐标系的曲率和夹角进行限制
#define GEOMETRY_LIMIT_MODE_AXIS    (1)    //根据轴坐标系的曲率和夹角进行限制
#define GEOMETRY_LIMIT_MODE_NONE    (2)    //曲率和夹角都不约束

typedef struct GroupDvMaxLimit
{
	short type;                         // 0: 老约束模式，1：新约束模式
	short optMode;                      // 是否对dvmax约束进行优化
}TGroupDvMaxLimit;

typedef struct GroupCmdVelLimit
{
    short enable;                      // 0: 关闭该功能，1：启用该功能
    short mode;                        // 窗宽移动模式，0：每次移动一个数据，1：每次移动一整个窗宽
    long n1;                           // 1秒中能发送的指令数
    long n2;                           // 进行速度约束计算的窗宽
}TGroupCmdVelLimit;

GT_API GTN_GroupLookAheadEnable(short core, short group, TListInfo* pListInfo=NULL);
GT_API GTN_GroupLookAheadDisable(short core, short group, TListInfo* pListInfo=NULL);
GT_API GTN_SetGroupLookAheadParameter(short core, short group, TGroupLookAheadParameter* pLookAheadPrm, TListInfo* pListInfo=NULL);
GT_API GTN_GetGroupLookAheadSegCount(short core,short group,long *pSegCount);
GT_API GTN_GetLookAheadErrorInfo(short core, short group, TLookAheadErrorInfo* pLaErrorInfo);
GT_API GTN_SetGroupBlendingParameter(short core, short group, TGroupBlendingParameter* pPrm, TListInfo* pListInfo=NULL);
GT_API GTN_SetGroupCartesianCoordinateAxisLimit(short core,short group,short enable,TListInfo *pListInfo=NULL);
GT_API GTN_SetGroupLookAheadFunc(short core,short group,short mode,void *pLookAheadFunc,TListInfo *pListInfo=NULL);
GT_API GTN_SetGroupMinLinearLength(short core,short group,short type,double value,TListInfo *pListInfo=NULL);

typedef struct RotateAxisPos
{
    double value[2];
    double reserve[14];
}TRotateAxisPos;

typedef struct ToolVector
{
    double value[3];
    double reserve[13];
}TToolVector;

typedef union InclinePlaneRotateAxisInfoUnion
{
    TRotateAxisPos rotateAxisPos;
    TToolVector toolVector;
    double value[16];
}TInclinePlaneRotateAxisInfoUnion;

typedef struct InclinedPlaneRotateAxisInfo
{
    short mode;
    short pad[3];
    TInclinePlaneRotateAxisInfoUnion data;
}TInclinedPlaneRotateAxisInfo;

typedef struct SelectConfig
{
    short configIndex;
    short reserve1[3];
    double reserve2[9];
}TSelectConfig;

typedef union GroupInclinedPlanePrmUnion
{
    TSelectConfig select;
    double value[10];
}TGroupInclinedPlanePrmUnion;

typedef struct GroupInclinedPlanePrm
{
    short mode;
    short reserve[3];
    TGroupInclinedPlanePrmUnion data;
}TGroupInclinedPlanePrm;

GT_API GTN_SetGroupInclinedPlaneRotateAxisPos(short core, short group, TInclinedPlaneRotateAxisInfo* pInfo, double* pRotateAxisPos, short activeRotateAxis = 1, TListInfo* pListInfo = NULL);
GT_API GTN_GetGroupInclinedPlaneRotateAxisPos(short core, short group, double* pRotateAxisPos);
GT_API GTN_CalGroupInclinedPlaneRotateAxisPos(short core, short group, TToolVector* pToolVector, double* pRotateAxisPos1, double* pRotateAxisPos2, TListInfo* pListInfo = NULL);
GT_API GTN_SetGroupInclinedPlanePrm(short core, short group, TGroupInclinedPlanePrm* pPrm, TListInfo* pListInfo=NULL);
GT_API GTN_GetGroupInclinedPlanePrm(short core, short group, TGroupInclinedPlanePrm* pPrm);

#define GROUP_MOVE_PRM_RESERVE2_POS_MASK                    (0) // 位置有效轴，0：所有轴的位置都生效，非0：指定轴的位置生效
#define GROUP_MOVE_PRM_RESERVE2_START_POINT_MODE            (2) // 起点位置模式，0：起点为上一段的终点，1：起点为执行到该指令时实时获取的当前点位置
typedef struct GroupMoveParameter
{
    double velocity;			// 名义最大速度，用来和速度倍率相乘
    double acceleration;        // 加速度
	double reserve1[3];
    double deceleration;        // 减速度
    short  overrideSelect;		// 倍率选择
    short  endVelocityMode;     // 终点速度模式，0：无效，1：减速到零，2：为指定的终点速度（保留）
    short  orientationDir;      // 姿态变化方向，0：短路径方向，1：长路径方向
    short  reserve2[3];
    long   reserve3[3];
} TGroupMoveParameter;

// 圆弧参数
#define CIRCULAR_PARAMETER_MAX_ERROR             (0) // 圆弧最大允许误差
#define CIRCULAR_PARAMETER_ORIENTAION_PASS_MODE  (1) // 圆弧是否经过中间点的姿态
#define CIRCULAR_PARAMETER_AUXILIARY_START_MASTER_POS (2) // 圆弧起点的动态坐标系主轴位置
#define CIRCULAR_PARAMETER_AUXILIARY_MASTER_POS  (3) // 圆弧辅助点的动态坐标系主轴位置

//圆弧描述模式
#define CIRCULAR_MODE_PLANE_CENTER_DIR           (0) //终点+圆心+方向，用于平面圆弧
#define CIRCULAR_MODE_PLANE_RADIUS_DIR           (1) //终点+半径(有符号，决定优/劣弧)+方向，用于平面圆弧
#define CIRCULAR_MODE_SPACE_BORDER               (2) //终点+中间点，用于空间圆弧
#define CIRCULAR_MODE_SPACE_CENTER               (3) //终点+圆心+优/劣弧，用于空间圆弧
#define CIRCULAR_MODE_SPACE_RADIUS               (4) //终点+垂直平面的向量+半径(有符号，决定优/劣弧)+方向，用于空间圆弧
#define CIRCULAR_MODE_PLANE_RELATIVE_CENTER_DIR  (5) //终点+相对圆心+方向，用于平面圆弧
#define CIRCULAR_MODE_SPACE_BORDER_FOUR          (6) //终点+中间点+辅助起点，用于四点空间圆弧
#define CIRCULAR_MODE_PLANE_CENTER_DIR_SPIRAL              (10) //终点+圆心+方向，用于涡旋插补
#define CIRCULAR_MODE_PLANE_RELATIVE_CENTER_DIR_SPIRAL     (11) //终点+相对圆心+方向，用于涡旋插补

//圆弧插补平面
#define CIRCULAR_PLANE_XY                        (0)
#define CIRCULAR_PLANE_YZ                        (1)
#define CIRCULAR_PLANE_ZX                        (2)
#define CIRCULAR_PLANE_XYZ                       (3)
#define CIRCULAR_PLANE_XY_HELIX                  (10)
#define CIRCULAR_PLANE_YZ_HELIX                  (11)
#define CIRCULAR_PLANE_ZX_HELIX                  (12)

//终点指定模式
#define CIRCULAR_END_POINT_MODE_END_POINT        (0) //终点由终点位置指定
#define CIRCULAR_END_POINT_MODE_CENTRAL_ANGLE    (1) //终点由圆心角指定

#define CIRCULAR_PATH_CHOICE_CW					 (0) //顺时针
#define CIRCULAR_PATH_CHOICE_CCW				 (1) //逆时针

typedef struct CircularPlaneCenterDirData
{
    short endPointMode;        //圆弧终点指定模式，0：终点为输入的终点位置，1：终点由圆心角决定
    short arcPlane;            //圆弧插补平面，XY,YZ,ZX
    short arcPathChoice;       //圆弧路径选择，平面圆弧时为方向(1：逆时针，-1：顺时针)，空间圆弧为优劣弧(1：劣弧，-1：优弧)
    short pad;
    double centralAngle;       //圆心角
    double centerPoint[8];     //圆弧圆心
}TCircularPlaneCenterDirData;

typedef struct CircularPlaneRadiusDirData
{
    short endPointMode;        //圆弧终点指定模式，0：终点为输入的终点位置，1：终点由圆心角决定
    short arcPlane;            //圆弧插补平面，XY,YZ,ZX
    short arcPathChoice;       //圆弧路径选择，平面圆弧时为方向(1：逆时针，-1：顺时针)，空间圆弧为优劣弧(1：劣弧，-1：优弧)
    short pad;
    double centralAngle;       //圆心角
    double arcRadius;          //圆弧半径
}TCircularPlaneRadiusDirData;

typedef struct CircularSpaceBorderData
{
    short endPointMode;        //圆弧终点指定模式，0：终点为输入的终点位置，1：终点由圆心角决定
    short pad[3];
    double centralAngle;       //圆心角
    double auxPoint[8];        //中间点
}TCircularSpaceBorderData;

typedef struct CircularSpaceBorderFourData
{
    short endPointMode;                 //圆弧终点指定模式，0：终点为输入的终点位置，1：终点由圆心角决定
    short auxPointCommandCoord;         //中间点位置描述坐标系
    short auxPointOrientationMode;      //中间点位置描述姿态
    short auxPointConfigIndex;          //中间点构型解
    short auxStartPointCommandCoord;    //辅助起点位置描述坐标系
    short auxStartPointOrientationMode; //辅助起点位置描述姿态
    short auxStartPointConfigIndex;     //辅助起点构型解
    short pad;
    double centralAngle;       //圆心角
    double auxPoint[8];        //中间点
    double auxStartPoint[8];   //辅助起点位置，用于描述圆弧
}TCircularSpaceBorderFourData;

typedef struct CircularPlaneCenterDirSpiralData
{
    short endPointMode;        //圆弧终点指定模式，0：终点为输入的终点位置，1：终点由圆心角决定
    short arcPlane;            //圆弧插补平面，XY,YZ,ZX
    short arcPathChoice;       //圆弧路径选择，平面圆弧时为方向(1：逆时针，-1：顺时针)，空间圆弧为优劣弧(1：劣弧，-1：优弧)
    short periodCount;         //圈数
    double centralAngle;       //圆心角
    double centerPoint[8];     //圆弧圆心
    double periodDeltaRadius;  //一圈对应的半径变化量
}TCircularPlaneCenterDirSpiralData;

typedef union CircularModeUnion
{
    TCircularPlaneCenterDirData centerDir;       //当圆弧模式为圆心方向，或者相对圆心方向时，共用该结构体
    TCircularPlaneRadiusDirData radiusDir;
    TCircularSpaceBorderData spaceBorder;
    TCircularSpaceBorderFourData spaceBorderFour;
    TCircularPlaneCenterDirSpiralData spiral;    //当圆弧模式为圆心方向，或者相对圆心方向时，共用该结构体
    double data[32];
}TCircularModeUnion;

typedef struct CircularParameter
{
    short arcMode;             //圆弧描述模式
    short pad[3];
    TCircularModeUnion data;
}TCircularParameter;

#define GATE_MODE_NORMAL                         (0)

typedef struct Gate
{
	double h1;     // 起点垂直提升高度
	double h2;     // 终点垂直下降高度
	double h3;     // 两段曲线段的最小过渡高度
	double k1;     // 上升曲线段平移量占平移段的百分比
	double k2;     // 下降曲线段平移量占平移段的百分比
	short curveMode;       // 过渡曲线模式
	short oriChangeMode;   // 姿态变化模式
	short reserve1[6];
	double reserve2[8];
}TGate;

typedef union GateUnion
{
	TGate gate;
	double data[32];
}TGateUnion;

typedef struct GatePrm
{
	short mode;             //门型轨迹描述模式
	short reserve[3];
	TGateUnion data;
}TGatePrm;

GT_API GTN_MoveGateAbsolute(short core,short group,double endPoint[],TGatePrm *pGatePrm,TGroupMoveParameter *pPrm,TListInfo *pListInfo=NULL);
GT_API GTN_MoveLinearAbsolute(short core, short group, double pos[], short dir[], TGroupMoveParameter* pPrm, TListInfo* pListInfo = NULL);
GT_API GTN_MoveCircularAbsolute(short core, short group, double endPoint[], TCircularParameter* pCircularPrm, TGroupMoveParameter* pPrm, TListInfo* pListInfo = NULL);


#define WEAVE_TYPE_SINE                            (0)
#define WEAVE_TYPE_SINE_DIR_X                      (WEAVE_TYPE_SINE+100)

// 描述摆弧类型及参数
typedef struct GroupWeaveParameter
{
	short type;           // 摆弧类型
	short weaveMode;      // 摆弧模式：0：该结构体参数为摆弧参数，1：该结构体参数为预摆弧参数
	short frequencyMode;  // 频率模式：0：正负摆弧为同一频率，1：正负摆弧通过weavePrm[0]和weavePrm[1]设置频率比例
	short pad1;
	double amplitude;     // 摆弧振幅，单位：mm
	double frequency;     // 摆弧频率，单位：Hz
	double weavePrm[10];  // 摆弧几何参数
	short startPosition;  // 摆弧开始位置
	short dweelFullStop;  // 完全停止使能
	short pad2[2];
	double dweelLeft;     // 摆弧左停留时间，单位：ms
	double dwellMid;      // 摆弧中停留时间，单位：ms
	double dwellRight;    // 摆弧右停留时间，单位：ms
	short oriControlMode; // 摆弧姿态控制模式
	short pad3[3];
	double rotateAngleX;  // 外倾角，单位：度
	double rotateAngleY;  // 前倾角，单位：度
	double rotateAngleZ;  // 旋转角，单位：度
}TGroupWeaveParameter;

#define GROUP_SUPERPOSITION_TABLE_MAX       (4)    // 最大叠加表个数
#define GROUP_SUPERPOSITION_TABLE_DATA_MAX  (128)  // 叠加表的最大数据个数

typedef struct GroupSuperpositionTablePrm
{
	short mode;              // 模式：目前只支持设置为0
	short dataCount;         // 叠加表的数据点个数
	short reserve1[6];
	double period;           // 叠加表的周期
	double reserve2[5];
} TGroupSuperpositionTablePrm;

typedef struct GroupSuperpositionTableData
{
	double value[8];
} TGroupSuperpositionTableData;

typedef struct GroupSuperpositionTableStatus
{
	short workIndex;
	short reserve1[7];
	double reserve2[14];
} TGroupSuperpositionTableStatus;

#define GROUP_SUPERPOSITION_MODE_DIRECT            (0)  // 直接叠加模式
#define GROUP_SUPERPOSITION_MODE_WEAVE             (1)  // 摆弧叠加模式
#define GROUP_SUPERPOSITION_MODE_WEAVE_EX          (2)  // 摆弧叠加扩展模式
#define GROUP_SUPERPOSITION_MODE_TABLE             (3)  // 查表叠加模式
#define GROUP_SUPERPOSITION_MODE_DIRECT_RELATIVE   (10) // 相对叠加模式

// 设置直接叠加值
typedef struct GroupSuperpositionValueDirect
{
	double value[8];                         // 叠加值
	double reserve[24];
}TGroupSuperpositionValueDirect;

#define GROUP_SUPERPOSITON_WEAVE_EX_RESERVE_STOP_VEL	   (0)       // 摆弧叠加停止速度
typedef struct GroupSuperpositionWeaveEx
{
	short mode;                              // 摆弧叠加子模式 0：摆弧叠加模式 1：预摆弧模式
	short pad[3];                            // 对齐
	double time;                             // 预摆弧模式的持续时间
	double velocity;                         // 预摆弧模式的速度
	double reserve[29];
}TGroupSuperpositionWeaveEx;

#define GROUP_SUPERPOSITON_TABLE_RESERVE2_STOP_VEL	       (0)        // 摆弧叠加停止速度
typedef struct GroupSuperpositionTable
{
	short mode;                              // 查表叠加子模式 0：普通叠加模式 1：预叠加模式
	short startIndex;                        // 起始叠加表的索引
	short count;                             // 叠加表数量，count=1时，表示循环叠加索引为startIndex的表，count>1时，则表示依次叠加从startIndex开始的count个表
	short reserve1[5];
	double time;                             // 预叠加模式的持续时间
	double velocity;                         // 预叠加模式的速度
	double reserve2[28];
}TGroupSuperpositionTable;

typedef union GroupSuperpositionUnion
 {
	TGroupSuperpositionValueDirect direct;   // 直接叠加值
	TGroupSuperpositionWeaveEx weaveEx;      // 摆弧叠加扩展模式
	TGroupSuperpositionTable table;          // 查表叠加模式
	double data[32];
}TGroupSuperpositionUnion;

typedef struct GroupSuperposition
{
	short enable;         // 叠加是否使能
	short mode;           // 叠加模式 0：直接叠加 1：根据时间进行摆弧叠加 2：摆弧叠加扩展模式
	short pad[2];
	double smoothTime;    // 平滑时间
	TGroupSuperpositionUnion data;
}TGroupSuperposition;

GT_API GTN_SetGroupWeaveParameter(short core,short group,TGroupWeaveParameter *pWeave,TListInfo *pListInfo=NULL);
GT_API GTN_GetGroupWeaveParameter(short core,short group,TGroupWeaveParameter *pWeave);
GT_API GTN_SetGroupSuperpositionTable(short core,short group,short tableIndex,TGroupSuperpositionTablePrm *pTablePrm,TGroupSuperpositionTableData *pData);
GT_API GTN_GetGroupSuperpositionTable(short core,short group,short tableIndex,TGroupSuperpositionTablePrm *pTablePrm,TGroupSuperpositionTableData *pData);
GT_API GTN_GetGroupSuperpositionTableStatus(short core,short group,TGroupSuperpositionTableStatus *pTableStatus);
GT_API GTN_SetGroupSuperposition(short core,short group,short coordSystem,short index,TGroupSuperposition *pGroupSuperposition,TListInfo *pListInfo=NULL);
GT_API GTN_GetGroupSuperposition(short core,short group,short coordSystem,short index,TGroupSuperposition *pGroupSuperposition);
GT_API GTN_GetGroupSuperpositionValue(short core,short group,short coordSystem,short index,double *pValue);

/*-----------------------------------------------------------*/
/* CBT Command                                               */
/*-----------------------------------------------------------*/
#define CBT_MODE_CONTINUOUS  (0)
#define CBT_MODE_INTERMITTEN (1)

#define CBT_EXECUTE_MODE_AUTO          (0)    // 自动模式
#define CBT_EXECUTE_MODE_COMMAND       (1)    // 指令驱动模式

#define CBT_MASTER_MOTION_TYPE_LINEAR  (0)    // 直线传送带
#define CBT_MASTER_MOTION_TYPE_ROTATE  (1)    // 旋转传送带

typedef struct CbtPrm
{
	short masterMode;     // 主轴模式0：连续传送带，1：间歇传送带
	short masterType;     // 主轴类型
	short masterIndex;    // 主轴索引
	short cbtTag;         // 传送带标识，对应TCbtPieceData中的cbtTag，用于标识工件是否为传送带需要处理的工件

	double detectPos;     // 传送带检测位相对传送带坐标系零点的X轴偏移量
	double parkPos;       // 传送带泊车位相对传送带坐标系零点的X轴偏移量
	double stopPos;       // 传送带停止位相对传送带坐标系零点的X轴偏移量
	double delay;         // 同步区任务执行前的延时时间
	double sampleTime;    // 传送带速度采样时间
	double velMax;        // 机器人最大追赶速度
	double acc;           // 机器人追赶加速度
	double cbOrigin[6];   // 传送带坐标系零点相对机器人MCS的偏移和旋转

	short group;          // 工作机器人
	short masterMotionType;// 主轴运动类型，0：直线传送带，1：旋转传送带
	short masterMotionDir; // 相对传送带坐标系的运动方向，仅对旋转传送带有效，0：顺时针，1：逆时针
	short reserve1[5];
	double reserve2[10];
}TCbtPrm;

typedef struct CbtGroupPrm
{
	short group;
	short pieceConfigIndex;// 机器人在传送带上的工件坐标系上进行工作时（包括同步定位点）的构型解
	short waitMode;        // 0：在原地等待工件，1：到等待位等待工件
	short waitProfileCoord;
	short waitCmdPosCoord;
	short waitConfigIndex;// 机器人处于等待位的构型解
	short reserve1[2];

	double locatePoint[6];// 机器人运动到传送带上的工件定位点，为PCS下的位置和姿态
	double waitPoint[6];  // 传送带没有工件或者工件还未到达时，机器人回到等待位，为MCS下的位置和姿态

	double groupVel;      // 机器人运动速度
	double groupAcc;      // 机器人运动加速度
	double groupDec;      // 机器人运动减速度

	short reserve2[8];
	double reserve3[10];
}TCbtGroupPrm;

typedef struct CbtPieceData
{
	short cbtTag;            // 传送带标识，对应TCbtPrm中的cbtTag，用于标识本工件是否为cbtIndex对应的传送带需要处理的工件
	short pieceTag;          // 工件标识，可以用于标识每一个工件
	short reserve1[2];
	double masterPos;        // 工件经过检测位时的主轴位置
	double pose[6];          // 工件相对传送带坐标系的位置和姿态
	double reserve2[5];
	double processTime;      // 工件在同步区的处理时间
}TCbtPieceData;

typedef struct CoupleCbtPrm
{
	short cbtIndex1;
	short cbtIndex2;
	short reserve1[2];
	double reserve2[10];
}TCoupleCbtPrm;

typedef struct StartCbtPrm
{
	short executeMode;
	short reserve1[3];
	double reserve2[10];
}TStartCbtPrm;

#define CBT_STATE_IDLE                    (0) //空闲状态
#define CBT_STATE_BACK_TO_WAIT_POINT      (10) //没有工件，回到等待位
#define CBT_STATE_WAITTING_PIECE          (20) //在原位或者等待位等待工件
#define CBT_STATE_LOCATING                (110) //等待机械手到位
#define CBT_STATE_WAITING_SYNCHRONIZATION (120) //等待机械手和传送带同步
#define CBT_STATE_SYNCHRONIZATION         (130) //同步状态
#define CBT_STATE_DELAY                   (140) //延时状态
#define CBT_STATE_EXECUTING_TASK          (150) //同步任务执行状态
#define CBT_STATE_EXITING_SYNCHRONIZATION (160) //退出同步区
#define CBT_STATE_STOPPING                (300) //停止状态
#define CBT_STATE_DONE                    (400) //完成状态

// CBT停止的详细信息
#define CBT_STOP_INFO_NONE                               (0)
#define CBT_STOP_INFO_CU_OUT_OF_RANGE                    (1)
#define CBT_STOP_INFO_GET_CU_INFO                        (2)
#define CBT_STOP_INFO_ENABLE_GROUP_DYNAMICA_TRANSFORM    (3)
#define CBT_STOP_INFO_DISABLE_GROUP_DYNAMICA_TRANSFORM   (4)
#define CBT_STOP_INFO_START_GROUP_LOCATE_MOTION          (5)
#define CBT_STOP_INFO_START_GROUP_WAIT_MOTION            (6)
#define CBT_STOP_INFO_START_CU                           (7)
#define CBT_STOP_INFO_USER_STOP                          (10)
#define CBT_STOP_INFO_USER_STOP_COMMAND_LIST             (11)
#define CBT_STOP_INFO_USER_STOP_IN_LIST                  (12)
#define CBT_STOP_INFO_GROUP_ERROR                        (50)
#define CBT_STOP_INFO_PROFILE_ERROR                      (51)

typedef struct CbtInfo
{
	short execute;                    // 传送带是否在运行
	short state;                      // 传送带的执行状态
	short stopInfo;                   // 传送带运行停止时的详细信息
	short reserve1[3];
	short pieceTag;                   // 当前正在处理的工件标识
	short pieceEmpty;                 // 工件fifo是否为空
	unsigned long pieceReceive;       // 传送带接受到的所有工件数量
	unsigned long pieceRemainder;     // 传送带剩余还未处理的工件数量
	unsigned long pieceProcess;       // 传送带已处理的工件数量，包括丢弃的工件数量
	unsigned long validPieceProcess;  // 传送带有效处理的工件数量
	unsigned long reserve2[6];

	double catchUpStartPos;          // 追赶轴启动位置相对传送带坐标系的位置
	double actualPiecePos;           // 实际工件相对传送带坐标系的位置
	double virtualPiecePos;          // 由追赶轴带着的虚拟工件相对传送带坐标系的位置
}TCbtInfo;

GT_API GTN_SetCbtPrm(short core,short cbtIndex,TCbtPrm *pPrm);
GT_API GTN_GetCbtPrm(short core,short cbtIndex,TCbtPrm *pPrm);
GT_API GTN_SetCbtGroupPrm(short core,short cbtIndex,TCbtGroupPrm *pPrm);
GT_API GTN_GetCbtGroupPrm(short core,short cbtIndex,TCbtGroupPrm *pPrm);
GT_API GTN_ClearCbtPieceData(short core,short cbtIndex,short type);
GT_API GTN_SetCbtPieceData(short core,short cbtIndex,TCbtPieceData *pCbtPieceData,short count=1);
GT_API GTN_GetCbtDiscardPieceData(short core,short cbtIndex,short count,TCbtPieceData *pCbtPieceData,short *pActualCount);
GT_API GTN_SetCbtTask(short core,short cbtIndex,short taskType,void *pTaskData);
GT_API GTN_GetCbtTask(short core,short cbtIndex,short *pTaskType,void *pTaskData);
GT_API GTN_StartCoupleCbt(short core,short coupleCbtIndex,TCoupleCbtPrm *pPrm);
GT_API GTN_StopCoupleCbt(short core,short coupleCbtIndex);
GT_API GTN_GetCbtInfo(short core,short cbtIndex,TCbtInfo *pInfo);
GT_API GTN_ClearCbtInfo(short core,short beltIndex);
GT_API GTN_StartCbt(short core,short cbtIndex,TStartCbtPrm *pStartCbtPrm,TListInfo *pListInfo=NULL);
GT_API GTN_StopCbt(short core,short cbtIndex,TListInfo *pListInfo=NULL);
GT_API GTN_WaitCbtPiece(short core,short cbtIndex,TListInfo *pListInfo=NULL);

/*-----------------------------------------------------------*/
/* CollisionCheck Command                                    */
/*-----------------------------------------------------------*/
#define COLLISION_CHECK_MAX            (8)       // 碰撞检测的最大组数
#define COLLISION_CHECK_OBJECT_MAX     (16)      // 碰撞检测对象的最大个数

#define OBJECT_TYPE_NONE               (-1)      // 没有设置
#define OBJECT_TYPE_CUBOID             (1)       // 长方体
#define OBJECT_TYPE_CYLINDER           (2)       // 圆柱体

#define OBJECT_LOCATE_FIX              (0)       // 检测对象固定不动
#define OBJECT_LOCATE_TOOL             (1)       // 检测对象位于工具侧
#define OBJECT_LOCATE_WORK_PIECE       (2)       // 检测对象位于工件侧

#define AXIS_SIDE_TOOL                 (0)       // 直线轴位于工具侧
#define AXIS_SIDE_WORK_PIECE           (1)       // 直线轴位于工件侧

// 长方体描述信息
typedef struct ObjectCuboid
{
    double axis[3][3];                           // 长方体的3个轴向向量，向量矢量和为1
    double length[3];                            // 长方体的3个轴向的长度
} TObjectCuboid;

// 圆柱体描述信息
typedef struct ObjectCylinder
{
    double axis[3];                              // 圆柱体的轴向向量，向量矢量和为1
    double radius;                               // 圆柱体半径
    double height;                               // 圆柱体的高度
} TObjectCylinder;

// 检测对象的形状信息
typedef union ObjectShape
{
    TObjectCuboid cuboid;                        // 长方体描述信息
    TObjectCylinder cylinder;                    // 圆柱体描述信息
    double data[32];
} TObjectShape;

// 碰撞检测对象的参数
typedef struct CollisionCheckObjectPrm
{
    short shapeType;                             // 检测对象的类型
    short reserve1[3];                           // 保留值，必须设置为0
    TObjectShape shape;                          // 检测对象的形状描述信息

    short source;                                // 检测对象的位置参考源：MC_GROUP_PROFILE；MC_PROFILE；MC_NONE等
                                                 // 参考源为group类型时，检测对象由group中的轴带动，参考坐标系为group的MCS坐标系
                                                 // 参考源为MC_PROFILE，检测对象由规划轴带动，参考坐标系为规划轴零点
                                                 // 参考源为MC_NONE时，检测对象固定不动，参考坐标系为其中心

    short index[3];                              // 参考源索引
                                                 // 参考源为group类型时，index[0]为group索引，index[1~2]设置为0
                                                 // 参考源为MC_PROFILE，index[0~3]分别指示带着检测对象在XYZ方向运动的轴索引，如果所处维度没有轴运动，设置成0
                                                 // 参考源为MC_NONE时，index[0~3]设置为0

    short locate;                                // 当参考源为group类型时，需要设置检测对象所处位置类型
                                                 // OBJECT_LOCATE_FIX：检测对象固定不动
                                                 // OBJECT_LOCATE_TOOL：检测对象位于工具侧，由工具侧的轴带着运动
                                                 // OBJECT_LOCATE_WORK_PIECE：检测对象位于工件侧，由工件侧的轴带着运动
    short side[3];                               // 当参考源为group类型时，需要设置直线轴所处方位即直线轴位于工具侧还是工件侧，0：工具侧，1：工件侧

    double offset[3];                            // 检测对象的零点相对参考源零点的偏移
                                                 // 参考源为group类型时，当locate为OBJECT_LOCATE_TOOL时，设置的偏移量为相对group的名义工具末端的坐标，locate为其他类型时，设置的偏移量为相对MCS零点的坐标
                                                 // 参考源为profile类型时，参考源零点为轴零点
                                                 // 参考源为MC_NONE时，参考源零点为本身的零点

    short reserve2[4];                           // 保留值，必须设置为0
} TCollisionCheckObjectPrm;

typedef struct CollisionCheckPrm
{
    short objectIndex[2];                        // 进行碰撞检测的两个检测对象的索引
    short reserve1[2];                           // 保留值，必须设置为0
    short rotateMode;                            // 两个位置参考源坐标轴是否存在旋转，0：没有旋转，只有同向或者反向，通过dir参数设置，1：有旋转，通过其他指令设置，目前仅支持模式0.
    short dir[3];                                // 两个检测对象的位置参考源XYZ的方向是否反向，0：同向，1：反向
                                                 // 当两个参考源相同，或者存在固定不动参考源类型时，必然同向

    double originOffset[3];                      // 当位置参考源不一样时，设置进行碰撞检测的两个检测对象的参考零点的偏移，如果参考源一样，该参数设置为0
                                                 // 零点偏移为第二个物体零点相对第一个物体坐标系零点和方向的偏移距离

    double distance;                             // 碰撞检测距离
    double reserve2[8];                          // 保留值，必须设置为0
} TCollisionCheckPrm;

typedef struct CollisionCheckStatus
{
    short enable;                                // 碰撞检测使能标志
    short checkResult;                           // 两个碰撞检测对象是否进入了碰撞检测范围
    short reserve1[6];
    double reserve2[4];
}TCollisionCheckStatus;

/**
 * @brief 设置碰撞检测对象的参数
 * @param core 核号
 * @param objectIndex 碰撞检测对象的索引，从1开始
 * @param pPrm 碰撞检测对象的参数
 * @return
*/
GT_API GTN_SetCollisionCheckObjectPrm(short core,short objectIndex,TCollisionCheckObjectPrm *pPrm);
GT_API GTN_GetCollisionCheckObjectPrm(short core,short objectIndex,TCollisionCheckObjectPrm *pPrm);

/**
 * @brief 设置碰撞检测的参数
 * @param core 核号
 * @param checkIndex 碰撞检测的索引，从1开始
 * @param pPrm 碰撞检测的参数
 * @return
*/
GT_API GTN_SetCollisionCheckPrm(short core,short checkIndex,TCollisionCheckPrm *pPrm);
GT_API GTN_GetCollisionCheckPrm(short core,short checkIndex,TCollisionCheckPrm *pPrm);

/**
 * @brief 使能或关闭碰撞检测功能
 * @param core 核号
 * @param checkIndex 碰撞检测的索引，从1开始
 * @param enable 使能参数
 * @param pListInfo 指令流参数
 * @return
*/
GT_API GTN_SetCollisionCheckEnable(short core,short checkIndex,short enable,TListInfo *pListInfo=NULL);

/**
 * @brief 获取碰撞检测功能的状态
 * @param core 核号
 * @param checkIndex 碰撞检测的索引，从1开始
 * @param pStatus 碰撞检测模块状态
 * @return
*/
GT_API GTN_GetCollisionCheckStatus(short core,short checkIndex,TCollisionCheckStatus *pStatus);

/**
 * @brief 清除碰撞检测功能的状态
 * @param core 核号
 * @param checkIndex 碰撞检测的索引，从1开始
 * @return
*/
GT_API GTN_ClearCollisionCheckStatus(short core,short checkIndex);

/*--------- -------------------------------------------------*/
/* Gantry                                                    */
/*-----------------------------------------------------------*/
#define GANTRY_MODE_NONE					 			(-1)
#define GANTRY_MODE_OPEN_LOOP_GANTRY					(1)
#define GANTRY_MODE_DECOUPLE_POSITION_LOOP				(2)
#define GANTRY_MODE_RINGNET_DRIVER						(10)

GT_API GT_SetGantryMode(short group, short master, short slave, short mode, long syncErrorLimit = 1000);
GT_API GT_GetGantryMode(short group, short* pMaster, short* pSlave, short* pMode, long* pSyncErrorLimit);
GT_API GT_SetGantryPid(short group, TPid* pGantryPid, TPid* pYawPid);
GT_API GT_GetGantryPid(short group, TPid* pGantryPid, TPid* pYawPid);
GT_API GT_GantryAxisOn(short group);
GT_API GT_GantryAxisOff(short group);

GT_API GTN_SetGantryMode(short core, short group, short master, short slave, short mode, long syncErrorLimit = 1000);
GT_API GTN_GetGantryMode(short core, short group, short* pMaster, short* pSlave, short* pMode, long* pSyncErrorLimit);
GT_API GTN_SetGantryPid(short core, short group, TPid* pGantryPid, TPid* pYawPid);
GT_API GTN_GetGantryPid(short core, short group, TPid* pGantryPid, TPid* pYawPid);
GT_API GTN_GantryAxisOn(short core, short group);
GT_API GTN_GantryAxisOff(short core, short group);
GT_API GTN_SetAxisInputShapeVerI(short core, short axis, short enable, short freq, double z);

typedef struct GantryTorqueProtectPrm
{
    short windowSize;										// 窗口大小
    short lostSize;											// 使能后多少拍以后开始检查
    long torqueLimitMax;									// 最大值
    long torqueLimitMin;									// 最小值

    short continuousProtectCount;							// 窗宽范围内，实际电流连续这么多拍超过最大值或者最小值则保护触发
    short oscillationPositiveProtectCount;					// 窗宽范围内，实际电流总共这么多拍大于最大值则保护触发
    short oscillationNegativeProtectCount;					// 窗宽范围内，实际电流总共这么多拍小于最小值则保护触发
    short reserve[15];
} TGantryTorqueProtectPrm;
GT_API GTN_SetGantryProtectPrm(short core, short group, short torqueProtectType, TGantryTorqueProtectPrm* pPrm);
GT_API GTN_GetGantryProtectPrm(short core, short group, short torqueProtectType, TGantryTorqueProtectPrm* pPrm);
GT_API GTN_GantryProtectEnable(short core, short group, short enable, short protectType, short subProtectType);
typedef struct GantryInf
{
    long torqueProtectSts;												// 触发状态
    long torqueProtectEnable;											// 触发使能状态
    short reserve[20];
} TGantryInf;
GT_API GTN_GetGantryInf(short core, short group, TGantryInf* pPrm);

GT_API GTN_SetGantrySynchErrorCompensate2DTable(short core, short tableIndex, TCompensate2DTable* pTable, long* pData, short extend);
GT_API GTN_GetGantrySynchErrorCompensate2DTable(short core, short tableIndex, TCompensate2DTable* pTable, short* pExtend);
GT_API GTN_SetGantrySynchErrorCompensate2D(short core, short group, TCompensate2D* pComp2d);
GT_API GTN_GetGantrySynchErrorCompensate2D(short core, short group, TCompensate2D* pComp2d);
GT_API GTN_GetGantrySynchErrorCompensate2DValue(short core, short group, double* pValue);
GT_API GT_SetGantrySynchErrorCompensate2DTable(short tableIndex, TCompensate2DTable* pTable, long* pData, short extend);
GT_API GT_GetGantrySynchErrorCompensate2DTable(short tableIndex, TCompensate2DTable* pTable, short* pExtend);
GT_API GT_SetGantrySynchErrorCompensate2D(short group, TCompensate2D* pComp2d);
GT_API GT_GetGantrySynchErrorCompensate2D(short group, TCompensate2D* pComp2d);
GT_API GT_GetGantrySynchErrorCompensate2DValue(short group, double* pValue);

typedef struct SecondOrderFilterPara
{
    short active;

    double a0;
    double a1;
    double a2;

    double b1;
    double b2;

} TSecondOrderFilterPara;

#define TYPE_CONTROL_OUTPUT_FILTER			(1)
#define TYPE_YAW_CONTROL_OUTPUT_FILTER		(2)
#define FILTER_COUNT_MAX					(2)

GT_API GT_SetSecondOrderFilterPara(short control, short type, short index, TSecondOrderFilterPara* pFilterPrm);
GT_API GT_GetSecondOrderFilterPara(short control, short type, short index, TSecondOrderFilterPara* pFilterPrm);

GT_API GTN_SetSecondOrderFilterPara(short core, short control, short type, short index, TSecondOrderFilterPara* pFilterPrm);
GT_API GTN_GetSecondOrderFilterPara(short core, short control, short type, short index, TSecondOrderFilterPara* pFilterPrm);

typedef struct _OCA_CTRL_GEN_
{
    short  s16Gain;
    short  s16Offset;
    double dFrq;
    long   u32RunPeriod;
    short  u16SigType;
} OCA_CTRL_GEN;

typedef struct ExcitationPara
{
    short gain;
    short offset;
    short frq;		// hz
    double runTime;	// s
} TExcitationPara;

typedef struct ExcitationTrapPara
{
    double vel;
    double acc;
    double dec;
    double runTime;	// s
} TExcitationTrpPara;

#define NO_EXCIT						(0xff)
#define OPEN_LOOP_EXCIT					(1)
#define SIGNAL_TYPE_STEP				(3)
#define SIGNAL_TYPE_SIN					(0x10)
#define SIGNAL_TYPE_TRAP				(0x11)
#define SIGNAL_TYPE_NOTHING				(0xFF)


GT_API GT_SetGenerator(OCA_CTRL_GEN* pGenStr);
GT_API GT_GetGenerator(OCA_CTRL_GEN* pGenStr);
GT_API GT_StepResponse(short control, short gain, double time);
GT_API GT_SetExctLoopMode(short control, short exciteLoopMode);
GT_API GT_GetExctLoopMode(short control, short* pExciteLoopMode);
GT_API GT_SetExcitation(short axis, short object, short type, void* pParameter);

GT_API GTN_SetGenerator(short core, OCA_CTRL_GEN* pGenStr);
GT_API GTN_GetGenerator(short core, OCA_CTRL_GEN* pGenStr);
GT_API GTN_StepResponse(short core, short control, short gain, double time);
GT_API GTN_SetExctLoopMode(short core, short control, short exciteLoopMode);
GT_API GTN_GetExctLoopMode(short core, short control, short* pExciteLoopMode);


/*-----------------------------------------------------------*/
/* DMA                                                       */
/*-----------------------------------------------------------*/
GT_API GT_CrdHsOn(short crd, short fifo, short link = 1, unsigned short threshold = 200, short lookAheadInMc = 0);
GT_API GT_CrdHsOff(short crd, short fifo);

GT_API GTN_CrdHsOn(short core, short crd, short fifo, short dmaBuf = 1, unsigned short threshold = 200, short lookAheadInMc = 0);
GT_API GTN_CrdHsOff(short core, short crd, short fifo);
GT_API GTN_GetCrdHsPrm(short core, short crd, short fifo, short* pEnable, short* pLink, unsigned short* pThreshold, short* pLookAheadInMc);

/*-----------------------------------------------------------*/
/* Others                                                    */
/*-----------------------------------------------------------*/
GT_API GTN_SetRetainValue(short core, unsigned long address, short count, short* pData);
GT_API GTN_GetRetainValue(short core, unsigned long address, short count, short* pData);

typedef struct GPIOConfig
{
    short direction;
    short reserve[3];
} TGPIOConfig;
GT_API GTN_SetGPIOConfig(short core, short station, long effectiveLevel, long direction);
GT_API GTN_SetGPIOConfigEx(short core, short station, short index, TGPIOConfig* pGPIOConfig, short count);
GT_API GTN_GetGPIOConfigEx(short core, short station, short index, TGPIOConfig* pGPIOConfig, short count);
GT_API GTN_GetGPIOStatus(short core, short station, short type, short* pStatus, short index, short count);

GT_API GTN_PosCurrFeedForward(short core, short profile, double pos, long gtime, short torque, short gtype, short fifo = 0);
GT_API GTN_SetMotionMode(short core, short axis, short motionMode);
GT_API GTN_GetMotionMode(short core, short axis, short* motionMode);
GT_API GTN_PosCurrFeedForwardArray(short core, short profile, double* pPos, long time, short* pTorque, short* pType, short fifo, short count);
GT_API GTN_GetNetStatus(short core, short* pNetSts);
GT_API GTN_GetPowerSts(short core, long* pSts);
GT_API GTN_SynchPrfPosWithEncPos(short core, short profile, short encoder);

GT_API GTN_SetProfileTime(short core, long profileTime, long delay, long stepCoef);
GT_API GTN_GetProfileTime(short core, long* pProfileTime, long* pDelay, long* pStepCoef);
GT_API GTN_SetCoreTime(short core, long profileTime, long delay, long stepCoef);
GT_API GTN_GetCoreTime(short core, long* pProfileTime, long* pDelay, long* pStepCoef);
GT_API GTN_SetCorePeriod(short core, double period);
GT_API GTN_GetCorePeriod(short core, double* pPeriod, short* pFlag, short* pInfo);
typedef struct ControlInfo
{
    double refPos;
    double refPosFilter;
    double refPosFilter2;
    double cntPos;
    double cntPosFilter;

    double error;
    double refVel;
    double refAcc;

    short value;
    short valueFilter;

    short offset;
} TControlInfo;

typedef struct CommandCount
{
    unsigned long notify;
    unsigned long receive;
    unsigned long execute;
    unsigned long retry;
    unsigned long receiveError;
    unsigned long echo;
} TCommandCount;

GT_API GT_ClearCommandCount(void);
GT_API GT_GetCommandCount(TCommandCount* pCommandCount);
GT_API GT_SetServoTime(long servoTime, long delay, long stepCoef);
GT_API GT_GetServoTime(long* pServoTime, long* pDelay, long* pStepCoef);
GT_API GT_GetControlInfo(short control, TControlInfo* pControlInfo);

GT_API GTN_ClearCommandCount(short core);
GT_API GTN_GetCommandCount(short core, TCommandCount* pCommandCount);
GT_API GTN_SetServoTime(short core, long servoTime, long delay, long stepCoef);
GT_API GTN_GetServoTime(short core, long* pServoTime, long* pDelay, long* pStepCoef);
GT_API GTN_GetControlInfo(short core, short control, TControlInfo* pControlInfo);

GT_API GT_SetLongVar(short index, long value);
GT_API GT_GetLongVar(short index, long* pValue);
GT_API GT_SetDoubleVar(short index, double value);
GT_API GT_GetDoubleVar(short index, double* pValue);

GT_API GT_GetBufWaitDiStatus(short crd, short* pDiType, unsigned short* pDiIndex, unsigned short* pLevel, short* pContinueTime, long* pOverTime, short* pFlagMode, long* pSegNum, short* pEnable, long* pCount, short fifo);
GT_API GT_GetBufWaitLongVarStatus(short crd, short* pIndex, long* pValue, short* pFlagMode, long* pSegNum, short* pEnable, short* pStatus, short fifo);
GT_API GT_ClearBufWaitStatus(short crd, short fifo);

GT_API GT_BufWaitDi(short crd, short diType, unsigned short diIndex, unsigned short level, short continueTime, long overTime, short flagMode, long segNum, short fifo);
GT_API GT_BufWaitLongVar(short crd, short index, long value, long overTime, short flagMode, long segNum, short fifo);

GT_API GTN_SetLongVar(short core, short index, long value);
GT_API GTN_GetLongVar(short core, short index, long* pValue);
GT_API GTN_SetDoubleVar(short core, short index, double value);
GT_API GTN_GetDoubleVar(short core, short index, double* pValue);

GT_API GTN_GetBufWaitDiStatus(short core, short crd, short* pDiType, unsigned short* pDiIndex, unsigned short* pLevel, short* pContinueTime, long* pOverTime, short* pFlagMode, long* pSegNum, short* pEnable, long* pCount, short fifo);
GT_API GTN_GetBufWaitLongVarStatus(short core, short crd, short* pIndex, long* pValue, short* pFlagMode, long* pSegNum, short* pEnable, short* pStatus, short fifo);
GT_API GTN_ClearBufWaitStatus(short core, short crd, short fifo);
GT_API GTN_BufWaitDi(short core, short crd, short diType, unsigned short diIndex, unsigned short level, short continueTime, long overTime, short flagMode, long segNum, short fifo);
GT_API GTN_BufWaitLongVar(short core, short crd, short index, long value, long overTime, short flagMode, long segNum, short fifo);
GT_API GTN_BufWaitWatchVar(short core, short crd, TWatchCondition* pWatchCondition, TWaitTimeout* pWaitTimeout, long segNum, short fifo);

GT_API GT_BufDoBit(short crd, unsigned short doType, unsigned short index, short value, short fifo);
GT_API GTN_BufDoBit(short core, short crd, unsigned short doType, unsigned short index, short value, short fifo);
GT_API GTN_BufDoBitDelay(short core, short crd, unsigned short doType, unsigned short index, short value, long delayTime, short fifo);
GT_API GTN_BufSetOverride(short core, short crd, double synVelRatio, short mode, short fifo);

typedef struct StopPos
{
    double stopPos;
    short mode;
    short pad[3];
}TStopPos;
GT_API GTN_BufStopPos(short core, short crd, short profile, TStopPos* pStopPos, short modal, short fifo);
GT_API GTN_BufSetSoftLimit(short core, short crd, short axis, long positive, long negative, short fifo);


typedef struct BufWaitDiStatusEx
{
    short type;
    short enable;
    short flagMode;
    short diType;
    short diIndex;
    short diValue;
    unsigned short continueTime;
    unsigned short trigDelay;
    unsigned long overTime;
    unsigned long counter;
    long longVar;
    long segNum;
    short stop;
    short overTimeStop;
    short pad1[2];
}TBufWaitDiStatusEx;
GT_API GTN_GetBufWaitDiStatusEx(short core, short crd, short fifo, TBufWaitDiStatusEx* pStatus);



//////////////////////////////////////////////////////////////////////////
//Standard Home
//////////////////////////////////////////////////////////////////////////

#define STANDARD_HOME_STAGE_IDLE  (0) //未启动回原点
#define STANDARD_HOME_STAGE_START (1) //启动回原点
#define STANDARD_HOME_STAGE_SEARCH_LIMIT (10) //寻找限位
#define STANDARD_HOME_STAGE_LEAVE_LIMIT (11) //脱离限位
#define STANDARD_HOME_STAGE_TRY_HOME_LIMIT (12) //可能搜索到Home或限位
#define STANDARD_HOME_STAGE_SEARCH_HOME (20) //正在搜索Home
#define STANDARD_HOME_STAGE_RETURN_HOME (21) //运动到捕获位置
#define STANDARD_HOME_STAGE_RETURN_HOME_WAIT_STOP (22) //等待停止
#define STANDARD_HOME_STAGE_SEARCH_INDEX  (30) //正在搜索Index
#define STANDARD_HOME_STAGE_GO_HOME       (80) //正在运动到原点
#define STANDARD_HOME_STAGE_END           (100) //回原点结束
#define STANDARD_HOME_STAGE_START_CHECK (-1) //启动回原点前自检
#define STANDARD_HOME_STAGE_CHECKING (-2) //自检中

#define STANDARD_HOME_ERROR_NONE		(0) //未发生错误
#define STANDARD_HOME_ERROR_DISABLE	(10) //执行回原点的轴未使能
#define STANDARD_HOME_ERROR_ALARM		(20) //执行回原点的轴报警
#define STANDARD_HOME_ERROR_STOP		(30) //未完成回原点，被停止运动
#define STANDARD_HOME_ERROR_ON_LIMIT   (40) //触发了限位无法继续
#define STANDARD_HOME_ERROR_NO_HOME	(50) //未找到Home
#define STANDARD_HOME_ERROR_NO_INDEX  (60) //未找到Index
#define STANDARD_HOME_ERROR_NO_LIMIT (70) //未找到限位
#define STANDARD_HOME_ERROR_SET_CAPTURE (80) //设置捕获错误
#define STANDARD_HOME_ERROR_ENCODER_DIR_SCALE (-1) //规划器与编码器方向方向相反或者当量不一致
#define STANDARD_HOME_ERROR_NO_MOTOR_STOP_CHECK	(-2) //没有相关的电机到位判断


typedef struct StandardHomePrm
{
    short mode;		      // 回原点模式取值范围1~36
    double highSpeed;     // 搜索Home的速度，单位pulse/ms
    double lowSpeed;	  // 搜索Index的速度，单位pulse/ms
    double acc;		      // 回零加速度，单位pulse/ms^2
    long offset;          // 回零偏移量，单位pulse
    short check;          // 是否启用自检功能，1-启用，其它值-不启用
    short autoZeroPos;    // 回零完毕是否自动清零，1-自动清零，其它值-不清零
    long motorStopDelay;  //电机到位延时，单位：控制周期
    short pad1[3];	      // 保留（不需要设置）
} TStandardHomePrm;

typedef struct StandardHomeStatus
{
    short run;           // 是正在进行回原点，0—已停止运动，1-正在回原点
    short stage;         // 回原点运动的阶段
    short error;         // 回原点过程的发生的错误
    short pad1[3];       // 保留（无具体含义）
    long capturePos;     // 捕获到Home或Index时刻的编码器位置
    long targetPos;      // 需要运动到的目标位置（原点位置或者原点位置+偏移量），在搜索Limit时或者搜索Home或Index时，设置的搜索距离为0，那么该值显示为805306368
} TStandardHomeStatus;

GT_API GT_ExecuteStandardHome(short axis, TStandardHomePrm* pHomePrm);
GT_API GT_GetStandardHomePrm(short axis, TStandardHomePrm* pHomePrm);
GT_API GT_GetStandardHomeStatus(short axis, TStandardHomeStatus* pHomeStatus);

GT_API GTN_ExecuteStandardHome(short core, short axis, TStandardHomePrm* pHomePrm);
GT_API GTN_GetStandardHomePrm(short core, short axis, TStandardHomePrm* pHomePrm);
GT_API GTN_GetStandardHomeStatus(short core, short axis, TStandardHomeStatus* pHomeStatus);

#define POS_COMPARE_RESULT_FIFO_MODE_STATIC                (0)
#define POS_COMPARE_RESULT_FIFO_MODE_LOOP                  (1)





GT_API GTN_SetPosCompareFifoMode(short core, short index, short mode);
GT_API GTN_GetPosCompareFifoMode(short core, short index, short* pMode);
GT_API GTN_GetPosCompareLatchValue(short core, short index, long count, long* pDataX, long* pDataY, long* pCount, TLatchValueInfo* pInfo);


typedef struct TaskMoveEscape
{
    unsigned long profileMask;
    long offset;
    double vel;
    double acc;
}TTaskMoveEscape;

typedef struct EventStatus
{
    short eventHit;
    short occupyStatus;
    short enableStatus;
    short pad1;
    long pad2[2];
    double pad3[2];
}TEventStatus;

typedef struct TaskStatus
{
    short start;
    short execute;
    short occupyStatus;
    short executeCount;
    long pad2[2];
    double pad3[2];
}TTaskStatus;
GT_API GTN_GetEventStatus(short core, short index, TEventStatus* pStatus);
GT_API GTN_GetTaskStatus(short core, short index, TTaskStatus* pStatus);
GT_API GTN_SetTriggerMoveEscape(short core, short trigger, TTaskMoveEscape* pPrm);
GT_API GTN_GetPrfRemainderPos(short core, short profile, double* pValue, short count, unsigned long* pClock);


/*-----------------------------------------------------------*/
/* MPG							                                        */
/*-----------------------------------------------------------*/
typedef struct MpgInfo
{
    double pos;
    double vel;
    double reserve[2];
    long di;
    long reserve1[3];
}TMpgInfo;

GT_API GTN_EnableMpgRoundOff(short core, short mpgIndex, short count = 1);
GT_API GTN_DisableMpgRoundOff(short core, short mpgIndex, short count = 1);
GT_API GTN_ReadMpgInfo(short core, short mpg, TMpgInfo* pMpgInfo);
GT_API GTN_WriteMpgPos(short core, short mpg, double* pPos, short count = 1);
GT_API GTN_ReadAuEncPos(short core, short encoder, double* pPos, short count = 1);
GT_API GTN_WriteAuEncPos(short core, short encoder, double* pPos, short count = 1);
GT_API GTN_ClearMcStatus(short core);
GT_API GTN_SetMcInfo(short core, long info, long index, unsigned long data);
GT_API GTN_GetMcInfo(short core, long info, long index, unsigned long* pData);
GT_API GTN_SetMcMode(short core, short mode, short value);
GT_API GTN_GetMcMode(short core, short mode, short* pValue);



GT_API GTN_GetRNMasterInfo(short core, unsigned short* pPhyId, unsigned short* pType, unsigned short* pInfo);
GT_API GTN_SetRNMasterInfo(short core, unsigned short phyId, unsigned short type, unsigned short info);
GT_API GTN_DmaWriteBlock(short core, unsigned long long offsetBytes, void* pBuf, unsigned long bytes, int wait);

GT_API GTN_RN_MltPcPduRd(short core, unsigned char* pData, unsigned char des_id, unsigned short byte_start_offset, unsigned short byte_num);
GT_API GTN_RN_MltPcPduRdUpdate(short core, unsigned char des_id);
GT_API GTN_RN_MltPcPduWr(short core, unsigned char* pData, unsigned char des_id, unsigned short byte_start_offset, unsigned short byte_num);
GT_API GTN_RN_MltPcPduWrUpdate(short core, unsigned char des_id);




//////////////////////////////////////////////////////////////////////////
//Command List
//////////////////////////////////////////////////////////////////////////

#define LIST_STATUS_RESERVE2_REVERSE_LAST_USER_TAG          (4)      // 指令流回退时，保留区最后一段的用户标签
#define COMMAND_LIST_MODE_STATIC        (0)
#define COMMAND_LIST_MODE_DYNAMIC       (1)

typedef struct CommandListStatus
{
    short execute;
    short empty;
    short stopInfo;
    short reserve1;
    short motionDone;
    short commandType;
    short command;
    short direction;
    long executeSegNum;
    long remainderSegCount;
	long userTag;
	long reserve2[5];
} TCommandListStatus;

typedef struct CommandListConfig
 {
    short mode;
    short elementSize;
    short reserve1[6];
    long forwardSpace;
    long reverseSpace;
    long reserve2[6];
} TCommandListConfig;

typedef struct CommandListLink
{
    short command;
    short index[2];
    short reserve[5];
} TCommandListLink;

typedef struct CommandListStartParameter
{
    short startMode;
    short reserve1[15];
} TCommandListStartParameter;

typedef struct CommandListStaticParameter
{
    unsigned long loop;
    short reserve1[10];
    long reserve2[2];
} TCommandListStaticParameter;

typedef struct CommandListEstimatePrm
{
    double velOverride;
    double reserve[40];
} TCommandListEstimatePrm;

typedef struct CommandListEstimateInfo
{
    short mode;
    short reserve1[3];
    double time;
    double reserve2[8];
} TCommandListEstimateInfo;

GT_API GTN_GetCommandListSpace(short core, short list, long* pSpace);
GT_API GTN_GetCommandListStatus(short core, short list, TCommandListStatus* pStatus);
GT_API GTN_ClearCommandListData(short core, short list, TListInfo* pListInfo = NULL);
GT_API GTN_CommandListDataEnd(short core, short list);
GT_API GTN_StartCommandList(short core, short list, TListInfo* pListInfo = NULL);
GT_API GTN_StartMultiCommandList(short core, unsigned long mask, TListInfo* pListInfo = NULL);
GT_API GTN_StopCommandList(short core, short index, short stopMode, TListInfo* pListInfo = NULL);
GT_API GTN_SetCommandListStartParameter(short core, short list, TCommandListStartParameter* pPrm, TListInfo* pListInfo);
GT_API GTN_GetCommandListStartParameter(short core, short list, TCommandListStartParameter* pPrm);
GT_API GTN_StopMultiCommandList(short core, unsigned long mask, short stopMode, TListInfo* pListInfo = NULL);
GT_API GTN_ClearCommandListStatus(short core, short index, TListInfo* pListInfo = NULL);
GT_API GTN_SetCommandListConfig(short core, short index, TCommandListConfig* pConfig);
GT_API GTN_GetCommandListConfig(short core, short index, TCommandListConfig* pConfig);
GT_API GTN_SetCommandListLink(short core, short linkId, short list, TCommandListLink* pLink);
GT_API GTN_GetCommandListLink(short core, short linkId, short* pList, TCommandListLink* pLink);
GT_API GTN_ClearCommandListLink(short core, short index);
GT_API GTN_DeleteCommandListLink(short core, short linkId);
GT_API GTN_SetCommandListStopParameter(short core, short list, TCommandListStopParameter* pPrm, TListInfo* pListInfo);
GT_API GTN_GetCommandListStopParameter(short core, short list, TCommandListStopParameter* pPrm);
GT_API GTN_SetCommandListDirection(short core, short index, short direction, TListInfo* pListInfo);
GT_API GTN_GetCommandListDirection(short core, short index, short* pDirection);
GT_API GTN_CommandListDataPush(short core, short list);
GT_API GTN_SetCommandListLinkGroup(short core, short list, unsigned long linkGroupMask);
GT_API GTN_GetCommandListLinkGroup(short core, short list, unsigned long* pLinkGroupMask);
GT_API GTN_SetCommandListStaticParameter(short core, short list, TCommandListStaticParameter* pPrm);
GT_API GTN_GetCommandListStaticParameter(short core, short list, TCommandListStaticParameter* pPrm);
GT_API GTN_SetCommandListEstimateMode(short core, short list, short mode, TCommandListEstimatePrm* pEstimatePrm);
GT_API GTN_GetCommandListEstimateInfo(short core, short list, TCommandListEstimateInfo* pEstimateInfo);

// 新架构DMA指令，
GT_API GTN_CommandListHsMode(short core,short list,short hsEnable,short link,unsigned short threshold=1,short lookAheadInMc=0);
GT_API GTN_BatchListCommandBegin(short core,short list);
GT_API GTN_BatchListCommandEnd(short core,short list);

#define COMMAND_LIST_MPG_MODE_BIDIRECTION				        (0)			//正负向都可以固定缓冲区模式
#define COMMAND_LIST_MPG_MODE_POS						        (1)			//MPG正方向转动 正向插补
#define COMMAND_LIST_MPG_MODE_NEG						        (-1)		//MPG负方向转动 正向插补
#define COMMAND_LIST_MPG_MODE_WINDOW 					        (2)			//正负向都可以滚动缓冲区模式
#define COMMAND_LIST_MPG_MODE_SIM						        (100)

#define COMMAND_LIST_MPG_FIXED_VEL_COUNT_MAX     (8)
typedef struct CommandListMpgPrm
{
    short enable;                                                    // 使能标志
    short master;                                                    // 手轮主轴号
    short filterTime;                                                // 手轮主轴滤波时间
    short mode;                                                      // 手轮的模式
    short fixedMpgVelCount;                                          // 手轮引导的挡位个数
    short pad1[3];
    long masterEven;                                              // 手轮主轴比例
    long slaveEven;                                               // 手轮从轴比例
    long pad2[2];
    double fixedMpgVel[COMMAND_LIST_MPG_FIXED_VEL_COUNT_MAX];        // 手轮引导的挡位Ratio值
    double ratioUpdateTime;                                          // 手轮引导倍率刷新时间
    double pad3[3];
}TCommandListMpgPrm;

GT_API GTN_SetCommandListMpgMode(short core,short list,TCommandListMpgPrm *pCommandListMpgPrm,TListInfo *pListInfo);
GT_API GTN_GetCommandListMpgMode(short core,short list,TCommandListMpgPrm *pCommandListMpgPrm);
typedef struct ProfileScale
{
    short  count;
    short  reverse1[3];
    double alpha[4];
    double beta[4];
}TProfileScale;
GT_API GTN_SetAxisScale(short core, short profile, TProfileScale* pScale, TListInfo* pListInfo = NULL);
GT_API GTN_GetAxisScale(short core, short profile, TProfileScale* pScale);

/*-----------------------------------------------------------*/
/* Axis                                                      */
/*-----------------------------------------------------------*/
#define AXIS_MOTION_CONSTRAINT_RESERVE1_DV_MAX_LIMIT       (0) // 用于设置dvmax的约束是否生效，0：根据模型默认，1：生效，-1：不生效
typedef struct AxisMotionConstraint
{
    double velMax;
    double accMax;
    double decMax;
    double jerkMax;
    double dvMax;
    short reverseLimitMode;
    short reserve1[3];
    double reserve2[8];
} TAxisMotionConstraint;
GT_API GTN_SetAxisMotionConstraint(short core, short axis, TAxisMotionConstraint* pPrm, TListInfo* pListInfo);
GT_API GTN_GetAxisMotionConstraint(short core, short axis, TAxisMotionConstraint* pPrm);

typedef struct ScaleParameter
{
    short  count;
    short  reserve[3];
    double alpha[4];
    double beta[4];
}TScaleParameter;
GT_API GTN_SetScaleParameter(short core, short type, short index, TScaleParameter* pScalePrm, TListInfo* pListInfo);
GT_API GTN_GetScaleParameter(short core, short type, short index, TScaleParameter* pScalePrm);

GT_API GTN_SetAxisGearRatio(short core, short profile, double* pGearRatio, short count, TListInfo* pListInfo);
GT_API GTN_GetAxisGearRatio(short core, short profile, double* pGearRatio, short count);

typedef struct AxisMotorParameter
{
    double currentToTorqueCoef;
    double ratedCurrent;
    double reserve[4];
} TAxisMotorParameter;
GT_API GTN_SetAxisMotorParameter(short core, short profile, TAxisMotorParameter* pPrm, TListInfo* pListInfo);
GT_API GTN_GetAxisMotorParameter(short core, short profile, TAxisMotorParameter* pPrm);

typedef struct AxisStopParameter
{
    double deceleration;
    double jerk;
    short reserve1[4];
    double reserve2[5];
} TAxisStopParameter;
GT_API GTN_SetAxisStopParameter(short core, short profile, short type, TAxisStopParameter* pPrm, TListInfo* pListInfo);
GT_API GTN_GetAxisStopParameter(short core, short profile, short type, TAxisStopParameter* pPrm);
GT_API GTN_GetProfileGroupInfo(short core, short profile, short* pGroup, short* pIndentInGroup);

GT_API GTN_SetVelOverride(short core, short index, double velRatio, TListInfo* pListInfo);
GT_API GTN_GetVelOverride(short core, short index, double* pVelRatio);

/*-----------------------------------------------------------*/
/* New Watch  Code                                           */
/*-----------------------------------------------------------*/
#define WATCH_GROUP_TIMER					(0)
#define WATCH_GROUP_BACKGROUND				(1)

#define WATCH_MODE_STATIC					(0)
#define WATCH_MODE_LOOP						(1)
#define WATCH_MODE_DYNAMIC					(2)

#define WATCH_EVENT_MAX						(8)

#define WATCH_EVENT_RUN						(1)
#define WATCH_EVENT_START					(10)
#define WATCH_EVENT_STOP					(20)
#define WATCH_EVENT_OFF						(30)

#define WATCH_CONDITION_NONE				(0)

#define WATCH_CONDITION_EQ					(1)
#define WATCH_CONDITION_NE					(2)
#define WATCH_CONDITION_GE					(3)
#define WATCH_CONDITION_LE					(4)
#define WATCH_CONDITION_GT  				(5)
#define WATCH_CONDITION_LT  				(6)
#define WATCH_CONDITION_CHANGE_TO			(11)
#define WATCH_CONDITION_CHANGE				(12)
#define WATCH_CONDITION_UP					(13)
#define WATCH_CONDITION_DOWN				(14)
#define WATCH_CONDITION_CHANGE_BEYOND		(15)
#define WATCH_CONDITION_CHANGE_MAX			(16)

#define WATCH_CONDITION_REMAIN_AT			(21)
#define WATCH_CONDITION_REMAIN				(22)
#define WATCH_CONDITION_CROSS_POSITIVE		(23)
#define WATCH_CONDITION_CROSS_NEGATIVE		(24)

#define WATCH_CONDITION_NEAREST				(31)
#define WATCH_CONDITION_DELTA				(32)


#define WATCH_VAR_NONE						(0)

#define WATCH_VAR_UUID						(1000)

#define WATCH_VAR_POWER_ON_COUNT			(1060)

#define WATCH_VAR_DSP_PROGRAM_COUNT			(1070)

#define WATCH_VAR_DSP_TEMPERATURE			(1072)

#define WATCH_VAR_FPGA_PROGRAM_COUNT		(1080)

#define WATCH_VAR_CLOCK						(1200)
#define WATCH_VAR_PRF_LOOP					(1201)
#define WATCH_VAR_PRF_PERIOD				(1205)
#define WATCH_VAR_PRF_TIME					(1206)

#define WATCH_VAR_SKIP_MODULE				(1211)
#define WATCH_VAR_TEST_MODE					(1212)

#define WATCH_VAR_COMMAND_CODE				(1220)
#define WATCH_VAR_COMMAND_DATA				(1221)
#define WATCH_VAR_COMMAND_COUNT				(1222)
#define WATCH_VAR_COMMAND_READ_FLAG			(1223)

#define WATCH_VAR_LINK_PORT_TX_USE			(1660)
#define WATCH_VAR_LINK_PORT_TX_REQUIRE		(1661)
#define WATCH_VAR_LINKPORT_TX_STATUS		(1662)
#define WATCH_VAR_LINKPORT_TX_ERROR_COUNT	(1663)
#define WATCH_VAR_LINKPORT_TX_SKIP_MAX		(1664)

#define WATCH_VAR_SPORT_STATUS				(1682)
#define WATCH_VAR_SPORT_ERROR_COUNT			(1683)


#define WATCH_VAR_PRF_POS					(6000)
#define WATCH_VAR_PRF_VEL					(6001)
#define WATCH_VAR_PRF_ACC					(6002)
#define WATCH_VAR_PRF_POS_RAW				(6003)
#define WATCH_VAR_PRF_VEL_RAW				(6004)

#define WATCH_VAR_PRF_VEL_F32				(6011)
#define WATCH_VAR_PRF_ACC_F32				(6012)
#define WATCH_VAR_PRF_JERK_F32				(6013)

#define WATCH_VAR_PRF_VEL_64				(6021)

#define WATCH_VAR_PRF_RUN					(6200)
#define WATCH_VAR_LIMIT_POSITIVE_STATUS     (6201)
#define WATCH_VAR_LIMIT_NEGATIVE_STATUS     (6202)

// 6600 Trap

#define WATCH_VAR_PT_PRF_POS_F64			(6650)
#define WATCH_VAR_PT_SPACE					(6660)
#define WATCH_VAR_PT_RECEIVE				(6662)
#define WATCH_VAR_PT_EXECUTE				(6663)

#define WATCH_VAR_TRAP_TOTAL_TIME           (6670)
#define WATCH_VAR_TRAP_REMAIN_TIME          (6671)

// 6700 PVT

// 6750 Follow

// 7000 MoveAbsolute
#define WATCH_VAR_MOVE_ABSOLUTE_DONE		(7008)

// 7050 MoveVelocity
#define WATCH_VAR_MOVE_VELOCITY_IN_VELOCITY	(7058)

#define WATCH_VAR_CRD_PRF_POS				(8000)
#define WATCH_VAR_CRD_PRF_VEL				(8001)
#define WATCH_VAR_CRD_PRF_ACC				(8002)
#define WATCH_VAR_CRD_PRF_VEL_RAW			(8003)

#define WATCH_VAR_CRD_PRF_TEMP_DATA         (8010)

#define WATCH_VAR_CRD_RUN					(8200)

#define WATCH_VAR_CRD_SEGMENT_NUMBER		(8202)
#define WATCH_VAR_CRD_SEGMENT_NUMBER_USER	(8203)
#define WATCH_VAR_CRD_COMMAND_RECEIVE		(8204)
#define WATCH_VAR_CRD_COMMAND_EXECUTE		(8205)

#define WATCH_VAR_CRD_FOLLOW_SLAVE_POS		(8600)
#define WATCH_VAR_CRD_FOLLOW_SLAVE_VEL		(8601)

#define WATCH_VAR_CRD_FOLLOW_STAGE			(8610)

#define WATCH_VAR_CONTOUR_PEDAL_POS			(8800)
#define WATCH_VAR_CONTOUR_PRF_POS			(8801)
#define WATCH_VAR_CONTOUR_PRF_POS_SHAPING	(8802)
#define WATCH_VAR_CONTOUR_ENC_POS			(8803)
#define WATCH_VAR_CONTOUR_DAC               (8804)

#define WATCH_VAR_CONTOUR_FIFO_POINT_COUNT	(8808)
#define WATCH_VAR_CONTOUR_FIFO_POINT_MAX	(8809)

#define WATCH_VAR_LIST_EXECUTE				(9240)
#define WATCH_VAR_LIST_SEG_NUM              (9250)

#define WATCH_VAR_CATCH_UP_STATE                           (12000)
#define WATCH_VAR_CATCH_UP_COMMAND                         (12001)
#define WATCH_VAR_CATCH_UP_MASTER_POS                      (12002)
#define WATCH_VAR_CATCH_UP_MASTER_VEL                      (12003)
#define WATCH_VAR_CATCH_UP_SLAVE_POS                       (12004)
#define WATCH_VAR_CATCH_UP_SLAVE_VEL                       (12005)
#define WATCH_VAR_CATCH_UP_PIECE_POS                       (12006)
#define WATCH_VAR_CATCH_UP_PIECE_ID                        (12007)
#define WATCH_VAR_CATCH_UP_PIECE_COUNT                     (12008)
#define WATCH_VAR_CATCH_UP_PIECE_FIFO_COMMAND_RECEIVE      (12009)
#define WATCH_VAR_CATCH_UP_PIECE_FIFO_COMMAND_SEND         (12010)
#define WATCH_VAR_CATCH_UP_SYNCH_DISTANCE                  (12011)
#define WATCH_VAR_CATCH_UP_SYNCN_TIME                      (12012)
#define WATCH_VAR_CATCH_UP_PIECE_START_POS                 (12013)
#define WATCH_VAR_CATCH_UP_PIECE_SYNCH_POS                 (12014)
#define WATCH_VAR_CATCH_UP_SLAVE_START_POS                 (12015)
#define WATCH_VAR_CATCH_UP_SLAVE_SYNCH_POS                 (12016)

#define WATCH_VAR_POS_COMPARE_COMMAND_RECEIVE	(17422)
#define WATCH_VAR_POS_COMPARE_COMMAND_SEND		(17423)
#define WATCH_VAR_POS_COMPARE_COMMAND_LEFT		(17424)
#define WATCH_VAR_POS_COMPARE_COMMAND_TX		(17425)
#define WATCH_VAR_POS_COMPARE_PULSE_COUNT		(17426)
#define WATCH_VAR_POS_COMPARE_PSO_ON_OFF		(17427)
#define WATCH_VAR_POS_COMPARE_PSO_REG			(17430)
#define	WATCH_VAR_POS_COMPARE_PSO_REG_EXT		(17431)
#define	WATCH_VAR_POS_COMPARE_ENC_PSO_ON_OFF	(17432)

#define WATCH_VAR_SCAN_PRF_POS				(18000)
#define WATCH_VAR_SCAN_PRF_VEL				(18001)
#define WATCH_VAR_SCAN_PRF_ACC				(18002)

#define WATCH_VAR_SCAN_PRF_POS_X			(18010)
#define WATCH_VAR_SCAN_PRF_POS_Y			(18020)
#define WATCH_VAR_SCAN_PRF_POS_Z			(18030)

#define WATCH_VAR_SCAN_RUN					(18200)

#define WATCH_VAR_SCAN_SEGMENT_NUMBER		(18202)

#define WATCH_VAR_SCAN_COMMAND_RECEIVE		(18422)
#define WATCH_VAR_SCAN_COMMAND_SEND			(18423)
#define WATCH_VAR_SCAN_COMMAND_LEFT			(18424)
#define WATCH_VAR_SCAN_COMMAND_TX			(18425)

#define WATCH_VAR_LASER_HSIO				(18600)
#define WATCH_VAR_LASER_POWER				(18601)
#define WATCH_VAR_LASER_WORK_RATIO			(18602)

#define WATCH_VAR_AXIS_PRF_POS				(20000)
#define WATCH_VAR_AXIS_PRF_VEL				(20001)
#define WATCH_VAR_AXIS_PRF_ACC				(20002)

#define WATCH_VAR_AXIS_PRF_POS_OTHER		(20004)
#define WATCH_VAR_AXIS_PRF_VEL_OTHER		(20005)

#define WATCH_VAR_AXIS_PRF_POS_FILTER		(20010)
#define WATCH_VAR_AXIS_PRF_VEL_FILTER		(20011)

#define WATCH_VAR_AXIS_TEMP_PRF_POS_FILTER  (20012)
#define WATCH_VAR_AXIS_PRF_POS_FILTER_BEFORE_COMP (20013)

#define WATCH_VAR_AXIS_PRF_VEL_64			(20021)

#define WATCH_VAR_AXIS_STATUS		        (20100)

#define WATCH_VAR_AXIS_RUN		            (20200)

#define WATCH_VAR_AXIS_ENC_POS_FILTER		(20310)
#define WATCH_VAR_AXIS_ENC_VEL_FILTER		(20311)

#define WATCH_VAR_AXIS_ENC_VEL_64_FILTER	(20321)

#define WATCH_VAR_AXIS_PREDICTION_POS_IN    (20350)
#define WATCH_VAR_AXIS_PREDICTION_POS_IN_2  (20351)
#define WATCH_VAR_AXIS_PREDICTION_POS_OUT   (20352)

#define WATCH_VAR_AXIS_PREDICTION_VEL_IN    (20355)
#define WATCH_VAR_AXIS_PREDICTION_VEL_IN_2  (20356)
#define WATCH_VAR_AXIS_PREDICTION_VEL_OUT   (20357)

#define WATCH_VAR_AXIS_PREDICTION_OUT       (20360)
#define WATCH_VAR_AXIS_PREDICTION_FILTER_POS                    (20361)
#define WATCH_VAR_AXIS_PREDICTION_FILTER_VEL                    (20362)

#define WATCH_VAR_POSCOMPARE_PREDICTION_ENABLE                  (20370)
#define WATCH_VAR_POSCOMPARE_PREDICTION_WORK                    (20371)
#define WATCH_VAR_POSCOMPARE_PREDICTION_OUTPUT_COUNT_TOTAL      (20372)

#define WATCH_VAR_DRIVER_ACT_VEL			(20400)
#define WATCH_VAR_STIMULATE_DATA			(20401)
#define WATCH_VAR_DRIVER_CURRENT_KVFF		(20402)

#define WATCH_VAR_CONTROL_ERROR				(20500)

#define WATCH_VAR_BACKLASH_VALUE			(21000)
#define WATCH_VAR_LEADSCREW_VALUE			(21001)
#define WATCH_VAR_FRICTION_VALUE			(21003)

#define WATCH_VAR_ENC_POS					(30000)
#define WATCH_VAR_ENC_VEL					(30001)
#define WATCH_VAR_AU_ENC_POS				(30002)
#define WATCH_VAR_AU_ENC_VEL				(30003)

#define WATCH_VAR_PULSE_POS					(30010)

#define WATCH_VAR_ENC_VEL_64				(30021)
#define WATCH_VAR_AU_ENC_VEL_64				(30022)
#define WATCH_VAR_ENC_VEL_64_1MS			(30023)
#define WATCH_VAR_AU_ENC_VEL_64_1MS			(30024)

#define WATCH_VAR_MPG_INFO					(30200)
#define WATCH_VAR_AU_ENCODER_EX_POS         (30201)
#define WATCH_VAR_AU_ENCODER_EX_VEL         (30202)
#define WATCH_VAR_MPG_ENCODER_POS           (30203)
#define WATCH_VAR_MPG_ENCODER_VEL           (30204)

#define WATCH_VAR_GPI						(31000)
#define WATCH_VAR_LIMIT_POSITIVE			(31010)
#define WATCH_VAR_LIMIT_NEGATIVE			(31020)
#define WATCH_VAR_ALARM   		        	(31030)
#define WATCH_VAR_HOME             			(31040)
#define WATCH_VAR_ARRIVE           			(31050)
#define WATCH_VAR_ARRIVE_DI                                            (31055)  // 到位输入信号
#define WATCH_VAR_SERVO_READY_DI                                (31056)  // 伺服使能完成输入信号
#define WATCH_VAR_SERVO_READY_TO_SWITCH_ON_DI     (31057)  // 伺服使能准备就绪输入信号
#define WATCH_VAR_BANK_GPI					(31060)

#define WATCH_VAR_GPO						(32000)
#define WATCH_VAR_ENABLE					(32010)
#define WATCH_VAR_CLEAR 					(32020)

#define WATCH_VAR_AI     					(33000)
#define WATCH_VAR_AU_AI                     (33100)

#define WATCH_VAR_AO     					(34000)

#define WATCH_VAR_TRIGGER_EXECUTE			(38000)
#define WATCH_VAR_TRIGGER_STATUS			(38001)
#define WATCH_VAR_TRIGGER_POSITION			(38002)

#define WATCH_VAR_TRIGGER_COUNT				(38010)

#define WATCH_VAR_TRIGGER_NOTIFY_ENABLE			(38020)
#define WATCH_VAR_TRIGGER_NOTIFY_STATUS_ECHO	(38021)
#define WATCH_VAR_TRIGGER_NOTIFY_CLEAR_WAIT		(38022)

#define WATCH_VAR_TRIGGER_DELTA_CROSS_COUNT		(38110)


#define WATCH_VAR_POS_LOOP_ERROR			(40000)
#define WATCH_VAR_POS_LOOP_REF_POS			(40001)
#define WATCH_VAR_POS_LOOP_FILTER_VALUE		(40002)

#define WATCH_VAR_FIR_POS_IN				(48000)
#define WATCH_VAR_FIR_POS_OUT				(48001)

#define WATCH_VAR_WATCH_ENABLE				(52000)
#define WATCH_VAR_WATCH_TIME				(52001)
#define WATCH_VAR_WATCH_EVENT_VALUE_WORK	(52002)

#define WATCH_VAR_INT32						(52020)
#define WATCH_VAR_INT64						(52021)
#define WATCH_VAR_FLOAT						(52022)
#define WATCH_VAR_DOUBLE					(52023)
#define WATCH_VAR_BOOL						(52024)


#define WATCH_VAR_TERMINAL_LIMIT_POSITIVE		(53000)
#define WATCH_VAR_TERMINAL_LIMIT_NEGATIVE		(53001)
#define WATCH_VAR_TERMINAL_ALARM				(53002)
#define WATCH_VAR_TERMINAL_HOME					(53003)
#define WATCH_VAR_TERMINAL_GPI					(53004)
#define WATCH_VAR_TERMINAL_ARRIVE				(53005)
#define WATCH_VAR_TERMINAL_MPG					(53009)

#define WATCH_VAR_TERMINAL_ENABLE				(53010)
#define WATCH_VAR_TERMINAL_CLEAR				(53011)
#define WATCH_VAR_TERMINAL_GPO					(53012)

#define WATCH_VAR_TERMINAL_DAC					(53020)

#define WATCH_VAR_TERMINAL_PULSE				(53022)
#define WATCH_VAR_TERMINAL_ENCODER				(53023)
#define WATCH_VAR_TERMINAL_ADC					(53024)

#define WATCH_VAR_TERMINAL_AU_ENCODER			(53026)

#define WATCH_VAR_TERMINAL_PRF_POS				(53030)

#define WATCH_VAR_TERMINAL_TRIGGER_POSITION	    (53040)
#define WATCH_VAR_TERMINAL_TRIGGER_STATUS	    (53041)

#define WATCH_VAR_TERMINAL_COMMAND				(53100)
#define WATCH_VAR_TERMINAL_STATUS				(53101)

#define WATCH_VAR_EVENT_HIT						(60000)
#define WATCH_VAR_TASK_START					(61000)
#define WATCH_VAR_TASK_WORK						(61001)


#define WATCH_VAR_ATL_TORQUE			        (61201)//读取驱动器的实际电流，比例关系为数值1000对应于驱动器电机参
#define WATCH_VAR_CONSTANT                      (61405)

#define VAR_FORMAT_INT						(1)
#define VAR_FORMAT_FLOAT					(2)
#define VAR_FORMAT_DOUBLE					(3)

#define WATCH_LOAD_MODE_NONE				(0)
#define WATCH_LOAD_MODE_BOOT				(2)
#define WATCH_LOAD_MODE_RUN					(3)



#define VAR_CALCULATE_MAX					(32)

#define VAR_CALCULATE_NONE					(0)
#define VAR_CALCULATE_OR					(1)
#define VAR_CALCULATE_AND					(3)
#define VAR_CALCULATE_NOT					(5)

#define VAR_CALCULATE_ADD					(11)
#define VAR_CALCULATE_SUB					(12)
#define VAR_CALCULATE_MUL					(13)
#define VAR_CALCULATE_DIV					(14)

// 用户接口高速读元素结构体
typedef struct ReadHsCommand
{
    long code;                                      // 需要读取的高速读元素编码,参见WATCH变量编码
    short index;                                    // 需要读取的高速读元素索引,从1开始
    short subIndex;                                 // 需要读取的高速读元素子索引,从1开始
    short pad1[2];                                  // 预留
}TReadHsCommand;
GT_API GTN_AddReadHs(short core, TReadHsCommand* pCommand);
GT_API GTN_LoadReadHsConfig(short core, char* pFile);
GT_API GTN_ReadHsOn(short core, short enable, short mode, double interval);
GT_API GTN_ClearReadHs(short core);
GT_API GTN_SetReadHs(short core, short enable, short mode, short interval);
GT_API GTN_ReadHsReadBuffer(short core, short* pData, long count);
GT_API GTN_FlushReadHs(short core, short wait);

typedef struct AxisArrivePrm
{
    short mode;
    short pad0;
    long band;		//控制器判断到位误差带
    long time;		//控制器判断到位时间
    long pad1[2];
}TAxisArrivePrm;
GT_API GTN_SetAxisArriveMode(short core, short axis, TAxisArrivePrm* pPrm);
GT_API GTN_GetAxisArriveMode(short core, short axis, TAxisArrivePrm* pPrm);


#define MAX_CRDMPG_FIXEDMPGVEL 8
typedef struct CrdMpgPrm
{
    short enable;                                                        //使能标志
    short master;                                                        //手轮主轴号
    short filterTime;                                                    //手轮主轴滤波时间
    short mode;                                                          //手轮的模式
    short fixedMpgVelCount;                                              //手轮引导的挡位个数
    short pad1[3];
    long masterEven;                                                     //手轮主轴比例
    long slaveEven;                                                      //手轮从轴比例
    long pad2[2];
    double fixedMpgVel[MAX_CRDMPG_FIXEDMPGVEL];                          //手轮引导的挡位Ratio值
    double ratioUpdateTime;                                              //手轮引导倍率刷新时间
    double pad3[3];
}TCrdMpgPrm;
GT_API GTN_SetCrdMPGMode(short core, short crd, short enable, short master, long masterEven, long slaveEven, short filterTime, short mode);
GT_API GTN_GetCrdMPGMode(short core, short crd, short* pEnable, short* pMaster, long* pMasterEven, long* pSlaveEven, short* pFilterTime, short* pMode, short* pFifoEnd);
GT_API GTN_SetCrdMPGModeEx(short core, short crd, TCrdMpgPrm* pMpgPrm);
GT_API GTN_GetCrdMPGModeEx(short core, short crd, short* pFifoEnd, TCrdMpgPrm* pCrdMpgPrm);


typedef struct TriggerProfilePrm
{
    short mode;	    //运动类型,0-点位，6-PVT，其它类型暂时不支持
    short enable;   //是否使能，0-不使能，1-使能
    short trigger;	//trigger索引，取值从1开始
    short pad1;		//保留
    long distance;	//触发时偏移量，可正可负，单位：脉冲
    long posLimit;  //重新规划后，触发位置+偏移量不能超过该值
    double vel;     //目标速度，重新规划的目标速度，单位：脉冲/ms
    double acc;     //需要提速时的加速度，单位：脉冲/ms
    double dec;		//运动到触发偏移量的减速度，单位：脉冲/ms
    double percent;	//减速段S型曲线时间百分比，例如60表示60%
    double reserve[4];
}TTriggerProfilePrm;

typedef struct TriggerProfileStatus
{
    short mode;
    short enable;
    short execute;			//是否执行中,0-未执行，1-执行
    short status;			//执行过程中的状态，正常为0，异常则返回错误码
    long endPos;			//终点位置（捕获+偏移量）
    long reserve[7];
}TTriggerProfileStatus;

#define TRIGGER_PROFILE_STATUS_NONE              (0)                 // 状态正常
#define TRIGGER_PROFILE_STATUS_ERROR_END_POS     (1)                 // 捕获到的位置或者设置的位置异常
#define TRIGGER_PROFILE_STATUS_VEL               (2)                 // 设置的目标速度小于当前运动速度
GT_API GTN_SetTriggerProfilePrm(short core, short profile, TTriggerProfilePrm* pPrm);
GT_API GTN_GetTriggerProfileStatus(short core, short profile, TTriggerProfileStatus* pSts);


GT_API GTN_PrintLogInfo(short core, const char* pFileName, long start = 0, long count = 0);
GT_API GTN_PrintLogInfoDebug(short core, const char* pFileName, long start = 0, long count = 0);
GT_API GTN_PrintMcStsInfo(short core, const char* pFileName, short type, short index, short count);
GT_API GTN_PrintCommandInfo(short core, const char* pFileName, long start, long count);

#define COMMANDINFO_DATA_MAX	     (225)
typedef struct CommandInfoData
{
    unsigned long   commandCode;                   //日志的指令字
    short   commandRtn;							   //2word对齐
    short   errorCode;
    unsigned long  clockTime;                      //控制器的时钟
    long    segmentNum;                            //段号
    long    userTag;                               //用户标签
    short   dataLength;
    unsigned short  data16[COMMANDINFO_DATA_MAX];  //
} TCommandInfoData;
//获取指令错误信息
GT_API GTN_GetLastCommandError(short core, TCommandInfoData* getCommandInfoData, long start = -1, long count = 1);

/*-----------------------------------------------------------*/
/* ILC                                                       */
/*-----------------------------------------------------------*/
typedef struct IlcResult
{
    double ErrorMax;
    double ErrorAvg;
    double ErrorRms;
    double pad1[9];
    short  pad2[10];
}TIlcResult;

GT_API GTN_InitIlc(short core);
GT_API GTN_StartIlc(short core, short crd);
GT_API GTN_StopIlc(short core, short crd, TIlcResult* pPrm);
GT_API GTN_SaveIlcFile(short core, char* pIlcFile);
GT_API GTN_LoadIlcFile(short core, short crd, char* filePath);

GT_API GTN_SetIterationLearnData(short core, short axis, double* pData, unsigned long count);
GT_API GTN_EnableIterationLearn(short core, short axis, short mode, double k);

GT_API GTN_SetIteraionData(short core, long count, double xPrf, double yPrf, double xEnc, double yEnc, short pso);
GT_API GTN_GetIteraionData(short core, long count, double* xPrf, double* yPrf, double* xEnc, double* yEnc, double* xRef, double* yRef, double* error, double* xErr, double* yErr, short* pso, long* index);







/*----------------*/
/*振镜激光相关指令*/
/*----------------*/
#define    SCAN_LASER_MODE_DUTY_RATIO          (0)
#define    SCAN_LASER_MODE_FREQUENCY           (1)
#define    SCAN_LASER_MODE_ANALOG              (2)
#define    SCAN_LASER_MODE_PARALLEL            (4)
#define    SCAN_LASER_MODE_NONE               (10)

/*占空比模式固定参数*/
typedef struct LaserDutyRatioModeParameterPro
{
    double minDutyRatio;
    double maxDutyRatio;
    double frequency;
}TLaserDutyRatioModeParameterPro;
/*频率模式固定参数*/
typedef struct LaserFrequencyModeParameterPro
{
    double minFrequency;
    double maxFrequency;
    double pulseWidth;
}TLaserFrequencyModeParameterPro;
/*并口式固定参数*/
typedef struct lasetParallelModeParameterPro
{
    double minParallel;                                    // 并口激光模式下的并口激光最小限制值
    double maxParallel;                                    // 并口激光模式下的并口激光最大限制值
}TlasetParallelModeParameterPro;
/*模拟量模式固定参数*/
typedef struct laserAnalogModeParameterPro
{
    double minVoltage;
    double maxVoltage;
}TlaserAnalogModeParameterPro;
/*激光各个模式固定参数*/
typedef union LaserParameterUnionPro
{
    TLaserDutyRatioModeParameterPro dutyRatioModePrm;
    TLaserFrequencyModeParameterPro frequencyModePrm;
    TlaserAnalogModeParameterPro analogModePrm;
    TlasetParallelModeParameterPro parallelModePrm;
    double data[8];
}TLaserParameterUnionPro;
/*激光信息参数*/
typedef struct LaserInfoPro
{
    unsigned short laserOn;   //激光开关状态
    unsigned short laserMode; //PWM输出模式，宏定义
    short pad[2];             //对齐
    double power;             //激光能量
    TLaserParameterUnionPro laserPrm;
}TLaserInfoPro;

typedef struct LaserParameterPro
{
    unsigned short laserMode; //PWM输出模式，定义宏
    short pad[3];             //对齐
    TLaserParameterUnionPro laserPrm;
}TLaserParameterPro;

GT_API GTN_GetScanLaserLinkPro(short core, short scanCrd, short* pLaserChannel);
GT_API GTN_SetScanLaserLinkPro(short core, short scanCrd, short laserChannel, TListInfo* pListInfo = NULL);//振镜激光绑定,laserChannel为0则是解绑
GT_API GTN_GetScanLaserInfoPro(short core, short scanCrd, TLaserInfoPro* pPrm);
GT_API GTN_SetScanLaserEnablePro(short core, short scanCrd, short laserEnable, TListInfo* pListInfo = NULL);//激光开关光
GT_API GTN_SetScanLaserPrmPro(short core, short scanCrd, TLaserParameterPro* pLaserPrm, TListInfo* pListInfo = NULL);//激光模式和固定参数设置
GT_API GTN_SetScanLaserPowerPro(short core, short scanCrd, double power, TListInfo* pListInfo = NULL);//激光能量设置
GT_API GTN_SetScanLaserDelayPro(short core, short scanCrd, double laserOnDelay, double laserOffDelay, TListInfo* pListInfo);

/*----------------*/
/*新版振镜激光相关指令，PWM、并口、模拟量可以同时在缓冲区中或者立即指令进行控制*/
/*----------------*/
typedef struct LaserPwmPrmPro
{
    double minDuty;                              // 占空比能量限制最小值，取值范围：[0,100]，单位：%
    double maxDuty;                              // 占空比能量限制最大值，取值范围：[0,100]，单位：%
    double minFrequency;                         // 频率能量限制最小值，取值范围：[0,1562.5]：单位：kHz
    double maxFrequency;                         // 频率能量限制最大值，取值范围：[0,1562.5]：单位：kHz
    double minPulseWidth;                        // 脉宽能量限制最小值，取值范围：[0,65535]，单位：us
    double maxPulseWidth;                        // 脉宽能量限制最大值，取值范围：[0,65535]，单位：us
}TLaserPwmPrmPro;
GT_API GTN_SetScanLaserPwmPrmPro(short core, short scanCrd, TLaserPwmPrmPro* pPrm, TListInfo* pListInfo);
GT_API GTN_GetScanLaserPwmPrmPro(short core, short scanCrd, TLaserPwmPrmPro* pPrm);

GT_API GTN_SetScanLaserPwmDutyPro(short core, short scanCrd, double duty, TListInfo* pListInfo);
GT_API GTN_SetScanLaserPwmFrequencyPro(short core, short scanCrd, double frequency, TListInfo* pListInfo);
GT_API GTN_SetScanLaserPwmPulseWidthPro(short core, short scanCrd, double pulseWidth, TListInfo* pListInfo);

GT_API GTN_SetScanLaserVoltagePrmPro(short core, short scanCrd, double minVoltage, double maxVoltage, TListInfo* pListInfo);
GT_API GTN_GetScanLaserVoltagePrmPro(short core, short scanCrd, double* pMinVoltage, double* pMaxVoltage);
GT_API GTN_SetScanLaserVoltagePro(short core, short scanCrd, double voltage, TListInfo* pListInfo);

GT_API GTN_SetScanLaserParallelPrmPro(short core, short scanCrd, double minParallel, double maxParallel, TListInfo* pListInfo);
GT_API GTN_GetScanLaserParallelPrmPro(short core, short scanCrd, double* pMinParallel, double* pMaxParallel);
GT_API GTN_SetScanLaserParallelPro(short core, short scanCrd, double parallel, TListInfo* pListInfo);

/*----------------*/
/*振镜相关指令    */
/*----------------*/
#define    SCAN_MOTION_MODE_NONE                (0)
#define    SCAN_MOTION_MODE_JUMP                (1)
#define    SCAN_MOTION_MODE_JUMP_POINT          (2)
#define    SCAN_MOTION_MODE_JUMP_TIME           (3)
#define    SCAN_MOTION_MODE_JUMP_TIME_POINT     (4)
#define    SCAN_MOTION_MODE_MARK                (5)
#define    SCAN_MOTION_MODE_MARK_TIME           (6)

#define    SCAN_MOTION_CIRCLE_DIR_CW            (0)
#define    SCAN_MOTION_CIRCLE_DIR_CCW           (1)
/*振镜前瞻参数*/
typedef struct ScanLookAheadParameterPro
{
    short lookAheadNum;   //前瞻段数
    short highSpeedMode;
    short pad[2];         //对齐
    double time;          //时间常数
    double radiusRatio;   //曲率限制调节参数
    double reserve[2];    //保留
}TScanLookAheadParameterPro;

/*振镜状态*/
typedef struct ScanStatusPro
{
    short run;                    //振镜运动标志
    short space;
    unsigned short fifoEmpty;     //跑空标志
    short pad1;                   //对齐
    unsigned long segmentNumber;  //执行段号
    unsigned long commandReceive; //接收到的指令数
    unsigned long commandSend;    //已发送的指令数
    long pad2;                    //对齐
    double prfVel;                //合成规划速度
}TScanStatusPro;

//Jump运动、Mark运动
typedef struct VelModePro
{
    double acc;
    double dec;
    double vel;
}TVelModePro;

typedef struct TimeModePro
 {
    double acc;
    double dec;
    unsigned long time;
    long pad;//对齐
}TTimeModePro;

typedef struct VelPointModePro
{
    double acc;
    double dec;
    double vel;
    unsigned long motionDelayTime;
    unsigned long laserDelayTime;
}TVelPointModePro;

typedef struct TimePointModePro
{
    double acc;
    double dec;
    unsigned long time;
    unsigned long motionDelayTime;
    unsigned long laserDelayTime;
    long pad;//对齐
}TTimePointModePro;

typedef union ScanMotionPrmUnionPro
{
    TVelModePro velMode;
    TTimeModePro timeMode;
    TVelPointModePro velPointMode;
    TTimePointModePro timePointMode;
    double data[8];
}TScanMotionPrmUnionPro;

typedef struct ScanLinearMotionPro
{
    double pos[3];
    double reserve;
    TScanMotionPrmUnionPro motionPrm;
}TScanLinearMotionPro;

typedef struct ScanCircularMotionPro
{
    double endPos[3];
    double radius;//半径正负区分优弧劣弧
    short dir;
    short pad[3];//对齐
    TScanMotionPrmUnionPro motionPrm;
}TScanCircularMotionPro;

typedef struct ScanDelayParameterPro
{
    short multiMarkDelayMode;
    unsigned short jumpDelayLengthLimit;
    short pad[2];//对齐
    double multiMarkLaserOffDelay;
    double multiMarkDelayConst;
    double markDelay;
    double minJumpDelay;
    double maxJumpDelay;
}TScanDelayParameterPro;

typedef struct ScanArcMotionPro
 {
    double endPos[3];               // 终点位置
    double centerPos[3];            // 圆心位置，即相对于起点位置的偏移量
    short circleDir;                // 运动方向
    short pad[3];//对齐
    TScanMotionPrmUnionPro motionPrm;
}TScanArcMotionPro;

/**
 * @brief 批量数据包发送状态
*/
typedef struct BatchDataSendingSts
{
    short enable;                               /*!< 使能状态                       */
    int16_t batchCmdReceive;                      /*!< 批量数据包接收个数             */
    int16_t batchCmdSend;                         /*!< 批量数据包发送个数             */
    int16_t fifoSpace;                            /*!< 批量数据包fifo剩余空间         */
}TBatchDataSendingSts;

/**
 * @brief 振镜批量发送数据状态
*/
typedef struct ScanBatchDataSendingSts
{
    TBatchDataSendingSts batchDataSts;            /*!< 批量包发送状态                 */
    int16_t batchSendCount;                       /*!< 当前振镜发送的包个数           */
    int16_t wordDataSendCount;                    /*!< 当前振镜发送的数据个数         */
    int16_t stationFifoSpace;                     /*!< 当前从站振镜Fifo剩余空间       */
    int32_t cmdCount;                             /*!< 已经发送到模块的指令段数       */
    int16_t reserve[7];                           /*!< 保留参数                       */
}TScanBatchDataSendingSts;
/**
 * @brief 读取批量数据传输状态和振镜数据发送状态
 * @param core 核索引
 * @param scanIndex 振镜缩影
 * @param pScanBatchDataSendingSts
 * @return 0：    指令执行成功
 *         8：    dsp固件版本不支持
 *         17052：资源类型参数不支持
 *         17055：内部参数错误，只能取值为0
 *         17058：资源类型参数超范围，取值范围[1,4]
 *         其他负值返回值：参考等环网返回值说明
*/
GT_API GTN_GetScanBatchDataSendingSts(int16_t core,int16_t scanIndex,TScanBatchDataSendingSts *pScanBatchDataSendingSts);
/**
 * @brief 使能振镜批量数据发送，需要搭配新的模块固件使用
 * @param core 核索引
 * @param enable 使能或者关闭振镜批量数据发送，0：关闭，1：打开
 * @return 0：    指令执行成功
 *         8：    dsp固件版本不支持
 *         17052：资源类型参数不支持
 *         17054：使能参数错误，取值范围[0,1]
 *         17055：内部参数错误，只能取值为0
 *         其他负值返回值：参考等环网返回值说明
*/
GT_API GTN_EnableScanBatchDataSend(int16_t core,int16_t enable);

GT_API GTN_ScanInitPro(short core, short scanCrd, TScanLookAheadParameterPro* pPrm, TListInfo* pListInfo);
GT_API GTN_GetScanCrdPosPro(short core, short scanCrd, double* pPos);
GT_API GTN_GetScanStatusPro(short core, short scanCrd, TScanStatusPro* pPrm);
GT_API GTN_ScanLinearPro(short core, short scanCrd, short motionMode, TScanLinearMotionPro* pPrm, TListInfo* pListInfo);
GT_API GTN_ScanCircularPro(short core, short scanCrd, short motionMode, TScanCircularMotionPro* pPrm, TListInfo* pListInfo);
GT_API GTN_ScanArcMotionPro(short core, short scanCrd, short motionMode, TScanArcMotionPro* pPrm, TListInfo* pListInfo);

#define SCAN_MARK_DELAY_MODE_CONST				(0)	// 固定时间延时模式
#define SCAN_MARK_DELAY_MODE_CHANGE				(1)	// 变延时模式

GT_API GTN_SetScanDelayPrmPro(short core, short scanCrd, TScanDelayParameterPro* pPrm, TListInfo* pListInfo = NULL);
GT_API GTN_SetScanMotionDelayPro(short core, short scanCrd, double delayTime, TListInfo* pListInfo);
GT_API GTN_GetScanExecuteTimePro(short core, short scanCrd, double* pExecuteTime);
GT_API GTN_ClearScanExecuteTimePro(short core, short scanCrd);

#define LASER_ENABLE_DO	             (76)    //激光器使能信号
#define LASER_RED_LED_DO             (77)    //激光器红灯指示信号
#define LASER_POWER_LATCH_DO         (78)    //激光器功率锁存信号
GT_API GTN_SetScanLaserIOPro(short core, short scanCrd, short doType, long doValue, TListInfo* pListInfo = NULL);

//20200713同步GTS的冰海定制，编码器断线检测
GT_API GT_SetEncResponseCheck(short control, short dacThreshold, double minEncVel, long time);
GT_API GT_GetEncResponseCheck(short control, short* pDacThreshold, double* pMinEncVel, long* pTime);
GT_API GT_EnableEncResponseCheck(short control);
GT_API GT_DisableEncResponseCheck(short control);

GT_API GTN_SetEncResponseCheck(short core, short control, short dacThreshold, double minEncVel, long time);
GT_API GTN_GetEncResponseCheck(short core, short control, short* pDacThreshold, double* pMinEncVel, long* pTime);
GT_API GTN_EnableEncResponseCheck(short core, short control);
GT_API GTN_DisableEncResponseCheck(short core, short control);

/************************************************************************/
/* 旋转轴功能                                                           */
/************************************************************************/
#define ROTARY_DIRECTION_SELECT_MODE_DEFAULT                  (0)
#define ROTARY_DIRECTION_SELECT_MODE_SMART                    (1)

typedef enum McDirection
{
	MC_POSITIVE_DIRECTION,
	MC_NEGATIVE_DIRECTION,
	MC_CURRENT_DIRECTION,
	MC_SHORTEST_WAY,
	MC_BY_POS,
} EMcDirection;

typedef struct RotaryConfig
{
    short rotary;
    short pad[3];
    double start;
    double length;
    double reserve;
    double pulse;
} TRotaryConfig;

typedef struct LineAbsolutePrm
{
    short fifo;
    short overrideNum;
    short g0Mode;
    short reserve1[5];
    long segNum;
    long reserve2[3];
    double vel;
    double acc;
    double velEnd;
    double velLimit;
    double reserve3[8];
} TLineAbsolutePrm;

GT_API GTN_GetAxisPrfPosRotary(short core, short axis, double* pTheta, double* pRound, short count = 1);
GT_API GTN_GetProfileRotaryConfig(short core, short profile, TRotaryConfig* pConfig);
GT_API GTN_GetPrfPosRotary(short core, short profile, double* pTheta, double* pRound, short count = 1);
GT_API GTN_SetCrdScale(short core, short crd, short dimension, double alpha, double beta);
GT_API GTN_GetCrdScale(short core, short crd, short dimension, double* pAlpha, double* pBeta);
GT_API GTN_GetCrdRotaryConfig(short core, short crd, short dimension, TRotaryConfig* pConfig);
GT_API GTN_GetCrdPosRotary(short core, short crd, short dimension, double* pTheta, double* pRound, short count = 1);
GT_API GTN_GetEncoderRotaryConfig(short core, short encoder, TRotaryConfig* pConfig);
GT_API GTN_GetEncPosRotary(short core, short encoder, double* pTheta, double* pRound, short count = 1);
GT_API GTN_SetAxisRotaryConfig(short core, short axis, TRotaryConfig* pConfig);
GT_API GTN_GetAxisRotaryConfig(short core, short axis, TRotaryConfig* pConfig);
GT_API GTN_ZeroAxisRotaryRound(short core, short axis, short count = 1);
GT_API GTN_LineAbsoluteEx(short core, short crd, double* pPos, short* pDir, TLineAbsolutePrm* pPrm);
GT_API GTN_SetAxisRotaryDirectionSelectMode(short core, short axis, short mode);
GT_API GTN_GetAxisRotaryDirectionSelectMode(short core, short axis, short* pMode);

typedef struct MoveAbsoluteProPrm
{
    double pos;
    double vel;
    double acc;
    double dec;
    short percent;
    short pad1[3];
    double velStart;
    double velEnd;
    short accStartPercent;
    short decEndPercent;
    short dir;
    short pad2;
    short reserve[10];
} TMoveAbsoluteProPrm;
GT_API GTN_MoveAbsolutePro(short core, short profile, TMoveAbsoluteProPrm* pPrm);

/*-----------------------------------------------------------*/
//原config.h：配置功能，包括主卡和模块                       */
/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
/* conifg of controller                                      */
/*-----------------------------------------------------------*/
#define RES_LIMIT                       (8)
#define RES_ALARM                       (8)
#define RES_HOME                        (8)
#define RES_GPI                         (16)
#define RES_ARRIVE                      (8)
#define RES_MPG                         (7)
#define RES_ENABLE                      (8)
#define RES_CLEAR                       (8)
#define RES_GPO                         (16)
#define RES_DAC                         (12)
#define RES_STEP                        (8)
#define RES_PULSE                       (8)
#define RES_ENCODER                     (11)
#define RES_LASER                       (2)
#define AXIS_MAX                        (8)
#define PROFILE_MAX                     (8)
#define CONTROL_MAX                     (8)
#define PRF_MAP_MAX                     (2)
#define ENC_MAP_MAX                     (2)

typedef struct DiConfig
{
    short active;
    short reverse;
    short filterTime;
} TDiConfig;

typedef struct CountConfig
{
    short active;
    short reverse;
    short filterType;

    short captureSource;
    short captureHomeSense;
    short captureIndexSense;
} TCountConfig;

typedef struct DoConfig
{
    short active;
    short axis;
    short axisItem;
    short reverse;
} TDoConfig;

typedef struct StepConfig
{
    short active;
    short axis;
    short mode;
    short parameter;
    short reverse;
} TStepConfig;

typedef struct DacConfig
{
    short active;
    short control;
    short reverse;
    short bias;
    short limit;
} TDacConfig;

typedef struct AdcConfig
{
    short active;
    short reverse;
    double a;
    double b;
    short filterMode;
} TAdcConfig;

typedef struct ControlConfig
{
    short active;
    short axis;
    short encoder1;
    short encoder2;
    long  errorLimit;
    short filterType[3];
    short encoderSmooth;
    short controlSmooth;
} TControlConfig;

typedef struct ControlConfigEx
{
    short refType;
    short refIndex;

    short feedbackType;
    short feedbackIndex;

    long  errorLimit;
    short feedbackSmooth;
    short controlSmooth;
} TControlConfigEx;

typedef struct ProfileConfig
{
    short  active;
    double decSmoothStop;
    double decAbruptStop;
} TProfileConfig;

typedef struct AxisConfig
{
    short active;
    short alarmType;
    short alarmIndex;
    short limitPositiveType;
    short limitPositiveIndex;
    short limitNegativeType;
    short limitNegativeIndex;
    short smoothStopType;
    short smoothStopIndex;
    short abruptStopType;
    short abruptStopIndex;
    long  prfMap;
    long  encMap;
    short prfMapAlpha[PRF_MAP_MAX];
    short prfMapBeta[PRF_MAP_MAX];
    short encMapAlpha[ENC_MAP_MAX];
    short encMapBeta[ENC_MAP_MAX];
} TAxisConfig;

typedef struct AxisConfigPro
{
	short active;
	short alarmType;
	short alarmIndex;
	short limitPositiveType;
	short limitPositiveIndex;
	short limitNegativeType;
	short limitNegativeIndex;
	short smoothStopType;
	short smoothStopIndex;
	short abruptStopType;
	short abruptStopIndex;
	short prfMap[PRF_MAP_MAX];
	short encMap[ENC_MAP_MAX];
	short prfMapAlpha[PRF_MAP_MAX];
	short prfMapBeta[PRF_MAP_MAX];
	short encMapAlpha[ENC_MAP_MAX];
	short encMapBeta[ENC_MAP_MAX];
} TAxisConfigPro;

#define PROFILE_MAX                     (8)

typedef struct McConfig
{
    TProfileConfig profile[PROFILE_MAX];
    TAxisConfig    axis[AXIS_MAX];
    TControlConfig control[CONTROL_MAX];
    TDacConfig     dac[RES_DAC];
    TStepConfig    step[RES_STEP];
    TCountConfig   encoder[RES_ENCODER];
    TCountConfig   pulse[RES_PULSE];
    TDoConfig      enable[RES_ENABLE];
    TDoConfig      clear[RES_CLEAR];
    TDoConfig      gpo[RES_GPO];
    TDiConfig      limitPositive[RES_LIMIT];
    TDiConfig      limitNegative[RES_LIMIT];
    TDiConfig      alarm[RES_ALARM];
    TDiConfig      home[RES_HOME];
    TDiConfig      gpi[RES_GPI];
    TDiConfig      arrive[RES_ARRIVE];
    TDiConfig      mpg[RES_MPG];
} TMcConfig;

GT_API GT_SaveConfig(char* pFile);
GT_API GT_SetDiConfig(short diType, short diIndex, TDiConfig* pDi);
GT_API GT_GetDiConfig(short diType, short diIndex, TDiConfig* pDi);
GT_API GT_SetDoConfig(short doType, short doIndex, TDoConfig* pDo);
GT_API GT_GetDoConfig(short doType, short doIndex, TDoConfig* pDo);
GT_API GT_SetStepConfig(short step, TStepConfig* pStep);
GT_API GT_GetStepConfig(short step, TStepConfig* pStep);
GT_API GT_SetDacConfig(short dac, TDacConfig* pDac);
GT_API GT_GetDacConfig(short dac, TDacConfig* pDac);
GT_API GT_SetAdcConfig(short adc, TAdcConfig* pAdc);
GT_API GT_GetAdcConfig(short adc, TAdcConfig* pAdc);
GT_API GT_SetCountConfig(short countType, short countIndex, TCountConfig* pCount);
GT_API GT_GetCountConfig(short countType, short countIndex, TCountConfig* pCount);
GT_API GT_SetControlConfig(short control, TControlConfig* pControl);
GT_API GT_GetControlConfig(short control, TControlConfig* pControl);
GT_API GT_SetControlConfigEx(short control, TControlConfigEx* pControl);
GT_API GT_GetControlConfigEx(short control, TControlConfigEx* pControl);
GT_API GT_SetProfileConfig(short profile, TProfileConfig* pProfile);
GT_API GT_GetProfileConfig(short profile, TProfileConfig* pProfile);
GT_API GT_SetAxisConfig(short axis, TAxisConfig* pAxis);
GT_API GT_GetAxisConfig(short axis, TAxisConfig* pAxis);
GT_API GT_ProfileScale(short axis, short alpha, short beta);
GT_API GT_EncScale(short axis, short alpha, short beta);

GT_API GT_EncSns(unsigned short sense);
GT_API GT_LmtSns(unsigned short sense);
GT_API GT_GpiSns(unsigned short sense);
GT_API GT_SetAdcFilter(short adc, short filterTime);


GT_API GT_GetConfigTable(short type, short* pCount);
GT_API GT_GetConfigTableAll();

GT_API GT_SetMcConfig(TMcConfig* pMc);
GT_API GT_GetMcConfig(TMcConfig* pMc);

GT_API GT_SetMcConfigToFile(TMcConfig* pMc, char* pFile);
GT_API GT_GetMcConfigFromFile(TMcConfig* pMc, char* pFile);
GT_API GTN_SetMcConfig(short core, TMcConfig* pMc);
GT_API GTN_GetMcConfig(short core, TMcConfig* pMc);
GT_API GTN_SetMcConfigToFile(short core, TMcConfig* pMc, char* pFile);
GT_API GTN_GetMcConfigFromFile(short core, TMcConfig* pMc, char* pFile);
GT_API GTN_SaveConfig(short core, char* pFile);
GT_API GTN_SetDiConfig(short core, short diType, short diIndex, TDiConfig* pDi);
GT_API GTN_GetDiConfig(short core, short diType, short diIndex, TDiConfig* pDi);
GT_API GTN_SetDoConfig(short core, short doType, short doIndex, TDoConfig* pDo);
GT_API GTN_GetDoConfig(short core, short doType, short doIndex, TDoConfig* pDo);
GT_API GTN_SetStepConfig(short core, short step, TStepConfig* pStep);
GT_API GTN_GetStepConfig(short core, short step, TStepConfig* pStep);
GT_API GTN_SetDacConfig(short core, short dac, TDacConfig* pDac);
GT_API GTN_GetDacConfig(short core, short dac, TDacConfig* pDac);
GT_API GTN_SetAdcConfig(short core, short adc, TAdcConfig* pAdc);
GT_API GTN_GetAdcConfig(short core, short adc, TAdcConfig* pAdc);
GT_API GTN_SetCountConfig(short core, short countType, short countIndex, TCountConfig* pCount);
GT_API GTN_GetCountConfig(short core, short countType, short countIndex, TCountConfig* pCount);
GT_API GTN_SetControlConfig(short core, short control, TControlConfig* pControl);
GT_API GTN_GetControlConfig(short core, short control, TControlConfig* pControl);
GT_API GTN_SetControlConfigEx(short core, short control, TControlConfigEx* pControl);
GT_API GTN_GetControlConfigEx(short core, short control, TControlConfigEx* pControl);
GT_API GTN_SetProfileConfig(short core, short profile, TProfileConfig* pProfile);
GT_API GTN_GetProfileConfig(short core, short profile, TProfileConfig* pProfile);
GT_API GTN_SetAxisConfig(short core, short axis, TAxisConfig* pAxis);
GT_API GTN_GetAxisConfig(short core, short axis, TAxisConfig* pAxis);
GT_API GTN_SetAxisConfigPro(short core, short axis, TAxisConfigPro* pAxisConfigPro);     // 核内轴数超过32轴使用该指令
GT_API GTN_GetAxisConfigPro(short core, short axis, TAxisConfigPro* pAxisConfigPro);     // 核内轴数超过32轴使用该指令
GT_API GTN_ProfileScale(short core, short axis, short alpha, short beta);
GT_API GTN_EncScale(short core, short axis, short alpha, short beta);

/*-----------------------------------------------------------*/
/* Config of Laser and Scan                                  */
/*-----------------------------------------------------------*/
typedef struct ScanCommandMotion
{
    long segmentNumber;
    short x;
    short y;
    long deltaX;
    long deltaY;
    long vel;
    long acc;
}TScanCommandMotion;

typedef struct ScanCommandMotionDelay
{
    long delay;
}TScanCommandMotionDelay;

typedef struct ScanCommandDo
{
    short doType;
    short doMask;
    short doValue;
}TScanCommandDo;

typedef struct ScanCommandDoDelay
{
    long delay;
}TScanCommandDoDelay;

typedef struct ScanCommandLaser
{
    short mask;
    short value;
}TScanCommandLaser;

typedef struct ScanCommandLaserDelay
{
    long laserOnDelay;
    long laserOffDelay;
}TScanCommandLaserDelay;

typedef struct ScanCommandLaserPower
{
    long power;
}TScanCommandLaserPower;

typedef struct ScanCommandLaserFrequency
{
    long frequency;
}TScanCommandLaserFrequency;

typedef struct ScanCommandLaserPulseWidth
{
    long pulseWidth;
}TScanCommandLaserPulseWidth;

typedef struct ScanCommandDa
{
    short daIndex;
    short daValue;
}TScanCommandDa;

typedef struct ScanMap
{
    short module;
    short fifo;
} TScanMap;

GT_API GTN_SetScanMap(short core, short scan, TScanMap* pMap);
GT_API GTN_GetScanMap(short core, short scan, TScanMap* pMap);
GT_API GTN_ClearScanMap(short core);
GT_API GTN_UpdateScanMap(short core);

/*-----------------------------------------------------------*/
//原ringnet.h：和等环网网络相关功能的指令                    */
/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
/* Ringnet                                                  */
/*-----------------------------------------------------------*/
#define RTN_SUCCESS				                    (0)
#define RTN_MALLOC_FAIL						            (-100) /* malloc memory fail */
#define RTN_FREE_FAIL							       			(-101) /* free memory or delete the object fail */
#define RTN_NULL_POINT						            (-102) /* the param point input is null */
#define RTN_ERR_ORDER						            	(-103) /* call the function order is wrong, some msg isn't validable */
#define RTN_PCI_NULL							        		(-104) /* the pci address is empty, can't access the pci device*/
#define RTN_PARAM_OVERFLOW			              (-105) /* the param input is too larget*/
#define RTN_LINK_FAIL							       			(-106) /* the two ports both link fail*/
#define RTN_IMPOSSIBLE_ERR				            (-107) /* it means the system or same function work wrong*/
#define RTN_TOPOLOGY_CONFLICT			            (-108) /* the id conflict*/
#define RTN_TOPOLOGY_ABNORMAL		              (-109) /* scan the net abnormal*/
#define RTN_STATION_ALONE				              (-110) /* the device no id, it means the device id is 0xF0 */
#define RTN_WAIT_OBJECT_OVERTIME	            (-111) /* multi thread wait for object overtime */
#define RTN_ACCESS_OVERFLOW			              (-112) /* data[i];  i is larger than the define */
#define RTN_NO_STATION						            (-113) /* the station accessed not existent */
#define RTN_OBJECT_UNCREATED			            (-114) /* the object not created yet*/
#define RTN_PARAM_ERR						            	(-115) /* the param input is wrong*/
#define RTN_PDU_CFG_ERR                       (-116) /*Pdu DMA Cfg Err*/
#define RTN_PCI_FPGA_ERR					            (-117) /*PCI op err or FPGA op err*/
#define RTN_CHECK_RW_ERR					            (-118) /*data write to reg, then rd out, and check err */
#define RTN_REMOTE_UNEABLE				            (-119) /*the device which will be ctrl by net can't be ctrl by net function*/

#define RTN_NET_REQ_DATA_NUM_ZERO		          (-120) /*mail op or pdu op req data num can't be 0*/
#define RTN_WAIT_NET_OBJECT_OVERTIME	        (-121) /* net op multi thread wait for object overtime */
#define RTN_WAIT_NET_RESP_OVERTIME		        (-122) /* Can't wait for resp */
#define RTN_WAIT_NET_RESP_ERR				          (-123) /*wait mailbox op err*/
#define RTN_INITIAL_ERR								    		(-124) /*initial the device err*/
#define RTN_PC_NO_READY							        	(-125) /*find the station'pc isn't work*/
#define RTN_STATION_NO_EXIST					        (-126)
#define RTN_MASTER_FUNCTION					          (-127) /* this funciton only used by master */

#define RTN_NOT_ALL_RETURN							    	(-128) /* the GT_RN_PtProcessData funciton fail return */
#define RTN_NUM_NOT_EQUAL							   			(-129) /* the station number of RingNet do not equal  the station number of CFG */

#define RTN_CHECK_STATION_ONLINE_NUM_ERR		  (-130) /*Check no slave*/
#define RTN_FILE_ERR_OPEN											(-131) /*open file error*/
#define RTN_FILE_ERR_FORMAT							    	(-132) /*parse file error*/
#define RTN_FILE_ERR_MISSMATCH					      (-133) /*file info is not match with the actual ones*/
#define RTN_DMALIST_ERR_MISSMATCH			        (-134) /*can't find the slave*/

#define RTN_REQUSET_MAIL_BUS_OVERTIME		      (-150) /*Requset Mail Bus Err*/
#define RTN_INSTRCTION_ERR							    	(-151) /*instrctions err*/
#define RTN_MAIL_RESP_REQ_ERR						    	(-152) /*RN_MailRespReq  err*/
#define RTN_CTRL_SRC_ERR											(-153) /* the controlled source  is error */
#define RTN_PACKET_ERR												(-154) /*packet is error*/
#define RTN_STATION_ID_ERR							    	(-155) /*the device id is not in the right rang*/
#define RTN_WAIT_NET_PDU_RESP_OVERTIME	      (-156) /*net pdu op wait overtime*/
#define RTN_ETHCAT_ENC_POS_ERR					      (-157) /**/

#define RTN_IDLINK_PACKET_ERR			            (-200) /*ilink master  decode err! packet_length is not match*/
#define RTN_IDLINK_PACKET_END_ERR		          (-201) /* the ending of ilink packet is not 0xFF*/
#define RTN_IDLINK_TYPER_ERR					        (-202) /* the type of ilink module is error*/
#define RTN_IDLINK_LOST_CNT 					        (-203) /* the ilink module has lost connection*/
#define RTN_IDLINK_CTRL_SRC_ERR			          (-204) /* the controlled source of ilink module is error */
#define RTN_IDLINK_UPDATA_ERR				          (-205) /* the ilink module updata error*/
#define RTN_IDLINK_NUM_ERR					         	(-206) /* the ilink num larger the IDLINK_MAX_NUM(30) */
#define RTN_IDLINK_NUM_ZERO					          (-207) /* the ilink num is zero */

#define RTN_NO_PACKET							        		(301)  /* no valid packet */
#define RTN_RX_ERR_PDU_PACKET				          (-302) /* ERR PDU PACKET */
#define RTN_STATE_MECHINE_ERR				          (-303)
#define RTN_PCI_DSP_UN_FINISH				          (304)
#define RTN_SEND_ALL_FAIL						        	(-305)
#define RTN_STATION_CLOSE					            (310)
#define RTN_STATION_RESP_FAIL				          (311)

#define RTN_UPDATA_MODAL_ERR			            (-330) /* update the modal in normal way fail*/

#define RTN_NO_MAIL_DATA						        	(340) /*There is no mail data*/
#define RTN_NO_PDU_DATA						            (341) /*There is no pdu data*/

#define RTN_FILE_PARAM_NUM_ERR					      (-500)
#define RTN_FILE_PARAM_LEN_ERR					      (-501)
#define RTN_FILE_MALLOC_FAIL									(-502)
#define RTN_FILE_FREE_FAIL										(-503)
#define RTN_FILE_PARAM_ERR							   		(-504)
#define RTN_FILE_NOT_EXSITS						    		(505)
#define RTN_FILE_CREATE_FAIL									(510)
#define RTN_FILE_DELETE_FAIL									(511)
#define RTN_FIFE_CRC_CHECK_ERR								(-512)
#define RTN_FIFE_FUNCTION_ID_RETURN_ERR				(-600)
#define RTN_DLL_WINCE													(-800)
#define RTN_DLL_WIN32													(-801)
#define RTN_XML_STATION_ERR										(-900)//dma config file confilit with slave type


GT_API GT_RN_GetEncPos(short encoder, double* pValue, short count, unsigned long* pClock);
GT_API GT_RN_GetAxisError(short axis, double* pValue, short count, unsigned long* pClock);
GT_API GT_RN_GetPrfMode(short axis, long* pValue, short count, unsigned long* pClock);
GT_API GT_RN_GetAuEncPos(short encoder, double* pValue, short count, unsigned long* pClock);
GT_API GT_RN_GetCaptureStatus(short encoder, short* pStatus, long* pValue, short count, unsigned long* pClock);
GT_API GT_RN_GetSts(short axis, long* pSts, short count, unsigned long* pClock);
GT_API GT_RN_GetPowerSts(long* pValue);
GT_API GT_RN_GetEcatAxisACTArray(short axis, short* pCur, short* pTorque, short count);
GT_API GT_RN_PtSpaceArray(short profile, short* pSpace, short fifo, short count);
GT_API GT_RN_GetDoEx(short doType, long* pValue);
GT_API GT_RN_GetDiEx(short diType, long* pValue);
GT_API GT_RN_GetDo(short doType, long* pValue);
GT_API GT_RN_GetDi(short diType, long* pValue);
GT_API GTN_LoadRingNetConfig(short core, char* pFile);

GT_API GTN_CheckRingNetStructure(short core, char* pFile, unsigned short* pStatus);


/*-----------------------------------------------------------*/
/* 读取SPORT包数据                                           */
/*-----------------------------------------------------------*/
#define SPORT_COUNT                     (256)

typedef struct TerminalData
{
    unsigned long terminalTxBuf[SPORT_COUNT];
    unsigned long terminalRxBuf[SPORT_COUNT];
}TTerminalData;

GT_API GTN_ReadTerminalData(short core, TTerminalData* pTerminalData);

/*-----------------------------------------------------------*/
/* Config of module                                          */
/*-----------------------------------------------------------*/
#define TERMINAL_OPERATION_NONE             (0)
#define TERMINAL_OPERATION_SKIP             (1)
#define TERMINAL_OPERATION_CLEAR            (2)
#define TERMINAL_OPERATION_RESET_MODULE     (3)

#define TERMINAL_OPERATION_PROGRAM          (11)

typedef struct RingNetCrcStatus
{
    unsigned long portACrcOkCnt;
    unsigned short portACrcErrorCnt;
    unsigned long portBCrcOkCnt;
    unsigned short portBCrcErrorCnt;
    unsigned long reserve;//目前用于读取FLASH总数据长度
} TRingNetCrcStatus;

typedef struct TerminalError
{
    unsigned short errorCountReceive;
    unsigned short errorCountPackageDown;
    unsigned short errorCountPackageUp;
    unsigned short reserve[13];
} TTerminalError;

typedef struct TerminalMap
{
    short moduleDataType;
    short moduleDataIndex;
    short dataIndex;
    short dataCount;
} TTerminalMap;

GT_API GT_SetMailbox(short core, short station, unsigned short byteAddress, unsigned short* pData, unsigned short wordCount, unsigned short dataMode, unsigned short desId, unsigned short type);
GT_API GT_GetMailbox(short core, short station, unsigned short byteAddress, unsigned short* pData, unsigned short wordCount, unsigned short dataMode, unsigned short desId, unsigned short type);

GT_API GTN_LoadTerminalConfig(short core, char* pFile);
GT_API GTN_SaveTerminalConfig(short core, char* pFile);
GT_API GTN_TerminalOn(short core, short index);
GT_API GTN_TerminalSynch(short core, short index);
GT_API GTN_TerminalSynchEx(short core, short index, short synchEnable, short synchSource);
GT_API GTN_GetRingNetCrcStatus(short core, short index, TRingNetCrcStatus* pRingNetCrcStatus);
GT_API GTN_GetTerminalError(short core, short index, TTerminalError* pTerminalError);

GT_API GTN_GetTerminalPhyId(short core, short count, short* pPhyId);
GT_API GTN_GetTerminalLinkStatus(short core, short count, short* ringNetType, short* pLinkStatus);
GT_API GTN_SetTerminalMap(short core, short dataType, short moduleIndex, TTerminalMap* pMap);
GT_API GTN_GetTerminalMap(short core, short dataType, short moduleIndex, TTerminalMap* pMap);
GT_API GTN_ClearTerminalMap(short core, short dataType);
GT_API GTN_SetTerminalMode(short core, short station, unsigned short mode);
GT_API GTN_GetTerminalMode(short core, short station, unsigned short* pMode);
GT_API GTN_SetTerminalTest(short core, short station, short index, unsigned short value);
GT_API GTN_GetTerminalTest(short core, short station, short index, unsigned short* pValue);
GT_API GTN_SetTerminalOperation(short core, short operation);
GT_API GTN_GetTerminalOperation(short core, short* pOperation);



#define TERMINAL_LOAD_MODE_NONE             (0)
#define TERMINAL_LOAD_MODE_BOOT             (2)

typedef struct TerminalStatus
{
    unsigned short type;
    short id;
    long status;
    unsigned long synchCount;
    unsigned long ringNetType;
    unsigned long portStatus;
    unsigned long sportDropCount;
    unsigned long reserve[7];
} TTerminalStatus;

GT_API GTN_TerminalInit(short core, short detect = 1);
GT_API GTN_GetTerminalVersion(short core, short index, TVersion* pTerminalVersion);
GT_API GTN_SetTerminalPermit(short core, short index, short dataType, unsigned short permit);
GT_API GTN_GetTerminalPermit(short core, short index, short dataType, unsigned short* pPermit);
GT_API GTN_SetTerminalPermitEx(short core, short station, short dataType, short* permit, short index, short count);
GT_API GTN_GetTerminalPermitEx(short core, short station, short dataType, short* pPermit, short index, short count);

GT_API GTN_SetTerminalSafeMode(short core, short index, short safeMode);
GT_API GTN_GetTerminalSafeMode(short core, short index, short* pSafeMode);
GT_API GTN_ClearTerminalSafeMode(short core, short index);
GT_API GTN_GetTerminalStatus(short core, short index, TTerminalStatus* pTerminalStatus);
GT_API GTN_GetTerminalType(short core, short count, unsigned short* pType, short* pTypeConnect = NULL);
GT_API GTN_SetTerminalType(short core, short count, short* pType);
GT_API GTN_GetTerminalMapCount(short core, short dataType, short moduleIndex, short* pMapCount);
GT_API GTN_GetTerminalMapData(short core, short dataType, short moduleIndex, short mapIndex, TTerminalMap* pMap);

GT_API GTN_FindStation(short core, short station, unsigned long time);

GT_API GTN_ProgramTerminalConfig(short core, short loadMode);
GT_API GTN_GetTerminalConfigLoadMode(short core, short* pLoadMode);




GT_API GTN_SetMailbox(short core, short station, unsigned short byteAddress, unsigned short* pData, unsigned short wordCount, unsigned short dataMode, unsigned short desId, unsigned short type);
GT_API GTN_GetMailbox(short core, short station, unsigned short byteAddress, unsigned short* pData, unsigned short wordCount, unsigned short dataMode, unsigned short desId, unsigned short type);

/*-----------------------------------------------------------*/
/* UUID					                                            */
/*-----------------------------------------------------------*/
GT_API GTN_SetUuid(short core, char* pUuid, short count);
GT_API GTN_GetUuid(short core, char* pCode, short count);

/*-----------------------------------------------------------*/
/* FLASH					                                            */
/*-----------------------------------------------------------*/
GT_API GTN_SetFlashValue(short core, unsigned long address, short count, short* pData);
GT_API GTN_GetFlashValue(short core, unsigned long address, short count, short* pData);

/*-----------------------------------------------------------*/
/* 网络恢复指令                                              */
/*-----------------------------------------------------------*/
GT_API GTN_RN_RingNetRecover(short core, short station);
/*-----------------------------------------------------------*/
/* GSHD最大最小力矩设置                                      */
/*-----------------------------------------------------------*/
typedef struct TorqueLimit
{
    unsigned short torqueMax;
    unsigned short torquePostive;
    unsigned short torqueNegitive;
    short reserve1;
    double reserve2[4];
}TTorqueLimit;
GT_API GTN_RN_SetMaxTorqueLimit(short core, unsigned short station, short axis, unsigned short torqueMax, unsigned short torquePostive, unsigned short torqueNegitive);
GT_API GTN_RN_GetMaxTorqueLimit(short core, unsigned short station, short axis, unsigned short* torqueMax, unsigned short* torquePostive, unsigned short* torqueNegitive);
GT_API GTN_RN_SetTorqueLimit(short core, short axis, TTorqueLimit* pTorqueLimit);
GT_API GTN_RN_GetTorqueLimit(short core, short axis, TTorqueLimit* pTorqueLimit);

GT_API GTN_SetPrfTorque(short core, short axis, short prfTorque);
GT_API GTN_GetAtlTorque(short core, short axis, short* atlTorque);
GT_API GTN_SetPrfTorqueArray(short core, short axis, short* prfTorque, short count);
GT_API GTN_GetAtlTorqueArray(short core, short axis, short* pAtlTorque, short count);
GT_API GTN_GetSafeTorqueOffStatus(short core, long* pSts);

typedef struct TorqueTriggerPrm
{
    short	enable;
    short	latchType;
    short	latchIndex;
    short	probeType;
    short	probeIndex;
    short	windowOnly;
    long	firstPosition;
    long	lastPosition;
    long	continueTime;
    double	currentValue;
    double	pad2[3];
}TTorqueTriggerPrm;
GT_API GTN_SetTorqueTriggerPrm(short core, short index, TTorqueTriggerPrm* pPrm);
GT_API GTN_GetTorqueTriggerPrm(short core, short index, TTorqueTriggerPrm* pPrm);





/*-----------------------------------------------------------*/
/* GSHD闭环参数设置                                          */
/*-----------------------------------------------------------*/
typedef struct ServoPosLoopPidMode0
{
    double value;
    double reverse[4];
}TServoPosLoopPidMode0;

typedef union ServoPosLoopPidUnion
{
    TServoPosLoopPidMode0 servoPosLoopPidMode0;
}TServoPosLoopPidUnion;

typedef struct ServoPosLoopPid
{
    short mode;
    TServoPosLoopPidUnion servoPosLoopPidPrm;
}TServoPosLoopPid;

GT_API GTN_RN_SetServoPosLoopPid(short core, short axis, TServoPosLoopPid* pServoPosLoopPid);
GT_API GTN_RN_GetServoPosLoopPid(short core, short axis, TServoPosLoopPid* pServoPosLoopPid);

typedef struct ServoSpdLoopPidMode0
{
    double value;
    double reverse[4];
}TServoSpdLoopPidMode0;

typedef union ServoSpdLoopPidUnion
{
    TServoSpdLoopPidMode0 servoSpdLoopPidMode0;
}TServoSpdLoopPidUnion;

typedef struct ServoSpdLoopPid
{
    short mode;
    short reverse[3];
    TServoSpdLoopPidUnion servoSpdLoopPidPrm;
}TServoSpdLoopPid;

GT_API GTN_RN_SetServoSpdLoopPid(short core, short axis, TServoSpdLoopPid* pServoSpdLoopPid);
GT_API GTN_RN_GetServoSpdLoopPid(short core, short axis, TServoSpdLoopPid* pServoSpdLoopPid);

#define SERBVO_POS_LOOP                   (0)		// 驱动器位置环
#define SERBVO_SPD_LOOP                   (1)   // 驱动器速度环

GT_API GTN_RN_GetServoPid(short core, short axis, short loop, short mode, void* pServoPid);
GT_API GTN_RN_SetServoPid(short core, short axis, short loop, short mode, void* pServoPid);
GT_API GTN_RN_SetServoPidRatio(short core, short axis, double ratio);
GT_API GTN_RN_GetServoPidRatio(short core, short axis, double* pRatio);


/*-----------------------------------------------------------*/
/* 安全模式设置                                              */
/*-----------------------------------------------------------*/
GT_API GTN_RN_ClearStationSafeModeStatus(short cardIndex, short stationPhyId);
GT_API GTN_RN_IlinkSetSafeModeControl(short cardIndex, short stationPhyId, short modulePhyId, short enable, short clearMode);
GT_API GTN_RN_IlinkClearSafeModeStatus(short cardIndex, short stationPhyId, short modulePhyId);
GT_API GTN_RN_IlinkSetSafeModeOut(short cardIndex, short stationPhyId, short modulePhyId, short type, short index, short* pEnable, double* pValue, short count);
GT_API GTN_RN_GetStationOfflineCount(short cardIndex, long* pStationPhyId, short count, short* pStationOfflineCount);

/*-----------------------------------------------------------*/
//原LookAheadEx.h：和前瞻相关功能的指令                      */
/*-----------------------------------------------------------*/
#define LA_AXIS_NUM           (8)
#define LA_WORK_AXIS_NUM      (6)
#define LA_MACHINE_AXIS_NUM   (5)

//轴的参数信息（各轴最大速度，各轴最大加速度，各轴最大速度变化量）是否限制速度模式
#define AXIS_LIMIT_NONE       (0)       //轴无限制
#define AXIS_LIMIT_MAX_VEL    (1)       //轴最大速度限制
#define AXIS_LIMIT_MAX_ACC    (2)       //轴最大加速度限制
#define AXIS_LIMIT_MAX_DV     (4)       //轴最大速度跳变量限制

#define KIN_MSG_BUFFER_SIZE   32

//速度规划模式
typedef enum VelMode
{
    T_CURVE = 0,
    S_CURVE,
    S_CURVE_NEW,                  //根据加加速度、最大加速度进行S曲线速度前瞻，2015.11.16
    S_CURVE_SMOOTH,

    VEL_MODE_MAX = 0x10000,         //确保长度为4Byte
}EVelMode;

//工件坐标系下轨迹是否限制速度模式
typedef enum WorkLimitMode
{
    WORK_LIMIT_INVALID = 0,       //工件坐标系信息不限制
    WORK_LIMIT_VALID,           //工件坐标系限制生效

    WORK_LIMIT_MODE_MAX = 0x10000,//确保长度为4Byte
}EWorkLimitMode;

//设置的速度定义规则
typedef enum VelSettingDef
{
    NORMAL_DEF_VEL = 0,             //输入为轴坐标系所有轴的合成速度
    NUM_DEF_VEL,                  //以NUM系统的规则定义
    CUT_DEF_VEL,                  //速度为切削速度

    VEL_SETTING_DEF_MAX = 0x10000,  //确保长度为4Byte
}EVelSettingDef;

//设置的加速度定义规则
typedef enum AccSettingDef
{
    NORMAL_DEF_ACC = 0,             //输入即输出
    LONG_AXIS_ACC,                //长轴最大速度

    ACC_SETTING_DEF_MAX = 0x10000,  //确保长度为4Byte
}EAccSettingDef;

//机床类型
typedef enum MachineMode
{
    NORMAL_THREE_AXIS = 0,      //标准三轴机床模式
    MULTI_AXES,               //多轴联动模式
    FIVE_AXIS,                //五轴机床模式,轴坐标系为主，工件坐标系为辅
    FIVE_AXIS_WORK,           //五轴机床模式，工件坐标系为主，轴坐标系为辅
    ROBOT,                    //机器人模式，轴坐标系为主，工件坐标系为辅，2014.12.3
    ROBOT_WORK,

    MACHINE_MODE_MAX = 0x10000, //确保长度为4Byte
}EMachineMode;

//前瞻参数结构体
typedef struct LookAheadParameter
{
    int lookAheadNum;					//前瞻段数
    double time;						//时间常数
    double radiusRatio;					//曲率限制调节参数
    double vMax[LA_AXIS_NUM];			//各轴的最大速度
    double aMax[LA_AXIS_NUM];			//各轴的最大加速度
    double DVMax[LA_AXIS_NUM];			//各轴的最大速度变化量（在时间常数内）
    double scale[LA_AXIS_NUM];			//各轴的脉冲当量
    short axisRelation[LA_AXIS_NUM];	//输入坐标和内部坐标的对应关系
    char machineCfgFileName[128];		//机床配置文件名
}TLookAheadParameter;

//////////////////////////////////////
typedef struct RC_KIN_CONFIG
{
    short RobotType;
    short reserved1;

    short KinParUse[18];
    double KinPar[18];
    short KinLimitUse[12];
    double KinLimitMin[12];
    double KinLimitMax[12];
    double KinLimitMinShift[12];
    double KinLimitMaxShift[12];

    short AxisUse[8];
    char AxisPosSignSwitch[8];
    double AxisPosOffset[8];

    short CartUnitUse[6];
    char CartPosKCSSignSwitch[6];
    short reserved2[3];
    double CartPosKCSOffset[6];
}RC_KIN_CONFIG;

typedef struct RC_ERROR_INTERFACE
{
    char Error;
    short ErrorID;
    char Message[129];
}RC_ERROR_INTERFACE;

typedef struct RC_MSG_BUFFER_ELEMENT
{
    short ErrorID;
    char Message[129];
    char LogTime[32];
    long InternalID;
}RC_MSG_BUFFER_ELEMENT;

typedef struct RC_MSG_BUFFER
{
    short LastMsgIndex;
    RC_MSG_BUFFER_ELEMENT MsgElement[KIN_MSG_BUFFER_SIZE];
    long LastMsgID;
}RC_MSG_BUFFER;

//正逆解方向
typedef enum TransDir
{
    FORWARD_TRANS = 0,            //正解
    INVERSE_TRANS,              //逆解

    TRANS_DIR_MAX = 0x10000,	// 确保长度为4Byte
}ETransDir;

//旋转轴范围设置
typedef struct RotationAxisRange
{
    int primaryAxisRangeOn;              //第一旋转轴限定范围是否生效，0：不生效，1：生效
    int slaveAxisRangeOn;                //第二旋转轴限定范围是否生效，0：不生效，1：生效
    double maxPrimaryAngle;              //第一旋转轴最大值
    double minPrimaryAngle;              //第一旋转轴最小值
    double maxSlaveAngle;                //第二旋转轴最大值
    double minSlaveAgnle;                //第二旋转轴最小值
}TRotationAxisRange;

//选解参数
typedef enum GroupSelect
{
    Continuous = 0,
    Group_1,
    Group_2,
}EGroupSelect;

typedef enum OptimizeState
{
    OPT_OFF = 0,
    OPT_ON,
}OptimizeState;

typedef enum OptimizeMethod
{
    NO_OPT = 0,
    OPT_BLENDING,
    OPT_CIRCLEFITTING,
    OPT_CUBICSPLINE,
    OPT_BSPLINE,
}OptimizeMethod;

typedef enum ErrorID
{
    INIT_ERROR = 1,		                    //没有进行参数初始化
    PASSWORD_ERROR,		                    //密码错误，请在固高运动控制平台上运行
    INDATA_ERROR,		                    //输入数据错误（检查圆弧数据是否正确）
    PRE_PROCESS_ERROR,
    TOOL_RADIUS_COMPENSATE_ERROR_INOUT,		//刀具半径补偿错误：进入/结束刀补处不能是圆弧
    TOOL_RADIUS_COMPENSATE_ERROR_NOCROSS,	//刀具半径补偿错误：数据不合理，无法计算交点
    USERDATA_ERROR,
}ErrorID;

//轨迹优化参数结构体
typedef struct OptimizeParamUser
{
    OptimizeState usePathOptimize;	//是否使用路径优化：OPT_OFF:不使用	OPT_ON:使用

    float tolerance;				//公差(suggest: rough:0.1, pre-finish:0.05, finish:0.01)

    OptimizeMethod optimizeMethod;	//选择曲线优化方式

    OptimizeState keepLargeArc;		//是否保留大圆弧：OPT_OFF：不保留， OPT_ON：保留

    float blendingMinError;			//blending的最小设定误差

    float blendingMaxAngle;			//blending的最大角度限制（即当线段向量角度大于该角度时，不做blending，单位：度）

}TOptimizeParamUser;

typedef struct ErrorInfo
{
    ErrorID errorID;		//错误号(INIT_ERROR:未初始化参数；PRE_PROCESS_ERROR:预处理模块错误；
    //TOOL_RADIUS_COMPENSATE_ERROR:刀具半径补偿错误；)
    long errorRowNum;		//错误行号
}TErrorInfo;

typedef struct PreStartPos
{
    double Pos[LA_AXIS_NUM];
}TPreStartPos;

typedef enum MachineType
{
    //W:工件侧，T：刀具侧，L：45度
    //双转台
    MT_RW_C_ON_B=0,//B 为第一旋转轴，C 为第二旋转轴
    MT_RW_B_ON_A,	//A 为第一旋转轴，B 为第二旋转轴
    MT_RW_A_ON_B,	//B 为第一旋转轴，A 为第二旋转轴
    MT_RW_C_ON_A,	//A 为第一旋转轴，C 为第二旋转轴
    //双摆头
    MT_DT_B_ON_A,	//A 为第一旋转轴，B 为第二旋转轴
    MT_DT_A_ON_B,	//B 为第一旋转轴，A 为第二旋转轴
    MT_DT_A_ON_C,	//C 为第一旋转轴，A 为第二旋转轴
    MT_DT_B_ON_C,	//C 为第一旋转轴，B 为第二旋转轴
    //转台摆头，刀具侧为第一旋转轴，工件侧为第二旋转轴
    MT_T_A_W_B,	//A 为第一旋转轴，B 为第二旋转轴
    MT_T_B_W_A,	//B 为第一旋转轴，A 为第二旋转轴
    MT_T_A_W_C,	//A 为第一旋转轴，C 为第二旋转轴
    MT_T_B_W_C,	//B 为第一旋转轴，C 为第二旋转轴
}EMachineType;

typedef struct MachCfgInfo
{
    EMachineType machineType;            //机床类型
    short reserve1[2];                   //保留参数
    double primaryAxisPoint[3];          //第一旋转轴中心在MCS的坐标
    double slaveAxisPoint[3];            //第二旋转轴中心在MCS的坐标
    double toolLocationPoint[3];         //刀具坐标系中心在MCS的坐标
    short dirMode;                       //方向描述模式
    short reserve2[2];                   //保留参数
    short dir[5];                        //各轴方向
    double axisVector[5][3];             //各轴轴线方向
}TMachCfgInfo;


GT_API GT_SetupLookAheadCrd(short crd, EMachineMode machineMode);
GT_API GT_SetVelDefineModeLa(short crd, EVelSettingDef velDefMode);
GT_API GT_SetAccDefineModeLa(short crd, EAccSettingDef accDefMode);
GT_API GT_SetAxisLimitModeLa(short crd, int* pAxisLimitMode);
GT_API GT_SetWorkLimitModeLa(short crd, EWorkLimitMode workLimitMode);
GT_API GT_SetAxisFollowModeLa(short crd, int* pFollowMode);
GT_API GT_SetAxisVelValidModeLa(short crd, int velValidAxis);
GT_API GT_SetArcAllowErrorLa(short crd, double error);
GT_API GT_SetMinEvenVelTimeLa(short crd, double evenTime);
GT_API GT_SetMinDccAngleLa(short crd, double dccAngle);
GT_API GT_SetProfilePeriod(short crd, double profilePeriod);
GT_API GT_SetFilterTime(short crd, long filtNum);
GT_API GT_SetPrecisionControl(short crd, short mode, double error);
GT_API GT_SetVelSmoothModeLa(short crd, short smoothMode);
GT_API GT_SetVelModeLa(short crd, EVelMode velMode);
GT_API GT_InitLookAheadEx(short crd, TLookAheadParameter* pLookAheadPara, short fifo, short motionMode = 0, TPreStartPos* pPreStartPos = NULL);
GT_API GT_PrintLACmdLa(short crd, int printFlag, int clearFile);

GT_API GT_GetLookAheadSegCountEx(short crd, long* pSegCount, short fifo = 0);
GT_API GT_GetMotionTimeEx(short crd, double* pTime, short fifo = 0);
GT_API GT_SetUserSegNumEx(short crd, long segNum, short fifo = 0);
GT_API GT_CrdDataEx(short crd, TCrdData* pCrdData, short fifo = 0);

GT_API GT_LnXYEx(short crd, double x, double y, double synVel, double synAcc, long segNum, short override2, short fifo = 0);
GT_API GT_LnXYG0Ex(short crd, double x, double y, double synVel, double synAcc, long segNum, short override2, short fifo = 0);
GT_API GT_LnXYZEx(short crd, double x, double y, double z, double synVel, double synAcc, long segNum, short override2, short fifo = 0);
GT_API GT_LnXYZG0Ex(short crd, double x, double y, double z, double synVel, double synAcc, long segNum, short override2, short fifo = 0);
GT_API GT_LnXYZAEx(short crd, double x, double y, double z, double a, double synVel, double synAcc, long segNum, short override2, short fifo = 0);
GT_API GT_LnXYZAG0Ex(short crd, double x, double y, double z, double a, double synVel, double synAcc, long segNum, short override2, short fifo = 0);
GT_API GT_LnXYZACEx(short crd, double* pPos, short posMask, double synVel, double synAcc, long segNum, short override2, short fifo = 0);
GT_API GT_LnXYZACG0Ex(short crd, double* pPos, short posMask, double synVel, double synAcc, long segNum, short override2, short fifo = 0);
GT_API GT_LnXYZACUVWEx(short crd, double* pPos, short posMask, double synVel, double synAcc, long segNum, short override2, short fifo = 0);
GT_API GT_LnXYZACUVWG0Ex(short crd, double* pPos, short posMask, double synVel, double synAcc, long segNum, short override2, short fifo = 0);

GT_API GT_ArcXYREx(short crd, double x, double y, double radius, short circleDir, double synVel, double synAcc, long segNum, short override2, short fifo = 0);
GT_API GT_ArcYZREx(short crd, double y, double z, double radius, short circleDir, double synVel, double synAcc, long segNum, short override2, short fifo = 0);
GT_API GT_ArcZXREx(short crd, double z, double x, double radius, short circleDir, double synVel, double synAcc, long segNum, short override2, short fifo = 0);
GT_API GT_ArcXYCEx(short crd, double x, double y, double xCenter, double yCenter, short circleDir, double synVel, double synAcc, long segNum, short override2, short fifo = 0);
GT_API GT_ArcYZCEx(short crd, double y, double z, double yCenter, double zCenter, short circleDir, double synVel, double synAcc, long segNum, short override2, short fifo = 0);
GT_API GT_ArcZXCEx(short crd, double z, double x, double zCenter, double xCenter, short circleDir, double synVel, double synAcc, long segNum, short override2, short fifo = 0);
GT_API GT_ArcXYZEx(short crd, double x, double y, double z, double interX, double interY, double interZ, double synVel, double synAcc, long segNum, short override2, short fifo = 0);
GT_API GT_HelixXYRZEx(short crd, double x, double y, double z, double radius, short circleDir, double synVel, double synAcc, long segNum, short override2, short fifo = 0);
GT_API GT_HelixXYCZEx(short crd, double x, double y, double z, double xCenter, double yCenter, short circleDir, double synVel, double synAcc, long segNum, short override2, short fifo = 0);
GT_API GT_HelixYZRXEx(short crd, double x, double y, double z, double radius, short circleDir, double synVel, double synAcc, long segNum, short override2, short fifo = 0);
GT_API GT_HelixYZCXEx(short crd, double x, double y, double z, double yCenter, double zCenter, short circleDir, double synVel, double synAcc, long segNum, short override2, short fifo = 0);
GT_API GT_HelixZXRYEx(short crd, double x, double y, double z, double radius, short circleDir, double synVel, double synAcc, long segNum, short override2, short fifo = 0);
GT_API GT_HelixZXCYEx(short crd, double x, double y, double z, double zCenter, double xCenter, short circleDir, double synVel, double synAcc, long segNum, short override2, short fifo = 0);

GT_API GT_HelixXYRMultiZEx(short crd, double* pPos, double radius, short circleDir, double synVel, double synAcc, long segNum, short override2, short fifo = 0);
GT_API GT_HelixXYCMultiZEx(short crd, double* pPos, double xCenter, double yCenter, short circleDir, double synVel, double synAcc, long segNum, short override2, short fifo = 0);
GT_API GT_HelixYZRMultiXEx(short crd, double* pPos, double radius, short circleDir, double synVel, double synAcc, long segNum, short override2, short fifo = 0);
GT_API GT_HelixYZCMultiXEx(short crd, double* pPos, double yCenter, double zCenter, short circleDir, double synVel, double synAcc, long segNum, short override2, short fifo = 0);
GT_API GT_HelixZXRMultiYEx(short crd, double* pPos, double radius, short circleDir, double synVel, double synAcc, long segNum, short override2, short fifo = 0);
GT_API GT_HelixZXCMultiYEx(short crd, double* pPos, double zCenter, double xCenter, short circleDir, double synVel, double synAcc, long segNum, short override2, short fifo = 0);

GT_API GT_BufDelayEx(short crd, unsigned short delayTime, short fifo = 0);
GT_API GT_BufIOEx(short crd, unsigned short doType, unsigned short doMask, unsigned short doValue, short fifo = 0);
GT_API GT_BufDAEx(short crd, short channel, short daValue, short fifo = 0);
GT_API GT_BufEnableDoBitPulseEx(short crd, short doType, short doIndex, unsigned short highLevelTime, unsigned short lowLevelTime, long pulseNum, short firstLevel, short fifo);
GT_API GT_BufDisableDoBitPulseEx(short crd, short doType, short doIndex, short fifo);
GT_API GT_BufGearEx(short crd, short gearAxis, double deltaPos, short fifo = 0);
GT_API GT_BufMoveEx(short crd, short moveAxis, double pos, double vel, double acc, short modal, short fifo = 0);
GT_API GT_BufFollowMasterEx(short crd, TBufFollowMaster* pBufFollowMaster, short fifo = 0);
GT_API GT_BufFollowEventCrossEx(short crd, TBufFollowEventCross* pEventCross, short fifo = 0);
GT_API GT_BufFollowEventTriggerEx(short crd, TBufFollowEventTrigger* pEventTrigger, short fifo = 0);
GT_API GT_BufFollowStartEx(short crd, long masterSegment, long slaveSegment, long masterFrameWidth, short fifo = 0);
GT_API GT_BufFollowNextEx(short crd, long width, short fifo = 0);
GT_API GT_BufFollowReturnEx(short crd, double vel, double acc, short smoothPercent, short fifo = 0);

GT_API GT_BufSmartCutterOnEx(short crd, short index, short fifo);
GT_API GT_BufSmartCutterOffEx(short crd, short index, short fifo);

GT_API GT_BufLaserOnEx(short crd, short fifo = 0, short channel = 0);
GT_API GT_BufLaserOffEx(short crd, short fifo = 0, short channel = 0);
GT_API GT_BufLaserFollowModeEx(short crd, short source, short fifo, short channel, double startPower = 0);
GT_API GT_BufLaserFollowTableEx(short crd, short tableId, double minPower, double maxPower, short fifo, short channel);
GT_API GT_BufLaserFollowOffEx(short crd, short fifo, short channel);
GT_API GT_BufLaserPrfCmdEx(short crd, double laserPower, short fifo = 0, short channel = 0);
GT_API GT_BufLaserFollowRatioEx(short crd, double ratio, double minPower, double maxPower, short fifo, short channel);

typedef struct Pos
{
    double machinePos[LA_MACHINE_AXIS_NUM];
    double workPos[LA_WORK_AXIS_NUM];
}TPos;
GT_API GTN_SetupLookAheadCrd(short core, short crd, EMachineMode machineMode);
GT_API GTN_SetAxisFollowModeLa(short core, short crd, int* pFollowMode);
GT_API GTN_SetMinEvenVelTimeLa(short core, short crd, double evenTime);
GT_API GTN_SetMinDccAngle(short core, short crd, double dccAngle);
GT_API GTN_SetMinDccAngleLa(short core, short crd, double dccAngle);
GT_API GTN_SetVelDefineModeLa(short core, short crd, EVelSettingDef velDefMode);
GT_API GTN_SetAccDefineModeLa(short core, short crd, EAccSettingDef accDefMode);
GT_API GTN_SetMaxOverrideLa(short core, short crd, double maxSynVelRatio);
GT_API GTN_SetAxisLimitModeLa(short core, short crd, int* pAxisLimitMode);
GT_API GTN_SetWorkLimitModeLa(short core, short crd, EWorkLimitMode workLimitMode);
GT_API GTN_SetAxisVelValidModeLa(short core, short crd, int velValidAxis);
GT_API GTN_SetAngleCoefTableLa(short core, short crd, short count, double* pAngle, double* pCoef);
GT_API GTN_SetVelModeLa(short core, short crd, EVelMode velMode);
GT_API GTN_SetVelSmoothModeLa(short core, short crd, short smoothMode);
GT_API GTN_SetVelLimitAngleThresholdLa(short core, short crd, double angleThreshold);
GT_API GTN_SetRadiusRatioTableLa(short core, short crd, short count, double* pRadius, double* pRatio);
GT_API GTN_SetCurveVelLimitModeLa(short core,short crd,short mode);
GT_API GTN_PrintLACmdLa(short core, short crd, int printFlag, int clearFile);
GT_API GTN_UpdateMachineBuildingFileLa(short core, short crd, int update);
GT_API GTN_InitialMachineBuilding(short core, short crd, char* pMachineCfgFileName, double* machineCoordCenter, double* workCoordCenter, double toolLength);
GT_API GTN_InitialMachineBuildingEx(short core, short crd, char* pMachineCfgFileName, double* machineCoordCenter, double* workCoordCenter, double toolLength);
GT_API GTN_InitialMachineBuildingPara(short core, short crd, TMachCfgInfo* pMachCfgInfo, double* machineCoordCenter, double* workCoordCenter, double toolLength);
GT_API GTN_GetLookAheadParaEx(short core, short crd, short fifo, TLookAheadParameter* pLookAheadPrm, short* pMotionMode);

// 五轴
GT_API GTN_CrdRTCPOn(short core, short crd, short fifo = 0);
GT_API GTN_CrdRTCPOff(short core, short crd, short fifo = 0);
GT_API GTN_SetNonlinearErrorControl(short core, short crd, int enable, double nonlinearError);
GT_API GTN_EnableDiscreateArc(short core, short crd, short enable, double arcError);
GT_API GTN_MachineForwardTrans(short core, short crd, double* pMachinePos, double* pWorkPos);
GT_API GTN_MachineRTCPTrans(short core, short crd, double* pInputPos, double* pMachinePos, double* pWorkPos);
GT_API GTN_SetCompToolLength(short core, short crd, double compToolLength);
GT_API GTN_SetCompWorkCoordOffset(short core, short crd, double* pCompWorkOffset);
GT_API GTN_UpdateMachineBuildingFile(short core, short crd, int update);
GT_API GTN_SetRotationAxisRange(short core, short crd, TRotationAxisRange* pRotationAxisRange);
GT_API GTN_SetInverseSolutionSelectPara(short core, short crd, EGroupSelect groupSelect, int priorAxisSet);
GT_API GTN_SetPowerOffDo(short core, short mode, short doIndex, short doValue);
GT_API GTN_MachineTransformation(short core, short crd, int posType, double* pPrePos, double* pPos, int* pPosNum, TPos** pReturnPos);
GT_API GTN_SetStartPointProcessMode(short core, short crd, short enable, double z, short fifo);
GT_API GTN_SetWorkCrdLaserFollowMode(short core, short crd, short enbale, short fifo, short chn);

GT_API GTN_SetAxisVelValidCompModeLa(short core, short crd, int enable, int* pCompAxis);
GT_API GTN_SetFollowAxisProcessModeLa(short core, short crd, int AxisFollowMode);
GT_API GTN_SetCmdVelLimitLa(short core, short crd, int enable, int n1, int n2, int mode);
GT_API GTN_InitLookAheadEx(short core, short crd, TLookAheadParameter* pLookAheadParameter, short fifo, short motionMode = 0, TPreStartPos* pPreStartPos = NULL);
GT_API GTN_InitLookAheadPara(short core, short crd, long lookAheadNum, double time, double radiusRatio, double scale, short fifo);
GT_API GTN_SetFollowAxisParaLa(short core, short crd, int* pAxisLimitMode, double* pVmax, double* pAmax, double* pDVmax);
GT_API GTN_SetFollowAxisParaLaPro(short core, short crd, int* pAxisLimitMode, double* pVmax, double* pAmax, double* pDVmax, short count);

GT_API GTN_SetBlendingParaLa(short core, short crd, short paraType, double para, double minAngle, double maxAngle);


GT_API GTN_GetLookAheadSegCountEx(short core, short crd, long* pSegCount, short fifo = 0);
GT_API GTN_GetMotionTimeEx(short core, short crd, double* pTime, short fifo);
GT_API GTN_SetUserSegNumEx(short core, short crd, long segNum, short fifo);
GT_API GTN_CrdDataEx(short core, short crd, TCrdData* pCrdData, short fifo = 0);

GT_API GTN_LnXYEx(short core, short crd, double x, double y, double synVel, double synAcc, long segNum, short override2, short fifo = 0);
GT_API GTN_LnXYG0Ex(short core, short crd, double x, double y, double synVel, double synAcc, long segNum, short override2, short fifo = 0);
GT_API GTN_LnXYZEx(short core, short crd, double x, double y, double z, double synVel, double synAcc, long segNum, short override2, short fifo = 0);
GT_API GTN_LnXYZG0Ex(short core, short crd, double x, double y, double z, double synVel, double synAcc, long segNum, short override2, short fifo = 0);
GT_API GTN_LnXYZAEx(short core, short crd, double x, double y, double z, double a, double synVel, double synAcc, long segNum, short override2, short fifo = 0);
GT_API GTN_LnXYZAG0Ex(short core, short crd, double x, double y, double z, double a, double synVel, double synAcc, long segNum, short override2, short fifo = 0);
GT_API GTN_LnXYZACEx(short core, short crd, double* pPos, short posMask, double synVel, double synAcc, long segNum, short override2, short fifo);
GT_API GTN_LnXYZACG0Ex(short core, short crd, double* pPos, short posMask, double synVel, double synAcc, long segNum, short override2, short fifo);
GT_API GTN_LnXYZACUVWEx(short core, short crd, double* pPos, short posMask, double synVel, double synAcc, long segNum, short override2, short fifo);
GT_API GTN_LnXYZACUVWG0Ex(short core, short crd, double* pPos, short posMask, double synVel, double synAcc, long segNum, short override2, short fifo);
GT_API GTN_ArcXYREx(short core, short crd, double x, double y, double radius, short circleDir, double synVel, double synAcc, long segNum, short override2, short fifo = 0);
GT_API GTN_ArcYZREx(short core, short crd, double y, double z, double radius, short circleDir, double synVel, double synAcc, long segNum, short override2, short fifo = 0);
GT_API GTN_ArcZXREx(short core, short crd, double z, double x, double radius, short circleDir, double synVel, double synAcc, long segNum, short override2, short fifo = 0);
GT_API GTN_ArcXYCEx(short core, short crd, double x, double y, double xCenter, double yCenter, short circleDir, double synVel, double synAcc, long segNum, short override2, short fifo = 0);
GT_API GTN_ArcYZCEx(short core, short crd, double y, double z, double yCenter, double zCenter, short circleDir, double synVel, double synAcc, long segNum, short override2, short fifo = 0);
GT_API GTN_ArcZXCEx(short core, short crd, double z, double x, double zCenter, double xCenter, short circleDir, double synVel, double synAcc, long segNum, short override2, short fifo = 0);
GT_API GTN_ArcXYZEx(short core, short crd, double x, double y, double z, double interX, double interY, double interZ, double synVel, double synAcc, long segNum, short override2, short fifo = 0);
GT_API GTN_ArcXYRACEx(short core, short crd, double x, double y, double a, double c, double radius, short circleDir, double synVel, double synAcc, long segNum, short override2, short fifo = 0);
GT_API GTN_ArcXYCACEx(short core, short crd, double x, double y, double a, double c, double xCenter, double yCenter, short circleDir, double synVel, double synAcc, long segNum, short override2, short fifo = 0);
GT_API GTN_ArcXYZACEx(short core, short crd, double x, double y, double z, double a, double c, double interX, double interY, double interZ, double interA, double interC, double synVel, double synAcc, long segNum, short override2, short fifo = 0);
GT_API GTN_HelixXYRZEx(short core, short crd, double x, double y, double z, double radius, short circleDir, double synVel, double synAcc, long segNum, short override2, short fifo = 0);
GT_API GTN_HelixXYCZEx(short core, short crd, double x, double y, double z, double xCenter, double yCenter, short circleDir, double synVel, double synAcc, long segNum, short override2, short fifo = 0);
GT_API GTN_BufDelayEx(short core, short crd, unsigned short delayTime, short fifo = 0);
GT_API GTN_BufGearEx(short core, short crd, short gearAxis, double deltaPos, short fifo = 0);
GT_API GTN_BufMoveEx(short core, short crd, short moveAxis, double pos, double vel, double acc, short modal, short fifo = 0);
GT_API GTN_BufMoveSpEx(short core, short crd, short moveAxis, double pos, double vel, short smoothTime, short modal, short enableRatio, short fifo);
GT_API GTN_BufMoveSpSetPrmEx(short core, short crd, short moveAxis, double acc, double dec, double velMax, short fifo);
GT_API GTN_BufIOEx(short core, short crd, unsigned short doType, unsigned short doMask, unsigned short doValue, short fifo = 0);
GT_API GTN_BufEnableDoBitPulseEx(short core, short crd, short doType, short doIndex, unsigned short highLevelTime, unsigned short lowLevelTime, long pulseNum, short firstLevel, short fifo);
GT_API GTN_BufDisableDoBitPulseEx(short core, short crd, short doType, short doIndex, short fifo);
GT_API GTN_BufDAEx(short core, short crd, short chn, short daValue, short fifo = 0);
GT_API GTN_BufAuDAEx(short core, short crd, short chn, short daValue, short fifo = 0);

GT_API GTN_BufFollowMasterEx(short core, short crd, TBufFollowMaster* pBufFollowMaster, short fifo = 0);
GT_API GTN_BufFollowEventCrossEx(short core, short crd, TBufFollowEventCross* pEventCross, short fifo = 0);
GT_API GTN_BufFollowEventTriggerEx(short core, short crd, TBufFollowEventTrigger* pEventTrigger, short fifo = 0);
GT_API GTN_BufFollowStartEx(short core, short crd, long masterSegment, long slaveSegment, long masterFrameWidth, short fifo = 0);
GT_API GTN_BufFollowNextEx(short core, short crd, long width, short fifo = 0);
GT_API GTN_BufFollowReturnEx(short core, short crd, double vel, double acc, short smoothPercent, short fifo = 0);


// Smart Cutter
typedef struct SmartCutterPrm
{
    short x;						// X轴对应的规划轴
    short y;						// Y轴对应的规划轴
    short c;						// C轴对应的规划轴

    short tableRadiusIndex;			// 半径补偿表索引
    short tableAngleIndex;			// C轴补偿表索引

    short directionReverse;			// 默认规划位置增大时角度也是增大的

    long offset;		            // C轴旋转角度为0时的规划位置
    long resolution;				// C轴每转脉冲数

    short adcIndex;					// ADC索引
    double adcThreshold;			// ADC触发补偿的阈值
} TSmartCutterPrm;

typedef struct SmartCutterInfo
{
    short enable;
    short execute;
    double radiusValue;
    double angleValue;
} TSmartCutterInfo;

GT_API GTN_SmartCutterOn(short core, short index);
GT_API GTN_SmartCutterOff(short core, short index);
GT_API GTN_SetSmartCutterPrm(short core, short index, TSmartCutterPrm* pPrm);
GT_API GTN_GetSmartCutterPrm(short core, short index, TSmartCutterPrm* pPrm);
GT_API GTN_BufSmartCutterOn(short core, short crd, short smartCutterIndex, short fifo);
GT_API GTN_BufSmartCutterOff(short core, short crd, short smartCutterIndex, short fifo);
GT_API GTN_BufSmartCutterOnEx(short core, short crd, short index, short fifo = 0);
GT_API GTN_BufSmartCutterOffEx(short core, short crd, short index, short fifo = 0);
GT_API GTN_GetSmartCutterInfo(short core, short index, TSmartCutterInfo* pInfo);
GT_API GTN_SetSmartCutterValue(short core, short index, double radiusValue, double angleValue);
GT_API GTN_SmartCutterStart(short core, short index);


GT_API GT_BufWaitDiEx(short crd, short diType, unsigned short diIndex, unsigned short level, short continueTime, long overTime, short flagMode, long segNum, short fifo);
GT_API GT_BufWaitLongVarEx(short crd, short index, long value, long overTime, short flagMode, long segNum, short fifo);
GT_API GTN_BufWaitDiEx(short core, short crd, short diType, unsigned short diIndex, unsigned short level, short continueTime, long overTime, short flagMode, long segNum, short fifo);
GT_API GTN_BufWaitLongVarEx(short core, short crd, short index, long value, long overTime, short flagMode, long segNum, short fifo);
GT_API GTN_BufWaitWatchVarEx(short core, short crd, TWatchCondition* pWaitCondition, TWaitTimeout* pWaitTimeout, long segNum, short fifo);
GT_API GTN_BufSetWaitLongVarEx(short core,short crd,short index,long value,short fifo);

GT_API GT_BufDoBitEx(short crd, unsigned short doType, unsigned short index, short value, short fifo);
GT_API GTN_BufDoBitEx(short core, short crd, unsigned short doType, unsigned short index, short value, short fifo);
GT_API GTN_BufDoBitDelayEx(short core, short crd, unsigned short doType, unsigned short index, short value, long delayTime, short fifo);
GT_API GTN_BufSetOverrideEx(short core, short crd, double synVelRatio, short mode, short fifo);
GT_API GTN_BufStopPosEx(short core, short crd, short profile, TStopPos* pStopPos, short modal, short fifo);
GT_API GTN_BufSetSoftLimitEx(short core, short crd, short axis, double positive, double negative, short fifo);

// 缓存区位置比较
GT_API GTN_BufPosComparePsoPrm(short core, short crd, short index, TPosComparePsoPrm* pPrm, short fifo);
GT_API GTN_BufPosComparePsoPrmPro(short core, short crd, short index, TPosComparePsoPrmPro* pPrmPro, short fifo);
GT_API GTN_BufPosCompareStart(short core, short crd, short fifo, short index);
GT_API GTN_BufPosCompareStop(short core, short crd, short fifo, short index);

GT_API GT_BufPosCompareStartEx(short core, short crd, short fifo, short index);
GT_API GT_BufPosCompareStopEx(short core, short crd, short fifo, short index);
GT_API GTN_BufPosComparePsoPrmEx(short core, short crd, short index, TPosComparePsoPrm* pPrm, short fifo);
GT_API GTN_BufPosComparePsoPrmProEx(short core, short crd, short index, TPosComparePsoPrmPro* pPrmPro, short fifo);
GT_API GTN_BufPosCompareStartEx(short core, short crd, short fifo, short index);
GT_API GTN_BufPosCompareStopEx(short core, short crd, short fifo, short index);

GT_API GTN_BufPosComparePulse(short core, short crd, short index, short outputMode, short level, unsigned short outputPulseWidth, short fifo);
GT_API GTN_BufPosComparePulseEx(short core, short crd, short index, short outputMode, short level, unsigned short outputPulseWidth, short fifo);

GT_API GTN_BufPosCompareMultiPulse(short core, short crd, short index, TPosCompareMultiPulse* pPosComparePulse, short fifo);
GT_API GTN_BufPosCompareMultiPulseEx(short core, short crd, short index, TPosCompareMultiPulse* pPosComparePulse, short fifo);

/**
 * @brief 前瞻插补缓冲区设置pso间距
 * @param core 核号
 * @param crd 插补坐标系号
 * @param posCompareIndex pso索引
 * @param synchPos pso间距，精度：取至8位小数，单位：pulse
 * @param fifo 插补坐标系缓存区号
 * @return
*/
GT_API GTN_BufSetPosComparePsoSynchPosEx(short core,short crd,short posCompareIndex,double synchPos,short fifo);

GT_API GTN_BufPsoStart(short core, short crd, short fifo, short index, long x, long y);
GT_API GTN_BufPsoStop(short core, short crd, short fifo, short index, long x, long y);

GT_API GTN_BufPsoStartEx(short core, short crd, short fifo, short index, double x, double y);
GT_API GTN_BufPsoStopEx(short core, short crd, short fifo, short index, double x, double y);

/*-----------------------------------------------------------*/
/* 压力补偿                                            */
/*-----------------------------------------------------------*/
typedef struct AxisPressPid
{
    double kp;
    double ki;
    double kd;
    double integralLimit;	//积分极限
    double derivativeLimit;	//微分极限
    double limit;			//调节限制（力或电压）
    double pad1[4];
}TAxisPressPid;

#define PRESS_COMPENSATE_MODE_LINEAR                       (0)                           // 线性补偿
#define PRESS_COMPENSATE_MODE_TABLE	                       (1)                           // 查表补偿
#define PRESS_COMPENSATE_MODE_REGION_LEARN                 (2)                           // 区域内自学习补偿

typedef struct AxisPressCompensate
{
    short enable;		           //是否使能，0-关闭，1-使能
    short type;			           //输入类型，电压或网络模块数据
    short dimension;	           //补偿输入的维度，最大3维
    short index[3];		           //输入的索引,DAC从1开始，ECAT IO模块站号从0开始
    short subIndex[3];             //输入的子索引，,DAC该参数无效，ECAT IO模块的IOMAP从0开始
    short mode;			           //模式，线性还是查表
    short revolveAxis;             //用来计算旋转角度的轴号
    short regionAxisIndex;         //补偿功能区间有效的参考轴
    short relatedMasterIndex;      //随动主轴的索引
    short pad1[3];
    double target;			       //目标力或电压
    double thredshold;		       //thredshold，什么时候开始补偿
    double deadZone;		       //死区力或电压，死区内不补偿
    double factor;			       //力和位移的转化系数
    double revolveOffset;	       //初始旋转的脉冲数，默认合成方向和ingdex[0]的方向重合
    double revolveScale;	       //旋转轴的一圈脉冲数
    TAxisPressPid pid;
    long compPosMaxP;              //输出补偿位置区间[N,P]的端点P
    long compPosMaxN;              //输出补偿位置区间[N,P]的端点N
    double k;	                   //补偿量滤波系数 0-1 数值越大滤波越强
    double pad2[4];
    long activeRegionP;            //补偿功能有效的规划位置区间[N,P]的端点P
    long activeRegionN;            //补偿功能有效的规划位置区间[N,P]的端点N
    long activeRegionInterval;     //补偿功能有效的规划位置区间内的自学习间隔，当mode为自学习PRESS_COMPENSATE_MODE_REGION_LEARN时有效
    long relatedMasterEven;        //随动主轴的比例
    long relatedSlaveEven;         //随动从轴的比例
    long pad3[3];
} TAxisPressCompensate;

GT_API GT_SetAxisPressCompensate(short axis, TAxisPressCompensate* pPressComp);
GT_API GT_GetAxisPressCompensate(short axis, TAxisPressCompensate* pPressComp);
GT_API GTN_SetAxisPressCompensate(short core, short axis, TAxisPressCompensate* pPressComp);
GT_API GTN_GetAxisPressCompensate(short core, short axis, TAxisPressCompensate* pPressComp);
GT_API GT_SetAxisPressCompensateTable(short axis, short index, long count, double* pPressData, double* pPosData);
GT_API GT_SelectAxisPressCompensateTable(short axis, short index);
GT_API GTN_SetAxisPressCompensateTable(short core, short axis, short index, long count, double* pPressData, double* pPosData);
GT_API GTN_SelectAxisPressCompensateTable(short core, short axis, short index);

typedef struct AxisPressCompensateFixFactor
{
    short type;			      //输入类型，电压或网络模块数据
    short dimension;	      //补偿输入的维度，最大3维
    short index[3];		      //输入的索引,DAC从1开始，ECAT IO模块站号从0开始
    short subIndex[3];        //输入的子索引，,DAC该参数无效，ECAT IO模块的IOMAP从0开始

    double targetMax;	      //标定的力最大值，达到或超过该值时标定结束
    double targetMin;	      //标定的力最小值，达到或小于该值时标定结束
    double factor;		      //力和位移的转化系数，启动时传入0,获取状态时得到标定值。

    long fixRegionP;          //标定规划位置区间[N,P]的端点P，启动位置的相对位置
    long fixRegionN;          //标定规划位置区间[N,P]的端点N，启动位置的相对位置
    long fixRegionInterval;   //标定规划位置区间内的标定间隔

    long tmp[16];
} TAxisPressCompensateFixFactor;

GT_API GTN_StartAxisPressCompensateFixFactor(short core, short axis, TAxisPressCompensateFixFactor* pPressComp);
GT_API GTN_GetAxisPressCompensateFixFactorStatus(short core, short axis, short* pFixFactorSts, TAxisPressCompensateFixFactor* pPressComp);

typedef struct AxisPressCompensateFixPid
{
    short type;			 //输入类型，电压或网络模块数据
    short dimension;     //补偿输入的维度，最大3维
    short index[3];		 //输入的索引,DAC从1开始，ECAT IO模块站号从0开始
    short subIndex[3];   //输入的子索引，,DAC该参数无效，ECAT IO模块的IOMAP从0开始

    double target;		 //目标力或电压
    double factor;		 //力和位移的转化系数
    TAxisPressPid pid;

    long fixPosLimitP;   //标定位置区间[N,P]的端点P
    long fixPosLimitN;   //标定补偿位置区间[N,P]的端点N
    double thredshold;   //thredshold，什么时候开始补偿
    double deadZone;	 //死区力或电压，死区内不补偿

    //自整定内部参数，调试使用，后续用户接口没有该部分
    double Knp;			 //压力环全局增益
    double K1;			 //全局增益递增系数1
    double K2;			 //全局增益递增系数2
    double K3;			 //全局增益递增系数3
    double Krise;		 //上升系数
    double Kpeak;		 //峰值系数
    double Tset;		 //目标响应时间
    double Td;			 //保守响应时间

    double tmp[8];
} TAxisPressCompensateFixPid;

GT_API GTN_StartAxisPressCompensateFixPid(short core, short axis, TAxisPressCompensateFixPid* pPressComp);
GT_API GTN_GetAxisPressCompensateFixPidStatus(short core, short axis, short* pFixPidSts, TAxisPressCompensateFixPid* pPressComp);

GT_API GTN_AxisPressCompensateEnable(short core, short axis, short enable, double deadZone, double factor, TListInfo* pListInfo);
GT_API GTN_SetAxisPressCompensateTarget(short core, short axis, double target, double thredshold, TListInfo* pListInfo);

GT_API GTN_BufAxisPressCompensatePrmEx(short core, short crd, short axis, double target, double thredshold, short fifo = 0);
GT_API GTN_BufAxisPressCompensateEx(short core, short crd, short axis, short enable, double deadZone, double factor, short fifo = 0);

GT_API GTN_SetCrdUserDataEndVelLa(short core, short crd, short crdUserDataType, double endVel);

/*-----------------------------------------------------------*/
/* PVT模式缓冲区激光操作                                      */
/*-----------------------------------------------------------*/
#define PVT_OPERATION_TYPE_LASER_ON					(2)
#define PVT_OPERATION_TYPE_BUF_DA					(4)
#define PVT_OPERATION_TYPE_LASER_CMD				(5)
#define PVT_OPERATION_TYPE_LASER_FOLLOW_RATIO		(6)
#define PVT_OPERATION_TYPE_LASER_FOLLOW_MODE		(25)
#define PVT_OPERATION_TYPE_LASER_FOLLOW_OFF			(26)
#define PVT_OPERATION_TYPE_BUF_DO_BIT				(94)
#define CRD_OPERATION_TYPE_BUF_POS_COMPARE_START 	(90)
#define CRD_OPERATION_TYPE_BUF_POS_COMPARE_STOP 	(91)

typedef struct PVTDoBit
{
    short doType;
    short index;
    short value;
}TPVTDoBit;

typedef struct PVTDA
{
    short chn;
    short daValue;
}TPVTDA;

typedef struct PVTLaserSwitch
{
    short chn;
    short enable;
}TPVTLaserSwitch;

typedef struct PVTLaserPrfCmd
{
    short chn;
    double power;
}TPVTLaserPrfCmd;

typedef struct PVTLaserFollowRatio
{
    short  chn;
    double ratio;
    double minPower;
    double maxPower;
}TPVTLaserFollowRatio;

typedef struct PVTLaserFollowOff
{
    short  chn;
}TPVTLaserFollowOff;

typedef struct PVTLaserFollowMode
{
    short  chn;
    short  source;
    double startPower;
}TPVTLaserFollowMode;


/*-----------------------------------------------------------*/
/* 绝对值编码器	                                            */
/*-----------------------------------------------------------*/
GT_API GTN_RN_GetAbsEncPosEx(short core, short encoder, long long* pEncPos);
GT_API GTN_RN_SetEncMultiLinesEx(short core, short encoder, unsigned long long multiLines);

GT_API GTN_RN_SetEncMultiLines(short core, short stationId, short axis, unsigned long long multiLines);
GT_API GTN_SetAbsEncMultiTurnRange(short cardIndex, short encoder, double range);
GT_API GTN_ReadAbsEncPos(short core, short encoder, double* pPos);

//-------------------------------------------------------------------------------------------------------
// 功能说明：读取绝对值编码器的多圈计数和单圈绝对位置，物理索引接口
// input：cardIndex----卡号，取值范围：[1,16]
// input：stationPhyId----物理站号，取值范围：[0,64]
// input：encoder----编码器在当前站的物理序号，取值范围：[1,8]
// output：pMultiPos----读取的多圈计数
// output：pSinglePos----读取的单圈绝对位置
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_GetAbsEncMultiAndSinglePos(short cardIndex, short stationPhyId, short encoder, unsigned long* pMultiPos, unsigned long* pSinglePos);

//-------------------------------------------------------------------------------------------------------
// 功能说明：读取绝对值编码器的多圈计数和单圈绝对位置，物理索引接口
// input：encoder----编码器在当前站的物理序号，取值范围：[1,8]
// output：pMultiPos----读取的多圈计数
// output：pSinglePos----读取的单圈绝对位置
//-------------------------------------------------------------------------------------------------------
GT_API GTN_GetAbsEncMultiAndSinglePos(short core,short encoder,unsigned long*pMultiPos,unsigned long*pSinglePos);

/*-----------------------------------------------------------*/
/* 加密功能	                                            		*/
/*-----------------------------------------------------------*/
typedef struct EncryptConfigPrm
{
    char uuid[32];		// 原始密钥
    char mcInf[32];		//
}TEncryptConfigPrm;

typedef struct EncryptData
{
    char keyValue[32];		// 密钥
    char data[32];			// 读取到的或者写下去的新的数据
}TEncryptData;

typedef struct EncryptChipInf
{
    short type;					// 芯片类型
    short userKeySlotOffset;		// 秘钥槽偏置
    short userDataSlotOffset;		// 数据槽偏置
    short cardType;		// 控制卡类型
    short reverse1[8];
}TEncryptChipInf;

typedef struct EncryptAuxInf
{
    char sn[9];				// 芯片序列号
    char random[32];		// 返回的随机数
    TEncryptChipInf chipInf;	// 芯片其他信息
}TEncryptAuxInf;

typedef struct EncryptDataPro
{
    char data[32];			// 读取到的加密数据
    char hMac[32];			//
}TEncryptDataPro;

GT_API GTN_SetEncryptConfig(short core, TEncryptConfigPrm* pConfigPrm, short* pSts);
// 基础版本加密功能
//  加密读写数据
GT_API GTN_GetEncryptData(short core, TEncryptData* pReadPrm);
GT_API GTN_SetEncryptData(short core, TEncryptData* pWritePrm);
// 晋级版本，开放解密函数，用户根据读取的密文、设置的密钥解密
GT_API GTN_GetEncryptAuxInf(short core, TEncryptAuxInf* pAuxInf, short* pSts);
GT_API GTN_SetEncryptDataPro(short core, short slotIndex, short block, TEncryptDataPro* pWritePrm, short* pSts);
GT_API GTN_GetEncryptDataPro(short core, short slotIndex, short block, TEncryptDataPro* pReadPrm, short* pSts);

// 晋级版本，更新秘钥
typedef struct EncryptKey
{
    char oldKey[32];		// 原始密钥
    char newKey[32];		// 新密钥
}TEncryptKey;

GT_API GTN_UpdateEncryptKey(short core, TEncryptKey* pKeyInf, short* pSts);

/*-----------------------------------------------------------*/
/* press				                                           	 */
/*-----------------------------------------------------------*/
#define LOOP_MODE_POSITION               (0)
#define LOOP_MODE_PRESS                  (1)
#define LOOP_MODE_NONE                   (2)

#define LIMIT_TYPE_PRESS				(7)
#define LIMIT_TYPE_TORQUE				(8)

#define MC_PRESS                        (600)
#define MC_TORQUE                       (601)

typedef struct LoopMode
{
    short loopMode;
    short pressProfileMode;
    short pad[2];
} TLoopMode;

typedef struct PressPrm
{
    short active;			// 功能使能
    short pad1;
    long scale;				// 当量，物理量1牛（N）转换到脉冲的当量。
    short linkAxis;			// 压力规划器与那个轴相关联。
    short pad2[3];
} TPressPrm;

typedef struct PressTargetPrm
{
    double acc;				// 力矩加速度
    double dec;				// 力矩加速度
    double pressStart;		// 起跳力矩
    short  smoothTime;		// 平滑时间
    short pad[3];
} TPressTargetPrm;

typedef struct PressArrayData
{
    double pressTarget;		// 目标压力
    long pressTime;			// 爬升时间 ms单位
    long holdTime;			// 保持时间
} TPressArrayData;

typedef struct PressArray
{
    short count;
    short pad1[3];
    TPressArrayData buffer[4];
    short exit;				// 规划完成后是否切换到位置闭环模式
    short pad2[3];
} TPressArray;

typedef struct PressPid
{
    double kp;
    double ki;
    double kd;
    double kvff;
    double kaff;
    long integralLimit;
    long derivativeLimit;
    short limitMax;
    short limitMin;
    short pad[2];
} TPressPid;

typedef struct PressAutoSwitchPrm
{
    short limit1;
    short limit2;
    short time;
    short triggerCondition;        // 压力急停触发条件，PRESS_GREATER_THAN_LIMIT,PRESS_LESS_THAN_LIMIT,
    short loopMode;					// 触发后切换到什么模式，什么规划。
    short pressProfileMode;
    short pad[2];
} TPressAutoSwitchPrm;

typedef struct StopOffset
{
    long distance;		// 回退距离
    short pad[2];
    double vel;			// 回速度
    double acc;			// 回退加速度
} TStopOffset;

GT_API GTN_SetLoopMode(short core, short axis, TLoopMode* pMode);
GT_API GTN_GetLoopMode(short core, short axis, TLoopMode* pMode);
GT_API GTN_SetPressPrm(short core, short pressAxis, TPressPrm* pPressPrm);
GT_API GTN_GetPressPrm(short core, short pressAxis, TPressPrm* pPressPrm);
GT_API GTN_SetPressTarget(short core, short pressProfile, double value, TPressTargetPrm* pPressTargetPrm);
GT_API GTN_GetPressTarget(short core, short pressProfile, double* pValue, TPressTargetPrm* pPressTargetPrm);
GT_API GTN_SetPressArray(short core, short pressProfile, TPressArray* pPressArray);
GT_API GTN_GetPressArray(short core, short pressProfile, TPressArray* pPressArray);
GT_API GTN_SetPressPid(short core, short pressControl, TPressPid* pPid);
GT_API GTN_GetPressPid(short core, short pressControl, TPressPid* pPid);
GT_API GTN_GetPrfPress(short core, short pressProfile, double* pValue, double* pValueFilter);
GT_API GTN_GetAtlPress(short core, short pressAxis, double* pValue);
GT_API GTN_GetPressStatus(short core, short pressAxis, long* pStatus);
GT_API GTN_SetPressAutoSwitchPrm(short core, short pressAxis, TPressAutoSwitchPrm* pPrm);
GT_API GTN_GetPressAutoSwitchPrm(short core, short pressAxis, TPressAutoSwitchPrm* pPrm);
GT_API GTN_PressAutoSwitchEnable(short core, short pressAxis, short enable);
GT_API GTN_GetPressFeedbackType(short core, short pressAxis, short* pFbType, short* pFbIndex);
GT_API GTN_SetPressFeedbackType(short core, short pressAxis, short fbType, short fbIndex);
GT_API GTN_SetPrfPress(short core, short pressProfile, double prfPressValue);
GT_API GTN_SetStopOffset(short core, short axis, short mode, TStopOffset* pStopOffset);
GT_API GTN_GetStopOffset(short core, short axis, short mode, TStopOffset* pStopOffset);
GT_API GTN_SetPressRange(short core, short pressAxis, short centerSynch, long range);
GT_API GTN_GetPressRange(short core, short pressAxis, long* pCenterPos, long* pRange);
GT_API GTN_SetPressCross(short core, short pressAxis, short dir, long pos);
GT_API GTN_GetPressCross(short core, short pressAxis, short* pDir, long* pPos);
GT_API GTN_SetPressTrigger(short core, short pressAxis, short pressThread, short triggerCondition);
GT_API GTN_GetPressTrigger(short core, short pressAxis, short* pPressThread, short* pTriggerCondition);
GT_API GTN_PressTriggerOn(short core, short pressAxis);
GT_API GTN_PressTriggerOff(short core, short pressAxis);
GT_API GTN_GetPressTriggeredPos(short core, short pressAxis, short* pEnable, double* pCntPos, short* pTriggeredPress);

/*-----------------------------------------------------------*/
/* Command											                              */
/*-----------------------------------------------------------*/
GT_API GTN_GetCommandExecuteTime(short core, short mode, double* pTime);
GT_API GTN_ClearCommandExecuteTime(short core, short mode);
GT_API GTN_GetCmdCount(short core, short crd, short* pResult, short fifo);
GT_API GTN_InitCmdExExtern(short core, short cmd);
GT_API GTN_GetFloat64ExExtern(short core, double* pData);
GT_API GTN_AddFloat64ExExtern(short core, double data);
GT_API GTN_SendCommandExExtern(short core);


typedef struct CombineAxes
{
    short master[2];
    short masterValueSource[2];
    long gearRatioNumerator[2];
    long gearRatioDenominator[2];
} TCombineAxes;

typedef struct CombineAxesStatus
{
    short enable;
    double slavePos;
    double slaveVel;
} TCombineAxesStatus;

GT_API GTN_SetCombineAxes(short core, short index, TCombineAxes* pCombineAxes);
GT_API GTN_GetCombineAxes(short core, short index, TCombineAxes* pCombineAxes);
GT_API GTN_CombineAxesOn(short core, short index);
GT_API GTN_CombineAxesOff(short core, short index);
GT_API GTN_GetCombineAxesStatus(short core, short index, TCombineAxesStatus* pCombineAxesStatus);

typedef struct Addition
{
    short type;
    short index[2];
} TAddition;

typedef struct AdditionStatus
{
    double pos;
    double vel;
} TAdditionStatus;

GT_API GTN_SetAxisAddition(short core, short axis, short dataType, TAddition* pAddition);
GT_API GTN_GetAxisAddition(short core, short axis, short dataType, TAddition* pAddition);
GT_API GTN_GetAxisAdditionStatus(short core, short axis, short dataType, TAdditionStatus* pAdditionStatus);
GT_API GTN_ZeroAxisAdditionPos(short core,short axis,short dataType,short count=1);

typedef struct ContourConfig
{
    short profileType[3];
    short profileIndex[3];
    short profileId[3];
    short encoderType[3];
    short encoderIndex[3];
    short encoderId[3];
} TContourConfig;

typedef struct ContourControl
{
    double kp[3];
    double ki[3];
    short errorWindow[3];
    short limit[3];
} TContourControl;

typedef struct ContourMode
{
    short cornerStepIn;
    short cornerStepOut;
    short reserved1[2];
    double cornerTheta;
    short reserved2[4];
    double window;
    double para;
} TContourMode;

GT_API GTN_SetContourConfig(short core, short index, TContourConfig* pContourConfig);
GT_API GTN_GetContourConfig(short core, short index, TContourConfig* pContourConfig);
GT_API GTN_SetContourControl(short core, short index, TContourControl* pContourControl);
GT_API GTN_GetContourControl(short core, short index, TContourControl* pContourControl);
GT_API GTN_SetContourMode(short core, short index, TContourMode* pContourMode);
GT_API GTN_GetContourMode(short core, short index, TContourMode* pContourMode);
GT_API GTN_SetContourEnable(short core, short index, short enable);
GT_API GTN_SetControlAddition(short core, short index, TAddition* pAddition);
GT_API GTN_GetControlAddition(short core, short index, TAddition* pAddition);
GT_API GTN_SetDisplayDip(short core, short mode);
GT_API GTN_GetDisplayDip(short core, short* pMode);
GT_API GTN_SetMcOverride(short core, short axis, short mode, double override, double smoothTime);
GT_API GTN_GetMcOverride(short core, short axis, short mode, double* pOverrideTarget, double* pSmoothTime, double* pOverride);
typedef struct PVTPosCompareStart
{
    short  index;
}TPVTPosCompareStart;

typedef struct PVTPosCompareStop
 {
    short  index;
}TPVTPosCompareStop;
/*-----------------------------------------------------------*/
/* 工具函数，逻辑和物理转换									 */
/*-----------------------------------------------------------*/
typedef struct ResPhyInfo
{
    short index;
    short id;
    short reverse1[2];
    double reverse2[4];
}TResPhyInfo;
GT_API GTN_GetResPhyInfo(short core, short type, short index, TResPhyInfo* pStationPhyInfo);

// 逻辑信息结构体
typedef struct LogicInfo
{
    short core;                                  // 逻辑核号,从1开始
    short dataType;                              // 需要转换的资源类型
    short index;                                 // 逻辑索引号,从1开始
    short reserve[5];                            // 保留空间
} TLogicInfo;

// 物理信息结构体
typedef struct PhysicInfo
{
    short cardIndex;                             // 物理卡号,从1开始
    short stationPhyId;                          // 物理站号
    short modulePhyId;                           // 扩展模块站号
    short dataType;                              // 需要转换的资源类型
    short index;                                 // 物理索引号
    short reserve[3];                            // 保留空间
} TPhysicInfo;
#define PORT_A_GENERAL_GLINK_II		(1)
#define PORT_B_GENERAL_GLINK_II		(2)
#define PORT_C_HIGH_SPEED_GLINK_II	(3)
#define PORT_D_ETHER_CAT			(4)
GT_API GTN_ReadPhysicalMap(void);
GT_API ConvertPhysical(short core, short dataType, short terminal, short index);
GT_API ConvertLogical(TLogicInfo* pLogicInfo, TPhysicInfo* pPhysicInfo);

/*---------------------------------------------------------------------------------------------*/
/* 串行通讯指令（通用485/232通信指令）（gts.lib中导出，推荐使用）							   */
/*---------------------------------------------------------------------------------------------*/
GT_API GTN_RN_ComSerialOpen(short cardIndex, short stationPhyId, short comIndex);
GT_API GTN_RN_ComSerialClose(short cardIndex, short stationPhyId, short comIndex);
GT_API GTN_RN_ComSerialRead(short cardIndex, short stationPhyId, short comIndex, unsigned long  readLen, unsigned long* pResLen, unsigned char* pData);
GT_API GTN_RN_ComSerialReadFast(short cardIndex, short stationPhyId, short comIndex, unsigned long readLen, unsigned long* pResLen, unsigned char* pData);
GT_API GTN_RN_ComSerialWrite(short cardIndex, short stationPhyId, short comIndex, unsigned long writeLen, unsigned long* pResLen, unsigned char* pData);
GT_API GTN_RN_ComSerialGetState(short cardIndex, short stationPhyId, short comIndex, unsigned char* pState);
GT_API GTN_RN_ComSerialSetSettings(short cardIndex, short stationPhyId, short comIndex, unsigned long baudrate, unsigned char stopBits, unsigned char parity);
GT_API GTN_RN_ComSerialClearErr(short cardIndex, short stationPhyId, short comIndex, unsigned char flag);
GT_API GTN_RN_ComSerialSetMode(short cardIndex, short stationPhyId, short comIndex, unsigned short comMode);
GT_API GTN_RN_ComSerialGetRecvFifoCnt(short cardIndex, short stationPhyId, short comIndex, short* pCount);

//扩展模块物理寻址指令集
GT_API GTN_RN_SetExtDiDoReversePhysical(short cardIndex, short stationPhyId, short moduleId, long* pInputReverse, long inputCount, long* pOutputReverse, long outputCount);
GT_API GTN_RN_GetExtDiDoReversePhysical(short cardIndex, short stationPhyId, short moduleId, long* pInputReverse, long inputCount, long* pOutputReverse, long outputCount);
GT_API GTN_RN_GetExtAoPhysical(short cardIndex, short stationPhyId, short moduleId, long aoIndex, double* pValue, long count);
GT_API GTN_RN_GetExtAoValuePhysical(short cardIndex, short stationPhyId, short moduleId, long aoIndex, long* pValue, long count);
GT_API GTN_RN_GetExtAiPhysical(short cardIndex, short stationPhyId, short moduleId, long aiIndex, double* pValue, long count);
GT_API GTN_RN_GetExtAiValuePhysical(short cardIndex, short stationPhyId, short moduleId, long aiIndex, long* pValue, long count);
GT_API GTN_RN_SetExtAoValuePhysical(short cardIndex, short stationPhyId, short moduleId, long aoIndex, long* pValue, long count);
GT_API GTN_RN_SetExtAoPhysical(short cardIndex, short stationPhyId, short moduleId, long aoIndex, double* pValue, long count);
GT_API GTN_RN_GetExtDiBitPhysical(short cardIndex, short stationPhyId, short moduleId, long diIndex, unsigned long* pValue, long count);
GT_API GTN_RN_GetExtDiPhysical(short cardIndex, short stationPhyId, short moduleId, unsigned long* pValue, long count);
GT_API GTN_RN_GetExtDoPhysical(short cardIndex, short stationPhyId, short moduleId, unsigned  long* pValue, long count);
GT_API GTN_RN_SetExtDoBitPhysical(short cardIndex, short stationPhyId, short moduleId, long doIndex, unsigned long* pValue, long count);
GT_API GTN_RN_SetExtDoPhysical(short cardIndex, short stationPhyId, short moduleId, unsigned long* pValue, long count);
GT_API GTN_RN_SetExtModuleMode(short cardIndex, short mode);
GT_API GTN_RN_GetExtModuleMode(short cardIndex, short* pMode);

GT_API GTN_RN_UpdateExtModuleFirmware(short cardIndex, short stationPhyId, short moduleId, const char* pFile);
GT_API GTN_RN_SetExtModuleComLedSts(short cardIndex, short stationPhyId, short moduleId, long value);
GT_API GTN_RN_GetExtModuleInfo(short cardIndex, short stationPhyId, short moduleId, short infoType, long* pValue);


























/*------------------------------------------------------------------------*/
/*------------------------------------------------------------------------*/
/*                        开发版本功能函数                                */
/*------------------------------------------------------------------------*/
/*------------------------------------------------------------------------*/

// 用户手动配置(XML)初始化网络指令
GT_API GTN_OpenCard(short channel = 5, void* pPrm = NULL, char* pFileName = NULL);
/**
 * @brief 读取设备句柄
 * @param cardNum 卡号，取值范围[1,8]
 * @param pHandle 返回设备句柄
 * @return 0表示开卡成功，非0表示开卡失败
 * @notice （7）卡号超出范围
 *         （-6）设备未打开
*/
GT_API GTN_GetHandle(short cardNum,HANDLE *pHandle);
GT_API GTN_InitRingNet(short mode, char* pFileName);
GT_API GTN_SetJtGshdCount(short core, short gshdCount, short lastStationAxisCount);
GT_API GT_GetCardType(unsigned long* pCardTypeArray, short cardTypeArraySize, short* pCardTypeCount);
GT_API GTN_GetCardType(unsigned long* pCardTypeArray, short cardTypeArraySize, short* pCardTypeCount);

#define NET_INIT_MODE_AUTO                       (0)    // 自动扫描网络，默认模式
#define NET_INIT_MODE_SKIP_TERMINAL              (1)    // 支持ECAT xml文件跳站模式，
#define NET_INIT_MODE_MULT_OPEN                  (2)    // 双开模式，

#define NET_INIT_MODE_MOTION_STUDIO              (99)   // GVN时MotionStudio后门模式，不需要XML
#define NET_INIT_MODE_XML_STRICT                 (100)  // XML严格开卡模式

#define NO_NET                                   (0xf)
#define RING_NET                                 (6)
#define ETHERCAT_NET                             (8)
#define ETHERCAT_RING_MISC_NET                   (9)

#define NET_SRC_NONE                             (NO_NET)
#define NET_SRC_FROM_RING_NET                    (RING_NET)
#define NET_SRC_FROM_ECAT_NET                    (ETHERCAT_NET)
typedef struct SlaveRelateInfo
{
    short relateEnable;
    short netSrc;                                // 软件资源对应的外部网络。来源于RingNet还是EtherCAT,或者其他网络。
    short netSrcIndex;                           // 外部网络对应的索引。
    short reserve1[9];
    double reserve2[3];
}TSlaveRelateInfo;

GT_API GTN_NetInit(short mode, void* pPrm, short overTime, long* pStatus);
GT_API GTN_SetMcSrcToSlaveRelateInfo(short core, short mcType, TSlaveRelateInfo* pSlaveInfo, short count);
GT_API GTN_GetMcSrcToSlaveRelateInfo(short core, short mcIndex, short type, TSlaveRelateInfo* pSlaveInfo, short count);

GT_API GTN_SetAxisCircularSafetyZone(short core, short safetyZoneIndex, short enable, TAxisCircularSafetyZone* pPrm);
GT_API GTN_GetAxisCircularSafetyZone(short core, short safetyZoneIndex, short* pEnable, TAxisCircularSafetyZone* pPrm);
GT_API GTN_SetPrfRefSafetyZone(short core, short profile, short enable, TProfileReferenceSafetyZone* pPrm);
GT_API GTN_GetPrfRefSafetyZone(short core, short profile, short* pEnable, TProfileReferenceSafetyZone* pPrm);
GT_API GTN_GetPrfRefSafetyZoneSts(short core, short profile, long* pSts);
GT_API GTN_GetAuEncVelFilter(short core, short type, short encoder, short* pFilterNum);
GT_API GTN_SetAuEncVelFilter(short core, short type, short encoder, short filterNum);

GT_API GTN_SetPosEx(short core, short profile, double pos);
GT_API GTN_GetPosEx(short core, short profile, double* pPos);

GT_API GTN_PrfTrapEx(short core, short profile);
GT_API GTN_GetTrapRunTime(short core, short profile, TTrapPrm* pPrm, double currentPos, double targetPos, double targetVel, double runPos, double* pRunTime);
GT_API GTN_GetTrapRunPos(short core, short profile, TTrapPrm* pPrm, double currentPos, double targetPos, double targetVel, double runTime, double* pRunPos);

GT_API GTN_PvtTableUserData(short core, short tableId, short userDataType, double time, void* pData);

typedef struct PvtTableMoveAbsolutePrm
{
    double distance;
    double vM;				// 设置的最大速度
    double acc;				// 设置的z最大加速度
    double pa1;				// 加速百分比1
    double pa2;             // 加速百分比2
    double dec;				// 设置的z最大减速度
    double pd1;				// 减速度百分比1
    double pd2;				// 减速度百分比2

    double velM;			// 实际能达到的最大速度,返回读取的
    double accM;			// 实际能达到的最大加速度,返回读取的
    double decM;			// 实际能达到的最大减速度,返回读取的
    double time;			// 实际规划时间，返回读取的
}TPvtTableMoveAbsolutePrm;
GT_API GTN_PvtTableMoveAbsoluteTwoSegment(short core, short profile, TPvtTableMoveAbsolutePrm* pTargetPos1, double* pDelay, TPvtTableMoveAbsolutePrm* pTargetPos2, short count);

/*-----------------------------------------------------------*/
/* Command Array			                                       */
/*-----------------------------------------------------------*/
GT_API GTN_SetTriggerArray(short core, short trigger, TTrigger* pTrigger, short count);

GT_API GTN_GetFunIdPro(short core, char* pFunName, short* pFunId);
GT_API GTN_GetVarIdPro(short core, char* pFunName, char* pVarName, TVarInfo* pVarInfo);

GT_API GTN_SetVarValueEx(short core, short page, TVarInfo* pVarInfo, double* pValue, short count = 1);
GT_API GTN_GetVarValueEx(short core, short page, TVarInfo* pVarInfo, double* pValue, short count = 1);

#define SLAVE_MIDDLE_POS_DIMENSION_MAX             (3)
typedef struct BufLeapFrogInfo
{
    double targetPos;						// 蛙跳目标位置
    double safetyPos;						// 安全工艺位置
    double slaveMiddlePos[SLAVE_MIDDLE_POS_DIMENSION_MAX];				// 从轴中间点位置信息
} TBufLeapFrogInfo;
GT_API GTN_BufLeapFrog(short core, short crd, short axis, TBufLeapFrogInfo* pLeapFrogInfo, short modal, short fifo);
GT_API GTN_BufLeapFrogEx(short core, short crd, short axis, TBufLeapFrogInfo* pLeapFrogInfo, short modal, short fifo = 0);
GT_API GTN_SetCrdBufCommandDelay(short core, short crd, short fifo, short enable);

GT_API GTN_SetCrdStopMode(short core, short crd, short stopMode);
GT_API GTN_GetCrdStopMode(short core, short crd, short* pStopMode);

typedef struct CrdMoveInfo
{
    double synDistance;		// 当前段合成距离
    double movedsynDistance;	// 当前段已运动完成的距离
    double reserve1[10];		// 保留
}TCrdMoveInfo;
GT_API GTN_GetCrdMoveInfo(short core, short crd, TCrdMoveInfo* pCrdMoveInfo, short fifo = 0);

GT_API GTN_LaserFollowMode(short core, short laserChannel, short* pLaserFollowSource, double* pLaserFollowStartPower, short laserCount);
GT_API GTN_LaserFollowRatio(short core, short laserChannel, double* pLaserFollowRatio, double* pLaserFollowMinPower, double* pLaserFollowMaxPower, short laserCount);

GT_API GTN_BufMultiLaserFollowOn(short core, short crd, long laserChannelMask, short fifo);
GT_API GTN_BufMultiLaserFollowOff(short core, short crd, long laserChannelMask, short fifo);
GT_API GTN_BufMultiLaserOn(short core, short crd, long laserChannelMask, short fifo);
GT_API GTN_BufMultiLaserOff(short core, short crd, long laserChannelMask, short fifo);

GT_API GTN_BufMultiLaserFollowOnEx(short core, short crd, long laserChannelMask, short fifo);
GT_API GTN_BufMultiLaserFollowOffEx(short core, short crd, long laserChannelMask, short fifo);
GT_API GTN_BufMultiLaserOnEx(short core, short crd, long laserChannelMask, short fifo);
GT_API GTN_BufMultiLaserOffEx(short core, short crd, long laserChannelMask, short fifo);

GT_API GTN_SetCrdContourErrorControlPrm(short core, short crd, short crdAxis, short enable, double* pCoef, short coefCount);
GT_API GTN_GetCrdContourErrorControlPrm(short core, short crd, short crdAxis, short* pEnable, double* pCoef, short* pCoefCount);

GT_API GTN_SetAxisMotionSmoothParameter(short core, short mode, void* pPrm, long axisMask, TListInfo* pListInfo);
GT_API GTN_GetAxisMotionSmoothParameter(short core, short axis, short* pMode, void* pPrm, short count);

GT_API GTN_SetStopSmoothTime(short core, short profile, short smoothTime);
GT_API GTN_GetStopSmoothTime(short core, short profile, short* pSmoothTime);

GT_API GTN_SetCrossCompMode(short core, short axis, short cycleMode, short revCompCycle);
GT_API GTN_GetCrossCompMode(short core, short axis, short* pCycleMode, short* pRevCompCycle);

GT_API GTN_WriteDigitalOutputPro(short core, TDigitalOutputPro* pDo, TListInfo* pListInfo);
GT_API GTN_SetStopIoPro(short core, short axis, short stopType, TStopIoPrm* pPrm, short count, TListInfo* pListInfo);
GT_API GTN_GetStopIoPro(short core,short axis,short stopType,TStopIoPrm *pPrm,short *pCount);
GT_API GTN_SetStopIoPrm(short core,short axis,short stopType,short inputType,short inputIndex,short mode,short modePrm);

GT_API GTN_InitFPGAEncoder(short core);
GT_API GTN_SetFPGAEncoder(short core, short enable, long intervalTime, short reverseMask);
GT_API GTN_GetFPGAEncoder(short core, long* pDataChn1, long* pDataChn2, long* pDataChn3, long* pDataChn4, long* pCount, short* pOverFlowFlag);

GT_API GTN_SetExtModuleAccessMode(short core, short mode);
GT_API GTN_GetExtModuleAccessMode(short core, short* pMode);

typedef struct PosCompareFollowCrd
{
    short followMode;     //位置比较能量跟随模式,0：脉宽跟随
    short hsoOutputMode;  //hso输出模式，0：不输出，1：按照位置比较指令设置参数输出，2：按照该条指令设置值输出。
    short timeScale;      //时间精度，0：1us，1：0.1us
    short pad;
    double maxValue;
    double minValue;
    double k;
    double b;
}TPosCompareFollowCrd;
GT_API GTN_SetCrdPosCompareFollowPulseWidthPrm(short core, short crd, short posCompareIndex, short hsoIndex, TPosCompareFollowCrd* pPosCompareFollowCrd, short count);
GT_API GTN_CrdPosCompareFollowPulseWidthEnable(short core, short crd, short posCompareIndex, short enable);

typedef struct PosCompareOnOffList
{
    short index;              //位置比较索引号
    short enable;
    short reserve1[2];
    long  reserve2[2];
}TPosCompareOnOffList;
GT_API GTN_PosCompareOnOffList(short core, short group, TPosCompareOnOffList* pPosCompareOnOff, TListInfo* pListInfo);

typedef struct PosComparePsoPrmList
{
    short index;
    short reserve[3];
    unsigned long count;
    long syncPos;
    long time;
} TPosComparePsoPrmList;
GT_API GTN_PosComparePsoPrmList(short core, short group, TPosComparePsoPrmList* pPrm, TListInfo* pListInfo);

GT_API GTN_SetScanAlarmAutoStopMode(short core, short mode, unsigned long errorcodeCheckMask, unsigned long errorcodeCheckRef, short crd);
GT_API GTN_GetScanAlarmAutoStopMode(short core, short* pMode, unsigned long* pErrorcodeCheckMask, unsigned long* pErrorcodeCheckRef, short crd);
GT_API GTN_GetScanErrorCode(short core, unsigned long* pErrorCode, short crd);
GT_API GTN_LoadScanCorrectionTable(short core, short scan, char* pFile, short type);

GT_API GTN_AddTaskPro(short core, short taskType, void* pTaskData, short* pTaskIndex);

typedef struct McVarResInfo
{
    unsigned short resType;         // watch变量的类型
    unsigned short resIndex;        // watch变量的索引
    unsigned short resSubIndex;     // watch变量的子索引
    unsigned short resCount;        // 一次性需要读取watch变量的数量
    double resValue[256];           // 读取到的watch变量的值
} TMcVarResInfo;
GT_API GTN_GetMcVarArray(short core, short resInfoCount, TMcVarResInfo* pResInfo);

typedef struct WatchConditionVar
{
    TWatchVar var;
    unsigned short condition;
    double value;
    double continueTime;
} TWatchConditionVar;

typedef struct WaitForConditionInfo
{
    short conditionDone;
    short modal;
    short reserve1[2];
    long segNum;
    long userTag;
    long reserve2[8];
}TWaitForConditionInfo;

GT_API GTN_WaitForConditionVar(short core, TWatchConditionVar* pWaitConditionVarList, short conditionCount, short operation, TWaitTimeout* pTimeout, TWatchVar* pWaitResultVar, TListInfo* pListInfo);
GT_API GTN_GetWaitForConditionInfo(short core, short list, TWaitForConditionInfo* pWaitForConditionInfo);

GT_API GTN_BufSetLongVar(short core, short crd, short index, long value, short fifo);
GT_API GTN_BufSetDoubleVar(short core, short crd, short index, double value, short fifo);

GT_API GTN_SetGantryModePro(short core, short group, short master, short slave, short mode, long syncErrorLimit);

#define PREDICTION_MODE_AXIS_VELOCITY_TO_FOLLOWERROR                 (0)
#define PREDICTION_MODE_AXIS_VELOCITY_TO_VELOCITY                    (1)
#define PREDICTION_MODE_AXIS_POSITION_TO_POSITION                    (2)
GT_API GTN_SetAxisPrediction(short core, short axis, short mode, short enable, double* pCoef, short count1, short count2);
GT_API GTN_GetAxisPrediction(short core, short axis, short* pMode, short* pEnable, double* pCoef, short* pCount1, short* pCount2);

GT_API GTN_InitializeCommandListConfig(short core, TCommandListConfig* pConfig, short count = 2);

/*-----------------------------------------------------------*/
/* TIMER功能相关                                      */
/*-----------------------------------------------------------*/

#define  START_TIMER_MAX		32

typedef struct StartTimerPrm
{
    short mode;				// 保留，只能为0
    short clear;			// 是否清除初始记录值。0，不清除初始值，1：清除初始值，重新记录初始值
    short reserve[2];
}TStartTimerPrm;

typedef struct TimerInfo
{
    short run;
    short reserve[3];
    double value;
}TTimerInfo;

GT_API GTN_StartTimer(short core, short index, TStartTimerPrm* pStartTimerPrm, short count, TListInfo* pListInfo);
GT_API GTN_StopTimer(short core, short index, short count, TListInfo* pListInfo);
GT_API GTN_GetTimerInfo(short core, short index, TTimerInfo* pInfo, short count);

GT_API GTN_CheckTerminalLicenseRingNet(short core, short station, short fuctionType, long* pInfo);
/*-----------------------------------------------------------*/
/* 网络恢复指令                                              */
/*-----------------------------------------------------------*/
GT_API GTN_RN_Recover(short cardIndex);
/*-----------------------------------------------------------*/
/* GSHD最大最小力矩设置                                      */
/*-----------------------------------------------------------*/
GT_API GTN_SetTorqueLimit(short core, short axis, TTorqueLimit* pTorqueLimit);
GT_API GTN_GetTorqueLimit(short core, short axis, TTorqueLimit* pTorqueLimit);

GT_API GTN_SetServoPosLoopPid(short core, short axis, TServoPosLoopPid* pServoPosLoopPid);
GT_API GTN_GetServoPosLoopPid(short core, short axis, TServoPosLoopPid* pServoPosLoopPid);

typedef struct ServoParamReader
{
    unsigned long objectIndex;   //一级指令字
    unsigned short subIndex;     //二级指令字
    unsigned short reserve;      //保留位
    short byteSize;              //字节尺寸
    short memType;               //对象存储位置 0-RAM  1-FLASH
}TServoParamReader;
GT_API GTN_ReadServoParamInfo(short core, short axis, TServoParamReader* pTServoParamReader, unsigned char* pData);
GT_API GTN_WriteServoParamInfo(short core, short axis, TServoParamReader* pTServoParamReader, unsigned char* pData);
GT_API GTN_RN_GetServoAlarmInfo(short core, short axis, unsigned long* pAlarmCode);

GT_API GTN_WriteServo35thHomingFlag(short core, short axis, short flag);

GT_API GTN_SetServoSpdLoopPid(short core, short axis, TServoSpdLoopPid* pServoSpdLoopPid);
GT_API GTN_GetServoSpdLoopPid(short core, short axis, TServoSpdLoopPid* pServoSpdLoopPid);

GT_API GTN_SetServoPid(short core, short axis, short loop, short mode, void* pServoPid);
GT_API GTN_GetServoPid(short core, short axis, short loop, short mode, void* pServoPid);
GT_API GTN_SetServoPidRatio(short core, short axis, double ratio);
GT_API GTN_GetServoPidRatio(short core, short axis, double* pRatio);

typedef struct ServoNonlinearGlobalGain
{
    double gain1;  // 一段增益
    double gain2;  // 二段增益
    double smoothTime; // 平滑时间，单位ms
    double spdLimitUpper; // 速度上限，单位rpm
    double spdLimitLower; // 速度下限，单位rpm
    double reserved[4]; //保留
}TServoNonlinearGlobalGain;
GT_API GTN_RN_GetServoNonlinearGlobalGain(short core, short axis, short mode, short dataType, TServoNonlinearGlobalGain* pServoPid);
GT_API GTN_RN_SetServoNonlinearGlobalGain(short core, short axis, short mode, short dataType, TServoNonlinearGlobalGain* pServoGlobalGain);


/*-----------------------------------------------------------*/
/* 安全模式设置                                              */
/*-----------------------------------------------------------*/
GT_API GTN_RN_SetStationSafeModeControl(short cardIndex, short stationPhyId, short enable, short clearMode);
GT_API GTN_RN_SetStationSafeModeOut(short cardIndex, short stationPhyId, short type, short index, short* pEnable, double* pValue, short count);

GT_API GTN_SetVelLookAheadModeLa(short core, short crd, int VelMode);  //设速度前瞻模式模式
GT_API GTN_SetProfileModeLa(short core, short crd, short profileMode);


GT_API GTN_BufSetLongVarEx(short core, short crd, short index, long value, short fifo);
GT_API GTN_BufSetDoubleVarEx(short core, short crd, short index, double value, short fifo);


typedef struct PosCompareCommandPos
{
    long segmentNumber;
    long posX;
    long posY;
    short gpo;					// 24V
    short hso;					// 5V
    short reserve[4];			// 确保指令长度为16Word
} TPosCompareCommandPos;

typedef struct PosCompareCommandTime
{
    long segmentNumber;
    long time;
    short pad[2];
    short gpo;
    short hso;
}TPosCompareCommandTime;

typedef union PosComparePredictiondataUnion
{
    TPosCompareCommandPos pos;
    TPosCompareCommandTime time;
    short data16[12];
}TPosComparePredictiondataUnion;

typedef struct PosComparePredictionData
{
    short type;
    short pad[3];
    TPosComparePredictiondataUnion data;
}TPosComparePredictionData;

GT_API GTN_SetPosComparePredictionDataToDsp(short core, short posCompareIndex, short count, TPosComparePredictionData* pData, short* pWriteCount);

/*-----------------------------------------------------------*/
/* 绝对值辅助编码器相关                                      */
/*-----------------------------------------------------------*/
GT_API GTN_SetAuAbsEncMultiTurnRange(short core, short encoder, double range);
GT_API GTN_ReadAuAbsEncPos(short core, short encoder, double* pPos);

GT_API GTN_SetEncryptConfigRingNet(short core, short station, TEncryptConfigPrm* pConfigPrm, short* pSts);
GT_API GTN_GetEncryptDataRingNet(short core, short station, TEncryptData* pReadPrm);
GT_API GTN_SetEncryptDataRingNet(short core, short station, TEncryptData* pWritePrm);

GT_API GTN_ClearPressStatus(short core, short pressAxis);

/*-----------------------------------------------------------*/
/* 椭圆插补指令                                              */
/*-----------------------------------------------------------*/
#define ELLIPSE_AUX_POINT_COUNT                 (5)

#define ELLIPSE_MODE_AUX_POINT                  (0)
#define ELLIPSE_MODE_STANDARD                   (1)

#define ELLIPSE_MODE_AUX_POINT_2D               (0)
#define ELLIPSE_MODE_STANDARD_2D                (1)

typedef struct EllipseAuxPoint
{
    double pos[ELLIPSE_AUX_POINT_COUNT][INTERPOLATION_AXIS_MAX];    // 椭圆上辅助点坐标
}TEllipseAuxPoint;

typedef struct EllipseStandard
{
    double centerPoint[INTERPOLATION_AXIS_MAX];// 椭圆圆心坐标
    double theta;                              // 椭圆旋转角度
    double a;                                  // 椭圆长轴
    double b;                                  // 椭圆短轴
}TEllipseStandard;

typedef union EllipseParameterUnion
{
    TEllipseAuxPoint auxPoint;                 // 辅助点模式参数
    TEllipseStandard standard;                 // 标准模式参数

    double reserve[60];
} TEllipseParameterUnion;

typedef struct EllipseParameter
{
    double endPoint[INTERPOLATION_AXIS_MAX];   // 终点坐标
    short plane;                               // 椭圆平面选择，0：XY；1：YZ；2：ZX
    short dir;                                 // 椭圆方向，0：顺时针；1：逆时针
    short overrideSelect;                      // 速度倍率选择，0：第1组倍率；1：第2组倍率
    short mode;                                // 椭圆模式，目前只支持参数0(辅助点模式)

    TEllipseParameterUnion data;               // 保留参数
} TEllipseParameter;

//-------------------------------------------------------
//功能说明：椭圆插补描述参数,模式：ELLIPSE_MODE_AU_POINT_2D
//plane--------------椭圆平面选择，INTERPOLATION_CIRCLE_PLAT_XY(0)：XY；INTERPOLATION_CIRCLE_PLAT_YZ(1)：YZ；INTERPOLATION_CIRCLE_PLAT_ZX(2)：ZX
//dir----------------椭圆方向，0：顺时针；1：逆时针
//overrideSelect-----速度倍率选择，0：第1组倍率；1：第2组倍率
//pad----------------占位变量，不需要传入
//endPoint1----------终点坐标1,意义根据plane来定，如果palne为XY平面，则endPoint1、pos1为X坐标，endPoint2、pos2为Y坐标
//endPoint2----------终点坐标2
//pos1---------------椭圆上辅助点坐标1
//pos2---------------椭圆上辅助点坐标2
//-------------------------------------------------------
typedef struct EllipseAuxPoint2D
{
    short plane;
    short dir;
    short overrideSelect;
    short pad;

    double endPoint1;
    double endPoint2;

    double pos1[ELLIPSE_AUX_POINT_COUNT];
    double pos2[ELLIPSE_AUX_POINT_COUNT];
} TEllipseAuxPoint2D;

//-------------------------------------------------------
//功能说明：椭圆插补描述参数,模式：ELLIPSE_MODE_STANDARD_2D
//plane--------------椭圆平面选择，INTERPOLATION_CIRCLE_PLAT_XY(0)：XY；INTERPOLATION_CIRCLE_PLAT_YZ(1)：YZ；INTERPOLATION_CIRCLE_PLAT_ZX(2)：ZX
//dir----------------椭圆方向，0：顺时针；1：逆时针
//overrideSelect-----速度倍率选择，0：第1组倍率；1：第2组倍率
//pad----------------占位变量，不需要传入
//endPoint1----------终点坐标1,意义根据plane来定，如果palne为XY平面，则endPoint1为X坐标，endPoint2为Y坐标
//endPoint2----------终点坐标2
//centerPoint1-------椭圆圆心坐标1，意义根据plane来定，如果palne为XY平面，则centerPoint1为X坐标，centerPoint2为Y坐标
//centerPoint2-------椭圆圆心坐标2
//theta--------------椭圆旋转角度，单位：度
//a------------------椭圆长轴
//b------------------椭圆短轴，短轴必须比长轴短
//-------------------------------------------------------
typedef struct EllipseStandard2D
{
    short plane;
    short dir;
    short overrideSelect;
    short pad;

    double endPoint1;
    double endPoint2;

    double centerPoint1;
    double centerPoint2;

    double theta;
    double a;
    double b;
}TEllipseStandard2D;

GT_API GTN_Ellipse(short core, short crd, TEllipseParameter* pEllipse, double synVel, double synAcc, double velEnd, long segNum, short fifo = 0);
GT_API GTN_EllipseEx(short core, short crd, TEllipseParameter* pEllipse, double synVel, double synAcc, double velEnd, long segNum, short fifo = 0);

GT_API GTN_EllipsePro(short core, short crd, short mode, void* pData, double synVel, double synAcc, double velEnd, long segNum, short fifo = 0);
GT_API GTN_EllipseProEx(short core, short crd, short mode, void* pData, double synVel, double synAcc, double velEnd, long segNum, short fifo = 0);


/*-----------------------------------------------------------*/
/* 批处理指令功能        	                                   */
/*-----------------------------------------------------------*/
#define BATCH_COMMAND_END_MODE_DEFAULT                     (0)
#define BATCH_COMMAND_END_MODE_GET_MC_VAR_EX               (1)

typedef struct BatchCommandResult
{
    short commandCount;                // 当前批处理指令总数
    short errorCommandNumber;          // 执行出错的指令序号
    short errorCode;                   // 执行出错的指令错误详细信息（部分指令有详细信息）
    short reserve[5];
}TBatchCommandResult;

GT_API GTN_BatchCommandBegin(short core);
GT_API GTN_BatchCommandEnd(short core, TBatchCommandResult* pResult, short mode, void* pPrm, void* pValue, short count);

/*-----------------------------------------------------------*/
/* 调高器指令                                                */
/*-----------------------------------------------------------*/
// 调高器相关函数
#define MC_HEIGHT_FREQUENCY            (450)
// 设置、读取调高器控制参数
typedef struct HeightControlPrm
{
   short active;                 // 高度控制的使能标志。
   short ctlMode;                // 高度闭环控制还是开环补偿。
   short linkHeightCtlGroup;     // 关联第几组高度控制。
   short feedbackType;           // 高度控制的反馈类型，保留，必须为0,既反馈类型为高度频率信号
   short feedbackIndex;          // 高度控制的反馈类型对应的索引。
   short interval;               // 采样间隔时间，中断周期为单位。
   long errorLimit;              // 跟随误差。
   short reserve[8];             // 保留参数，必须为0。
}THeightControlPrm;

GT_API GTN_SetAxisHeightControlPrm(short core, short axis, THeightControlPrm* pPrm);
GT_API GTN_GetAxisHeightControlPrm(short core, short axis, THeightControlPrm* pPrm);

// 设置、读取调高器差值表
GT_API GTN_SetHeightInterpolationTable(short core, short heightCtlGroup, long n, double* pPosEnc, double* pPosHeight);
GT_API GTN_GetHeightInterpolationTable(short core, short heightCtlGroup, long n, double* pPosEnc, double* pPosHeight, long* pRealCount);

GT_API GTN_SetAxisHeightControlPid(short core, short axis, TPid* pPid);
GT_API GTN_ZerophaseFilter(short core, long dataLen, double inputdata[], double smoothArray[]);

GT_API GTN_GetAxisErrorStatusLink(short core, short axis, short* pLinkAxis, short* pLinkMode, short* pCcount);
GT_API GTN_SetAxisErrorStatusLink(short core, short axis, short* pLinkAxis, short* pLinkMode, short count);

typedef struct HeightCrossPrm
{
    short type;               // 参考类型是编码器位置还是规划位置还是电容高度
    short index;              // 编码器或者规划的索引
    long  crossPos;           // 启动或者关闭跟随功能的穿越高度
    short dir;                // 启动或者关闭的穿越方向
    short pad1[3];
}THeightCrossPrm;

typedef struct HeightFollowPrm
{
    short keepEnable;              // 0:运动到指定高度,1:运动到指定高度后，保持高度跟随
    short pad1[3];
    THeightCrossPrm enableData;    // 自动启动高度跟随参数
    THeightCrossPrm disableData;   // 自动关闭高度跟随参数
}THeightFollowPrm;

#define HEIGHT_DIR_CROSS_POSITIVE            (1)
#define HEIGHT_DIR_CROSS_NEGATIVE            (2)
#define HEIGHT_DIR_CROSS_EQUAL               (3)

#define HEIGHT_FOLLOW_ENABLE_MODE_MANUAL                  (0)
#define HEIGHT_FOLLOW_ENABLE_MODE_AUTO                    (1)
// 自动启动/关闭跟随功能还是应用程序启动或者关闭高度跟随功能
GT_API GTN_SetAxisHeightFollowMode(short core, short axis, short mode, THeightFollowPrm* pPrm);


// 调高器位置跟随功能打开、关闭
GT_API GTN_AxisHeightControlEnable(short core, short axis, short enable, long pos = 10);
// 读取调高器位置
GT_API GTN_GetHeightInterpolationPos(short core, short heightCtlGroup, double* pHeightFrq, double* pHeightPosRaw, double* pInterpolationPos);
// 读取调高器状态信息

typedef struct HeightInf
{
    short ctlMode;         // 当前轴闭环模式，电机编码器闭环，还是调高器频率闭环。
    short errorSts;        // 错误状态，
    double cmpValue;       // 补偿值
}THeightInf;
GT_API GTN_GetAxisHeightInf(short core, short axis, THeightInf* pHeightInf);
// 清除调高器状态
GT_API GTN_ClearAxisHeightSts(short core, short axis);

//FIR滤波器
#define HEIGHT_FILTER_TYPE_FREQUENCY         (1)
#define HEIGHT_FILTER_TYPE_CMP_VALUE         (2)
GT_API GTN_GetAxisHeightFrqFilter(short core, short axis, short type, short* pFilterNum);
GT_API GTN_SetAxisHeightFrqFilter(short core, short axis, short type, short filterNum);


// 标定功能

#define CALIBRATION_MODE_DIFFERENT_STEP      (1)
// 标定stage
#define DIFFERENT_STEP_CALIBRATION_STAGE_NONE                     (-1)//空闲
#define DIFFERENT_STEP_CALIBRATION_STAGE_HOME_SEARCH              (1)//碰板过程
#define DIFFERENT_STEP_CALIBRATION_STAGE_GO_OFFSET                (2)//运动偏置
#define DIFFERENT_STEP_CALIBRATION_STAGE_CALIBRATION_INIT         (3)//开始标定前的初始化
#define DIFFERENT_STEP_CALIBRATION_STAGE_CALIBRATION              (4)//标定过程
#define DIFFERENT_STEP_CALIBRATION_STAGE_CALIBRATION_DONE         (5)//标定完成
#define DIFFERENT_STEP_CALIBRATION_STAGE_CALIBRATION_STOP         (6)//标定停止

#define DIFFERENT_STEP_CALIBRATION_STAGE_HOME_START_ERROR          (100)
#define DIFFERENT_STEP_CALIBRATION_STAGE_HOME_SEARCH_ERROR         (101)
#define DIFFERENT_STEP_CALIBRATION_STAGE_GO_OFFSET_ERROR           (102)
#define DIFFERENT_STEP_CALIBRATION_STAGE_CALIBRATION_ERROR         (103)
#define DIFFERENT_STEP_CALIBRATION_STAGE_NOT_AXIS_ON               (104)
#define DIFFERENT_STEP_CALIBRATION_STAGE_SET_TABLE_ERROR           (105)

// 最多标定点数。
#define DIFFERENT_STEP_CALIBRATION_STEP_MAX                     (4000)

#define CALIBRATION_MODE_DIFFERENT_STEP      (1)
#define CALIBRATION_STEP_MAX         (50)

typedef struct DifferentStepCalibration
{
   short heightCtlGroup;                 // 高度控制的组号，第几路高度反馈输入
   short motorEncoder;                   // 第几路编码器
   short pad[2];                         // 保留必须为0

   double homeVel;                      // 碰板的速度 （脉冲/毫秒）,大于0
   double homeDir;                      // 碰板的运动方向
   long frqMutationStopValue;           // 碰板时频率突变停止阈值,大于0
   long offset;                         // 碰板后反向停靠偏置。从停靠位置开始标定。大于0

   double distance;                     // 标定距离 (脉冲单位）,大于0
   double calibrationVel;               // 标定速度,大于0
   double calibrationStepCount;         // 标定段数,大于0
   double calibrationCountPerStep[CALIBRATION_STEP_MAX]; // 每段标定点数,大于0，总点数最多4000个点。
   double reserve[4];                   // 保留必须为0
}TDifferentStepCalibration;

typedef union HeightCalibration
{
    TDifferentStepCalibration differentStep;
}THeightCalibration;

typedef struct HeightCalibrationPrm
{
    short mode;                     // 目前只能是变步距线性标定，CALIBRATION_MODE_DIFFERENT_STEP
    THeightCalibration prm;
}THeightCalibrationPrm;

GT_API GTN_ClearAxisHeightCalibrationSts(short core, short axis);
GT_API GTN_StopAxisHeightCalibration(short core, short axis);
GT_API GTN_RunAxisHeightCalibration(short core, short axis, short* pStage);
GT_API GTN_SetAxisHeightCalibration(short core, short axis, THeightCalibrationPrm* pPrm);

typedef struct FrqMutationStopPrm
{
    long positiveMutationValue;              // 接近挡板时，调高器频率突变量阈值，正整数。
    short positiveMutationStopLinkLimitType; // 接近挡板时,频率突变停止触发后关联的限位类型。
    long negativeMutationValue;              //远离挡板时，调高器频率突变量阈值，正整数。
    short negativeMutationStopLinkLimitType; // 远离挡板时,频率突变停止触发后关联的限位类型。
}TFrqMutationStopPrm;
GT_API GTN_HeightFrqMutationStopEnable(short core, short axis, short enable, TFrqMutationStopPrm* pMutationStopPrm);
GT_API GTN_GetHeightFrqMutationStopEnable(short core, short axis, short* pEnable, TFrqMutationStopPrm* pMutationStopPrm);

GT_API GTN_GoToAxisFollowHeight(short core, short axis, long followCmd, THeightFollowPrm* pFollowPrm, double trapVel, TTrapPrm* pTrapPrm);

GT_API GTN_GetAxisHeightBand(short core, short axis, long* pBand, long* pTime);
GT_API GTN_SetAxisHeightBand(short core, short axis, long band, long time);

#define HEIGHT_FINDEDGE_LATCH_MAX   4

typedef struct HeightFindEdgePrm
{
    short type;
    short moveAxis;
    short latchNum;
    short reserve1;
    short latchType[HEIGHT_FINDEDGE_LATCH_MAX];
    short latchIndex[HEIGHT_FINDEDGE_LATCH_MAX];
    long  heightDiff;
    long reserve2;
    double findTime;
}THeightFindEdgePrm;

typedef struct HeightFindEdgeResult
{
    short findEdgeSts;
    short error;
    double latchValue[HEIGHT_FINDEDGE_LATCH_MAX];
}THeightFindEdgeResult;

GT_API GTN_SetHeightFindEdgePrm(short core, short heightFbIndex, THeightFindEdgePrm* pFindEdgePrm);
GT_API GTN_GetHeightFindEdgeResult(short core, short heightFbIndex, THeightFindEdgeResult* pFindEdgeResult);
GT_API GTN_GetHeightFindEdgeCoef(short core, short heightFbIndex, double* alpha, double* beta);
GT_API GTN_CloseHeightFindEdge(short core, short heightFbIndex);
GT_API GTN_SetHeightFindEdgeStopMask(short core, short heightFbIndex, long mask, long option);
GT_API GTN_GetHeightFindEdgeStopMask(short core, short heightFbIndex, long* mask, long* option);

typedef struct SetAxisLeapFrogPrm
{
    double vel;         // 蛙跳速度
    double acc;         // 蛙跳加速度
    double dec;         // 蛙跳减速度
    double smoothTime;  // 平滑时间

    double safetyVel;   // 运动到安全工艺高度时的速度

    short slaveType;    // 从轴的类型。
    short slaveIndex;   // 从轴对应的索引。
    long followCmd;     // 跟随高度
    short followEnable; // 运动到跟随高度是否保持。
    short enable;       // 蛙跳使能
    short reserve1[2];
    double reserve2[4];
}TSetAxisLeapFrogPrm;
GT_API GTN_SetAxisLeapFrogPrm(short core, short axis, TSetAxisLeapFrogPrm* pAxisLeapFrogPrm);

GT_API GTN_SetLightPowerCtrlPara(short core, unsigned short ctrlMode, unsigned short* pDutyCycle, unsigned short* pPulseWidth);
GT_API GTN_GetLightPowerCtrlPara(short core, unsigned short* pCtrlMode, unsigned short* pDutyCycle, unsigned short* pPulseWidth);

GT_API GTN_UTL_ConvertPhysicToLogic(TPhysicInfo* pPhysicInfo, TLogicInfo* pLogicInfo);
GT_API GTN_OpenSimulation(short core, double time, double ratio);
GT_API GTN_CloseSimulation(short core);
GT_API GTN_GetSimulationStatus(short core, short* pSts);

//-------------------------------------------------------------------------------------------------
// 路径补偿功能
//-------------------------------------------------------------------------------------------------
// 路径补偿点结构体
typedef struct CompensatePathPoint
{
    double pos[2];                               // x轴、y轴位置
    double compensateValue;	                     // z轴补偿位置
}TCompensatePathPoint;

// 路径补偿表结构体
typedef struct CompensatePathParameter
{
    short tableIndex;                            // 路径补偿表索引
    short axisType[2];                           // 查询路径补偿表X、Y方向位置类型，MC_PROFILE:规划位置 MC_ENCODER:编码器位置
    short axisIndex[2];                          // 查询路径补偿表X、Y方向所使用的轴号
    short reserve1[3];                            // 保留值，必须为0
    double offset[2];                            // x轴、y轴偏移
    double compensateVelMax;                     // 补偿值变化的最大速度 ，单位：pulse/ms，用于（1）结束补偿时，消除补偿值的过程（2）超出补偿的误差带时，补偿值变化的过程
    double approachRadius;                       // 补偿起始点和补偿终点的有效区域半径
    double workBand;                             // 补偿运行时的误差带
    double workHeight;                           // z轴工作高度
    double reserve2[2];							 // 保留值，必须为0
}TCompensatePathParameter;

// 路径补偿状态结构体
typedef struct CompensatePathStatus
{
    short enable;								 // 路径补偿使能标志
    short state;								 // 路径补偿状态
    short errorStatus;                           // 路径补偿是否超出errorBand  0:未超出误差带 1：超出误差带
    short pointIndex;                            // 路径补偿当前点的补偿起点索引
    double compensateStartPos[2];                // 路径补偿当前点的补偿起点位置
    double compensateValue;                      // 路径补偿当前点的补偿量
    double reserve1[2];                          // 保留值，为0
}TCompensatePathStatus;

// 设置路径补偿表
GT_API GTN_SetCompensatePathTable(short core, short tableIndex, TCompensatePathPoint* pCompensatePathPoint, short pointCount);
// 设置路径补偿参数
GT_API GTN_SetCompensatePathParameter(short core, short axis, TCompensatePathParameter* pCompensatePath);
// 读取路径补偿参数
GT_API GTN_GetCompensatePathParameter(short core, short axis, TCompensatePathParameter* pCompensatePath);
// 启动路径补偿
GT_API GTN_StartCompensatePath(short core, short axis, short startMode = 0, void* pParameter = NULL);
// 关闭路径补偿
GT_API GTN_StopCompensatePath(short core, short axis, short stopMode = 0, void* pParameter = NULL);
// 获取路径补偿状态
GT_API GTN_GetCompensatePathStatus(short core, short axis, TCompensatePathStatus* pCompensatePathStatus);

/*-----------------------------------------------------------*/
/* 辅助绝对值编码器						                              */
/*-----------------------------------------------------------*/
//-----------------------------------------------------------------------------------
// 功能说明：配置辅助绝对值编码器的多圈范围，该指令辅助编码器不包含手轮，物理索引接口
// input：cardIndex----卡号，取值范围：[1,16]
// input：stationPhyId----物理站号，取值范围：[0,64]
// input：encoder----辅助编码器在当前站的物理序号，取值范围：[1,8]
// input：range----辅助编码器多圈计数的最大值。例如：编码器多圈是16位的，则range=65536，
//-----------------------------------------------------------------------------------
GT_API GTN_RN_SetAuAbsEncMultiTurnRange(short cardIndex, short stationPhyId, short encoder, double range);

//-----------------------------------------------------------------------------------
// 功能说明：读取辅助绝对值编码器的位置，该指令辅助编码器不包含手轮，物理索引接口
// input：cardIndex----卡号，取值范围：[1,16]
// input：stationPhyId----物理站号，取值范围：[0,64]
// input：encoder----辅助编码器在当前站的物理序号，取值范围：[1,8]
// output：pPos----读取的绝对值辅助编码器的位置值
//-----------------------------------------------------------------------------------
GT_API GTN_RN_ReadAuAbsEncPos(short cardIndex, short stationPhyId, short encoder, double* pPos);

//-----------------------------------------------------------------------------------
// 功能说明：配置绝对值编码器的多圈范围，物理索引接口
// input：cardIndex----卡号，取值范围：[1,16]
// input：stationPhyId----物理站号，取值范围：[0,64]
// input：encoder----编码器在当前站的物理序号，取值范围：[1,8]
// input：range----编码器多圈计数的最大值。例如：编码器多圈是16位的，则range=65536，
//-----------------------------------------------------------------------------------
GT_API GTN_RN_SetAbsEncMultiTurnRange(short cardIndex, short stationPhyId, short encoder, double range);

//-------------------------------------------------------------------------------------------------------
// 功能说明：读取绝对值编码器的位置值，物理索引接口
// input：cardIndex----卡号，取值范围：[1,16]
// input：stationPhyId----物理站号，取值范围：[0,64]
// input：encoder----编码器在当前站的物理序号，取值范围：[1,8]
// output：pPos----读取的绝对值编码器的位置值
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_ReadAbsEncPos(short cardIndex, short stationPhyId, short encoder, double* pPos);

/*-----------------------------------------------------------*/
/* 串行通讯指令（本地485/232通信指令）  (需链接gt_rn.lib，不推荐使用)*/
/*-----------------------------------------------------------*/
GT_API GT_RN_ComOpen(short index);
GT_API GT_RN_ComClose(short index);
GT_API GT_RN_ComRead(short index, unsigned long readLen, unsigned long* pResLen, unsigned char* pData);
GT_API GT_RN_ComWrite(short index, unsigned long writeLen, unsigned long* pResLen, unsigned char* pData);
GT_API GT_RN_ComGetState(short index, unsigned char* pState);
GT_API GT_RN_ComSetSettings(short index, unsigned long baudrate, unsigned char stopBits, unsigned char parity);
GT_API GT_RN_ComClearErr(short index, unsigned char flag);
GT_API GT_RN_ComSetMode(short index, unsigned short comMode);


/*-----------------------------------------------------------*/
/* 串行通讯指令（通用485/232通信指令）(需链接gt_rn.lib，不推荐使用)*/
/*-----------------------------------------------------------*/
GT_API GTN_RN_SerialComOpen(short cardIndex, short stationphyId, short comIndex);
GT_API GTN_RN_SerialComClose(short cardIndex, short stationphyId, short comIndex);
GT_API GTN_RN_SerialComRead(short cardIndex, short stationphyId, short comIndex, unsigned long readLen, unsigned long* pResLen, unsigned char* pData);
GT_API GTN_RN_SerialComWrite(short cardIndex, short stationphyId, short comIndex, unsigned long writeLen, unsigned long* pResLen, unsigned char* pData);
GT_API GTN_RN_SerialComGetState(short cardIndex, short stationphyId, short comIndex, unsigned char* pState);
GT_API GTN_RN_SerialComSetSettings(short cardIndex, short stationphyId, short comIndex, unsigned long baudrate, unsigned char stopBits, unsigned char parity);
GT_API GTN_RN_SerialComClearErr(short cardIndex, short stationphyId, short comIndex, unsigned char flag);
GT_API GTN_RN_SerialComSetMode(short cardIndex, short stationphyId, short comIndex, unsigned short comMode);

//读取网络辅助编码器
GT_API GTN_RN_GetRemotAuEncPos(short cardIndex, short stationPhyId, short auEncIndex, double* pAuEncPos, short anEncCount);

//-------------------------------------------------------------------------------------------------------
//获取轴模块LED显示模式
//GTN_RN_ReadLedDispalyMode(short cardIndex, short stationPhyId, unsigned char *pMode, unsigned char *pRadix)
// mode:灯板数码管显示模式：
//0：工作模式(上电初始状态显示零，轴使能后显示小数点，具体切换方式待定)；
//1：站号模式(显示站号，短暂显示拨码变动)；
//2：报警模式(预留)；
//radix:灯板数码管显示进制： 0：二进制；  1：八进制；  2：十进制；  3：十六进制；
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_ReadLedDispalyMode(short cardIndex, short stationPhyId, unsigned char* pMode, unsigned char* pRadix);
//-------------------------------------------------------------------------------------------------------
//设置轴模块LED显示模式
//GTN_RN_WriteLedDisplayMode(short cardIndex, short stationPhyId, unsigned char mode, unsigned char radix)
// mode:灯板数码管显示模式：
//0：工作模式(上电初始状态显示零，轴使能后显示小数点，具体切换方式待定)；
//1：站号模式(显示站号，短暂显示拨码变动)；
//2：报警模式(预留)；
//radix:灯板数码管显示进制： 0：二进制；  1：八进制；  2：十进制；  3：十六进制；
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_WriteLedDisplayMode(short cardIndex, short stationPhyId, unsigned char mode, unsigned char radix);
//-------------------------------------------------------------------------------------------------------
//导出伺服参数
//GTN_RN_ExportServoParams(short cardIndex, short stationPhyId, short axis, char *pFilePath, char *pFileName)
// cardIndex:从1开始
// stationPhyId:从0开始
// axis:轴号
// pFilePath：模板文件的路径
// pFileName：导出的文件路径及名称
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_ExportServoParams(short cardIndex, short stationPhyId, short axis, char* pFilePath, char* pFileName);
//-------------------------------------------------------------------------------------------------------
//导入伺服参数
//GTN_RN_ImportServoParams(short cardIndex, short stationPhyId, short axis, char *pFilePath, char *pFileName)
// cardIndex:从1开始
// stationPhyId:从0开始
// axis:轴号
// pFilePath：模板文件的路径
// pFileName：导出的文件路径及名称
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_ImportServoParams(short cardIndex, short stationPhyId, short axis, char* pFilePath, char* pFileName);

//获取多轴绝对值编码器
typedef struct AbsEncPos
{
    short stationId;
    short axis;
    unsigned long long multiLines;
    double pos;
}StAbsEncPos;
GT_API GTN_RN_GetMultiAbsEncPos(short cardIndex, StAbsEncPos* pStAbsEncPos, short count);

//-------------------------------------------------------------------------------------------------------
// 设置编码器单圈/多圈位数（目前仅402支持）
// cardIndex：卡号
// stationPhyId：站号
// singleBitNum：单圈绝对值位数，目前支持25位
// multiBitNum：多圈绝对值位数， 目前写0
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_SetAbsEncRange(short cardIndex, short stationPhyId, short singleBitNum, short multiBitNum);

//-------------------------------------------------------------------------------------------------------
// 设置编码器配置信息
// cardIndex：卡号
// stationPhyId：站号
// encoder：物理轴号
// type：配置类型
// 0：编码器单圈线数
// 1：编码器控制状态字
// 2：编码器协议类型
// 3：编码器属性信息
// 4：编码器类型
// 5：正余弦插值倍率
// 6：单圈线数与绝对位置线数比
// pData:写入的数据，详细说明请参考指令说明
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_SetEncoderInfo(short cardIndex, short stationPhyId, short encoder, short type, unsigned long* pData);

//-------------------------------------------------------------------------------------------------------
// 读取编码器配置信息
// cardIndex：卡号
// stationPhyId：站号
// encoder：物理轴号
// type：配置类型
// 0：编码器单圈线数
// 1：编码器控制状态字
// 2：编码器协议类型
// 3：编码器属性信息
// 4：编码器类型
// 5：正余弦插值倍率
// 6：单圈线数与绝对位置线数比
// pData:读取的数据，详细说明请参考指令说明
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_GetEncoderInfo(short cardIndex, short stationPhyId, short encoder, short type, unsigned long* pData);

//-------------------------------------------------------------------------------------------------------
// 初始化PC端DLL Buffer大小
// cardIndex:从1开始
// stationPhyId:从0开始
// bufferAddrWidth：设置DLL Buffer的空间大小
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_ScanCmpInitialWrBuffer(short cardIndex, short stationPhyId, unsigned short bufferAddrWidth);
//-------------------------------------------------------------------------------------------------------
// 获取当前DLL Buffer中的有效数据量和还有多少剩余空间可以写入
// cardIndex:从1开始
// stationPhyId:从0开始
// pValidWordNum：当前DLL Buffer还有多少word空间的数据，没有发送到轴模块
// pRemainsWordSpase: 当前DLL Buffer还有多少word空间的数据，允许用户写入
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_ScanCmpGetWrBufferStatus(short cardIndex, short stationPhyId, unsigned long* pValidWordNum, unsigned long* pRemainsWordSpase);
//-------------------------------------------------------------------------------------------------------
// 将数据写入DLL Buffer中
// cardIndex:从1开始
// stationPhyId:从0开始
// pData：要写入的数据指针
// pWordNumVaild: 期望写入的数据的word的个数
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_ScanCmpWrBufferData(short cardIndex, short stationPhyId, unsigned short* pData, unsigned long wordNum, unsigned long* pWordNumVaild);
//-------------------------------------------------------------------------------------------------------
// 根据轴模块的buffer状态，将DLL Buffer中的数据读取，更新到轴模块缓冲区。
// cardIndex:从1开始
// stationPhyId:从0开始
// dataType：需要压入缓冲区的数据类型: 0：SCAN，振镜相关数据1：CMP，位置比较相关数据
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_ScanCmpUpdateBufferDataToFPGA(short cardIndex, short stationPhyId, short dataType);
//-------------------------------------------------------------------------------------------------------
// 使能轴模块缓冲区，使能后轴模块从指令缓冲区读取指令（只应用于振镜）
// cardIndex:从1开始
// stationPhyId:从0开始
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_ScanEnableFifo(short cardIndex, short stationPhyId);
//-------------------------------------------------------------------------------------------------------
// 关闭轴模块缓冲区，关闭后轴模块暂停从缓冲区读取指令（只应用于振镜）
// cardIndex:从1开始
// stationPhyId:从0开始
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_ScanDisableFifo(short cardIndex, short stationPhyId);
//-------------------------------------------------------------------------------------------------------
// 将驱动器内部参数读取到xml文件中
// cardIndex:		    卡号
// stationPhyId:        站号
// path:                需要写入的xml文件路径+名称
// tpfUpdataProgressPt: 回调函数，用来记录读取参数到xml文件进度
// ptrv:                回调函数tpfUpdataProgressPt第一个参数
// progress:            回调函数tpfUpdataProgressPt第二个参数， 表示进度
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_UploadServoParaFile(short cardIndex, short stationPhyId, char* path, void(*tpfUpdataProgressPt)(void*, short*), void* ptrv, short& progress);
//-------------------------------------------------------------------------------------------------------
// 将xml文件内容写入驱动器
// cardIndex:		    卡号
// stationPhyId:        站号
// path:                需要写入的xml文件路径+名称
// tpfUpdataProgressPt: 回调函数，用来记录读取参数到xml文件进度
// ptrv:                回调函数tpfUpdataProgressPt第一个参数
// progress:            回调函数tpfUpdataProgressPt第二个参数， 表示进度
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_DownloadServoParaFile(short cardIndex, short stationPhyId, char* path, void(*tpfUpdataProgressPt)(void*, short*), void* ptrv, short& progress);
//-------------------------------------------------------------------------------------------------------
// 获取在线升级进度
// cardIndex:		    卡号
// stationPhyId:        站号
// progress:            获取到的进度,取值范围[0..100]
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_UpdateProgress(short cardIndex, short stationPhyId, unsigned short progress);

//获取轴停止详细信息功能
GT_API GTN_GetAxisStopInfo(short core, short axis, long* pInfo, double* pPos, short count = 1);
GT_API GTN_ClearAxisStopInfo(short core, short axis, short count = 1);
GT_API GTN_SetAxisStopInfoMode(short core, short axis, short* pMode, short count);
GT_API GTN_GetAxisStopInfoMode(short core, short axis, short* pMode, short count);

typedef struct TPvtLinePrm
{
    short mode;  //参数生效模式
    short smoothTimer;  //平稳时间
    long moveTime;  //运动时间
    long pad;       //保留位
    double synVel;   //合成目标速度
    double synAcc;  //合成加速度
    double synJeck; //合成加加速度
}TPvtLinePrm;
GT_API GTN_PvtLine(short core, short axisCount, short* moveAxis, long* pos, TPvtLinePrm* pPrm);

GT_API GTN_GetTerminalLinkCount(short core, unsigned short* pTerminalLinkCount);


//-------------------------------------------------------------------------------------------------------
// 设置本地域ID
// cardIndex:		卡号
// stationPhyId:    站号，从0开始
// regionId:        需要写入的本地域ID，取值范围[0..15]（设置之前默认为240）
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_SetLocalHubID(short cardIndex, short stationPhyId, short regionId);

//-------------------------------------------------------------------------------------------------------
// 读取本地域ID
// cardIndex:		卡号
// stationPhyId:    站号，从0开始
// pRegionId:       读取到的本地域ID，取值范围[0..15]
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_GetLocalHubID(short cardIndex, short stationPhyId, short* pRegionId);

//-------------------------------------------------------------------------------------------------------
// 配置网络中hub信息
// cardIndex:		卡号
// stationPhyId:    站号，从0开始
// regionLocalId:   配置本地域ID，取值范围[0..15]
// pRegionID:       配置网络中存在的域ID， 取值范围[0..15]
// regionNums:      配置网络中域的个数，取值范围[0..16]
// comByteSize:     配置域之间的通信字节数(bytes)，取值范围[0..240]
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_SetMultiHubCfgInfo(short cardIndex, short stationPhyId, short regionLocalId, short* pRegionID, short regionNums, short comByteSize);

//-------------------------------------------------------------------------------------------------------
// 读取网络中hub信息
// cardIndex:		卡号
// stationPhyId:    站号，从0开始
// pRegionLocalId:  读取本地域ID，取值范围[0..15]
// pRegionID:       读取网络中存在的域ID， 取值范围[0..15]
// pRegionNums:     读取网络中域的个数，取值范围[0..16]
// pComByteSize:    读取域之间的通信字节数(bytes)，取值范围[0..240]
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_GetMultiHubCfgInfo(short cardIndex, short stationPhyId, short* pRegionLocalId, short* pRegionID, short* pRegionNums, short* pComByteSize);

//-------------------------------------------------------------------------------------------------------
// 写数据到目的域
// cardIndex:		卡号
// stationPhyId:    站号，从0开始
// regionId:        写入的目的域ID，取值范围[0..15]
// pData:           写入的数据
// byteOffset:      写入数据的地址偏置，此值为4的倍数，取值范围[0..240]
// byteNum:         要写入的数据字节数，此值为4的倍数，取值范围[0..240]
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_MultiHubPcComWr(short cardIndex, short stationPhyId, short regionId, unsigned char* pData, unsigned short byteOffset, unsigned short byteNum);

//-------------------------------------------------------------------------------------------------------
// 从目标域读取数据
// cardIndex:		卡号
// stationPhyId:    站号，从0开始
// regionId:        写入的目的域ID，取值范围[0..15]
// pData:           写入的数据
// byteOffset:      写入数据的地址偏置，此值为4的倍数，取值范围[0..240]
// byteNum:         要写入的数据字节数，此值为4的倍数，取值范围[0..240]
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_MultiHubPcComRd(short cardIndex, short stationPhyId, short regionId, unsigned char* pData, unsigned short byteOffset, unsigned short byteNum);

//-------------------------------------------------------------------------------------------------------
// 读取Eeprom数据。
// cardIndex:		卡号
// stationPhyId:    站号，从0开始
// ofst:            需要读取的Eeprom地址
// pValue:          读取到的值
// num:             需要读取的个数
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_ReadEepromData(short cardIndex, short stationPhyId, unsigned short ofst, unsigned char* pValue, unsigned short num);

//-------------------------------------------------------------------------------------------------------
// 写入Eeprom数据。
// cardIndex:		卡号
// stationPhyId:    站号，从0开始
// ofst:            需要写入的Eeprom地址
// pValue:          需要写入的值
// num:             需要写入的个数
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_WriteEepromData(short cardIndex, short stationPhyId, unsigned short ofst, unsigned char* pValue, unsigned short num);

//-------------------------------------------------------------------------------------------------------
// 获取单个资源描述信息
// cardIndex:		卡号
// stationPhyId:    站号，从0开始
// slotInfo:        [15--8]表示板卡类型： 1: 底板；2：子板；3：通讯板 [7--0]表示板卡序号：高八位为1(底板)时：取值范围 1；高八位为2（子板）时：取值范围[1..7]; 高八位为3（通讯板）时：取值范围 1
// mcDef:           资源宏定义，详细说明参考附录I
// pData:           获取到的资源值（16位数值）
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_GetStationSlotResourceEx(short cardIndex, short stationPhyId, short slotInfo, short mcDef, unsigned short* pData);

GT_API GTN_RN_GetStationInfomation(short cardIndex, short stationPhyId, unsigned char infoType, unsigned char infoCount, unsigned long* pInfo);

//-------------------------------------------------------------------------------------------------------
// 500冗余设置心跳，切换主从cpu对Ilink模块控制权；默认从cpu可控，主cpu给模块发心跳时，控制权切换到主cpu
// cardIndex:		卡号
// stationPhyId:    站号，从0开始
// moduleId:        Ilink模块Id，取值范围[0..63]
// data:            写入心跳值，bit0按照0，1翻转，超过10ms不翻转，认为没有心跳发生
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_IlinkSetHeartbeat(short cardIndex, short stationPhyId, short moduleId, unsigned short data);

//-------------------------------------------------------------------------------------------------------
// 500冗余主从CPU通信写
// cardIndex:		卡号
// stationPhyId:    站号，从0开始
// dataCount:       一次通讯写入的数据量个数，取值范围[1..256]dword
// data:            写入的数据
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_IlinkPcComWr(short cardIndex, short stationPhyId, unsigned short dataCount, unsigned long* data);

//-------------------------------------------------------------------------------------------------------
// 500冗余主从CPU通信读
// cardIndex:		卡号
// stationPhyId:    站号，从0开始
// dataCount:       一次通讯读取的数据量个数，取值范围[1..256]dword
// data:            读取到的数据
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_IlinkPcComRd(short cardIndex, short stationPhyId, unsigned short dataCount, unsigned long* data);

//-------------------------------------------------------------------------------------------------------
// 	获取单个卡槽资源描述信息
// cardIndex:		卡号
// stationPhyId:    站号，从0开始
// pData:           获取到的资源描述信息
// dataCount:       需要获取的资源描述信息个数
// pResCount:       实际获取的资源描述信息个数
// slotInfo:        [15--8]表示板卡类型：
//  1: 底板；2：子板；3：通讯板
//	[7--0]表示板卡序号：
//	高八位为1(底板)时：取值范围 1；
//	高八位为2（子板）时：取值范围[1..7];
//  高八位为3（通讯板）时：取值范围 1
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_GetStationSlotResource(short cardIndex, short stationPhyId, unsigned long* pData, short dataCount, short* pResCount, short slotInfo);

GT_API GTN_RN_WriteResourceTable(short cardIndex, unsigned long* pResDef, short count, char* pData, short* pDataSize);
//-------------------------------------------------------------------------------------------------------
// 采样滤波参数设置
// cardIndex:		    卡号
// stationPhyId:        站号
// fltLength:           滑动平均滤波点数，默认值100，计算方式，fltLength = 0.443*采样频率/截止频率
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_SamplingFilterSet(short cardIndex, short stationPhyId, short fltLength);

//-------------------------------------------------------------------------------------------------------
// 采样滤波参数读取
// cardIndex:		    卡号
// stationPhyId:        站号
// fltLength:           读取到的滑动平均滤波点数，默认值100，计算方式，fltLength = 0.443*采样频率/截止频率
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_SamplingFilterGet(short cardIndex, short stationPhyId, short* pFltLength);

//-------------------------------------------------------------------------------------------------------
// PC开卡，打开网络端口
// cardIndex:		    卡号
// callBackFun:         回调函数
// pParamA:             回调参数，可填NULL
//-------------------------------------------------------------------------------------------------------
typedef short (*FunCallBack)(void*, long, void*);
GT_API GTN_RN_InitialUserCallBack(short cardIndex, FunCallBack callBackFun, void* pParamA);

//-------------------------------------------------------------------------------------------------------
// PC通讯写数据
// cardIndex:		    卡号
// phyStationId:        PC通讯时，需要发送对端的ID，如果发送端位PC端，id的取值范围[0,31]，如果发送端为控制器端，id=0xF2
// pData:               回调参数，可填NULL
// byteNum：            PC通信字节数，取值范围[0,240]
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_UserOverRnWrite(short cardIndex, short phyStationId, unsigned char* pData, long byteNum);

//-------------------------------------------------------------------------------------------------------
// 写phy寄存器数值
// cardIndex:		    卡号
// stationPhyId:        站号
// phyIndex:            phy芯片索引，取值范围[1..2]
// phyAddr:             phy芯片地址， 取值0
// regAddr:             phy寄存器地址
// data:               要写入寄存器的数值
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_WritePhyRegValue(short cardIndex, short stationPhyId, short phyIndex, short phyAddr, short regAddr, short data);

//-------------------------------------------------------------------------------------------------------
// 读phy寄存器数值
// cardIndex:		    卡号
// stationPhyId:        站号
// phyIndex:            phy芯片索引，取值范围[1..2]
// phyAddr:             phy芯片地址， 取值0
// regAddr:             phy寄存器地址
// pData:               读取到的寄存器值
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_ReadPhyRegValue(short cardIndex, short stationPhyId, short phyIndex, short phyAddr, short regAddr, short* pData);

//-------------------------------------------------------------------------------------------------------
// 读phy寄存器数值
// cardIndex:		    卡号
// stationPhyId:        站号
// phyIndex:            phy芯片索引，取值范围[1..2]
// pData:               读取到状态值 。数据定义为：
//                      [15]=wr_active/wr_ing(1表示正在写数据)
//                      [12:8]=reg_addr(寄存器地址)
//                      [7]=rd_active/rd_ing(1表示正在读数据)
//                      [4:0]=phy_addr(phy芯片地址)
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_ReadPhySts(short cardIndex, short stationPhyId, short phyIndex, short* pData);

//-------------------------------------------------------------------------------------------------------
// 初始化振镜
// cardIndex:          卡号
// stationPhyId:        站号
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_ScanInitial(short cardIndex, short stationPhyId);

// 描述极耳形状结构体：
typedef struct BatteryTargetSegment
{
    unsigned short type;        //极耳段类型0 : 直线1 : 圆弧
    unsigned short closewise;  //圆弧方向，仅圆弧使用
    unsigned long cutEnable;    //是否切割，0: 不切割；1:切割
    double targetPosX;          //X轴目标位置，单位mm
    double targetPosY;          //Y轴目标位置，单位mm
    double circleCenterX;       //圆心所在X轴坐标，单位mm (仅圆弧使用)
    double circleCenterY;       //圆心所在Y轴坐标，单位mm(仅圆弧使用)
}StBatteryTargetSegment;

#define MAX_SEGMENT_NUM (100)
typedef struct ScanShape
{
    unsigned short segmentNum;//一个极耳的构成段数
    unsigned short cycleNum;//多少个极耳构成一个大循环
    StBatteryTargetSegment batteryTargetSegment[MAX_SEGMENT_NUM];//极耳每一段描述信息
    double segmentIncreaseX[MAX_SEGMENT_NUM]; //单位mm--第n+1个极耳和第n个极耳在每一个插补段的x方向变化量。如果极耳是固定不变，则该值为0。
    double segmentIncreaseY[MAX_SEGMENT_NUM]; //单位mm--第n+1个极耳和第n个极耳在每一个插补段的y方向变化量。如果极耳是固定不变，则该值为0。
}StScanShape;

//-------------------------------------------------------------------------------------------------------
// 描述极耳形状
// cardIndex:          卡号
// stationPhyId:        站号
// pParam:             描述极耳形状结构体
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_ScanBatteryShape(short cardIndex, short stationPhyId, StScanShape* pParam);

typedef struct ScanBatteryParam
{
    long encAxisSrc; //主轴编码器在模块的哪一个轴上，0：X轴；1：Y轴。
    long prfEncSel;//计数源选择，0：使用编码器作为计数源；1：使用规划器作为计数源。
    double encPulsePreDistance;  //实际极耳mm对应主轴编码器脉冲数(pulse/mm),编码器脉冲为倍频后的。
    double scanPulsePreDistance;  //实际极耳mm对应振镜脉冲数（bit/mm）
} StScanBatteryParam;

//-------------------------------------------------------------------------------------------------------
// 设置振镜相关参数
// cardIndex:          卡号
// stationPhyId:        站号
// pParam:             描述极耳形状结构体
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_SetScanBatteryParam(short cardIndex, short stationPhyId, StScanBatteryParam* pParam);

//-------------------------------------------------------------------------------------------------------
// 获取振镜相关参数
// cardIndex:          卡号
// stationPhyId:        站号
// pParam:             描述极耳形状结构体
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_GetScanBatteryParam(short cardIndex, short stationPhyId, StScanBatteryParam* pParam);

//振镜属性参数，用于解决振镜安装反或编码器反等问题：
typedef struct ScanAttributeCfg
{
    unsigned short followMode : 1;    //跟随模式：0：振镜按照设定速度运动；1：振镜按照跟随轴速度运动
    unsigned short followAxisSel : 1; //振镜跟随轴选择。0：X轴； 1：Y轴
    unsigned short reserveBit2 : 1;
    unsigned short fdbackReverse : 1;  //材料切割的合成运动从起始点开始运动的方向是X轴是正向还是负向。1：负向。0：正向。
    unsigned short axisXReverse : 1;  //经过axi_src_sel和axi_x_sel选择的X轴否和实际物理方向相反。：取反。：不取反。
    unsigned short axisYReverse : 1;  //经过axi_src_sel和axi_x_sel选择的Y轴否和实际物理方向相反。：取反。：不取反。
    unsigned short reserveBit7to6 : 2;
    unsigned short scanXReverse : 1; //振镜的X轴安装是否和实际物理方向相反。1：取反。0：不取反。
    unsigned short scanYReverse : 1; //振镜的Y轴安装是否和实际物理方向相反。1：取反。0：不取反。
    unsigned short scanZReverse : 1; //振镜的Z轴安装是否和实际物理方向相反。1：取反。0：不取反。
    unsigned short scanXYSwitch : 1; //振镜的XY输出是否要交换。1：交换。0：不交换。
    unsigned short scanBXReverse : 1;  //附属振镜的X轴安装是否和实际物理方向相反。1：取反。0：不取反。
    unsigned short scanBYReverse : 1;  //附属振镜的Y轴安装是否和实际物理方向相反。1：取反。0：不取反。
    unsigned short scanBZReverse : 1;  //附属振镜的Z轴安装是否和实际物理方向相反。1：取反。0：不取反。
    unsigned short scanBXYSwitch : 1;  //附属振镜的XY输出是否要交换。1：交换。0：不交换。
}StScanAttributeCfg;

//-------------------------------------------------------------------------------------------------------
// 设置振镜属性
// cardIndex:          卡号
// stationPhyId:        站号
// pParam:             振镜属性参数
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_SetScanAttribute(short cardIndex, short stationPhyId, StScanAttributeCfg* pParam);

//-------------------------------------------------------------------------------------------------------
// 获取振镜属性
// cardIndex:          卡号
// stationPhyId:        站号
// pParam:             振镜属性参数
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_GetScanAttribute(short cardIndex, short stationPhyId, StScanAttributeCfg* pParam);

//-------------------------------------------------------------------------------------------------------
// 描述极耳形状
// cardIndex:          卡号
// stationPhyId:        站号
// type:               振镜轨迹类型： 1：三角形  2：八字形
// pParam:             保留，写NULL
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_ScanShapeType(short cardIndex, short stationPhyId, unsigned long type, StScanShape* pParam = NULL);

//-------------------------------------------------------------------------------------------------------
// 设置振镜工作模式
// cardIndex:          卡号
// stationPhyId:        站号
// mode:               0：空闲模式，暂停当前正在执行的程序 1：运行模式，程序处于工作状态 2：复位模式，复位振镜所有工作状态。
// pParam:             保留，写NULL
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_GetScanBatteryMode(short cardIndex, short stationId, unsigned long* pMode, void* pParam = NULL);

//-------------------------------------------------------------------------------------------------------
// 读取振镜工作模式
// cardIndex:          卡号
// stationPhyId:        站号
// mode:               0：空闲模式，暂停当前正在执行的程序 1：运行模式，程序处于工作状态 2：复位模式，复位振镜所有工作状态。
// pParam:             保留，写NULL
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_SetScanBatteryMode(short cardIndex, short stationPhyId, unsigned long mode, void* pParam = NULL);

//-------------------------------------------------------------------------------------------------------
// 设置驱动器的分辨率
// cardIndex:          卡号
// stationPhyId:       站号
// axis:               轴号
// resolution:         电机转n圈对应多少个脉冲
// n:                  电机转的圈数
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_WriteFollowResolution(short cardIndex, short stationPhyId, short axis, unsigned long resolution, unsigned short n);

//-------------------------------------------------------------------------------------------------------
// 设置是否开启全闭环
// cardIndex:          卡号
// stationPhyId:       站号
// axis:               轴号
// en:                 是否开启全闭环，0：关闭全闭环，1：开启全闭环
// assoPosErr:         是否关联跟随误差，0：不关联跟随
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_EnablePosCloseLoop(short cardIndex, short stationPhyId, short axis, bool en, bool assoPosErr);



/*------------------------------------------------------------------------*/
/*------------------------------------------------------------------------*/
/*                        EtherCAT网络协议功能函数		                    */
/*------------------------------------------------------------------------*/
/*------------------------------------------------------------------------*/

//EtherCAT
#define ECAT_STATE_OP    (2)

#define ECAT_AXIS_MAX		12
#define ECAT_MODE_HOMING	6
#define ECAT_MODE_CSP		8
#define ECAT_MODE_CSV		9
#define ECAT_MODE_CST		10
#define ECAT_MODE_PV		3
#define ECAT_MODE_PT		4

#define ECAT_PROBE_TRIGGER_TYPE_FIRST_EVENT	0
#define ECAT_PROBE_TRIGGER_TYPE_CONTINUES	1

#define ECAT_PROBE_TRIGGER_LEVEL_POS		1
#define ECAT_PROBE_TRIGGER_LEVEL_NEG		-1
#define ECAT_PROBE_TRIGGER_LEVEL_DUL		0

#define  ECAT_ACT_ENC_POS	0
#define  ECAT_ACT_ENC_VEL	1
#define  ECAT_ACT_CURRENT	2
#define  ECAT_ACT_TORQUE	3
#define  ECAT_ACT_CT		4

#define ERROR_CODE_NO_SLAVE		-10
#define ERROR_CODE_NO_ENI		-9
#define ERROR_CODE_CONFIG_FAIL	-8
#define ERROR_CODE_ENI_ERROR	-1

typedef struct SlaveInfo
{
    int32_t slave_cnt;
    int32_t slave_type;
    int32_t motion_cnt;
    int32_t io_nmap;
    int32_t io_length;
    uint32_t Vid;
    uint32_t Pid;
    int32_t io_type;             //1 输入  2 输出  3 输入输出
    uint32_t revision_number;
    uint32_t serial_number;
    int32_t dump[8];
    uint16_t alias;
} TSlaveInfo;

typedef struct EcatErrInfo
{
    short dcError;
    short workingCountErrorCnt; //发现断线计数
    short ecatCommStatus;      //总线通讯状态
    short workingCount;        //当前WorkingCount值
    short offlineFlag;         //断线标志
    short rootOfflineFlag;    //根部断线标志
    short workingCountFirstError;  //发现断线时的计数
    short profileTimeOutFlag;      //规划超时标志
    short dump[20];
} TEcatErrInfo;

//EtherCAT interface
// 设置Home回零参数，如果指针为空，则不需要发sdo指令 [Thunder-2020-9-24]
GT_API GTN_SetEcatHomingPrmPro( short core, short axis,short method,double speed1,double speed2,double acc,long offset,unsigned short probeFunction,unsigned short mask);
GT_API GTN_SetEcatHomingPrmEx(short core, short axis, short* psMethod, double* pdSpeed1, double* pdSpeed2, double* pdAcc, long* plOffset, unsigned short* pusProbeFunction);
GT_API GTN_SetEcatHomingPrm(short core, short axis, short method, double speed1, double speed2, double acc, long offset, unsigned short probeFunction);
GT_API GTN_StartEcatHoming(short core, short axis);
GT_API GTN_SetHomingMode(short core, short axis, short mode);
GT_API GTN_GetEcatHomingStatus(short core, short axis, unsigned short* homingStatus);
GT_API GTN_StopEcatHoming(short core, short axis);
GT_API GTN_SetTouchProbeFunction(short core, short axis, short ProbePrm);
GT_API GTN_SetTouchProbeFunctionEx(short core, short axis, short Probe1Enable, short Probe1TriggerType, short Probe1TriggerLevel, short Probe2Enable, short Probe2TriggerType, short Probe2TriggerLevel);
GT_API GTN_GetTouchProbeStatus(short core, short axis, unsigned short* probeStatus, long* probe1PosValue, long* probe1NegValue, long* probe2PosValue, long* probe2NegValue);
GT_API GTN_SetEcatGpioConfig(short core, short effectiveLevel, short direction);
GT_API GTN_SetEcatAxisOnThreshold(short core, short axis, unsigned short threshold);
GT_API GTN_GetEcatAxisOnThreshold(short core, short axis, unsigned short* threshold);
GT_API GTN_InitEcatComm(short core);
GT_API GTN_InitEcatCommEx(short core, char* eniFilePath);
GT_API GTN_StartEcatComm(short core);
GT_API GTN_IsEcatReady(short core, short* pStatus);
GT_API GTN_TerminateEcatComm(short core);
GT_API GTN_GetEcatErrorCode(short core, short axis, unsigned short* pErrorCode);
GT_API GTN_GetDcError(short core, short* pError);
GT_API GTN_GetEcatDcErrorEx(short core, TEcatErrInfo* pEcatErrorInfo);
GT_API GTN_SetEcatAxisMode(short core, short axis, short mode);
GT_API GTN_GetEcatAxisMode(short core, short axis, unsigned short* drvMode);
GT_API GTN_SetEcatAxisPV(short core, short axis, long velocity);
GT_API GTN_SetEcatAxisPT(short core, short axis, short torque);
GT_API GTN_GetEcatEncPos(short core, short axis, long* pEncPos);
GT_API GTN_GetEcatAuxEncPos(short core, short axis, long* pAuxEncPos);
GT_API GTN_GetEcatEncVel(short core, short axis, long* pVelocity);
GT_API GTN_GetEcatAxisAtlCurrent(short core, short axis, short* pCur);
GT_API GTN_GetEcatAxisAtlTorque(short core, short axis, short* pTorque);
GT_API GTN_GetEcatAxisAV(short core, short axis, long* pVel);
GT_API GTN_GetEcatAxisACT(short core, short axis, short* pCur, short* pTorque);
GT_API GTN_GetEcatMCType(short core, short* pType);
GT_API GTN_SetMcEcatAxisNum(short core, short Num);
GT_API GTN_GetMcEcatAxisNum(short core, short* pNum);
GT_API GTN_SetPosScale(short core, short axis, unsigned short posScale);
GT_API GTN_GetPosScale(short core, short axis, unsigned short* posScale);
GT_API GTN_GetEcatAxisAI(short core, short axis, short channel, short* pValue);
GT_API GTN_SetEcatAxisAO(short core, short axis, unsigned short AoValue);
GT_API GTN_GetEcatAxisAO(short core, short axis, unsigned short* pAo);
GT_API GTN_GetEcatAxisDI(short core, short axis, unsigned long* pDi);
GT_API GTN_GetEcatAxisDIEx(short core, short axis, unsigned long* pDi, short count);
GT_API GTN_SetEcatAxisDO(short core, short axis, unsigned long DoValue);
GT_API GTN_GetEcatAxisDO(short core, short axis, unsigned long* pDo);
GT_API GTN_SetEcatAxisDOBit(short core, short axis, short bitOffset, char DoBitValue);
GT_API GTN_GetEcatAxisDOBit(short core, short axis, short bitOffset, char* pDoBitValue);
GT_API GTN_GetEcatSlaves(short core, short* pSlaveMotionCnt, short* pSlaveIOCnt);
GT_API GTN_GetEcatSlaveInfo(short core, short slaveIndex, TSlaveInfo* pSlaveinfo);
GT_API GTN_GetEcatAxisPE(short core, short axis, long* pPosErr);
GT_API GTN_SetEcatRawData(short core, unsigned short offset, unsigned short nByteSize, unsigned char* pValue);
GT_API GTN_GetEcatRawData(short core, unsigned short offset, unsigned short nByteSize, unsigned char* pValue);
GT_API GTN_SetEcatAxisTorqueOffset(short core, short axis, short torqueOffset);
GT_API GTN_GetEcatAxisTorqueOffset(short core, short axis, short* pTorqueOffset);
GT_API GTN_GetEcatPdoLength(short core, short* pPdoLen);
GT_API GTN_EcatSDODownload(short core, unsigned short slave_position, unsigned short index, unsigned char subindex, unsigned char* data, unsigned int data_size, unsigned int* abort_code);
GT_API GTN_EcatSDOUpload(short core, unsigned short slave_position, unsigned short index, unsigned char subindex, unsigned char* target, unsigned int target_size, unsigned int* result_size, unsigned int* abort_code);
GT_API GTN_EcatSDODownloadEx(short core, unsigned short axisNumber, unsigned short index, unsigned char subindex, unsigned char* data, unsigned int data_size, unsigned int* abort_code);
GT_API GTN_EcatSDOUploadEx(short core, unsigned short axisNumber, unsigned short index, unsigned char subindex, unsigned char* target, unsigned int target_size, unsigned int* result_size, unsigned int* abort_code);
GT_API GTN_EcatIOReadInput(short core, unsigned short slaveno, unsigned short offset, unsigned short nSize, unsigned char* pValue);
GT_API GTN_EcatIOReadOutput(short core, unsigned short slaveno, unsigned short offset, unsigned short nSize, unsigned char* pValue);
GT_API GTN_EcatIOWriteOutput(short core, unsigned short slaveno, unsigned short offset, unsigned short nSize, unsigned char* pValue);
GT_API GTN_EcatIOBitReadInput(short core,unsigned short slaveno,unsigned short offset,unsigned short Index,unsigned char *pValue);
GT_API GTN_EcatIOBitReadOutput(short core,unsigned short slaveno,unsigned short offset,unsigned short Index,unsigned char *pValue);
GT_API GTN_EcatIOBitWriteOutput(short core, unsigned short slaveno, unsigned short offset, short Index, unsigned char value);
GT_API GTN_EcatIOSynch(short core);
GT_API GTN_GetEcatAxisPdoData(short core, short axis, unsigned short object, unsigned char* pValue);

typedef struct EcatInitPrm
{
    unsigned short skip_count;
    unsigned short netOpenSts;
    unsigned short* slave_position;
    char* eniFilePath;
    unsigned short reserve2[20];
}TEcatInitPrm;

GT_API GTN_InitEcatComm_MultiTask(short core, TEcatInitPrm* pPrm);
GT_API GTN_InitEcatComm_PhysicalID(short core);

// 新增在CSV模式下，目标速度60ff的最大值，单位驱动器60ff单位[Thunder-2019-10-11]
GT_API GTN_SetEcatAxisMaxTargetVel(short core, short axis, unsigned long targetMaxVel);
GT_API GTN_GetEcatAxisMaxTargetVel(short core, short axis, unsigned long* targetMaxVel);

//厂商自定义了对象字0x6040的高8位时，将高8位的值填入指令，内部和低8位位或运算
GT_API GTN_SetEcatAxisCtrlwordEx(short core, short axis, unsigned short ctrlex);
GT_API GTN_GetEcatAxisCtrlwordEx(short core, short axis, unsigned short* ctrlex);

GT_API GTN_SetEcatAxisLmtHomeIndex(short core,short axis,short index_LimitN,short index_LimitP,short index_Home);
GT_API GTN_GetEcatAxisLmtHomeIndex(short core,short axis,short *index_LimitN,short *index_LimitP,short *index_Home);
GT_API GTN_RelateEcatIOSlaveToMCIOModule(short core, short slaveindex, short IOModuleindex);
GT_API GTN_SetMCIOModuleValue(short core, short IOModuleindex, short iomapindex, long* val, short cnt);
GT_API GTN_GetMCIOModuleValue(short core, short IOModuleindex, short iomapindex, long* val, short cnt);

// 新增设置占位轴的指令
GT_API GTN_InitMcAxisGap(short core);
GT_API GTN_SetMcAxisGap(short core,short axis,short count);
GT_API GTN_GetMcAxisGap(short core,short axis,short *actualAxisNo,short count);
GT_API GTN_RelateEcatSlaveToMcAuEncoder(short core, short auenc, short ecatAxisIndex);
GT_API GTN_RelateEcatSlaveToMcMpgEncoder(short core, short mpg, short ecatAxisIndex);
GT_API GTN_RelateEcatSlaveToMcAuEncoderEx(short core, short auenc, short ecatIndex, short ecatType, short pdoOffset, short pdoByteLength);
GT_API GTN_RelateEcatSlaveToMcMpgDi(short core, short mpg, short ecatAxisIndex, short bitoffset);
GT_API GTN_RelateEcatSlaveToMcMpgDiEx(short core, short mpg, short ecatIndex, short ecatType, short bitoffset, short pdoOffset);
GT_API GTN_RelateEcSlvToMcAuEnc(short core,short auenc,short ecatAxisIndex);         //CPAC函数名称过长
GT_API GTN_RelateEcSlvToMcMpgEnc(short core,short mpg,short ecatAxisIndex);
GT_API GTN_RelateEcatSlaveToMcMpgEncoderEx(short core, short mpg, short ecatIndex, short ecatType, short pdoOffset, short pdoByteLength);
GT_API GTN_RelateEcatSlaveToMcAdc(short core, short adc, short ecatIndex, short ecatType, short pdoOffset, short pdoByteLength);
GT_API GTN_RelateEcatSlaveToMcAuAdc(short core, short auadc, short ecatIndex, short ecatType, short pdoOffset, short pdoByteLength);
GT_API GTN_RelateEcatSlaveToMcGpiBit(short core, short gpi, short ecatIndex, short ecatType, short bitoffset, short pdoOffset);
GT_API GTN_RelateEcatSlaveToMcGpoBit(short core, short gpo, short ecatIndex, short ecatType, short bitoffset, short pdoOffset);
GT_API GTN_LoadRelateEcatSlaveConfig(short core, char* pFile);
GT_API GTN_RelateEcatSlaveToMcDac(short core, short dac, short ecatIndex, short ecatType, short pdoOffset, short pdoByteLength);
GT_API GTN_RelateEcatSlaveToMcAuDac(short core, short auDac, short ecatIndex, short ecatType, short pdoOffset, short pdoByteLength);
GT_API GTN_GetMcRelateEcatSlaveInfo(short core, short mcType, short index, short* pEcatIndex, short* pEcatType, short* pPdoOffset, short* pBitoffset, short* pdPoByteLength);
GT_API GTN_GetAxisModuleInfo(short core, short axis, short* pModule, short* pSubAxis);

typedef void(*TCallbackFunction)(const short data[512], void* pUserDataBack);
GT_API GTN_RegisterCallbackFunction(short core, TCallbackFunction callbackFunction, void* pUserData = NULL);
GT_API GTN_RN_ReadEEPROM(short cardIndex, short stationPhyId, short axis, unsigned short ofst, unsigned char* pValue, unsigned short num);
GT_API GTN_RN_WriteEEPROM(short cardIndex, short stationPhyId, short axis, unsigned short ofst, unsigned char* pValue, unsigned short num);
GT_API GTN_RN_ResetServoDSP(short cardIndex, short stationPhyId, short axis);
GT_API GTN_ResetServoDSP(short core, short axis);
GT_API GTN_RN_WriteSetId(short cardIndex, short stationPhyId, short setId);
GT_API GTN_RN_ReadSetId(short cardIndex, short stationPhyId, short* pSetId);
GT_API GTN_RN_DisableSetId(short cardIndex, short stationPhyId);
GT_API GTN_RN_FORWrite(short cardIndex, short stationPhyId, short* pData, unsigned long dataCount, unsigned long* pDataCount, char* pFileName);
GT_API GTN_RN_SetStationSlotResourceEx(short cardIndex, short stationPhyId, char* pFileName, short slotInfo);
GT_API GTN_GetCoreInfo(short cardIndex, short* pCoreIndex, short* pCoreCount, short* pCoreValidFlag, short cardCount);
typedef struct ResTypeMapInfo
{
    short resType;//实际资源类型
    short resIndex;//实际资源序号
    //short mapType;
    short mapIndex;
    short mapCount;
} TResTypeMapInfo;
typedef struct TerminalResMapInfo
{
    short resMapType;//逻辑资源类型
    short resMapCount;//逻辑资源个数
    TResTypeMapInfo resmapInfo[64];//每种资源的最大resMap表格
} TTerminalResMapInfo;
typedef struct TerminalInfo
{
    short netType;
    unsigned short terminalType;
    unsigned short terminalSubType;
    short phyId;
    short slotCount;
    unsigned short slotType[16];
    unsigned short slotSubType[16];//当前站每个槽上所插子板的子类型，没插默认为0
    short stationResMapCount;
    TTerminalResMapInfo terminalResMapInfo[64];//每个站的最大资源类型
} TTerminalInfo;
GT_API GTN_GetTerminalInfo(short core, short index, TTerminalInfo* pTerminalInfo);

/*-----------------------------------------------------------*/
/* RemoteChannel                                             */
/*-----------------------------------------------------------*/

#define RING_NET_PDU_FAST_LINK_TYPE_NONE                   (0)
#define RING_NET_PDU_FAST_LINK_TYPE_REMOTE_AU_ENCODER      (1)
#define RING_NET_PDU_FAST_LINK_TYPE_POS_COMPARE_LINEAR     (2)

typedef struct RemoteChannelPrm
{
    short stationPhyId;
    short index;
    short type;
} TRemoteChannelPrm;

typedef struct RemoteChannelInfo
{
    short enableFlag;
    short occupy;
    short type;
    short index;
    short lastReceiveError;
    short lastSendError;
    unsigned short errorCount;
    unsigned short continueErrorCount;
} TRemoteChannelInfo;

GT_API GTN_GetRemoteChannelResult(short core, double* pValue, short channelIndex, short channelCount);
GT_API GTN_GetRemoteChannelInfo(short core, TRemoteChannelInfo* pInfo, short channelIndex, short channelCount);
GT_API GTN_GetRemoteChannelPrm(short core, TRemoteChannelPrm* pPrm, short channelIndex, short channelCount);
GT_API GTN_SetRemoteChannelPrm(short core, TRemoteChannelPrm* pPrm, short* pChannelIndex, short channelCount);
GT_API GTN_ClearRemoteChannel(short core,short channelIndex,short channelCount);
//---------------------------------------------------------
// LaserPro
//---------------------------------------------------------

// 激光打开模式
#define LASER_ON_MODE_DEFAULT                    (0)

// 激光关闭模式
#define LASER_OFF_MODE_POWER_OFF                 (0)  // 激光关闭时，能量也关闭
#define LASER_OFF_MODE_POWER_HOLD                (1)  // 激光关闭时，能量保持最后输出的值

// 激光能量跟随模式
#define LASER_FOLLOW_MODE_NONE                   (0)  // 未设置能量跟随
#define LASER_FOLLOW_MODE_RATIO                  (1)  // 按比例系数进行能量跟随
#define LASER_FOLLOW_MODE_TABLE                  (2)  // 按能量跟随表进行能量跟随
#define LASER_FOLLOW_MODE_DUO_TABLE              (3)  // 双表能量跟随

#define LASER_FOLLOW_POWER_TYPE_DUTY             (0)
#define LASER_FOLLOW_POWER_TYPE_FREQUENCY        (1)
#define LASER_FOLLOW_POWER_TYPE_PULSE_WIDTH      (2)
#define LASER_FOLLOW_POWER_TYPE_VOLTAGE          (3)
#define LASER_FOLLOW_POWER_TYPE_PARALLEL         (4)

// 激光能量跟随合成速度源
#define LASER_FOLLOW_SYNCH_VEL_SOURCE_PROFILE              (0)
#define LASER_FOLLOW_SYNCH_VEL_SOURCE_ENCODER              (1)

typedef struct LaserPowerPro
{
    short laserOn;                               // 激光开关光信号
    short laserEnable;                           // 激光器预使能信号
    short laserRed;                              // 激光器红灯指示信号
    short laserPowerLatch;                       // 激光器功率锁存信号
    double duty;                                 // 当前PWM的占空比值，单位：%
    double frequency;                            // 当前PWM的频率值，单位：kHz
    double pulseWidth;                           // 当前PWM的脉宽值，单位：us
    double voltage;                              // 当前激光模拟量值，单位：V
    double parallel;                             // 当前激光并口能量值
}TLaserPowerPro;

typedef struct LaserPro
{
    short laserOn;                               // 激光开关光信号
    short laserEnable;                           // 激光器预使能信号
    short laserRed;                              // 激光器红灯指示信号
    short laserPowerLatch;                       // 激光器功率锁存信号
    double duty;                                 // 当前PWM的占空比值，单位：%
    double frequency;                            // 当前PWM的频率值，单位：kHz
    double pulseWidth;                           // 当前PWM的脉宽值，单位：us
    double voltage;                              // 当前激光模拟量值，单位：V
    double parallel;                             // 当前激光并口能量值

    double minDuty;                              // 占空比能量限制最小值，取值范围：[0,100]，单位：%
    double maxDuty;                              // 占空比能量限制最大值，取值范围：[0,100]，单位：%
    double minFrequency;                         // 频率能量限制最小值，取值范围：[0,1562.5]：单位：kHz
    double maxFrequency;                         // 频率能量限制最大值，取值范围：[0,1562.5]：单位：kHz
    double minPulseWidth;                        // 脉宽能量限制最小值，取值范围：[0,65535]，单位：us
    double maxPulseWidth;                        // 脉宽能量限制最大值，取值范围：[0,65535]，单位：us
    double minVoltage;                           // 激光模拟量电压最小值，取值范围：[0,10]，单位：V
    double maxVoltage;                           // 激光模拟量电压最大值，取值范围：[0,10]，单位：V
    double minParallel;                          // 激光并口能量最小值，取值范围：[0,255]
    double maxParallel;                          // 激光并口能量最大值，取值范围：[0,255]

    double laserOnDelay;                         // 激光开光延时时间，单位：us
    double laserOffDelay;                        // 激光关光延时时间，单位：us
}TLaserPro;

typedef struct LaserFollowPrmPro
{
    short powerType;                             // 激光能量跟随的能量信号类型：0：占空比，1：频率，2：脉宽，3：模拟量，4：并口
    short group;                                 // 激光能量跟随的group号
    short source;                                // 激光能量跟随合成速度源，0：规划器合成速度，1：编码器合成速度
    short coordSystem;                           // 激光能量跟随合成速度的坐标系
    double ratio;                                // 激光能量跟随比例系数
    double minPower;                             // 激光能量跟随能量最小限制值
    double maxPower;                             // 激光能量跟随能量最大限制值
}TLaserFollowPrmPro;

typedef struct LaserFollowTablePrmPro
{
    short powerType;                             // 激光能量跟随的能量信号类型：0：占空比，1：频率，2：脉宽，3：模拟量，4：并口
    short group;                                 // 激光能量跟随的group号
    short source;                                // 激光能量跟随合成速度源，0：规划器合成速度，1：编码器合成速度
    short coordSystem;                           // 激光能量跟随合成速度的坐标系
    short tableId;                               // 激光能量跟随表的表号，取值范围：[1,2]
    short pad[3];
    double minPower;                             // 激光能量跟随能量最小限制值
    double maxPower;                             // 激光能量跟随能量最大限制值
}TLaserFollowTablePrmPro;

typedef struct LaserFollowDuoTablePrmPro
{
    short group;                                 // 激光能量跟随的group号
    short source;                                // 激光能量跟随合成速度源，0：规划器合成速度，1：编码器合成速度
    short coordSystem;                           // 激光能量跟随合成速度的坐标系
    short dutyTableId;                           // 激光能量跟随占空比表的表号，取值范围：[1,2]
    short frequencyTableId;                      // 激光能量跟随频率表的表号，取值范围：[1,2]
    short pad[3];
    double minDuty;                              // 激光能量跟随占空比最小限制值，取值范围：[0,100]，单位：%
    double maxDuty;                              // 激光能量跟随占空比最大限制值，取值范围：[0,100]，单位：%
    double minFrequency;                         // 激光能量跟随频率最小限制值，取值范围：[0,1562.5]，单位：kHz
    double maxFrequency;                         // 激光能量跟随频率最大限制值，取值范围：[0,1562.5]，单位：kHz
}TLaserFollowDuoTablePrmPro;

typedef struct LaserFollowPro
{
    short enable;                                // 使能
    short mode;                                  // 激光能量跟随模式
    short errorCode;                             // 能量跟随整个功能的错误信息，主要标识出错的地方
    short returnValue;                           // 能量跟随整个功能出错地方的返回值

    double power;                                // 当前周期实际输出能量值
    double powerAnother;                         // 同时调整两种能量时的另一种能量值

    TLaserFollowPrmPro ratioPrm;                 // 当mode=LASER_FOLLOW_MODE_RATIO时对应的参数
    TLaserFollowTablePrmPro tablePrm;            // 当mode=LASER_FOLLOW_MODE_TABLE时对应的参数
    TLaserFollowDuoTablePrmPro duoTablePrm;      // 当mode=LASER_FOLLOW_MODE_DUO_TABLE时对应的参数
}TLaserFollowPro;

GT_API GTN_SetLaserEnablePro(short core, short laserChannel, short enable, short mode = 0, TListInfo* pListInfo = NULL);
GT_API GTN_SetLaserDelayPro(short core, short laserChannel, double laserOnDelay, double laserOffDelay, TListInfo* pListInfo = NULL);
GT_API GTN_SetLaserPwmPrmPro(short core, short laserChannel, TLaserPwmPrmPro* pPrm, TListInfo* pListInfo = NULL);
GT_API GTN_SetLaserPwmDutyPro(short core, short laserChannel, double duty, TListInfo* pListInfo = NULL);
GT_API GTN_SetLaserPwmFrequencyPro(short core, short laserChannel, double frequency, TListInfo* pListInfo = NULL);
GT_API GTN_SetLaserPwmPulseWidthPro(short core, short laserChannel, double pulseWidth, TListInfo* pListInfo = NULL);
GT_API GTN_SetLaserVoltagePrmPro(short core, short laserChannel, double minVoltage, double maxVoltage, TListInfo* pListInfo = NULL);
GT_API GTN_SetLaserVoltagePro(short core, short laserChannel, double voltage, TListInfo* pListInfo = NULL);
GT_API GTN_SetLaserParallelPrmPro(short core, short laserChannel, double minParallel, double maxParallel, TListInfo* pListInfo = NULL);
GT_API GTN_SetLaserParallelPro(short core, short laserChannel, double parallel, TListInfo* pListInfo = NULL);
GT_API GTN_SetLaserIoPro(short core, short laserChannel, short doType, short doValue, TListInfo* pListInfo = NULL);
GT_API GTN_GetLaserPowerPro(short core, short laserChannel, TLaserPowerPro* pLaserPowerPro);
GT_API GTN_GetLaserPro(short core, short laserChannel, TLaserPro* pLaserPro);
GT_API GTN_SetLaserFollowEnablePro(short core, short laserChannel, short enable, TListInfo* pListInfo = NULL);
GT_API GTN_SetLaserFollowPrmPro(short core, short laserChannel, TLaserFollowPrmPro* pPrm, TListInfo* pListInfo = NULL);
GT_API GTN_SetLaserFollowTablePrmPro(short core, short laserChannel, TLaserFollowTablePrmPro* pPrm, TListInfo* pListInfo = NULL);
GT_API GTN_SetLaserFollowDuoTablePrmPro(short core, short laserChannel, TLaserFollowDuoTablePrmPro* pPrm, TListInfo* pListInfo = NULL);
GT_API GTN_ClearLaserFollowTablePro(short core, short laserChannel, short tableId);
GT_API GTN_SetLaserFollowTablePro(short core, short laserChannel, short tableId, long count, double* pSynchVel, double* pPower);
GT_API GTN_GetLaserFollowTablePro(short core, short laserChannel, short tableId, long count, double* pSynchVel, double* pPower, long* pReadCount);
GT_API GTN_GetLaserFollowPro(short core, short laserChannel, TLaserFollowPro* pLaserFollowPro);
GT_API GTN_SetLaserPowerLatchIoPrameterPro(short core, short laserChannel, double waitTime, double pulseWidth, double enable, TListInfo* pListInfo = NULL);

#define SCAN_START_IN_LIST_MODE_INDIVIDUAL                 (0)
#define SCAN_START_IN_LIST_MODE_CHECK                      (1)

/**
 * @brief 启动振镜运动，支持指令流模式
 * @param core 核号，索引从1开始
 * @param scanCrd 振镜坐标系号，索引从1开始
 * @param pListInfo 指令流信息结构体参数指针
 * @param pListInfo->reserve1[1] 指令流中振镜启动模式，0：独立模式，1：检查模式，指令流执行过程中如果振镜出错，则停止指令流
 * @return 17001：TListInfo结构体中list参数错误
 *         17002：TListInfo结构体中modal参数错误
 *         17053：scanCrd振镜坐标系号参数错误
 *         17501：TListInfo结构体中reserve1[1]振镜启动模式参数错误
 *         11001：调用过GTN_CommandListDataEnd，且指令流还未执行完成，不需要压数据
 *         11002：指令流缓冲区满了，不允许压数据
 *         11003：当前指令数据大小大于指令流单个元素大小，需要调用指令GTN_SetCommandListConfig重新设置元素大小
 *         11004：当前指令流没有分配缓冲区空间，需要调用指令GTN_SetCommandListConfig配置指令流缓冲区空间
 *         11091：当前网络上没有对应的振镜资源
 *         11915：有振镜正在使用当前指令流，不允许压入振镜启动和停止指令
*/
GT_API GTN_ScanCrdStartPro(short core,short scanCrd,TListInfo *pListInfo);

/**
 * @brief 停止振镜运动，支持指令流模式
 * @param core 核号，索引从1开始
 * @param scanCrd 振镜坐标系号，索引从1开始
 * @param stopType 振镜停止模式，0：复位停止，立即停止并且复位振镜，1：立即停止，2：停止到段末
 * @param pListInfo 指令流信息结构体参数指针
 * @return 17001：TListInfo结构体中list参数错误
 *         17002：TListInfo结构体中modal参数错误
 *         17052：stopType振镜停止模式参数错误
 *         17053：scanCrd振镜坐标系号参数错误
 *         11001：调用过GTN_CommandListDataEnd，且指令流还未执行完成，不需要压数据
 *         11002：指令流缓冲区满了，不允许压数据
 *         11003：当前指令数据大小大于指令流单个元素大小，需要调用指令GTN_SetCommandListConfig重新设置元素大小
 *         11004：当前指令流没有分配缓冲区空间，需要调用指令GTN_SetCommandListConfig配置指令流缓冲区空间
 *         11091：当前网络上没有对应的振镜资源
 *         11915：有振镜正在使用当前指令流，不允许压入振镜启动和停止指令
*/
GT_API GTN_ScanCrdStopPro(short core,short scanCrd,short stopType,TListInfo *pListInfo);

typedef struct PidAlign
{
    double kp;
    double ki;
    double kd;
    double kvff;
    double kaff;
    long   integralLimit;
    long   derivativeLimit;
    short  limit;
    short  reserve[3];
}TPidAlign;

typedef struct GantryPrmPro
{
    short mode;
    short master;
    short slave;
    short reserve[3];
    long syncErrorLimit;
    TPidAlign gantryMasterPid;
    TPidAlign gantrySlavePid;
    TPidAlign masterPid;
    TPidAlign slavePid;
}TGantryPrmPro;

GT_API GTN_SetGantryPrmPro(short core, short group, TGantryPrmPro* pGantryPrmPro);


//---------------------------------------------------------
// KeSai
//---------------------------------------------------------
typedef struct StGearParam
{
    unsigned short axis_id;
    unsigned short axis_mode;   // fixed to 1 for Gear
    unsigned short axis_ctrl;   // 0: IDLE; 1: stop; 2: fast_stop; 3: run
    unsigned short mode;        // 0 : IDLE； 1：挂靠（run），2：脱离 3：
    unsigned short reserve;     // must be 0.
    unsigned short reserve2;    // must be 0.
    long modify_delta;
    long ratio_a;      // 可以为正值
    long ratio_b;      // 必须为正值
}TStGearParam;

typedef struct StTrapParam
{
    unsigned short axis_id;
    unsigned short axis_mode;   // fixed to 2 for Trap
    unsigned short axis_ctrl;   // 0: IDLE; 1: stop; 2: fast_stop; 3: run;
    unsigned short reserve;     // must be 0.
    long long target_pos;
    unsigned long max_vel;
    unsigned short reserve2[2]; // must be 0.
}TStTrapParam;

typedef struct StJogParam
{
    unsigned short axis_id;
    unsigned short axis_mode;   // fixed to 3 for Jog
    unsigned short axis_ctrl;   // 0: IDLE; 1: stop; 2: fast_stop; 3: run;
    unsigned short reserve;     // must be 0.

    long target_vel;
    unsigned long max_acc;
    unsigned long reserve1;
    unsigned short reserve2[2]; // must be 0.
}TStJogParam;

typedef struct StGantrayTrapParam
{
    unsigned short axis_id;
    unsigned short axis_mode;   // fixed to 8 for grantry trap
    unsigned short axis_ctrl;   // 0: IDLE; 1: stop; 2: fast_stop; 3: run;
    unsigned short reserve;     // must be 0.

    long long target_pos;
    unsigned long max_vel;

    long main_axis_shift;
    long second_axis_shift;
    unsigned short reserve2[2];  // must be 0.
}TStGantrayTrapParam;

typedef struct StDriverVelParam
{
    unsigned short axis_id;
    unsigned short axis_mode;   // fixed to 16 for driver vel
    unsigned short axis_ctrl;   // 0: IDLE; 1: stop; 2: fast_stop; 3: run ;
    unsigned short reserve;     // must be 0.

    long target_vel;
    unsigned short reserve3[2]; // must be 0.
    long long reserve2;
}TStDriverVelParam;

typedef struct StDriverCurParam
{
    unsigned short axis_id;
    unsigned short axis_mode;   // fixed to 17 for driver acc
    unsigned short axis_ctrl;   // 0: IDLE; 1: stop; 2: fast_stop; 3: run ;
    unsigned short reserve;     // must be 0.

    long target_acc;
    unsigned short reserve3[2]; // must be 0.
    long long reserve2;
}TStDriverCurParam;

typedef struct SetMultiMcFunctionPrm
{
    short profileMode;
    unsigned short reserve2[3]; //must be 0.
    TStTrapParam trapPrm;
    TStJogParam jogPrm;
    TStGearParam gearPrm;
    TStGantrayTrapParam gantrayTrapPrm;
    TStDriverVelParam driverVelPrm;
    TStDriverCurParam driverCurPrm;
}TSetMultiMcFunctionPrm;

typedef struct MultiMcFunctionReadPrm
{
    unsigned short prmErrorInf;
    short readCommandVarTableIndex;   // 读指令变量表索引。
    short readCount;                  // 要读的变量的数量。
    short pad[2];                     // 字节对齐，保留
    short realReadCount;              // 实际读变量的数量
    double* pVarValue;                // 读变量的值
}TMultiMcFunctionReadPrm;

GT_API GTN_StartMcMultiFunction(short core, unsigned short axisCount, unsigned short writeEnable, TSetMultiMcFunctionPrm* pSetPrm, TMultiMcFunctionReadPrm* pGetPrm);
GT_API GTN_LoadReadCommandVarTable(short core, short tableIndex, char* pFile);
GT_API GTN_ReadCommandVarTable(short core, short tableIndex, double* pVarValue, short count, short* pRealCount);

//---------------------------------------------------------
// Axis Simulation
//---------------------------------------------------------

#define AXISSIMULATION_RESERVE1_SUBMODE           (0)

typedef struct AxisSimulationParameter
{
    short enable;
    short reserve1[3];
    double reserve2[4];
} TAxisSimulationParameter;

GT_API GTN_SetAxisSimulationParameter(short core, short profile, TAxisSimulationParameter* pPrm, short count);
GT_API GTN_GetAxisSimulationParameter(short core, short profile, TAxisSimulationParameter* pPrm, short count);
GT_API GTN_GetAxisSimulationBeginPos(short core, short profile, double* pBeginPos, short count);
//卡号写入Flash功能
GT_API GTN_ProgramCardNumToFlash(short core, short coreNum);
GT_API GTN_ReadCardNumFromFlash(short core, short* pCardNum);


//等环网采样功能
typedef struct SamplingVar
{
    short varId;
    short stationId;
    short varType;
    short varIndex;
}StSamplingVar;
GT_API GTN_RN_SamplingInit(short cardIndex, short stationPhyId);
GT_API GTN_RN_SamplingAddVar(short cardIndex, short stationPhyId, StSamplingVar stVar);
GT_API GTN_RN_SamplingReadData(short cardIndex, short stationPhyId, short varIdndex, double* pBuffer, long bufSize, long* pReadCount);
GT_API GTN_RN_SamplingClear(short cardIndex, short stationPhyId, short mode);
GT_API GTN_RN_SamplingPrintData(short cardIndex, short stationPhyId, const char* pFileName, long startIndex, unsigned long printCount);
GT_API GTN_RN_SamplingGetInfo(short cardIndex, short stationPhyId, short infoType, unsigned long* pInfo);
//-------------------------------------------------------------------------------------------------------
// 数据采集功能：加载配置文件
// cardIndex:从1开始
// stationPhyId:从0开始
// pFileName：导出的文件路径及名称
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_LoadFileConfig(short cardIndex, short stationPhyId, char* pFileName);
//-------------------------------------------------------------------------------------------------------
// 数据采集功能：读取采集数据
// cardIndex:从1开始
// stationPhyId:从0开始
// pData：数据返回，用户需要定义成数组
// dataCount：设置需要回读的数据个数，取值范围[0,4096]
// pResDataCount：返回实际读到的数据个数
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_HighSpeedSamplingGetAllData(short cardIndex, short stationPhyId, unsigned short* pData, unsigned long dataCount, unsigned long* pResDataCount);
//-------------------------------------------------------------------------------------------------------
// 数据采集功能：关闭DMA数据采集
// cardIndex:从1开始
// stationPhyId:从0开始
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_HighSpeedSamplingOffAll(short cardIndex, short stationPhyId);
//-------------------------------------------------------------------------------------------------------
// 数据采集功能：开启DMA数据采集
// cardIndex:从1开始
// stationPhyId:从0开始
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_HighSpeedSamplingOnAll(short cardIndex, short stationPhyId);

//initMode等于0为默认行为，再次开卡时会复位到初始状态；initMode等于1为保持模式，再次开卡保持上一次输出的值。
GT_API GTN_SetExtModuleInitMode(short core, short initMode);
GT_API GTN_GetExtModuleInitMode(short core, short* pInitMode);

//-----------------------------------------------------------------------------------
// 读取Ilink扩展模块数据
// input：cardIndex----卡号，取值范围：[1,16]，在多主单网络中，卡号是1
// input：stationPhyId----物理站号，取值范围：[0,64]
// input：moduleId----扩展模块ID，取值范围：[0,63]
// input：address----扩展模块地址
// output：data----读取的值
// output：dwordNum----读取的数据个数
//-----------------------------------------------------------------------------------
GT_API GTN_RN_IlinkRdPduData32(short cardIndex, short stationPhyId, unsigned char moduleId, unsigned long address, unsigned long* data, unsigned long dwordNum);

//-----------------------------------------------------------------------------------
// 写Ilink扩展模块数据
// input：cardIndex----卡号，取值范围：[1,16]，在多主单网络中，卡号是1
// input：stationPhyId----物理站号，取值范围：[0,64]
// input：moduleId----扩展模块ID，取值范围：[0,63]
// input：address----扩展模块地址
// output：data----读取的值
// output：dwordNum----读取的数据个数
//-----------------------------------------------------------------------------------
GT_API GTN_RN_IlinkWrPduData32(short cardIndex, short stationPhyId, unsigned char moduleId, unsigned long address, unsigned long* data, unsigned long dwordNum);

/*-----------------------------------------------------------*/
/* Catch Up                                                  */
/*-----------------------------------------------------------*/
#define CATCH_UP_STATE_IDLE                                                (0)   // 空闲状态
#define CATCH_UP_STATE_WAIT                                                (100) // 等待启动追踪
#define CATCH_UP_STATE_CATCH_UP                                            (200) // 追踪主轴，尚未达到同步
#define CATCH_UP_STATE_SYNCH                                               (300) // 点胶头和工件同步
#define CATCH_UP_STATE_STOP_SYNCH                                          (400) // 点胶头减速停止
#define CATCH_UP_STATE_STOP_SYNCH_DONE                                     (500) // 点胶头减速停止

//-----------------------------------------------------------------------------
// 0 等待用户调用启动指令
//-----------------------------------------------------------------------------
#define CATCH_UP_STATE_IDLE                                                (0)  // 空闲状态

//-----------------------------------------------------------------------------
// 100 等待启动追踪
//-----------------------------------------------------------------------------
#define CATCH_UP_STATE_WAIT                                                (100)

#define CATCH_UP_STATE_WAIT_FOR_PIECE_IN_SITU_VIA_FOLLOW                   (CATCH_UP_STATE_WAIT+12)  // 在原位等待工件出现

#define CATCH_UP_STATE_WAIT_FOR_PIECE_IN_SITU_SYNCH_ONLY_VIA_FOLLOW        (CATCH_UP_STATE_WAIT+22)  // 在原位等待工件出现
#define CATCH_UP_STATE_WAIT_FOR_START_IN_SITU_SYNCH_ONLY_VIA_FOLLOW        (CATCH_UP_STATE_WAIT+23)  // 在原位等待工件穿越泊车位

#define CATCH_UP_STATE_WAIT_FOR_PIECE_IN_SITU_VIA_TRAP                     (CATCH_UP_STATE_WAIT+32)  // 在原位等待工件超过泊车位
#define CATCH_UP_STATE_WAIT_FOR_START_IN_SITU_VIA_TRAP                     (CATCH_UP_STATE_WAIT+33)  // 在原位等待工件超过泊车位

#define CATCH_UP_STATE_WAIT_FOR_PIECE_IN_SITU_SYNCH_ONLY_VIA_TRAP          (CATCH_UP_STATE_WAIT+42)  // 在原位等待工件超过泊车位
#define CATCH_UP_STATE_WAIT_FOR_START_IN_SITU_SYNCH_ONLY_VIA_TRAP          (CATCH_UP_STATE_WAIT+43)  // 在原位等待工件超过泊车位

#define CATCH_UP_STATE_TO_PARKING_NO_PIECE_VIA_FOLLOW                      (CATCH_UP_STATE_WAIT+70) // 没有工件，FOLLOW运动到泊车位
#define CATCH_UP_STATE_TO_PARKING_VIA_FOLLOW                               (CATCH_UP_STATE_WAIT+71) // 工件位置小于泊车位，FOLLOW运动到泊车位
#define CATCH_UP_STATE_WAIT_FOR_PIECE_IN_PARKING_VIA_FOLLOW                (CATCH_UP_STATE_WAIT+72) // 在泊车位等待工件出现
#define CATCH_UP_STATE_WAIT_FOR_START_IN_PARKING_VIA_FOLLOW                (CATCH_UP_STATE_WAIT+73) // 在泊车位等待工件超越启动位置

#define CATCH_UP_STATE_TO_PARKING_NO_PIECE_VIA_TRAP                        (CATCH_UP_STATE_WAIT+80) // 没有工件，点位运动到泊车位
#define CATCH_UP_STATE_TO_PARKING_VIA_TRAP                                 (CATCH_UP_STATE_WAIT+81) // 没有工件，点位运动到泊车位
#define CATCH_UP_STATE_WAIT_FOR_PIECE_IN_PARKING_VIA_TRAP                  (CATCH_UP_STATE_WAIT+82) // 在泊车位等待工件出现
#define CATCH_UP_STATE_WAIT_FOR_START_IN_PARKING_VIA_TRAP                  (CATCH_UP_STATE_WAIT+83) // 在泊车位等待工件超越启动位置
#define CATCH_UP_STATE_TO_RETURN_POSITION_VIA_TRAP                         (CATCH_UP_STATE_WAIT+84) // 从轴在工钱前超过sa+so处，并且返回位置大于泊车位，点位运动返回目标位置
#define CATCH_UP_STATE_WAIT_FOR_START_IN_RETURN_POSITION_VIA_TRAP          (CATCH_UP_STATE_WAIT+85) // 在返回位置等待工件超越启动位置
#define CATCH_UP_STATE_CALCULATE_RETURN_POS_TO_PARKING_VIA_TRAP            (CATCH_UP_STATE_WAIT+86)	// 在返回泊车的过程中有工件压入，则动态计算返回的合适的目标位置

//-----------------------------------------------------------------------------
// 200 追踪主轴，尚未达到同步
//-----------------------------------------------------------------------------
#define CATCH_UP_STATE_CATCH_UP                                            (200)

#define CATCH_UP_STATE_CATCH_UP_VIA_FOLLOW                                 (CATCH_UP_STATE_CATCH_UP+12) // 立即启动追赶

#define CATCH_UP_STATE_RETURN_CATCH_UP_VIA_FOLLOW                          (CATCH_UP_STATE_CATCH_UP+16) // 点胶头返回目标位置并立即启动追赶

#define CATCH_UP_STATE_TO_PIECE_NO_SLOPE_VIA_TRAP                          (CATCH_UP_STATE_CATCH_UP+20) // 工件静止，点位运动到工件位置，没有离合区
#define CATCH_UP_STATE_TO_PIECE_SLOPE_ESCAPE_VIA_TRAP                      (CATCH_UP_STATE_CATCH_UP+21) // 没有脱离离合区就停止了，强制进入Gear同步区，点位运动到工件位置
#define CATCH_UP_STATE_CATCH_UP_IN_SLOPE_VIA_TRAP                          (CATCH_UP_STATE_CATCH_UP+22) // 工件运动，点位同向运动到工件位置
#define CATCH_UP_STATE_CATCH_UP_SLOPE_DONE_VIA_TRAP                        (CATCH_UP_STATE_CATCH_UP+23) // 工件静止，点位运动到工件位置，没有离合区

#define CATCH_UP_STATE_RETURN_CATCH_UP_IN_SLOPE_VIA_TRAP                   (CATCH_UP_STATE_CATCH_UP+26) // 点位相向运动到工件位置

#define CATCH_UP_STATE_STOPPING_TO_RETURN_POS_VIA_TRAP                     (CATCH_UP_STATE_CATCH_UP+28) // 在返回泊车的过程中有工件压入，在合适位置停止并反向追赶

//-----------------------------------------------------------------------------
// 300 从轴和主轴已经同步
//-----------------------------------------------------------------------------
#define CATCH_UP_STATE_SYNCH                                               (300) // 点胶头和工件同步

#define CATCH_UP_STATE_PROCESS                                             (CATCH_UP_STATE_SYNCH+14) // 点胶头执行插补运动
#define CATCH_UP_STATE_PROCESS_DONE                                        (CATCH_UP_STATE_SYNCH+16) // 点胶头插补完成

//-----------------------------------------------------------------------------
// 400 从轴和主轴脱离同步
//-----------------------------------------------------------------------------
#define CATCH_UP_STATE_STOP_SYNCH                                          (400) // 点胶头减速停止

#define CATCH_UP_STATE_USER_STOPPING                                       (CATCH_UP_STATE_STOP_SYNCH+10) // 用户调用工艺模块停止指令，并且正在停止过程中

#define CATCH_UP_STATE_OUT_OF_RANGE_STOPPING                               (CATCH_UP_STATE_STOP_SYNCH+12) // 点胶头超过停止位
#define CATCH_UP_STATE_ERROR_STOPPING                                      (CATCH_UP_STATE_STOP_SYNCH+13) // cu模块处理出错停止

//-----------------------------------------------------------------------------
// 500 从轴和主轴脱离同步完成
//-----------------------------------------------------------------------------
#define CATCH_UP_STATE_STOP_SYNCH_DONE                                     (500) // 点胶头减速停止

#define CATCH_UP_STATE_USER_STOP_DONE                                      (CATCH_UP_STATE_STOP_SYNCH_DONE+10) // 用户调用工艺模块停止指令，并且已经停止完成。在该状态允许启动工艺模块
#define CATCH_UP_STATE_OUT_OF_RANGE_STOP_DONE                              (CATCH_UP_STATE_STOP_SYNCH_DONE+12) // 点胶头超过停止位急停完成
#define CATCH_UP_STATE_ERROR_STOP_DONE                                     (CATCH_UP_STATE_STOP_SYNCH_DONE+13) // cu模块处理出错停止完成


#define CATCH_UP_RETURN_MODE_FOLLOW                                  (0)
#define CATCH_UP_RETURN_MODE_TRAP                                    (1)

#define CATCH_UP_FIFO_EMPTY_ACTION_TO_PARKING                        (0)
#define CATCH_UP_FIFO_EMPTY_ACTION_IDLE                              (1)

#define CATCH_UP_STOP_MODE_DEFAULT                                   (0) // 默认停止模式，用户调用指令停止CatchUp后，CatchUp处于使能状态
#define CATCH_UP_STOP_MODE_DISABLE                                   (1) // 关闭使能停止模式，用户调用指令停止CatchUp后，关闭CatchUp使能

#define CATCH_UP_MASTER_MODE_CONTINUOUS                              (0)
#define CATCH_UP_MASTER_MODE_INTERMITTENT                            (1)

#define CATCH_UP_SLAVE_MODE_DEFAULT                                  (0)
#define CATCH_UP_SLAVE_MODE_SYNCH_ONLY                               (1)

#define CATCH_UP_OUT_OF_RANGE_ACTION_STOP_MASTER                     (0)
#define CATCH_UP_OUT_OF_RANGE_ACTION_STOP_SLAVE                      (1)
#define CATCH_UP_OUT_OF_RANGE_ACTION_STOP_ALL                        (2)

#define CATCH_UP_STOP_INFO_NONE                                      (0)
#define CATCH_UP_STOP_INFO_STOP_SYNCH                                (1)
#define CATCH_UP_STOP_INFO_USER_STOP                                 (2)
#define CATCH_UP_STOP_INFO_OUT_OF_RANGE_STOP                         (3)
#define CATCH_UP_STOP_INFO_ERROR_STOP                                (4)

#define CATCH_UP_ERROR_CODE_NONE                                                     (0)
#define CATCH_UP_ERROR_CODE_MASTER_IS_NONE                                           (1)
#define CATCH_UP_ERROR_CODE_SLAVE_IS_NONE                                            (2)
#define CATCH_UP_ERROR_CODE_START_MOTION_STATE                                       (3)
#define CATCH_UP_ERROR_CODE_START_MOTION_MODE                                        (4)
#define CATCH_UP_ERROR_CODE_START_MOTION_RUN                                         (5)
#define CATCH_UP_ERROR_CODE_START_MOTION_PROFILE_MODE                                (6)
#define CATCH_UP_ERROR_CODE_START_MOTION_FOLLOW_CLEAR                                (7)
#define CATCH_UP_ERROR_CODE_START_MOTION_FOLLOW_DATA                                 (8)
#define CATCH_UP_ERROR_CODE_START_MOTION_SLAVE_TYPE                                  (9)
#define CATCH_UP_ERROR_CODE_START_MOTION_FOLLOW_START                                (10)
#define CATCH_UP_ERROR_CODE_STOP_MOTION_MODE                                         (11)
#define CATCH_UP_ERROR_CODE_OUT_OF_RANGE_STOP_MOTION_MODE                            (12)
#define CATCH_UP_ERROR_CODE_START_PROCESS_MODE                                       (13)
#define CATCH_UP_ERROR_CODE_START_PROCESS_CRD_START                                  (14)
#define CATCH_UP_ERROR_CODE_STOP_SYNCH_MODE                                          (15)
#define CATCH_UP_ERROR_CODE_IS_STOP_SYNCH_DONE_STATE                                 (16)
#define CATCH_UP_ERROR_CODE_IS_PARKING_MOTION_DONE_VIA_FOLLOW_STATE                  (17)
#define CATCH_UP_ERROR_CODE_IS_CATCH_UP_MOTION_DONE_VIA_FOLLOW_STATE                 (18)
#define CATCH_UP_ERROR_CODE_IS_RETURN_CATCH_UP_MOTION_DONE_VIA_FOLLOW_STATE          (19)
#define CATCH_UP_ERROR_CODE_IS_PROCESS_DONE_STATE                                    (20)
#define CATCH_UP_ERROR_CODE_POP_FIFO_PIECE_COUNT                                     (21)
#define CATCH_UP_ERROR_CODE_POP_FIFO_PIECE_SET_PRF_POS                               (22)
#define CATCH_UP_ERROR_CODE_ALLOCATE_MEMORY_POOL                                     (23)
#define CATCH_UP_ERROR_CODE_STATE_WAIT_MODE                                          (24)
#define CATCH_UP_ERROR_CODE_START_SLAVE_ADDITION_TRAP_MOTION_SET_POS                 (25)
#define CATCH_UP_ERROR_CODE_START_SLAVE_ADDITION_TRAP_MOTION_TYPE                    (26)
#define CATCH_UP_ERROR_CODE_START_SLAVE_ADDITION_TRAP_MOTION_UPDATE                  (27)
#define CATCH_UP_ERROR_CODE_START_SLAVE_GEAR_MOTION_SET_GEAR_RATIO                   (28)
#define CATCH_UP_ERROR_CODE_START_SLAVE_GEAR_MOTION_TYPE                             (29)
#define CATCH_UP_ERROR_CODE_START_SLAVE_GEAR_MOTION_GEAR_START                       (30)
#define CATCH_UP_ERROR_CODE_START_SLAVE_GEAR_MOTION_SET_GEAR_SLOPE_END               (31)
#define CATCH_UP_ERROR_CODE_IS_SLOPE_DONE_PROFILE_MODE                               (32)
#define CATCH_UP_ERROR_CODE_IS_SLAVE_ADDITION_TRAP_MOTION_DONE_PROFILE_MODE          (33)
#define CATCH_UP_ERROR_CODE_IS_SLAVE_ADDITION_TRAP_MOTION_DONE_POS                   (34)
#define CATCH_UP_ERROR_CODE_IS_TO_PIECE_VIA_TRAP_MOTION_DONE_STATE                   (35)
#define CATCH_UP_ERROR_CODE_IS_TO_PIECE_VIA_TRAP_MOTION_DONE_PROFILE_MODE            (36)
#define CATCH_UP_ERROR_CODE_IS_PARKING_VIA_TRAP_MOTION_DONE_STATE                    (37)
#define CATCH_UP_ERROR_CODE_GET_GEAR_SLOPE_LEFT_PROFILE_MODE                         (38)
#define CATCH_UP_ERROR_CODE_CU_STATE                                                 (39)
#define CATCH_UP_ERROR_CODE_CU_MASTER_MODE                                           (40)
#define CATCH_UP_ERROR_CODE_START_MOTION_PRF_FOLLOW                                  (41)
#define CATCH_UP_ERROR_CODE_START_MOTION_SET_FOLLOW_MASTER                           (42)
#define CATCH_UP_ERROR_CODE_MASTER_VEL_ZERO                                          (43)
#define CATCH_UP_ERROR_CODE_MASTER_VEL_GREATER_THAN_SLAVE_VEL_MAX                    (44)
#define CATCH_UP_ERROR_CODE_MASTER_MOVE_DIRECTION                                    (45)
#define CATCH_UP_ERROR_CODE_START_SLAVE_TRAP_MOTION_PRF_TRAP                         (46)
#define CATCH_UP_ERROR_CODE_START_SLAVE_TRAP_MOTION_SET_TRAP_PRM                     (47)
#define CATCH_UP_ERROR_CODE_START_SLAVE_TRAP_MOTION_SET_VEL                          (48)
#define CATCH_UP_ERROR_CODE_START_SLAVE_TRAP_MOTION_SET_POS                          (49)
#define CATCH_UP_ERROR_CODE_START_SLAVE_TRAP_MOTION_UPDATE                           (50)
#define CATCH_UP_ERROR_CODE_START_SLAVE_TRAP_MOTION_SLAVE_TYPE                       (51)
#define CATCH_UP_ERROR_CODE_IS_SLAVE_TRAP_MOTION_DONE_PROFILE_MODE                   (52)
#define CATCH_UP_ERROR_CODE_IS_SLAVE_TRAP_MOTION_DONE_POS                            (53)
#define CATCH_UP_ERROR_CODE_IS_SLAVE_TRAP_STOP_DONE_PROFILE_MODE                     (54)

typedef struct CatchUpPrm
{
    short masterMode;                  // 主轴运动模式，0：连续运动；
                                       // 1：间歇运动,从轴需要2个虚拟轴
    short masterType;                  // 主轴类型，MC_ENCODER、MC_AU_ENCODER
    short masterIndex;                 // 主轴索引

    short slaveMode;                   // 从轴模式，0：完全控制从轴运动；1：从轴只负责和主轴达到速度同步和位置同步
    short slaveType;                   // 从轴类型，网络式控制器/卡目前只支持从轴类型为：MC_VIRTUAL_PROFILE
    short slaveIndex;                  // 从轴索引
    short slaveAdditionType;           // 从轴叠加轴类型，网络式控制器/卡目前只支持从轴叠加轴类型为：MC_VIRTUAL_PROFILE
    short slaveAdditionIndex;          // 从轴叠加轴索引
    short slaveLink;                   // 从轴叠加到坐标轴索引，0表示不叠加（从轴是一个物理轴）

    short outOfRangeAction;            // 从轴超过加工范围的行为，0：停止主轴，1：停止从轴，2：停止主轴和从轴

    short trapSmoothTime;              // 返回时点位运动的平滑时间,单位：ms
    short followSmoothPercent;         // 正向追踪传送带时的follow运动的S曲线所占加速时间的百分比，目前不支持，必须填0

    double synchOffset;                // 同步位置偏移，mm
    double detectPos;                  // 工件检测位置，mm
    double parkPos;                    // 泊车位，mm
    double stopPos;                    // 停止位，mm

    double slaveVelMax;                // 从轴最大速度，mm/s
    double slaveAcc;                   // 从轴加速度，mm/s^2

    double sampleTime;                 // 速度采样时间，ms

    double reserve2[5];
} TCatchUpPrm;

typedef struct StartCatchUpPrm
{
    short mode;                        // 启动模式：0用Follow模式返回，1用点位模式返回
    short fifoEmptyAction;             // 工件缓冲区为空时从轴行为，0：从轴返回泊车位
                                       // 1：等待压入工件位置以后再决定从轴的动作
    short reserve1[2];
    long reserve2[4];
    double reserve3[4];
} TStartCatchUpPrm;

typedef struct StopCatchUpPrm
{
    short mode;                        // 停止模式
    short reserve1[3];
    long reserve2[4];
    double reserve3[4];
} TStopCatchUpPrm;

typedef struct StartCatchUpProcessPrm
{
    short mode;                        // 启动模式：0每个工件由用户压入插补数据，工艺模块只负责启动插补
    short crdIndex;                    // 坐标系索引
    short reserve1[2];
    long reserve2[4];
    double reserve3[4];
} TStartCatchUpProcessPrm;

typedef struct StopCatchUpSynchPrm
{
    short mode;                        // 停止模式：0按照slaveAcc急停
    short reserve1[3];
    long reserve2[4];
    double reserve3[4];
} TStopCatchUpSynchPrm;

typedef struct CatchUpPieceFifo
{
    unsigned long id;                  // 工件ID
    long reserve1[3];
    double pos;                        // 工件对应的主轴编码器位置
    double reserve2[3];
} TCatchUpPieceFifo;

typedef struct CatchUpStatus
{
    short state;                       // 当前状态
    short stopInfo;                    // 停止原因
    short errorCode;                   // 执行出错错误码
    short reserve1[5];
    unsigned long id;                  // 当前工件ID
    unsigned long pieceCount;          // 加工工件数量
    long reserve2[6];
    double synchDistance;              // 进入同步区的位移
    double synchTime;                  // 进入同步区的时间
    double reserve3[6];
} TCatchUpStatus;

typedef struct CatchUpInfo
{
    short state;
    short stopInfo;                    // 停止原因
    short errorCode;                   // 执行出错错误码
    short reserve1[5];
    long reserve2[8];
    double pieceStartPos;
    double slaveStartPos;
    double pieceSynchPos;
    double slaveSynchPos;
    double pieceDistanceMin;           // 保持同步点不变的工件最小间距
    double reserve3[8];
} TCatchUpInfo;

GT_API GTN_SetCatchUpPrm(short core, short catchUpIndex, TCatchUpPrm* pPrm);
GT_API GTN_GetCatchUpPrm(short core, short catchUpIndex, TCatchUpPrm* pPrm);
GT_API GTN_StartCatchUp(short core, short catchUpIndex, TStartCatchUpPrm* pPrm);
GT_API GTN_StopCatchUp(short core, short catchUpIndex, TStopCatchUpPrm* pPrm);
GT_API GTN_StartCatchUpProcess(short core, short catchUpIndex, TStartCatchUpProcessPrm* pPrm);
GT_API GTN_StopCatchUpSynch(short core, short catchUpIndex, TStopCatchUpSynchPrm* pPrm);
GT_API GTN_PushCatchUpPieceFifo(short core, short catchUpIndex, TCatchUpPieceFifo* pFifo);
GT_API GTN_GetCatchUpStatus(short core, short catchUpIndex, TCatchUpStatus* pStatus);
GT_API GTN_GetCatchUpInfo(short core, short catchUpIndex, TCatchUpInfo* pInfo);
GT_API GTN_ClearCatchUpStatus(short core, short catchUpIndex);


#define DI_STOP_VALUE_LOW_LEVEL                            (0)  // Di触发值，电平触发，有效值0
#define DI_STOP_VALUE_HIG_LEVEL                            (1)  // Di触发值，电平触发，有效值1
#define DI_STOP_VALUE_FALLING_EDGE                         (2)  // Di触发值，沿触发，下降沿
#define DI_STOP_VALUE_RISING_EDGE                          (3)  // Di触发值，沿触发，上升沿

#define DI_STOP_AXSI_STOP_TYPE_NONE                        (-1)                                  // Di触发，轴无操作
#define DI_STOP_AXIS_STOP_TYPE_ABRUPT                      (0)                                   // Di触发，轴紧急停止
#define DI_STOP_AXIS_STOP_TYPE_SMOOTH                      (1)                                   // Di触发，轴平滑停止
#define DI_STOP_AXIS_STOP_TYPE_ABRUPT_THEN_AXIS_OFF        (DI_STOP_AXIS_STOP_TYPE_ABRUPT + 10)  // Di触发，轴紧急停止后下使能
#define DI_STOP_AXIS_STOP_TYPE_SMOOTH_THEN_AXIS_OFF        (DI_STOP_AXIS_STOP_TYPE_SMOOTH + 10)  // Di触发，轴平滑停止后下使能

//-------------------------------------------------------------------------------------------------------
// 设置di关联停止轴
// core：核号
// diType：di类型
// diIndex：di索引
// diValue：di触发值,0：电平触发，有效值0，1：电平触发，有效值1，2：沿触发，下降沿，3：沿触发，上升沿
// stopType：停止类型。-1：取消功能，0：紧急停止，1：平滑停止，10：紧急停止后下使能，11：平滑停止后下使能
// pLinkAxisMask：di触发后需要停止的轴掩码数组指针
// axisMaskCount：轴掩码数组大小
//-------------------------------------------------------------------------------------------------------
GT_API GTN_SetStopIoLinkAxes(short core, short diType, short diIndex, short diValue, short stopType, unsigned long* pLinkAxisMask, short axisMaskCount = 1);
GT_API GTN_GetStopIoLinkAxes(short core, short diType, short diIndex, short* pDiValue, short* pStopType, unsigned long* pLinkAxisMask, short* pAxisMaskCount);

//-------------------------------------------------------------------------------------------------------
// 设置di关联输出值
// core：核号
// diType：di类型
// diIndex：di索引
// diValue：di触发值,0：电平触发，有效值0，1：电平触发，有效值1，2：沿触发，下降沿，3：沿触发，上升沿
// doType：di触发后需要输出的do类型，MC_NONE：取消功能
// pDoMask：di触发后需要输出的do索引掩码数组指针
// pDoValue：di触发后需要输出的do值掩码数组指针
// doMaskCount：do掩码数组大小
//-------------------------------------------------------------------------------------------------------
GT_API GTN_SetStopIoLinkDo(short core, short diType, short diIndex, short diValue, short doType, unsigned long* pDoMask, unsigned long* pDoValue, short doMaskCount = 1);
GT_API GTN_GetStopIoLinkDo(short core, short diType, short diIndex, short* pDiValue, short* pDoType, unsigned long* pDoMask, unsigned long* pDoValue, short* pDoMaskCount);

//-------------------------------------------------------------------------------------------------------
// 设置规划运动限制参数，脉冲单位
// core：核号
// profile：规划器号
// velMax：规划运动最大速度限制，单位：脉冲/ms
// accMax：规划运动最大加速度限制，单位：脉冲/ms^2
// 注意：
// （1）当规划运动超过最大限制，则规划轴会紧急停止
// （2）当用户设置velMax或者accMax其中一个为0，则表示取消规划运动最大限制检查
// （3）与指令GTN_SetAxisMotionConstraint作用一样，区别是一个为脉冲单位，一个是物理单位，且两边设置都生效
//-------------------------------------------------------------------------------------------------------
GT_API GTN_SetProfileMotionConstraint(short core, short profile, double velMax, double accMax);
GT_API GTN_GetProfileMotionConstraint(short core, short profile, double* pVelMax, double* pAccMax);

//设置位置比较输出比较源是否有效功能
GT_API GTN_PosCompareBufCmdEnable(short core, short posCompareIndex, short enable);
typedef struct PosCompareBufSourceEnable
{
    unsigned long segmentNumber;
    short xAxisEnable;
    short yAxisEnable;
    short zAxisEnable;
    short reserve[5];
}TPosCompareBufSourceEnable;
GT_API GTN_PosCompareBufSourceEnable(short core, short posCompareIndex, TPosCompareBufSourceEnable* pSourceEnable);
typedef struct PosCompareBufEnableInfo
{
    short bufCmdEnable;
    short xAxisEnable;
    short yAxisEnable;
    short zAxisEnable;
    short reserve[4];
}TPosCompareBufEnableInfo;
GT_API GTN_GetPosCompareBufEnableInfo(short core, short posCompareIndex, TPosCompareBufEnableInfo* pEnableInfo);


//-------------------------------------------------------------------------------------------------------
// 避障功能工具函数
// 根据起点、终点、中间点，以及运动参数计算PVT点数据
//-------------------------------------------------------------------------------------------------------
#define MP_POINT_MAX                        (16)        // 单段速最多8个点已经足够，2段速12个点已经足够

#define MP_ERROR_NONE                       (0)
#define MP_ERROR_VEL                        (700)
#define MP_ERROR_ACC                        (701)
#define MP_ERROR_DEC                        (702)
#define MP_ERROR_POS_END                    (703)
#define MP_ERROR_POS_OUT_OF_RANGE           (704)
#define MP_ERROR_PERCENT                    (705)
#define MP_ERROR_ROOT                       (706)
#define MP_ERROR_SQRT                       (707)
#define MP_ERROR_SEGMENT_TYPE               (708)
#define MP_ERROR_TIME_NOT_FIND              (709)

#define MP_SEGMENT_ASCEND_1                 (1)
#define MP_SEGMENT_ASCEND_TRAP              (2)
#define MP_SEGMENT_ASCEND_3                 (3)
#define MP_SEGMENT_EVEN                     (4)
#define MP_SEGMENT_DESCEND_5                (5)
#define MP_SEGMENT_DESCEND_TRAP             (6)
#define MP_SEGMENT_DESCEND_7                (7)
#define MP_SEGMENT_END                      (8)

typedef struct MpParameter
{
    double posBegin;
    double posEnd;
    double acc;                             // 加速度
    double dec;                             // 减速度
    double percentAcc;                      // 加速段百分比
    double percentDec;                      // 减速段百分比

    double vel;                             // 最大速度
}TMpParameter;

typedef struct Mp2Parameter
{
    double posBegin;
    double posEnd;
    double acc;                             // 加速度
    double dec;                             // 减速度
    double percentAcc;                      // 加速段百分比
    double percentDec;                      // 减速段百分比

    double vel1;                            // vel1第一段的最大速度
    double vel2;                            // vel2第二段的最大速度，当vel1=vel2时，则按照一段速计算
    double posMiddle;                       // posMiddle二段速度切换时的位置，必须在起点位置和终点位置之间
}TMp2Parameter;

typedef struct MpPoint
{
    double time;                            // 当前段的起点时间
    double vel;                             // 当前段的起点速度
    double pos;                             // 当前段的起点位置

    double accDir;                          // 当前段的加速度方向
    double am;                              // 当前段的最大加速度
    double jerk;                            // 当前段的加加速度
    unsigned short segmentType;             // 当前段类型，1-7
}TMpPoint;

typedef struct MpResult
{
    short dir;                              // 1：正向运动；-1：负向运动
    unsigned short pad[2];
    unsigned short knotCount;               // 节点数量
    TMpPoint knot[MP_POINT_MAX];            // S曲线所有节点的位置、速度、时间
}TMpResult;

/**
 * @brief 计算速度曲线各个节点的位置、速度和时间
 * @param mpp 运动参数
 * @param mpr 速度曲线各个节点的位置、速度和时间
 * @return 错误码
*/
GT_API GTN_UTL_MovePercentShape(const TMpParameter* pMp, TMpResult* pMpr);

/**
 * @brief 计算速度曲线各个节点的位置、速度和时间
 * @param mpp 运动参数
 * @param mpr 速度曲线各个节点的位置、速度和时间
 * @return 错误码
*/
GT_API GTN_UTL_MovePercent2Shape(const TMp2Parameter* pMp2, TMpResult* pMpr);

/**
 * @brief 根据曲线形态和起止点位置，计算不同中间点的信息
 * @param mpr 速度曲线各个节点的位置、速度和时间
 * @param posBegin 起点位置
 * @param posEnd 终点位置
 * @param posMiddle 速度曲线各个节点的位置、速度和时间
 * @param pPoint 中间点运动信息
 * @return 错误码
*/
GT_API GTN_UTL_MovePercentTime(const TMpResult* pMpr, double posBegin, double posEnd, double posMiddle, TMpPoint* pPoint);

//-----------------------------------------------------------------------------------
// 设置和读取驱动器的编码器的输出分辨率
//-----------------------------------------------------------------------------------
GT_API GTN_RN_GetGlink2OutputResolution(short cardIndex, short stationPhyId, short axis, unsigned long* pValue);
GT_API GTN_RN_SetGlink2OutputResolution(short cardIndex, short stationPhyId, short axis, unsigned long  value);

//-------------------------------------------------------------------------------------------------------
// 根据XML配置信息，网络恢复功能，匠心
//-------------------------------------------------------------------------------------------------------
typedef struct OnlineStationType
{
    short stationId;
    unsigned short stationType;
    unsigned short subType;
    short reserve1;
    short reserve2;
    short reserve3;
    short reserve4;
    short reserve5;
}TOnlineStationType;
GT_API GTN_RN_RingNetRecoverEx(short cardIndex, short stationPhyId, short mode, short* pParam);
GT_API GTN_RN_GetOnlineDeviceNum(short cardIndex, short* pOnlineDeviceNum);
GT_API GTN_RN_GetOnlineStationType(short cardIndex, TOnlineStationType* pOnlineStationType, short stationNums, short* pResNums);
GT_API GTN_InsertCallbackEvent(short core, short motionMode, short index, short eventNumber, TListInfo* pListInfo);
// 多PC多开
GT_API GT_TcpServerPortInit(char* hostIp, unsigned short hostPort);
GT_API GT_TcpServerPortDeInit(void);
GT_API GT_TcpClientPortInit(char* hostIp, unsigned short hostPort);
GT_API GT_TcpClientPortDeInit(void);

//-------------------------------------------------------------------------------------------------
// 环路切换参数结构体
//-------------------------------------------------------------------------------------------------
typedef struct SwitchLoopParameter
{
    short enable;                      // 使能或者禁用环路切换功能,0: 禁用环路切换功能,1: 启用环路切换功能
    short mode;                        // 切换模式,0: 由闭环切换至开环,1: 由开环切换至闭环

    long dir;                          // 闭环切开环时阈值的判断方向,0: 小于阈值时进行切换,1: 大于阈值时进行切换
    long threshold;                    // 环路切换的阈值,位置环跟随误差超过该阈值时切换环路,单位: 脉冲
    long openLoopDac;                  // 从闭环切换到开环之后的dac输出值,单位: dac的bit位
    double overTime;                   // 环路切换跟随误差超时时间,当跟随误差超过阈值并持续该时间后,进行切换

    long posOffset;                    // 开环切换闭环时，规划位置相对于编码器位置的偏移量,单位: 脉冲
    short reserve[2];                  // 保留
}TSwitchLoopParameter;

//-------------------------------------------------------------------------------------------------
// 齿槽力补偿参数结构体
//-------------------------------------------------------------------------------------------------
typedef struct TorqueForceCompensatePara
{
    long loopFlag;                     // 是否需要进行循环补偿,对于极距循环和每圈循环的情况使用,可以使用相同的补偿表循环补偿,0: 不进行循环补偿,1: 循环补偿
    long tableMax;                     // 齿槽力补偿表的最大数量值,正向表和反向表的数量必须一致

    double startPos;                   // 进行齿槽力补偿的编码器区间的起始位置值(相对于index位置),单位: 脉冲
    double length;                     // 进行齿槽力补偿的编码器区间的长度,单位: 脉冲
    double refPos;                     // 参考位置值,即当前位置系统中index信号的位置值,单位: 脉冲

    double positiveOffset;             // 正向补偿时,补偿值偏移量,单位: 力矩输出单位,该值更多适用于抵消摩擦力
    double positiveScale;              // 正向补偿时,补偿比例,该比例乘以补偿表的数值,作为补偿值进行输出

    double negativeOffset;             // 负向补偿时,补偿值偏移量,单位: 力矩输出单位,该值更多适用于抵消摩擦力
    double negativeScale;              // 负向补偿时,补偿比例,该比例乘以补偿表的数值,作为补偿值进行输出
}TTorqueForceCompensatePara;
GT_API GTN_SetVelLoopPid(short core, short control, short index, TPid* pPid);
GT_API GTN_GetVelLoopPid(short core, short control, short index, TPid* pPid);
GT_API GTN_SetVelLoopLowpassPara(short core, short control, short index, short enableFlag, double freq, double damping);
GT_API GTN_GetVelLoopLowpassPara(short core, short control, short index, short* pEnableFlag, double* pFreq, double* pDamping);
GT_API GTN_SetVelLoopNotchPara(short core, short control, short index, short enableFlag, double centerFreq, double bandWidth, double notchDepth);
GT_API GTN_GetVelLoopNotchPara(short core, short control, short index, short* pEnableFlag, double* pCenterFreq, double* pBandWidth, double* pNotchDepth);
GT_API GTN_SetVelLoopKjff(short core, short control, short index, double kjff);
GT_API GTN_GetVelLoopKjff(short core, short control, short index, double* pKjff);
GT_API GTN_SetSwitchLoopParameter(short core, short control, TSwitchLoopParameter* pSwitchLoopPara);
GT_API GTN_GetSwitchLoopParameter(short core, short control, TSwitchLoopParameter* pSwitchLoopPara);
GT_API GTN_SetForceValue(short core, short control, short index, double forceValue);
GT_API GTN_GetForceValue(short core, short control, short index, double* pForceValue);
GT_API GTN_SetLoopOutputProtect(short core, short control, double outputValueMax, long overTime);
GT_API GTN_GetLoopOutputProtect(short core, short control, double* pOutputValueMax, long* pOverTime, short* pStatus);
GT_API GTN_SwitchControlLoopMode(short core, short control, short index, short loopMode, double loopOutput, short switchMode);
GT_API GTN_SetFeedForwordValueLimit(short core, short control, double velMax, double accMax, double jerkMax);
GT_API GTN_GetFeedForwordValueLimit(short core, short control, double* pVelMax, double* pAccMax, double* pJerkMax);
GT_API GTN_GetControlLoopMode(short core, short control, short index, short* loopMode);

GT_API GTN_SetSpringCompensatePara(short core, short control, short enable, long refPos, double k, double b, double limit);
GT_API GTN_GetSpringCompensatePara(short core, short control, short* pEnable, long* pRefPos, double* pK, double* pB, double* pLimit);
GT_API GTN_SetTorqueForceComp(short core, short control, TTorqueForceCompensatePara* pPara, double* pPositiveComp, double* pNegativeComp);
GT_API GTN_EnableTorqueForceComp(short core, short control, short mode);

#define FILTER_LOCATION_FEEDBACK_POS	(0)
#define FILTER_LOCATION_VEL_LOOP_ERROR	(1)
#define FILTER_LOCATION_VEL_LOOP_OUTPUT	(2)
#define FILTER_LOCATION_FEEDBACK_VEL	(3)
#define FILTER_LOCATION_FORCE_CONTROL_OUTPUT         (4)
GT_API GTN_SetLowpassFilterFirstOrder(short core, short control, short index, short filterLocation, short filterIndex, short enableFlag, double freq);
GT_API GTN_GetLowpassFilterFirstOrder(short core, short control, short index, short filterLocation, short filterIndex, short* pEnableFlag, double* pFreq);
GT_API GTN_SetLowpassFilterSecondOrder(short core, short control, short index, short filterLocation, short filterIndex, short enableFlag, double freq, double damping);
GT_API GTN_GetLowpassFilterSecondOrder(short core, short control, short index, short filterLocation, short filterIndex, short* pEnableFlag, double* pFreq, double* pDamping);
GT_API GTN_SetNotchFilter(short core, short control, short index, short filterLocation, short filterIndex, short enableFlag, double centerFreq, double bandWidth, double notchDepth);
GT_API GTN_GetNotchFilter(short core, short control, short index, short filterLocation, short filterIndex, short* pEnableFlag, double* pCenterFreq, double* pBandWidth, double* pNotchDepth);


typedef struct DualAxisParameter
{
    short masterIndex;                  // 主轴索引
    short slaveIndex[2];                // 从轴索引
    short pad1;                         // 保留参数，必须为0

    double stopDec;                     // X1轴的停止减速度
    double stopJerk;                    // X1轴的停止jerk

    double borderPositive[2];           // 从轴正向边界
    double borderNegative[2];           // 从轴负向边界
}TDualAxisParameter;

typedef struct DualAxisStatus
{
    short enable;
    short errorId;
    short pad1[2];

    short state[2];
    short border[2];

    double pad2[6];
}TDualAxisStatus;

GT_API GTN_SetDualAxisParameter(short core, short index, TDualAxisParameter* pPrm);
GT_API GTN_GetDualAxisParameter(short core, short index, TDualAxisParameter* pPrm);
GT_API GTN_EnableDualAxis(short core,short index,short enable);
GT_API GTN_GetDualAxisStatus(short core,short index,TDualAxisStatus *pSts);
GT_API GTN_ResetDualAxis(short core,short index);


typedef struct MoveAbsoluteSineParameter
{
    double pos;                        // 目标位置
    double vel;                        // 最大速度
    double acc;                        // 最大加速度
    double dec;                        // 最大减速度
    double jerkAcc;                    // 加速段最大jerk
    double jerkDec;                    // 减速段最大jerk

    double velEvenTime;                // 暂未实现，必须为0
    double accEvenTime;                // 暂未实现，必须为0
    double decEvenTime;                // 暂未实现，必须为0

    unsigned short bufferMode;         // 暂未实现，必须为0
    unsigned short pad1;               // 保留参数，必须为0
    unsigned short pad2;               // 保留参数，必须为0
    unsigned short pad3;               // 保留参数，必须为0

    double reserve1;                   // 保留参数，必须为0
    double reserve2;                   // 保留参数，必须为0
    double reserve3;                   // 保留参数，必须为0
    double reserve4;                   // 保留参数，必须为0
}TMoveAbsoluteSineParameter;

GT_API GTN_MoveAbsoluteSine(short core, short profile, TMoveAbsoluteSineParameter* pPrm, TListInfo* pListInfo = NULL, short group = 0);
GT_API GTN_GetMoveAbsoluteSineParameter(short core, short profile, TMoveAbsoluteSineParameter* pPrm);

typedef struct MoveAbsoluteJerkParameter
{
    double pos;                        // 目标位置
    double vel;                        // 最大速度
    double acc;                        // 最大加速度
    double dec;                        // 最大减速度
    double jerkAcc;                    // 加速段最大jerk
    double jerkDec;                    // 减速段最大jerk

    double velEvenTime;                // 暂未实现，必须为0
    double accEvenTime;                // 暂未实现，必须为0
    double decEvenTime;                // 暂未实现，必须为0

    unsigned short bufferMode;         // 暂未实现，必须为0
    unsigned short pad1;               // 保留参数，必须为0
    unsigned short pad2;               // 保留参数，必须为0
    unsigned short pad3;               // 保留参数，必须为0

    double reserve1;                   // 保留参数，必须为0
    double reserve2;                   // 保留参数，必须为0
    double reserve3;                   // 保留参数，必须为0
    double reserve4;                   // 保留参数，必须为0
}TMoveAbsoluteJerkParameter;

typedef struct MoveAbsoluteJerkStatus
{
   	unsigned short motionDone;         // 运动完成标志，1表示运动完成
	unsigned short pad1[3];            // 保留参数

	double timeElapse;                 // 运行时间
	double timeLeft;                   // 剩余时间

    double aimPos;                     // 目标位置
    double prfPos;                     // 当前规划位置
    double prfVel;                     // 当前规划速度
    double prfAcc;                     // 当前规划加速度

    double reserve1[4];                // 保留参数
}TMoveAbsoluteJerkStatus;

GT_API GTN_MoveAbsoluteJerk(short core, short profile, TMoveAbsoluteJerkParameter* pPrm, TListInfo* pListInfo = NULL, short group = 0);
GT_API GTN_GetMoveAbsoluteJerkParameter(short core, short profile, TMoveAbsoluteJerkParameter* pPrm);
GT_API GTN_GetMoveAbsoluteJerkStatus(short core,short profile,TMoveAbsoluteJerkStatus *pStatus);
GT_API GTN_MultiMoveAbsoluteJerk(short core,short *pProfileArray,TMoveAbsoluteJerkParameter *pPrmArray,short count,short mode,TListInfo *pListInfo=NULL,short group=0);
GT_API GTN_GetMultiMoveAbsoluteJerkTime(short core,short *pProfileArray,TMoveAbsoluteJerkParameter *pPrmArray,short count,short mode,double *pTimeArray);
GT_API GTN_QueryMoveAbsoluteJerkStatusAccordingTime(short core,short profile,TMoveAbsoluteJerkParameter* pPrm,short count,double posBegin,double time,TMoveAbsoluteJerkStatus *pStatus);
GT_API GTN_QueryMoveAbsoluteJerkStatusAccordingPos(short core,short profile,TMoveAbsoluteJerkParameter* pPrm,short count,double posBegin,double pos,TMoveAbsoluteJerkStatus *pStatus);

typedef struct Compensate3D
{
    short enable;           // 1：使能三维补偿；0：关闭三维补偿
    short tableIndex;       // 三维补偿表索引
    short axisType[3];      // 三维补偿参考轴类型
    short axisIndex[3];     // 三维补偿参考轴索引
    short reserve1[4];      // 保留参数，必须为0
} TCompensate3D;

typedef struct Compensate3DTable
{
    long count[3];          // count[0]：X轴方向补偿点数
                            // count[1]：Y轴方向补偿点数
                            // count[2]：Z轴方向补偿点数

    long pad1[5];           // 保留参数，必须为0

    double posBegin[3];     // posBegin[0]：补偿区域X轴起点
                            // posBegin[1]：补偿区域Y轴起点
                            // posBegin[2]：补偿区域Z轴起点

    double step[3];         // step[0]：补偿区域X轴方向补偿点间距
                            // step[1]：补偿区域Y轴方向补偿点间距
                            // step[2]：补偿区域Z轴方向补偿点间距

    double reserve1[4];     // 保留参数，必须为0
} TCompensate3DTable;

typedef struct Compensate3DPoint
{
    long pointIndex[3];     // pointIndex[0]：补偿点X轴索引
                            // pointIndex[1]：补偿点Y轴索引
                            // pointIndex[2]：补偿点Z轴索引

    long pad1;              // 保留参数，必须为0
    double pointValue;      // 补偿值
} TCompensate3DPoint;

typedef struct Compensate3DPointRange
{
    long rangeIndex[3];     // rangeIndex[0]：X轴方向回读补偿点起始点索引
                            // rangeIndex[1]：Y轴方向回读补偿点起始点索引
                            // rangeIndex[2]：Z轴方向回读补偿点起始点索引

    long rangeCount[3];     // rangeCount[0]：X轴方向回读补偿点数量
                            // rangeCount[1]：Y轴方向回读补偿点数量
                            // rangeCount[2]：Z轴方向回读补偿点数量
} TCompensate3DPointRange;

GT_API GTN_SetCompensate3D(short core, short axis, TCompensate3D* pCompensate3D);
GT_API GTN_GetCompensate3D(short core, short axis, TCompensate3D* pCompensate3D);
GT_API GTN_SetCompensate3DTable(short core, short tableIndex, TCompensate3DTable* pTable,TCompensate3DPoint *pPointArray,long pointCount,short extend);
GT_API GTN_GetCompensate3DTable(short core, short tableIndex, TCompensate3DTable* pTable,TCompensate3DPoint *pPointArray,TCompensate3DPointRange* pPointRange,short *pExtend);
GT_API GTN_GetCompensate3DValue(short core, short axis, double *pValue);

typedef struct FirCompensateParameter
{
	uint16_t sourceType;        // 数据源类型
	uint16_t sourceIndex;       // 数据源索引

	uint32_t filterTime;        // 滤波时间，单位ms

	uint16_t filterMode;        // 0：不滤波；1：fir；2：fir + compensate
	uint16_t reserve1[3];       // 保留参数，必须为0

	double reserve2[4];         // 保留参数，必须为0
}TFirCompensateParameter;

typedef struct FirCompensateValue
{
    double posRaw;                             // 原始位置
    double posFir;                             // 滤波后位置
    double posFirCompensate;                   // “滤波+补偿”后位置

    double velRaw;                             // 原始速度
    double velFir;                             // 滤波后速度
    double velFirCompensate;                   // “滤波+补偿”后速度
}TFirCompensateValue;

GT_API GTN_SetFirCompensateParameter(short core,uint16_t index,TFirCompensateParameter *pPrm);
GT_API GTN_GetFirCompensateParameter(short core,uint16_t index,TFirCompensateParameter *pPrm);
GT_API GTN_ClearFirCompensate(short core);
GT_API GTN_GetFirCompensateValue(short core,uint16_t index,TFirCompensateValue* pValue);

typedef struct TrackParameter
{
    uint16_t enable;                    // 使能Track模式
    uint16_t mode;                      // 工作模式，0：梯形速度曲线；
    uint16_t sourceType;                // 0：追踪位置来自用户指令；1：追踪位置来自编码器
    uint16_t sourceIndex;               // 追踪位置来自编码器时的编码器索引

    uint16_t estimate;                  // 0：关闭追踪位置预估；1：打开追踪位置预估
    uint16_t pad1[3];                   // 保留参数，必须为0

    double sampleTime;                  // 采样时间，单位ms，必须大于规划周期的2倍

    double smoothCoef;                  // 平滑系数，[0,1),0表示关闭平滑

    double reserve1[4];                 // 保留参数，必须为0
}TTrackParameter;

GT_API GTN_PrfTrack(int16_t core,int16_t profile);
GT_API GTN_SetTrackParameter(int16_t core, int16_t profile, TTrackParameter* pPrm);
GT_API GTN_GetTrackParameter(int16_t core,int16_t profile,TTrackParameter *pPrm);
GT_API GTN_SetTrackPosition(int16_t core,int16_t profile,double pos);


typedef struct ExactStopPrm
{
    short enable;             // 准停功能使能标志，只能为0或者1。
    short reserve1[3]; 		  // 保留，必须为0
    double stopPos;			  // 准停位置，单位：度
    double reserve2[8]; 	  // 保留，必须为0
}TExactStopPrm;

GT_API GTN_SetAxisExactStopPrm(short core, short axis, short mode, TExactStopPrm* pPrm, TListInfo* pListInfo = NULL);
GT_API GTN_GetAxisExactStopPrm(short core, short axis, short* pMode, TExactStopPrm* pPrm);

typedef struct RnMailSlaveDeviceCfg
{
    unsigned short cfgSize;
    unsigned short pduWrEn;
    unsigned short pduWrSize;
    unsigned short pduRdEn;
    unsigned short pduRdSize;
    unsigned short cmdEn;
    unsigned short fifoWrEn;
    unsigned short fifoRdEn;
}StRnMailSlaveDeviceCfg;

typedef struct RnMailSlaveDeviceUpDateParam
{
    unsigned short cfgSize;
    unsigned short pduWrEn;
    unsigned short pduRdEn;
    unsigned short cmdEn;
    unsigned short fifoWrEn;
    unsigned short fifoRdEn;
}StRnMailSlaveDeviceUpDateParam;

/**
 * @brief  通讯设置。
 * @param cardIndex 卡号。
 * @param stationPhyId 物理站ID。
 * @param pParam 参数。
 * @return
*/
GT_API GTN_RN_MailSlaveDeviceCfg(short cardIndex, short stationPhyId, unsigned short* pParam);
/**
 * @brief  更新通讯。
 * @param cardIndex 卡号。
 * @param stationPhyId 物理站ID。
 * @param pParam 参数，NULL表示更新所有通讯方式。
 * @return
*/
GT_API GTN_RN_MailSlaveDeviceUpdate(short cardIndex, short stationPhyId, unsigned short* pParam);

/**
 * @brief  写段数据。
 * @param cardIndex 卡号。
 * @param stationPhyId 物理站ID。
 * @param pData 写的数据。
 * @param wordOffset 段起始地址的偏移。
 * @param wordNum 要写的个数。
 * @param pWordNumValid 实际写的个数。
 * @return
*/
GT_API GTN_RN_MailSlaveDeviceWrSeg(short cardIndex, short stationPhyId, unsigned short* pData, unsigned short wordOffset, unsigned short wordNum, unsigned short* pWordNumValid);
/**
 * @brief  读段数据。
 * @param cardIndex 卡号。
 * @param stationPhyId 物理站ID。
 * @param pData 读到的段数据。
 * @param wordOffset 段起始地址的偏移。
 * @param wordNum 要读的个数。
 * @param pWordNumValid 实际读的个数。
 * @return
*/
GT_API GTN_RN_MailSlaveDeviceRdSeg(short cardIndex, short stationPhyId, unsigned short* pData, unsigned short wordOffset, unsigned short wordNum, unsigned short* pWordNumValid);
/**
 * @brief  写入请求数据。
 * @param cardIndex 卡号。
 * @param stationPhyId 物理站ID。
 * @param pData 写的数据。
 * @param wordNum 要写的个数。
 * @param pWordNumValid 实际写的个数。
 * @return
*/
GT_API GTN_RN_MailSlaveDeviceWrCmd(short cardIndex, short stationPhyId, unsigned short* pData, unsigned short wordNum, unsigned short* pWordNumValid);
/**
 * @brief  读取应答数据。
 * @param cardIndex 卡号。
 * @param stationPhyId 物理站ID。
 * @param pData 读到的数据。
 * @param wordNum 要读的个数。
 * @param pWordNumValid 实际读到的个数。
 * @param pCmdRtn 应答模式的返回值。
 * @return
*/
GT_API GTN_RN_MailSlaveDeviceRdCmd(short cardIndex, short stationPhyId, unsigned short* pData, unsigned short wordNum, unsigned short* pWordNumValid, unsigned short* pCmdRtn);
/**
 * @brief  写块区数据。
 * @param cardIndex 卡号。
 * @param stationPhyId 物理站ID。
 * @param pData 写的数据。
 * @param wordNum 要写的个数。
 * @param pWordNumValid 实际写的个数。
 * @return
*/
GT_API GTN_RN_MailSlaveDeviceWrBlock(short cardIndex, short stationPhyId, unsigned short* pData, unsigned short wordNum, unsigned short* pWordNumValid);
/**
 * @brief  读块区数据。
 * @param cardIndex 卡号。
 * @param stationPhyId 物理站ID。
 * @param pData 读到的数据。
 * @param wordNum 要读的个数。
 * @param pWordNumValid 实际读到的个数。
 * @return
*/
GT_API  GTN_RN_MailSlaveDeviceRdBlock(short cardIndex, short stationPhyId, unsigned short* pData, unsigned short wordNum, unsigned short* pWordNumValid);
/**
 * @brief  获取写的块区状态。
 * @param cardIndex 卡号。
 * @param stationPhyId 物理站ID。
 * @param pValidWordNum 写的块区有效(未下发)个数。
 * @param pRemainsWordSpase 写的块区剩余个数。
 * @return
*/
GT_API GTN_RN_MailSlaveDeviceGetWrBlockStatus(short cardIndex, short stationPhyId, unsigned short* pValidWordNum, unsigned short* pRemainsWordSpase);
/**
 * @brief  获取读的块区状态。
 * @param cardIndex 卡号。
 * @param stationPhyId 物理站ID。
 * @param pValidWordNum 读的块区有效(未下发)个数。
 * @param pRemainsWordSpase 读的块区剩余个数。
 * @return
*/
GT_API GTN_RN_MailSlaveDeviceGetRdBlockStatus(short cardIndex, short stationPhyId, unsigned short* pValidWordNum, unsigned short* pRemainsWordSpase);

/**
 * @brief  获取应答模式状态机。
 * @param cardIndex 卡号。
 * @param stationPhyId 物理站ID。
 * @param pCmdStatus 应答模式的状态机。
 * @param pCmdRtn 应答模式的返回值。
 * @return
*/
GT_API GTN_RN_MailSlaveDeviceGetCmdStatus(short cardIndex, short stationPhyId, unsigned short* pCmdStatus, unsigned short* pCmdRtn);

/**
 * @brief  获取输入IO功能配置。
 * @param cardIndex 卡号。
 * @param stationPhyId 物理站ID。
 * @param axis 物理轴号，从0开始。
 * @param inputIndex 输入IO序号，取值范围[0,15]。
 * @param pFuncIndex IO功能定义：
                     0： OUTPUT_FUNC_NULL,
                     1： OUTPUT_FUNC_ALARM,
                     2： OUTPUT_FUNC_MOVING,
                     3： OUTPUT_FUNC_HOME_FINISH,
                     4： OUTPUT_FUNC_VEL_ACHIEVE,
                     5： OUTPUT_FUNC_PHASE_SEARCH_FINISH,
                     6： OUTPUT_FUNC_SERVO_READY,
                     7： OUTPUT_FUNC_INDEX_SEARCHED,
                     8： OUTPUT_FUNC_RESERVED,
                     9： OUTPUT_FUNC_CURRENT_OVER_LIMIT,
                     10：OUTPUT_FUNC_BRAKE,
                     11：OUTPUT_FUNC_POS_FIXED,
                     12：OUTPUT_FUNC_IO_POS_0,
                     13：OUTPUT_FUNC_IO_POS_1,
                     14：OUTPUT_FUNC_IO_POS_2,
                     15：OUTPUT_FUNC_IO_POS_3,
                     16：OUTPUT_FUNC_IO_POS_4,
                     17：OUTPUT_FUNC_IO_POS_5,
                     18：OUTPUT_FUNC_CURRENT_LEVEL,
                     19：OUTPUT_FUNC_LIMIT_WARNING
 * @param pReverse 此IO是否取反。0：未取反,1：取反
 * @return
*/
GT_API GTN_RN_GetOutputFunc(short cardIndex, short stationPhyId, short axis, short outputIndex, short* pFuncIndex, bool* pReverse);
/**
 * @brief  获取输入IO功能配置。
 * @param cardIndex 卡号。
 * @param stationPhyId 物理站ID。
 * @param axis 物理轴号，从0开始。
 * @param inputIndex 输入IO序号，取值范围[0,15]。
 * @param pFuncIndex IO功能定义。
                     0： INPUT_FUNC_NULL,
                     1： INPUT_FUNC_SERVO_ON,
                     2： INPUT_FUNC_STOP,
                     3： INPUT_FUNC_CLR_ALARM,
                     4： INPUT_FUNC_MODE_SWITCH,
                     5： INPUT_FUNC_FIX_POS,
                     6： INPUT_FUNC_HOME,
                     7： INPUT_FUNC_POSITIVE_LIMIT,
                     8： INPUT_FUNC_NEGATIVE_LIMIT,
                     9： INPUT_FUNC_HOME_SWITCH,
                     10：INPUT_FUNC_IO_POS_MOTION_0,
                     11：INPUT_FUNC_IO_POS_MOTION_1,
                     12：INPUT_FUNC_IO_POS_MOTION_2,
                     13：INPUT_FUNC_IO_POS_MOTION_3,
                     14：INPUT_FUNC_IO_POS_MOTION_4,
                     15：INPUT_FUNC_IO_POS_MOTION_5,
                     16：INPUT_FUNC_IO_START_POS_MOTION,
                     17：INPUT_FUNC_IO_TASK_0,
                     18：INPUT_FUNC_IO_TASK_1,
                     19：INPUT_FUNC_IO_TASK_2,
                     20：INPUT_FUNC_IO_TASK_3,
                     21：INPUT_FUNC_IO_MOTION_PAUSE_OR_RESUME,
                     22：INPUT_FUNC_PID_SWITCH
 * @param pReverse 此IO是否取反。0：未取反,1：取反
 * @return
*/
GT_API GTN_RN_GetInputFunc(short cardIndex, short stationPhyId, short axis, short inputIndex, short* pFuncIndex, bool* pReverse);
/**
 * @brief  获取编码器输入分辨率。
 * @param cardIndex 卡号。
 * @param stationPhyId 物理站ID。
 * @param axis 物理轴号，从0开始。
 * @param pValue 编码器输出分辨率。
 * @return
*/
GT_API GTN_RN_GetGlink2InputResolution(short cardIndex, short stationPhyId, short axis, unsigned long* pValue);
/**
 * @brief  设置编码器输入分辨率。
 * @param cardIndex 卡号。
 * @param stationPhyId 物理站ID。
 * @param axis 物理轴号，从0开始。
 * @param value 编码器输出分辨率。
 * @return
*/
GT_API GTN_RN_SetGlink2InputResolution(short cardIndex, short stationPhyId, short axis, unsigned long value);

/**
 * @brief  设置编码器滤波参数。
 * @param core 核号。
 * @param type 编码器类型。
 * @param encoder 编码器序号。
 * @param filterTime 滤波时间，us。
 * @return
*/
GT_API GTN_SetTerminalEncoderFilterTime(short core, short type, short encoder, double filterTime);
/**
 * @brief 读取编码器滤波参数。
 * @param core 核号。
 * @param station 逻辑站号，由于需要mailBox操作，需要传逻辑站号。
 * @param type 编码器类型。
 * @param encoder 编码器序号。
 * @param filterTime 滤波时间，us。
 * @return
*/
GT_API GTN_GetTerminalEncoderFilterTime(short core, short type, short encoder, double* pFilterTime);

/**
 * @brief  读取单通道采集数据(其中编码器值为增量值)。
 * @param cardIndex 卡号。
 * @param stationPhyId 物理站ID。
 * @param varIdndex 需要读取的变量索引(从1开始，取值由GTN_RN_SamplingAddVar指令添加的顺序决定)。
 * @param pBuffer 读取到数据存放的数组。
 * @param bufSize 需要读取的数据个数。
 * @param pReadCount 实际返回的读取个数。
 * @return
*/
GT_API GTN_RN_SamplingReadDataEx(short cardIndex, short stationPhyId, short varIdndex, double* pBuffer, unsigned long bufSize, unsigned long* pReadCount);
/**
 * @brief  打印所有采集数据到文件(打印前需关闭采集, 其中编码器值为增量值)。
 * @param cardIndex 卡号。
 * @param stationPhyId 物理站ID。
 * @param pFileName 打印输出的文件路径。
 * @param startIndex 打印的起始地址，取值范围>=0。
 * @param printCount 需要打印的个数，取值范围>0。
 * @return
*/
GT_API GTN_RN_SamplingPrintDataEx(short cardIndex, short stationPhyId, const char* pFileName, unsigned long startIndex, unsigned long printCount);
/**
 * @brief 多通道采集初始化。
 * @param cardIndex 卡号。
 * @param group 初始化第几组采集，取值范围:1、当samplMode=0时，group取值[1,2]
                2、当samplMode=1时，group取值[1,n](暂未实现)。
 * @param samplMode 采样模式。0：独立模式。1：复用模式(暂未实现)
 * @return
*/
GT_API GTN_RN_MultiSamplingInit(short cardIndex, short group, short samplMode);
/**
 * @brief 开始采集。
 * @param cardIndex 卡号。
 * @param group 初始化第几组采集，取值范围:1、当samplMode=0时，group取值[1,2]
                2、当samplMode=1时，group取值[1,n](暂未实现)。
 * @return
*/
GT_API GTN_RN_MultiSamplingOn(short cardIndex, short group);
/**
 * @brief 结束采集。
 * @param cardIndex 卡号。
 * @param group 初始化第几组采集，取值范围:1、当samplMode=0时，group取值[1,2]
                2、当samplMode=1时，group取值[1,n](暂未实现)。
 * @return
*/
GT_API GTN_RN_MultiSamplingOff(short cardIndex, short group);
/**
 * @brief 打印采集数据到文件(打印前需关闭采集, 其中编码器值为寄存器原始值)。
 * @param cardIndex 卡号。
 * @param group 初始化第几组采集，取值范围:1、当samplMode=0时，group取值[1,2]
                2、当samplMode=1时，group取值[1,n](暂未实现)。
 * @param pFileName 打印输出的文件路径。
 * @param startIndex 打印的起始地址，取值范围>=0。
 * @param printCount 需要打印的个数，取值范围>0。
 * @return
*/
GT_API GTN_RN_MultiSamplingPrint(short cardIndex, short group, const char* pFileName, unsigned long startIndex, unsigned long printCount);
/**
 * @brief 添加数据采集变量。
 * @param cardIndex 卡号。
 * @param group 初始化第几组采集，取值范围:1、当samplMode=0时，group取值[1,2]
                2、当samplMode=1时，group取值[1,n](暂未实现)。
 * @param stVar 数据类型。
 * @return
*/
GT_API GTN_RN_MultiSamplingAddVar(short cardIndex, short group, StSamplingVar stVar);
/**
 * @brief 读取单个采集数据(其中编码器值为寄存器原始值)。
 * @param cardIndex 卡号。
 * @param group 初始化第几组采集，取值范围:1、当samplMode=0时，group取值[1,2]
                2、当samplMode=1时，group取值[1,n](暂未实现)。
 * @param varIdndex 需要读取的变量索引(从1开始，取值由GTN_RN_MultiSamplingAddVar指令添加的顺序决定)。
 * @param pBuffer 读取到数据存放的数组。
 * @param bufSize 需要读取的数据个数。
 * @param pReadCount 实际返回的读取个数。
 * @return
*/
GT_API GTN_RN_MultiSamplingRead(short cardIndex, short group, short varIdndex, double* pBuffer, unsigned long bufSize, unsigned long* pReadCount);
/**
 * @brief 清除状态及采样信息。
 * @param cardIndex 卡号。
 * @param group 初始化第几组采集，取值范围:1、当samplMode=0时，group取值[1,2]
                2、当samplMode=1时，group取值[1,n](暂未实现)。
 * @param mode 取值：1：清除GTN_RN_MultiSamplingAddVar添加的采集变量。

 * @return
*/
GT_API GTN_RN_MultiSamplingClear(short cardIndex, short group, short mode);
/**
 * @brief 读取单个采集数据(其中编码器值为增量值)。
 * @param cardIndex 卡号。
 * @param group 初始化第几组采集，取值范围:1、当samplMode=0时，group取值[1,2]
                2、当samplMode=1时，group取值[1,n](暂未实现)。
 * @param varIdndex 需要读取的变量索引(从1开始，取值由GTN_RN_MultiSamplingAddVar指令添加的顺序决定)。
 * @param pBuffer 读取到数据存放的数组。
 * @param bufSize 需要读取的数据个数。
 * @param pReadCount 实际返回的读取个数。
 * @return
*/
GT_API GTN_RN_MultiSamplingReadEx(short cardIndex, short group, short varIdndex, double* pBuffer, unsigned long bufSize, unsigned long* pReadCount);
/**
 * @brief 读取单个采集数据(其中编码器值为增量值)。
 * @param cardIndex 卡号。
 * @param group 初始化第几组采集，取值范围:1、当samplMode=0时，group取值[1,2]
                2、当samplMode=1时，group取值[1,n](暂未实现)。
 * @param varIdndex 需要读取的变量索引(从1开始，取值由GTN_RN_MultiSamplingAddVar指令添加的顺序决定)。
 * @param pBuffer 读取到数据存放的数组。
 * @param bufSize 需要读取的数据个数。
 * @param pReadCount 实际返回的读取个数。
 * @return
*/
GT_API GTN_RN_MultiSamplingPrintEx(short cardIndex, short group, const char* pFileName, unsigned long startIndex, unsigned long printCount);
/**
 * @brief 获取采集信息。
 * @param cardIndex 卡号。
 * @param group 初始化第几组采集，取值范围:1、当samplMode=0时，group取值[1,2]
                2、当samplMode=1时，group取值[1,n](暂未实现)。
 * @param infoType 需要获取的信息类型：
                   0: 采集使能信息
                   1：采集到的所有数据数量（包含包格式）
                   2：采集到的单通道数据量（纯数据量）
                   3：获取丢包数量
                   4：获取FIFO溢出状态
 * @param pInfo 获取到的信息。
 * @return
*/
GT_API  GTN_RN_MultiSamplingGetInfo(short cardIndex, short group, short infoType, unsigned long *pInfo);
/**
 * @brief  设置当前编码器增量值（调用GTN_RN_HighSpeedSamplingOnAll指令前设置）。
 * @param core 核号。
 * @param encoder 编码器序号。
 * @param pos 编码器位置。
 * @return
*/
GT_API GTN_SamplingSetEncPos(short core, short encoder, double pos);
/**
 * @brief  设置编码器方向（调用GTN_RN_HighSpeedSamplingOnAll指令前设置）。
 * @param core 核号。
 * @param encoder 编码器序号。
 * @param dir 写入的编码器方向。0：反转1：正转。
 * @return
*/
GT_API GTN_SamplingSetEncDir(short core, short encoder, short dir);

/**
 * @brief  设置当前辅助编码器增量值（调用GTN_RN_HighSpeedSamplingOnAll指令前设置）。
 * @param core 核号。
 * @param auEncoderType 辅助编码器类型。
 * @param auEncoder 辅助编码器序号。
 * @param pValue 辅助编码器位置。
 * @return
*/
GT_API GTN_SamplingSetAuEncPos(short core,short auEncoderType,short auEncoder,double pos);
/**
 * @brief  设置辅助编码器方向（调用GTN_RN_HighSpeedSamplingOnAll指令前设置）。
 * @param core 核号。
 * @param auEncoderType 辅助编码器类型。
 * @param encoder 辅助编码器序号。
 * @param dir 辅助编码器方向。
 * @return
*/
GT_API GTN_SamplingSetAuEncDir(short core,short auEncoderType,short auEncoder,short dir);
/**
 * @brief  读取驱动器的伺服状态值。（轴上新增硬件输入信号）。
 * @param core 核号。
 * @param axis 起始轴号。
 * @param pSts 伺服状态输入值。bit0：伺服准备就绪信号，硬件为GTM轴子板新版的轴硬件口PIN-21。
                          bit1：伺服使能完成信号，硬件为GTM轴子板新版的轴硬件口PIN-3。有硬件输入信号时，0：输入无效。1：输入有效。
 * @param count 读取的轴数。
 * @param pClock 读取的控制卡时钟。
 * @return
*/
GT_API GTN_GetServoStatus(short core, short axis, long* pSts, short count, unsigned long* pClock);
/**
 * @brief   打开驱动器实际上使能状态，在GT_GetSts的bit9表示实际驱动器上使能状态
 * @param core 核号
 * @param axis 轴号从1开始，指开启该功能的轴
 * @param diType Di资源类型
 * @param diIndex Di资源的索引,从1开始
 * @return 指令返回值列表 7:参数错误 轴或Di索引越界
 */
GT_API GTN_ServoReadyOn(short core, short axis, short diType, short diIndex);
/**
 * @brief   关闭驱动器实际上使能状态，GT_GetSts的bit9表示GT_AxisEnable的指令状态
 * @param core 核号
 * @param axis 轴号从1开始，指开启该功能的轴
 * @return 指令返回值列表 7:参数错误 轴索引越界
 */
GT_API GTN_ServoReadyOff(short core, short axis);


//-------------------------------------------------------------------
// Waveform：波形控制功能
//-------------------------------------------------------------------

// 波形控制工作模式
#define WAVEFORM_WORK_MODE_DEFAULT                         (0)       // 默认模式，一路输出，是能后立即输出
#define WAVEFORM_WORK_MODE_LINK_POS_COMPARE                (1)       // 关联位置比较模式，一路输出，当前站上第一路位置比较输出一个点才输出一个波形

typedef struct WaveformParameter
{
     double time;                      // 波形控制时间轴，单位：ms。波形控制功能使能的时刻时间轴为0。
     double value;                     // 波形控制目标能量，含义和取值范围取决于输出的类型。
                                       // 目前仅支持输出模拟量，取值范围：[-10,10]V
 }TWaveformParameter;                  // 波形控制参数结构体

typedef struct WaveformOutput
{
    short type;                        // 波形控制能量输出类型，目前仅支持一下类型
                                       // MC_DAC(20)：轴模拟量
                                       // MC_AU_DAC(19)：非轴模拟量
                                       // MC_LASER_AO(72)：激光模拟量
    short index;                       // 波形控制能量输出类型索引，索引从1开始
    short laserOn;                     // 自动控制激光开关光，即波形能量输出时自动开启激光开关信号，输出结束后关闭激光开关信号
    short pad;                         // 保留参数，必须设置成0
}TWaveformOutput;                      // 波形控制输出参数结构体

typedef struct WaveformStatus
{
    short enable;                      // 波形控制使能状态，0：关闭，1：输出
    short outputType;                  // 波形控制能量输出类型，目前仅支持一下类型
                                       // MC_DAC(20)：轴模拟量
                                       // MC_AU_DAC(19)：非轴模拟量
                                       // MC_LASER_AO(72)：激光模拟量
    short outputIndex;                 // 波形控制能量输出类型对应的索引，取值范围取决于对应的类型
    short laserOn;                     // 激光开关光信号
    short loopCount;                   // 已经循环的次数
    short pad[3];                      // 保留参数
    double outputValue;                // 当前输出的能量值
}TWaveformStatus;                      // 波形控制状态参数结构体

/**
 * @brief 下载波形控制数据
 * @param core 核号，索引从1开始
 * @param index 波形控制索引，索引从1开始
 * @param pPrm 波形控制数据结构体数组
 * @param count 波形控制数据个数
 * @param loopCount 波形控制循环次数
 * @return 17051：波形控制数据数量count超限，取值范围：[1,50]
 *         17053：索引参数错误
 *         17055：核号参数错误，目前每张卡只有第一个核支持波形控制功能
 *         17061：循环次数参数错误
 *         17501：波形控制时间参数错误，时间必须为升序
 *         17502：波形控制能量值参数错误，如果输出类型为模拟量，单位：V
 *         11059：波形功能已经使能，不允许重新下载数据
 *         11087：数据Fifo已经压满了，需要减少数据点数量
*/
GT_API GTN_LoadWaveformParameter(short core,short index,TWaveformParameter *pPrm,short count,short loopCount);

/**
 * @brief 使能波形控制开关
 * @param core 核号，索引从1开始
 * @param index 波形控制索引，索引从1开始
 * @param pOutput 波形控制能量输出参数
 * @param enable 波形控制使能，0：关闭，1-打开
 * @return 17053：索引参数错误
 *         17054：使能参数错误，取值范围：[0,1]
 *         17055：核号参数错误，目前每张卡只有第一个核支持波形控制功能
 *         17100：波形控制能量输出结构体TWaveformOutput，pad参数错误，必须为0
 *         17503：波形控制能量输出结构体TWaveformOutput，type参数错误
 *         17504：波形控制能量输出结构体TWaveformOutput，index参数错误
 *         17505：波形控制能量输出结构体TWaveformOutput，laserOn参数错误
 *         11050：波形功能正在工作，不允更改输出类型和索引
 *         11055：波形功能工作模式不对，不允使能，检查Waveform和WaveformPair指令是否混用
 *         11059：波形功能已经使能，不允许更改输出类型和索引
 *         11091：网络中没有接带波形控制功能的从站
 *         11501：发送数据锁开启，不允许使能波形控制
 *         11502：波形控制输出类型资源没有映射
 *         11503：波形控制功能资源和输出类型资源不在同一个从站上
 *         11504：波形控制数据为0，不允许使能波形控制
*/
GT_API GTN_EnableWaveform(short core,short index,TWaveformOutput *pOutput,short enable);

/**
 * @brief 获取波形控制功能状态
 * @param core 核号，索引从1开始
 * @param index 波形控制索引，索引从1开始
 * @param pStatus 波形控制功能状态结构体指针
 * @return 17053：索引参数错误
 *         17055：核号参数错误，目前每张卡只有第一个核支持波形控制功能
*/
GT_API GTN_GetWaveformStatus(short core,short index,TWaveformStatus *pStatus);

/**
 * @brief 波形控制开关使能
 * @param core 核号，索引从1开始
 * @param crd 坐标系号，索引从1开始
 * @param index 波形控制索引号，索引从1开始
 * @param pOutput 波形控制输出信号参数结构体指针
 * @param enable 波形控制使能信号，取值范围：[0,1]
 * @param modal 模态参数，取值范围：[0,1]
 * @param fifo 插补坐标系缓冲区号，取值范围：[0,1]
 * @return
*/
GT_API GTN_BufEnableWaveform(short core,short crd,short index,TWaveformOutput *pOutput,short enable,short modal,short fifo);

/**
 * @brief 波形控制开关使能
 * @param core 核号，索引从1开始
 * @param crd 坐标系号，索引从1开始
 * @param index 波形控制索引号，索引从1开始
 * @param pOutput 波形控制输出信号参数结构体指针
 * @param enable 波形控制使能信号，取值范围：[0,1]
 * @param modal 模态参数，取值范围：[0,1]
 * @param fifo 插补坐标系缓冲区号，取值范围：[0,1]
 * @return
*/
GT_API GTN_BufEnableWaveformEx(short core,short crd,short index,TWaveformOutput *pOutput,short enable,short modal,short fifo);

/**
 * @brief 设置波形控制功能模式
 * @param core 核号，索引从1开始
 * @param index 波形控制索引，索引从1开始
 * @param pOutput 波形控制能量输出参数
 * @param enable 波形控制使能，0：关闭，1-打开
 * @return 17053：索引参数错误
 *         17055：核号参数错误，目前每张卡只有第一个核支持波形控制功能
 *         17056：模式参数错误
 *         11059：波形功能已经使能，不允许更改模式
 *         4：当前控制卡固件型号不支持设置的模式，需要更换成更高级功能的卡固件
*/
GT_API GTN_SetWaveformMode(short core,short index,short mode);

/**
 * @brief 获取波形控制功能模式
 * @param core 核号，索引从1开始
 * @param index 波形控制索引，索引从1开始
 * @param pOutput 波形控制能量输出参数
 * @param enable 波形控制使能，0：关闭，1-打开
 * @return 17053：索引参数错误
 *         17055：核号参数错误，目前每张卡只有第一个核支持波形控制功能
*/
GT_API GTN_GetWaveformMode(short core,short index,short *pMode);


//-------------------------------------------------------------------
// WaveformPair：波形控制两路输出功能
// （1）第一路输出为矩形波，描述参数为幅值、低电平时间和高电平时间
// （2）第二路输出为任意波形，描述参数为幅值，每个幅值依次与矩形波的上/下升沿对应
// （3）允许单独设置某一路输出的控制权，即在波形输出过程中，是否按波形控制模块输出值输出
//-------------------------------------------------------------------

#define RECTANGLE_WAVE_START_MODE_LOW_LEVEL          (0)   // 矩形波输出模式，启动输出后先输出低电平，再输出高电平
#define RECTANGLE_WAVE_START_MODE_HIGH_LEVEL         (1)   // 矩形波输出模式，启动输出后先输出高电平，再输出低电平

typedef struct RectangleWavePrm
 {
    short startMode;                   // 矩形波输出模式，定义启动输出后的行为：
                                       // 0：启动输出后先输出低电平，再输出高电平
                                       // 1：启动输出后先输出高电平，再输出低电平
    short pad[3];                      // 对齐
    double voltage;                    // 矩形波输出高电平对应的电压值，取值范围：(0,10]，单位：V
    double highLevelTime;              // 矩形波输出高电平（输出值为voltage）保持时间，必须大于0，单位：ms
    double lowLevelTime;               // 矩形波输出低电平（输出值为0V）保持时间，必须大于0，单位：ms
}TRectangleWavePrm;                    // 波形输出矩形波参数结构体

typedef struct RectangleWaveDutyMode
{
    short startMode;                   // 矩形波输出模式，定义启动输出后的行为：
                                       // 0：启动输出后先输出低电平，再输出高电平
                                       // 1：启动输出后先输出高电平，再输出低电平
    short pad[3];                      // 对齐
    double voltage;                    // 矩形波输出高电平对应的电压值，取值范围：(0,10]，单位：V
    double frequency;                  // 矩形波输出频率，取值范围：(0,100)，单位：kHz
    double dutyRatio;                  // 矩形波输出占空比，即低电平时间占总周期时间的比例，取值范围：(0,100)，单位：%
}TRectangleWaveDutyMode;               // 波形输出矩形波占空比描述结构体

typedef struct StageWavePrm
{
    short stepCount;                   // 台阶波输出台阶个数，取值范围：[1,4000]
    short pad[3];                      // 对齐
    double voltageStart;               // 台阶波输出起始电压值，取值范围：(0,10]，单位：V
    double voltageEnd;                 // 台阶波输出终点电压值，取值范围：(0,10]，单位：V
    double aheadTime;                  // 台阶波达到目标台阶值的提前时间，取值范围与矩形波宽度相关，单位：ms
}TStageWavePrm;                        // 波形输出台阶波参数结构体

typedef struct WaveformPairStatus
{
    short enable;                      // 波形控制使能状态，0：关闭，1：输出
    short laserOn;                     // 激光开关光信号
    short loopCount;                   // 已经循环的次数
    short rectOutputType;              // 矩形波输出类型
    short rectOutputIndex;             // 矩形波输出类型的索引
    short rectOutputValid;             // 矩形波输出有效状态
    short outputType;                  // 第二路波形能量输出类型
    short outputIndex;                 // 第二路波形能量输出类型的索引
    short outputValid;                 // 第二路波形能量输出有效状态
    short pad[3];                      // 对齐
    double rectVoltage;                // 矩形波输出电压值
    double outputVoltage;              // 第二路波形能量输出电压值
}TWaveformPairStatus;                  // 波形控制两路输出状态结构体

/**
 * @brief 设置波形控制两路输出波形参数
 * @param core 核号，索引从1开始
 * @param index 波形控制索引，索引从1开始
 * @param pRectWave 矩形波波形参数结构体指针
 * @param pVoltageData 第二路波形输出电压值数据数组，数组大小为dataCount。每个数据输出时间按顺序与矩形波的上升和下降沿对齐
 * @param dataCount 第二路波形输出电压值数据数组大小
 * @param loopCount 循环输出次数
 * @return 17051：波形控制数据数量dataCount超限，取值范围：[1,50]
 *         17053：索引参数错误
 *         17055：核号参数错误，目前每张卡只有第一个核支持波形控制功能
 *         17061：循环次数参数错误
 *         17100：矩形波结构体TRectangleWavePrm成员变量pad参数错误，必须为0
 *         17502：波形控制能量值pVoltageData参数错误
 *         17510：矩形波结构体TRectangleWavePrm成员变量startMode参数错误
 *         17511：矩形波结构体TRectangleWavePrm成员变量voltage参数错误
 *         17512：矩形波结构体TRectangleWavePrm成员变量highLevelTime参数错误
 *         17513：矩形波结构体TRectangleWavePrm成员变量lowLevelTime参数错误
 *         11059：波形功能已经使能，不允许重新下载数据
 *         11087：数据Fifo已经压满了，需要减少数据点数量
*/
GT_API GTN_SetWaveformPairPrm(short core,short index,TRectangleWavePrm *pRectWave,double *pVoltageData,short dataCount,short loopCount);

/**
 * @brief 设置波形控制两路输出提前模式波形参数
 * @param core 核号，索引从1开始
 * @param index 波形控制索引，索引从1开始
 * @param pRectWave 矩形波波形占空比模式参数结构体指针
 * @param pStageWave 台阶波波形参数结构体指针
 * @param loopCount 循环输出次数
 * @return 17053：索引参数错误
 *         17055：核号参数错误，目前每张卡只有第一个核支持波形控制功能
 *         17061：循环次数参数错误
 *         17100：矩形波结构体TRectangleWaveDutyMode成员变量pad参数错误，必须为0
 *         17100：台阶波结构体TStageWavePrm成员变量pad参数错误，必须为0
 *         17510：矩形波结构体TRectangleWaveDutyMode成员变量startMode参数错误
 *         17511：矩形波结构体TRectangleWaveDutyMode成员变量voltage参数错误
 *         17515：矩形波结构体TRectangleWaveDutyMode成员变量frequency参数错误
 *         17516：矩形波结构体TRectangleWaveDutyMode成员变量dutyRatio参数错误
 *         17517：台阶波结构体TStageWavePrm成员变量stepCount参数错误
 *         17518：台阶波结构体TStageWavePrm成员变量voltageStart参数错误
 *         17519：台阶波结构体TStageWavePrm成员变量voltageEnd参数错误
 *         17520：台阶波结构体TStageWavePrm成员变量aheadTime参数错误
 *         17521：台阶波单个台阶增量过小，必须大于0.0003V
 *         17522：台阶波爬台阶时间小于10us
 *         11059：波形功能已经使能，不允许重新下载数据
*/
GT_API GTN_SetWaveformPairAhead(short core,short index,TRectangleWaveDutyMode *pRectWave,TStageWavePrm *pStageWave,short loopCount);

/**
 * @brief 使能波形控制两路输出开关
 * @param core 核号，索引从1开始
 * @param index 波形控制索引，索引从1开始
 * @param pRectOutput 矩形波输出参数结构体指针
 * @param pOutput2 第二路波输出参数结构体指针
 * @param enable 波形控制使能，0：关闭，1-打开
 * @return 17053：索引参数错误
 *         17054：使能参数错误，取值范围：[0,1]
 *         17055：核号参数错误，目前每张卡只有第一个核支持波形控制功能
 *         17100：波形控制能量输出结构体TWaveformOutput，pad参数错误，必须为0
 *         17503：波形控制第二路能量输出结构体TWaveformOutput，type参数错误
 *         17504：波形控制第二路能量输出结构体TWaveformOutput，index参数错误
 *         17505：波形控制第二路能量输出结构体TWaveformOutput，laserOn参数错误
 *         17507：波形控制矩形波能量输出结构体TWaveformOutput，type参数错误
 *         17508：波形控制矩形波能量输出结构体TWaveformOutput，index参数错误
 *         17509：波形控制矩形波能量输出结构体TWaveformOutput，laserOn参数错误
 *         11050：波形功能正在工作，不允许使能
 *         11055：波形功能工作模式不对，不允使能，检查Waveform和WaveformPair指令是否混用
 *         11059：波形功能已经使能，不允许更改输出类型和索引
 *         11087：波形控制数据Fifo已经满了，用户数据过多
 *         11091：网络中没有接带波形控制功能的从站
 *         11502：波形控制第二路输出类型资源没有映射
 *         11503：波形控制功能资源和第二路输出类型资源不在同一个从站上
 *         11504：波形控制数据为0，不允许使能波形控制
 *         11506：波形控制矩形波输出类型资源没有映射
 *         11507：波形控制功能资源和矩形波输出类型资源不在同一个从站上
 *         11508：取数据失败，内部执行错误
*/
GT_API GTN_EnableWaveformPair(short core,short index,TWaveformOutput *pRectOutput,TWaveformOutput *pOutput,short enable);

/**
 * @brief 单独设置硬件输出口为波形控制有效/无效
 * @param core 核号，索引从1开始
 * @param index 波形控制索引，索引从1开始
 * @param pOutput 波形控制能量输出参数结构体指针
 * @param valid 对应的硬件输出口控制权为波形控制功能有效标志
 *              0表示该硬件输出口不按照波形控制功能设置的能量输出
 *              1表示该硬件输出口按照波形控制功能设置的能量输出
 * @return 17053：索引参数错误
 *         17055：核号参数错误，目前每张卡只有第一个核支持波形控制功能
 *         17514：有效标志参数错误，取值范围：[0,1]
 *         11055：波形功能工作模式不对，不允使能，检查Waveform和WaveformPair指令是否混用
 *         11091：没有调用指令GTN_EnableWaveformPair使能波形输出，或者波形输出已经结束
 *         11505：设置的输出类型不在波形控制功能中
*/
GT_API GTN_SetWaveformPairOutputValid(short core,short index,TWaveformOutput *pOutput,short valid);

/**
 * @brief 获取波形控制两路输出功能状态
 * @param core 核号，索引从1开始
 * @param index 波形控制索引，索引从1开始
 * @param pStatus 波形控制两路输出功能状态结构体指针
 * @return 17053：索引参数错误
 *         17055：核号参数错误，目前每张卡只有第一个核支持波形控制功能
*/
GT_API GTN_GetWaveformPairStatus(short core,short index,TWaveformPairStatus *pStatus);

typedef struct DynamicCompensateParameter
{
    unsigned short enable;             // 使能动态误差补偿

    unsigned short dimension;          // 目前只能为2
    unsigned short moveAxis1;          // 运动轴1索引
    unsigned short moveAxis2;          // 运动轴2索引
    unsigned short source;             // 31:用运动轴的规划位置计算补偿量；23：用运动轴的编码器位置计算补偿量

    unsigned short compensateAxis;     // 补偿轴索引

    unsigned short mode;               // 工作模式
                                       // 0：点胶头动态调高模式

    unsigned short pad1;               // 保留参数，必须为0

    double reserve1;                   // 保留参数，必须为0
    double reserve2;                   // 保留参数，必须为0
    double reserve3;                   // 保留参数，必须为0
    double reserve4;                   // 保留参数，必须为0
}TDynamicCompensateParameter;

typedef struct DynamicCompensatePoint
{
    unsigned short pointType;         // 数据点类型，必须为0
    unsigned short pad1;              // 保留参数，必须为0
    unsigned short pad2;              // 保留参数，必须为0
    unsigned short pad3;              // 保留参数，必须为0

    double compensateValue;            // 补偿值
    double compensateDistance;         // 补偿区间长度

    double reserve1;                   // 保留参数，必须为0
    double reserve2;                   // 保留参数，必须为0
    double reserve3;                   // 保留参数，必须为0
    double reserve4;                   // 保留参数，必须为0
}TDynamicCompensatePoint;

typedef struct DynamicCompensateStatus
{
    unsigned short enable;               // 动态补偿使能状态
    unsigned short execute;              // 动态补偿进行中
    unsigned short pad1;                 // 保留参数
    unsigned short pad2;                 // 保留参数

    unsigned long pointReceive;         // 接收到到补偿点数量
    unsigned long pointUse;             // 用过的补偿点数量

    TDynamicCompensatePoint point;      // 当前正在使用的补偿点

    double pos1;                       // 当前位置1
    double pos2;                       // 当前位置2
    double travelDistance;             // 当前走过的合成位移（以上一个补偿点作为起点）
    double compensateValue;            // 当前补偿值

    double reserve1;                   // 保留参数
    double reserve2;                   // 保留参数
    double reserve3;                   // 保留参数
    double reserve4;                   // 保留参数
}TDynamicCompensateStatus;

GT_API GTN_SetDynamicCompensateParameter(short core, short dcIndex, TDynamicCompensateParameter* pPrm);
GT_API GTN_GetDynamicCompensateParameter(short core, short dcIndex, TDynamicCompensateParameter* pPrm);
GT_API GTN_SetDynamicCompensatePoint(short core, short dcIndex, TDynamicCompensatePoint* pPoint);
GT_API GTN_GetDynamicCompensateStatus(short core, short dcIndex, TDynamicCompensateStatus* pStatus);

/**
 * @brief profinet通讯写
 * @param cardIndex 卡号
 * @param stationPhyId 物理站号
 * @param slotIndex 卡槽号，取值范围[1..14]
 * @param pData 写入的数据
 * @param byteNum 写入数据的长度，单位byte，取值范围[0..64]
 * @return
*/
GT_API GTN_RN_WritePNData(short cardIndex, short stationPhyId, short slotIndex, unsigned char* pData, unsigned short byteNum);

/**
 * @brief profinet通讯读
 * @param cardIndex 卡号
 * @param stationPhyId 物理站号
 * @param slotIndex 卡槽号，取值范围[1..14]
 * @param pData 读取的数据
 * @param byteNum 读取数据的长度，单位byte，取值范围[0..64]
 * @return
*/
GT_API GTN_RN_ReadPNData(short cardIndex, short stationPhyId, short slotIndex, unsigned char* pData, unsigned short byteNum);

/**
 * @brief profinet通讯写
 * @param cardIndex 卡号
 * @param stationPhyId 物理站号
 * @param pData 写入的数据
 * @param byteNum 写入数据的长度，单位byte，取值范围[0..250]
 * @return 0表示成功，非0表示失败
*/
GT_API GTN_RN_WritePNDataEx(short cardIndex, short stationPhyId, unsigned char* pData, unsigned short byteNum);
/**
 * @brief profinet通讯读
 * @param cardIndex 卡号
 * @param stationPhyId 物理站号
 * @param pData 读取的数据
 * @param byteNum 读取数据的长度，单位byte，取值范围[0..250]
 * @param pResByteNum 实际读取的长度，单位byte
 * @return 0表示成功，非0表示失败
*/
GT_API GTN_RN_ReadPNDataEx(short cardIndex, short stationPhyId, unsigned char* pData, unsigned short byteNum, unsigned short *pResByteNum);

#define USER_MAX_STATION_NUM_V1          (256)

typedef struct RingInfVer1
{
    unsigned char m_master_flag;//	: 1;
    unsigned char m_port_link_up_a ;//	: 1;
    unsigned char m_port_link_up_b ;//	: 1;
    unsigned char m_id_auto_set	;//	: 1;

    unsigned char m_sync_status;//		: 1;
    unsigned char m_dc_status;//			: 2;

    unsigned char m_device_num;//			: 8;
    unsigned char m_device_id;//			: 8;

    unsigned short m_sync_cycle;//			: 16;
    unsigned short rsvd0;//			: 16;

    unsigned char m_station_id_list[USER_MAX_STATION_NUM_V1] ;
    unsigned char m_station_ready[USER_MAX_STATION_NUM_V1];
    //add by luo.mj
    unsigned long m_crc_ok_cnt_a  ;
    unsigned long m_crc_err_cn_a   ;
    unsigned long m_crc_ok_cn_b   ;
    unsigned long m_crc_err_cn_b   ;
}StRingInfVer1;

/**
 * @brief 获取网络上从站信息,支持256个从站
 * @param cardIndex 卡号
 * @param stationPhyId 物理站号
 * @param StRingInfVer1 从站信息结构体指针
 * @return 0表示成功，非0表示失败
*/
GT_API GTN_RN_GetRemoteInfVer1(short cardIndex,short stationPhyId,StRingInfVer1* pInf);

#define RN_LWR                         (0x0)
#define RN_AWR                         (0x1)     // 绝对寻址写操作
#define RN_LMWR                        (0x2)
#define RN_TCWR                        (0x3)
#define RN_LRD                         (0x4)
#define RN_ARD                         (0x5)     // 绝对寻址读操作
#define RN_LMRD                        (0x6)
#define RN_TCRD                        (0x7)

typedef struct
{
    uint16_t fineCnt                   :9;       // 当前精采样缓冲区数据个数
    uint16_t fineRdFull                :1;       // 当前精采样缓冲区是否已满
    uint16_t rvsd                      :3;       // 保留
    uint16_t ehmiEnMask                :1;       // 1:表示ehminEn被修改
    uint16_t ehmiEn                    :1;       // 1:表示当前设备为ehmi设备
    uint16_t fineRdErr                 :1;       // fifo已空，仍读取精采样数据，发生读取错误。该标志位读清除
    uint16_t roundRdCnt                :10;      // 当前粗采样缓冲区中数据个数
    uint16_t roundRdFull               :1;       // 当前粗采样缓冲区是否已满
    uint16_t roundRemoteRdEnMask       :1;       // 当前值为时，roundRemoteRdEn才会被修改
    uint16_t roundRemoteRdEn           :1;       // 1:粗采样远端读使能
    uint16_t roundWrSrcMask            :1;       // 当前值为时，roundWrSrc才会被修改
    uint16_t roundWrSrc                :1;       // 0:缓存本地粗采样数据；:缓冲来自网络的远端数据
    uint16_t roundRdErr                :1;       // fifo已空，仍读取粗采样数据，发生读取错误。该标志位读清除
}TKsSampleStatus;

/**
 * @brief 网络报文发送，非阻塞模式
 * @param cardIndex 卡号
 * @param stationPhyId 物理站号
 * @param byteAddr 发送的地址，byte地址
 * @param pData 发送的数据数组头指针
 * @param dataNum 发送的数据数组大小，单位：word
 * @param cmd 指令类型：
 *            （1）RN_AWR(0x1)：绝对寻址写
 *            （2）RN_ARD(0x1)：绝对寻址读
 * @param addMode 地址模式：
 *                （1）0：每次访问同一个地址
 *                （2）1：每次访问地址自动加1
 * @param fastEn 快速发送使能
 *               （1）0：普通发送
 *               （2）1：快速发送
 * @return
*/
GT_API GTN_RN_TxNetPacket(int16_t cardIndex,int16_t stationPhyId,uint16_t byteAddr,uint16_t *pData,uint16_t dataNum,uint8_t cmd,uint8_t addMode,uint8_t fastEn);

/**
 * @brief 网络报文接收，非阻塞模式
 * @param cardIndex 卡号
 * @param stationPhyId 物理站号
 * @param byteAddr 接收的地址，byte地址
 * @param pData 接收的数据数组头指针
 * @param dataNum 接收的数据数组大小，单位：word
 * @param cmd 指令类型：
 *            （1）RN_AWR(0x1)：绝对寻址写
 *            （2）RN_ARD(0x1)：绝对寻址读
 * @param addMode 地址模式：
 *                （1）0：每次访问同一个地址
 *                （2）1：每次访问地址自动加1
 * @param fastEn 快速接收使能
 *               （1）0：普通接收
 *               （2）1：快速接收
 * @return 0表示执行成功，非0表示执行失败
*/
GT_API GTN_RN_RxNetPacket(int16_t cardIndex,int16_t stationPhyId,uint16_t byteAddr,uint16_t *pData,uint16_t dataNum,uint8_t cmd,uint8_t addMode,uint8_t fastEn);

/**
 * @brief 设置采样状态
 * @param cardIndex 卡号
 * @param stationPhyId 物理站号
 * @param pStatus 采样状态控制字结构体指针，具体含义参考头文件结构体详细说明
 * @return 0表示执行成功，非0表示执行失败
*/
GT_API GTN_RN_SetKsSampleStatus(int16_t cardIndex,int16_t stationPhyId,TKsSampleStatus *pStatus);

/**
 * @brief 获取采样状态
 * @param cardIndex 卡号
 * @param stationPhyId 物理站号
 * @param pStatus 采样状态控制字结构体指针，具体含义参考头文件结构体详细说明
 * @return 0表示执行成功，非0表示执行失败
*/
GT_API GTN_RN_GetKsSampleStatus(int16_t cardIndex,int16_t stationPhyId,TKsSampleStatus *pStatus);

/**
 * @brief 清除采样状态
 * @param cardIndex 卡号
 * @param stationPhyId 物理站号
 * @return 0表示执行成功，非0表示执行失败
*/
GT_API GTN_RN_SetKsSampleClear(int16_t cardIndex,int16_t stationPhyId);

/**
 * @brief 设置采样掩码
 * @param cardIndex 卡号
 * @param stationPhyId 物理站号
 * @param mask 掩码
 * @return 0表示执行成功，非0表示执行失败
*/
GT_API GTN_RN_SetKsSampleMask(int16_t cardIndex,int16_t stationPhyId,uint32_t mask);

/**
 * @brief 获取采样掩码
 * @param cardIndex 卡号
 * @param stationPhyId 物理站号
 * @param pMask 获取到的掩码
 * @return 0表示执行成功，非0表示执行失败
*/
GT_API GTN_RN_GetKsSampleMask(int16_t cardIndex,int16_t stationPhyId,uint32_t *pMask);

/**
 * @brief 获取采样版本
 * @param cardIndex 卡号
 * @param stationPhyId 物理站号
 * @param pVersion 获取到的采样版本
 * @return 0表示执行成功，非0表示执行失败
*/
GT_API GTN_RN_GetKsSampleVersion(int16_t cardIndex,int16_t stationPhyId,uint32_t *pVersion);

/**
 * @brief 获取精采样数据
 * @param cardIndex 卡号
 * @param stationPhyId 物理站号
 * @param pData 获取到的数据数组头指针
 * @param dWordNums 需要获取的数据个数，单位：dWord，取值范围：[1,508]
 * @return 0表示执行成功，非0表示执行失败
*/
GT_API GTN_RN_GetKsSampleFineData(int16_t cardIndex,int16_t stationPhyId,uint32_t *pData,uint16_t dWordNums);

/**
 * @brief 获取粗采样数据
 * @param cardIndex 卡号
 * @param stationPhyId 物理站号
 * @param pData 获取到的数据数组头指针
 * @param dWordNums 需要获取的数据个数，单位：dWord，取值范围：[1,508]
 * @return 0表示执行成功，非0表示执行失败
*/
GT_API GTN_RN_GetKsSampleRoundData(int16_t cardIndex,int16_t stationPhyId,uint32_t *pData,uint16_t dWordNums);

/*-----------------------------------------------------------*/
/* Lua                                                       */
/*-----------------------------------------------------------*/
#define MOTION_PROGRAM_CLEAR_ALL_TASK          (1)
#define MOTION_PROGRAM_CLEAR_TASK              (2)
#define MOTION_PROGRAM_CLEAR_FILE              (3)

#define MOTION_PROGRAM_MODE_DEBUG              (0)
#define MOTION_PROGRAM_MODE_RELEASE            (1)

typedef struct
{
	long emb_state;
	long emb_curr_line;
} EmbState;

typedef struct
{
    unsigned short level_;
    unsigned long index_;
    char data_[128];
} EmbLog;

/**
 * @brief 函数简要说明-清除运动程序相关信息
 * @param core    核号
 * @param mode    清除模式，mode = MOTION_PROGRAM_CLEAR_ALL_TASK，清除所有的task；mode = MOTION_PROGRAM_CLEAR_TASK：清除指定的task；mode = MOTION_PROGRAM_CLEAR_FILE：清除所有已经下载的文件
 * @param task    清除线程的索引，mode = MOTION_PROGRAM_CLEAR_TASK时该参数生效
 *
 * @return 返回说明
 *     -<em>非0</em> 执行失败
 *     -<em>0</em> 执行成功
 */
GT_API GTN_ClearMp(short core,short mode,short task);

/**
 * @brief 函数简要说明-下载运动程序脚本,最多下载24个运动程序文件
 * @param core    核号
 * @param pFileMpName     运动程序运行文件路径
 *
 * @return 返回说明
 *     -<em>非0</em> 执行失败
 *     -<em>0</em> 执行成功
 */
GT_API GTN_DownLoadFileMP(int16_t core,char *pFileMpName);

/**
 * @brief 函数简要说明-运动程序和task绑定
 * @param core    核号
 * @param task    绑定线程的索引，GVN目前支持8个线程
 * @param pFileMpName     运动程序运行文件路径
 *
 * @return 返回说明
 *     -<em>非0</em> 执行失败
 *     -<em>0</em> 执行成功
 */
GT_API GTN_BindMP(int16_t core,int16_t task,char *pFileMpName);

/**
 * @brief 函数简要说明-启动线程
 * @param core    核号
 * @param task    启动线程的索引，GVN目前支持8个线程
 * @param mode    线程执行模式，mode = MOTION_PROGRAM_MODE_RELEASE：release模式，该模式下只能暂停、继续、停止线程运行;mode = MOTION_PROGRAM_MODE_DEBUG：debug模式，该模式下可以支持暂停、继续、停止、断点、单步进入、单步跳出、单步结束、添加断点、删除断点、删除所有断点、获取所有断点等功能（该模式暂时不支持）
 *
 * @return 返回说明
 *     -<em>非0</em> 执行失败
 *     -<em>0</em> 执行成功
 */
GT_API GTN_RunMP(int16_t core,int16_t task,int16_t mode);

/**
 * @brief 函数简要说明-终止线程
 * @param core    核号
 * @param task    终止线程的索引，GVN目前支持8个线程
 *
 * @return 返回说明
 *     -<em>非0</em> 执行失败
 *     -<em>0</em> 执行成功
 */
GT_API GTN_StopMP(int16_t core,int16_t task);

/**
 * @brief 函数简要说明-设置所有运动程序文件公共变量值,32位整型变量共有10000个，所有的运动程序文件对10000个变量有写权限
 * @param core      核号
 * @param index     设置变量写入的起始索引
 * @param pValue    设置变量的写入值，该变量为数组，大小是count
 * @param count     设置变量的写入个数
 *
 * @return 返回说明
 *     -<em>非0</em> 执行失败
 *     -<em>0</em> 执行成功
 */
GT_API GTN_SetPublicInt32VarValueMP(int16_t core,int32_t index, int* pValue, int32_t count);

/**
 * @brief 函数简要说明-读取所有运动程序文件公共变量值,32位整型变量共有10000个，所有的运动程序文件对10000个变量有读权限
 * @param core      核号
 * @param index     设置读取变量的起始索引
 * @param pValue    读取变量的值，该变量为数组，大小是count
 * @param count     设置变量的读取个数
 *
 * @return 返回说明
 *     -<em>非0</em> 执行失败
 *     -<em>0</em> 执行成功
 */
GT_API GTN_GetPublicInt32VarValueMP(int16_t core,int32_t index, int* pValue, int32_t count);

/**
 * @brief 函数简要说明-设置所有运动程序文件公共变量值,64位浮点型变量共有10000个，所有的运动程序文件对10000个变量有写权限
 * @param core      核号
 * @param index     设置变量写入的起始索引
 * @param pValue    设置变量的写入值，该变量为数组，大小是count
 * @param count     设置变量的写入个数
 *
 * @return 返回说明
 *     -<em>非0</em> 执行失败
 *     -<em>0</em> 执行成功
 */
GT_API GTN_SetPublicFloat64VarValueMP(int16_t core,int32_t index, double* pValue, int32_t count);

/**
 * @brief 函数简要说明-读取所有运动程序文件公共变量值,64位浮点型变量共有10000个，所有的运动程序文件对10000个变量有读权限
 * @param core      核号
 * @param index     设置读取变量的起始索引
 * @param pValue    读取变量的值，该变量为数组，大小是count
 * @param count     设置变量的读取个数
 *
 * @return 返回说明
 *     -<em>非0</em> 执行失败
 *     -<em>0</em> 执行成功
 */
GT_API GTN_GetPublicFloat64VarValueMP(int16_t core,int32_t index, double* pValue, int32_t count);

/**
 * @brief 函数简要说明-读取所有运动程序文件执行状态
 * @param core      核号
 * @param task      task的编号,从这个task开始读取
 * @param pStatus   读取运动程序执行状态，该变量为数组，大小是count
 * @param count     读取变量的大小
 *
 * @return 返回说明
 *     -<em>非0</em> 执行失败
 *     -<em>0</em> 执行成功
 */
GT_API GTN_GetRunStateMP(int16_t core, int16_t task, EmbState* pState, int16_t Count);

/**
 * @brief 函数简要说明-设置任务的打印的日志级别
 * @param core      核号
 * @param task      task的编号
 * @param level     需要打印的日志级别,0: 不打印,1: 打印error日志,2: 打印warning日志及以上,3: 打印所有日志
  *
 * @return 返回说明
 *     -<em>非0</em> 执行失败
 *     -<em>0</em> 执行成功
 */
GT_API GTN_SetLogLevelMP(int16_t core,int16_t task,int16_t level);

/**
 * @brief 函数简要说明-获取任务的打印的日志级别
 * @param core      核号
 * @param task      task的编号
 * @param level     需要打印的日志级别,0: 不打印,1: 打印error日志,2: 打印warning日志及以上,3: 打印所有日志
  *
 * @return 返回说明
 *     -<em>非0</em> 执行失败
 *     -<em>0</em> 执行成功
 */
GT_API GTN_GetLogLevelMP(int16_t core,int16_t task,int16_t *pLevel);

/**
 * @brief 函数简要说明-读取日志信息(读取到数组变量中)
 * @param core         核号
 * @param task         task的编号
 * @param max_count    希望读取到的数量,一般为用户定义数组元素的最大值
 * @param pRead_count  读取到的日志的数量
 * @param pLog         读取到的日志信息数组
 * @param pMore_logs   读取之后，还剩余的日志数量
  *
 * @return 返回说明
 *     -<em>非0</em> 执行失败
 *     -<em>0</em> 执行成功
 */
GT_API GTN_ReadNewLogsMP(int16_t core, int16_t task, int16_t max_count, int16_t *pRead_count, EmbLog *pLog, uint32_t *pMore_logs);

/**
 * @brief 函数简要说明-获取当前是否有日志信息可以获取
 * @param core      核号
 * @param task      task的编号
 * @param pYes      是否有日志信息,0: 没有,1: 有
  *
 * @return 返回说明
 *     -<em>非0</em> 执行失败
 *     -<em>0</em> 执行成功
 */
GT_API GTN_HasNewLogMP(int16_t core, int16_t task, int32_t *pYes);

/**
 * @brief 函数简要说明-获取当前是否有日志信息可以获取
 * @param core           核号
 * @param task           task的编号
 * @param pLogFileName   需要存储日志信息的文件名称(含路径)
 * @param pLogs_saved    文件中存储的日志信息的数量
  *
 * @return 返回说明
 *     -<em>非0</em> 执行失败
 *     -<em>0</em> 执行成功
 */
GT_API GTN_SaveLogToFileMP(int16_t core, int16_t task, char *pLogFileName, uint32_t *pLogs_saved);

/**
 * @brief 函数简要说明-下载轴相关参数
 * @param core      核号
 * @param axis      轴号
 * @param index     设置变量的起始索引,具体定义参照枚举
 * @param axis_ref  设置的变量值，该变量为数组，大小是count
 * @param count     一次性设置的变量的数量
 *
 * @return 返回说明
 *     -<em>非0</em> 执行失败
 *     -<em>0</em> 执行成功
 */
GT_API GTN_DownloadAxisRefMP(int16_t core, int16_t axis, int16_t index, double* axis_ref, int16_t count);

/**
 * @brief 函数简要说明-获取轴相关参数
 * @param core      核号
 * @param axis      轴号
 * @param index     读取变量的起始索引,具体定义参照枚举
 * @param axis_ref  读取的变量值，该变量为数组，大小是count
 * @param count     一次性读取的变量的数量
 *
 * @return 返回说明
 *     -<em>非0</em> 执行失败
 *     -<em>0</em> 执行成功
 */
GT_API GTN_UploadAxisRefMP(int16_t core, int16_t axis, int16_t index, double* axis_ref, int16_t count);


typedef struct
{
   short enable;                    // 软捕获功能使能标志
   short reserve1[6];               // 保留值,必须为0

   short loop;                      // 捕获次数
   TWatchCondition condition;       // 捕获条件信息

   short reserve2[3];               // 保留值,必须为0
   short latchVarCount;             // 捕获触发后要锁存的变量个数
   TWatchVar latchVar[8];           // 捕获触发后要锁存的变量信息

   double reserve3[8];              // 保留值,必须为0
}TSoftTriggerPrm;

typedef struct
{
   unsigned long clock;             // 锁存时间
   short reserve[2];                // 保留值，返回为0
   double latchValue[8];            // 锁存变量的值
}TSoftTriggerLatchValue;

typedef struct
{
   short enable;                    // 软捕获功能使能标志
   short triggerFlag;               // 捕获触发标志
   short reminderCount;             // 缓冲区中剩余未读取的捕获数量
   short reserve1[7];               // 保留，返回值为0.
   long totalCount;                 // 总的触发次数
   double reserve2[8];              // 保留，返回值为0.
}TSoftTriggerInfo;

/**
 * @brief 设置软件捕获参数
 * @param core 核号
 * @param triggerIndex 软件捕获索引
 * @param pPrm 软件捕获参数
 * @param triggerCount 同时设置软件捕获的数量,
 * @param pListInfo 指令流信息，保留为NULL
 * @return 错误码
 17001 参数pListInfo错误，目前只能为NULL。
 17051 参数triggerCount错误，范围为[1,4],同时triggerIndex+triggerCount必须在范围[1,4]内。
 17053 参数triggerIndex错误，范围为[1,4]。
 17054 参数pPrm->enable错误，只能为0或者1。
 17100 参数pPrm中的保留参数错误，必须为0。
 17501 参数pPrm中的loop参数错误，范围为[0,128]
 17502 参数pPrm中的latchVarCount参数错误，范围为[1,8]
 17505 参数pPrm中的condition.condition参数错误。
 11503 解析参数pPrm中的condition.var失败，检测condition.var参数是否正确
 11504 获取参数pPrm中的condition.var的初值错误，检测condition.var参数是否正确
 11505 解析参数pPrm中的latchVar失败，检测latchVar参数是否正确
 11506 内部错误，配置存储锁存信息的缓冲区出错
 17745 参数pPrm为空指针
 */
GT_API GTN_SetSoftTriggerPrm(short core,short triggerIndex,TSoftTriggerPrm *pPrm,short triggerCount,TListInfo *pListInfo=NULL);

/**
 * @brief 清除软件捕获状态和数据
 * @param core 核号
 * @param triggerIndex 软件捕获索引
 * @param triggerCount 同时清除软件捕获的数量,
 * @return 错误码
 17051 参数triggerCount错误，范围为[1,4],同时triggerIndex+triggerCount必须在范围[1,4]内。
 17053 参数triggerIndex错误，范围为[1,4]。
*/
GT_API GTN_ClearSoftTrigger(short core,short triggerIndex,short triggerCount);

/**
 * @brief 读取软件捕获信息
 * @param core 核号
 * @param triggerIndex 软件捕获索引
 * @param pInfo 软件捕获信息
 * @param triggerCount 同时读取软件捕获信息的数量,
 * @return
 17051 参数triggerCount错误，范围为[1,4],同时triggerIndex+triggerCount必须在范围[1,4]内。
 17053 参数triggerIndex错误，范围为[1,4]。
 17745 参数pInfo为空指针
*/
GT_API GTN_GetSoftTriggerInfo(short core,short triggerIndex,TSoftTriggerInfo *pInfo,short triggerCount);

/**
 * @brief 读取软件捕获锁存值
 * @param core 核号
 * @param triggerIndex 软件捕获索引
 * @param start 读取捕获锁存信息的起始索引
 * @param readCount 读取捕获锁存信息的数量
 * @param pLatchValue 软件捕获锁存信息
 * @param pRealCount 实际读取的捕获次数数量,
 * @return 错误码
 11501 内部错误，从存储捕获值的缓冲区中拿数错误。
 11502 内部错误，从存储捕获值的缓冲区中拿到的数全0。
 17053 参数triggerIndex错误，范围为[1,4]。
 17503 start参数错误，必须大于1，小于GTN_SetSoftTriggerPrm设置的触发次数
 17504 readCount参数错误，范围为[1,12]
*/
GT_API GTN_GetSoftTriggerLatchValue(short core,short triggerIndex,short start,short readCount,TSoftTriggerLatchValue *pLatchValue,short *pRealCount);

GT_API GTN_SetCompensate2DTableOffset(short core,short tableIndex,double xOffset,double yOffset);
GT_API GTN_GetCompensate2DTableOffset(short core,short tableIndex,double *pXOffset,double *pYOffset);
GT_API GTN_BufSetCompensate2DTableOffsetEx(short core,short crd,short tableIndex,double xOffset,double yOffset,short fifo);

/**
 * @brief 设置高速采集PSO触发计数初始值。
 * @param cardIndex 卡号。
 * @param psoIndex 第几路PSO，根据实际资源确认，目前最多4路。
 * @param psoCnt 设置的采集PSO触发计数初值。
 * @return 0表示成功，非0表示失败
*/
GT_API GTN_RN_SamplingSetPsoCnt(short cardIndex, short stationPhyId,short psoIndex,unsigned long psoCnt);

/**
 * @brief mailBox读写寄存器，根据cmd不同可选择不同寻址方式。
 * @param cardIndex 卡号。
 * @param stationPhyId 物理站号。
 * @param mode 报文模式。0：PDU报文，1：MailBox报文。
 * @param cmd 地址寻址方式，0：逻辑寻址写。1：绝对寻址写。2：逻辑地址映射寻址写。3：报文发送配置区寻址写。4：逻辑寻址读。5：绝对寻址读。6：逻辑地址营社区寻址读。7：报文发送配置区寻址读。
 * @param byteAddr 寄存器地址，byte地址。
 * @param pData 要写入/读取的数据。
 * @param wordNum 要写入/读取的数据个数，取值范围[0,240]，单位word。
 * @param desCh 访问的目的通道。0：FPGA通道。1：PCI通道。2：DSP通道。3：DSP通道。
 * @param needReq 是否需要应答。0：不需要应答。1:需要应答。
 * @param addrMod 地址访问模式。0：如果有多个word数据写入，每次访问同一个地址。1：如果有多个word数据写入，每次访问地址+1。
 * @param waitLevel 默认写0。
 * @param distance 默认写0xF0。
 * @return 0表示成功，非0表示失败
*/
GT_API GTN_RN_ComHandler(short cardIndex, short stationPhyId,unsigned char mode, unsigned char cmd, unsigned short byteAddr, short* pData, unsigned short wordNum,
    unsigned char desCh, unsigned char needReq, unsigned char addrMod, unsigned char waitLevel, unsigned char distance);

/**
 * @author
 * @brief 扩展模块通用导出指令
 * @param cardIndex 卡号。
 * @param stationPhyId 物理站号。
 * @param moduleId 扩展模块ID，取值范围：[0..63]
 * @param mode 模式
 * @param pParam 保留
 * @return 0表示成功，非0表示失败
*/
GT_API GTN_RN_IlinkGeneralCmd(short cardIndex, short stationPhyId, short moduleId, short mode, void* pParam);

/**
 * @author
 * @brief 扩展模块通用导出指令
 * @param cardIndex 卡号。
 * @param stationPhyId 物理站号。
 * @param pResCount 实际获取到的网络设备个数
 * @param pInfo 获取到的网络设备信息。高16位：设备ID；低16位：设备类型
 * @param count 需要获取的网络设备个数
 * @return 0表示成功，非0表示失败
*/
GT_API GTN_RN_GetGroupInfo(short cardIndex, short stationPhyId, unsigned short* pResCount, uint32_t* pInfo, unsigned short count);

/**
 * @brief 读取等环网是否下移到DSP
 * @param core 核号
 * @param pFlag 等环网下移标志位，0：未下移。1：下移。
 * @return 0表示成功，非0表示失败
*/
GT_API GTN_GetDspRingNetFlag(short core,short *pFlag);

GT_API GTN_SetEcatSlavePdo(short core, unsigned short station, unsigned short index, unsigned char subIndex, unsigned char* pData, unsigned int data_size);
GT_API GTN_GetEcatSlavePdo(short core, unsigned short station, unsigned short index, unsigned char subIndex, unsigned char* pData, unsigned int data_size);




