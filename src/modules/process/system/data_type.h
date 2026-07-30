#pragma once

#include <map>
#include <string>
#include <vector>

#include <QString>

#include "message_code.h"
#include "magic_enum.hpp"

#define MaxNestingNumber 1000
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
#define MAXIMUM 1E32

using std::map;
using std::string;
using std::vector;
using magic_enum::enum_cast;
using magic_enum::enum_name;
using magic_enum::enum_names;

enum class PermissionLevel
{
    Developers = 9,
    Factory = 7,
    Simulate = 6,
    Administrator = 4,
    Technician = 2,
    Operator = 1,
    None = 0
};

enum class Axis
{
    X = 0, Y = 1, Z = 2, A = 3, B = 4, C = 5
};

enum class SystemStatus
{
    UnInit, Initializing,
    Idle,
    Paused, Pausing,
    Processing, LaserProcessing,
    Error
};

enum class RunMode
{
    SignMode = 0,
    CuttingTest = 1,
    ProcessTest = 2
};

enum class ItemType
{
    Start, Stop,
    If, Loop, Group, RunGroup,
    IO, Camera, Monitor,
    Calculation, Compare,
    Base,
    Wait, Commands, Feeding, RunGroupCheck,
    Axis, AxesMove, Measurement, MarkAcquire, Alignment,
    AutoFocus, EnergySwitch, Cutting, OverCutting
};

enum class ItemState
{
    StateSave,
    Disable, Enable, Editing, Unavailable,
    Run, Stop, Pause, Unuse, Unrun
};

struct Item
{
    int iParentIndex;
    int iChildrenIndex;
    ItemType Type;
    map<QString, QString> maps;
};

enum class DigitalIN
{
    Start, Stop, InterLock, RemnantsMonitor, SafetyLightCurtain,
    PressureMonitor, WaterLeakageMonitor, WaterTankMonitor,
    IN1, IN2, IN3, IN4, IN5, IN6, IN7, IN8,
    IN9, IN10, IN11, IN12, IN13, IN14, IN15, IN16,
    IN17, IN18, IN19, IN20, IN21, IN22, IN23, IN24,
    IN25, IN26, IN27, IN28, IN29, IN30, IN31, IN32,
    IN33, IN34, IN35, IN36, IN37, IN38, IN39, IN40,
    IN41, IN42, IN43, IN44, IN45, IN46, IN47, IN48,
    IN49, IN50, IN51, IN52, IN53, IN54, IN55, IN56,
    IN57, IN58, IN59, IN60, IN61, IN62, IN63, IN64
};

enum class DigitalOUT
{
    Laser, Blow, Chuck, Pliers, Water, Pump,
    RedLight, YellowLight, GreenLight, Buzzer,
    Blow2,
    OUT1, OUT2, OUT3, OUT4, OUT5, OUT6, OUT7, OUT8,
    OUT9, OUT10, OUT11, OUT12, OUT13, OUT14, OUT15, OUT16,
    OUT17, OUT18, OUT19, OUT20, OUT21, OUT22, OUT23, OUT24,
    OUT25, OUT26, OUT27, OUT28, OUT29, OUT30, OUT31, OUT32,
    OUT33, OUT34, OUT35, OUT36, OUT37, OUT38, OUT39, OUT40,
    OUT41, OUT42, OUT43, OUT44, OUT45, OUT46, OUT47, OUT48,
    OUT49, OUT50, OUT51, OUT52, OUT53, OUT54, OUT55, OUT56,
    OUT57, OUT58, OUT59, OUT60, OUT61, OUT62, OUT63, OUT64
};

enum class AnalogIN
{
    WaterLevel, WaterPressure, Pressure,
    IN1, IN2, IN3, IN4, IN5, IN6, IN7, IN8,
    IN9, IN10, IN11, IN12, IN13, IN14, IN15, IN16,
    IN17, IN18, IN19, IN20, IN21, IN22, IN23, IN24,
    IN25, IN26, IN27, IN28, IN29, IN30, IN31, IN32,
    IN33, IN34, IN35, IN36, IN37, IN38, IN39, IN40,
    IN41, IN42, IN43, IN44, IN45, IN46, IN47, IN48,
    IN49, IN50, IN51, IN52, IN53, IN54, IN55, IN56,
    IN57, IN58, IN59, IN60, IN61, IN62, IN63, IN64
};

enum class AnalogOUT
{
    Laser, Pressure,
    OUT1, OUT2, OUT3, OUT4, OUT5, OUT6, OUT7, OUT8,
    OUT9, OUT10, OUT11, OUT12, OUT13, OUT14, OUT15, OUT16,
    OUT17, OUT18, OUT19, OUT20, OUT21, OUT22, OUT23, OUT24,
    OUT25, OUT26, OUT27, OUT28, OUT29, OUT30, OUT31, OUT32,
    OUT33, OUT34, OUT35, OUT36, OUT37, OUT38, OUT39, OUT40,
    OUT41, OUT42, OUT43, OUT44, OUT45, OUT46, OUT47, OUT48,
    OUT49, OUT50, OUT51, OUT52, OUT53, OUT54, OUT55, OUT56,
    OUT57, OUT58, OUT59, OUT60, OUT61, OUT62, OUT63, OUT64
};
