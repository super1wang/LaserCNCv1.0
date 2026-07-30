#include "modules/process/setting/builtin_io_defs.h"

#include <cstddef>

namespace lcnc::process {

namespace {

// 默认值取自旧版 Setting_IOIndex::InitSetting；showInMain 仅对常用切换项打开
// （激光、吹气、夹头、出/回水泵），其余预设默认关闭。

constexpr BuiltinIODef kDigitalOUT[] = {
    // sectionKey,    tomlKey,        nameZh,    defaultIndex, active, enabled, showInMain
    // 中文翻译：激光
    { "DigitalOUT",   "aLaser",       "laser",     "0.4",        true,   true,    true  },
    // 中文翻译：吹气
    { "DigitalOUT",   "aBlow",        "blow air",     "0.2",        true,   true,    true  },
    // 中文翻译：夹头
    { "DigitalOUT",   "aChuck",       "chuck",     "0.0",        true,   true,    true  },
    // 中文翻译：夹爪
    { "DigitalOUT",   "aPliers",      "Gripper",     "0.1",        true,   true,    false },
    // 中文翻译：出水泵
    { "DigitalOUT",   "aWater",       "Outlet pump",   "0.10",       true,   true,    true  },
    // 中文翻译：回水泵
    { "DigitalOUT",   "aPump",        "Return water pump",   "0.11",       true,   true,    true  },
    // 中文翻译：红灯
    { "DigitalOUT",   "aRedLight",    "red light",     "0.7",        true,   true,    false },
    // 中文翻译：黄灯
    { "DigitalOUT",   "aYellowLight", "yellow light",     "0.6",        true,   true,    false },
    // 中文翻译：绿灯
    { "DigitalOUT",   "aGreenLight",  "green light",     "0.5",        true,   true,    false },
    // 中文翻译：蜂鸣器
    { "DigitalOUT",   "aBuzzer",      "buzzer",   "1.0",        true,   true,    false },
};

constexpr BuiltinIODef kDigitalIN[] = {
    // 中文翻译：开始
    { "DigitalIN",    "aStart",                "start",      "0.0", true, true, false },
    // 中文翻译：停止
    { "DigitalIN",    "aStop",                 "stop",      "0.1", true, true, false },
    // 中文翻译：互锁
    { "DigitalIN",    "aInterLock",            "interlock",      "0.3", true, true, false },
    // 中文翻译：安全光幕
    { "DigitalIN",    "aSafetyLightCurtain",   "safety light curtain",  "0.7", true, true, false },
    // 中文翻译：气压监控
    { "DigitalIN",    "aPressureMonitor",      "Air pressure monitoring",  "0.2", true, true, false },
    // 中文翻译：余料监控
    { "DigitalIN",    "aRemnantsMonitor",      "Remaining material monitoring",  "0.5", true, true, false },
    // 中文翻译：液位监控
    { "DigitalIN",    "aWaterLeakageMonitor",  "Liquid level monitoring",  "0.4", true, true, false },
    // 中文翻译：水箱监控
    { "DigitalIN",    "aWaterTankMonitor",     "Water tank monitoring",  "0.6", true, true, false },
};

constexpr BuiltinIODef kAnalogOUT[] = {
    // 中文翻译：激光功率
    { "AnalogOUT",    "aLaser",      "Laser power",  "1", true, true, false },
    // 中文翻译：气压设定
    { "AnalogOUT",    "aPressure",   "Air pressure setting",  "0", true, true, false },
};

constexpr BuiltinIODef kAnalogIN[] = {
    // 中文翻译：液位
    { "AnalogIN",     "aWaterLevel",     "liquid level",  "0", true, true, false },
    // 中文翻译：水压
    { "AnalogIN",     "aWaterPressure",  "water pressure",  "1", true, true, false },
    // 中文翻译：气压
    { "AnalogIN",     "aPressure",       "air pressure",  "2", true, true, false },
};

template <std::size_t N>
constexpr BuiltinIODefList makeList(const BuiltinIODef (&arr)[N])
{
    return { arr, static_cast<int>(N) };
}

} // namespace

BuiltinIODefList builtinDigitalOUT() { return makeList(kDigitalOUT); }
BuiltinIODefList builtinDigitalIN()  { return makeList(kDigitalIN);  }
BuiltinIODefList builtinAnalogOUT()  { return makeList(kAnalogOUT);  }
BuiltinIODefList builtinAnalogIN()   { return makeList(kAnalogIN);   }

} // namespace lcnc::process
