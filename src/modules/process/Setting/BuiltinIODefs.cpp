#include "modules/process/Setting/BuiltinIODefs.h"

#include <cstddef>

namespace lcnc::process {

namespace {

// 默认值取自旧版 Setting_IOIndex::InitSetting；showInMain 仅对常用切换项打开
// （激光、吹气、夹头、出/回水泵），其余预设默认关闭。

constexpr BuiltinIODef kDigitalOUT[] = {
    // sectionKey,    tomlKey,        nameZh,    defaultIndex, active, enabled, showInMain
    { "DigitalOUT",   "aLaser",       "激光",     "0.4",        true,   true,    true  },
    { "DigitalOUT",   "aBlow",        "吹气",     "0.2",        true,   true,    true  },
    { "DigitalOUT",   "aChuck",       "夹头",     "0.0",        true,   true,    true  },
    { "DigitalOUT",   "aPliers",      "夹爪",     "0.1",        true,   true,    false },
    { "DigitalOUT",   "aWater",       "出水泵",   "0.10",       true,   true,    true  },
    { "DigitalOUT",   "aPump",        "回水泵",   "0.11",       true,   true,    true  },
    { "DigitalOUT",   "aRedLight",    "红灯",     "0.7",        true,   true,    false },
    { "DigitalOUT",   "aYellowLight", "黄灯",     "0.6",        true,   true,    false },
    { "DigitalOUT",   "aGreenLight",  "绿灯",     "0.5",        true,   true,    false },
    { "DigitalOUT",   "aBuzzer",      "蜂鸣器",   "1.0",        true,   true,    false },
};

constexpr BuiltinIODef kDigitalIN[] = {
    { "DigitalIN",    "aStart",                "开始",      "0.0", true, true, false },
    { "DigitalIN",    "aStop",                 "停止",      "0.1", true, true, false },
    { "DigitalIN",    "aInterLock",            "互锁",      "0.3", true, true, false },
    { "DigitalIN",    "aSafetyLightCurtain",   "安全光幕",  "0.7", true, true, false },
    { "DigitalIN",    "aPressureMonitor",      "气压监控",  "0.2", true, true, false },
    { "DigitalIN",    "aRemnantsMonitor",      "余料监控",  "0.5", true, true, false },
    { "DigitalIN",    "aWaterLeakageMonitor",  "液位监控",  "0.4", true, true, false },
    { "DigitalIN",    "aWaterTankMonitor",     "水箱监控",  "0.6", true, true, false },
};

constexpr BuiltinIODef kAnalogOUT[] = {
    { "AnalogOUT",    "aLaser",      "激光功率",  "1", true, true, false },
    { "AnalogOUT",    "aPressure",   "气压设定",  "0", true, true, false },
};

constexpr BuiltinIODef kAnalogIN[] = {
    { "AnalogIN",     "aWaterLevel",     "液位",  "0", true, true, false },
    { "AnalogIN",     "aWaterPressure",  "水压",  "1", true, true, false },
    { "AnalogIN",     "aPressure",       "气压",  "2", true, true, false },
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
