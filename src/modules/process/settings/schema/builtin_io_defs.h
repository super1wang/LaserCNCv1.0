#pragma once

namespace lcnc::process {

/**
 * @brief 预设 IO 描述。
 *
 * 用作 settings 的种子值：启动时若 toml 中对应 sectionKey/tomlKey 还没有
 * 子表，就按这里的默认写入；已有的不动。预设 IO 在 UI 上不可改名/改类型/
 * 删除，但可关启用。
 */
struct BuiltinIODef
{
    const char* sectionKey;       ///< "DigitalIN" / "DigitalOUT" / "AnalogIN" / "AnalogOUT"
    const char* tomlKey;          ///< 如 "aLaser"
    const char* nameZh;           ///< 默认中文名
    const char* defaultIndex;     ///< 默认索引（数字量如 "0.4"，模拟量如 "1"）
    bool        defaultActive;    ///< true=高电平有效（仅 Digital 有意义；Analog 写入 toml 但解析端忽略）
    bool        defaultEnabled;   ///< 默认是否启用
    bool        defaultShowInMain;///< 仅 DigitalOUT 用：是否在主界面 IO 栏自动生成按钮
};

struct BuiltinIODefList
{
    const BuiltinIODef* items;
    int                 count;
};

/// 预设数字量输出（激光、吹气、夹头等）。
BuiltinIODefList builtinDigitalOUT();
/// 预设数字量输入（开始、停止、互锁等）。
BuiltinIODefList builtinDigitalIN();
/// 预设模拟量输出（激光功率、气压设定）。
BuiltinIODefList builtinAnalogOUT();
/// 预设模拟量输入（液位、水压、气压采样）。
BuiltinIODefList builtinAnalogIN();

} // namespace lcnc::process
