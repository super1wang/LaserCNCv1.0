#pragma once

#include "core/kernel/i_service.h"
#include "core/settings/app_settings.h"

namespace lcnc {

/**
 * @brief 应用配置服务接口。
 *
 * 当前阶段只暴露 @ref AppSettings 引用；后续 Phase 3/4 接入 CamSettings /
 * ProcessSettings 时将扩展为：
 * @code
 *   template<class T> T& get();   // 按类型取配置实例
 * @endcode
 *
 * 这里先用具体引用，避免一次到位的接口膨胀；CAM/Process 模块迁移时
 * 可在本接口上增加专用方法或泛型版本。
 */
class ISettingsService : public IService
{
public:
    /// 取应用主配置（mainwindow.toml）。
    virtual AppSettings& app() = 0;

    /// 触发一次保存（写当前内存值到磁盘）。
    virtual bool save() = 0;
};

} // namespace lcnc
