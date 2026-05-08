#pragma once

#include "core/kernel/i_service.h"

#include <QString>

class QObject;
class CamConfig;

namespace lcnc {

/**
 * @brief CAM 模块对外门面接口（Phase 7）。
 *
 * 仅暴露 UI/命令最常用的动作：切换至机台视图、读取/修改 CAM 配置。
 * 本接口继承 @ref IService；CamModule 不再直接继承 IService 以避免多重继承。
 */
class ICamFacade : public IService
{
public:
    ~ICamFacade() override = default;

    /// 用于让调用方挂接 CamModule 的 Qt 信号。
    virtual QObject* asQObject() = 0;

    /// 切换 3D 视图至机台工作区（载入并定位）。
    virtual void requestMachineView() = 0;

    /// 模块持有的持久化配置（cam.toml）。
    virtual CamConfig& config() = 0;
    virtual const CamConfig& config() const = 0;
};

} // namespace lcnc
