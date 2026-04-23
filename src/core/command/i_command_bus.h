#pragma once

#include <QString>

#include "core/kernel/i_service.h"

class CommandBase;
class CommandContainer;
class QAction;

namespace lcnc {

/**
 * @brief 命令总线服务接口。
 *
 * 包装现有 @ref CommandContainer。命令注册仍由各模块的 init 阶段完成
 * （`bus.underlying()->addCommand<CmdXxx>("Name")`）；查找/触发统一通
 * 过本接口。
 *
 * Phase 2/3/4 中将扩充 @ref invoke 形式以支持参数化（QVariantMap），
 * 当前阶段只暴露按名查找。
 */
class ICommandBus : public IService
{
public:
    /**
     * @brief 按名查找命令。返回 nullptr 表示未注册。
     */
    virtual CommandBase* findCommand(const QString& name) const = 0;

    /**
     * @brief 按名查找命令对应的 QAction（便于 ribbon/菜单挂载）。
     */
    virtual QAction* findAction(const QString& name) const = 0;

    /**
     * @brief 同步触发命令（等价于点击其 QAction）。
     * @return false 表示未找到或命令当前不可用。
     */
    virtual bool invoke(const QString& name) = 0;
};

} // namespace lcnc
