#pragma once

/**
 * @file i_service.h
 * @brief 微内核共享服务的标记基类。
 *
 * 在 LaserCNC 微内核架构中，"服务"指**跨模块共享的能力对象**（日志、
 * 配置、任务调度、文档注册表等）。所有具体服务接口都需公开继承本类，
 * 这样 @ref ServiceRegistry 才能用统一的 @c std::shared_ptr<IService>
 * 进行类型擦除存储与按类型查找。
 *
 * 服务接口本身只暴露**纯虚方法 + 信号声明**（如有），不应包含任何
 * 实现细节，以便：
 *   - 模块只 include 接口头，避免对实现的硬依赖；
 *   - 测试可注入 mock 实现。
 */
namespace lcnc {

class IService
{
public:
    virtual ~IService() = default;
};

} // namespace lcnc
