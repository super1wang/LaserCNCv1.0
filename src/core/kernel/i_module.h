#pragma once

#include <QString>
#include <QStringList>

namespace lcnc {

class IKernel;

/**
 * @brief 模块元信息（注册时声明）。
 *
 * 字段语义：
 *   - @c id              进程内唯一短标识（如 "cad" / "cam" / "process"）；
 *   - @c displayName     UI 中可展示的中文名；
 *   - @c version         模块语义化版本号（"1.0.0"）；
 *   - @c dependencies    依赖的其他模块 id 列表，由 @ref ModuleRegistry
 *                         在启动时做拓扑排序，保证依赖先于被依赖者初始化。
 */
struct ModuleInfo
{
    QString     id;
    QString     displayName;
    QString     version;
    QStringList dependencies;
};

/**
 * @brief 微内核中"插件模块"的契约。
 *
 * 生命周期顺序由 @ref ModuleRegistry 管理：
 *   1. 构造（main 中 @c addModule 时）；
 *   2. @ref init(kernel) — 拿到内核引用，向 @c ServiceRegistry 注册自身
 *      服务、向 @c ICommandBus 注册命令、向 @c EventBus 订阅感兴趣事件。
 *      此阶段**不应主动操作其他模块的服务**（依赖虽然 init 已完成，
 *      但 start 才算业务可用）；
 *   3. @ref start() — 全部依赖已完成 init 之后调用；可在此发布
 *      "ModuleReady" 事件、加载默认数据等；
 *   4. @ref stop() — Kernel shutdown 反序调用；用于注销订阅、释放资源。
 *
 * 任一阶段返回 false / 抛异常都会被 Kernel 视为模块启动失败，整体启动
 * 流程会回滚（停掉已 start 的模块，不再 start 后续模块）。
 *
 * 实现要点：
 *   - 模块 **不** 持有 @ref Kernel 指针；只在 @c init 调用期间使用引用；
 *     如果 start/stop 阶段也需访问内核能力，应在 init 中保存到自有成员
 *     （建议保存 weak_ptr 或具体服务接口而非 IKernel*）。
 */
class IModule
{
public:
    virtual ~IModule() = default;

    /// 返回元信息；调用应是廉价 const 方法（仅返回成员或字面量）。
    virtual ModuleInfo info() const = 0;

    /**
     * @brief 注册阶段：把自己提供的服务/命令注册到内核。
     * @return false 表示初始化失败，将停止整体启动。
     */
    virtual bool init(IKernel& kernel) = 0;

    /**
     * @brief 激活阶段：所有模块（含依赖）均已 init。
     * @return false 表示激活失败。
     */
    virtual bool start() = 0;

    /**
     * @brief 反向释放阶段：取消订阅、注销服务、释放资源。
     */
    virtual void stop() = 0;
};

} // namespace lcnc
