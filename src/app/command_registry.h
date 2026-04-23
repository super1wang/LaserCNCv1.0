#pragma once

class AppContext;
class CommandContainer;
class WidgetOccView;
class QObject;

namespace lcnc::app {

/**
 * @brief 集中向 @p container 注册全部命令并完成 Ribbon 视图相关的连线。
 *
 * 调用顺序：
 *   1. 创建/赋值 AppContext 与 CommandContainer 后调用本函数；
 *   2. 函数内部依次调用 cad/cam/process 模块的 registerCommands；
 *   3. 注册"显示"模块自有命令（FitAll、Wireframe、Shaded 等）；
 *   4. 把这些显示命令的 QAction triggered 信号连接到 @p occView。
 *
 * 不持有任何对象指针；调用方负责对象生命周期。
 *
 * @param container 已 new 完成的命令容器
 * @param context 应用上下文（命令构造需要）
 * @param occView OCC 3D 视图，用于显示模式切换
 * @param parent 占位 QAction 的父对象（一般为 MainWindow）
 */
void registerAllCommands(CommandContainer* container,
                         AppContext* context,
                         WidgetOccView* occView,
                         QObject* parent);

} // namespace lcnc::app
