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
 // 中文翻译：显示
 *   3. 注册"show"模块自有命令（FitAll、Wireframe、Shaded 等）；
 *   4. 显示命令执行时通过 AppContext 动态取得当前 active OccView。
 *
 * 不持有任何对象指针；调用方负责对象生命周期。
 *
 * @param container 已 new 完成的命令容器
 * @param context 应用上下文（命令构造需要）
 * @param occView 启动期默认 OCC 视图，仅用于兼容旧调用和 QActionGroup parent 兜底
 * @param parent 占位 QAction 的父对象（一般为 MainWindow）
 */
void registerAllCommands(CommandContainer* container,
                         AppContext* context,
                         WidgetOccView* occView,
                         QObject* parent);

} // namespace lcnc::app
