#pragma once

class CommandContainer;
class SARibbonCategory;
class QObject;
class QWidget;

namespace lcnc::process {

/**
 * @brief 注册 Process 模块（激光加工执行）的命令到 CommandContainer。
 *
 * 当前 Process 模块通过独立命令对象转发到 IProcessFacade，Ribbon 只负责挂载动作。
 *
 * @param container 主窗口创建的命令容器
 */
void registerCommands(CommandContainer* container);

/**
 // 中文翻译：激光加工
 * @brief 把 激光加工 Ribbon 选项卡填充到 @p category（"Laser processing" 标签页）。
 *
 * 包括"连接 / 流程（占位）/ 运行 / 参数（占位）"四个面板。
 *
 * @param category SARibbon 创建的空白选项卡
 * @param container 命令容器（保留参数以与 cad/cam 一致）
 * @param parent 占位 QAction 的父对象（一般为 MainWindow）
 */
void buildRibbonTab(SARibbonCategory* category,
                    CommandContainer* container,
                    QObject* parent);

} // namespace lcnc::process
