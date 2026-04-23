#pragma once

class CommandContainer;
class SARibbonCategory;
class QObject;

namespace lcnc::cad {

/**
 * @brief 注册 CAD 模块所有命令到 CommandContainer。
 *
 * 调用方负责保证 @p container 已经构造、@p kernel 已 bootstrap（CAD 命令
 * 执行时会通过 Kernel::current() 取得 CadModule / LcncApplication 等）。
 *
 * 命令按面板组织：File（新建/打开/保存/导入/导出/关闭）、Edit（撤销/重做）、
 * Primitives、Transforms、Boolean、Measurement、Delete。
 *
 * @param container 主窗口创建的命令容器
 */
void registerCommands(CommandContainer* container);

/**
 * @brief 把 CAD Ribbon 选项卡填充到 @p category（"CAD" 标签页）。
 *
 * 包括"历史 / 基本体 / 操作 / 测量 / 草图（占位）"五个面板。命令必须先经
 * @ref registerCommands 注册。
 *
 * @param category SARibbon 创建的空白选项卡
 * @param container 命令容器（由其 findAction 取按钮关联）
 * @param parent 占位 QAction 的父对象（一般传 MainWindow）
 */
void buildRibbonTab(SARibbonCategory* category,
                    CommandContainer* container,
                    QObject* parent);

} // namespace lcnc::cad
