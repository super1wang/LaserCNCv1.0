#pragma once

class CommandContainer;
class SARibbonCategory;
class QObject;

namespace lcnc::cam {

/**
 * @brief 注册 CAM 模块所有命令到 CommandContainer。
 *
 * 包括"机台控制"（载入/压缩/挂工件/卸载/导出）与"刀路"（生成/导引/预览/重算）
 * 两组命令。命令实际执行时通过 Kernel::current().service<CamModule>() 取得 CamModule。
 *
 * @param container 主窗口创建的命令容器
 */
void registerCommands(CommandContainer* container);

/**
 * @brief 把 CAM Ribbon 选项卡填充到 @p category（"CAM" 标签页）。
 *
 * 包括"机台 / 刀路 / G代码（占位）"三个面板。
 *
 * @param category SARibbon 创建的空白选项卡
 * @param container 命令容器
 * @param parent 占位 QAction 的父对象
 */
void buildRibbonTab(SARibbonCategory* category,
                    CommandContainer* container,
                    QObject* parent);

} // namespace lcnc::cam
