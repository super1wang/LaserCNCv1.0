#pragma once

#include "modules/process/cutting/process_cutting_plan_service.h"

#include <QPointer>
#include <QWidget>

class QPushButton;
class QTableWidget;
class QLabel;

namespace lcnc::process {

/**
 * @brief Process 模块的"加工链表"配置面板（v2：左右两栏）。
 *
 * 左：图层 → 工具 配置 (QTableWidget)
 *     col0 = 图层名（只读）
 *     col1 = 工具选择 (QComboBox，候选来自 service->availableToolNames())
 *
 * 右：当前切割链表预览 (QTableWidget)
 *     col0 = 序号（从 1 开始）
 *     col1 = 轮廓名
 *
 * 顶部按钮：「同步 CAM」「应用」「重置」。
 * 排序策略 / Manual 顺序的编辑入口移到 Ribbon「加工顺序」面板，本面板只读消费 service。
 */
class WidgetCuttingPlanPanel : public QWidget
{
    Q_OBJECT
public:
    explicit WidgetCuttingPlanPanel(ProcessCuttingPlanService* service,
                                    QWidget* parent = nullptr);
    ~WidgetCuttingPlanPanel() override;

public slots:
    /// 把 service 当前状态同步到 UI（service 变更 / 同步 CAM 后调用）。
    void refreshFromService();

private slots:
    void onSyncFromCam();
    void onApply();
    void onResetFromService();

private:
    void rebuildLayerTable();
    void rebuildPreview();

    QPointer<ProcessCuttingPlanService> m_service;

    QTableWidget* m_layerTable{nullptr};   ///< 左栏
    QTableWidget* m_preview{nullptr};      ///< 右栏
    QPushButton*  m_btnSyncCam{nullptr};
    QPushButton*  m_btnApply{nullptr};
    QPushButton*  m_btnReset{nullptr};
    QLabel*       m_statusLabel{nullptr};

    bool m_suppressTableSignals{false};
};

} // namespace lcnc::process
