#pragma once

#include "core/settings/app_settings.h"

#include <QObject>
#include <QFlags>
#include <QMap>
#include <QString>
#include <AIS_Shape.hxx>
#include <Standard_Handle.hxx>

class GuiDocument;
class QTimer;

namespace lcnc::view {

/**
 * @brief 渲染参数变更类型，用于 Apply 时做最小刷新。
 */
enum class RenderDirtyFlag {
    None       = 0x00,
    Profile    = 0x01, ///< 渲染方法、AA、阴影、LOD、材质等 profile 参数
    Colors     = 0x02, ///< 工件色、机台分轴色
    Background = 0x04, ///< 当前 V3d_View 背景色
    Highlight  = 0x08, ///< 选中/Hover 高亮样式
    DefaultDisplay = 0x10, ///< 启动/新 view 默认显示模式（不修改当前 runtime display）
    All        = 0x1F
};
Q_DECLARE_FLAGS(RenderDirtyFlags, RenderDirtyFlag)

/**
 * @brief 单个 GuiDocument 的渲染参数管理器。
 *
 * RenderingManager 随 GuiDocument 创建，保存该文档/view 当前渲染 profile；
 * Options Apply 只投递 dirty flags，manager 使用 0ms QTimer 合并应用，避免
 * dialog 点击 Apply 时同步遍历大模型阻塞 UI。
 */
class RenderingManager : public QObject
{
    Q_OBJECT
public:
    /// 创建与一个 GuiDocument 绑定的渲染管理器。
    explicit RenderingManager(GuiDocument* document, QObject* parent = nullptr);

    /// 设置当前文档是 CAD view 还是 CAM/机台 view。
    void setMachineView(bool machineView);

    /// 更新本 manager 的 profile 与颜色配置，不立即重绘。
    void configure(const RenderProfileSettings& profile, const ColorSettings& colors);

    /// 请求异步应用 dirty 参数；多个请求会在下一次事件循环合并。
    void requestApply(RenderDirtyFlags flags);

    /// 立即应用 dirty 参数；用于新 view 创建后的初始化。
    void applyNow(RenderDirtyFlags flags);

    /// 应用当前 view 的运行时显示模式（Ribbon 使用），不写 AppSettings。
    void setRuntimeDisplayMode(int displayMode, bool faceBoundary);

    /// 当前 view 运行时 displayMode。
    int runtimeDisplayMode() const { return m_runtimeDisplayMode; }

    /// 当前 view 是否运行时显示面边线。
    bool runtimeFaceBoundary() const { return m_runtimeFaceBoundary; }

    /// 按当前配置批量更新文档中所有 AIS_Shape 的颜色、材质与质量。
    void applyDocumentStyles(const QMap<QString, Handle(AIS_Shape)>& aisMap);

    /// 当前 profile（已按 view 类型选择）。
    const RenderProfileSettings& profile() const { return m_profile; }

    /// 当前颜色配置。
    const ColorSettings& colors() const { return m_colors; }

private:
    RenderProfileSettings effectiveProfile() const;
    void applyViewRenderingParams();
    void applyBackground();
    void applyHighlight();
    void applyLighting();
    void applyShapeStyle(const QString& entry,
                         const Handle(AIS_Shape)& ais,
                         bool isMachineShape,
                         bool isWorkpieceShape,
                         const QString& axisName);
    void flushPendingApply();

    GuiDocument* m_document{nullptr};
    QTimer* m_applyTimer{nullptr};
    RenderDirtyFlags m_pendingFlags{RenderDirtyFlag::None};
    RenderProfileSettings m_profile;
    ColorSettings m_colors;
    bool m_machineView{false};
    int m_runtimeDisplayMode{1};
    bool m_runtimeFaceBoundary{false};
};

} // namespace lcnc::view

Q_DECLARE_OPERATORS_FOR_FLAGS(lcnc::view::RenderDirtyFlags)
