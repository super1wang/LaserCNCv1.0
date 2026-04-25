#pragma once

#include <QObject>
#include <QList>
#include <QMap>
#include "core/document/lcnc_application.h"
#include "core/settings/app_settings.h"
#include "view/rendering_manager.h"

class GuiDocument;
class GraphicsScene;

/**
 * @brief GUI-layer application manager.
 *
 * Mirrors LcncApplication's document list but attaches a GuiDocument (which
 * owns a GraphicsScene) to every open document.  All 3D scene operations
 * should go through the GuiDocument, not directly to the OCC viewer.
 */
class GuiApplication : public QObject
{
    Q_OBJECT
public:

    // ── GUI document access ───────────────────────────────────────────────────
    GuiDocument*         guiDocument(DocumentId id) const;
    QList<GuiDocument*>  guiDocuments() const;
    GuiDocument*         activeGuiDocument() const;    GuiDocument*         machineGuiDocument() const;

    // ── 全局显示模式（线框/着色/带边着色） ────────────────────────────────
    /// 当前默认 displayMode（AIS_WireFrame=0 / AIS_Shaded=1）。新建 AIS 时使用，
    /// 由 commands_display 中的三个互斥命令统一更新。
    int  currentDisplayMode()       const { return m_currentDisplayMode; }
    bool currentFaceBoundaryDraw()  const { return m_currentFaceBoundary; }
    /// 应用全局 displayMode 到所有现有 GuiDocument 的所有 AIS_Shape，并保存为后续新建 AIS 的默认。
    /// 自动同步 context 默认 drawer 的 FaceBoundaryDraw 状态。
    void setCurrentDisplayMode(int displayMode, bool faceBoundary);

    /// 将应用程序选项中的 CAD/CAM 渲染配置投递给现有 GuiDocument。
    /// 根据 @p applyCadViews / @p applyCamView 只刷新需要变更的视图，dirty flags 由 Options diff 生成。
    void requestApplyRenderingSettings(const lcnc::RenderProfileSettings& cadProfile,
                                       const lcnc::RenderProfileSettings& camProfile,
                                       const lcnc::ColorSettings& colors,
                                       lcnc::view::RenderDirtyFlags dirtyFlags,
                                       bool applyCadViews,
                                       bool applyCamView);

signals:
    void guiDocumentAdded(DocumentId id);
    void guiDocumentClosed(DocumentId id);
    void activeGuiDocumentChanged(DocumentId id);

public:
    /// Constructed once by lcnc::Kernel during registerCoreServices, after
    /// LcncApplication is in place. Ctor wires LcncApplication signals.
    explicit GuiApplication(QObject* parent = nullptr);
    ~GuiApplication() override;

private:
    static GuiApplication* s_instance;

    void onDocumentAdded(DocumentId id);
    void onDocumentClosed(DocumentId id);
    void onActiveDocumentChanged(DocumentId id);

    QMap<DocumentId, GuiDocument*> m_guiDocs;

    int  m_currentDisplayMode{1};      ///< AIS_Shaded
    bool m_currentFaceBoundary{false}; ///< 是否绘制面边线（带边着色）
};
