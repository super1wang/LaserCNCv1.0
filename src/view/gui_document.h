#pragma once

#include <QObject>
#include <QList>
#include <QMap>
#include <QSet>
#include <QString>
#include <QVector>
#include <Standard_Handle.hxx>
#include <AIS_Shape.hxx>
#include <V3d_View.hxx>
#include <AIS_InteractiveContext.hxx>
#include <Aspect_NeutralWindow.hxx>

#include <cstdint>

#include "core/project/project_types.h"
#include "core/settings/app_settings.h"
#include "view/graphics_scene.h"

class AIS_ViewCube;
class AIS_Trihedron;
class LcncDocument;
class QTimer;

namespace lcnc::view {
class RenderingManager;
}

/**
 * @brief GUI-layer wrapper for the shared workspace view.
 *
 * Owns one GraphicsScene and one V3d_View. Project data is supplied explicitly
 * by workspace composition code; this class keeps only display objects and view
 * state.
 *
 * Gizmos (ViewCube, RGB Trihedron) are created per-document so they
 * survive document switches without needing to be re-initialised.
 */
class GuiDocument : public QObject
{
    Q_OBJECT
public:
    explicit GuiDocument(QObject* parent = nullptr);
    ~GuiDocument() override;

    void setSourceDocument(LcncDocument* document);
    LcncDocument*  sourceDocument() const { return m_sourceDocument; }
    GraphicsScene* scene()      const { return m_scene; }
    /// Returns the per-document rendering parameter manager.
    lcnc::view::RenderingManager* renderingManager() const { return m_renderingManager; }

    // ── Workspace View ────────────────────────────────────────────────────────
    bool hasView() const { return !m_view.IsNull(); }
    const Handle(V3d_View)&               view()    const { return m_view;    }
    const Handle(AIS_InteractiveContext)& context() const;

    /// Called by WidgetOccView when this workspace becomes visible.
    /// Creates the V3d_View the first time; just syncs window size afterwards.
    void attachView(const Handle(Aspect_NeutralWindow)& win, int w, int h);

    void resizeView(int w, int h);
    void fitAll();
    bool dumpWorkpiecePreview(const QString& filePath, int width, int height);

    // Gizmo accessors used by WidgetOccView for ViewCube click handling
    const Handle(AIS_ViewCube)& viewCube()  const { return m_viewCube; }
    QTimer*                     animTimer() const { return m_animTimer; }

    // ── Shape registration ────────────────────────────────────────────────────
    Handle(AIS_Shape) displayShape(const TopoDS_Shape& shape,
                                   const QString& name,
                                   bool           fitAll = false);
    Handle(AIS_Shape) displayShape(lcnc::ProjectDomain domain,
                                   LcncDocument* document,
                                   const TopoDS_Shape& shape,
                                   const QString& name,
                                   bool fitAll = false);
    void eraseEntity(const QString& labelEntry);
    void eraseEntity(DocumentId documentId, const QString& labelEntry);
    void eraseDocument(DocumentId documentId);
    void eraseDomain(lcnc::ProjectDomain domain);
    void rebuildDisplay(LcncDocument* document = nullptr);
    void rebuildDomain(lcnc::ProjectDomain domain, LcncDocument* document);
    void setRenderQualityPreset(lcnc::RenderQualityPreset quality);
    lcnc::RenderQualityPreset renderQualityPreset() const { return m_renderQualityPreset; }
    void applyMachineDisplayStyle();
    Handle(AIS_Shape) aisShape(const QString& labelEntry) const;
    Handle(AIS_Shape) aisShape(DocumentId documentId, const QString& labelEntry) const;
    /// 返回属于给定域集合的所有已注册 AIS（按 m_displayObjects 的 domain 过滤）。
    /// 供 RenderingManager 把“显示模式”切换限定到工件+机台，不波及刀路/引导/gizmo 等。
    QVector<Handle(AIS_Shape)> displayShapesForDomains(const QSet<lcnc::ProjectDomain>& domains) const;

    // ── CAM contour bodies (Phase C: ContourId-keyed AIS) ─────────────────
    // 工件/机台路径继续走 (DocumentId, entry) 键；CAM 轮廓本体改为以稳定的
    // ContourId 寻址，内部仍复用 m_displayObjects（entry 合成为 "cam:<id>"），
    // 因此 selectedEntries / applyMachineDisplayStyle / rebuildDomain 等遍历型
    // 接口对工件/机台无任何感知变化。
    /// 显示一条 CAM 轮廓体；若 contourId 已存在则替换其 AIS。
    Handle(AIS_Shape) displayContourBody(std::uint64_t contourId,
                                         const TopoDS_Shape& wire,
                                         const QString& name);
    /// 查找；不存在返回空 handle。
    Handle(AIS_Shape) aisShapeForContour(std::uint64_t contourId) const;
    /// 删除单条；不存在则忽略。
    void              eraseContour(std::uint64_t contourId);
    /// 删除所有 CAM 域的 DisplayObject（替代 eraseDomain(ProjectDomain::Cam)）。
    void              eraseAllContours();
    /// 反向：从当前选中 AIS 集中提取 contourId 列表（仅看 CAM 域 + 合成 entry 前缀）。
    QVector<std::uint64_t> selectedContourIds() const;
    /// Re-applies CAM contour picking after display/visibility changes.
    void restoreCamContourSelectionModes();

    /// Returns label entries of all currently selected AIS shapes.
    QStringList selectedEntries() const;
    QStringList selectedEntries(DocumentId documentId) const;
    // ── Axis transform update ─────────────────────────────────────────────
    void setEntitySelectionMode(int selectionMode);
    /// Recomputes and applies AIS local transforms for all axis-assigned
    /// machine shapes and all mounted workpieces at their current positions.
    void updateAxisTransforms(LcncDocument* document = nullptr);
    /// Recomputes transforms for a composed machine workspace where workpieces
    /// are displayed from the Workpiece domain but mounted by machine kinematics.
    void updateMachineWorkspaceTransforms(LcncDocument* machineDocument,
                                          LcncDocument* workpieceDocument);
signals:
    void displayUpdated();

private:
    struct DisplayKey {
        DocumentId documentId{kInvalidDocumentId};
        QString entry;

        bool operator<(const DisplayKey& other) const
        {
            if (documentId != other.documentId)
                return documentId < other.documentId;
            return entry < other.entry;
        }
    };

    struct DisplayObject {
        lcnc::ProjectDomain domain{lcnc::ProjectDomain::Project};
        int entityKind{0};
        DocumentId documentId{kInvalidDocumentId};
        LcncDocument* document{nullptr};
        QString entry;
        Handle(AIS_Shape) ais;
    };

    void initGizmos();   ///< Called once inside attachView() after m_view is created
    lcnc::ProjectDomain domainForDocument(LcncDocument* document) const;
    int displayObjectCount(lcnc::ProjectDomain domain) const;
    bool eraseKey(const DisplayKey& key, bool updateViewer = true);
    bool eraseKeys(const QList<DisplayKey>& keys, bool updateViewer = true);
    bool eraseDocumentObjects(DocumentId documentId);
    bool eraseDomainObjects(lcnc::ProjectDomain domain, bool updateViewer = true);
    void registerDisplayObject(lcnc::ProjectDomain domain,
                               LcncDocument* document,
                               int entityKind,
                               const QString& entry,
                               const Handle(AIS_Shape)& ais);
    bool fitDisplayObjects(int priority, bool update);

    LcncDocument*  m_sourceDocument{nullptr};
    GraphicsScene* m_scene{nullptr};
    lcnc::view::RenderingManager* m_renderingManager{nullptr};
    QMap<DisplayKey, DisplayObject> m_displayObjects;
    lcnc::RenderQualityPreset m_renderQualityPreset{lcnc::RenderQualityPreset::Medium};

    // Per-document view state (created once, persistent)
    Handle(V3d_View)      m_view;
    Handle(AIS_ViewCube)  m_viewCube;
    Handle(AIS_Trihedron) m_trihedron;
    QTimer*               m_animTimer{nullptr};
};
