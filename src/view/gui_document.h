#pragma once

#include <QObject>
#include <QList>
#include <QMap>
#include <Standard_Handle.hxx>
#include <AIS_Shape.hxx>
#include <V3d_View.hxx>
#include <AIS_InteractiveContext.hxx>
#include <Aspect_NeutralWindow.hxx>

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
    /// Returns label entries of all currently selected AIS shapes.
    QStringList selectedEntries() const;
    QStringList selectedEntries(DocumentId documentId) const;
    // ── Axis transform update ─────────────────────────────────────────────
    void setEntitySelectionMode(int selectionMode);
    /// Recomputes and applies AIS local transforms for all axis-assigned
    /// machine shapes and all mounted workpieces at their current positions.
    void updateAxisTransforms(LcncDocument* document = nullptr);
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
        DocumentId documentId{kInvalidDocumentId};
        LcncDocument* document{nullptr};
        QString entry;
        Handle(AIS_Shape) ais;
    };

    void initGizmos();   ///< Called once inside attachView() after m_view is created
    lcnc::ProjectDomain domainForDocument(LcncDocument* document) const;
    bool eraseKey(const DisplayKey& key);
    bool eraseKeys(const QList<DisplayKey>& keys);
    bool eraseDocumentObjects(DocumentId documentId);
    bool eraseDomainObjects(lcnc::ProjectDomain domain);
    void registerDisplayObject(lcnc::ProjectDomain domain,
                               LcncDocument* document,
                               const QString& entry,
                               const Handle(AIS_Shape)& ais);

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
