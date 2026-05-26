#include "view/gui_document.h"
#include "core/kernel/kernel.h"
#include "core/logging/logger.h"
#include "core/settings/app_settings.h"
#include "core/document/lcnc_document.h"
#include "core/document/xcaf_utils.h"
#include "core/kinematics/machine_kinematics.h"
#include "core/project/lcnc_project_manager.h"
#include "view/rendering_manager.h"
#include "view/shape_object_driver.h"

#include <TDF_LabelSequence.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <TDataStd_Integer.hxx>

// View and gizmo includes
#include <AIS_ViewCube.hxx>
#include <AIS_Trihedron.hxx>
#include <Geom_Axis2Placement.hxx>
#include <Prs3d_DatumAspect.hxx>
#include <Prs3d_DatumParts.hxx>
#include <Graphic3d_TransformPers.hxx>
#include <Graphic3d_MaterialAspect.hxx>
#include <Graphic3d_AspectFillArea3d.hxx>
#include <Aspect_TypeOfTriedronPosition.hxx>
#include <V3d_Viewer.hxx>
#include <QTimer>
#include <QSet>
#include <QList>

namespace {

Quantity_Color axisDisplayColor(const QString& axisName)
{
    if (axisName == QStringLiteral("BASE"))
        return Quantity_Color(0.62, 0.64, 0.68, Quantity_TOC_RGB);
    if (axisName == QStringLiteral("X"))
        return Quantity_Color(0.90, 0.27, 0.18, Quantity_TOC_RGB);
    if (axisName == QStringLiteral("Y"))
        return Quantity_Color(0.14, 0.66, 0.28, Quantity_TOC_RGB);
    if (axisName == QStringLiteral("Z"))
        return Quantity_Color(0.18, 0.48, 0.94, Quantity_TOC_RGB);
    if (axisName == QStringLiteral("A"))
        return Quantity_Color(0.93, 0.60, 0.08, Quantity_TOC_RGB);
    if (axisName == QStringLiteral("B"))
        return Quantity_Color(0.10, 0.70, 0.70, Quantity_TOC_RGB);
    if (axisName == QStringLiteral("C"))
        return Quantity_Color(0.76, 0.23, 0.79, Quantity_TOC_RGB);
    return Quantity_Color(0.78, 0.78, 0.80, Quantity_TOC_RGB);
}

} // namespace

GuiDocument::GuiDocument(QObject* parent)
    : QObject(parent)
    , m_scene(new GraphicsScene(this))
    , m_renderingManager(new lcnc::view::RenderingManager(this, this))
{
    m_renderingManager->setMachineView(true);
    if (auto* settings = lcnc::Kernel::current().appSettings()) {
        m_renderingManager->configure(
            settings->camViewRendering,
            settings->colors);
        m_renderingManager->applyNow(lcnc::view::RenderDirtyFlag::All);
    }
}

GuiDocument::~GuiDocument() = default;

void GuiDocument::setSourceDocument(LcncDocument* document)
{
    m_sourceDocument = document;
}

const Handle(AIS_InteractiveContext)& GuiDocument::context() const
{
    return m_scene->context();
}

void GuiDocument::attachView(const Handle(Aspect_NeutralWindow)& win, int w, int h)
{
    if (!m_view.IsNull()) {
        // Already created — just sync window size so resize works correctly
        win->SetSize(w, h);
        m_view->MustBeResized();
        return;
    }

    // ── First-time view creation ──────────────────────────────────────────
    win->SetSize(w, h);
    m_view = m_scene->viewer()->CreateView();
    m_view->SetWindow(win);
    if (!win->IsMapped())
        win->Map();

    m_view->MustBeResized();

    // 必须先把 RenderingManager 当前运行时显示模式（context 默认 + 已 Display 的形体
    // 模式 + face boundary）应用到上下文，再触发首次 Redraw 让 OCC 计算所有形体的
    // 表示。否则 FitAll 在表示尚未生成时会拿不到包围盒，导致首次 attach 时模型不可见。
    if (m_renderingManager)
        m_renderingManager->applyNow(lcnc::view::RenderDirtyFlag::All);
    initGizmos();
    m_view->Redraw();
    m_scene->logOpenGlContextState("document");

    if (!m_displayObjects.isEmpty()) {
        m_view->FitAll(0.01, false);
        m_view->ZFitAll();
        m_view->Redraw();
    } else {
        m_view->ZFitAll();
    }

    // Animation timer for ViewCube rotation (owned by this QObject)
    m_animTimer = new QTimer(this);
    m_animTimer->setInterval(16); // ~60 fps
    connect(m_animTimer, &QTimer::timeout, this, [this]() {
        if (!m_viewCube.IsNull() && m_viewCube->HasAnimation()) {
            Standard_Boolean stillGoing = m_viewCube->UpdateAnimation(Standard_False);
            if (!m_view.IsNull())
                m_view->Redraw();
            if (!stillGoing || !m_viewCube->HasAnimation())
                m_animTimer->stop();
        } else {
            m_animTimer->stop();
        }
    });
}

void GuiDocument::resizeView(int w, int h)
{
    if (m_view.IsNull()) return;
    auto win = Handle(Aspect_NeutralWindow)::DownCast(m_view->Window());
    if (!win.IsNull()) win->SetSize(w, h);
    m_view->MustBeResized();
}

void GuiDocument::fitAll()
{
    if (m_view.IsNull()) return;
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "GuiDocument::fitAll count={}",
               m_displayObjects.size());
    m_view->FitAll(0.01, true);
    m_view->ZFitAll();
    m_view->Redraw();
}

void GuiDocument::initGizmos()
{
    if (m_view.IsNull()) return;
    const Handle(AIS_InteractiveContext)& ctx = m_scene->context();
    if (ctx.IsNull()) return;

    // ── RGB Coordinate Axis Trihedron (lower-left corner) ────────────────
    Handle(Geom_Axis2Placement) coordSys = new Geom_Axis2Placement(gp::XOY());
    m_trihedron = new AIS_Trihedron(coordSys);
    m_trihedron->SetDatumDisplayMode(Prs3d_DM_WireFrame);
    m_trihedron->SetDatumPartColor(Prs3d_DatumParts_XAxis,  Quantity_Color(1.0, 0.0, 0.0, Quantity_TOC_RGB));
    m_trihedron->SetDatumPartColor(Prs3d_DatumParts_XArrow, Quantity_Color(1.0, 0.0, 0.0, Quantity_TOC_RGB));
    m_trihedron->SetDatumPartColor(Prs3d_DatumParts_YAxis,  Quantity_Color(0.0, 0.8, 0.0, Quantity_TOC_RGB));
    m_trihedron->SetDatumPartColor(Prs3d_DatumParts_YArrow, Quantity_Color(0.0, 0.8, 0.0, Quantity_TOC_RGB));
    m_trihedron->SetDatumPartColor(Prs3d_DatumParts_ZAxis,  Quantity_Color(0.2, 0.5, 1.0, Quantity_TOC_RGB));
    m_trihedron->SetDatumPartColor(Prs3d_DatumParts_ZArrow, Quantity_Color(0.2, 0.5, 1.0, Quantity_TOC_RGB));
    const Handle(Prs3d_DatumAspect)& da = m_trihedron->Attributes()->DatumAspect();
    if (!da.IsNull()) {
        for (Prs3d_DatumParts part : {Prs3d_DatumParts_XAxis, Prs3d_DatumParts_YAxis, Prs3d_DatumParts_ZAxis,
                                      Prs3d_DatumParts_XArrow, Prs3d_DatumParts_YArrow, Prs3d_DatumParts_ZArrow})
            da->LineAspect(part)->SetWidth(2.5);
    }
    m_trihedron->SetSize(60.0);
    Handle(Graphic3d_TransformPers) tPers =
        new Graphic3d_TransformPers(Graphic3d_TMF_TriedronPers,
                                    Aspect_TOTP_LEFT_LOWER,
                                    Graphic3d_Vec2i(70, 70));
    m_trihedron->SetTransformPersistence(tPers);
    ctx->Display(m_trihedron, AIS_WireFrame, 0, false);
    ctx->Deactivate(m_trihedron); // not selectable

    // ── ViewCube (upper-right corner) ─────────────────────────────────────
    m_viewCube = new AIS_ViewCube();
    m_viewCube->SetSize(70.0);
    const Quantity_Color silver(0.80, 0.80, 0.80, Quantity_TOC_RGB);
    m_viewCube->SetBoxColor(silver);
    m_viewCube->SetTextColor(Quantity_Color(0.15, 0.15, 0.15, Quantity_TOC_RGB));
    // Keep SetAutoStartAnimation at its default (true) so StartAnimation() works.
    // WidgetOccView calls StartAnimation() directly (not HandleClick) to avoid
    // the internal guard that HandleClick has.
    m_viewCube->SetDuration(0.35);
    Handle(Graphic3d_TransformPers) vPers =
        new Graphic3d_TransformPers(Graphic3d_TMF_TriedronPers,
                                    Aspect_TOTP_RIGHT_UPPER,
                                    Graphic3d_Vec2i(110, 110));
    m_viewCube->SetTransformPersistence(vPers);
    ctx->Display(m_viewCube, false);

    // Apply self-lit material so all faces remain visible regardless of lighting
    auto applyBrightMat = [](const Handle(Prs3d_ShadingAspect)& asp,
                              const Quantity_Color& col) {
        if (asp.IsNull()) return;
        Handle(Graphic3d_AspectFillArea3d) fa = asp->Aspect();
        Graphic3d_MaterialAspect mat = fa->FrontMaterial();
        mat.SetAmbientColor (col);
        mat.SetDiffuseColor (Quantity_Color(col.Red()  * 0.5,
                                            col.Green()* 0.5,
                                            col.Blue() * 0.5, Quantity_TOC_RGB));
        mat.SetEmissiveColor(Quantity_Color(col.Red()  * 0.55,
                                            col.Green()* 0.55,
                                            col.Blue() * 0.55, Quantity_TOC_RGB));
        mat.SetSpecularColor(Quantity_Color(0.15, 0.15, 0.15, Quantity_TOC_RGB));
        fa->SetFrontMaterial(mat);
        fa->SetBackMaterial(mat);
    };
    applyBrightMat(m_viewCube->Attributes()->ShadingAspect(), silver);
    applyBrightMat(m_viewCube->BoxEdgeStyle(),   Quantity_Color(0.70, 0.70, 0.70, Quantity_TOC_RGB));
    applyBrightMat(m_viewCube->BoxCornerStyle(), silver);
    m_viewCube->SynchronizeAspects();
}

Handle(AIS_Shape) GuiDocument::displayShape(const TopoDS_Shape& shape,
                                             const QString&      name,
                                             bool                fitAll)
{
    return displayShape(domainForDocument(m_sourceDocument), m_sourceDocument, shape, name, fitAll);
}

Handle(AIS_Shape) GuiDocument::displayShape(lcnc::ProjectDomain domain,
                                             LcncDocument* document,
                                             const TopoDS_Shape& shape,
                                             const QString& name,
                                             bool fitAll)
{
    Handle(AIS_Shape) ais = m_scene->displayShape(shape, fitAll, true, false);
    registerDisplayObject(domain, document, name, ais);
    applyMachineDisplayStyle();
    if (m_renderingManager)
        m_renderingManager->setRuntimeDisplayMode(m_renderingManager->runtimeDisplayMode(),
                                                  m_renderingManager->runtimeFaceBoundary());
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "GuiDocument::displayShape name={} fit={} count={}",
               name.toStdString(), fitAll, m_displayObjects.size());
    if (!m_view.IsNull()) {
        if (fitAll) {
            m_view->FitAll(0.01, false);
            m_view->ZFitAll();
        }
        m_view->Redraw();
    }
    emit displayUpdated();
    return ais;
}

void GuiDocument::eraseEntity(const QString& labelEntry)
{
    if (labelEntry.isEmpty())
        return;

    QList<DisplayKey> matches;
    for (auto it = m_displayObjects.cbegin(); it != m_displayObjects.cend(); ++it) {
        if (it.value().entry == labelEntry)
            matches.append(it.key());
    }

    if (matches.size() == 1 && eraseKey(matches.first()))
        emit displayUpdated();
}

void GuiDocument::eraseEntity(DocumentId documentId, const QString& labelEntry)
{
    if (labelEntry.isEmpty())
        return;

    if (eraseKey(DisplayKey{documentId, labelEntry}))
        emit displayUpdated();
}

void GuiDocument::eraseDocument(DocumentId documentId)
{
    if (eraseDocumentObjects(documentId))
        emit displayUpdated();
}

void GuiDocument::eraseDomain(lcnc::ProjectDomain domain)
{
    const int beforeDomainCount = displayObjectCount(domain);
    const int beforeTotalCount = m_displayObjects.size();
    if (eraseDomainObjects(domain))
        emit displayUpdated();
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "GuiDocument::eraseDomain domain={} beforeDomain={} beforeTotal={} afterDomain={} afterTotal={}",
               static_cast<int>(domain),
               beforeDomainCount,
               beforeTotalCount,
               displayObjectCount(domain),
               m_displayObjects.size());
}

void GuiDocument::rebuildDisplay(LcncDocument* document)
{
    LcncDocument* doc = document ? document : m_sourceDocument;
    if (!doc) {
        LCNC_DEBUG(lcnc::LogCode::Generic,
                   "GuiDocument::rebuildDisplay no source document");
        return;
    }

    rebuildDomain(domainForDocument(doc), doc);
}

void GuiDocument::rebuildDomain(lcnc::ProjectDomain domain, LcncDocument* document)
{
    const int beforeDomainCount = displayObjectCount(domain);
    const int beforeTotalCount = m_displayObjects.size();
    eraseDomainObjects(domain, false);

    if (!document) {
        LCNC_DEBUG(lcnc::LogCode::Generic,
                   "GuiDocument::rebuildDomain domain={} no document beforeDomain={} beforeTotal={} afterTotal={}",
                   static_cast<int>(domain),
                   beforeDomainCount,
                   beforeTotalCount,
                   m_displayObjects.size());
        emit displayUpdated();
        return;
    }

    m_sourceDocument = document;

    // Erase only tracked shapes — do NOT call eraseAll() which would also
    // erase the overlay gizmos (ViewCube / Trihedron) stored in the same context.
    Handle(XCAFDoc_ShapeTool) st = document->shapeTool();
    TDF_LabelSequence freeShapes;
    st->GetFreeShapes(freeShapes);

    for (int i = 1; i <= freeShapes.Length(); ++i) {
        TDF_Label lbl   = freeShapes.Value(i);
        TopoDS_Shape sh = XcafUtils::shape(lbl);
        if (!sh.IsNull()) {
            Handle(AIS_Shape) ais = m_scene->displayShape(sh, false, true, false);
            registerDisplayObject(domain, document, XcafUtils::entry(lbl), ais);
        }
    }

    applyMachineDisplayStyle();
    if (m_renderingManager)
        m_renderingManager->setRuntimeDisplayMode(m_renderingManager->runtimeDisplayMode(),
                                                  m_renderingManager->runtimeFaceBoundary());
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "GuiDocument::rebuildDomain domain={} docId={} beforeDomain={} beforeTotal={} freeShapes={} afterDomain={} afterTotal={}",
               static_cast<int>(domain),
               document->id(),
               beforeDomainCount,
               beforeTotalCount,
               freeShapes.Length(),
               displayObjectCount(domain),
               m_displayObjects.size());

    const Handle(AIS_InteractiveContext)& ctx = m_scene->context();
    if (!ctx.IsNull())
        ctx->UpdateCurrentViewer();
    if (!m_view.IsNull()) {
        m_view->Redraw();
    }

    emit displayUpdated();
}

void GuiDocument::setRenderQualityPreset(lcnc::RenderQualityPreset quality)
{
    if (m_renderQualityPreset == quality)
        return;

    m_renderQualityPreset = quality;
    if (m_renderingManager) {
        lcnc::RenderProfileSettings profile = m_renderingManager->profile();
        switch (quality) {
        case lcnc::RenderQualityPreset::High:
            profile.qualityPreset = lcnc::RenderQualityPreset::High;
            break;
        case lcnc::RenderQualityPreset::Low:
            profile.qualityPreset = lcnc::RenderQualityPreset::Low;
            break;
        case lcnc::RenderQualityPreset::Medium:
        case lcnc::RenderQualityPreset::Custom:
            profile.qualityPreset = lcnc::RenderQualityPreset::Medium;
            break;
        }
        m_renderingManager->configure(profile, m_renderingManager->colors());
        m_renderingManager->requestApply(
            lcnc::view::RenderDirtyFlags(lcnc::view::RenderDirtyFlag::Profile) |
            lcnc::view::RenderDirtyFlag::Colors);
        return;
    }

    applyMachineDisplayStyle();
}

void GuiDocument::applyMachineDisplayStyle()
{
    if (m_renderingManager) {
        QMap<DocumentId, LcncDocument*> documents;
        QMap<DocumentId, QMap<QString, Handle(AIS_Shape)>> shapesByDocument;
        for (auto it = m_displayObjects.cbegin(); it != m_displayObjects.cend(); ++it) {
            const DisplayObject& object = it.value();
            if (!object.document || object.ais.IsNull())
                continue;
            documents.insert(object.documentId, object.document);
            shapesByDocument[object.documentId].insert(object.entry, object.ais);
        }

        LcncDocument* previousSource = m_sourceDocument;
        for (auto it = shapesByDocument.cbegin(); it != shapesByDocument.cend(); ++it) {
            m_sourceDocument = documents.value(it.key(), nullptr);
            m_renderingManager->applyDocumentStyles(it.value());
        }
        m_sourceDocument = previousSource;
    }
}

Handle(AIS_Shape) GuiDocument::aisShape(const QString& labelEntry) const
{
    if (labelEntry.isEmpty())
        return {};

    if (m_sourceDocument) {
        Handle(AIS_Shape) ais = aisShape(m_sourceDocument->id(), labelEntry);
        if (!ais.IsNull())
            return ais;
    }

    Handle(AIS_Shape) found;
    int matches = 0;
    for (auto it = m_displayObjects.cbegin(); it != m_displayObjects.cend(); ++it) {
        if (it.value().entry != labelEntry)
            continue;
        found = it.value().ais;
        ++matches;
        if (matches > 1)
            return {};
    }
    return matches == 1 ? found : Handle(AIS_Shape)();
}

Handle(AIS_Shape) GuiDocument::aisShape(DocumentId documentId, const QString& labelEntry) const
{
    return m_displayObjects.value(DisplayKey{documentId, labelEntry}).ais;
}

void GuiDocument::setEntitySelectionMode(int selectionMode)
{
    const Handle(AIS_InteractiveContext)& ctx = m_scene->context();
    if (ctx.IsNull())
        return;

    for (auto it = m_displayObjects.cbegin(); it != m_displayObjects.cend(); ++it) {
        const Handle(AIS_Shape)& ais = it.value().ais;
        if (ais.IsNull())
            continue;

        ctx->Deactivate(ais);
        ctx->Activate(ais, selectionMode, Standard_False);
    }
}

// ── Axis transform update ──────────────────────────────────────────────────────

QStringList GuiDocument::selectedEntries() const
{
    return selectedEntries(kInvalidDocumentId);
}

QStringList GuiDocument::selectedEntries(DocumentId documentId) const
{
    QStringList result;
    if (!m_scene) return result;
    const Handle(AIS_InteractiveContext)& ctx = m_scene->context();
    if (ctx.IsNull()) return result;
    for (ctx->InitSelected(); ctx->MoreSelected(); ctx->NextSelected()) {
        // Compare raw pointers — Handle equality compares underlying object addresses
        const AIS_InteractiveObject* objPtr = ctx->SelectedInteractive().get();
        for (auto it = m_displayObjects.cbegin(); it != m_displayObjects.cend(); ++it) {
            const DisplayObject& object = it.value();
            if (documentId != kInvalidDocumentId && object.documentId != documentId)
                continue;
            if (object.ais.get() == objPtr) {
                result << object.entry;
                break;
            }
        }
    }
    return result;
}

void GuiDocument::updateAxisTransforms(LcncDocument* document)
{
    LcncDocument* doc = document ? document : m_sourceDocument;
    if (!doc) return;

    MachineKinematics* kin = doc->machineKinematics();
    const Handle(AIS_InteractiveContext)& ctx = m_scene->context();
    if (ctx.IsNull()) return;

    const DocumentId documentId = doc->id();
    for (auto it = m_displayObjects.constBegin(); it != m_displayObjects.constEnd(); ++it) {
        const DisplayObject& object = it.value();
        if (object.documentId != documentId)
            continue;

        const QString& entry = object.entry;
        const Handle(AIS_Shape)& ais = object.ais;
        if (ais.IsNull()) continue;

        gp_Trsf t;
        const QString machineAxis = kin->axisForShape(entry);
        const QString wpcAxis     = kin->mountedAxis(entry);

        if (!machineAxis.isEmpty())
            t = kin->computeShapeTransform(entry);
        else if (!wpcAxis.isEmpty())
            t = kin->computeWpcTransform(entry);
        else
            continue;  // not assigned — leave transform unchanged

        ais->SetLocalTransformation(t);
        ctx->RecomputePrsOnly(ais, Standard_False);
    }
}

void GuiDocument::updateMachineWorkspaceTransforms(LcncDocument* machineDocument,
                                                   LcncDocument* workpieceDocument)
{
    if (!machineDocument)
        return;

    MachineKinematics* kin = machineDocument->machineKinematics();
    const Handle(AIS_InteractiveContext)& ctx = m_scene->context();
    if (!kin || ctx.IsNull())
        return;

    const DocumentId machineDocumentId = machineDocument->id();
    const DocumentId workpieceDocumentId = workpieceDocument ? workpieceDocument->id() : kInvalidDocumentId;
    for (auto it = m_displayObjects.constBegin(); it != m_displayObjects.constEnd(); ++it) {
        const DisplayObject& object = it.value();
        const bool machineObject = object.documentId == machineDocumentId;
        const bool workpieceObject = object.documentId == workpieceDocumentId;
        if (!machineObject && !workpieceObject)
            continue;

        const Handle(AIS_Shape)& ais = object.ais;
        if (ais.IsNull())
            continue;

        gp_Trsf t;
        bool hasTransform = false;
        if (machineObject) {
            const QString machineAxis = kin->axisForShape(object.entry);
            if (!machineAxis.isEmpty()) {
                t = kin->computeShapeTransform(object.entry);
                hasTransform = true;
            } else if (!kin->mountedAxis(object.entry).isEmpty()) {
                t = kin->computeWpcTransform(object.entry);
                hasTransform = true;
            }
        } else if (workpieceObject) {
            const QString wpcAxis = kin->mountedAxis(object.entry);
            if (!wpcAxis.isEmpty()) {
                t = kin->computeWpcTransform(object.entry);
                hasTransform = true;
            } else {
                hasTransform = true;
            }
        }

        if (!hasTransform)
            continue;

        ais->SetLocalTransformation(t);
        ctx->RecomputePrsOnly(ais, Standard_False);
    }
}

lcnc::ProjectDomain GuiDocument::domainForDocument(LcncDocument* document) const
{
    if (!document)
        return lcnc::ProjectDomain::Project;

    lcnc::ProjectDomain domain = lcnc::ProjectDomain::Project;
    if (auto* manager = lcnc::Kernel::current().projectManager())
        manager->domainForDocument(document->id(), &domain);
    return domain;
}

bool GuiDocument::eraseKey(const DisplayKey& key, bool updateViewer)
{
    auto it = m_displayObjects.find(key);
    if (it == m_displayObjects.end())
        return false;

    if (!it.value().ais.IsNull())
        m_scene->eraseShape(it.value().ais, updateViewer);
    m_displayObjects.erase(it);
    return true;
}

bool GuiDocument::eraseKeys(const QList<DisplayKey>& keys, bool updateViewer)
{
    bool changed = false;
    for (const DisplayKey& key : keys)
        changed = eraseKey(key, false) || changed;
    if (changed && updateViewer && m_scene && !m_scene->context().IsNull())
        m_scene->context()->UpdateCurrentViewer();
    return changed;
}

bool GuiDocument::eraseDocumentObjects(DocumentId documentId)
{
    QList<DisplayKey> keys;
    for (auto it = m_displayObjects.cbegin(); it != m_displayObjects.cend(); ++it) {
        if (it.value().documentId == documentId)
            keys.append(it.key());
    }
    return eraseKeys(keys);
}

int GuiDocument::displayObjectCount(lcnc::ProjectDomain domain) const
{
    int count = 0;
    for (auto it = m_displayObjects.cbegin(); it != m_displayObjects.cend(); ++it) {
        if (it.value().domain == domain)
            ++count;
    }
    return count;
}

bool GuiDocument::eraseDomainObjects(lcnc::ProjectDomain domain, bool updateViewer)
{
    QList<DisplayKey> keys;
    for (auto it = m_displayObjects.cbegin(); it != m_displayObjects.cend(); ++it) {
        if (it.value().domain == domain)
            keys.append(it.key());
    }
    return eraseKeys(keys, updateViewer);
}

void GuiDocument::registerDisplayObject(lcnc::ProjectDomain domain,
                                        LcncDocument* document,
                                        const QString& entry,
                                        const Handle(AIS_Shape)& ais)
{
    if (entry.isEmpty() || ais.IsNull())
        return;

    const DocumentId documentId = document ? document->id() : kInvalidDocumentId;
    DisplayObject object;
    object.domain = domain;
    object.documentId = documentId;
    object.document = document;
    object.entry = entry;
    object.ais = ais;
    m_displayObjects.insert(DisplayKey{documentId, entry}, object);
}
