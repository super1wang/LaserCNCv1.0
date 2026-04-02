#include "gui/gui_document.h"
#include "base/lcnc_application.h"
#include "base/lcnc_document.h"
#include "base/xcaf_utils.h"
#include "base/machine_kinematics.h"
#include "graphics/shape_object_driver.h"

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

GuiDocument::GuiDocument(DocumentId id, QObject* parent)
    : QObject(parent)
    , m_docId(id)
    , m_scene(new GraphicsScene(this))
{}

GuiDocument::~GuiDocument() = default;

LcncDocument* GuiDocument::document() const
{
    return LcncApplication::instance()->documentById(m_docId);
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

    m_view->SetBgGradientColors(
        Quantity_Color(0.30, 0.35, 0.42, Quantity_TOC_RGB),
        Quantity_Color(0.12, 0.15, 0.20, Quantity_TOC_RGB),
        Aspect_GFM_VER, false);

    m_view->MustBeResized();
    m_view->ZFitAll();
    initGizmos();
    m_view->Redraw();

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
    Handle(AIS_Shape) ais = m_scene->displayShape(shape, fitAll);
    m_aisMap.insert(name, ais);
    emit displayUpdated();
    return ais;
}

void GuiDocument::eraseEntity(const QString& labelEntry)
{
    if (m_aisMap.contains(labelEntry)) {
        m_scene->eraseShape(m_aisMap.value(labelEntry));
        m_aisMap.remove(labelEntry);
        emit displayUpdated();
    }
}

void GuiDocument::rebuildDisplay()
{
    LcncDocument* doc = document();
    if (!doc) return;

    // Erase only tracked shapes — do NOT call eraseAll() which would also
    // erase the overlay gizmos (ViewCube / Trihedron) stored in the same context.
    for (auto it = m_aisMap.constBegin(); it != m_aisMap.constEnd(); ++it)
        m_scene->eraseShape(it.value());
    m_aisMap.clear();

    Handle(XCAFDoc_ShapeTool) st = doc->shapeTool();
    TDF_LabelSequence freeShapes;
    st->GetFreeShapes(freeShapes);

    for (int i = 1; i <= freeShapes.Length(); ++i) {
        TDF_Label lbl   = freeShapes.Value(i);
        TopoDS_Shape sh = XcafUtils::shape(lbl);
        if (!sh.IsNull()) {
            Handle(AIS_Shape) ais = m_scene->displayShape(sh);
            // Use a coarser tessellation for machine entities so that AIS
            // transform updates during simulation are cheap to re-render.
            Handle(TDataStd_Integer) kindAttr;
            if (lbl.FindAttribute(TDataStd_Integer::GetID(), kindAttr) &&
                kindAttr->Get() == static_cast<int>(LcncDocument::EntityKind::Machine))
            {
                ais->SetOwnDeviationCoefficient(0.05);
                ais->SetOwnDeviationAngle(0.35); // ~20°
                m_scene->redisplayShape(ais);
            }
            m_aisMap.insert(XcafUtils::entry(lbl), ais);
        }
    }

    emit displayUpdated();
}

Handle(AIS_Shape) GuiDocument::aisShape(const QString& labelEntry) const
{
    return m_aisMap.value(labelEntry, Handle(AIS_Shape)());
}

// ── Axis transform update ──────────────────────────────────────────────────────

QStringList GuiDocument::selectedEntries() const
{
    QStringList result;
    if (!m_scene) return result;
    const Handle(AIS_InteractiveContext)& ctx = m_scene->context();
    if (ctx.IsNull()) return result;
    for (ctx->InitSelected(); ctx->MoreSelected(); ctx->NextSelected()) {
        // Compare raw pointers — Handle equality compares underlying object addresses
        const AIS_InteractiveObject* objPtr = ctx->SelectedInteractive().get();
        for (auto it = m_aisMap.cbegin(); it != m_aisMap.cend(); ++it) {
            if (it.value().get() == objPtr) {
                result << it.key();
                break;
            }
        }
    }
    return result;
}

void GuiDocument::updateAxisTransforms()
{
    LcncDocument* doc = document();
    if (!doc) return;

    MachineKinematics* kin = doc->machineKinematics();
    const Handle(AIS_InteractiveContext)& ctx = m_scene->context();
    if (ctx.IsNull()) return;

    for (auto it = m_aisMap.constBegin(); it != m_aisMap.constEnd(); ++it) {
        const QString&         entry = it.key();
        const Handle(AIS_Shape)& ais = it.value();
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

    if (!m_view.IsNull())
        m_view->Redraw();
}
