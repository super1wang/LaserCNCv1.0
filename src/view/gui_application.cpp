#include "view/gui_application.h"
#include "view/gui_document.h"
#include "view/graphics_scene.h"
#include "view/rendering_manager.h"
#include "core/document/lcnc_application.h"
#include "core/kernel/kernel.h"
#include "core/logging/logger.h"

#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <AIS_ListOfInteractive.hxx>
#include <AIS_ListIteratorOfListOfInteractive.hxx>
#include <Prs3d_Drawer.hxx>

// ── Singleton accessor ────────────────────────────────────────────────────────
//   Lifecycle owned by lcnc::Kernel — see kernel.cpp::registerCoreServices.
//   Use lcnc::Kernel::current().guiApp() to access this object.
GuiApplication* GuiApplication::s_instance = nullptr;

GuiApplication::GuiApplication(QObject* parent)
    : QObject(parent)
{
    Q_ASSERT_X(!s_instance, "GuiApplication",
               "second GuiApplication instance — must be Kernel-owned only");
    s_instance = this;

    LcncApplication* app = lcnc::Kernel::current().app();
    connect(app, &LcncApplication::documentAdded,
            this, &GuiApplication::onDocumentAdded);
    connect(app, &LcncApplication::documentClosed,
            this, &GuiApplication::onDocumentClosed);
    connect(app, &LcncApplication::activeDocumentChanged,
            this, &GuiApplication::onActiveDocumentChanged);
}

GuiApplication::~GuiApplication()
{
    if (s_instance == this) s_instance = nullptr;
}

// ── Slots ──────────────────────────────────────────────────────────────────────
void GuiApplication::onDocumentAdded(DocumentId id)
{
    auto* guiDoc = new GuiDocument(id, this);
    m_guiDocs.insert(id, guiDoc);
    emit guiDocumentAdded(id);
}

void GuiApplication::onDocumentClosed(DocumentId id)
{
    if (GuiDocument* gd = m_guiDocs.take(id)) {
        emit guiDocumentClosed(id);
        delete gd;
    }
}

void GuiApplication::onActiveDocumentChanged(DocumentId id)
{
    emit activeGuiDocumentChanged(id);
}

// ── Accessors ──────────────────────────────────────────────────────────────────
GuiDocument* GuiApplication::guiDocument(DocumentId id) const
{
    return m_guiDocs.value(id, nullptr);
}

QList<GuiDocument*> GuiApplication::guiDocuments() const
{
    return m_guiDocs.values();
}

GuiDocument* GuiApplication::activeGuiDocument() const
{
    return guiDocument(lcnc::Kernel::current().app()->activeDocumentId());
}

GuiDocument* GuiApplication::machineGuiDocument() const
{
    return guiDocument(lcnc::Kernel::current().app()->machineDocumentId());
}

// ── 全局显示模式 ───────────────────────────────────────────────────────────────
void GuiApplication::setCurrentDisplayMode(int displayMode, bool faceBoundary)
{
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "GuiApplication::setCurrentDisplayMode mode={} edges={}",
               displayMode, faceBoundary);
    m_currentDisplayMode  = displayMode;
    m_currentFaceBoundary = faceBoundary;

    auto applyToScene = [displayMode, faceBoundary](GraphicsScene* scene) {
        if (!scene) return;
        const Handle(AIS_InteractiveContext)& ctx = scene->context();
        if (ctx.IsNull()) return;
        // 1) 默认 drawer 同步 — 影响后续新建对象与 SetDisplayMode 全局调用
        ctx->DefaultDrawer()->SetFaceBoundaryDraw(faceBoundary);
        // 2) 全局默认 displayMode（不会覆盖已经被 per-object SetDisplayMode 设过的对象）
        ctx->SetDisplayMode(displayMode, Standard_False);
        // 3) 强制覆盖每一个 AIS_Shape 的 per-object displayMode + 面边线
        AIS_ListOfInteractive list;
        ctx->DisplayedObjects(list);
        for (AIS_ListIteratorOfListOfInteractive it(list); it.More(); it.Next()) {
            const Handle(AIS_InteractiveObject)& obj = it.Value();
            if (Handle(AIS_Shape) sh = Handle(AIS_Shape)::DownCast(obj); !sh.IsNull()) {
                ctx->SetDisplayMode(sh, displayMode, Standard_False);
                if (!sh->Attributes().IsNull()) {
                    sh->Attributes()->SetFaceBoundaryDraw(faceBoundary);
                }
                ctx->Redisplay(sh, Standard_False);
            }
        }
        ctx->UpdateCurrentViewer();
    };

    int sceneCount = 0;
    for (auto* gd : m_guiDocs) {
        if (gd && gd->scene()) { applyToScene(gd->scene()); ++sceneCount; }
    }
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "GuiApplication::setCurrentDisplayMode applied to {} scenes",
               sceneCount);
}

void GuiApplication::requestApplyRenderingSettings(
    const lcnc::RenderProfileSettings& cadProfile,
    const lcnc::RenderProfileSettings& camProfile,
    const lcnc::ColorSettings& colors,
    lcnc::view::RenderDirtyFlags dirtyFlags,
    bool applyCadViews,
    bool applyCamView)
{
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "GuiApplication::requestApplyRenderingSettings flags={} cad={} cam={}",
               static_cast<int>(dirtyFlags.toInt()), applyCadViews, applyCamView);
    const DocumentId machineId = lcnc::Kernel::current().app()->machineDocumentId();
    int requested = 0;
    for (auto* gd : m_guiDocs) {
        if (!gd || !gd->renderingManager())
            continue;
        const bool isMachineDoc = gd->documentId() == machineId;
        if ((isMachineDoc && !applyCamView) || (!isMachineDoc && !applyCadViews))
            continue;

        gd->renderingManager()->setMachineView(isMachineDoc);
        gd->renderingManager()->configure(isMachineDoc ? camProfile : cadProfile, colors);
        gd->renderingManager()->requestApply(dirtyFlags);
        ++requested;
    }
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "GuiApplication::requestApplyRenderingSettings requested {} docs", requested);
}
