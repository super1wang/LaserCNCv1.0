#include "view/gui_application.h"

#include "core/kernel/kernel.h"
#include "core/logging/logger.h"
#include "core/project/lcnc_project_manager.h"
#include "view/graphics_scene.h"
#include "view/gui_document.h"
#include "view/rendering_manager.h"

#include <AIS_InteractiveContext.hxx>
#include <AIS_ListIteratorOfListOfInteractive.hxx>
#include <AIS_ListOfInteractive.hxx>
#include <AIS_Shape.hxx>
#include <Prs3d_Drawer.hxx>

#include <utility>

GuiApplication* GuiApplication::s_instance = nullptr;

GuiApplication::GuiApplication(QObject* parent)
    : QObject(parent)
{
    Q_ASSERT_X(!s_instance, "GuiApplication",
               "second GuiApplication instance — must be Kernel-owned only");
    s_instance = this;

    if (auto* project = lcnc::Kernel::current().projectManager()) {
        connect(project, &lcnc::LcncProjectManager::workspaceAdded,
                this, [this](ProjectWorkspaceId id) { ensureGuiDocument(id); });
        connect(project, &lcnc::LcncProjectManager::workspaceAboutToClose,
                this, &GuiApplication::closeGuiDocument);
        connect(project, &lcnc::LcncProjectManager::activeWorkspaceChanged,
                this, &GuiApplication::setActiveWorkspace);

        const ProjectWorkspaceId activeId = project->activeWorkspaceId();
        if (activeId != kInvalidProjectWorkspaceId) {
            ensureGuiDocument(activeId);
            setActiveWorkspace(activeId);
        }
    }
}

GuiApplication::~GuiApplication()
{
    if (s_instance == this)
        s_instance = nullptr;
}

GuiDocument* GuiApplication::activeGuiDocument() const
{
    return guiDocument(m_activeWorkspaceId);
}

GuiDocument* GuiApplication::guiDocument(ProjectWorkspaceId id) const
{
    return m_guiDocuments.value(id, nullptr);
}

GuiDocument* GuiApplication::ensureGuiDocument(ProjectWorkspaceId id)
{
    if (id == kInvalidProjectWorkspaceId)
        return nullptr;
    if (GuiDocument* existing = guiDocument(id))
        return existing;

    GuiDocument* document = createWorkspaceGuiDocument();
    m_guiDocuments.insert(id, document);
    emit guiDocumentReady(id, document);
    if (id == m_activeWorkspaceId) {
        emit workspaceGuiDocumentReady();
        emit workspaceGuiDocumentChanged(document);
        emit activeGuiDocumentChanged(id, document);
    }
    return document;
}

void GuiApplication::closeGuiDocument(ProjectWorkspaceId id)
{
    GuiDocument* document = m_guiDocuments.value(id, nullptr);
    if (!document)
        return;

    const bool wasActive = (id == m_activeWorkspaceId);
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "GuiApplication::closeGuiDocument workspace={} doc={}",
               id, static_cast<void*>(document));

    emit guiDocumentAboutToClose(id, document);
    if (wasActive)
        emit workspaceGuiDocumentAboutToClose(document);

    m_guiDocuments.remove(id);
    if (wasActive)
        m_activeWorkspaceId = kInvalidProjectWorkspaceId;

    document->deleteLater();
}

GuiDocument* GuiApplication::createWorkspaceGuiDocument()
{
    auto* document = new GuiDocument(this);
    applyDisplayModeToDocument(document);
    if (!m_machineCoordinateAxes.isEmpty())
        document->setMachineCoordinateFrame(m_machineCoordinateAxes);
    return document;
}

void GuiApplication::resetWorkspaceGuiDocument()
{
    const ProjectWorkspaceId id = m_activeWorkspaceId;
    if (id == kInvalidProjectWorkspaceId)
        return;

    GuiDocument* oldDocument = m_guiDocuments.value(id, nullptr);
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "GuiApplication::resetWorkspaceGuiDocument workspace={} old={}",
               id, static_cast<void*>(oldDocument));
    if (oldDocument) {
        emit guiDocumentAboutToClose(id, oldDocument);
        emit workspaceGuiDocumentAboutToClose(oldDocument);
        oldDocument->deleteLater();
    }

    GuiDocument* document = createWorkspaceGuiDocument();
    m_guiDocuments.insert(id, document);
    emit guiDocumentReady(id, document);
    emit workspaceGuiDocumentReady();
    emit workspaceGuiDocumentChanged(document);
    emit activeGuiDocumentChanged(id, document);
}

void GuiApplication::setActiveWorkspace(ProjectWorkspaceId id)
{
    if (m_activeWorkspaceId == id)
        return;

    m_activeWorkspaceId = id;
    GuiDocument* document = ensureGuiDocument(id);
    emit activeGuiDocumentChanged(id, document);
    emit workspaceGuiDocumentChanged(document);
    if (document)
        emit workspaceGuiDocumentReady();
}

void GuiApplication::applyDisplayModeToDocument(GuiDocument* document) const
{
    if (document && document->renderingManager()) {
        document->renderingManager()->setRuntimeDisplayMode(m_currentDisplayMode,
                                                            m_currentFaceBoundary);
    }
}

void GuiApplication::setCurrentDisplayMode(int displayMode, bool faceBoundary)
{
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "GuiApplication::setCurrentDisplayMode mode={} edges={}",
               displayMode, faceBoundary);
    m_currentDisplayMode = displayMode;
    m_currentFaceBoundary = faceBoundary;

    for (GuiDocument* workspace : std::as_const(m_guiDocuments)) {
        applyDisplayModeToDocument(workspace);
        GraphicsScene* scene = workspace ? workspace->scene() : nullptr;
        if (!scene)
            continue;

        const Handle(AIS_InteractiveContext)& ctx = scene->context();
        if (ctx.IsNull())
            continue;

        ctx->DefaultDrawer()->SetFaceBoundaryDraw(faceBoundary);
        ctx->SetDisplayMode(displayMode, Standard_False);

        AIS_ListOfInteractive list;
        ctx->DisplayedObjects(list);
        for (AIS_ListIteratorOfListOfInteractive it(list); it.More(); it.Next()) {
            const Handle(AIS_InteractiveObject)& obj = it.Value();
            if (Handle(AIS_Shape) shape = Handle(AIS_Shape)::DownCast(obj); !shape.IsNull()) {
                ctx->SetDisplayMode(shape, displayMode, Standard_False);
                if (!shape->Attributes().IsNull())
                    shape->Attributes()->SetFaceBoundaryDraw(faceBoundary);
                ctx->Redisplay(shape, Standard_False);
            }
        }
        ctx->UpdateCurrentViewer();
    }
}

void GuiApplication::setMachineCoordinateFrame(const QList<MachineAxisDef>& axes)
{
    m_machineCoordinateAxes = axes;
    for (GuiDocument* document : m_guiDocuments) {
        if (document)
            document->setMachineCoordinateFrame(axes);
    }
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
    if (!applyCadViews && !applyCamView)
        return;

    const bool preferCamProfile = applyCamView;
    for (GuiDocument* workspace : std::as_const(m_guiDocuments)) {
        if (!workspace || !workspace->renderingManager())
            continue;
        workspace->renderingManager()->setMachineView(preferCamProfile);
        workspace->renderingManager()->configure(preferCamProfile ? camProfile : cadProfile, colors);
        workspace->renderingManager()->requestApply(dirtyFlags);
    }
}
