#include "view/gui_application.h"

#include "core/kernel/kernel.h"
#include "core/logging/logger.h"
#include "core/project/lcnc_project_manager.h"
#include "view/graphics_scene.h"
#include "view/gui_document.h"
#include "view/rendering_manager.h"

#include <AIS_InteractiveContext.hxx>
#include <Prs3d_Drawer.hxx>

#include <utility>

GuiApplication* GuiApplication::s_instance = nullptr;

GuiApplication::GuiApplication(QObject* parent)
    : QObject(parent)
{
    Q_ASSERT_X(!s_instance, "GuiApplication",
               "second GuiApplication instance — must be Kernel-owned only");
    s_instance = this;

    if (const auto* settings = lcnc::Kernel::current().appSettings()) {
        const lcnc::StartupDisplayMode defaultMode = settings->camViewRendering.defaultDisplayMode;
        m_currentDisplayMode = defaultMode == lcnc::StartupDisplayMode::Wireframe
            ? AIS_WireFrame : AIS_Shaded;
        m_currentFaceBoundary = defaultMode == lcnc::StartupDisplayMode::ShadedWithEdges;
    }

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
        emit activeGuiDocumentReady();
        emit activeGuiDocumentChanged(document);
        emit activeWorkspaceDocumentChanged(id, document);
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
        emit activeGuiDocumentAboutToClose(document);

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
        emit activeGuiDocumentAboutToClose(oldDocument);
        oldDocument->deleteLater();
    }

    GuiDocument* document = createWorkspaceGuiDocument();
    m_guiDocuments.insert(id, document);
    emit guiDocumentReady(id, document);
    emit activeGuiDocumentReady();
    emit activeGuiDocumentChanged(document);
    emit activeWorkspaceDocumentChanged(id, document);
}

void GuiApplication::setActiveWorkspace(ProjectWorkspaceId id)
{
    if (m_activeWorkspaceId == id)
        return;

    m_activeWorkspaceId = id;
    GuiDocument* document = ensureGuiDocument(id);
    emit activeWorkspaceDocumentChanged(id, document);
    emit activeGuiDocumentChanged(document);
    if (document)
        emit activeGuiDocumentReady();
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
        ctx->SetDisplayMode(displayMode, false);
        // context 默认显示模式 + DefaultDrawer 面边线，供后续新建对象（含未显式
        // 指定模式的 XCAF 工件/机台对象）继承。工件/机台 AIS_Shape 的逐对象模式
        // 切换已由 applyDisplayModeToDocument -> RenderingManager::
        // setRuntimeDisplayMode 按域（Workpiece/Machine）处理，这里不再遍历全部
        // 已显示对象强制覆盖--否则刀路/引导锥/球/gizmo/草绘/CAM 轮廓等会被一并
        // 切到线框，破坏它们各自创建时确定的模式（如红色刀头锥应始终为实体）。
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
