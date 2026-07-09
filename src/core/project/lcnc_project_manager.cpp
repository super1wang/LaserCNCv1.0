#include "core/project/lcnc_project_manager.h"

#include "core/document/lcnc_document.h"
#include "core/logging/logger.h"
#include "core/project/cam/cam_data_manager.h"
#include "core/project/cam/cam_toolpath_io.h"
#include "core/project/lcnc_project_package.h"
#include "core/project/project_workspace.h"

#include <QFileInfo>

#include <BinXCAFDrivers.hxx>
#include <BRep_Builder.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <IGESControl_Reader.hxx>
#include <Poly_Triangulation.hxx>
#include <RWStl.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <TopoDS_Face.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XmlXCAFDrivers.hxx>

namespace lcnc {

namespace {

QString defaultProjectName(const QString& name)
{
    const QString trimmed = name.trimmed();
    return trimmed.isEmpty() ? QStringLiteral("LaserCNC 项目") : trimmed;
}

} // namespace

LcncProjectManager::LcncProjectManager(QObject* parent)
    : QObject(parent)
{
    Handle(XCAFApp_Application) occApp = XCAFApp_Application::GetApplication();
    BinXCAFDrivers::DefineFormat(occApp);
    XmlXCAFDrivers::DefineFormat(occApp);

    ensureProject();
    session().clearDirty();
}

LcncProjectManager::~LcncProjectManager() = default;

LcncProjectSession& LcncProjectManager::session()
{
    ensureProject();
    if (ProjectWorkspace* current = activeWorkspace())
        return current->session();
    return fallbackSession();
}

const LcncProjectSession& LcncProjectManager::session() const
{
    if (ProjectWorkspace* current = activeWorkspace())
        return current->session();
    return fallbackSession();
}

void LcncProjectManager::ensureProject()
{
    if (activeWorkspace())
        return;

    const ProjectWorkspaceId id = m_nextWorkspaceId++;
    auto workspace = makeWorkspace(id, QStringLiteral("LaserCNC 项目"));
    m_workspaces.emplace(id, workspace);
    m_activeWorkspaceId = id;
    emit workspaceAdded(id);
    emit activeWorkspaceChanged(id);
}

ProjectWorkspace* LcncProjectManager::activeWorkspace() const
{
    return workspace(m_activeWorkspaceId);
}

ProjectWorkspace* LcncProjectManager::workspace(ProjectWorkspaceId id) const
{
    const auto it = m_workspaces.find(id);
    return it == m_workspaces.end() ? nullptr : it->second.get();
}

QList<ProjectWorkspaceId> LcncProjectManager::workspaceIds() const
{
    QList<ProjectWorkspaceId> ids;
    ids.reserve(static_cast<int>(m_workspaces.size()));
    for (const auto& [id, workspace] : m_workspaces) {
        Q_UNUSED(workspace)
        ids.append(id);
    }
    return ids;
}

std::shared_ptr<ProjectWorkspace> LcncProjectManager::createDetachedWorkspace(const QString& name)
{
    const ProjectWorkspaceId id = m_nextWorkspaceId++;
    return makeWorkspace(id, defaultProjectName(name));
}

ProjectWorkspaceId LcncProjectManager::adoptWorkspace(
    const std::shared_ptr<ProjectWorkspace>& workspace,
    bool emitProjectOpened)
{
    if (!workspace || !workspace->workpieceDocument())
        return kInvalidProjectWorkspaceId;

    workspace->bindMachineDocument(machineDocument());
    const ProjectWorkspaceId id = workspace->id();
    m_workspaces[id] = workspace;

    emit workspaceAdded(id);
    setActiveWorkspace(id);

    emit projectDirtyChanged(workspace->session().isDirty());
    if (emitProjectOpened)
        emit projectOpened(workspace->session().projectPath());

    return id;
}

bool LcncProjectManager::closeWorkspace(ProjectWorkspaceId id)
{
    const auto it = m_workspaces.find(id);
    if (it == m_workspaces.end())
        return false;

    const bool wasActive = (m_activeWorkspaceId == id);
    emit workspaceAboutToClose(id);
    m_workspaces.erase(it);
    emit workspaceClosed(id);

    if (!wasActive)
        return true;

    ProjectWorkspaceId nextId = kInvalidProjectWorkspaceId;
    if (!m_workspaces.empty())
        nextId = m_workspaces.rbegin()->first;

    m_activeWorkspaceId = kInvalidProjectWorkspaceId;
    setActiveWorkspace(nextId);
    if (nextId == kInvalidProjectWorkspaceId) {
        emit activeWorkspaceChanged(kInvalidProjectWorkspaceId);
        emit projectReset();
        emit projectDirtyChanged(false);
    }
    return true;
}

void LcncProjectManager::closeAllWorkspaces()
{
    QList<ProjectWorkspaceId> ids = workspaceIds();
    for (ProjectWorkspaceId id : ids)
        closeWorkspace(id);
}

void LcncProjectManager::setActiveWorkspace(ProjectWorkspaceId id)
{
    if (id != kInvalidProjectWorkspaceId && !workspace(id))
        return;
    if (m_activeWorkspaceId == id)
        return;

    m_activeWorkspaceId = id;
    emit activeWorkspaceChanged(id);

    if (ProjectWorkspace* current = activeWorkspace()) {
        current->bindMachineDocument(machineDocument());
        emit projectDirtyChanged(current->session().isDirty());
    } else {
        emit projectDirtyChanged(false);
    }
}

std::uint64_t LcncProjectManager::beginSingleDocumentOpen()
{
    const std::uint64_t generation = ++m_singleDocumentOpenGeneration;
    if (m_documentOpenMode == DocumentOpenMode::SingleDocument)
        closeAllWorkspaces();
    return generation;
}

bool LcncProjectManager::isSingleDocumentOpenCurrent(std::uint64_t generation) const
{
    return m_documentOpenMode != DocumentOpenMode::SingleDocument
           || generation == m_singleDocumentOpenGeneration;
}

LcncDocument* LcncProjectManager::newProject(const QString& name)
{
    if (m_documentOpenMode == DocumentOpenMode::SingleDocument)
        closeAllWorkspaces();

    auto workspace = createDetachedWorkspace(defaultProjectName(name));
    resetWorkspace(workspace.get(), defaultProjectName(name));
    const ProjectWorkspaceId id = adoptWorkspace(workspace);
    if (id == kInvalidProjectWorkspaceId)
        return nullptr;

    emit projectDirtyChanged(false);
    emit domainDataChanged(ProjectDomain::Workpiece);
    emit domainDataChanged(ProjectDomain::Cam);
    return workspace->workpieceDocument();
}

LcncDocument* LcncProjectManager::openProject(const QString& filePath, QString* errorMsg)
{
    if (!LcncProjectPackage::isProjectPath(filePath)) {
        if (errorMsg)
            *errorMsg = QStringLiteral("不是 LaserCNC 项目文件: %1").arg(filePath);
        return nullptr;
    }

    if (m_documentOpenMode == DocumentOpenMode::SingleDocument)
        closeAllWorkspaces();

    const QString packagePath = LcncProjectPackage::packageDirectory(filePath);
    auto workspace = createDetachedWorkspace(QFileInfo(packagePath).completeBaseName());
    ProjectLoadResult result;
    if (!LcncProjectPackage::load(*workspace->workpieceDocument(), nullptr, workspace->camDocument(),
                                  packagePath, &result, errorMsg, workspace->camData())) {
        return nullptr;
    }

    const QString projectName = result.documentName.isEmpty()
        ? QFileInfo(packagePath).completeBaseName()
        : result.documentName;
    workspace->workpieceDocument()->setName(projectName);
    workspace->workpieceDocument()->setFilePath(result.packagePath.isEmpty() ? packagePath : result.packagePath);
    workspace->session().setProjectName(projectName);
    workspace->session().setProjectPath(workspace->workpieceDocument()->filePath());
    workspace->session().setManifest(result.manifest);
    workspace->session().workpiece().displayName = projectName;
    workspace->session().workpiece().sourceFilePath = result.manifest.sourceFilePath;
    workspace->syncSessionFromDocuments();
    workspace->session().clearDirty();

    adoptWorkspace(workspace, true);
    emit domainDataChanged(ProjectDomain::Workpiece);
    emit domainDataChanged(ProjectDomain::Cam);
    return workspace->workpieceDocument();
}

bool LcncProjectManager::saveProject(const QString& filePath, QString* errorMsg)
{
    ensureProject();
    ProjectWorkspace* current = activeWorkspace();
    if (!current) {
        if (errorMsg)
            *errorMsg = QStringLiteral("没有活动工程，无法保存");
        return false;
    }

    LcncProjectSession& currentSession = current->session();
    const QString targetPath = filePath.isEmpty() ? currentSession.projectPath() : filePath;
    if (targetPath.isEmpty()) {
        if (errorMsg)
            *errorMsg = QStringLiteral("项目路径为空，无法保存");
        return false;
    }

    ProjectSaveOptions options = currentSession.saveOptions();
    LcncProjectManifest manifest = currentSession.manifest();
    manifest.projectName = currentSession.projectName().trimmed().isEmpty()
        ? current->workpieceDocument()->name()
        : currentSession.projectName().trimmed();
    manifest.documentName = manifest.projectName;
    manifest.sourceFilePath = currentSession.workpiece().sourceFilePath;
    manifest.saveOptions = options;

    LcncProjectManifest savedManifest;
    const bool ok = LcncProjectPackage::save(*current->workpieceDocument(), nullptr, current->camDocument(),
                                             targetPath, manifest, options, &savedManifest, errorMsg,
                                             current->camData());
    if (!ok)
        return false;

    const QString packagePath = LcncProjectPackage::packageDirectory(targetPath);
    current->workpieceDocument()->setFilePath(packagePath);
    currentSession.setProjectPath(packagePath);
    currentSession.setManifest(savedManifest);
    currentSession.clearDirty();

    emit projectSaved(currentSession.projectPath());
    emit projectDirtyChanged(false);
    return true;
}

LcncDocument* LcncProjectManager::importWorkpieceModel(const QString& filePath, QString* errorMsg)
{
    if (LcncProjectPackage::isProjectPath(filePath))
        return openProject(filePath, errorMsg);

    QFileInfo fileInfo(filePath);
    if (m_documentOpenMode == DocumentOpenMode::SingleDocument)
        closeAllWorkspaces();

    auto workspace = createDetachedWorkspace(fileInfo.completeBaseName());
    LcncDocument* target = workspace->workpieceDocument();
    if (!target)
        return nullptr;

    if (!importGeometryFile(target, filePath, errorMsg)) {
        target->clearEntityKind(LcncDocument::EntityKind::Workpiece);
        return nullptr;
    }

    workspace->session().workpiece().displayName = fileInfo.completeBaseName();
    workspace->session().workpiece().sourceFilePath = fileInfo.absoluteFilePath();
    workspace->syncSessionFromDocuments();
    adoptWorkspace(workspace);
    notifyDomainChanged(ProjectDomain::Workpiece);
    return target;
}

bool LcncProjectManager::exportDomainAsStep(ProjectDomain domain, const QString& filePath, QString* errorMsg)
{
    LcncDocument* target = document(domain);
    if (!target) {
        if (errorMsg)
            *errorMsg = QStringLiteral("项目数据域不存在");
        return false;
    }

    STEPControl_Writer writer;
    TDF_LabelSequence labels;
    target->shapeTool()->GetFreeShapes(labels);
    Handle(XCAFDoc_ShapeTool) shapeTool = target->shapeTool();
    for (int index = 1; index <= labels.Length(); ++index) {
        TopoDS_Shape shape = shapeTool->GetShape(labels.Value(index));
        if (!shape.IsNull())
            writer.Transfer(shape, STEPControl_AsIs);
    }

    const bool ok = writer.Write(filePath.toUtf8().constData()) == IFSelect_RetDone;
    if (!ok && errorMsg)
        *errorMsg = QStringLiteral("保存失败: %1").arg(filePath);
    return ok;
}

void LcncProjectManager::clearDomain(ProjectDomain domain)
{
    ensureProject();
    ProjectWorkspace* current = activeWorkspace();
    LcncDocument* target = document(domain);
    if (!current || !target)
        return;

    switch (domain) {
    case ProjectDomain::Workpiece:
        target->clearEntityKind(LcncDocument::EntityKind::Workpiece);
        current->session().workpiece().clear();
        break;
    case ProjectDomain::Machine:
        target->clearEntityKind(LcncDocument::EntityKind::Machine);
        break;
    case ProjectDomain::Cam:
        target->clearEntityKind(LcncDocument::EntityKind::Cam);
        if (current->camData())
            current->camData()->clearToolpath();
        current->session().cam().clear();
        break;
    case ProjectDomain::Project:
        return;
    }

    notifyDomainChanged(domain);
}

void LcncProjectManager::notifyDomainChanged(ProjectDomain domain)
{
    ProjectWorkspace* current = activeWorkspace();
    if (current)
        current->syncSessionFromDocuments();
    markDomainDirty(domain);
    emit workspaceDomainChanged(m_activeWorkspaceId, domain);
    emit domainDataChanged(domain);
}

void LcncProjectManager::notifyDomainChanged(DocumentId documentId)
{
    ProjectDomain domain = ProjectDomain::Project;
    if (domainForDocument(documentId, &domain))
        notifyDomainChanged(domain);
}

LcncDocument* LcncProjectManager::document(ProjectDomain domain) const
{
    ProjectWorkspace* current = activeWorkspace();
    switch (domain) {
    case ProjectDomain::Workpiece:
        return current ? current->workpieceDocument() : nullptr;
    case ProjectDomain::Machine:
        return m_machineBorrowed ? m_machineBorrowed : m_machineDocument.get();
    case ProjectDomain::Cam:
        return current ? current->camDocument() : nullptr;
    case ProjectDomain::Project:
        return nullptr;
    }
    return nullptr;
}

LcncDocument* LcncProjectManager::domainDocumentById(DocumentId documentId) const
{
    for (const auto& [id, workspace] : m_workspaces) {
        Q_UNUSED(id)
        if (workspace && workspace->workpieceDocument()
            && workspace->workpieceDocument()->id() == documentId) {
            return workspace->workpieceDocument();
        }
    }
    if (m_machineBorrowed && m_machineBorrowed->id() == documentId)
        return m_machineBorrowed;
    if (m_machineDocument && m_machineDocument->id() == documentId)
        return m_machineDocument.get();
    return nullptr;
}

DocumentId LcncProjectManager::documentId(ProjectDomain domain) const
{
    LcncDocument* target = document(domain);
    return target ? target->id() : kInvalidDocumentId;
}

LcncDocument* LcncProjectManager::workpieceDocument() const
{
    return document(ProjectDomain::Workpiece);
}

LcncDocument* LcncProjectManager::machineDocument() const
{
    return document(ProjectDomain::Machine);
}

LcncDocument* LcncProjectManager::camDocument() const
{
    return document(ProjectDomain::Cam);
}

lcnc::cam::CamDataManager* LcncProjectManager::camData() const
{
    ProjectWorkspace* current = activeWorkspace();
    return current ? current->camData() : nullptr;
}

DocumentId LcncProjectManager::workpieceDocumentId() const
{
    return documentId(ProjectDomain::Workpiece);
}

DocumentId LcncProjectManager::machineDocumentId() const
{
    return documentId(ProjectDomain::Machine);
}

DocumentId LcncProjectManager::camDocumentId() const
{
    return documentId(ProjectDomain::Cam);
}

bool LcncProjectManager::domainForDocument(DocumentId documentId, ProjectDomain* domain) const
{
    if (!domain)
        return false;
    for (const auto& [id, workspace] : m_workspaces) {
        Q_UNUSED(id)
        if (workspace && workspace->workpieceDocument()
            && workspace->workpieceDocument()->id() == documentId) {
            *domain = ProjectDomain::Workpiece;
            return true;
        }
    }
    if (m_machineBorrowed && m_machineBorrowed->id() == documentId) {
        *domain = ProjectDomain::Machine;
        return true;
    }
    if (m_machineDocument && m_machineDocument->id() == documentId) {
        *domain = ProjectDomain::Machine;
        return true;
    }
    return false;
}

int LcncProjectManager::reserveDocumentId()
{
    return m_nextDocumentId++;
}

LcncDocument* LcncProjectManager::createMachineDocument()
{
    return createDomainDocument(ProjectDomain::Machine, QStringLiteral("机台工作台"));
}

void LcncProjectManager::attachMachineDocument(LcncDocument* borrowed)
{
    m_machineBorrowed = borrowed;
    bindMachineDocumentToWorkspaces();
    emit domainDataChanged(ProjectDomain::Machine);
}

bool LcncProjectManager::isDomainDocument(DocumentId documentId, ProjectDomain domain) const
{
    ProjectDomain resolved = ProjectDomain::Project;
    return domainForDocument(documentId, &resolved) && resolved == domain;
}

LcncDocument* LcncProjectManager::createDomainDocument(ProjectDomain domain, const QString& name)
{
    Q_UNUSED(domain)
    return new LcncDocument(m_nextDocumentId++, name);
}

void LcncProjectManager::resetWorkspace(ProjectWorkspace* workspace, const QString& projectName)
{
    if (!workspace)
        return;
    workspace->resetProjectState(projectName);
    workspace->bindMachineDocument(machineDocument());
}

bool LcncProjectManager::importGeometryFile(LcncDocument* document, const QString& filePath, QString* errorMsg)
{
    QFileInfo fileInfo(filePath);
    if (!document || filePath.isEmpty() || !fileInfo.exists()) {
        if (errorMsg)
            *errorMsg = QStringLiteral("文件不存在: %1").arg(filePath);
        return false;
    }

    const QString ext = fileInfo.suffix().toLower();
    bool ok = false;
    if (ext == "stp" || ext == "step") {
        STEPControl_Reader reader;
        if (reader.ReadFile(filePath.toUtf8().constData()) == IFSelect_RetDone) {
            reader.TransferRoots();
            for (int index = 1; index <= reader.NbShapes(); ++index) {
                TopoDS_Shape shape = reader.Shape(index);
                if (!shape.IsNull())
                    document->addShapeEntity(shape, QStringLiteral("Shape_%1").arg(index));
            }
            ok = true;
        }
    } else if (ext == "igs" || ext == "iges") {
        IGESControl_Reader reader;
        if (reader.ReadFile(filePath.toUtf8().constData()) == IFSelect_RetDone) {
            reader.TransferRoots();
            for (int index = 1; index <= reader.NbShapes(); ++index) {
                TopoDS_Shape shape = reader.Shape(index);
                if (!shape.IsNull())
                    document->addShapeEntity(shape, QStringLiteral("Shape_%1").arg(index));
            }
            ok = true;
        }
    } else if (ext == "stl") {
        Handle(Poly_Triangulation) mesh = RWStl::ReadFile(filePath.toUtf8().constData());
        if (!mesh.IsNull()) {
            BRep_Builder builder;
            TopoDS_Face face;
            builder.MakeFace(face);
            builder.UpdateFace(face, mesh);
            document->addShapeEntity(face, fileInfo.completeBaseName());
            ok = true;
        }
    } else {
        if (errorMsg)
            *errorMsg = QStringLiteral("暂不支持的文件格式: %1").arg(fileInfo.suffix());
        return false;
    }

    if (!ok && errorMsg)
        *errorMsg = QStringLiteral("无法读取文件: %1").arg(filePath);
    return ok;
}

void LcncProjectManager::syncWorkspaceSession(ProjectWorkspace* workspace) const
{
    if (workspace)
        workspace->syncSessionFromDocuments();
}

void LcncProjectManager::markDomainDirty(ProjectDomain domain)
{
    ProjectWorkspace* current = activeWorkspace();
    if (!current)
        return;
    const bool wasDirty = current->session().isDirty();
    current->session().markDirty(domain);
    if (current->session().isDirty() != wasDirty)
        emit projectDirtyChanged(current->session().isDirty());
}

LcncProjectSession& LcncProjectManager::fallbackSession()
{
    return m_emptySession;
}

const LcncProjectSession& LcncProjectManager::fallbackSession() const
{
    return m_emptySession;
}

std::shared_ptr<ProjectWorkspace> LcncProjectManager::makeWorkspace(ProjectWorkspaceId id, const QString& name)
{
    auto document = std::unique_ptr<LcncDocument>(
        createDomainDocument(ProjectDomain::Workpiece, defaultProjectName(name)));
    auto camData = std::make_unique<lcnc::cam::CamDataManager>();
    auto workspace = std::make_shared<ProjectWorkspace>(id, std::move(document), std::move(camData));
    workspace->bindMachineDocument(machineDocument());
    workspace->resetProjectState(defaultProjectName(name));
    return workspace;
}

void LcncProjectManager::bindMachineDocumentToWorkspaces()
{
    LcncDocument* machine = machineDocument();
    for (auto& [id, workspace] : m_workspaces) {
        Q_UNUSED(id)
        if (workspace)
            workspace->bindMachineDocument(machine);
    }
}

} // namespace lcnc
