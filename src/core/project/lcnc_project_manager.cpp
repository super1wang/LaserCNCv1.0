#include "core/project/lcnc_project_manager.h"

#include "core/document/lcnc_document.h"
#include "core/project/lcnc_project_package.h"

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
    m_session.clearDirty();
}

void LcncProjectManager::ensureProject()
{
    if (!m_workpieceDocument)
        m_workpieceDocument.reset(createDomainDocument(ProjectDomain::Workpiece, QStringLiteral("LaserCNC 项目")));
    if (!m_machineDocument)
        m_machineDocument.reset(createDomainDocument(ProjectDomain::Machine, QStringLiteral("机台文档")));
    if (!m_camDocument)
        m_camDocument.reset(createDomainDocument(ProjectDomain::Cam, QStringLiteral("CAM 数据")));

    syncSessionFromDocuments();
}

LcncDocument* LcncProjectManager::newProject(const QString& name)
{
    const QString projectName = defaultProjectName(name);
    ensureProject();
    resetProjectDocuments(projectName);
    m_session.resetProjectState();
    m_session.setProjectName(projectName);
    m_session.setProjectPath(QString());
    m_session.clearDirty();

    emit projectReset();
    emit projectDirtyChanged(false);
    emit domainDataChanged(ProjectDomain::Workpiece);
    emit domainDataChanged(ProjectDomain::Machine);
    emit domainDataChanged(ProjectDomain::Cam);
    return workpieceDocument();
}

LcncDocument* LcncProjectManager::openProject(const QString& filePath, QString* errorMsg)
{
    if (!LcncProjectPackage::isProjectPath(filePath)) {
        if (errorMsg)
            *errorMsg = QStringLiteral("不是 LaserCNC 项目文件: %1").arg(filePath);
        return nullptr;
    }

    const QString packagePath = LcncProjectPackage::packageDirectory(filePath);
    ensureProject();
    resetProjectDocuments(QFileInfo(packagePath).completeBaseName());
    m_session.resetProjectState();

    ProjectLoadResult result;
    if (!LcncProjectPackage::load(*workpieceDocument(), machineDocument(), camDocument(),
                                  packagePath, &result, errorMsg)) {
        return nullptr;
    }

    const QString projectName = result.documentName.isEmpty()
        ? QFileInfo(packagePath).completeBaseName()
        : result.documentName;
    workpieceDocument()->setName(projectName);
    workpieceDocument()->setFilePath(result.packagePath.isEmpty() ? packagePath : result.packagePath);
    m_session.setProjectName(projectName);
    m_session.setProjectPath(workpieceDocument()->filePath());
    m_session.setManifest(result.manifest);
    m_session.workpiece().displayName = projectName;
    m_session.workpiece().sourceFilePath = result.manifest.sourceFilePath;
    m_session.clearDirty();
    syncSessionFromDocuments();

    emit projectOpened(m_session.projectPath());
    emit projectDirtyChanged(false);
    emit domainDataChanged(ProjectDomain::Workpiece);
    emit domainDataChanged(ProjectDomain::Machine);
    emit domainDataChanged(ProjectDomain::Cam);
    return workpieceDocument();
}

bool LcncProjectManager::saveProject(const QString& filePath, QString* errorMsg)
{
    ensureProject();
    const QString targetPath = filePath.isEmpty() ? m_session.projectPath() : filePath;
    if (targetPath.isEmpty()) {
        if (errorMsg)
            *errorMsg = QStringLiteral("项目路径为空，无法保存");
        return false;
    }

    ProjectSaveOptions options = m_session.saveOptions();
    LcncProjectManifest manifest = m_session.manifest();
    manifest.projectName = m_session.projectName().trimmed().isEmpty()
        ? workpieceDocument()->name()
        : m_session.projectName().trimmed();
    manifest.documentName = manifest.projectName;
    manifest.sourceFilePath = m_session.workpiece().sourceFilePath;
    manifest.saveOptions = options;

    LcncProjectManifest savedManifest;
    const bool ok = LcncProjectPackage::save(*workpieceDocument(), machineDocument(), camDocument(),
                                             targetPath, manifest, options, &savedManifest, errorMsg);
    if (!ok)
        return false;

    const QString packagePath = LcncProjectPackage::packageDirectory(targetPath);
    workpieceDocument()->setFilePath(packagePath);
    m_session.setProjectPath(packagePath);
    m_session.setManifest(savedManifest);
    m_session.clearDirty();
    emit projectSaved(m_session.projectPath());
    emit projectDirtyChanged(false);
    return true;
}

LcncDocument* LcncProjectManager::importWorkpieceModel(const QString& filePath, QString* errorMsg)
{
    if (LcncProjectPackage::isProjectPath(filePath))
        return openProject(filePath, errorMsg);

    ensureProject();
    LcncDocument* target = workpieceDocument();
    if (!target)
        return nullptr;

    target->clearEntityKind(LcncDocument::EntityKind::Workpiece);
    m_session.workpiece().clear();
    if (!importGeometryFile(target, filePath, errorMsg)) {
        target->clearEntityKind(LcncDocument::EntityKind::Workpiece);
        return nullptr;
    }

    QFileInfo fileInfo(filePath);
    m_session.workpiece().displayName = fileInfo.completeBaseName();
    m_session.workpiece().sourceFilePath = fileInfo.absoluteFilePath();
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
    LcncDocument* target = document(domain);
    if (!target)
        return;

    switch (domain) {
    case ProjectDomain::Workpiece:
        target->clearEntityKind(LcncDocument::EntityKind::Workpiece);
        m_session.workpiece().clear();
        break;
    case ProjectDomain::Machine:
        target->clearEntityKind(LcncDocument::EntityKind::Machine);
        m_session.machine().clear();
        break;
    case ProjectDomain::Cam:
        target->clearEntityKind(LcncDocument::EntityKind::Cam);
        m_session.cam().clear();
        break;
    case ProjectDomain::Project:
        return;
    }

    notifyDomainChanged(domain);
}

void LcncProjectManager::notifyDomainChanged(ProjectDomain domain)
{
    syncSessionFromDocuments();
    markDomainDirty(domain);
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
    switch (domain) {
    case ProjectDomain::Workpiece:
        return m_workpieceDocument.get();
    case ProjectDomain::Machine:
        return m_machineDocument.get();
    case ProjectDomain::Cam:
        return m_camDocument.get();
    case ProjectDomain::Project:
        return nullptr;
    }
    return nullptr;
}

LcncDocument* LcncProjectManager::domainDocumentById(DocumentId documentId) const
{
    if (m_workpieceDocument && m_workpieceDocument->id() == documentId)
        return m_workpieceDocument.get();
    if (m_machineDocument && m_machineDocument->id() == documentId)
        return m_machineDocument.get();
    if (m_camDocument && m_camDocument->id() == documentId)
        return m_camDocument.get();
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
    if (m_workpieceDocument && m_workpieceDocument->id() == documentId) {
        *domain = ProjectDomain::Workpiece;
        return true;
    }
    if (m_machineDocument && m_machineDocument->id() == documentId) {
        *domain = ProjectDomain::Machine;
        return true;
    }
    if (m_camDocument && m_camDocument->id() == documentId) {
        *domain = ProjectDomain::Cam;
        return true;
    }
    return false;
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

void LcncProjectManager::resetProjectDocuments(const QString& projectName)
{
    ensureProject();
    workpieceDocument()->clearProjectData();
    machineDocument()->clearProjectData();
    camDocument()->clearProjectData();
    workpieceDocument()->setName(projectName);
    workpieceDocument()->setFilePath(QString());
    machineDocument()->setName(QStringLiteral("机台文档"));
    machineDocument()->setFilePath(QString());
    camDocument()->setName(QStringLiteral("CAM 数据"));
    camDocument()->setFilePath(QString());
    syncSessionFromDocuments();
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

void LcncProjectManager::syncSessionFromDocuments()
{
    m_session.bindDomainDocuments(workpieceDocument(), machineDocument(), camDocument());
    if (LcncDocument* workpiece = workpieceDocument()) {
        m_session.setProjectName(workpiece->name());
        m_session.setProjectPath(workpiece->filePath());
    }
}

void LcncProjectManager::markDomainDirty(ProjectDomain domain)
{
    const bool wasDirty = m_session.isDirty();
    m_session.markDirty(domain);
    if (m_session.isDirty() != wasDirty)
        emit projectDirtyChanged(m_session.isDirty());
}

} // namespace lcnc
