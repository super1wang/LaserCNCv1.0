#include "core/project/lcnc_project_manager.h"

#include "core/document/lcnc_document.h"
#include "core/project/cam/cam_data_manager.h"
#include "core/project/cam/cam_toolpath_io.h"
#include "core/project/lcnc_project_package.h"
#include "core/logging/logger.h"

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

    m_camData = std::make_unique<lcnc::cam::CamDataManager>();

    ensureProject();
    m_session.clearDirty();
}

LcncProjectManager::~LcncProjectManager() = default;

void LcncProjectManager::ensureProject()
{
    if (!m_workpieceDocument)
        m_workpieceDocument.reset(createDomainDocument(ProjectDomain::Workpiece, QStringLiteral("LaserCNC 项目")));
    // 统一工程文档：工件原始模型(EntityKind::Workpiece) 与 CAM 轮廓几何(EntityKind::Cam)
    // 同存于 m_workpieceDocument，不再有独立 CAM doc。ProjectDomain::Cam 保留为渲染域。
    // 机台 doc 由 CAM 的 MachineWorkspace 拥有（独立参考资产），仅通过 attachMachineDocument
    // 登记一个非拥有引用供视图域路由使用；本类不创建/拥有机台几何，也不持久化它。

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
    // 机台不进 .lcnc（参考资产，独立管理）→ 传 nullptr。
    // CAM 数据随包内一并读取（在解压目录销毁前），与几何同一事务。
    if (!LcncProjectPackage::load(*workpieceDocument(), nullptr, camDocument(),
                                  packagePath, &result, errorMsg, m_camData.get())) {
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

    // CAM 数据已在 LcncProjectPackage::load 内部随包读出到 m_camData（包内、解压目录销毁前）。
    // 视图刷新由 CAM 模块订阅 projectOpened 完成（onCamDataLoaded → 重连 wire + 显示）。
    emit projectOpened(m_session.projectPath());
    emit projectDirtyChanged(false);
    emit domainDataChanged(ProjectDomain::Workpiece);
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
    // 机台不进 .lcnc（参考资产，独立管理）→ 传 nullptr。
    // CAM 数据随包内一并写出（在打包前写入 staging 目录），与几何同一事务。
    const bool ok = LcncProjectPackage::save(*workpieceDocument(), nullptr, camDocument(),
                                             targetPath, manifest, options, &savedManifest, errorMsg,
                                             m_camData.get());
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
        // 机台是参考资产（CAM MachineWorkspace 独立管理），不属于工程数据。
        target->clearEntityKind(LcncDocument::EntityKind::Machine);
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
        // Phase D：CAM 注入的工作台 doc 优先；未注入时回落 manager 内 owned doc。
        return m_machineBorrowed ? m_machineBorrowed : m_machineDocument.get();
    case ProjectDomain::Cam:
        // 统一文档：CAM 轮廓几何与工件同存于工程文档。
        return m_workpieceDocument.get();
    case ProjectDomain::Project:
        return nullptr;
    }
    return nullptr;
}

LcncDocument* LcncProjectManager::domainDocumentById(DocumentId documentId) const
{
    if (m_workpieceDocument && m_workpieceDocument->id() == documentId)
        return m_workpieceDocument.get();
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
    if (m_machineBorrowed && m_machineBorrowed->id() == documentId) {
        *domain = ProjectDomain::Machine;
        return true;
    }
    if (m_machineDocument && m_machineDocument->id() == documentId) {
        *domain = ProjectDomain::Machine;
        return true;
    }
    // 统一文档：CAM 与工件共享同一 docId，按 docId 解析的"主域"归工件；
    // CAM 轮廓的域在 DisplayObject 级单独标注（ProjectDomain::Cam）。
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
    // 统一工程文档：clearProjectData 会清掉工件 + CAM 轮廓(EntityKind::Cam)；机台是
    // 独立参考资产，跨工程保留，不在此清空。
    workpieceDocument()->clearProjectData();
    workpieceDocument()->setName(projectName);
    workpieceDocument()->setFilePath(QString());
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
