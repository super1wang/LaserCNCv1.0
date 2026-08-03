#include "modules/cad/cad_module.h"
#include "core/kernel/kernel.h"
#include "modules/cad/document/cad_document_registry.h"
#include "modules/cad/selection/cad_selection_resolver.h"
#include "modules/cad/services/cad_modeling_session.h"
#include "modules/cad/services/cad_document_io_service.h"
#include "modules/cad/services/cad_algorithm_boundary.h"
#include "modules/cad/services/shape_service.h"
#include "modules/cad/task/cad_command_dispatcher.h"
#include "modules/cad/task/cad_command_request.h"

#include "core/algorithms/cad/primitives.h"
#include "core/algorithms/cad/transform_ops.h"
#include "core/document/lcnc_document.h"
#include "core/kernel/i_kernel.h"
#include "core/kernel/service_registry.h"
#include "core/logging/logger.h"
#include "core/project/lcnc_project_manager.h"
#include "core/project/lcnc_project_package.h"
#include "core/project/project_workspace.h"
#include "core/task/task_manager.h"
#include "core/document/xcaf_utils.h"
#include "view/gui_application.h"
#include "view/gui_document.h"

#include <QElapsedTimer>
#include <QFileInfo>

#include <BRep_Builder.hxx>
#include <BRepBndLib.hxx>
#include <BRepTools.hxx>
#include <Bnd_Box.hxx>
#include <StlAPI_Reader.hxx>
#include <TDF_LabelSequence.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS.hxx>
#include <TopExp_Explorer.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <Standard_Failure.hxx>

#include <algorithm>
#include <memory>
#include <stdexcept>

namespace {

QString activeSketchElementKey(int elementId)
{
    return QStringLiteral("__sketch_active_element_%1__").arg(elementId);
}

QString activeSketchHandleKey(int elementId, int handleIndex)
{
    return QStringLiteral("__sketch_active_handle_%1_%2__").arg(elementId).arg(handleIndex);
}

QString finishedSketchElementKey(int sketchId, int elementId)
{
    return QStringLiteral("__sketch_finished_element_%1_%2__").arg(sketchId).arg(elementId);
}

bool selectionContainsSketchElement(
    const lcnc::cad::selection::CadSelectionContext& context,
    int sketchId,
    int elementId)
{
    for (const auto& item : context.items) {
        if (item.domain != lcnc::cad::selection::CadSelectionDomain::SketchElement)
            continue;
        if (item.sketchElementId != elementId)
            continue;
        if (item.sketchId == sketchId || (sketchId == 0 && item.sketchId == 0))
            return true;
    }
    return false;
}

bool selectionContainsSketch(const lcnc::cad::selection::CadSelectionContext& context,
                             int sketchId)
{
    if (context.selectedSketchId == sketchId && context.hasSelectedSketch)
        return true;
    for (const auto& item : context.items) {
        if (item.domain == lcnc::cad::selection::CadSelectionDomain::Sketch
            && item.sketchId == sketchId) {
            return true;
        }
    }
    return false;
}

struct SketchHandlePoint {
    int handleIndex{0};
    QVector<double> params;
};

QList<SketchHandlePoint> sketchHandlePoints(const lcnc::cad::SketchElement& element)
{
    QList<SketchHandlePoint> handles;
    const QVector<double>& params = element.params;
    auto appendHandle = [&handles](int handleIndex, double x, double y) {
        SketchHandlePoint handle;
        handle.handleIndex = handleIndex;
        handle.params = {x, y};
        handles.append(std::move(handle));
    };

    switch (element.kind) {
    case lcnc::cad::SketchToolKind::Point:
        if (params.size() >= 2)
            appendHandle(0, params[0], params[1]);
        break;
    case lcnc::cad::SketchToolKind::Line:
        if (params.size() >= 4) {
            appendHandle(0, params[0], params[1]);
            appendHandle(1, params[2], params[3]);
        }
        break;
    case lcnc::cad::SketchToolKind::Arc:
        if (params.size() >= 6) {
            appendHandle(0, params[0], params[1]);
            appendHandle(1, params[2], params[3]);
            appendHandle(2, params[4], params[5]);
        }
        break;
    case lcnc::cad::SketchToolKind::Circle:
        if (params.size() >= 3) {
            appendHandle(0, params[0], params[1]);
            appendHandle(1, params[0] + params[2], params[1]);
        }
        break;
    case lcnc::cad::SketchToolKind::Rectangle:
        if (params.size() >= 4) {
            const double centerX = params[0];
            const double centerY = params[1];
            const double halfWidth = params[2] * 0.5;
            const double halfHeight = params[3] * 0.5;
            appendHandle(0, centerX, centerY);
            appendHandle(1, centerX + halfWidth, centerY + halfHeight);
            appendHandle(2, centerX - halfWidth, centerY + halfHeight);
            appendHandle(3, centerX - halfWidth, centerY - halfHeight);
            appendHandle(4, centerX + halfWidth, centerY - halfHeight);
        }
        break;
    case lcnc::cad::SketchToolKind::Polygon:
        if (params.size() >= 3) {
            appendHandle(0, params[0], params[1]);
            appendHandle(1, params[0] + params[2], params[1]);
        }
        break;
    case lcnc::cad::SketchToolKind::None:
    default:
        break;
    }
    return handles;
}

int entityCount(LcncDocument* doc, LcncDocument::EntityKind kind)
{
    return doc ? doc->entityLabels(kind).Length() : 0;
}

QString primitiveName(int primitiveIndex)
{
    switch (primitiveIndex) {
    case 1:
        // 中文翻译：圆柱体
        return QObject::tr("cylinder");
    case 2:
        // 中文翻译：球体
        return QObject::tr("sphere");
    case 3:
        // 中文翻译：圆锥体
        return QObject::tr("cone");
    case 4:
        // 中文翻译：圆环体
        return QObject::tr("torus");
    default:
        // 中文翻译：长方体
        return QObject::tr("cuboid");
    }
}

TopoDS_Shape buildPrimitiveShape(int primitiveIndex,
                                 const CadModule::PrimitiveParameters& params,
                                 QString* errMsg)
{
    return lcnc::cad::invokeCadAlgorithm([&] {
        switch (primitiveIndex) {
        case 1:
            return lcnc::cad_algo::makeCylinder(params.radius1, params.sizeZ);
        case 2:
            return lcnc::cad_algo::makeSphere(params.radius1);
        case 3:
            return lcnc::cad_algo::makeCone(params.radius1, params.radius2, params.sizeZ);
        case 4:
            return lcnc::cad_algo::makeTorus(params.radius1, params.radius2);
        default:
            return lcnc::cad_algo::makeBox(params.sizeX, params.sizeY, params.sizeZ);
        }
    }, errMsg);
}

TDF_Label labelByEntry(LcncDocument* doc, const QString& entry)
{
    if (!doc || entry.isEmpty())
        return {};

    TDF_LabelSequence freeShapes;
    doc->shapeTool()->GetFreeShapes(freeShapes);
    for (int index = 1; index <= freeShapes.Length(); ++index) {
        const TDF_Label label = freeShapes.Value(index);
        if (XcafUtils::entry(label) == entry)
            return label;
    }
    return {};
}

QList<TDF_Label> selectedShapeLabels(LcncDocument* doc, const QStringList& entries)
{
    QList<TDF_Label> labels;
    for (const QString& entry : entries) {
        const TDF_Label label = labelByEntry(doc, entry);
        if (!label.IsNull())
            labels.append(label);
    }
    return labels;
}

bool modelCenterForLabels(LcncDocument* doc,
                          const QList<TDF_Label>& labels,
                          gp_Pnt* outCenter,
                          QString* errMsg)
{
    if (!doc || labels.isEmpty() || !outCenter) {
        if (errMsg)
            // 中文翻译：请先选择要变换的形体
            *errMsg = QObject::tr("Please select the shape you want to transform first");
        return false;
    }

    Bnd_Box box;
    const Handle(XCAFDoc_ShapeTool) shapeTool = doc->shapeTool();
    for (const TDF_Label& label : labels) {
        const TopoDS_Shape shape = shapeTool->GetShape(label);
        if (!shape.IsNull())
            BRepBndLib::Add(shape, box);
    }
    if (box.IsVoid()) {
        if (errMsg)
            // 中文翻译：无法计算选中形体中心
            *errMsg = QObject::tr("Unable to calculate center of selected shape");
        return false;
    }

    Standard_Real xMin = 0.0;
    Standard_Real yMin = 0.0;
    Standard_Real zMin = 0.0;
    Standard_Real xMax = 0.0;
    Standard_Real yMax = 0.0;
    Standard_Real zMax = 0.0;
    box.Get(xMin, yMin, zMin, xMax, yMax, zMax);
    *outCenter = gp_Pnt((xMin + xMax) * 0.5,
                        (yMin + yMax) * 0.5,
                        (zMin + zMax) * 0.5);
    return true;
}

lcnc::cad_algo::TransformParams toAlgorithmTransform(
    const CadModule::TransformParameters& params,
    const gp_Pnt& referencePoint)
{
    lcnc::cad_algo::TransformParams algoParams;
    algoParams.translation = gp_Vec(params.translateX, params.translateY, params.translateZ);
    algoParams.referencePoint = referencePoint;
    algoParams.rotateXDeg = params.rotateX;
    algoParams.rotateYDeg = params.rotateY;
    algoParams.rotateZDeg = params.rotateZ;
    return algoParams;
}

void watchTask(QObject* owner, TaskId taskId, std::function<void(bool)> onFinished)
{
    auto connection = std::make_shared<QMetaObject::Connection>();
    *connection = QObject::connect(
        lcnc::Kernel::current().taskManager(),
        &TaskManager::taskFinished,
        owner,
        [taskId, onFinished = std::move(onFinished), connection](TaskId finishedId, bool success) mutable {
            if (finishedId != taskId)
                return;
            QObject::disconnect(*connection);
            onFinished(success);
        });
}

DocumentId ensureTargetDocument(CadModule* module,
                                DocumentId targetDocId,
                                const QString& defaultName,
                                bool* createdNew)
{
    if (createdNew)
        *createdNew = false;

    if (targetDocId != kInvalidDocumentId &&
        module->domainDocumentById(targetDocId) != nullptr) {
        return targetDocId;
    }

    if (createdNew)
        *createdNew = true;
    return module->newDocument(defaultName);
}

} // namespace


// ── IModule ───────────────────────────────────────────────────────────────────────
lcnc::ModuleInfo CadModule::info() const
{
    return {
        QStringLiteral("cad"),
        // 中文翻译：CAD模块
        QStringLiteral("CAD module"),
        QStringLiteral("1.0.0"),
        {}                       // 无依赖
    };
}

namespace {

class CadProjectExplorerProjectionAdapter final
    : public lcnc::cad::ICadProjectExplorerProjection
{
public:
    explicit CadProjectExplorerProjectionAdapter(CadModule& module) : m_module(module) {}

    LcncDocument* projectExplorerWorkpieceDocument() const override
    { return m_module.projectExplorerWorkpieceDocument(); }
    bool projectExplorerIsSketchEditing() const override
    { return m_module.projectExplorerIsSketchEditing(); }
    QList<lcnc::cad::ProjectExplorerSketchElement> projectExplorerActiveSketchElements() const override
    { return m_module.projectExplorerActiveSketchElements(); }
    QList<lcnc::cad::ProjectExplorerSketch> projectExplorerFinishedSketches(DocumentId id) const override
    { return m_module.projectExplorerFinishedSketches(id); }

private:
    CadModule& m_module;
};

} // namespace

bool CadModule::init(lcnc::IKernel& kernel)
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "CadModule::init begin");

    // 以 "不拥有" 语义注册为服务：所有权仍由 Kernel 的 ModuleRegistry。
    auto svcPtr = std::shared_ptr<CadModule>(this, [](CadModule*) {});
    kernel.services().registerService<CadModule>(svcPtr);
    // 同时以 Phase 7 门面接口注册，供 UI/命令以抽象类型查找。
    auto facadePtr = std::shared_ptr<lcnc::ICadFacade>(svcPtr, static_cast<lcnc::ICadFacade*>(this));
    kernel.services().registerService<lcnc::ICadFacade>(facadePtr);
    auto explorerProjection = std::make_shared<CadProjectExplorerProjectionAdapter>(*this);
    kernel.services().registerService<lcnc::cad::ICadProjectExplorerProjection>(explorerProjection);
    m_documentIoService = std::make_unique<lcnc::cad::CadDocumentIoService>(
        *lcnc::Kernel::current().projectManager(),
        *lcnc::Kernel::current().taskManager(), this);
    auto documentIoService = std::shared_ptr<lcnc::cad::CadDocumentIoService>(
        m_documentIoService.get(), [](lcnc::cad::CadDocumentIoService*) {});
    kernel.services().registerService<lcnc::cad::CadDocumentIoService>(documentIoService);

    // A workspace can also be closed by the project manager (for example when
    // replacing a single-document workspace), bypassing CadModule::closeDocument.
    // Reject that close if borrowed CAD work cannot reach a safe cancellation
    // point; the project manager must retain the owning document in this case.
    lcnc::Kernel::current().projectManager()->setWorkspaceCloseGuard(
        [this](ProjectWorkspaceId) {
            if (cancelOwnedTasks(10000))
                return true;
            LCNC_ERR(lcnc::LogCode::Generic,
                     "CadModule: workspace close deferred because CAD tasks are still running");
            return false;
        });

    m_initialized = true;
    LCNC_INFO(lcnc::LogCode::Generic, "CadModule init done");
    return true;
}

bool CadModule::start()
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "CadModule::start (no-op)");
    return true;
}

void CadModule::stop()
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "CadModule::stop begin");
    if (!m_initialized) {
        return;
    }
    if (!cancelOwnedTasks(10000)) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "CadModule::stop: file task cancellation timed out; retaining task-owned documents");
    }
    if (auto* project = lcnc::Kernel::current().projectManager())
        project->setWorkspaceCloseGuard({});
    m_initialized = false;
    LCNC_INFO(lcnc::LogCode::Generic, "CadModule stop done");
}

bool CadModule::cancelOwnedTasks(int timeoutMs)
{
    auto* taskMgr = lcnc::Kernel::current().taskManager();
    if (!taskMgr || m_taskScope.empty())
        return true;
    return m_taskScope.cancelAndWait(*taskMgr, timeoutMs);
}

CadModule::CadModule(QObject* parent)
    : QObject(parent)
    , m_modelingSession(std::make_unique<lcnc::cad::CadModelingSession>())
    , m_documentRegistry(std::make_unique<lcnc::cad::CadDocumentRegistry>())
    , m_commandDispatcher(std::make_unique<lcnc::cad::task::CadCommandDispatcher>(this))
{
    m_commandDispatcher->registerDefaultTools();
    auto* project = lcnc::Kernel::current().projectManager();
    connect(project, &lcnc::LcncProjectManager::projectReset, this, [this, project]() {
        const DocumentId id = project->workpieceDocumentId();
        m_documentRegistry->ensure(id);
        m_selectedSketchDocId = kInvalidDocumentId;
        m_selectedSketchId = 0;
        if (auto* gd = activeGuiDocument()) {
            gd->rebuildDomain(lcnc::ProjectDomain::Workpiece, project->workpieceDocument());
            if (auto* md = project->machineDocument())
                gd->updateMachineWorkspaceTransforms(md, project->workpieceDocument());
            gd->fitAll();
        }
        emit documentListChanged();
        emit workpieceStructureChanged();
        emit sketchSelectionChanged(0);
    });
    connect(project, &lcnc::LcncProjectManager::activeWorkspaceChanged,
            this, [this, project](ProjectWorkspaceId) {
                const DocumentId id = project->workpieceDocumentId();
                if (id != kInvalidDocumentId)
                    m_documentRegistry->ensure(id);
                m_selectedSketchDocId = kInvalidDocumentId;
                m_selectedSketchId = 0;
                emit documentListChanged();
                emit workpieceStructureChanged();
                emit sketchSelectionChanged(0);
            });
    // 项目打开后刷新工件显示：确保无论通过哪个入口打开，视图都会重建。
    connect(project, &lcnc::LcncProjectManager::projectOpened, this, [this, project](const QString&) {
        if (auto* gd = activeGuiDocument()) {
            gd->rebuildDomain(lcnc::ProjectDomain::Workpiece, project->workpieceDocument());
            if (auto* md = project->machineDocument())
                gd->updateMachineWorkspaceTransforms(md, project->workpieceDocument());
            gd->fitAll();
        }
    });
    connect(project, &lcnc::LcncProjectManager::domainDataChanged, this, [this, project](lcnc::ProjectDomain domain) {
        const DocumentId id = project->documentId(domain);
        if (id != kInvalidDocumentId)
            emit documentModified(id);
        if (domain == lcnc::ProjectDomain::Workpiece) {
            m_documentRegistry->ensure(project->workpieceDocumentId());
            emit workpieceStructureChanged();
        }
    });

    if (LcncDocument* doc = project->workpieceDocument())
        m_documentRegistry->ensure(doc->id());
}

// ── Document Management ────────────────────────────────────────────────────────

DocumentId CadModule::newDocument(const QString& name)
{
    return m_documentIoService ? m_documentIoService->createDocument(name)
                               : kInvalidDocumentId;
}

DocumentId CadModule::openDocument(const QString& filePath)
{
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "CadModule::openDocument begin path={}",
               filePath.toStdString());

    QFileInfo fileInfo(filePath);
    if (filePath.isEmpty() || !fileInfo.exists()) {
        LCNC_WARN(lcnc::LogCode::Generic,
                  "CadModule::openDocument missing file path={}",
                  filePath.toStdString());
        // 中文翻译：打开失败；文件不存在: %1
        emit operationFailed(tr("Open failed"), tr("File does not exist: %1").arg(filePath));
        return kInvalidDocumentId;
    }

    const QString ext = fileInfo.suffix().toLower();
    const bool isProjectPackage = lcnc::LcncProjectPackage::isProjectPath(filePath);
    if (!isProjectPackage &&
        ext != "stp" && ext != "step" &&
        ext != "igs" && ext != "iges" &&
        ext != "stl" && ext != "brep") {
        LCNC_WARN(lcnc::LogCode::Generic,
                  "CadModule::openDocument unsupported suffix path={} suffix={}",
                  filePath.toStdString(), fileInfo.suffix().toStdString());
        // 中文翻译：打开失败；暂不支持的文件格式: %1
        emit operationFailed(tr("Open failed"), tr("File format not supported yet: %1").arg(fileInfo.suffix()));
        return kInvalidDocumentId;
    }

    auto* project = lcnc::Kernel::current().projectManager();
    const std::uint64_t openGeneration = project->beginSingleDocumentOpen();
    if (openGeneration == 0) {
        // 中文翻译：打开失败；当前工程仍有后台任务，无法关闭
        emit operationFailed(tr("Open failed"),
                             tr("The current project still has background tasks and cannot be closed"));
        return kInvalidDocumentId;
    }
    auto pendingWorkspace = project->createDetachedWorkspace(fileInfo.completeBaseName());
    LcncDocument* pendingDoc = pendingWorkspace ? pendingWorkspace->workpieceDocument() : nullptr;
    if (!pendingDoc) {
        // 中文翻译：打开失败；无法创建目标文档
        emit operationFailed(tr("Open failed"), tr("Unable to create target document"));
        return kInvalidDocumentId;
    }
    const DocumentId docId = pendingDoc->id();

    if (isProjectPackage) {
        auto error = std::make_shared<QString>();
        auto loadResult = std::make_shared<lcnc::ProjectLoadResult>();
        const auto* documentIo = m_documentIoService.get();
        const TaskId taskId = lcnc::Kernel::current().taskManager()->run(
            // 中文翻译：打开工程: %1
            tr("Open project: %1").arg(fileInfo.fileName()),
            [filePath, pendingWorkspace, error, loadResult, documentIo](TaskProgress* prog) {
                prog->setRange(0, 100);
                // 中文翻译：读取工程包...
                prog->setStepName(QStringLiteral("Read project package..."));
                if (prog->isAbortRequested())
                    throw std::runtime_error("project open cancelled");
                prog->setValue(10);
                const QString packagePath = lcnc::LcncProjectPackage::packageDirectory(filePath);
                if (!lcnc::LcncProjectPackage::load(*pendingWorkspace->workpieceDocument(),
                                                     nullptr,
                                                     pendingWorkspace->camDocument(),
                                                     packagePath,
                                                     loadResult.get(),
                                                     error.get(),
                                                     pendingWorkspace->camData())) {
                    throw std::runtime_error("project open failed");
                }
                // 中文翻译：生成显示网格...
                prog->setStepName(QStringLiteral("Generate display grid..."));
                if (!documentIo || !documentIo->prepareDisplayMesh(
                        pendingWorkspace->workpieceDocument(), prog, error.get())) {
                    throw std::runtime_error("project display mesh preparation failed");
                }
                if (prog->isAbortRequested())
                    throw std::runtime_error("project open cancelled");
                prog->setValue(100);
            });

        m_taskScope.track(taskId);
        watchTask(this, taskId, [this, taskId, filePath, pendingWorkspace, loadResult, error, openGeneration](bool success) {
            m_taskScope.release(taskId);
            auto* project = lcnc::Kernel::current().projectManager();
            if (!project->isSingleDocumentOpenCurrent(openGeneration))
                return;
            if (!success) {
                // 中文翻译：打开失败
                emit operationFailed(tr("Open failed"),
                                     // 中文翻译：无法读取项目文件
                                     error->isEmpty() ? tr("Unable to read project file") : *error);
                return;
            }

            const QString packagePath = lcnc::LcncProjectPackage::packageDirectory(filePath);
            const QString projectName = loadResult->documentName.isEmpty()
                ? QFileInfo(packagePath).completeBaseName()
                : loadResult->documentName;
            pendingWorkspace->workpieceDocument()->setName(projectName);
            pendingWorkspace->workpieceDocument()->setFilePath(
                loadResult->packagePath.isEmpty() ? packagePath : loadResult->packagePath);
            pendingWorkspace->session().setProjectName(projectName);
            pendingWorkspace->session().setProjectPath(pendingWorkspace->workpieceDocument()->filePath());
            pendingWorkspace->session().setManifest(loadResult->manifest);
            pendingWorkspace->session().workpiece().displayName = projectName;
            pendingWorkspace->session().workpiece().sourceFilePath = loadResult->manifest.sourceFilePath;
            pendingWorkspace->syncSessionFromDocuments();
            pendingWorkspace->session().clearDirty();
            project->adoptWorkspace(pendingWorkspace, true);

            if (auto* gd = activeGuiDocument()) {
                gd->rebuildDomain(lcnc::ProjectDomain::Workpiece, project->workpieceDocument());
                gd->updateMachineWorkspaceTransforms(project->machineDocument(), project->workpieceDocument());
                gd->fitAll();
            }
        });
        return docId;
    }

    LCNC_DEBUG(lcnc::LogCode::Generic,
               "CadModule::openDocument replacing workpiece docId={} ext={} path={}",
               docId, ext.toStdString(), filePath.toStdString());

    auto error = std::make_shared<QString>();
    const auto* documentIo = m_documentIoService.get();
    const TaskId taskId = lcnc::Kernel::current().taskManager()->run(
        // 中文翻译：打开: %1
        tr("Open: %1").arg(fileInfo.fileName()),
        [filePath, ext, pendingWorkspace, error, documentIo](TaskProgress* prog) {
            LcncDocument* doc = pendingWorkspace ? pendingWorkspace->workpieceDocument() : nullptr;
            LCNC_DEBUG(lcnc::LogCode::Generic,
                       "CadModule::openDocument worker begin docId={} ext={} path={}",
                       doc ? doc->id() : kInvalidDocumentId,
                       ext.toStdString(),
                       filePath.toStdString());
            prog->setRange(0, 100);
            if (prog->isAbortRequested())
                throw std::runtime_error("document open cancelled");

            if (ext == "stp" || ext == "step") {
                // 中文翻译：读取 STEP...
                prog->setStepName(QStringLiteral("Read STEP..."));
                prog->setValue(50);
                // 中文翻译：转换形体...
                prog->setStepName(QStringLiteral("Transform body..."));
                if (!documentIo || !documentIo->importStepIntoDocument(doc, filePath, error.get())) {
                    throw std::runtime_error("step open failed");
                }
            } else if (ext == "igs" || ext == "iges") {
                // 中文翻译：读取 IGES...
                prog->setStepName(QStringLiteral("Read IGES..."));
                prog->setValue(50);
                // 中文翻译：转换形体...
                prog->setStepName(QStringLiteral("Transform body..."));
                if (!documentIo || !documentIo->importIgesIntoDocument(doc, filePath, error.get())) {
                    throw std::runtime_error("iges open failed");
                }
            } else if (ext == "stl") {
                if (!documentIo || !documentIo->importStlIntoDocument(doc,
                                                                       filePath,
                                                                       prog,
                                                                       error.get())) {
                    throw std::runtime_error("stl open failed");
                }
            } else if (ext == "brep") {
                if (!documentIo || !documentIo->importBrepIntoDocument(doc,
                                                                        filePath,
                                                                        prog,
                                                                        error.get())) {
                    throw std::runtime_error("brep open failed");
                }
            }

            // 中文翻译：生成显示网格...
            prog->setStepName(QStringLiteral("Generate display grid..."));
            if (!documentIo || !documentIo->prepareDisplayMesh(doc, prog, error.get()))
                throw std::runtime_error("display mesh preparation failed");

            LCNC_DEBUG(lcnc::LogCode::Generic,
                       "CadModule::openDocument worker done docId={} workpieceCount={}",
                       doc ? doc->id() : kInvalidDocumentId,
                       entityCount(doc, LcncDocument::EntityKind::Workpiece));
            if (prog->isAbortRequested())
                throw std::runtime_error("document open cancelled");
            prog->setValue(100);
        });

    LCNC_DEBUG(lcnc::LogCode::Generic,
               "CadModule::openDocument task scheduled docId={} taskId={}",
               docId, taskId);

    const QString displayName = fileInfo.completeBaseName();
    const QString sourceFilePath = fileInfo.absoluteFilePath();
    m_taskScope.track(taskId);
    watchTask(this, taskId, [this, taskId, pendingWorkspace, docId, displayName, sourceFilePath, error, openGeneration](bool success) {
        m_taskScope.release(taskId);
        auto* project = lcnc::Kernel::current().projectManager();
        if (!project->isSingleDocumentOpenCurrent(openGeneration))
            return;

        LCNC_DEBUG(lcnc::LogCode::Generic,
                   "CadModule::openDocument task done docId={} success={}",
                   docId, success);
        if (!success) {
            // 中文翻译：打开失败
            emit operationFailed(tr("Open failed"),
                                 // 中文翻译：打开文件失败
                                 error->isEmpty() ? tr("Failed to open file") : *error);
            return;
        }

        pendingWorkspace->session().workpiece().displayName = displayName;
        pendingWorkspace->session().workpiece().sourceFilePath = sourceFilePath;
        pendingWorkspace->syncSessionFromDocuments();
        project->adoptWorkspace(pendingWorkspace);
        refreshDisplay(docId);
    });

    return docId;
}

DocumentId CadModule::importStep(const QString& filePath, DocumentId targetDocId)
{
    QFileInfo fileInfo(filePath);
    if (filePath.isEmpty() || !fileInfo.exists()) {
        // 中文翻译：导入 STEP 失败；文件不存在: %1
        emit operationFailed(tr("Import STEP failed"), tr("File does not exist: %1").arg(filePath));
        return kInvalidDocumentId;
    }

    bool createdNew = false;
    const DocumentId docId = ensureTargetDocument(this, targetDocId, fileInfo.baseName(), &createdNew);
    LcncDocument* doc = domainDocumentById(docId);
    if (!doc) {
        // 中文翻译：导入 STEP 失败；无法创建目标文档
        emit operationFailed(tr("Import STEP failed"), tr("Unable to create target document"));
        return kInvalidDocumentId;
    }

    auto error = std::make_shared<QString>();
    const auto* documentIo = m_documentIoService.get();
    const TaskId taskId = lcnc::Kernel::current().taskManager()->run(
        // 中文翻译：导入 STEP: %1
        tr("Import STEP: %1").arg(fileInfo.fileName()),
        [filePath, doc, error, documentIo](TaskProgress* prog) {
            prog->setRange(0, 100);
            // 中文翻译：读取 STEP...
            prog->setStepName(QStringLiteral("Read STEP..."));
            if (prog->isAbortRequested())
                throw std::runtime_error("step import cancelled");

            prog->setValue(50);
            // 中文翻译：转换形体...
            prog->setStepName(QStringLiteral("Transform body..."));
            if (!documentIo || !documentIo->importStepIntoDocument(doc, filePath, error.get())) {
                throw std::runtime_error("step import failed");
            }
            // 中文翻译：生成显示网格...
            prog->setStepName(QStringLiteral("Generate display grid..."));
            if (!documentIo || !documentIo->prepareDisplayMesh(doc, prog, error.get()))
                throw std::runtime_error("display mesh preparation failed");
            if (prog->isAbortRequested())
                throw std::runtime_error("step import cancelled");
            prog->setValue(100);
        });

    const QString displayName = fileInfo.completeBaseName();
    const QString sourceFilePath = fileInfo.absoluteFilePath();
    m_taskScope.track(taskId);
    watchTask(this, taskId, [this, taskId, docId, createdNew, displayName, sourceFilePath, error](bool success) {
        m_taskScope.release(taskId);
        if (!success) {
            if (createdNew)
                closeDocument(docId);
            // 中文翻译：导入 STEP 失败
            emit operationFailed(tr("Import STEP failed"),
                                 // 中文翻译：导入 STEP 失败
                                 error->isEmpty() ? tr("Import STEP failed") : *error);
            return;
        }

        auto* project = lcnc::Kernel::current().projectManager();
        if (project->isDomainDocument(docId, lcnc::ProjectDomain::Workpiece)) {
            project->session().workpiece().displayName = displayName;
            project->session().workpiece().sourceFilePath = sourceFilePath;
        }
        refreshDisplay(docId);
    });

    return docId;
}

DocumentId CadModule::importStl(const QString& filePath, DocumentId targetDocId)
{
    QFileInfo fileInfo(filePath);
    if (filePath.isEmpty() || !fileInfo.exists()) {
        // 中文翻译：导入 STL 失败；文件不存在: %1
        emit operationFailed(tr("Import STL failed"), tr("File does not exist: %1").arg(filePath));
        return kInvalidDocumentId;
    }

    bool createdNew = false;
    const DocumentId docId = ensureTargetDocument(this, targetDocId, fileInfo.baseName(), &createdNew);
    LcncDocument* doc = domainDocumentById(docId);
    if (!doc) {
        // 中文翻译：导入 STL 失败；无法创建目标文档
        emit operationFailed(tr("Import STL failed"), tr("Unable to create target document"));
        return kInvalidDocumentId;
    }

    const auto task = m_documentIoService
        ? m_documentIoService->importStlAsync(doc, filePath)
        : lcnc::cad::CadDocumentIoService::ImportTask{};
    if (task.id == kInvalidTaskId) {
        // 中文翻译：导入 STL 失败
        emit operationFailed(tr("Import STL failed"),
                             task.error && !task.error->isEmpty()
                                 ? *task.error : tr("Import STL failed"));
        return kInvalidDocumentId;
    }

    const QString displayName = fileInfo.completeBaseName();
    const QString sourceFilePath = fileInfo.absoluteFilePath();
    m_taskScope.track(task.id);
    watchTask(this, task.id, [this, task, taskId = task.id, docId, createdNew, displayName, sourceFilePath](bool success) {
        m_taskScope.release(taskId);
        if (!success) {
            if (createdNew)
                closeDocument(docId);
            // 中文翻译：导入 STL 失败
            emit operationFailed(tr("Import STL failed"),
                                 // 中文翻译：导入 STL 失败
                                 task.error->isEmpty() ? tr("Import STL failed") : *task.error);
            return;
        }

        auto* project = lcnc::Kernel::current().projectManager();
        if (project->isDomainDocument(docId, lcnc::ProjectDomain::Workpiece)) {
            project->session().workpiece().displayName = displayName;
            project->session().workpiece().sourceFilePath = sourceFilePath;
        }
        refreshDisplay(docId);
    });

    return docId;
}

bool CadModule::saveDocument(DocumentId id, const QString& path)
{
    LcncDocument* doc = domainDocumentById(id);
    QString err;
    const bool ok = m_documentIoService
        && m_documentIoService->saveDocument(doc, path, &err);
    if (!ok)
        // 中文翻译：保存失败；保存文档失败
        emit operationFailed(tr("Save failed"), err.isEmpty() ? tr("Failed to save document") : err);
    return ok;
}

void CadModule::exportStep(DocumentId id, const QString& filePath)
{
    LcncDocument* doc = domainDocumentById(id);
    const auto task = m_documentIoService
        ? m_documentIoService->exportStepAsync(doc, filePath)
        : lcnc::cad::CadDocumentIoService::ExportTask{};
    if (task.id == kInvalidTaskId) {
        // 中文翻译：导出 STEP 失败
        emit operationFailed(tr("Export STEP failed"),
                             task.error && !task.error->isEmpty()
                                 ? *task.error : tr("Export STEP failed"));
        return;
    }

    m_taskScope.track(task.id);
    watchTask(this, task.id, [this, task, taskId = task.id](bool success) {
        m_taskScope.release(taskId);
        if (!success) {
            // 中文翻译：导出 STEP 失败
            emit operationFailed(tr("Export STEP failed"),
                                 // 中文翻译：导出 STEP 失败
                                 task.error->isEmpty() ? tr("Export STEP failed") : *task.error);
        }
    });
}

void CadModule::closeDocument(DocumentId id)
{
    // Background import/export jobs borrow the document.  Closing must wait
    // for their cooperative cancellation instead of invalidating that borrow.
    if (!cancelOwnedTasks(10000)) {
        // 中文翻译：仍有 CAD 后台任务在运行，无法关闭文档
        emit operationFailed(tr("Close document failed"),
                             tr("CAD background tasks are still running; the document remains open"));
        return;
    }
    if (m_documentIoService)
        (void)m_documentIoService->closeDocument(id);
}

DocumentId CadModule::importFile(const QString& filePath)
{
    const QString ext = QFileInfo(filePath).suffix().toLower();
    if (ext == "stl")
        return importStl(filePath);
    if (ext == "stp" || ext == "step")
        return importStep(filePath);
    return openDocument(filePath);
}

// ── Project Domain Access ─────────────────────────────────────────────────────

DocumentId CadModule::workpieceDocumentId() const
{
    return lcnc::Kernel::current().projectManager()->workpieceDocumentId();
}

LcncDocument* CadModule::workpieceDocument() const
{
    return lcnc::Kernel::current().projectManager()->workpieceDocument();
}

GuiDocument* CadModule::activeGuiDocument() const
{
    return lcnc::Kernel::current().guiApp()->activeGuiDocument();
}

LcncDocument* CadModule::domainDocumentById(DocumentId id) const
{
    return lcnc::Kernel::current().projectManager()->domainDocumentById(id);
}

void CadModule::requestWorkpieceView(DocumentId id)
{
    if (id == kInvalidDocumentId)
        id = workpieceDocumentId();
    emit workpieceViewRequested(id);
}

void CadModule::setEntityVisible(DocumentId docId, const QString& entry, bool visible)
{
    if (entry.isEmpty())
        return;

    if (auto* gd = activeGuiDocument()) {
        Handle(AIS_Shape) ais = gd->aisShape(docId, entry);
        if (ais.IsNull())
            return;

        if (visible)
            gd->scene()->displayObject(ais);
        else
            gd->scene()->eraseObject(ais);

        if (gd->hasView())
            gd->view()->Redraw();
    }
}

void CadModule::setEntriesVisible(DocumentId docId, const QStringList& entries, bool visible)
{
    for (const QString& entry : entries)
        setEntityVisible(docId, entry, visible);
}

void CadModule::setSelectedEntries(DocumentId docId, const QStringList& entries)
{
    GuiDocument* gd = activeGuiDocument();
    if (!gd)
        return;

    const Handle(AIS_InteractiveContext)& ctx = gd->context();
    if (ctx.IsNull())
        return;

    ctx->ClearSelected(false);
    for (const QString& entry : entries) {
        Handle(AIS_Shape) ais = gd->aisShape(docId, entry);
        if (!ais.IsNull())
            ctx->AddOrRemoveSelected(ais, false);
    }

    if (gd->hasView())
        gd->view()->Redraw();

    const QStringList selected = gd->selectedEntries(docId);
    if (auto* state = m_documentRegistry->ensure(docId)) {
        state->selectionContext() = lcnc::cad::selection::CadSelectionResolver::fromShapeEntries(
            docId, selected, true, isSketchEditing(), hasSelectedSketch(), m_selectedSketchId);
    }

    emit selectionChanged(docId, selected);
}

QStringList CadModule::selectedEntries(DocumentId docId) const
{
    if (auto* gd = activeGuiDocument())
        return gd->selectedEntries(docId);

    return {};
}

void CadModule::syncSelectionFromView(DocumentId docId)
{
    if (docId == kInvalidDocumentId)
        docId = workpieceDocumentId();

    const QStringList entries = selectedEntries(docId);
    if (auto* state = m_documentRegistry->ensure(docId)) {
        state->selectionContext() = lcnc::cad::selection::CadSelectionResolver::fromShapeEntries(
            docId, entries, docId != kInvalidDocumentId, isSketchEditing(), hasSelectedSketch(), m_selectedSketchId);
    }
    emit selectionChanged(docId, entries);
}

lcnc::cad::selection::CadSelectionContext CadModule::selectionContext(DocumentId docId) const
{
    if (docId == kInvalidDocumentId)
        docId = workpieceDocumentId();

    lcnc::cad::selection::CadSelectionContext context;
    if (const auto* state = m_documentRegistry->get(docId))
        context = state->selectionContext();
    context.docId = docId;
    context.hasDocument = docId != kInvalidDocumentId && domainDocumentById(docId);
    context.sketchEditing = isSketchEditing();
    context.selectedSketchId = m_selectedSketchId;
    context.hasSelectedSketch = hasSelectedSketch();

    const QStringList entries = selectedEntries(docId);
    if (!entries.isEmpty()) {
        context = lcnc::cad::selection::CadSelectionResolver::fromShapeEntries(
            docId, entries, context.hasDocument, context.sketchEditing,
            context.hasSelectedSketch, context.selectedSketchId);
    } else {
        context.selectedShapeCount = 0;
        bool hasNonShapeSelection = false;
        for (const auto& item : context.items) {
            if (item.domain != lcnc::cad::selection::CadSelectionDomain::DocumentShape
                && item.domain != lcnc::cad::selection::CadSelectionDomain::None) {
                hasNonShapeSelection = true;
                break;
            }
        }
        if (!context.hasSelectedSketch && !hasNonShapeSelection)
            context.items.clear();
    }
    return context;
}

void CadModule::setSelectionContext(const lcnc::cad::selection::CadSelectionContext& context)
{
    if (context.docId == kInvalidDocumentId)
        return;

    if (auto* state = m_documentRegistry->ensure(context.docId))
        state->selectionContext() = context;

    if (context.hasSelectedSketch && context.selectedSketchId > 0) {
        if (m_selectedSketchDocId != context.docId || m_selectedSketchId != context.selectedSketchId) {
            m_selectedSketchDocId = context.docId;
            m_selectedSketchId = context.selectedSketchId;
            emit sketchSelectionChanged(m_selectedSketchId);
        }
        if (auto* gd = activeGuiDocument()) {
            const Handle(AIS_InteractiveContext)& ctx = gd->context();
            if (!ctx.IsNull())
                ctx->ClearSelected(false);
            if (gd->hasView())
                gd->view()->Redraw();
        }
        emit selectionChanged(context.docId, {});
        return;
    }

    if (m_selectedSketchId > 0) {
        m_selectedSketchId = 0;
        m_selectedSketchDocId = kInvalidDocumentId;
        emit sketchSelectionChanged(0);
    }

    QStringList entries;
    for (const auto& item : context.items) {
        if (item.domain == lcnc::cad::selection::CadSelectionDomain::DocumentShape && !item.entry.isEmpty())
            entries.append(item.entry);
    }
    if (entries.isEmpty()) {
        if (auto* gd = activeGuiDocument()) {
            const Handle(AIS_InteractiveContext)& ctx = gd->context();
            if (!ctx.IsNull())
                ctx->ClearSelected(false);
            if (gd->hasView())
                gd->view()->Redraw();
        }
        emit selectionChanged(context.docId, {});
        return;
    }
    setSelectedEntries(context.docId, entries);
}

// ── Modeling Operations ────────────────────────────────────────────────────────

bool CadModule::moveShape(DocumentId docId, const TDF_Label& label, const gp_Vec& translation)
{
    return moveShapes(docId, { label }, translation);
}

bool CadModule::moveShapes(DocumentId docId, const QList<TDF_Label>& labels,
                           const gp_Vec& translation)
{
    LcncDocument* doc = domainDocumentById(docId);
    if (!doc || labels.isEmpty())
        return false;

    // 中文翻译：移动形体
    doc->openCommand(tr("moving body"));
    for (const TDF_Label& label : labels) {
        if (!ShapeService::moveShape(doc, label, translation)) {
            doc->abortCommand();
            return false;
        }
    }

    doc->commitCommand();
    refreshDisplay(docId);
    return true;
}

bool CadModule::rotateShape(DocumentId docId, const TDF_Label& label,
                             const gp_Ax1& axis, double angleDeg)
{
    return rotateShapes(docId, { label }, axis, angleDeg);
}

bool CadModule::rotateShapes(DocumentId docId, const QList<TDF_Label>& labels,
                             const gp_Ax1& axis, double angleDeg)
{
    LcncDocument* doc = domainDocumentById(docId);
    if (!doc || labels.isEmpty())
        return false;

    // 中文翻译：旋转形体
    doc->openCommand(tr("rotating shape"));
    for (const TDF_Label& label : labels) {
        if (!ShapeService::rotateShape(doc, label, axis, angleDeg)) {
            doc->abortCommand();
            return false;
        }
    }

    doc->commitCommand();
    refreshDisplay(docId);
    return true;
}

bool CadModule::deleteShape(DocumentId docId, const QString& entry)
{
    return deleteShapes(docId, { entry });
}

bool CadModule::deleteShapes(DocumentId docId, const QStringList& entries)
{
    LcncDocument* doc = domainDocumentById(docId);
    if (!doc || entries.isEmpty())
        return false;

    // 中文翻译：删除形体
    doc->openCommand(tr("Delete shape"));
    if (auto* gd = activeGuiDocument()) {
        for (const QString& entry : entries)
            gd->eraseEntity(docId, entry);
    }

    for (const QString& entry : entries)
        ShapeService::deleteShape(doc, entry);

    doc->commitCommand();
    refreshDisplay(docId);
    return true;
}

int CadModule::explodeShape(DocumentId docId, const TDF_Label& label, int entityKind)
{
    LcncDocument* doc = domainDocumentById(docId);
    if (!doc) return 0;

    // 中文翻译：拆解形体
    doc->openCommand(tr("Explode Shape"));

    // Erase original AIS object
    const QString entry = XcafUtils::entry(label);
    if (auto* gd = activeGuiDocument())
        gd->eraseEntity(docId, entry);

    int count = ShapeService::explodeShape(doc, label, entityKind);
    if (count <= 0) {
        doc->abortCommand();
        return 0;
    }

    doc->commitCommand();
    refreshDisplay(docId);
    return count;
}

TDF_Label CadModule::createShape(DocumentId docId, const TopoDS_Shape& shape,
                                  const QString& name, int entityKind)
{
    LcncDocument* doc = domainDocumentById(docId);
    if (!doc) return TDF_Label();

    // 中文翻译：创建形体
    doc->openCommand(tr("Create shapes"));
    TDF_Label label = ShapeService::addShape(doc, shape, name, entityKind);
    if (label.IsNull()) {
        doc->abortCommand();
        return TDF_Label();
    }

    doc->commitCommand();
    refreshDisplay(docId);
    return label;
}

void CadModule::requestPrimitiveTool(int primitiveIndex)
{
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "CadModule::requestPrimitiveTool primitive={}", primitiveIndex);
    emit primitiveToolRequested(primitiveIndex);
}

bool CadModule::buildPrimitivePreview(int primitiveIndex,
                                      const PrimitiveParameters& params,
                                      TopoDS_Shape* outShape,
                                      QString* errMsg)
{
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "CadModule::buildPrimitivePreview begin primitive={}", primitiveIndex);
    if (outShape)
        outShape->Nullify();

    QString buildError;
    const TopoDS_Shape shape = buildPrimitiveShape(primitiveIndex, params, &buildError);
    if (shape.IsNull()) {
        if (errMsg)
            *errMsg = buildError;
        LCNC_DEBUG(lcnc::LogCode::Generic,
                   "CadModule::buildPrimitivePreview end success=false reason={}",
                   buildError.toStdString());
        return false;
    }

    if (outShape)
        *outShape = shape;
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "CadModule::buildPrimitivePreview end success=true");
    return true;
}

bool CadModule::createPrimitive(int primitiveIndex, const PrimitiveParameters& params)
{
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "CadModule::createPrimitive begin primitive={}", primitiveIndex);

    QString buildError;
    const TopoDS_Shape shape = buildPrimitiveShape(primitiveIndex, params, &buildError);
    if (shape.IsNull()) {
        LCNC_WARN(lcnc::LogCode::Generic,
                  "CadModule::createPrimitive build failed: {}",
                  buildError.toStdString());
        // 中文翻译：创建基础体失败
        emit operationFailed(tr("Failed to create base body"), buildError);
        return false;
    }

    DocumentId docId = workpieceDocumentId();
    if (docId == kInvalidDocumentId)
        docId = newDocument(primitiveName(primitiveIndex));
    if (docId == kInvalidDocumentId || !domainDocumentById(docId)) {
        LCNC_WARN(lcnc::LogCode::Generic,
                  "CadModule::createPrimitive failed to prepare document");
        // 中文翻译：创建基础体失败；无法创建或激活目标文档
        emit operationFailed(tr("Failed to create base body"), tr("Unable to create or activate target document"));
        return false;
    }

    const TDF_Label label = createShape(
        docId,
        shape,
        primitiveName(primitiveIndex),
        static_cast<int>(LcncDocument::EntityKind::Workpiece));
    if (label.IsNull()) {
        LCNC_WARN(lcnc::LogCode::Generic,
                  "CadModule::createPrimitive createShape failed docId={}", docId);
        // 中文翻译：创建基础体失败；无法写入目标文档
        emit operationFailed(tr("Failed to create base body"), tr("Unable to write to target document"));
        return false;
    }

    requestWorkpieceView(docId);
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "CadModule::createPrimitive end success=true docId={}", docId);
    return true;
}

bool CadModule::previewTool(const QString& toolId,
                            const QVariantMap& params,
                            TopoDS_Shape* outShape,
                            QString* errMsg)
{
    if (!m_commandDispatcher) {
        if (errMsg)
            // 中文翻译：CAD 工具调度器未初始化
            *errMsg = tr("CAD tool scheduler not initialized");
        return false;
    }

    lcnc::cad::task::CadCommandRequest request;
    request.selection = selectionContext();
    request.params = params;
    request.preview = true;
    return m_commandDispatcher->preview(toolId, request, outShape, errMsg);
}

bool CadModule::executeTool(const QString& toolId,
                            const QVariantMap& params,
                            QString* errMsg)
{
    if (!m_commandDispatcher) {
        if (errMsg)
            // 中文翻译：CAD 工具调度器未初始化
            *errMsg = tr("CAD tool scheduler not initialized");
        return false;
    }

    lcnc::cad::task::CadCommandRequest request;
    request.selection = selectionContext();
    request.params = params;
    request.preview = false;
    const bool ok = m_commandDispatcher->execute(toolId, request, errMsg);
    if (!ok && errMsg && !errMsg->isEmpty())
        // 中文翻译：CAD 工具失败
        emit operationFailed(tr("CAD tool failed"), *errMsg);
    return ok;
}

bool CadModule::buildTransformPreview(const TransformParameters& params,
                                      TopoDS_Shape* outShape,
                                      double* refX,
                                      double* refY,
                                      double* refZ,
                                      QString* errMsg) const
{
    if (outShape)
        outShape->Nullify();

    const DocumentId docId = workpieceDocumentId();
    LcncDocument* doc = domainDocumentById(docId);
    if (!doc) {
        if (errMsg)
            // 中文翻译：请先导入或创建工件模型
            *errMsg = tr("Please import or create an workpiece model first");
        return false;
    }

    const QStringList entries = selectedEntries(docId);
    const QList<TDF_Label> labels = selectedShapeLabels(doc, entries);
    if (labels.isEmpty()) {
        if (errMsg)
            // 中文翻译：请先选择要变换的形体
            *errMsg = tr("Please select the shape you want to transform first");
        return false;
    }

    gp_Pnt referencePoint(0.0, 0.0, 0.0);
    if (params.referenceMode == 0
        && !modelCenterForLabels(doc, labels, &referencePoint, errMsg)) {
        return false;
    }
    if (refX)
        *refX = referencePoint.X();
    if (refY)
        *refY = referencePoint.Y();
    if (refZ)
        *refZ = referencePoint.Z();

    TopoDS_Compound compound;
    BRep_Builder builder;
    builder.MakeCompound(compound);
    const Handle(XCAFDoc_ShapeTool) shapeTool = doc->shapeTool();
    const auto algoParams = toAlgorithmTransform(params, referencePoint);
    for (const TDF_Label& label : labels) {
        const TopoDS_Shape shape = shapeTool->GetShape(label);
        if (shape.IsNull())
            continue;

        QString transformError;
        const TopoDS_Shape transformed = lcnc::cad::invokeCadAlgorithm(
            [&] { return lcnc::cad_algo::transformShape(shape, algoParams); },
            &transformError);
        if (transformed.IsNull()) {
            if (errMsg)
                *errMsg = transformError;
            return false;
        }
        builder.Add(compound, transformed);
    }

    if (compound.IsNull()) {
        if (errMsg)
            // 中文翻译：变换预览为空
            *errMsg = tr("Transform preview is empty");
        return false;
    }
    if (outShape)
        *outShape = compound;
    return true;
}

bool CadModule::applyTransform(const TransformParameters& params, QString* errMsg)
{
    auto fail = [this, errMsg](const QString& message) {
        if (errMsg)
            *errMsg = message;
        // 中文翻译：CAD 变换失败
        emit operationFailed(tr("CAD transformation failed"), message);
        return false;
    };

    const DocumentId docId = workpieceDocumentId();
    LcncDocument* doc = domainDocumentById(docId);
    if (!doc)
        // 中文翻译：请先导入或创建工件模型
        return fail(tr("Please import or create an workpiece model first"));

    const QStringList entries = selectedEntries(docId);
    const QList<TDF_Label> labels = selectedShapeLabels(doc, entries);
    if (labels.isEmpty())
        // 中文翻译：请先选择要变换的形体
        return fail(tr("Please select the shape you want to transform first"));

    gp_Pnt referencePoint(0.0, 0.0, 0.0);
    if (params.referenceMode == 0
        && !modelCenterForLabels(doc, labels, &referencePoint, errMsg)) {
        // 中文翻译：无法计算选中形体中心
        return fail(errMsg ? *errMsg : tr("Unable to calculate center of selected shape"));
    }

    const auto algoParams = toAlgorithmTransform(params, referencePoint);
    const Handle(XCAFDoc_ShapeTool) shapeTool = doc->shapeTool();
    // 中文翻译：变换形体
    doc->openCommand(tr("Transform shape"));
    for (const TDF_Label& label : labels) {
        const TopoDS_Shape shape = shapeTool->GetShape(label);
        QString transformError;
        const TopoDS_Shape transformed = lcnc::cad::invokeCadAlgorithm(
            [&] { return lcnc::cad_algo::transformShape(shape, algoParams); },
            &transformError);
        if (transformed.IsNull()) {
            doc->abortCommand();
            return fail(transformError);
        }
        shapeTool->SetShape(label, transformed);
    }
    doc->commitCommand();
    refreshDisplay(docId);
    return true;
}

bool CadModule::beginSketch(int planeIndex)
{
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "CadModule::beginSketch begin planeIndex={}", planeIndex);
    if (!m_modelingSession) {
        LCNC_ERR(lcnc::LogCode::InternalUnexpectedState,
                 "CadModule::beginSketch missing modeling session");
        // 中文翻译：新建草图失败；CAD 建模会话未初始化
        emit operationFailed(tr("New sketch failed"), tr("CAD modeling session not initialized"));
        return false;
    }

    DocumentId docId = workpieceDocumentId();
    if (docId == kInvalidDocumentId)
        // 中文翻译：草图建模
        docId = newDocument(tr("Sketch modeling"));
    if (docId == kInvalidDocumentId || !domainDocumentById(docId)) {
        LCNC_WARN(lcnc::LogCode::Generic,
                  "CadModule::beginSketch failed to prepare document");
        // 中文翻译：新建草图失败；无法创建或激活目标文档
        emit operationFailed(tr("New sketch failed"), tr("Unable to create or activate target document"));
        return false;
    }

    auto planeKind = lcnc::cad::SketchPlaneKind::XY;
    switch (planeIndex) {
    case 1:
        planeKind = lcnc::cad::SketchPlaneKind::YZ;
        break;
    case 2:
        planeKind = lcnc::cad::SketchPlaneKind::ZX;
        break;
    default:
        break;
    }

    m_modelingSession->beginSketch(planeKind);
    requestWorkpieceView(docId);
    emit sketchToolChanged(static_cast<int>(lcnc::cad::SketchToolKind::None));
    emit sketchElementsChanged();
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "CadModule::beginSketch end success=true docId={}", docId);
    return true;
}

bool CadModule::finishSketch()
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "CadModule::finishSketch begin");
    if (!m_modelingSession) {
        LCNC_ERR(lcnc::LogCode::InternalUnexpectedState,
                 "CadModule::finishSketch missing modeling session");
        // 中文翻译：完成草图失败；CAD 建模会话未初始化
        emit operationFailed(tr("Failed to complete sketch"), tr("CAD modeling session not initialized"));
        return false;
    }

    QString errMsg;
    if (!m_modelingSession->finishSketch(&errMsg)) {
        LCNC_WARN(lcnc::LogCode::Generic,
                  "CadModule::finishSketch failed: {}",
                  errMsg.toStdString());
        // 中文翻译：完成草图失败
        emit operationFailed(tr("Failed to complete sketch"), errMsg);
        return false;
    }

    DocumentId docId = workpieceDocumentId();
    if (docId == kInvalidDocumentId) {
        LCNC_WARN(lcnc::LogCode::Generic,
                  "CadModule::finishSketch no project document active");
        // 中文翻译：完成草图失败；项目文档不可用
        emit operationFailed(tr("Failed to complete sketch"), tr("Project documentation is not available"));
        return false;
    }

    auto* state = m_documentRegistry->ensure(docId);
    auto* manager = state ? &state->sketchManager() : nullptr;
    if (!manager) {
        // 中文翻译：完成草图失败；无法获取草图管理器
        emit operationFailed(tr("Failed to complete sketch"), tr("Unable to get sketch manager"));
        return false;
    }

    // 拉取 finished session 状态入管理器，随后重置 session 供下一个草图使用。
    const auto& elements = m_modelingSession->sketchElements();
    const lcnc::cad::SketchPlaneKind sessionPlane = m_modelingSession->finishedPlane();
    const TopoDS_Face profileFace = m_modelingSession->finishedProfileFace();
    const int sketchId = manager->addSketch(
        sessionPlane,
        std::vector<lcnc::cad::SketchElement>(elements.begin(), elements.end()),
        profileFace);
    m_modelingSession->clear();
    m_selectedSketchDocId = docId;
    m_selectedSketchId = sketchId;
    if (state) {
        state->presentationState().selectedSketchId = sketchId;
        state->selectionContext() = lcnc::cad::selection::CadSelectionResolver::fromProjectExplorerNode(
            docId,
            QStringLiteral("__sketch_finished_%1__").arg(sketchId),
            QString(),
            {});
    }
    emit sketchToolChanged(static_cast<int>(lcnc::cad::SketchToolKind::None));
    emit sketchElementsChanged();
    emit finishedSketchesChanged(docId);
    emit sketchSelectionChanged(sketchId);
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "CadModule::finishSketch end success=true sketchId={}", sketchId);
    return true;
}

bool CadModule::applyFeature(int featureIndex, double length, double angleDeg)
{
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "CadModule::applyFeature begin feature={} length={} angle={} sketchId={}",
               featureIndex, length, angleDeg, m_selectedSketchId);

    DocumentId docId = m_selectedSketchDocId;
    if (docId == kInvalidDocumentId)
        docId = workpieceDocumentId();
    auto* manager = m_documentRegistry->sketchManager(docId);
    if (!manager || m_selectedSketchId <= 0) {
        // 中文翻译：应用特征失败；请先完成并选择一个草图
        emit operationFailed(tr("Apply feature failed"), tr("Please complete and select a sketch first"));
        return false;
    }
    auto* record = manager->sketch(m_selectedSketchId);
    if (!record || record->profileFace.IsNull()) {
        // 中文翻译：应用特征失败；选中草图不包含可用轮廓
        emit operationFailed(tr("Apply feature failed"), tr("Selected sketch does not contain usable outlines"));
        return false;
    }

    auto featureKind = lcnc::cad::FeatureKind::Extrude;
    switch (featureIndex) {
    case 1:
        featureKind = lcnc::cad::FeatureKind::Revolve;
        break;
    case 2:
        featureKind = lcnc::cad::FeatureKind::Sweep;
        break;
    default:
        break;
    }

    QString featureErr;
    const TopoDS_Shape featureShape = lcnc::cad::CadModelingSession::buildFeatureFromRecord(
        record->plane, record->profileFace, featureKind, length, angleDeg, &featureErr);
    if (featureShape.IsNull()) {
        LCNC_WARN(lcnc::LogCode::Generic,
                  "CadModule::applyFeature build failed: {}",
                  featureErr.toStdString());
        // 中文翻译：应用特征失败
        emit operationFailed(tr("Apply feature failed"), featureErr);
        return false;
    }

    if (!domainDocumentById(docId)) {
        // 中文翻译：应用特征失败；目标文档不存在
        emit operationFailed(tr("Apply feature failed"), tr("The target document does not exist"));
        return false;
    }

    const TDF_Label label = createShape(
        docId,
        featureShape,
        m_modelingSession->defaultFeatureName(featureKind),
        static_cast<int>(LcncDocument::EntityKind::Workpiece));
    if (label.IsNull()) {
        // 中文翻译：应用特征失败；无法写入目标文档
        emit operationFailed(tr("Apply feature failed"), tr("Unable to write to target document"));
        return false;
    }

    requestWorkpieceView(docId);
    manager->markSketchUsedByFeature(m_selectedSketchId, XcafUtils::entry(label));
    emit finishedSketchesChanged(docId);
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "CadModule::applyFeature end success=true docId={}", docId);
    return true;
}

bool CadModule::buildFeaturePreview(int featureIndex,
                                    double length,
                                    double angleDeg,
                                    TopoDS_Shape* outShape,
                                    QString* errMsg)
{
    if (outShape)
        outShape->Nullify();

    DocumentId docId = m_selectedSketchDocId;
    auto* manager = m_documentRegistry->sketchManager(docId);
    if (!manager || m_selectedSketchId <= 0) {
        if (errMsg)
            // 中文翻译：请先选择一个草图
            *errMsg = tr("Please select a sketch first");
        return false;
    }
    auto* record = manager->sketch(m_selectedSketchId);
    if (!record || record->profileFace.IsNull()) {
        if (errMsg)
            // 中文翻译：选中草图不包含可用轮廓
            *errMsg = tr("Selected sketch does not contain usable outlines");
        return false;
    }

    auto featureKind = lcnc::cad::FeatureKind::Extrude;
    switch (featureIndex) {
    case 1: featureKind = lcnc::cad::FeatureKind::Revolve; break;
    case 2: featureKind = lcnc::cad::FeatureKind::Sweep; break;
    default: break;
    }
    QString featureErr;
    const TopoDS_Shape previewShape = lcnc::cad::CadModelingSession::buildFeatureFromRecord(
        record->plane, record->profileFace, featureKind, length, angleDeg, &featureErr);
    if (previewShape.IsNull()) {
        if (errMsg)
            *errMsg = featureErr;
        return false;
    }
    if (outShape)
        *outShape = previewShape;
    return true;
}

void CadModule::cancelModelingOperation()
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "CadModule::cancelModelingOperation");
    if (m_modelingSession) {
        const bool wasEditing = m_modelingSession->isSketchEditing();
        m_modelingSession->clear();
        if (wasEditing) {
            emit sketchToolChanged(static_cast<int>(lcnc::cad::SketchToolKind::None));
            emit sketchElementsChanged();
        }
    }
}

bool CadModule::isSketchEditing() const
{
    return m_modelingSession && m_modelingSession->isSketchEditing();
}

LcncDocument* CadModule::projectExplorerWorkpieceDocument() const
{
    return workpieceDocument();
}

bool CadModule::projectExplorerIsSketchEditing() const
{
    return isSketchEditing();
}

QList<lcnc::cad::ProjectExplorerSketchElement>
CadModule::projectExplorerActiveSketchElements() const
{
    QList<lcnc::cad::ProjectExplorerSketchElement> result;
    for (const SketchElementSnapshot& element : sketchElementSnapshots())
        result.append({element.id, element.label});
    return result;
}

QList<lcnc::cad::ProjectExplorerSketch>
CadModule::projectExplorerFinishedSketches(DocumentId documentId) const
{
    QList<lcnc::cad::ProjectExplorerSketch> result;
    for (const FinishedSketchSnapshot& sketch : finishedSketchSnapshots(documentId)) {
        lcnc::cad::ProjectExplorerSketch projected;
        projected.sketchId = sketch.sketchId;
        projected.name = sketch.name;
        projected.visible = sketch.visible;
        projected.usedByFeature = sketch.usedByFeature;
        for (const SketchElementSnapshot& element : sketch.elements)
            projected.elements.append({element.id, element.label});
        result.append(std::move(projected));
    }
    return result;
}

bool CadModule::hasSelectedSketch() const
{
    if (m_selectedSketchId <= 0)
        return false;
    auto* manager = m_documentRegistry->sketchManager(m_selectedSketchDocId);
    if (!manager)
        return false;
    auto* record = manager->sketch(m_selectedSketchId);
    return record && !record->profileFace.IsNull();
}

int CadModule::selectedSketchId() const
{
    return m_selectedSketchId;
}

void CadModule::setSelectedSketchId(int sketchId)
{
    if (m_selectedSketchId == sketchId)
        return;
    m_selectedSketchId = sketchId;
    if (sketchId > 0)
        m_selectedSketchDocId = workpieceDocumentId();
    else
        m_selectedSketchDocId = kInvalidDocumentId;
    if (auto* state = m_documentRegistry->ensure(m_selectedSketchDocId)) {
        state->presentationState().selectedSketchId = sketchId;
        state->selectionContext() = sketchId > 0
            ? lcnc::cad::selection::CadSelectionResolver::fromProjectExplorerNode(
                  m_selectedSketchDocId,
                  QStringLiteral("__sketch_finished_%1__").arg(sketchId),
                  QString(),
                  {})
            : lcnc::cad::selection::CadSelectionContext{};
    }
    emit sketchSelectionChanged(sketchId);
}

QList<CadModule::FinishedSketchSnapshot>
CadModule::finishedSketchSnapshots(DocumentId docId) const
{
    QList<FinishedSketchSnapshot> snaps;
    if (docId == kInvalidDocumentId)
        docId = workpieceDocumentId();
    auto* manager = m_documentRegistry->sketchManager(docId);
    if (!manager)
        return snaps;
    for (const auto& record : manager->sketches()) {
        FinishedSketchSnapshot snap;
        snap.sketchId = record.id;
        snap.name = record.name;
        snap.visible = record.visible;
        snap.usedByFeature = record.usage == lcnc::cad::SketchUsageState::UsedByFeature;
        for (const auto& element : record.elements) {
            SketchElementSnapshot e;
            e.id = element.id;
            e.kind = static_cast<int>(element.kind);
            e.label = element.label;
            snap.elements.append(e);
        }
        snaps.append(snap);
    }
    return snaps;
}

bool CadModule::setSketchVisible(DocumentId docId, int sketchId, bool visible)
{
    if (docId == kInvalidDocumentId)
        docId = workpieceDocumentId();
    auto* manager = m_documentRegistry->sketchManager(docId);
    if (!manager)
        return false;
    if (!manager->setSketchVisible(sketchId, visible))
        return false;
    emit finishedSketchesChanged(docId);
    return true;
}

bool CadModule::deleteSketch(DocumentId docId, int sketchId)
{
    if (docId == kInvalidDocumentId)
        docId = workpieceDocumentId();
    auto* manager = m_documentRegistry->sketchManager(docId);
    if (!manager)
        return false;
    if (!manager->removeSketch(sketchId))
        return false;
    if (m_selectedSketchId == sketchId) {
        m_selectedSketchId = 0;
        emit sketchSelectionChanged(0);
    }
    emit finishedSketchesChanged(docId);
    return true;
}

void CadModule::setSketchTool(int toolKind)
{
    if (!m_modelingSession)
        return;
    if (!m_modelingSession->isSketchEditing()) {
        LCNC_DEBUG(lcnc::LogCode::Generic,
                   "CadModule::setSketchTool ignored: sketch not editing");
        return;
    }
    const auto kind = static_cast<lcnc::cad::SketchToolKind>(toolKind);
    if (m_modelingSession->sketchTool() == kind)
        return;
    m_modelingSession->setSketchTool(kind);
    emit sketchToolChanged(toolKind);
}

int CadModule::sketchTool() const
{
    if (!m_modelingSession)
        return static_cast<int>(lcnc::cad::SketchToolKind::None);
    return static_cast<int>(m_modelingSession->sketchTool());
}

int CadModule::addSketchElement(int toolKind, const QVector<double>& params, QString* errMsg)
{
    if (!m_modelingSession) {
        if (errMsg)
            // 中文翻译：CAD 建模会话未初始化
            *errMsg = tr("CAD modeling session not initialized");
        return -1;
    }
    QString sessionError;
    const int id = m_modelingSession->addSketchElement(
        static_cast<lcnc::cad::SketchToolKind>(toolKind), params, &sessionError);
    if (id < 0) {
        if (errMsg)
            *errMsg = sessionError;
        return -1;
    }
    emit sketchElementsChanged();
    return id;
}

bool CadModule::removeSketchElement(int elementId)
{
    if (!m_modelingSession)
        return false;
    if (!m_modelingSession->removeSketchElement(elementId))
        return false;
    emit sketchElementsChanged();
    return true;
}

bool CadModule::moveSketchElement(int elementId, double deltaX, double deltaY, QString* errMsg)
{
    if (!m_modelingSession) {
        if (errMsg)
            // 中文翻译：CAD 建模会话未初始化
            *errMsg = tr("CAD modeling session not initialized");
        return false;
    }
    QString sessionError;
    if (!m_modelingSession->moveSketchElement(elementId, deltaX, deltaY, &sessionError)) {
        if (errMsg)
            *errMsg = sessionError;
        return false;
    }
    emit sketchElementsChanged();
    return true;
}

bool CadModule::moveSketchElementHandle(int elementId,
                                        int handleIndex,
                                        double deltaX,
                                        double deltaY,
                                        QString* errMsg)
{
    if (!m_modelingSession) {
        if (errMsg)
            // 中文翻译：CAD 建模会话未初始化
            *errMsg = tr("CAD modeling session not initialized");
        return false;
    }
    QString sessionError;
    if (!m_modelingSession->moveSketchElementHandle(
            elementId, handleIndex, deltaX, deltaY, &sessionError)) {
        if (errMsg)
            *errMsg = sessionError;
        return false;
    }
    emit sketchElementsChanged();
    return true;
}

QList<CadModule::SketchElementSnapshot> CadModule::sketchElementSnapshots() const
{
    QList<SketchElementSnapshot> snapshots;
    if (!m_modelingSession)
        return snapshots;
    for (const auto& element : m_modelingSession->sketchElements()) {
        SketchElementSnapshot snap;
        snap.id = element.id;
        snap.kind = static_cast<int>(element.kind);
        snap.label = element.label;
        snapshots.append(snap);
    }
    return snapshots;
}

QList<CadModule::SketchOverlaySnapshot> CadModule::sketchOverlaySnapshots(DocumentId docId) const
{
    QList<SketchOverlaySnapshot> snapshots;
    if (docId == kInvalidDocumentId)
        docId = workpieceDocumentId();
    if (docId == kInvalidDocumentId)
        return snapshots;

    const auto context = selectionContext(docId);

    if (m_modelingSession && m_modelingSession->isSketchEditing() && docId == workpieceDocumentId()) {
        const int plane = static_cast<int>(m_modelingSession->finishedPlane());
        for (const auto& element : m_modelingSession->sketchElements()) {
            SketchOverlaySnapshot snap;
            snap.key = activeSketchElementKey(element.id);
            snap.elementId = element.id;
            snap.kind = static_cast<int>(element.kind);
            snap.plane = plane;
            snap.params = element.params;
            snap.visible = true;
            snap.selected = selectionContainsSketchElement(context, 0, element.id);
            snap.activeSession = true;
            snap.draggable = true;
            snapshots.append(std::move(snap));

            for (const auto& handle : sketchHandlePoints(element)) {
                SketchOverlaySnapshot handleSnap;
                handleSnap.key = activeSketchHandleKey(element.id, handle.handleIndex);
                handleSnap.elementId = element.id;
                handleSnap.kind = static_cast<int>(lcnc::cad::SketchToolKind::Point);
                handleSnap.plane = plane;
                handleSnap.params = handle.params;
                handleSnap.visible = true;
                handleSnap.selected = snap.selected;
                handleSnap.activeSession = true;
                handleSnap.draggable = true;
                snapshots.append(std::move(handleSnap));
            }
        }
    }

    auto* manager = m_documentRegistry->sketchManager(docId);
    if (!manager)
        return snapshots;

    for (const auto& record : manager->sketches()) {
        const bool sketchSelected = selectionContainsSketch(context, record.id);
        for (const auto& element : record.elements) {
            SketchOverlaySnapshot snap;
            snap.key = finishedSketchElementKey(record.id, element.id);
            snap.sketchId = record.id;
            snap.elementId = element.id;
            snap.kind = static_cast<int>(element.kind);
            snap.plane = static_cast<int>(record.plane);
            snap.params = element.params;
            snap.visible = record.visible;
            snap.selected = sketchSelected || selectionContainsSketchElement(context, record.id, element.id);
            snap.activeSession = false;
            snap.usedByFeature = record.usage == lcnc::cad::SketchUsageState::UsedByFeature;
            snapshots.append(std::move(snap));
        }
    }
    return snapshots;
}

// ── Undo / Redo ────────────────────────────────────────────────────────────────

bool CadModule::canUndo(DocumentId docId) const
{
    LcncDocument* doc = domainDocumentById(docId);
    return doc && doc->canUndo();
}

bool CadModule::canRedo(DocumentId docId) const
{
    LcncDocument* doc = domainDocumentById(docId);
    return doc && doc->canRedo();
}

void CadModule::undo(DocumentId docId)
{
    LcncDocument* doc = domainDocumentById(docId);
    if (!doc || !doc->canUndo()) return;
    doc->undo();
    refreshDisplay(docId);
}

void CadModule::redo(DocumentId docId)
{
    LcncDocument* doc = domainDocumentById(docId);
    if (!doc || !doc->canRedo()) return;
    doc->redo();
    refreshDisplay(docId);
}

// ── Internal ───────────────────────────────────────────────────────────────────

void CadModule::refreshDisplay(DocumentId docId)
{
    auto* gd = activeGuiDocument();
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "CadModule::refreshDisplay docId={} gd={}",
               docId, static_cast<void*>(gd));
    if (gd) {
        lcnc::ProjectDomain domain = lcnc::ProjectDomain::Workpiece;
        lcnc::Kernel::current().projectManager()->domainForDocument(docId, &domain);
        gd->rebuildDomain(domain, domainDocumentById(docId));
        gd->fitAll();
    }
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(docId);
}
