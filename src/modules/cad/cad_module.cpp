#include "modules/cad/cad_module.h"
#include "core/kernel/kernel.h"
#include "modules/cad/services/shape_service.h"

#include "core/document/lcnc_application.h"
#include "core/document/lcnc_document.h"
#include "core/kernel/i_kernel.h"
#include "core/kernel/service_registry.h"
#include "core/logging/logger.h"
#include "core/task/task_manager.h"
#include "core/document/xcaf_utils.h"
#include "view/gui_application.h"
#include "view/gui_document.h"

#include <QFileInfo>

#include <BRep_Builder.hxx>
#include <BRepTools.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <IGESCAFControl_Reader.hxx>
#include <IGESControl_Reader.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <StlAPI_Reader.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TDocStd_Document.hxx>
#include <TDF_LabelSequence.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>

#include <memory>
#include <stdexcept>

namespace {

void appendUniqueEntries(QStringList* target, const QStringList& entries)
{
    if (!target)
        return;

    for (const QString& entry : entries) {
        if (!entry.isEmpty() && !target->contains(entry))
            target->append(entry);
    }
}

CadModule::DocumentTreeNode buildHierarchyNode(const LcncDocument::ShapeTreeNode& sourceNode,
                                               DocumentId docId,
                                               const QString& keyPrefix,
                                               int childIndex)
{
    CadModule::DocumentTreeNode targetNode;
    targetNode.nodeKey = sourceNode.entry.isEmpty()
        ? QStringLiteral("group:%1/%2").arg(keyPrefix).arg(childIndex)
        : QStringLiteral("entry:%1:%2").arg(docId).arg(sourceNode.entry);
    targetNode.entry = sourceNode.entry;
    targetNode.displayName = sourceNode.displayName.isEmpty()
        ? sourceNode.entry
        : sourceNode.displayName;

    if (sourceNode.entry.isEmpty()) {
        for (int index = 0; index < sourceNode.children.size(); ++index) {
            const auto& childSourceNode = sourceNode.children.at(index);
            CadModule::DocumentTreeNode childNode =
                buildHierarchyNode(childSourceNode, docId, targetNode.nodeKey, index);
            appendUniqueEntries(&targetNode.leafEntries, childNode.leafEntries);
            targetNode.children.append(std::move(childNode));
        }
    } else {
        targetNode.leafEntries.append(sourceNode.entry);
    }

    return targetNode;
}

QList<CadModule::DocumentTreeNode> buildFallbackNodes(LcncDocument* doc)
{
    QList<CadModule::DocumentTreeNode> nodes;
    if (!doc)
        return nodes;

    TDF_LabelSequence labels = doc->entityLabels(LcncDocument::EntityKind::Workpiece);
    nodes.reserve(labels.Length());

    for (int i = 1; i <= labels.Length(); ++i) {
        const TDF_Label label = labels.Value(i);
        const QString entry = XcafUtils::entry(label);
        QString displayName = XcafUtils::name(label);
        if (displayName.isEmpty())
            displayName = entry;

        CadModule::DocumentTreeNode node;
        node.nodeKey = QStringLiteral("entry:%1:%2").arg(doc->id()).arg(entry);
        node.displayName = displayName;
        node.entry = entry;
        node.leafEntries.append(entry);
        nodes.append(std::move(node));
    }

    return nodes;
}

int entityCount(LcncDocument* doc, LcncDocument::EntityKind kind)
{
    return doc ? doc->entityLabels(kind).Length() : 0;
}

bool importStepAsSingleShape(const QString& filePath,
                             LcncDocument* doc,
                             LcncDocument::EntityKind kind,
                             const QString& displayName,
                             QString* error)
{
    STEPControl_Reader reader;
    if (reader.ReadFile(filePath.toUtf8().constData()) != IFSelect_RetDone) {
        if (error)
            *error = QObject::tr("无法读取 STEP 文件: %1").arg(filePath);
        return false;
    }

    const Standard_Integer transferred = reader.TransferRoots();
    TopoDS_Shape shape = reader.OneShape();
    if (transferred <= 0 || shape.IsNull()) {
        if (error)
            *error = QObject::tr("STEP 文件未解析出可显示形体: %1").arg(filePath);
        return false;
    }

    doc->addShapeEntity(shape, displayName, kind);
    LCNC_WARN(lcnc::LogCode::Generic,
              "STEP XCAF import produced no entities; used single-shape fallback path={} transferred={}",
              filePath.toStdString(), transferred);
    return true;
}

bool importIgesAsSingleShape(const QString& filePath,
                             LcncDocument* doc,
                             LcncDocument::EntityKind kind,
                             const QString& displayName,
                             QString* error)
{
    IGESControl_Reader reader;
    if (reader.ReadFile(filePath.toUtf8().constData()) != IFSelect_RetDone) {
        if (error)
            *error = QObject::tr("无法读取 IGES 文件: %1").arg(filePath);
        return false;
    }

    const Standard_Integer transferred = reader.TransferRoots();
    TopoDS_Shape shape = reader.OneShape();
    if (transferred <= 0 || shape.IsNull()) {
        if (error)
            *error = QObject::tr("IGES 文件未解析出可显示形体: %1").arg(filePath);
        return false;
    }

    doc->addShapeEntity(shape, displayName, kind);
    LCNC_WARN(lcnc::LogCode::Generic,
              "IGES XCAF import produced no entities; used single-shape fallback path={} transferred={}",
              filePath.toStdString(), transferred);
    return true;
}

bool importStepWithFallback(const QString& filePath,
                            LcncDocument* doc,
                            LcncDocument::EntityKind kind,
                            const QString& displayName,
                            QString* error)
{
    const int beforeCount = entityCount(doc, kind);
    Handle(TDocStd_Document) xdeDoc =
        new TDocStd_Document(TCollection_ExtendedString("BinXCAF"));
    XCAFDoc_DocumentTool::Set(xdeDoc->Main());
    STEPCAFControl_Reader reader;
    reader.SetNameMode(Standard_True);
    if (reader.ReadFile(filePath.toUtf8().constData()) != IFSelect_RetDone) {
        if (error)
            *error = QObject::tr("无法读取 STEP 文件: %1").arg(filePath);
        return false;
    }

    const Standard_Boolean transferOk = reader.Transfer(xdeDoc);
    doc->importFromXcaf(xdeDoc, kind);
    const int importedCount = entityCount(doc, kind) - beforeCount;
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "STEP XCAF transfer path={} ok={} imported={}",
               filePath.toStdString(), static_cast<bool>(transferOk), importedCount);

    if (importedCount > 0)
        return true;

    return importStepAsSingleShape(filePath, doc, kind, displayName, error);
}

bool importIgesWithFallback(const QString& filePath,
                            LcncDocument* doc,
                            LcncDocument::EntityKind kind,
                            const QString& displayName,
                            QString* error)
{
    const int beforeCount = entityCount(doc, kind);
    Handle(TDocStd_Document) xdeDoc =
        new TDocStd_Document(TCollection_ExtendedString("BinXCAF"));
    XCAFDoc_DocumentTool::Set(xdeDoc->Main());
    IGESCAFControl_Reader reader;
    reader.SetNameMode(Standard_True);
    if (reader.ReadFile(filePath.toUtf8().constData()) != IFSelect_RetDone) {
        if (error)
            *error = QObject::tr("无法读取 IGES 文件: %1").arg(filePath);
        return false;
    }

    const Standard_Boolean transferOk = reader.Transfer(xdeDoc);
    doc->importFromXcaf(xdeDoc, kind);
    const int importedCount = entityCount(doc, kind) - beforeCount;
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "IGES XCAF transfer path={} ok={} imported={}",
               filePath.toStdString(), static_cast<bool>(transferOk), importedCount);

    if (importedCount > 0)
        return true;

    return importIgesAsSingleShape(filePath, doc, kind, displayName, error);
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
        lcnc::Kernel::current().app()->documentById(targetDocId) != nullptr) {
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
        QStringLiteral("CAD模块"),
        QStringLiteral("1.0.0"),
        {}                       // 无依赖
    };
}

bool CadModule::init(lcnc::IKernel& kernel)
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "CadModule::init begin");

    // 以 "不拥有" 语义注册为服务：所有权仍由 Kernel 的 ModuleRegistry。
    auto svcPtr = std::shared_ptr<CadModule>(this, [](CadModule*) {});
    kernel.services().registerService<CadModule>(svcPtr);
    // 同时以 Phase 7 门面接口注册，供 UI/命令以抽象类型查找。
    auto facadePtr = std::shared_ptr<lcnc::ICadFacade>(svcPtr, static_cast<lcnc::ICadFacade*>(this));
    kernel.services().registerService<lcnc::ICadFacade>(facadePtr);

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
    m_initialized = false;
    LCNC_INFO(lcnc::LogCode::Generic, "CadModule stop done");
}

CadModule::CadModule(QObject* parent)
    : QObject(parent)
{
    LcncApplication* lcnc = lcnc::Kernel::current().app();

    connect(lcnc, &LcncApplication::documentAdded, this, [this, lcnc](DocumentId id) {
        emit documentListChanged();
        if (!lcnc->isMachineDocument(id))
            emit documentTreeChanged();
    });
    connect(lcnc, &LcncApplication::documentClosed, this, [this, lcnc](DocumentId id) {
        emit documentListChanged();
        if (!lcnc->isMachineDocument(id))
            emit documentTreeChanged();
    });
    connect(lcnc, &LcncApplication::activeDocumentChanged, this, [this](DocumentId id) {
        emit activeDocumentChanged(id);
    });
    connect(lcnc, &LcncApplication::documentModified, this, [this, lcnc](DocumentId id) {
        emit documentModified(id);
        if (!lcnc->isMachineDocument(id))
            emit documentTreeChanged();
    });
}

// ── Document Management ────────────────────────────────────────────────────────

DocumentId CadModule::newDocument(const QString& name)
{
    LcncDocument* doc = lcnc::Kernel::current().app()->newDocument(name);
    return doc ? doc->id() : kInvalidDocumentId;
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
        emit operationFailed(tr("打开失败"), tr("文件不存在: %1").arg(filePath));
        return kInvalidDocumentId;
    }

    const QString ext = fileInfo.suffix().toLower();
    if (ext != "stp" && ext != "step" &&
        ext != "igs" && ext != "iges" &&
        ext != "stl" && ext != "brep") {
        LCNC_WARN(lcnc::LogCode::Generic,
                  "CadModule::openDocument unsupported suffix path={} suffix={}",
                  filePath.toStdString(), fileInfo.suffix().toStdString());
        emit operationFailed(tr("打开失败"), tr("暂不支持的文件格式: %1").arg(fileInfo.suffix()));
        return kInvalidDocumentId;
    }

    const DocumentId docId = newDocument(fileInfo.baseName());
    LcncDocument* doc = lcnc::Kernel::current().app()->documentById(docId);
    if (!doc) {
        LCNC_WARN(lcnc::LogCode::Generic,
                  "CadModule::openDocument failed to create document path={}",
                  filePath.toStdString());
        emit operationFailed(tr("打开失败"), tr("无法创建目标文档"));
        return kInvalidDocumentId;
    }

    doc->setFilePath(filePath);
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "CadModule::openDocument created docId={} ext={} path={}",
               docId, ext.toStdString(), filePath.toStdString());

    auto error = std::make_shared<QString>();
    const TaskId taskId = lcnc::Kernel::current().taskManager()->run(
        tr("打开: %1").arg(fileInfo.fileName()),
        [filePath, ext, doc, error](TaskProgress* prog) {
            LCNC_DEBUG(lcnc::LogCode::Generic,
                       "CadModule::openDocument worker begin docId={} ext={} path={}",
                       doc ? doc->id() : kInvalidDocumentId,
                       ext.toStdString(),
                       filePath.toStdString());
            prog->setRange(0, 100);

            if (ext == "stp" || ext == "step") {
                prog->setStepName(QStringLiteral("读取 STEP..."));
                prog->setValue(50);
                prog->setStepName(QStringLiteral("转换形体..."));
                if (!importStepWithFallback(filePath,
                                            doc,
                                            LcncDocument::EntityKind::Workpiece,
                                            QFileInfo(filePath).baseName(),
                                            error.get())) {
                    throw std::runtime_error("step open failed");
                }
            } else if (ext == "igs" || ext == "iges") {
                prog->setStepName(QStringLiteral("读取 IGES..."));
                prog->setValue(50);
                prog->setStepName(QStringLiteral("转换形体..."));
                if (!importIgesWithFallback(filePath,
                                            doc,
                                            LcncDocument::EntityKind::Workpiece,
                                            QFileInfo(filePath).baseName(),
                                            error.get())) {
                    throw std::runtime_error("iges open failed");
                }
            } else if (ext == "stl") {
                prog->setStepName(QStringLiteral("读取 STL..."));
                TopoDS_Shape shape;
                StlAPI_Reader reader;
                reader.Read(shape, filePath.toUtf8().constData());
                if (shape.IsNull()) {
                    *error = QObject::tr("无法读取 STL 文件: %1").arg(filePath);
                    throw std::runtime_error("stl open failed");
                }

                prog->setValue(80);
                doc->addShapeEntity(shape, QFileInfo(filePath).baseName(),
                                    LcncDocument::EntityKind::Workpiece);
            } else if (ext == "brep") {
                prog->setStepName(QStringLiteral("读取 BREP..."));
                TopoDS_Shape shape;
                BRep_Builder builder;
                BRepTools::Read(shape, filePath.toUtf8().constData(), builder);
                if (shape.IsNull()) {
                    *error = QObject::tr("无法读取 BREP 文件: %1").arg(filePath);
                    throw std::runtime_error("brep open failed");
                }

                doc->addShapeEntity(shape, QFileInfo(filePath).baseName(),
                                    LcncDocument::EntityKind::Workpiece);
            }

            LCNC_DEBUG(lcnc::LogCode::Generic,
                       "CadModule::openDocument worker done docId={} workpieceCount={}",
                       doc ? doc->id() : kInvalidDocumentId,
                       entityCount(doc, LcncDocument::EntityKind::Workpiece));
            prog->setValue(100);
        });

    LCNC_DEBUG(lcnc::LogCode::Generic,
               "CadModule::openDocument task scheduled docId={} taskId={}",
               docId, taskId);

    watchTask(this, taskId, [this, docId, error](bool success) {
        LCNC_DEBUG(lcnc::LogCode::Generic,
                   "CadModule::openDocument task done docId={} success={}",
                   docId, success);
        if (!success) {
            closeDocument(docId);
            emit operationFailed(tr("打开失败"),
                                 error->isEmpty() ? tr("打开文件失败") : *error);
            return;
        }

        refreshDisplay(docId);
    });

    return docId;
}

DocumentId CadModule::importStep(const QString& filePath, DocumentId targetDocId)
{
    QFileInfo fileInfo(filePath);
    if (filePath.isEmpty() || !fileInfo.exists()) {
        emit operationFailed(tr("导入 STEP 失败"), tr("文件不存在: %1").arg(filePath));
        return kInvalidDocumentId;
    }

    bool createdNew = false;
    const DocumentId docId = ensureTargetDocument(this, targetDocId, fileInfo.baseName(), &createdNew);
    LcncDocument* doc = lcnc::Kernel::current().app()->documentById(docId);
    if (!doc) {
        emit operationFailed(tr("导入 STEP 失败"), tr("无法创建目标文档"));
        return kInvalidDocumentId;
    }

    auto error = std::make_shared<QString>();
    const TaskId taskId = lcnc::Kernel::current().taskManager()->run(
        tr("导入 STEP: %1").arg(fileInfo.fileName()),
        [filePath, doc, error](TaskProgress* prog) {
            prog->setRange(0, 100);
            prog->setStepName(QStringLiteral("读取 STEP..."));

            prog->setValue(50);
            prog->setStepName(QStringLiteral("转换形体..."));
            if (!importStepWithFallback(filePath,
                                        doc,
                                        LcncDocument::EntityKind::Workpiece,
                                        QFileInfo(filePath).baseName(),
                                        error.get())) {
                throw std::runtime_error("step import failed");
            }
            prog->setValue(100);
        });

    watchTask(this, taskId, [this, docId, createdNew, error](bool success) {
        if (!success) {
            if (createdNew)
                closeDocument(docId);
            emit operationFailed(tr("导入 STEP 失败"),
                                 error->isEmpty() ? tr("导入 STEP 失败") : *error);
            return;
        }

        refreshDisplay(docId);
    });

    return docId;
}

DocumentId CadModule::importStl(const QString& filePath, DocumentId targetDocId)
{
    QFileInfo fileInfo(filePath);
    if (filePath.isEmpty() || !fileInfo.exists()) {
        emit operationFailed(tr("导入 STL 失败"), tr("文件不存在: %1").arg(filePath));
        return kInvalidDocumentId;
    }

    bool createdNew = false;
    const DocumentId docId = ensureTargetDocument(this, targetDocId, fileInfo.baseName(), &createdNew);
    LcncDocument* doc = lcnc::Kernel::current().app()->documentById(docId);
    if (!doc) {
        emit operationFailed(tr("导入 STL 失败"), tr("无法创建目标文档"));
        return kInvalidDocumentId;
    }

    auto error = std::make_shared<QString>();
    const TaskId taskId = lcnc::Kernel::current().taskManager()->run(
        tr("导入 STL: %1").arg(fileInfo.fileName()),
        [filePath, doc, error](TaskProgress* prog) {
            prog->setRange(0, 100);
            prog->setStepName(QStringLiteral("读取 STL..."));

            TopoDS_Shape shape;
            StlAPI_Reader reader;
            reader.Read(shape, filePath.toUtf8().constData());
            if (shape.IsNull()) {
                *error = QObject::tr("无法读取 STL 文件: %1").arg(filePath);
                throw std::runtime_error("stl import failed");
            }

            prog->setValue(80);
            doc->addShapeEntity(shape, QFileInfo(filePath).baseName(),
                                LcncDocument::EntityKind::Workpiece);
            prog->setValue(100);
        });

    watchTask(this, taskId, [this, docId, createdNew, error](bool success) {
        if (!success) {
            if (createdNew)
                closeDocument(docId);
            emit operationFailed(tr("导入 STL 失败"),
                                 error->isEmpty() ? tr("导入 STL 失败") : *error);
            return;
        }

        refreshDisplay(docId);
    });

    return docId;
}

bool CadModule::saveDocument(DocumentId id, const QString& path)
{
    LcncDocument* doc = lcnc::Kernel::current().app()->documentById(id);
    if (!doc) {
        emit operationFailed(tr("保存失败"), tr("找不到目标文档"));
        return false;
    }
    const QString target = path.isEmpty() ? doc->filePath() : path;
    if (target.isEmpty()) {
        emit operationFailed(tr("保存失败"), tr("未指定保存路径"));
        return false;
    }
    QString err;
    const bool ok = lcnc::Kernel::current().app()->saveDocument(id, target, &err);
    if (!ok)
        emit operationFailed(tr("保存失败"), err.isEmpty() ? tr("保存文档失败") : err);
    return ok;
}

void CadModule::exportStep(DocumentId id, const QString& filePath)
{
    LcncDocument* doc = lcnc::Kernel::current().app()->documentById(id);
    if (!doc) {
        emit operationFailed(tr("导出 STEP 失败"), tr("找不到目标文档"));
        return;
    }
    if (filePath.isEmpty()) {
        emit operationFailed(tr("导出 STEP 失败"), tr("未指定导出路径"));
        return;
    }

    auto error = std::make_shared<QString>();
    const TaskId taskId = lcnc::Kernel::current().taskManager()->run(
        tr("导出 STEP: %1").arg(QFileInfo(filePath).fileName()),
        [doc, filePath, error](TaskProgress* prog) {
            prog->setRange(0, 100);
            prog->setStepName(QStringLiteral("写入 STEP..."));

            Handle(XCAFDoc_ShapeTool) st = doc->shapeTool();
            TDF_LabelSequence shapes;
            st->GetFreeShapes(shapes);

            STEPControl_Writer writer;
            for (int i = 1; i <= shapes.Length(); ++i) {
                TopoDS_Shape sh = st->GetShape(shapes.Value(i));
                if (!sh.IsNull())
                    writer.Transfer(sh, STEPControl_AsIs);
            }

            if (writer.Write(filePath.toUtf8().constData()) != IFSelect_RetDone) {
                *error = QObject::tr("导出 STEP 失败: %1").arg(filePath);
                throw std::runtime_error("step export failed");
            }

            prog->setValue(100);
        });

    watchTask(this, taskId, [this, error](bool success) {
        if (!success) {
            emit operationFailed(tr("导出 STEP 失败"),
                                 error->isEmpty() ? tr("导出 STEP 失败") : *error);
        }
    });
}

void CadModule::closeDocument(DocumentId id)
{
    lcnc::Kernel::current().app()->closeDocument(id);
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

// ── Active Document ────────────────────────────────────────────────────────────

DocumentId CadModule::activeDocumentId() const
{
    return lcnc::Kernel::current().app()->activeDocumentId();
}

LcncDocument* CadModule::activeDocument() const
{
    return lcnc::Kernel::current().app()->activeDocument();
}

GuiDocument* CadModule::activeGuiDocument() const
{
    return lcnc::Kernel::current().guiApp()->activeGuiDocument();
}

LcncDocument* CadModule::documentById(DocumentId id) const
{
    return lcnc::Kernel::current().app()->documentById(id);
}

GuiDocument* CadModule::guiDocument(DocumentId id) const
{
    return lcnc::Kernel::current().guiApp()->guiDocument(id);
}

QList<LcncDocument*> CadModule::workpieceDocuments() const
{
    return lcnc::Kernel::current().app()->workpieceDocuments();
}

QList<CadModule::DocumentTreeDocument> CadModule::documentTreeDocuments() const
{
    QList<DocumentTreeDocument> documents;
    const QList<LcncDocument*> docs = workpieceDocuments();
    documents.reserve(docs.size());

    for (LcncDocument* doc : docs) {
        if (!doc)
            continue;

        DocumentTreeDocument documentNode;
        documentNode.documentId = doc->id();
        documentNode.nodeKey = QStringLiteral("doc:%1").arg(doc->id());
        documentNode.displayName = doc->name();

        const auto& hierarchy = doc->entityTree(LcncDocument::EntityKind::Workpiece);
        if (!hierarchy.isEmpty()) {
            documentNode.children.reserve(hierarchy.size());
            for (int index = 0; index < hierarchy.size(); ++index) {
                DocumentTreeNode childNode =
                    buildHierarchyNode(hierarchy.at(index), doc->id(), documentNode.nodeKey, index);
                appendUniqueEntries(&documentNode.leafEntries, childNode.leafEntries);
                documentNode.children.append(std::move(childNode));
            }
        } else {
            documentNode.children = buildFallbackNodes(doc);
            for (const auto& childNode : documentNode.children)
                appendUniqueEntries(&documentNode.leafEntries, childNode.leafEntries);
        }

        documents.append(std::move(documentNode));
    }

    return documents;
}

void CadModule::setActiveDocument(DocumentId id)
{
    lcnc::Kernel::current().app()->setActiveDocument(id);
}

void CadModule::requestWorkpieceView(DocumentId id)
{
    if (id == kInvalidDocumentId)
        id = activeDocumentId();
    emit workpieceViewRequested(id);
}

void CadModule::setEntityVisible(DocumentId docId, const QString& entry, bool visible)
{
    if (entry.isEmpty())
        return;

    if (auto* gd = guiDocument(docId)) {
        Handle(AIS_Shape) ais = gd->aisShape(entry);
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
    GuiDocument* gd = guiDocument(docId);
    if (!gd)
        return;

    const Handle(AIS_InteractiveContext)& ctx = gd->context();
    if (ctx.IsNull())
        return;

    ctx->ClearSelected(false);
    for (const QString& entry : entries) {
        Handle(AIS_Shape) ais = gd->aisShape(entry);
        if (!ais.IsNull())
            ctx->AddOrRemoveSelected(ais, false);
    }

    if (gd->hasView())
        gd->view()->Redraw();

    emit selectionChanged(docId, gd->selectedEntries());
}

QStringList CadModule::selectedEntries(DocumentId docId) const
{
    if (auto* gd = guiDocument(docId))
        return gd->selectedEntries();

    return {};
}

void CadModule::syncSelectionFromView(DocumentId docId)
{
    if (docId == kInvalidDocumentId)
        docId = activeDocumentId();

    emit selectionChanged(docId, selectedEntries(docId));
}

// ── Modeling Operations ────────────────────────────────────────────────────────

bool CadModule::moveShape(DocumentId docId, const TDF_Label& label, const gp_Vec& translation)
{
    return moveShapes(docId, { label }, translation);
}

bool CadModule::moveShapes(DocumentId docId, const QList<TDF_Label>& labels,
                           const gp_Vec& translation)
{
    LcncDocument* doc = lcnc::Kernel::current().app()->documentById(docId);
    if (!doc || labels.isEmpty())
        return false;

    doc->openCommand(tr("移动形体"));
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
    LcncDocument* doc = lcnc::Kernel::current().app()->documentById(docId);
    if (!doc || labels.isEmpty())
        return false;

    doc->openCommand(tr("旋转形体"));
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
    LcncDocument* doc = lcnc::Kernel::current().app()->documentById(docId);
    if (!doc || entries.isEmpty())
        return false;

    doc->openCommand(tr("删除形体"));
    if (auto* gd = lcnc::Kernel::current().guiApp()->guiDocument(docId)) {
        for (const QString& entry : entries)
            gd->eraseEntity(entry);
    }

    for (const QString& entry : entries)
        ShapeService::deleteShape(doc, entry);

    doc->commitCommand();
    refreshDisplay(docId);
    return true;
}

int CadModule::explodeShape(DocumentId docId, const TDF_Label& label, int entityKind)
{
    LcncDocument* doc = lcnc::Kernel::current().app()->documentById(docId);
    if (!doc) return 0;

    doc->openCommand(tr("拆解形体"));

    // Erase original AIS object
    const QString entry = XcafUtils::entry(label);
    if (auto* gd = lcnc::Kernel::current().guiApp()->guiDocument(docId))
        gd->eraseEntity(entry);

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
    LcncDocument* doc = lcnc::Kernel::current().app()->documentById(docId);
    if (!doc) return TDF_Label();

    doc->openCommand(tr("创建形体"));
    TDF_Label label = ShapeService::addShape(doc, shape, name, entityKind);
    if (label.IsNull()) {
        doc->abortCommand();
        return TDF_Label();
    }

    doc->commitCommand();
    refreshDisplay(docId);
    return label;
}

// ── Undo / Redo ────────────────────────────────────────────────────────────────

bool CadModule::canUndo(DocumentId docId) const
{
    LcncDocument* doc = lcnc::Kernel::current().app()->documentById(docId);
    return doc && doc->canUndo();
}

bool CadModule::canRedo(DocumentId docId) const
{
    LcncDocument* doc = lcnc::Kernel::current().app()->documentById(docId);
    return doc && doc->canRedo();
}

void CadModule::undo(DocumentId docId)
{
    LcncDocument* doc = lcnc::Kernel::current().app()->documentById(docId);
    if (!doc || !doc->canUndo()) return;
    doc->undo();
    refreshDisplay(docId);
}

void CadModule::redo(DocumentId docId)
{
    LcncDocument* doc = lcnc::Kernel::current().app()->documentById(docId);
    if (!doc || !doc->canRedo()) return;
    doc->redo();
    refreshDisplay(docId);
}

// ── Internal ───────────────────────────────────────────────────────────────────

void CadModule::refreshDisplay(DocumentId docId)
{
    auto* gd = lcnc::Kernel::current().guiApp()->guiDocument(docId);
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "CadModule::refreshDisplay docId={} gd={}",
               docId, static_cast<void*>(gd));
    if (gd) {
        gd->rebuildDisplay();
        gd->fitAll();
    }
    lcnc::Kernel::current().app()->notifyDocumentModified(docId);
}
