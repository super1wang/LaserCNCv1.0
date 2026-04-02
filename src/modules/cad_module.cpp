#include "modules/cad_module.h"
#include "modules/shape_service.h"

#include "base/lcnc_application.h"
#include "base/lcnc_document.h"
#include "base/task_manager.h"
#include "base/xcaf_utils.h"
#include "gui/gui_application.h"
#include "gui/gui_document.h"

#include <QFileInfo>

#include <BRep_Builder.hxx>
#include <BRepTools.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <IGESCAFControl_Reader.hxx>
#include <STEPCAFControl_Reader.hxx>
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

void watchTask(QObject* owner, TaskId taskId, std::function<void(bool)> onFinished)
{
    auto connection = std::make_shared<QMetaObject::Connection>();
    *connection = QObject::connect(
        TaskManager::instance(),
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
        LcncApplication::instance()->documentById(targetDocId) != nullptr) {
        return targetDocId;
    }

    if (createdNew)
        *createdNew = true;
    return module->newDocument(defaultName);
}

} // namespace

CadModule* CadModule::s_instance = nullptr;

CadModule* CadModule::instance()
{
    if (!s_instance)
        s_instance = new CadModule();
    return s_instance;
}

CadModule::CadModule(QObject* parent)
    : QObject(parent)
{
    LcncApplication* lcnc = LcncApplication::instance();

    connect(lcnc, &LcncApplication::documentAdded, this, [this](DocumentId) {
        emit documentListChanged();
    });
    connect(lcnc, &LcncApplication::documentClosed, this, [this](DocumentId) {
        emit documentListChanged();
    });
    connect(lcnc, &LcncApplication::activeDocumentChanged, this, [this](DocumentId id) {
        emit activeDocumentChanged(id);
    });
    connect(lcnc, &LcncApplication::documentModified, this, [this](DocumentId id) {
        emit documentModified(id);
    });
}

// ── Document Management ────────────────────────────────────────────────────────

DocumentId CadModule::newDocument(const QString& name)
{
    LcncDocument* doc = LcncApplication::instance()->newDocument(name);
    return doc ? doc->id() : kInvalidDocumentId;
}

DocumentId CadModule::openDocument(const QString& filePath)
{
    QFileInfo fileInfo(filePath);
    if (filePath.isEmpty() || !fileInfo.exists()) {
        emit operationFailed(tr("打开失败"), tr("文件不存在: %1").arg(filePath));
        return kInvalidDocumentId;
    }

    const QString ext = fileInfo.suffix().toLower();
    if (ext != "stp" && ext != "step" &&
        ext != "igs" && ext != "iges" &&
        ext != "stl" && ext != "brep") {
        emit operationFailed(tr("打开失败"), tr("暂不支持的文件格式: %1").arg(fileInfo.suffix()));
        return kInvalidDocumentId;
    }

    const DocumentId docId = newDocument(fileInfo.baseName());
    LcncDocument* doc = LcncApplication::instance()->documentById(docId);
    if (!doc) {
        emit operationFailed(tr("打开失败"), tr("无法创建目标文档"));
        return kInvalidDocumentId;
    }

    doc->setFilePath(filePath);

    auto error = std::make_shared<QString>();
    const TaskId taskId = TaskManager::instance()->run(
        tr("打开: %1").arg(fileInfo.fileName()),
        [filePath, ext, doc, error](TaskProgress* prog) {
            prog->setRange(0, 100);

            if (ext == "stp" || ext == "step") {
                prog->setStepName(QStringLiteral("读取 STEP..."));
                Handle(TDocStd_Document) xdeDoc =
                    new TDocStd_Document(TCollection_ExtendedString("BinXCAF"));
                XCAFDoc_DocumentTool::Set(xdeDoc->Main());
                STEPCAFControl_Reader reader;
                reader.SetNameMode(Standard_True);
                if (reader.ReadFile(filePath.toUtf8().constData()) != IFSelect_RetDone) {
                    *error = QObject::tr("无法读取 STEP 文件: %1").arg(filePath);
                    throw std::runtime_error("step open failed");
                }

                prog->setValue(50);
                prog->setStepName(QStringLiteral("转换形体..."));
                reader.Transfer(xdeDoc);
                doc->importFromXcaf(xdeDoc, LcncDocument::EntityKind::Workpiece);
            } else if (ext == "igs" || ext == "iges") {
                prog->setStepName(QStringLiteral("读取 IGES..."));
                Handle(TDocStd_Document) xdeDoc =
                    new TDocStd_Document(TCollection_ExtendedString("BinXCAF"));
                XCAFDoc_DocumentTool::Set(xdeDoc->Main());
                IGESCAFControl_Reader reader;
                reader.SetNameMode(Standard_True);
                if (reader.ReadFile(filePath.toUtf8().constData()) != IFSelect_RetDone) {
                    *error = QObject::tr("无法读取 IGES 文件: %1").arg(filePath);
                    throw std::runtime_error("iges open failed");
                }

                prog->setValue(50);
                prog->setStepName(QStringLiteral("转换形体..."));
                reader.Transfer(xdeDoc);
                doc->importFromXcaf(xdeDoc, LcncDocument::EntityKind::Workpiece);
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

            prog->setValue(100);
        });

    watchTask(this, taskId, [this, docId, error](bool success) {
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
    LcncDocument* doc = LcncApplication::instance()->documentById(docId);
    if (!doc) {
        emit operationFailed(tr("导入 STEP 失败"), tr("无法创建目标文档"));
        return kInvalidDocumentId;
    }

    auto error = std::make_shared<QString>();
    const TaskId taskId = TaskManager::instance()->run(
        tr("导入 STEP: %1").arg(fileInfo.fileName()),
        [filePath, doc, error](TaskProgress* prog) {
            prog->setRange(0, 100);
            prog->setStepName(QStringLiteral("读取 STEP..."));

            Handle(TDocStd_Document) xdeDoc =
                new TDocStd_Document(TCollection_ExtendedString("BinXCAF"));
            XCAFDoc_DocumentTool::Set(xdeDoc->Main());
            STEPCAFControl_Reader reader;
            reader.SetNameMode(Standard_True);
            if (reader.ReadFile(filePath.toUtf8().constData()) != IFSelect_RetDone) {
                *error = QObject::tr("无法读取 STEP 文件: %1").arg(filePath);
                throw std::runtime_error("step import failed");
            }

            prog->setValue(50);
            prog->setStepName(QStringLiteral("转换形体..."));
            reader.Transfer(xdeDoc);
            doc->importFromXcaf(xdeDoc, LcncDocument::EntityKind::Workpiece);
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
    LcncDocument* doc = LcncApplication::instance()->documentById(docId);
    if (!doc) {
        emit operationFailed(tr("导入 STL 失败"), tr("无法创建目标文档"));
        return kInvalidDocumentId;
    }

    auto error = std::make_shared<QString>();
    const TaskId taskId = TaskManager::instance()->run(
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
    LcncDocument* doc = LcncApplication::instance()->documentById(id);
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
    const bool ok = LcncApplication::instance()->saveDocument(id, target, &err);
    if (!ok)
        emit operationFailed(tr("保存失败"), err.isEmpty() ? tr("保存文档失败") : err);
    return ok;
}

void CadModule::exportStep(DocumentId id, const QString& filePath)
{
    LcncDocument* doc = LcncApplication::instance()->documentById(id);
    if (!doc) {
        emit operationFailed(tr("导出 STEP 失败"), tr("找不到目标文档"));
        return;
    }
    if (filePath.isEmpty()) {
        emit operationFailed(tr("导出 STEP 失败"), tr("未指定导出路径"));
        return;
    }

    auto error = std::make_shared<QString>();
    const TaskId taskId = TaskManager::instance()->run(
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
    LcncApplication::instance()->closeDocument(id);
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
    return LcncApplication::instance()->activeDocumentId();
}

LcncDocument* CadModule::activeDocument() const
{
    return LcncApplication::instance()->activeDocument();
}

GuiDocument* CadModule::activeGuiDocument() const
{
    return GuiApplication::instance()->activeGuiDocument();
}

void CadModule::setActiveDocument(DocumentId id)
{
    LcncApplication::instance()->setActiveDocument(id);
}

// ── Modeling Operations ────────────────────────────────────────────────────────

bool CadModule::moveShape(DocumentId docId, const TDF_Label& label, const gp_Vec& translation)
{
    return moveShapes(docId, { label }, translation);
}

bool CadModule::moveShapes(DocumentId docId, const QList<TDF_Label>& labels,
                           const gp_Vec& translation)
{
    LcncDocument* doc = LcncApplication::instance()->documentById(docId);
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
    LcncDocument* doc = LcncApplication::instance()->documentById(docId);
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
    LcncDocument* doc = LcncApplication::instance()->documentById(docId);
    if (!doc || entries.isEmpty())
        return false;

    doc->openCommand(tr("删除形体"));
    if (auto* gd = GuiApplication::instance()->guiDocument(docId)) {
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
    LcncDocument* doc = LcncApplication::instance()->documentById(docId);
    if (!doc) return 0;

    doc->openCommand(tr("拆解形体"));

    // Erase original AIS object
    const QString entry = XcafUtils::entry(label);
    if (auto* gd = GuiApplication::instance()->guiDocument(docId))
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
    LcncDocument* doc = LcncApplication::instance()->documentById(docId);
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
    LcncDocument* doc = LcncApplication::instance()->documentById(docId);
    return doc && doc->canUndo();
}

bool CadModule::canRedo(DocumentId docId) const
{
    LcncDocument* doc = LcncApplication::instance()->documentById(docId);
    return doc && doc->canRedo();
}

void CadModule::undo(DocumentId docId)
{
    LcncDocument* doc = LcncApplication::instance()->documentById(docId);
    if (!doc || !doc->canUndo()) return;
    doc->undo();
    refreshDisplay(docId);
}

void CadModule::redo(DocumentId docId)
{
    LcncDocument* doc = LcncApplication::instance()->documentById(docId);
    if (!doc || !doc->canRedo()) return;
    doc->redo();
    refreshDisplay(docId);
}

// ── Internal ───────────────────────────────────────────────────────────────────

void CadModule::refreshDisplay(DocumentId docId)
{
    if (auto* gd = GuiApplication::instance()->guiDocument(docId)) {
        gd->rebuildDisplay();
        gd->fitAll();
    }
    LcncApplication::instance()->notifyDocumentModified(docId);
}
