#include "app/commands_file.h"

#include <QAction>
#include <QFileDialog>
#include <QMessageBox>
#include <QStyle>
#include <QApplication>

#include "base/lcnc_application.h"
#include "base/lcnc_document.h"
#include "base/task_manager.h"
#include "gui/gui_application.h"
#include "gui/gui_document.h"

// OCC data exchange
#include <BRep_Builder.hxx>
#include <BRepTools.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <IGESControl_Reader.hxx>
#include <StlAPI_Reader.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_DocumentTool.hxx>

// ── CmdNewDocument ─────────────────────────────────────────────────────────────
CmdNewDocument::CmdNewDocument(IAppContext* ctx)
    : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/new_doc.svg"), tr("新建"), this);
    a->setShortcut(QKeySequence::New);
    a->setStatusTip(tr("新建空白文档"));
    setAction(a);
}

void CmdNewDocument::execute()
{
    app()->newDocument();
    context()->updateCommandStates();
}

// ── CmdOpenDocument ────────────────────────────────────────────────────────────
CmdOpenDocument::CmdOpenDocument(IAppContext* ctx)
    : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/open.svg"), tr("打开"), this);
    a->setShortcut(QKeySequence::Open);
    a->setStatusTip(tr("打开 XCAF / STEP / IGES 文件"));
    setAction(a);
}

void CmdOpenDocument::execute()
{
    const QString path = QFileDialog::getOpenFileName(
        nullptr, tr("打开文件"), QString(),
        tr("所有支持格式 (*.stp *.step *.igs *.iges *.brep *.xcaf);;"
           "STEP (*.stp *.step);;"
           "IGES (*.igs *.iges);;"
           "BREP (*.brep);;"
           "XCAF (*.xcaf)"));
    if (path.isEmpty()) return;

    QString err;
    LcncDocument* doc = app()->openDocument(path, &err);
    if (!doc) {
        QMessageBox::critical(nullptr, tr("打开失败"), err);
        return;
    }

    // Trigger GUI display rebuild
    if (GuiDocument* gd = guiApp()->guiDocument(doc->id()))
        gd->rebuildDisplay();

    context()->updateCommandStates();
}

// ── CmdSaveDocument ────────────────────────────────────────────────────────────
CmdSaveDocument::CmdSaveDocument(IAppContext* ctx)
    : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/save.svg"), tr("保存"), this);
    a->setShortcut(QKeySequence::Save);
    a->setStatusTip(tr("保存当前文档"));
    setAction(a);
}

bool CmdSaveDocument::isEnabled() const
{
    return context()->activeDocument() != nullptr;
}

void CmdSaveDocument::execute()
{
    LcncDocument* doc = context()->activeDocument();
    if (!doc) return;
    if (doc->filePath().isEmpty()) {
        // Delegate to Save As
        CmdSaveDocumentAs saveAs(context());
        saveAs.execute();
        return;
    }
    QString err;
    if (!app()->saveDocument(doc->id(), doc->filePath(), &err))
        QMessageBox::critical(nullptr, tr("保存失败"), err);
}

// ── CmdSaveDocumentAs ──────────────────────────────────────────────────────────
CmdSaveDocumentAs::CmdSaveDocumentAs(IAppContext* ctx)
    : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/save_as.svg"), tr("另存为"), this);
    a->setShortcut(QKeySequence::SaveAs);
    setAction(a);
}

bool CmdSaveDocumentAs::isEnabled() const
{
    return context()->activeDocument() != nullptr;
}

void CmdSaveDocumentAs::execute()
{
    LcncDocument* doc = context()->activeDocument();
    if (!doc) return;
    const QString path = QFileDialog::getSaveFileName(
        nullptr, tr("另存为"), doc->name(),
        tr("XCAF 二进制 (*.xcaf);;XCAF XML (*.xml)"));
    if (path.isEmpty()) return;
    QString err;
    if (!app()->saveDocument(doc->id(), path, &err))
        QMessageBox::critical(nullptr, tr("保存失败"), err);
}

// ── CmdImportStep ──────────────────────────────────────────────────────────────
CmdImportStep::CmdImportStep(IAppContext* ctx)
    : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/import.svg"), tr("导入STEP"), this);
    a->setStatusTip(tr("导入 STEP 文件到当前文档"));
    setAction(a);
}

void CmdImportStep::execute()
{
    const QString path = QFileDialog::getOpenFileName(
        nullptr, tr("导入 STEP"), QString(),
        tr("STEP 文件 (*.stp *.step)"));
    if (path.isEmpty()) return;

    // Ensure there is an active document
    LcncDocument* doc = context()->activeDocument();
    if (!doc) doc = app()->newDocument();

    TaskId taskId = taskMgr()->run(tr("导入 STEP: %1").arg(path),
        [path, doc](TaskProgress* prog) {
            prog->setStepName(QStringLiteral("读取 STEP..."));
            prog->setRange(0, 100);

            STEPControl_Reader reader;
            IFSelect_ReturnStatus status =
                reader.ReadFile(path.toUtf8().constData());
            if (status != IFSelect_RetDone) return;

            prog->setValue(50);
            prog->setStepName(QStringLiteral("转换形体..."));
            reader.TransferRoots();

            for (int i = 1; i <= reader.NbShapes(); ++i) {
                TopoDS_Shape sh = reader.Shape(i);
                if (!sh.IsNull()) {
                    QString name = QStringLiteral("Shape_%1").arg(i);
                    doc->addShapeEntity(sh, name, LcncDocument::EntityKind::Workpiece);
                }
            }
            prog->setValue(100);
        });

    // Refresh display/tree after import finishes
    connect(TaskManager::instance(), &TaskManager::taskFinished,
            this, [this, doc, taskId](TaskId id, bool ok) {
                if (id != taskId) return;
                if (ok && guiApp()->guiDocument(doc->id()))
                    guiApp()->guiDocument(doc->id())->rebuildDisplay();
                if (ok)
                    app()->notifyDocumentModified(doc->id());
            });
}

// ── CmdImportStl ───────────────────────────────────────────────────────────────
CmdImportStl::CmdImportStl(IAppContext* ctx)
    : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/import.svg"), tr("导入STL"), this);
    a->setStatusTip(tr("导入 STL 文件（机台/工件模型）"));
    setAction(a);
}

void CmdImportStl::execute()
{
    const QString path = QFileDialog::getOpenFileName(
        nullptr, tr("导入 STL"), QString(),
        tr("STL 文件 (*.stl)"));
    if (path.isEmpty()) return;

    LcncDocument* doc = context()->activeDocument();
    if (!doc) doc = app()->newDocument();

    TaskId taskId = taskMgr()->run(tr("导入 STL: %1").arg(path),
        [path, doc](TaskProgress* prog) {
            prog->setStepName(QStringLiteral("读取 STL..."));
            prog->setRange(0, 100);

            BRep_Builder builder;
            TopoDS_Shape shape;
            StlAPI_Reader reader;
            reader.Read(shape, path.toUtf8().constData());

            prog->setValue(80);
            if (!shape.IsNull()) {
                QString name = QFileInfo(path).baseName();
                doc->addShapeEntity(shape, name, LcncDocument::EntityKind::Workpiece);
            }
            prog->setValue(100);
        });

    connect(TaskManager::instance(), &TaskManager::taskFinished,
            this, [this, doc, taskId](TaskId id, bool ok) {
                if (id != taskId) return;
                if (ok && guiApp()->guiDocument(doc->id()))
                    guiApp()->guiDocument(doc->id())->rebuildDisplay();
                if (ok)
                    app()->notifyDocumentModified(doc->id());
            });
}

// ── CmdExportStep ──────────────────────────────────────────────────────────────
CmdExportStep::CmdExportStep(IAppContext* ctx)
    : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/export.svg"), tr("导出STEP"), this);
    setAction(a);
}

bool CmdExportStep::isEnabled() const
{
    return context()->activeDocument() != nullptr;
}

void CmdExportStep::execute()
{
    const QString path = QFileDialog::getSaveFileName(
        nullptr, tr("导出 STEP"), QString(),
        tr("STEP 文件 (*.stp *.step)"));
    if (path.isEmpty()) return;

    LcncDocument* doc = context()->activeDocument();
    if (!doc) return;

    taskMgr()->run(tr("导出 STEP"),
        [path, doc](TaskProgress* prog) {
            prog->setStepName(QStringLiteral("写入 STEP..."));
            prog->setRange(0, 100);

            Handle(XCAFDoc_ShapeTool) st = doc->shapeTool();
            TDF_LabelSequence shapes;
            st->GetFreeShapes(shapes);

            STEPControl_Writer writer;
            for (int i = 1; i <= shapes.Length(); ++i) {
                TopoDS_Shape sh = st->GetShape(shapes.Value(i));
                if (!sh.IsNull())
                    writer.Transfer(sh, STEPControl_AsIs);
            }
            writer.Write(path.toUtf8().constData());
            prog->setValue(100);
        });
}

// ── CmdCloseDocument ───────────────────────────────────────────────────────────
CmdCloseDocument::CmdCloseDocument(IAppContext* ctx)
    : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/close.svg"), tr("关闭"), this);
    a->setShortcut(QKeySequence::Close);
    setAction(a);
}

bool CmdCloseDocument::isEnabled() const
{
    return context()->activeDocument() != nullptr;
}

void CmdCloseDocument::execute()
{
    DocumentId id = context()->activeDocumentId();
    if (id == kInvalidDocumentId) return;
    app()->closeDocument(id);
    context()->updateCommandStates();
}
