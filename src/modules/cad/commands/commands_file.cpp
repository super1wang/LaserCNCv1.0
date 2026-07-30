#include "modules/cad/commands/commands_file.h"

#include "app/app_command_context.h"

#include <QAction>
#include <QFileDialog>

#include "core/document/lcnc_document.h"
#include "core/logging/logger.h"
#include "modules/cad/cad_module.h"

// ── CmdNewDocument ─────────────────────────────────────────────────────────────
CmdNewDocument::CmdNewDocument(IAppContext* ctx)
    : CommandBase(ctx)
{
    // 中文翻译：新建
    auto* a = new QAction(QIcon(":/icons/new_doc.svg"), tr("New"), this);
    a->setShortcut(QKeySequence::New);
    // 中文翻译：新建空白文档
    a->setStatusTip(tr("Create a new blank document"));
    setAction(a);
}

void CmdNewDocument::execute()
{
    context()->cadModule()->newDocument();
    context()->updateCommandStates();
}

// ── CmdOpenDocument ────────────────────────────────────────────────────────────
CmdOpenDocument::CmdOpenDocument(IAppContext* ctx)
    : CommandBase(ctx)
{
    // 中文翻译：打开
    auto* a = new QAction(QIcon(":/icons/open.svg"), tr("open"), this);
    a->setShortcut(QKeySequence::Open);
    // 中文翻译：打开 LaserCNC 项目或工件模型
    a->setStatusTip(tr("Open a LaserCNC project or workpiece model"));
    setAction(a);
}

void CmdOpenDocument::execute()
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "CmdOpenDocument::execute begin");
    const QString path = QFileDialog::getOpenFileName(
        // 中文翻译：打开文件
        nullptr, tr("open file"), QString(),
          // 中文翻译：所有支持格式 (*.lcnc project.toml *.stp *.step *.igs *.iges *.stl *.brep);;
          tr("All supported formats (*.lcnc project.toml *.stp *.step *.igs *.iges *.stl *.brep);;"
              // 中文翻译：LaserCNC 项目 (*.lcnc project.toml);;
              "LaserCNC project (*.lcnc project.toml);;"
           "STEP (*.stp *.step);;"
           "IGES (*.igs *.iges);;"
           "STL (*.stl);;"
           "BREP (*.brep)"));
    if (path.isEmpty()) {
        LCNC_DEBUG(lcnc::LogCode::Generic, "CmdOpenDocument::execute cancelled");
        return;
    }

    LCNC_DEBUG(lcnc::LogCode::Generic,
               "CmdOpenDocument::execute selected path={}",
               path.toStdString());
    const DocumentId docId = context()->cadModule()->openDocument(path);
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "CmdOpenDocument::execute openDocument returned docId={}",
               docId);
    if (docId != kInvalidDocumentId)
        context()->cadModule()->requestWorkpieceView(docId);
    context()->updateCommandStates();
}

// ── CmdSaveDocument ────────────────────────────────────────────────────────────
CmdSaveDocument::CmdSaveDocument(IAppContext* ctx)
    : CommandBase(ctx)
{
    // 中文翻译：保存
    auto* a = new QAction(QIcon(":/icons/save.svg"), tr("save"), this);
    a->setShortcut(QKeySequence::Save);
    // 中文翻译：保存当前项目
    a->setStatusTip(tr("Save current project"));
    setAction(a);
}

bool CmdSaveDocument::isEnabled() const
{
    return context()->workpieceDocument() != nullptr;
}

void CmdSaveDocument::execute()
{
    LcncDocument* doc = context()->workpieceDocument();
    if (!doc) return;
    if (doc->filePath().isEmpty()) {
        // Delegate to Save As
        CmdSaveDocumentAs saveAs(context());
        saveAs.execute();
        return;
    }
    context()->cadModule()->saveDocument(doc->id(), doc->filePath());
}

// ── CmdSaveDocumentAs ──────────────────────────────────────────────────────────
CmdSaveDocumentAs::CmdSaveDocumentAs(IAppContext* ctx)
    : CommandBase(ctx)
{
    // 中文翻译：另存为
    auto* a = new QAction(QIcon(":/icons/save_as.svg"), tr("save as"), this);
    a->setShortcut(QKeySequence::SaveAs);
    setAction(a);
}

bool CmdSaveDocumentAs::isEnabled() const
{
    return context()->workpieceDocument() != nullptr;
}

void CmdSaveDocumentAs::execute()
{
    LcncDocument* doc = context()->workpieceDocument();
    if (!doc) return;
    const QString path = QFileDialog::getSaveFileName(
        // 中文翻译：另存为
        nullptr, tr("save as"), doc->name(),
        // 中文翻译：LaserCNC 项目 (*.lcnc);;STEP (*.stp *.step)
        tr("LaserCNC Project (*.lcnc);;STEP (*.stp *.step)"));
    if (path.isEmpty()) return;
    context()->cadModule()->saveDocument(doc->id(), path);
}

// ── CmdImportStep ──────────────────────────────────────────────────────────────
CmdImportStep::CmdImportStep(IAppContext* ctx)
    : CommandBase(ctx)
{
    // 中文翻译：导入STEP
    auto* a = new QAction(QIcon(":/icons/import.svg"), tr("Import STEP"), this);
    // 中文翻译：导入 STEP 文件到当前文档
    a->setStatusTip(tr("Import STEP files into the current document"));
    setAction(a);
}

void CmdImportStep::execute()
{
    const QString path = QFileDialog::getOpenFileName(
        // 中文翻译：导入 STEP
        nullptr, tr("Import STEP"), QString(),
        // 中文翻译：STEP 文件 (*.stp *.step)
        tr("STEP files (*.stp *.step)"));
    if (path.isEmpty()) return;

    context()->cadModule()->importStep(path, context()->workpieceDocumentId());
}

// ── CmdImportStl ───────────────────────────────────────────────────────────────
CmdImportStl::CmdImportStl(IAppContext* ctx)
    : CommandBase(ctx)
{
    // 中文翻译：导入STL
    auto* a = new QAction(QIcon(":/icons/import.svg"), tr("Import STL"), this);
    // 中文翻译：导入 STL 文件（机台/工件模型）
    a->setStatusTip(tr("Import STL file (machine/workpiece model)"));
    setAction(a);
}

void CmdImportStl::execute()
{
    const QString path = QFileDialog::getOpenFileName(
        // 中文翻译：导入 STL
        nullptr, tr("Import STL"), QString(),
        // 中文翻译：STL 文件 (*.stl)
        tr("STL files (*.stl)"));
    if (path.isEmpty()) return;

    context()->cadModule()->importStl(path, context()->workpieceDocumentId());
}

// ── CmdExportStep ──────────────────────────────────────────────────────────────
CmdExportStep::CmdExportStep(IAppContext* ctx)
    : CommandBase(ctx)
{
    // 中文翻译：导出STEP
    auto* a = new QAction(QIcon(":/icons/export.svg"), tr("Export STEP"), this);
    setAction(a);
}

bool CmdExportStep::isEnabled() const
{
    return context()->workpieceDocument() != nullptr;
}

void CmdExportStep::execute()
{
    const QString path = QFileDialog::getSaveFileName(
        // 中文翻译：导出 STEP
        nullptr, tr("Export STEP"), QString(),
        // 中文翻译：STEP 文件 (*.stp *.step)
        tr("STEP files (*.stp *.step)"));
    if (path.isEmpty()) return;

    LcncDocument* doc = context()->workpieceDocument();
    if (!doc) return;

    context()->cadModule()->exportStep(doc->id(), path);
}

// ── CmdCloseDocument ───────────────────────────────────────────────────────────
CmdCloseDocument::CmdCloseDocument(IAppContext* ctx)
    : CommandBase(ctx)
{
    // 中文翻译：关闭
    auto* a = new QAction(QIcon(":/icons/close.svg"), tr("Close"), this);
    a->setShortcut(QKeySequence::Close);
    setAction(a);
}

bool CmdCloseDocument::isEnabled() const
{
    return context()->workpieceDocument() != nullptr;
}

void CmdCloseDocument::execute()
{
    DocumentId id = context()->workpieceDocumentId();
    if (id == kInvalidDocumentId) return;
    context()->cadModule()->closeDocument(id);
    context()->updateCommandStates();
}
