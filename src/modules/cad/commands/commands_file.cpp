#include "modules/cad/commands/commands_file.h"

#include <QAction>
#include <QFileDialog>

#include "core/document/lcnc_document.h"
#include "modules/cad/cad_module.h"

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
    context()->cadModule()->newDocument();
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
        tr("所有支持格式 (*.stp *.step *.igs *.iges *.stl *.brep);;"
           "STEP (*.stp *.step);;"
           "IGES (*.igs *.iges);;"
           "STL (*.stl);;"
           "BREP (*.brep)"));
    if (path.isEmpty()) return;

    context()->cadModule()->openDocument(path);
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
    context()->cadModule()->saveDocument(doc->id(), doc->filePath());
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
    context()->cadModule()->saveDocument(doc->id(), path);
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

    context()->cadModule()->importStep(path, context()->activeDocumentId());
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

    context()->cadModule()->importStl(path, context()->activeDocumentId());
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

    context()->cadModule()->exportStep(doc->id(), path);
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
    context()->cadModule()->closeDocument(id);
    context()->updateCommandStates();
}
