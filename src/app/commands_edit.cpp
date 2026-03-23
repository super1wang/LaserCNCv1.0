#include "app/commands_edit.h"

#include <QAction>
#include <QKeySequence>

#include "base/lcnc_application.h"
#include "base/lcnc_document.h"
#include "gui/gui_document.h"

// ── CmdUndo ────────────────────────────────────────────────────────────────────
CmdUndo::CmdUndo(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/undo.svg"), tr("撤销"), this);
    a->setShortcut(QKeySequence::Undo);
    a->setStatusTip(tr("撤销上一步操作"));
    setAction(a);
}

bool CmdUndo::isEnabled() const
{
    if (LcncDocument* d = context()->activeDocument())
        return d->canUndo();
    return false;
}

void CmdUndo::execute()
{
    if (LcncDocument* d = context()->activeDocument()) {
        d->undo();
        if (auto* gd = context()->activeGuiDocument())
            gd->rebuildDisplay();
        context()->app()->notifyDocumentModified(d->id());
        context()->updateCommandStates();
    }
}

// ── CmdRedo ────────────────────────────────────────────────────────────────────
CmdRedo::CmdRedo(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/redo.svg"), tr("重做"), this);
    a->setShortcut(QKeySequence::Redo);
    a->setStatusTip(tr("重做上一步操作"));
    setAction(a);
}

bool CmdRedo::isEnabled() const
{
    if (LcncDocument* d = context()->activeDocument())
        return d->canRedo();
    return false;
}

void CmdRedo::execute()
{
    if (LcncDocument* d = context()->activeDocument()) {
        d->redo();
        if (auto* gd = context()->activeGuiDocument())
            gd->rebuildDisplay();
        context()->app()->notifyDocumentModified(d->id());
        context()->updateCommandStates();
    }
}
