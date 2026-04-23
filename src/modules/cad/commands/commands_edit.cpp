#include "modules/cad/commands/commands_edit.h"

#include <QAction>
#include <QKeySequence>

#include "core/document/lcnc_document.h"
#include "modules/cad/cad_module.h"

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
    return context()->cadModule()->canUndo(context()->activeDocumentId());
}

void CmdUndo::execute()
{
    context()->cadModule()->undo(context()->activeDocumentId());
    context()->updateCommandStates();
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
    return context()->cadModule()->canRedo(context()->activeDocumentId());
}

void CmdRedo::execute()
{
    context()->cadModule()->redo(context()->activeDocumentId());
    context()->updateCommandStates();
}
