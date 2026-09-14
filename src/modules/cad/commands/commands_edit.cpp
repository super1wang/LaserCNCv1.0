#include "modules/cad/commands/commands_edit.h"

#include "app/app_command_context.h"

#include <QAction>
#include <QKeySequence>

#include "core/document/lcnc_document.h"
#include "modules/cad/cad_module.h"

// ── CmdUndo ────────────────────────────────────────────────────────────────────
CmdUndo::CmdUndo(IAppContext* ctx) : CommandBase(ctx)
{
    // 中文翻译：撤销
    auto* a = new QAction(QIcon("themeicons:undo.svg"), tr("Cancel"), this);
    a->setShortcut(QKeySequence::Undo);
    // 中文翻译：撤销上一步操作
    a->setStatusTip(tr("Undo the previous action"));
    setAction(a);
}

bool CmdUndo::isEnabled() const
{
    return context()->cadModule()->canUndo(context()->workpieceDocumentId());
}

void CmdUndo::execute()
{
    context()->cadModule()->undo(context()->workpieceDocumentId());
    context()->updateCommandStates();
}

// ── CmdRedo ────────────────────────────────────────────────────────────────────
CmdRedo::CmdRedo(IAppContext* ctx) : CommandBase(ctx)
{
    // 中文翻译：重做
    auto* a = new QAction(QIcon("themeicons:redo.svg"), tr("Redo"), this);
    a->setShortcut(QKeySequence::Redo);
    // 中文翻译：重做上一步操作
    a->setStatusTip(tr("Redo the previous step"));
    setAction(a);
}

bool CmdRedo::isEnabled() const
{
    return context()->cadModule()->canRedo(context()->workpieceDocumentId());
}

void CmdRedo::execute()
{
    context()->cadModule()->redo(context()->workpieceDocumentId());
    context()->updateCommandStates();
}
