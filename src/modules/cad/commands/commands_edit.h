#pragma once
#include "core/command/commands_api.h"

// ─────────────────────────────────────────────────────────────────────────────
// Edit commands (Undo / Redo)
// ─────────────────────────────────────────────────────────────────────────────

class CmdUndo : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "edit.undo";
    explicit CmdUndo(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};

class CmdRedo : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "edit.redo";
    explicit CmdRedo(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};
