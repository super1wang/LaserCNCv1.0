#pragma once
#include "core/command/commands_api.h"
#include <V3d_TypeOfOrientation.hxx>

// ─────────────────────────────────────────────────────────────────────────────
// Display / View commands
// ─────────────────────────────────────────────────────────────────────────────

class CmdFitAll : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "view.fitAll";
    explicit CmdFitAll(IAppContext* ctx);
    void execute() override;
};

class CmdViewOrient : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "view.orient";
    CmdViewOrient(IAppContext* ctx, V3d_TypeOfOrientation orient,
                  const QString& label, const QIcon& icon);
    void execute() override;

private:
    V3d_TypeOfOrientation m_orient;
};

class CmdToggleWireframe : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "view.wireframe";
    explicit CmdToggleWireframe(IAppContext* ctx);
    void execute() override;
};

class CmdToggleShaded : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "view.shaded";
    explicit CmdToggleShaded(IAppContext* ctx);
    void execute() override;
};

class CmdToggleShadedWithEdges : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "view.shadedEdges";
    explicit CmdToggleShadedWithEdges(IAppContext* ctx);
    void execute() override;
};
