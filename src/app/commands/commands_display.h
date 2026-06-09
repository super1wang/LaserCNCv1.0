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

/**
 * @brief 切换"物理坐标系世界轴"显示。
 *
 * 当前激活 GuiDocument 的场景上挂载/卸载 lcnc::view::WorldAxesRenderer。
 * 该 QAction 是 checkable，由 ribbon"文件→显示→坐标系"调用。
 */
class CmdToggleWorldAxes : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "view.worldAxes";
    explicit CmdToggleWorldAxes(IAppContext* ctx);
    void execute() override;
};

/**
 * @brief 打开应用程序选项对话框（图形渲染 / 选择高亮 / 应用程序 / 机台构型）。
 *
 * 由 ribbon"文件→应用→选项"按钮触发；对话框 Apply/OK 时
 * 即时把设置应用到所有 GuiDocument 与 AppSettings。
 */
class CmdShowOptions : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "app.options";
    explicit CmdShowOptions(IAppContext* ctx);
    void execute() override;
};
