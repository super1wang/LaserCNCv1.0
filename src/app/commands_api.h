#pragma once

#include <QObject>
#include <QAction>
#include <QString>
#include <QMap>
#include <functional>

#include "app/i_app_context.h"

/**
 * @brief Abstract base for all application commands.
 *
 * Each command owns one QAction (icon, text, shortcut) and implements
 * execute().  Commands are registered in a CommandContainer by a unique
 * string name so the Ribbon can look them up.
 *
 * Design mirrored from Mayo's app/commands_api.h.
 */
class CommandBase : public QObject
{
    Q_OBJECT
public:
    explicit CommandBase(IAppContext* ctx, QObject* parent = nullptr);
    virtual ~CommandBase() = default;

    virtual void execute() = 0;

    /// Subclasses override to indicate whether this command is currently usable.
    virtual bool isEnabled() const { return true; }

    QAction*     action() const { return m_action; }
    IAppContext*  context() const { return m_ctx; }

    // Convenience accessors
    LcncApplication* app()     const { return m_ctx->app();     }
    GuiApplication*  guiApp()  const { return m_ctx->guiApp();  }
    TaskManager*     taskMgr() const { return m_ctx->taskMgr(); }

protected:
    /// Subclasses call this in their constructor to supply the QAction.
    void setAction(QAction* a);

private slots:
    void onActionTriggered();

private:
    IAppContext* m_ctx;
    QAction*     m_action{nullptr};
};

// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Registry that owns and indexes all CommandBase instances by name.
 */
class CommandContainer : public QObject
{
    Q_OBJECT
public:
    explicit CommandContainer(IAppContext* ctx, QObject* parent = nullptr);

    template<typename CmdType, typename... Args>
    CmdType* addCommand(const QString& name, Args&&... args)
    {
        auto* cmd = new CmdType(m_ctx, std::forward<Args>(args)...);
        cmd->setParent(this);
        m_map.insert(name, cmd);
        return cmd;
    }

    CommandBase* findCommand(const QString& name) const;
    QAction*     findAction(const QString& name) const;

    /// Refresh enabled state for all registered commands.
    void updateAllStates();

private:
    IAppContext*                m_ctx;
    QMap<QString, CommandBase*> m_map;
};
