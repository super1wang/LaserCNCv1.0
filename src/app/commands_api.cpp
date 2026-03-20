#include "app/commands_api.h"

// ── CommandBase ────────────────────────────────────────────────────────────────
CommandBase::CommandBase(IAppContext* ctx, QObject* parent)
    : QObject(parent)
    , m_ctx(ctx)
{}

void CommandBase::setAction(QAction* a)
{
    m_action = a;
    connect(m_action, &QAction::triggered,
            this,     &CommandBase::onActionTriggered);
}

void CommandBase::onActionTriggered()
{
    if (isEnabled())
        execute();
}

// ── CommandContainer ───────────────────────────────────────────────────────────
CommandContainer::CommandContainer(IAppContext* ctx, QObject* parent)
    : QObject(parent)
    , m_ctx(ctx)
{}

CommandBase* CommandContainer::findCommand(const QString& name) const
{
    return m_map.value(name, nullptr);
}

QAction* CommandContainer::findAction(const QString& name) const
{
    if (auto* cmd = findCommand(name))
        return cmd->action();
    return nullptr;
}

void CommandContainer::updateAllStates()
{
    for (auto* cmd : m_map) {
        if (cmd->action())
            cmd->action()->setEnabled(cmd->isEnabled());
    }
}
