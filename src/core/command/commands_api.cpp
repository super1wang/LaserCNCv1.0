#include "core/command/commands_api.h"

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
    for (auto it = m_map.cbegin(); it != m_map.cend(); ++it) {
        auto* cmd = it.value();
        if (cmd->action())
            cmd->action()->setEnabled(cmd->isEnabled()
                && (!m_interactionLocked
                    || it.key().startsWith(QStringLiteral("view."))
                    || m_allowedWhileLocked.contains(it.key())));
    }
}

void CommandContainer::setInteractionLocked(bool locked, const QSet<QString>& allowedNames)
{
    m_interactionLocked = locked;
    m_allowedWhileLocked = allowedNames;
    updateAllStates();
}
