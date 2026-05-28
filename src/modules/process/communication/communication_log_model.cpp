#include "modules/process/communication/communication_log_model.h"

namespace lcnc::process {

CommunicationLogModel::CommunicationLogModel(QObject* parent)
    : QAbstractTableModel(parent)
{
}

int CommunicationLogModel::rowCount(const QModelIndex& parent) const
{
    return parent.isValid() ? 0 : m_messages.size();
}

int CommunicationLogModel::columnCount(const QModelIndex& parent) const
{
    return parent.isValid() ? 0 : 4;
}

QVariant CommunicationLogModel::data(const QModelIndex& index, int role) const
{
    if (!index.isValid() || index.row() < 0 || index.row() >= m_messages.size())
        return {};

    const CommunicationMessage& message = m_messages.at(index.row());
    if (role == Qt::ToolTipRole)
        return message.displayText();
    if (role != Qt::DisplayRole)
        return {};

    switch (index.column()) {
    case 0:
        return message.timestamp.toString(QStringLiteral("HH:mm:ss.zzz"));
    case 1:
        return message.deviceId;
    case 2:
        return communicationMessageDirectionText(message.direction);
    case 3:
        return message.displayText();
    default:
        return {};
    }
}

QVariant CommunicationLogModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (orientation != Qt::Horizontal || role != Qt::DisplayRole)
        return {};
    switch (section) {
    case 0:
        return tr("Time");
    case 1:
        return tr("Device");
    case 2:
        return tr("Dir");
    case 3:
        return tr("Payload");
    default:
        return {};
    }
}

void CommunicationLogModel::addMessage(const CommunicationMessage& message)
{
    if (m_capacity <= 0)
        return;
    while (m_messages.size() >= m_capacity) {
        beginRemoveRows(QModelIndex(), 0, 0);
        m_messages.removeFirst();
        endRemoveRows();
    }

    const int row = m_messages.size();
    beginInsertRows(QModelIndex(), row, row);
    m_messages.append(message);
    endInsertRows();
}

void CommunicationLogModel::clear()
{
    if (m_messages.isEmpty())
        return;
    beginResetModel();
    m_messages.clear();
    endResetModel();
}

void CommunicationLogModel::setCapacity(int capacity)
{
    m_capacity = qMax(1, capacity);
    while (m_messages.size() > m_capacity) {
        beginRemoveRows(QModelIndex(), 0, 0);
        m_messages.removeFirst();
        endRemoveRows();
    }
}

} // namespace lcnc::process