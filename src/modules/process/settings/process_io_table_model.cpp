#include "modules/process/settings/process_io_table_model.h"

namespace lcnc::process {

ProcessIoTableModel::ProcessIoTableModel(ProcessSettingsService* settings, QObject* parent)
    : QAbstractTableModel(parent), m_settings(settings)
{
    reload();
}

void ProcessIoTableModel::setBucket(ProcessIoBucket bucket)
{
    if (m_bucket == bucket)
        return;
    beginResetModel();
    m_bucket = bucket;
    m_channels = m_settings ? m_settings->ioChannels(m_bucket) : QVector<ProcessIoChannel>{};
    endResetModel();
}

void ProcessIoTableModel::reload()
{
    m_channels = m_settings ? m_settings->ioChannels(m_bucket) : QVector<ProcessIoChannel>{};
}

bool ProcessIoTableModel::hasActiveHigh() const
{
    return m_bucket == ProcessIoBucket::DigitalInput || m_bucket == ProcessIoBucket::DigitalOutput;
}

bool ProcessIoTableModel::hasShowInMain() const
{
    return m_bucket == ProcessIoBucket::DigitalOutput;
}

int ProcessIoTableModel::visibleColumnCount() const
{
    return hasShowInMain() ? 5 : (hasActiveHigh() ? 4 : 3);
}

int ProcessIoTableModel::rowCount(const QModelIndex& parent) const { return parent.isValid() ? 0 : m_channels.size(); }
int ProcessIoTableModel::columnCount(const QModelIndex& parent) const { return parent.isValid() ? 0 : visibleColumnCount(); }

QString ProcessIoTableModel::channelIdAt(int row) const
{
    return row >= 0 && row < m_channels.size() ? m_channels.at(row).id : QString();
}

QVariant ProcessIoTableModel::data(const QModelIndex& index, int role) const
{
    if (!index.isValid() || index.row() >= m_channels.size()) return {};
    const ProcessIoChannel& channel = m_channels.at(index.row());
    if (role == Qt::ToolTipRole && index.column() == Name)
        // 中文翻译：内置通道：名称和标识不可删除
        return channel.builtin ? tr("Built-in channel: name and logo cannot be deleted") : channel.id;
    if (index.column() == Name && (role == Qt::DisplayRole || role == Qt::EditRole)) return channel.name;
    if (index.column() == HardwareIndex && (role == Qt::DisplayRole || role == Qt::EditRole)) return channel.hardwareIndex;
    if (role == Qt::CheckStateRole) {
        if (hasActiveHigh() && index.column() == ActiveHigh) return channel.activeHigh ? Qt::Checked : Qt::Unchecked;
        if (index.column() == (hasActiveHigh() ? Enabled : ActiveHigh)) return channel.enabled ? Qt::Checked : Qt::Unchecked;
        if (hasShowInMain() && index.column() == ShowInMain) return channel.showInMain ? Qt::Checked : Qt::Unchecked;
    }
    return {};
}

bool ProcessIoTableModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
    if (!index.isValid() || !m_settings || index.row() >= m_channels.size()) return false;
    ProcessIoChannel next = m_channels.at(index.row());
    if (role == Qt::EditRole) {
        if (index.column() == Name) next.name = value.toString();
        else if (index.column() == HardwareIndex) next.hardwareIndex = value.toString();
        else return false;
    } else if (role == Qt::CheckStateRole) {
        const bool checked = value.toInt() == Qt::Checked;
        if (hasActiveHigh() && index.column() == ActiveHigh) next.activeHigh = checked;
        else if (index.column() == (hasActiveHigh() ? Enabled : ActiveHigh)) next.enabled = checked;
        else if (hasShowInMain() && index.column() == ShowInMain) next.showInMain = checked;
        else return false;
    } else return false;
    QString error;
    if (!m_settings->setIoChannel(m_bucket, next.id, next, &error)) {
        emit dataChanged(index, index, {Qt::ToolTipRole});
        return false;
    }
    m_channels[index.row()] = next;
    emit dataChanged(index, index, {Qt::DisplayRole, Qt::EditRole, Qt::CheckStateRole, Qt::ToolTipRole});
    return true;
}

Qt::ItemFlags ProcessIoTableModel::flags(const QModelIndex& index) const
{
    if (!index.isValid()) return Qt::NoItemFlags;
    Qt::ItemFlags result = Qt::ItemIsEnabled | Qt::ItemIsSelectable;
    const bool checkable = (hasActiveHigh() && index.column() == ActiveHigh)
        || index.column() == (hasActiveHigh() ? Enabled : ActiveHigh)
        || (hasShowInMain() && index.column() == ShowInMain);
    if (checkable) return result | Qt::ItemIsUserCheckable;
    if (index.column() == HardwareIndex || (index.column() == Name && !m_channels.at(index.row()).builtin))
        result |= Qt::ItemIsEditable;
    return result;
}

QVariant ProcessIoTableModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (orientation != Qt::Horizontal || role != Qt::DisplayRole) return {};
    // 中文翻译：名称
    if (section == Name) return tr("Name");
    // 中文翻译：通道索引
    if (section == HardwareIndex) return tr("Channel index");
    // 中文翻译：高电平有效
    if (hasActiveHigh() && section == ActiveHigh) return tr("Active high level");
    // 中文翻译：启用
    if (section == (hasActiveHigh() ? Enabled : ActiveHigh)) return tr("enable");
    // 中文翻译：显示到主界面
    if (hasShowInMain() && section == ShowInMain) return tr("Show to main interface");
    return {};
}

bool ProcessIoTableModel::addChannel(QString* error)
{
    if (!m_settings) return false;
    QString id;
    const bool ok = m_settings->addIoChannel(m_bucket, &id, error);
    if (!ok) return false;
    const int row = m_channels.size();
    beginInsertRows({}, row, row);
    m_channels = m_settings->ioChannels(m_bucket);
    endInsertRows();
    return ok;
}

bool ProcessIoTableModel::removeChannel(int row, QString* error)
{
    if (!m_settings || row < 0 || row >= m_channels.size()) return false;
    if (!m_settings->removeIoChannel(m_bucket, m_channels.at(row).id, error)) return false;
    beginRemoveRows({}, row, row);
    m_channels.removeAt(row);
    endRemoveRows();
    return true;
}

} // namespace lcnc::process
