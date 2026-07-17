#pragma once

#include "modules/process/settings/process_settings_service.h"

#include <QAbstractTableModel>

namespace lcnc::process {

class ProcessIoTableModel final : public QAbstractTableModel
{
    Q_OBJECT
public:
    explicit ProcessIoTableModel(ProcessSettingsService* settings, QObject* parent = nullptr);

    void setBucket(ProcessIoBucket bucket);
    ProcessIoBucket bucket() const { return m_bucket; }
    QString channelIdAt(int row) const;
    bool addChannel(QString* error = nullptr);
    bool removeChannel(int row, QString* error = nullptr);

    int rowCount(const QModelIndex& parent = {}) const override;
    int columnCount(const QModelIndex& parent = {}) const override;
    QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;
    bool setData(const QModelIndex& index, const QVariant& value, int role = Qt::EditRole) override;
    Qt::ItemFlags flags(const QModelIndex& index) const override;
    QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;

private:
    enum Column { Name, HardwareIndex, ActiveHigh, Enabled, ShowInMain };
    int visibleColumnCount() const;
    bool hasActiveHigh() const;
    bool hasShowInMain() const;
    void reload();

    ProcessSettingsService* m_settings{nullptr};
    ProcessIoBucket m_bucket{ProcessIoBucket::DigitalInput};
    QVector<ProcessIoChannel> m_channels;
};

} // namespace lcnc::process
