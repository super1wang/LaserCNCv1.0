#pragma once

#include "modules/process/communication/communication_message.h"

#include <QAbstractTableModel>
#include <QVector>

namespace lcnc::process {

class CommunicationLogModel : public QAbstractTableModel
{
    Q_OBJECT
public:
    explicit CommunicationLogModel(QObject* parent = nullptr);

    int rowCount(const QModelIndex& parent = QModelIndex()) const override;
    int columnCount(const QModelIndex& parent = QModelIndex()) const override;
    QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;
    QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;

    void addMessage(const CommunicationMessage& message);
    void clear();
    void setCapacity(int capacity);
    int capacity() const { return m_capacity; }

private:
    QVector<CommunicationMessage> m_messages;
    int m_capacity{500};
};

} // namespace lcnc::process