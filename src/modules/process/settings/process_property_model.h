#pragma once

#include "modules/process/settings/process_settings_service.h"

#include <QStandardItemModel>
#include <QStyledItemDelegate>

namespace lcnc::process {

class ProcessPropertyModel final : public QStandardItemModel
{
    Q_OBJECT

public:
    enum Roles { DescriptorRole = Qt::UserRole + 1, ObjectRole, GroupRole };

    explicit ProcessPropertyModel(ProcessSettingsService* settings, QObject* parent = nullptr);
    void setObject(const ParameterObjectDescriptor& object, const QString& filter = {});
    const ParameterDescriptor* descriptor(const QModelIndex& index) const;
    QString objectId(const QModelIndex& index) const;

    QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;
    bool setData(const QModelIndex& index, const QVariant& value, int role = Qt::EditRole) override;
    Qt::ItemFlags flags(const QModelIndex& index) const override;

signals:
    void fieldEdited(const QString& objectId, const QString& fieldId);

private:
    ProcessSettingsService* m_settings{nullptr};
    QString m_objectId;
    QVector<ParameterDescriptor> m_fields;
};

class ProcessPropertyDelegate final : public QStyledItemDelegate
{
public:
    explicit ProcessPropertyDelegate(QObject* parent = nullptr);
    QWidget* createEditor(QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
    void setEditorData(QWidget* editor, const QModelIndex& index) const override;
    void setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const override;
};

} // namespace lcnc::process
