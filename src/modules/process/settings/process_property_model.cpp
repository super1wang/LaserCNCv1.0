#include "modules/process/settings/process_property_model.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QMap>
#include <QSpinBox>

namespace lcnc::process {

ProcessPropertyModel::ProcessPropertyModel(ProcessSettingsService* settings, QObject* parent)
    : QStandardItemModel(parent), m_settings(settings)
{
    setColumnCount(2);
    setHorizontalHeaderLabels({tr("参数"), tr("值")});
}

void ProcessPropertyModel::setObject(const ParameterObjectDescriptor& object, const QString& filter)
{
    clear();
    setColumnCount(2);
    setHorizontalHeaderLabels({tr("参数"), tr("值")});
    m_objectId = object.id;
    m_fields.clear();
    QMap<QString, QStandardItem*> groups;
    for (const ParameterDescriptor& field : object.fields) {
        if (!filter.isEmpty() && !field.title.contains(filter, Qt::CaseInsensitive)
            && !field.group.contains(filter, Qt::CaseInsensitive))
            continue;
        QStandardItem* group = groups.value(field.group);
        if (!group) {
            group = new QStandardItem(field.group);
            group->setEditable(false);
            auto* spacer = new QStandardItem;
            spacer->setEditable(false);
            invisibleRootItem()->appendRow({group, spacer});
            groups.insert(field.group, group);
        }
        const int descriptorIndex = m_fields.size();
        m_fields.append(field);
        QString label = field.title;
        if (!field.unit.isEmpty()) label += QStringLiteral(" (%1)").arg(field.unit);
        auto* key = new QStandardItem(label);
        auto* value = new QStandardItem;
        key->setEditable(false);
        value->setData(descriptorIndex, DescriptorRole);
        value->setData(m_objectId, ObjectRole);
        group->appendRow({key, value});
    }
}

const ParameterDescriptor* ProcessPropertyModel::descriptor(const QModelIndex& index) const
{
    const QModelIndex valueIndex = index.column() == 1 ? index : index.siblingAtColumn(1);
    const QVariant descriptorValue = QStandardItemModel::data(valueIndex, DescriptorRole);
    if (!descriptorValue.isValid()) return nullptr;
    const int i = descriptorValue.toInt();
    return i >= 0 && i < m_fields.size() ? &m_fields.at(i) : nullptr;
}

QString ProcessPropertyModel::objectId(const QModelIndex& index) const
{
    return QStandardItemModel::data(index.column() == 1 ? index : index.siblingAtColumn(1), ObjectRole).toString();
}

QVariant ProcessPropertyModel::data(const QModelIndex& index, int role) const
{
    if (const auto* field = descriptor(index)) {
        if (index.column() == 1) {
            const QVariant value = m_settings ? m_settings->fieldValue(*field, objectId(index)) : QVariant{};
            if (field->type == ParameterValueType::Bool) {
                if (role == Qt::CheckStateRole)
                    return value.toBool() ? Qt::Checked : Qt::Unchecked;
                // A check box is the complete visual representation.  Returning
                // a string here made Qt open a transient editor and showed the
                // old true/false value only after it closed.
                if (role == Qt::DisplayRole)
                    return QVariant{};
            }
            if (role == Qt::DisplayRole || role == Qt::EditRole)
                return value;
        }
        if (index.column() == 1 && role == Qt::ToolTipRole)
            return field->description;
    }
    return QStandardItemModel::data(index, role);
}

bool ProcessPropertyModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
    if (index.column() != 1)
        return QStandardItemModel::setData(index, value, role);
    const auto* field = descriptor(index);
    if (!field || !m_settings) return false;
    if (role != Qt::EditRole && !(field->type == ParameterValueType::Bool && role == Qt::CheckStateRole))
        return QStandardItemModel::setData(index, value, role);
    QString error;
    const QVariant draftValue = field->type == ParameterValueType::Bool && role == Qt::CheckStateRole
        ? QVariant(value.toInt() == Qt::Checked) : value;
    if (!m_settings->setFieldValue(*field, objectId(index), draftValue, &error)) {
        QStandardItemModel::setData(index, error, Qt::ToolTipRole);
        return false;
    }
    QStandardItemModel::setData(index, QVariant(), Qt::ToolTipRole);
    emit dataChanged(index, index, {Qt::DisplayRole, Qt::EditRole, Qt::CheckStateRole, Qt::ToolTipRole});
    return true;
}

Qt::ItemFlags ProcessPropertyModel::flags(const QModelIndex& index) const
{
    auto result = QStandardItemModel::flags(index);
    if (index.column() == 1 && descriptor(index) && !descriptor(index)->readOnly) {
        if (descriptor(index)->type == ParameterValueType::Bool)
            result |= Qt::ItemIsUserCheckable;
        else
            result |= Qt::ItemIsEditable;
    }
    return result;
}

ProcessPropertyDelegate::ProcessPropertyDelegate(QObject* parent) : QStyledItemDelegate(parent) {}

QWidget* ProcessPropertyDelegate::createEditor(QWidget* parent, const QStyleOptionViewItem&, const QModelIndex& index) const
{
    const auto* propertyModel = dynamic_cast<const ProcessPropertyModel*>(index.model());
    const auto* field = propertyModel ? propertyModel->descriptor(index) : nullptr;
    if (!field) return nullptr;
    // Bool properties are edited through CheckStateRole.  Do not create an
    // editor: a QCheckBox editor only commits when editing ends.
    if (field->type == ParameterValueType::Bool) return nullptr;
    if (field->type == ParameterValueType::Int) { auto* editor = new QSpinBox(parent); editor->setRange(int(field->minimum), int(field->maximum)); return editor; }
    if (field->type == ParameterValueType::Double) { auto* editor = new QDoubleSpinBox(parent); editor->setRange(field->minimum, field->maximum); editor->setDecimals(field->decimals); return editor; }
    if (field->type == ParameterValueType::Enum || field->type == ParameterValueType::AxisRef || field->type == ParameterValueType::IoRef || field->type == ParameterValueType::ToolRef) {
        auto* editor = new QComboBox(parent); editor->addItems(field->enumValues); return editor;
    }
    return new QLineEdit(parent);
}

void ProcessPropertyDelegate::setEditorData(QWidget* editor, const QModelIndex& index) const
{
    const QVariant value = index.data(Qt::EditRole);
    if (auto* spin = qobject_cast<QSpinBox*>(editor)) spin->setValue(value.toInt());
    else if (auto* spin = qobject_cast<QDoubleSpinBox*>(editor)) spin->setValue(value.toDouble());
    else if (auto* combo = qobject_cast<QComboBox*>(editor)) combo->setCurrentText(value.toString());
    else if (auto* line = qobject_cast<QLineEdit*>(editor)) line->setText(value.toString());
}

void ProcessPropertyDelegate::setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const
{
    if (auto* spin = qobject_cast<QSpinBox*>(editor)) model->setData(index, spin->value());
    else if (auto* spin = qobject_cast<QDoubleSpinBox*>(editor)) model->setData(index, spin->value());
    else if (auto* combo = qobject_cast<QComboBox*>(editor)) model->setData(index, combo->currentText());
    else if (auto* line = qobject_cast<QLineEdit*>(editor)) model->setData(index, line->text());
}

} // namespace lcnc::process
