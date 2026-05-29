#pragma once

#include <QList>
#include <QMap>
#include <QString>
#include <QStringList>

namespace lcnc::process {

/**
 * @brief Describes one editable field discovered from a Process settings UI file.
 */
struct ProcessSettingsField
{
    QString pageId;
    QString objectName;
    QString fieldId;
    QString editorClass;
    QString valueType;
    QStringList options;
    QString visibleNodePath;
    bool visibleInTree{false};
};

/**
 * @brief Registers editable fields from legacy Setting .ui resources for typed settings migration.
 */
class ProcessSettingsFieldRegistry
{
public:
    void clear();

    bool registerUiFile(const QString& pageId,
                        const QString& resourcePath,
                        const QString& visibleNodePath,
                        bool visibleInTree,
                        QString* errorMessage = nullptr);

    void registerField(const ProcessSettingsField& field);

    const ProcessSettingsField* findField(const QString& fieldId) const;
    QList<ProcessSettingsField> fields() const;
    QList<ProcessSettingsField> fieldsForPage(const QString& pageId) const;
    int fieldCount() const { return m_fields.size(); }

private:
    QMap<QString, ProcessSettingsField> m_fields;
};

} // namespace lcnc::process