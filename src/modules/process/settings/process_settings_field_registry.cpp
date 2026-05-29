#include "modules/process/settings/process_settings_field_registry.h"

#include "core/logging/logger.h"

#include <QFile>
#include <QSet>
#include <QXmlStreamReader>

namespace lcnc::process {
namespace {

bool isEditableWidgetClass(const QString& className)
{
    return className == QStringLiteral("QLineEdit")
        || className == QStringLiteral("QComboBox")
        || className == QStringLiteral("QCheckBox")
        || className == QStringLiteral("QSpinBox")
        || className == QStringLiteral("QDoubleSpinBox")
        || className == QStringLiteral("QTextEdit")
        || className == QStringLiteral("QPlainTextEdit")
        || className == QStringLiteral("CSwitchWidget");
}

QString valueTypeForClass(const QString& className)
{
    if (className == QStringLiteral("QCheckBox") || className == QStringLiteral("CSwitchWidget"))
        return QStringLiteral("bool");
    if (className == QStringLiteral("QSpinBox"))
        return QStringLiteral("int");
    if (className == QStringLiteral("QDoubleSpinBox"))
        return QStringLiteral("double");
    return QStringLiteral("string");
}

} // namespace

void ProcessSettingsFieldRegistry::clear()
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "process.settings.registry: clear fields={}", m_fields.size());
    m_fields.clear();
}

bool ProcessSettingsFieldRegistry::registerUiFile(const QString& pageId,
                                                  const QString& resourcePath,
                                                  const QString& visibleNodePath,
                                                  bool visibleInTree,
                                                  QString* errorMessage)
{
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "process.settings.registry: scan ui page='{}' resource='{}'",
               pageId.toStdString(),
               resourcePath.toStdString());

    QFile file(resourcePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        const QString message = QStringLiteral("open failed: %1").arg(resourcePath);
        if (errorMessage)
            *errorMessage = message;
        LCNC_WARN(lcnc::LogCode::Generic,
                  "process.settings.registry: {}",
                  message.toStdString());
        return false;
    }

    int registeredCount = 0;
    QSet<QString> seenObjectNames;
    QXmlStreamReader xml(&file);
    while (!xml.atEnd()) {
        xml.readNext();
        if (!xml.isStartElement() || xml.name() != QStringLiteral("widget"))
            continue;

        const QString className = xml.attributes().value(QStringLiteral("class")).toString();
        const QString objectName = xml.attributes().value(QStringLiteral("name")).toString().trimmed();
        if (!isEditableWidgetClass(className) || objectName.isEmpty() || seenObjectNames.contains(objectName))
            continue;

        ProcessSettingsField field;
        field.pageId = pageId;
        field.objectName = objectName;
        field.fieldId = QStringLiteral("%1.%2").arg(pageId, objectName);
        field.editorClass = className;
        field.valueType = valueTypeForClass(className);
        field.visibleNodePath = visibleNodePath;
        field.visibleInTree = visibleInTree;

        seenObjectNames.insert(objectName);
        int depth = 1;
        while (!xml.atEnd() && depth > 0) {
            xml.readNext();
            if (xml.isStartElement()) {
                if (xml.name() == QStringLiteral("widget")) {
                    ++depth;
                } else if (className == QStringLiteral("QComboBox")
                           && xml.name() == QStringLiteral("string")) {
                    const QString option = xml.readElementText().trimmed();
                    if (!option.isEmpty() && !field.options.contains(option))
                        field.options.append(option);
                }
            } else if (xml.isEndElement() && xml.name() == QStringLiteral("widget")) {
                --depth;
            }
        }

        registerField(field);
        ++registeredCount;
    }

    if (xml.hasError()) {
        const QString message = QStringLiteral("xml parse failed: %1: %2")
            .arg(resourcePath, xml.errorString());
        if (errorMessage)
            *errorMessage = message;
        LCNC_WARN(lcnc::LogCode::Generic,
                  "process.settings.registry: {}",
                  message.toStdString());
        return false;
    }

    LCNC_INFO(lcnc::LogCode::Generic,
              "process.settings.registry: page='{}' fields={} visible={}",
              pageId.toStdString(),
              registeredCount,
              visibleInTree);
    return true;
}

void ProcessSettingsFieldRegistry::registerField(const ProcessSettingsField& field)
{
    if (field.fieldId.trimmed().isEmpty()) {
        LCNC_WARN(lcnc::LogCode::Generic, "process.settings.registry: skip empty field id");
        return;
    }
    if (m_fields.contains(field.fieldId)) {
        LCNC_WARN(lcnc::LogCode::Generic,
                  "process.settings.registry: duplicate field='{}' overwritten",
                  field.fieldId.toStdString());
    }
    m_fields.insert(field.fieldId, field);
}

const ProcessSettingsField* ProcessSettingsFieldRegistry::findField(const QString& fieldId) const
{
    const auto it = m_fields.constFind(fieldId);
    if (it == m_fields.cend())
        return nullptr;
    return &it.value();
}

QList<ProcessSettingsField> ProcessSettingsFieldRegistry::fields() const
{
    return m_fields.values();
}

QList<ProcessSettingsField> ProcessSettingsFieldRegistry::fieldsForPage(const QString& pageId) const
{
    QList<ProcessSettingsField> result;
    for (auto it = m_fields.cbegin(); it != m_fields.cend(); ++it) {
        if (it.value().pageId == pageId)
            result.append(it.value());
    }
    return result;
}

} // namespace lcnc::process