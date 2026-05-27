#include "modules/process/workflow/process_flow_store.h"

#include <QVariant>

#include <fstream>

namespace lcnc::process {

namespace {

void setError(QString* errorMessage, const QString& message)
{
    if (errorMessage)
        *errorMessage = message;
}

QString tableString(const toml::table& table, const std::string& key, const QString& fallback = QString())
{
    if (!table.count(key) || !table.at(key).is_string())
        return fallback;
    return QString::fromStdString(table.at(key).as_string());
}

bool tableBool(const toml::table& table, const std::string& key, bool fallback)
{
    if (!table.count(key) || !table.at(key).is_boolean())
        return fallback;
    return table.at(key).as_boolean();
}

QVariant variantFromToml(const toml::value& value)
{
    if (value.is_string())
        return QString::fromStdString(value.as_string());
    if (value.is_boolean())
        return value.as_boolean();
    if (value.is_integer())
        return static_cast<qlonglong>(value.as_integer());
    if (value.is_floating())
        return value.as_floating();
    return QVariant();
}

toml::value tomlFromVariant(const QVariant& value)
{
    switch (value.typeId()) {
    case QMetaType::Bool:
        return value.toBool();
    case QMetaType::Int:
    case QMetaType::LongLong:
    case QMetaType::UInt:
    case QMetaType::ULongLong:
        return value.toLongLong();
    case QMetaType::Double:
        return value.toDouble();
    default:
        return value.toString().toStdString();
    }
}

QVariantMap readParameters(const toml::table& table)
{
    QVariantMap parameters;
    if (!table.count("parameters") || !table.at("parameters").is_table())
        return parameters;

    const auto& parameterTable = table.at("parameters").as_table();
    for (const auto& entry : parameterTable) {
        const QVariant value = variantFromToml(entry.second);
        if (value.isValid())
            parameters.insert(QString::fromStdString(entry.first), value);
    }
    return parameters;
}

toml::value writeParameters(const QVariantMap& parameters)
{
    toml::value result(toml::table{});
    for (auto it = parameters.cbegin(); it != parameters.cend(); ++it)
        result[it.key().toStdString()] = tomlFromVariant(it.value());
    return result;
}

ProcessNode readNode(const toml::value& value, bool legacy)
{
    ProcessNode node;
    if (!value.is_table())
        return node;

    const auto& table = value.as_table();
    const QString typeText = tableString(table, "type", QStringLiteral("Base"));
    const QString stateText = tableString(table, "state", QStringLiteral("Enable"));
    const QString fallbackName = legacy ? typeText : defaultProcessNodeName(processNodeTypeFromString(typeText));
    const QString name = legacy
        ? tableString(table, "label", fallbackName)
        : tableString(table, "name", fallbackName);

    node.id = tableString(table, "id", createProcessNodeId());
    node.type = processNodeTypeFromString(typeText);
    node.name = name;
    node.state = processNodeStateFromString(stateText);
    node.enabled = legacy
        ? node.state != ProcessNodeState::Disabled
        : tableBool(table, "enabled", node.state != ProcessNodeState::Disabled);
    node.parameters = readParameters(table);

    if (legacy) {
        const QString info = tableString(table, "info");
        if (!info.isEmpty())
            node.parameters.insert(QStringLiteral("info"), info);
    }

    const char* childKey = legacy ? "children" : "children";
    if (table.count(childKey) && table.at(childKey).is_array()) {
        for (const toml::value& childValue : table.at(childKey).as_array())
            node.children.append(readNode(childValue, legacy));
    }
    return node;
}

toml::value writeNode(const ProcessNode& node)
{
    toml::value value(toml::table{});
    value["id"] = node.id.toStdString();
    value["type"] = processNodeTypeToString(node.type).toStdString();
    value["name"] = node.name.toStdString();
    value["enabled"] = node.enabled;
    value["state"] = processNodeStateToString(node.state).toStdString();
    value["parameters"] = writeParameters(node.parameters);

    toml::array children;
    for (const ProcessNode& child : node.children)
        children.push_back(writeNode(child));
    value["children"] = children;
    return value;
}

toml::value writeLegacyNode(const ProcessNode& node)
{
    toml::value value(toml::table{});
    value["id"] = node.id.toStdString();
    value["type"] = processNodeTypeToString(node.type).toStdString();
    value["state"] = node.enabled
        ? processNodeStateToString(node.state).toStdString()
        : std::string("Disable");
    value["label"] = node.name.toStdString();
    value["info"] = node.parameters.value(QStringLiteral("info")).toString().toStdString();

    toml::array children;
    for (const ProcessNode& child : node.children)
        children.push_back(writeLegacyNode(child));
    value["children"] = children;
    return value;
}

bool readNodesFromArray(const toml::value& arrayValue, bool legacy, QVector<ProcessNode>& output)
{
    if (!arrayValue.is_array())
        return false;

    output.clear();
    for (const toml::value& nodeValue : arrayValue.as_array())
        output.append(readNode(nodeValue, legacy));
    return true;
}

} // namespace

bool ProcessFlowStore::loadFromFile(const QString& filePath,
                                    ProcessFlowDocument& document,
                                    QString* errorMessage)
{
    if (filePath.trimmed().isEmpty()) {
        setError(errorMessage, QStringLiteral("Process file path is empty"));
        return false;
    }

    try {
        return loadFromToml(toml::parse(filePath.toStdString()), document, errorMessage);
    } catch (const std::exception& e) {
        setError(errorMessage, QString::fromLocal8Bit(e.what()));
        return false;
    }
}

bool ProcessFlowStore::saveToFile(const QString& filePath,
                                  const ProcessFlowDocument& document,
                                  QString* errorMessage)
{
    if (filePath.trimmed().isEmpty()) {
        setError(errorMessage, QStringLiteral("Process file path is empty"));
        return false;
    }

    std::ofstream out(filePath.toStdString(), std::ios::binary);
    if (!out.is_open()) {
        setError(errorMessage, QStringLiteral("Cannot open process file for writing"));
        return false;
    }

    out << toml::format(toToml(document));
    return true;
}

bool ProcessFlowStore::loadFromToml(const toml::value& root,
                                    ProcessFlowDocument& document,
                                    QString* errorMessage)
{
    if (!root.is_table() || !root.contains("Process") || !root.at("Process").is_table()) {
        setError(errorMessage, QStringLiteral("Missing Process table"));
        return false;
    }

    const auto& process = root.at("Process").as_table();
    QVector<ProcessNode> nodes;
    bool loaded = false;

    if (process.count("nodes"))
        loaded = readNodesFromArray(process.at("nodes"), false, nodes);
    if (!loaded && process.count("items"))
        loaded = readNodesFromArray(process.at("items"), true, nodes);

    if (!loaded) {
        setError(errorMessage, QStringLiteral("Missing Process nodes"));
        return false;
    }

    document.setRootNodes(std::move(nodes));
    document.markClean();
    return true;
}

toml::value ProcessFlowStore::toToml(const ProcessFlowDocument& document)
{
    toml::value root(toml::table{});
    toml::value process(toml::table{});
    process["schemaVersion"] = SchemaVersion;

    toml::array nodes;
    toml::array legacyItems;
    for (const ProcessNode& node : document.rootNodes()) {
        nodes.push_back(writeNode(node));
        legacyItems.push_back(writeLegacyNode(node));
    }

    process["nodes"] = nodes;
    process["items"] = legacyItems;
    root["Process"] = process;
    return root;
}

} // namespace lcnc::process
