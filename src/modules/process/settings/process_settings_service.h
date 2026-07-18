#pragma once

#include "core/kinematics/machine_configuration_service.h"
#include "modules/process/settings/process_parameter_registry.h"
#include "modules/process/settings/process_settings_types.h"

#include "toml.hpp"

#include <QHash>
#include <QStringList>

namespace lcnc::process {

struct ProcessSettingsSnapshot { toml::value value{toml::table{}}; };
struct ProcessSettingsChangeSet { QStringList domains; QStringList changedToolNames; bool empty() const { return domains.isEmpty() && changedToolNames.isEmpty(); } };
struct ProcessSettingsCommitResult { bool success{false}; ProcessSettingsChangeSet changes; QString error; };

// The sole Process configuration repository.  Its public surface is typed;
// TOML is confined to this implementation and is never read by devices or UI.
class ProcessSettingsService
{
public:
    ProcessSettingsService();

    bool initialize();
    void beginEdit();
    bool cancelEdit();
    bool hasChanges() const;
    ProcessSettingsSnapshot committedSnapshot() const { return m_committed; }
    ProcessSettingsCommitResult commit();

    void refreshDraft();
    QVector<ParameterObjectDescriptor> objects() const;
    QVariant fieldValue(const ParameterDescriptor& field, const QString& objectId) const;
    bool setFieldValue(const ParameterDescriptor& field, const QString& objectId, const QVariant& value, QString* error = nullptr);

    bool createTool(const QString& name, QString* error = nullptr);
    bool copyTool(const QString& source, const QString& target, QString* error = nullptr);
    bool renameTool(const QString& source, const QString& target, QString* error = nullptr);
    bool deleteTool(const QString& name, QString* error = nullptr);
    QStringList toolDisplayNames() const;

    QVector<ProcessIoChannel> ioChannels(ProcessIoBucket bucket) const;
    QStringList ioDisplayNames(ProcessIoBucket bucket) const;
    bool addIoChannel(ProcessIoBucket bucket, QString* createdId = nullptr, QString* error = nullptr);
    bool removeIoChannel(ProcessIoBucket bucket, const QString& id, QString* error = nullptr);
    bool setIoChannel(ProcessIoBucket bucket, const QString& id, const ProcessIoChannel& channel, QString* error = nullptr);
    QStringList ioReferenceLocations(const QString& id) const;

    // Runtime query API.  These methods are the only configuration access
    // permitted outside this class until each legacy adapter is fully typed.
    toml::table rawTable(ProcessConfigArea area, const QString& tableName = {}) const;
    QVariant rawValue(ProcessConfigArea area, const QString& tableName, const QString& key, const QVariant& fallback = {}) const;
    toml::table axisRuntimeTable(const QString& axisName) const;

private:
    QString rootDir() const;
    QString sectionName(ProcessConfigArea area, const QString& tableName = {}) const;
    toml::table& sectionRef(ProcessConfigArea area, const QString& tableName = {});
    const toml::table& sectionRef(ProcessConfigArea area, const QString& tableName = {}) const;
    bool loadDomain(const QString& fileName, QString* error);
    bool writeDomain(const QString& fileName, const QStringList& sections, QString* error) const;
    bool writeTools(QString* error, ProcessSettingsChangeSet* changes) const;
    bool writeTomlAtomically(const QString& filePath, const toml::value& value, QString* error) const;
    ProcessSettingsChangeSet changesSinceCommitted() const;
    bool validate(QString* error) const;
    QStringList toolNames() const;
    QString toolId(const QString& toolName) const;
    QVariant machineAxisValue(const QString& axisName, const QString& key) const;
    bool setMachineAxisValue(const QString& axisName, const QString& key, const QVariant& value, QString* error);
    void seedDefaults();
    void seedBuiltinIo();

    ProcessSettingsSnapshot m_committed;
    toml::value m_draft{toml::table{}};
    QVector<MachineAxisRuntimeConfig> m_axisDraft;
    ProcessParameterRegistry m_registry;
    bool m_axisDirty{false};
    mutable QHash<QString, QString> m_toolIds;
};

} // namespace lcnc::process
