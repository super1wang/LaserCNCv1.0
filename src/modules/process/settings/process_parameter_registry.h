#pragma once

#include "modules/process/settings/process_settings_types.h"

#include <QVariant>
#include <QVector>

#include <limits>

namespace lcnc::process {

class ProcessSettingsService;

enum class ParameterValueType { Bool, Int, Double, String, Enum, AxisRef, IoRef, ToolRef };

struct ParameterDescriptor
{
    QString id;
    QString title;
    QString description;
    QString group;
    ParameterValueType type{ParameterValueType::String};
    QString unit;
    QVariant defaultValue;
    double minimum{std::numeric_limits<double>::lowest()};
    double maximum{std::numeric_limits<double>::max()};
    int decimals{3};
    QStringList enumValues;
    /// Optional translated labels parallel to enumValues. Persisted values
    /// remain stable English tokens while the editor displays localized text.
    QStringList enumLabels;
    ProcessConfigArea area{ProcessConfigArea::Workflow};
    QString tableName;
    QString key;
    bool machineAxisField{false};
    bool readOnly{false};
};

struct ParameterObjectDescriptor
{
    QString id;
    QString title;
    QString category;
    QString iconName;
    QVector<ParameterDescriptor> fields;
    bool toolObject{false};
    bool ioTableObject{false};
    ProcessIoBucket ioBucket{ProcessIoBucket::DigitalInput};
};

// The single C++ field registry.  It owns parameter names, types, ranges and
// grouping; the UI only renders these descriptors.
class ProcessParameterRegistry
{
public:
    explicit ProcessParameterRegistry(const ProcessSettingsService& settings);
    QVector<ParameterObjectDescriptor> buildObjects() const;

private:
    const ProcessSettingsService& m_settings;
};

} // namespace lcnc::process
