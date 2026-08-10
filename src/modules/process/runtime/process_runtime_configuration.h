#pragma once

#include "modules/process/runtime/process_axis_types.h"
#include "modules/process/runtime/process_permission_types.h"

#include <QStringList>

#include <string>
#include <utility>

#include "magic_enum.hpp"

namespace lcnc::process {

// Runtime-only machine facts owned by ProcessModule.  This replaces the
// mutable DT globals for controller selection and is never persisted by a
// device adapter.
class ProcessRuntimeConfiguration
{
public:
    void setEnabledAxes(QStringList axes)
    {
        m_enabledAxes.clear();
        for (QString& axis : axes) {
            axis = axis.trimmed().toUpper();
            if (!axis.isEmpty() && axis != QStringLiteral("BASE") && !m_enabledAxes.contains(axis))
                m_enabledAxes.append(axis);
        }
    }

    bool isAxisEnabled(Axis axis) const
    {
        return m_enabledAxes.contains(QString::fromLatin1(magic_enum::enum_name(axis).data()));
    }

    bool hasEnabledAxes() const { return !m_enabledAxes.isEmpty(); }
    const QStringList& enabledAxes() const { return m_enabledAxes; }

    void setExtensionAxes(QStringList axes) { m_extensionAxes = std::move(axes); }
    const QStringList& extensionAxes() const { return m_extensionAxes; }
    bool isExtensionAxis(const QString& name) const { return m_extensionAxes.contains(name.trimmed(), Qt::CaseInsensitive); }

    void setSimulationMode(bool enabled) { m_simulationMode = enabled; }
    bool simulationMode() const { return m_simulationMode; }

    void setCustomerId(std::string customerId) { m_customerId = std::move(customerId); }
    const std::string& customerId() const { return m_customerId; }
    void setPermission(PermissionLevel permission) { m_permission = permission; }
    PermissionLevel permission() const { return m_permission; }

private:
    QStringList m_enabledAxes;
    QStringList m_extensionAxes;
    bool m_simulationMode{true};
    std::string m_customerId{"Standard"};
    PermissionLevel m_permission{PermissionLevel::Operator};
};

} // namespace lcnc::process
