#pragma once

#include "modules/process/workflow/process_node.h"

#include <QVariantMap>
#include <QVector>

namespace lcnc::process {

struct ProcessNodeDescriptor
{
    ProcessNodeType type{ProcessNodeType::Base};
    QString displayName;
    QString category;
    bool canHaveChildren{false};
    bool topLevelOnly{false};
    bool required{false};
    bool addable{true};
    bool deletable{true};
    bool disableable{true};
    bool movable{true};
    bool pluginEnabled{true};
    QVariantMap defaultParameters;
    QString executorKey;
};

class ProcessNodeRegistry
{
public:
    static const ProcessNodeRegistry& instance();

    const QVector<ProcessNodeDescriptor>& descriptors() const { return m_descriptors; }
    const ProcessNodeDescriptor* descriptor(ProcessNodeType type) const;
    QVector<ProcessNodeType> addableTypes() const;

    ProcessNode createDefaultNode(ProcessNodeType type) const;
    QString summary(const ProcessNode& node) const;
    bool canPlaceNode(ProcessNodeType type, const ProcessNodeType* parentType) const;
    bool canHaveChildren(ProcessNodeType type) const;
    bool isRequired(ProcessNodeType type) const;
    bool isDeletable(ProcessNodeType type) const;
    bool isDisableable(ProcessNodeType type) const;
    bool isMovable(ProcessNodeType type) const;

private:
    ProcessNodeRegistry();
    void registerBuiltIns();
    void add(ProcessNodeType type,
             const QString& category,
             bool canHaveChildren,
             bool topLevelOnly,
             QVariantMap defaults = {},
             const QString& executorKey = QString(),
             bool required = false,
             bool addable = true);

    QVector<ProcessNodeDescriptor> m_descriptors;
};

} // namespace lcnc::process
