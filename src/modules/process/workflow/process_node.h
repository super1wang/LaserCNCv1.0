#pragma once

#include "modules/process/workflow/process_node_type.h"

#include <QVariantMap>
#include <QVector>

namespace lcnc::process {

/**
 * @brief Data-only process workflow node with a stable id and typed children.
 */
struct ProcessNode
{
    ProcessNode();
    explicit ProcessNode(ProcessNodeType nodeType, const QString& nodeName = QString());

    QString id;
    ProcessNodeType type{ProcessNodeType::Base};
    QString name;
    bool enabled{true};
    ProcessNodeState state{ProcessNodeState::Enabled};
    QVariantMap parameters;
    QVector<ProcessNode> children;
};

QString createProcessNodeId();
QString defaultProcessNodeName(ProcessNodeType type);

} // namespace lcnc::process
