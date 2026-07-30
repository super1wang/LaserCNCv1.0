#pragma once

#include "modules/process/workflow/process_flow_document.h"

#include <QString>
#include <toml.hpp>

namespace lcnc::process {

/**
 * @brief Reads and writes the current process workflow TOML schema.
 */
class ProcessFlowStore
{
public:
    static constexpr int SchemaVersion = 1;

    static bool loadFromFile(const QString& filePath,
                             ProcessFlowDocument& document,
                             QString* errorMessage = nullptr);
    static bool saveToFile(const QString& filePath,
                           const ProcessFlowDocument& document,
                           QString* errorMessage = nullptr);

    static bool loadFromToml(const toml::value& root,
                             ProcessFlowDocument& document,
                             QString* errorMessage = nullptr);
    static toml::value toToml(const ProcessFlowDocument& document);
};

} // namespace lcnc::process
