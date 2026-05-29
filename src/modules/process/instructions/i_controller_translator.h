#pragma once

#include "modules/process/instructions/process_command.h"

namespace lcnc::process {

/**
 * @brief Translates controller-neutral commands to a specific motion controller profile.
 */
class IControllerTranslator
{
public:
    virtual ~IControllerTranslator() = default;
    virtual QString profileName() const = 0;
    virtual bool translate(const ProcessCommandBuffer& buffer, QStringList* output, QString* errorMessage = nullptr) const = 0;
};

/**
 * @brief Translator used by PureSimulation dry-run paths.
 */
class PureSimulationTranslator final : public IControllerTranslator
{
public:
    QString profileName() const override { return QStringLiteral("PureSimulation"); }
    bool translate(const ProcessCommandBuffer& buffer, QStringList* output, QString* errorMessage = nullptr) const override;
};

/**
 * @brief Translator shell for ACS-family controllers; SDK dispatch stays in adapters.
 */
class AcsControllerTranslator final : public IControllerTranslator
{
public:
    QString profileName() const override { return QStringLiteral("ACS"); }
    bool translate(const ProcessCommandBuffer& buffer, QStringList* output, QString* errorMessage = nullptr) const override;
};

/**
 * @brief Translator shell for GTN controllers; SDK dispatch stays in adapters.
 */
class GtnControllerTranslator final : public IControllerTranslator
{
public:
    QString profileName() const override { return QStringLiteral("GTN"); }
    bool translate(const ProcessCommandBuffer& buffer, QStringList* output, QString* errorMessage = nullptr) const override;
};

} // namespace lcnc::process