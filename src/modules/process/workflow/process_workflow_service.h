#pragma once

#include "core/kernel/i_service.h"
#include "modules/process/workflow/process_flow_document.h"

#include <QObject>
#include <QString>

namespace lcnc::process {

/**
 * @brief Process-flow document boundary used by UI and facade clients.
 *
 * Owns the current-schema workflow document and its file operations.  The
 * Process module only coordinates lifecycle and execution; it does not parse
 * or persist workflow files directly.
 */
class IProcessWorkflowService : public lcnc::IService
{
public:
    ~IProcessWorkflowService() override = default;

    virtual QObject* notifier() = 0;
    virtual ProcessFlowDocument& document() = 0;
    virtual const ProcessFlowDocument& document() const = 0;
    virtual void createNew() = 0;
    virtual bool load(const QString& filePath, QString* errorMessage = nullptr) = 0;
    virtual bool save(const QString& filePath, QString* errorMessage = nullptr) = 0;
};

class ProcessWorkflowService final : public QObject, public IProcessWorkflowService
{
    Q_OBJECT
public:
    explicit ProcessWorkflowService(QObject* parent = nullptr);

    QObject* notifier() override { return this; }
    ProcessFlowDocument& document() override { return m_document; }
    const ProcessFlowDocument& document() const override { return m_document; }
    void createNew() override;
    bool load(const QString& filePath, QString* errorMessage = nullptr) override;
    bool save(const QString& filePath, QString* errorMessage = nullptr) override;

    /// Announces a state-only change made by the workflow executor.
    void notifyChanged();

signals:
    void flowChanged();

private:
    ProcessFlowDocument m_document;
};

} // namespace lcnc::process
