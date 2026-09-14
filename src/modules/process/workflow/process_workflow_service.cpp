#include "modules/process/workflow/process_workflow_service.h"

#include "modules/process/workflow/process_flow_store.h"

namespace lcnc::process {

ProcessWorkflowService::ProcessWorkflowService(QObject* parent)
    : QObject(parent)
{
}

void ProcessWorkflowService::createNew()
{
    m_document.resetToDefault();
    m_document.markClean();
    emit flowChanged();
}

bool ProcessWorkflowService::load(const QString& filePath, QString* errorMessage)
{
    if (filePath.trimmed().isEmpty()) {
        if (errorMessage)
            // 中文翻译：流程文件路径为空
            *errorMessage = tr("Process file path is empty");
        return false;
    }

    if (!ProcessFlowStore::loadFromFile(filePath, m_document, errorMessage))
        return false;

    emit flowChanged();
    return true;
}

bool ProcessWorkflowService::save(const QString& filePath, QString* errorMessage)
{
    if (filePath.trimmed().isEmpty()) {
        if (errorMessage)
            // 中文翻译：流程文件路径为空
            *errorMessage = tr("Process file path is empty");
        return false;
    }

    if (!ProcessFlowStore::saveToFile(filePath, m_document, errorMessage))
        return false;

    m_document.markClean();
    return true;
}

void ProcessWorkflowService::notifyChanged()
{
    emit flowChanged();
}

} // namespace lcnc::process
