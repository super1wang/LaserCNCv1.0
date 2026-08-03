#pragma once

#include "core/kernel/i_service.h"
#include "core/project/project_types.h"
#include "core/task/task_manager.h"

#include <QObject>
#include <QString>
#include <memory>

class LcncDocument;

namespace lcnc {
class LcncProjectManager;
}

namespace lcnc::cad {

/**
 * @brief Transaction boundary for project document lifecycle operations.
 *
 * The service owns project-level new/save/close decisions.  Import/export
 * workers are added here incrementally so CadModule remains an event bridge
 * instead of becoming another file-format implementation.
 */
class CadDocumentIoService final : public QObject, public lcnc::IService
{
    Q_OBJECT
public:
    explicit CadDocumentIoService(lcnc::LcncProjectManager& projectManager,
                                  TaskManager& taskManager,
                                  QObject* parent = nullptr);

    struct ExportTask {
        TaskId id{kInvalidTaskId};
        std::shared_ptr<QString> error;
    };
    struct ImportTask {
        TaskId id{kInvalidTaskId};
        std::shared_ptr<QString> error;
    };

    DocumentId createDocument(const QString& name = QString()) const;
    bool saveDocument(LcncDocument* document,
                      const QString& path,
                      QString* errorMessage = nullptr) const;
    bool closeDocument(DocumentId documentId) const;
    ExportTask exportStepAsync(LcncDocument* document, const QString& filePath) const;
    ImportTask importStlAsync(LcncDocument* document, const QString& filePath) const;
    bool importStlIntoDocument(LcncDocument* document,
                               const QString& filePath,
                               TaskProgress* progress,
                               QString* errorMessage) const;
    ImportTask importBrepAsync(LcncDocument* document, const QString& filePath) const;
    bool importBrepIntoDocument(LcncDocument* document,
                                const QString& filePath,
                                TaskProgress* progress,
                                QString* errorMessage) const;
    bool importStepIntoDocument(LcncDocument* document,
                                const QString& filePath,
                                QString* errorMessage) const;
    bool importIgesIntoDocument(LcncDocument* document,
                                const QString& filePath,
                                QString* errorMessage) const;
    bool prepareDisplayMesh(LcncDocument* document,
                            TaskProgress* progress,
                            QString* errorMessage) const;

private:
    lcnc::LcncProjectManager& m_projectManager;
    TaskManager& m_taskManager;
};

} // namespace lcnc::cad
