#pragma once

#include "core/kernel/i_service.h"
#include "core/project/project_types.h"

#include <QObject>
#include <QString>

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
                                  QObject* parent = nullptr);

    DocumentId createDocument(const QString& name = QString()) const;
    bool saveDocument(LcncDocument* document,
                      const QString& path,
                      QString* errorMessage = nullptr) const;
    bool closeDocument(DocumentId documentId) const;

private:
    lcnc::LcncProjectManager& m_projectManager;
};

} // namespace lcnc::cad
