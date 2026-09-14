#pragma once

#include "core/kernel/i_service.h"
#include "core/project/project_types.h"

#include <QList>
#include <QString>

class LcncDocument;

namespace lcnc::cad {

struct ProjectExplorerSketchElement {
    int id{0};
    QString label;
};

struct ProjectExplorerSketch {
    int sketchId{0};
    QString name;
    bool visible{true};
    bool usedByFeature{false};
    QList<ProjectExplorerSketchElement> elements;
};

/** Read-only CAD projection consumed by the project explorer. */
class ICadProjectExplorerProjection : public lcnc::IService
{
public:
    ~ICadProjectExplorerProjection() override = default;

    virtual LcncDocument* projectExplorerWorkpieceDocument() const = 0;
    virtual bool projectExplorerIsSketchEditing() const = 0;
    virtual QList<ProjectExplorerSketchElement> projectExplorerActiveSketchElements() const = 0;
    virtual QList<ProjectExplorerSketch> projectExplorerFinishedSketches(DocumentId documentId) const = 0;
};

} // namespace lcnc::cad
