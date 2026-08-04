#include "modules/cam/services/cam_display_projection_service.h"

#include "core/document/lcnc_document.h"
#include "core/kinematics/machine_kinematics.h"
#include "view/gui_document.h"

#include <AIS_DisplayMode.hxx>
#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <Aspect_PolygonOffsetMode.hxx>
#include <Aspect_TypeOfLine.hxx>
#include <Graphic3d_ZLayerId.hxx>
#include <Prs3d_LineAspect.hxx>
#include <Quantity_Color.hxx>
#include <gp_Trsf.hxx>

#include <QMap>

namespace lcnc::cam {

class CamDisplayProjectionService::State
{
public:
    struct FaceEntry {
        Handle(AIS_Shape) ais;
        QString workpieceEntry;
    };
    struct DocumentProjection {
        Handle(AIS_InteractiveContext) context;
        QMap<std::uint64_t, FaceEntry> machiningFaceAis;
    };

    static void clear(DocumentProjection& projection)
    {
        if (!projection.context.IsNull()) {
            for (auto it = projection.machiningFaceAis.cbegin();
                 it != projection.machiningFaceAis.cend(); ++it) {
                if (!it.value().ais.IsNull())
                    projection.context->Remove(it.value().ais, Standard_False);
            }
            projection.context->UpdateCurrentViewer();
        }
        projection.machiningFaceAis.clear();
    }

    QMap<DocumentId, DocumentProjection> documents;
};

CamDisplayProjectionService::CamDisplayProjectionService() = default;
CamDisplayProjectionService::~CamDisplayProjectionService()
{
    if (!m_state)
        return;
    for (auto it = m_state->documents.begin(); it != m_state->documents.end(); ++it) {
        State::clear(*it);
    }
}

namespace {

DocumentId documentIdFor(const GuiDocument* document)
{
    return document && document->document() ? document->document()->id() : kInvalidDocumentId;
}

} // namespace

void CamDisplayProjectionService::clearMachiningFaces(GuiDocument* document)
{
    if (!m_state)
        return;
    const DocumentId id = documentIdFor(document);
    auto it = m_state->documents.find(id);
    if (it == m_state->documents.end())
        return;
    State::clear(*it);
    m_state->documents.erase(it);
}

void CamDisplayProjectionService::refreshMachiningFaces(
    GuiDocument* document,
    const std::vector<MachiningFaceDisplaySnapshot>& faces,
    bool visible)
{
    if (!m_state)
        m_state = std::make_unique<State>();
    if (!document || document->context().IsNull())
        return;

    const DocumentId id = documentIdFor(document);
    if (id == kInvalidDocumentId)
        return;
    auto& projection = m_state->documents[id];
    // A document can be reopened with a new viewer context.  Remove the old
    // presentation from the context that owns it before retaining the new one.
    if (!projection.context.IsNull() && projection.context != document->context())
        State::clear(projection);
    projection.context = document->context();
    State::clear(projection);
    const Handle(AIS_InteractiveContext)& context = projection.context;
    if (!visible) {
        context->UpdateCurrentViewer();
        return;
    }

    for (const MachiningFaceDisplaySnapshot& entry : faces) {
        if (entry.face.IsNull() || entry.role != MachiningFaceRole::MachiningSurface)
            continue;
        const Quantity_Color color = entry.manual
            ? Quantity_Color(1.0, 0.82, 0.0, Quantity_TOC_RGB)
            : Quantity_Color(0.0, 0.85, 1.0, Quantity_TOC_RGB);
        Handle(AIS_Shape) ais = new AIS_Shape(entry.face);
        ais->SetDisplayMode(AIS_Shaded);
        ais->SetColor(color);
        ais->SetTransparency(entry.manual ? 0.25 : 0.45);
        ais->SetPolygonOffsets(Aspect_POM_Fill, -1.0f, -1.0f);
        if (!ais->Attributes().IsNull()) {
            ais->Attributes()->SetFaceBoundaryDraw(true);
            ais->Attributes()->SetFaceBoundaryAspect(new Prs3d_LineAspect(
                color, Aspect_TOL_SOLID, entry.manual ? 3.0 : 2.0));
        }
        context->Display(ais, AIS_Shaded, 0, Standard_False);
        context->SetZLayer(ais, Graphic3d_ZLayerId_Top);
        context->Deactivate(ais);
        projection.machiningFaceAis.insert(entry.faceId, {ais, entry.workpieceEntry});
    }
    context->UpdateCurrentViewer();
}

void CamDisplayProjectionService::updateMachiningFaceTransforms(
    GuiDocument* document, const MachineKinematics* kinematics)
{
    if (!m_state || !kinematics)
        return;
    const DocumentId id = documentIdFor(document);
    auto it = m_state->documents.find(id);
    if (it == m_state->documents.end())
        return;
    auto& projection = *it;
    if (projection.context.IsNull())
        return;
    for (auto faceIt = projection.machiningFaceAis.begin();
         faceIt != projection.machiningFaceAis.end(); ++faceIt) {
        const Handle(AIS_Shape)& ais = faceIt.value().ais;
        if (ais.IsNull())
            continue;
        gp_Trsf transform;
        if (!faceIt.value().workpieceEntry.isEmpty())
            transform = kinematics->computeWpcTransform(faceIt.value().workpieceEntry);
        ais->SetLocalTransformation(transform);
        projection.context->RecomputePrsOnly(ais, Standard_False);
    }
}

} // namespace lcnc::cam
