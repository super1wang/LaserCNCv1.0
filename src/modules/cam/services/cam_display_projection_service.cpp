#include "modules/cam/services/cam_display_projection_service.h"

#include "view/gui_document.h"

#include <AIS_DisplayMode.hxx>
#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <Aspect_PolygonOffsetMode.hxx>
#include <Aspect_TypeOfLine.hxx>
#include <Graphic3d_ZLayerId.hxx>
#include <Prs3d_LineAspect.hxx>
#include <Quantity_Color.hxx>

#include <QMap>

namespace lcnc::cam {

class CamDisplayProjectionService::State
{
public:
    QMap<std::uint64_t, Handle(AIS_Shape)> machiningFaceAis;
};

CamDisplayProjectionService::CamDisplayProjectionService() = default;
CamDisplayProjectionService::~CamDisplayProjectionService() = default;

void CamDisplayProjectionService::clearMachiningFaces(GuiDocument* document)
{
    if (!m_state)
        return;
    if (document && !document->context().IsNull()) {
        const Handle(AIS_InteractiveContext)& context = document->context();
        for (auto it = m_state->machiningFaceAis.cbegin();
             it != m_state->machiningFaceAis.cend(); ++it) {
            if (!it.value().IsNull())
                context->Remove(it.value(), Standard_False);
        }
        context->UpdateCurrentViewer();
    }
    m_state->machiningFaceAis.clear();
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

    const Handle(AIS_InteractiveContext)& context = document->context();
    for (auto it = m_state->machiningFaceAis.cbegin();
         it != m_state->machiningFaceAis.cend(); ++it) {
        if (!it.value().IsNull())
            context->Remove(it.value(), Standard_False);
    }
    m_state->machiningFaceAis.clear();
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
        m_state->machiningFaceAis.insert(entry.faceId, ais);
    }
    context->UpdateCurrentViewer();
}

} // namespace lcnc::cam
